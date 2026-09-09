#!/usr/bin/env python3
"""Emit golden (observation, action) pairs for a deployed policy.

    emit_parity_vectors.py --onnx <..._g1.onnx> --clip <clip.pkl> --out <dir>
                           [--steps 200] [--seed 1]

WHY THIS EXISTS
---------------
Dimensional agreement between the training-side observation and the C++
runner's observation is checkable statically -- see
``scripts/practice_utility/validate_deploy_obs_config.py``. VALUE agreement is
not. Nothing in this repository can currently tell a correct deployment from
one where a term is transposed, a history buffer runs newest-first, a
quaternion convention is flipped, or joint values arrive in MuJoCo order where
Isaac order was meant. Every one of those produces a running robot that behaves
subtly, then catastrophically, wrong.

This writes the reference side of that test. It drives the policy through
``tools/mujoco_player.py`` -- the observation builder that reproduces the
project's Isaac receipts in MuJoCo, so it is the one implementation with
evidence behind it -- and records, for each control step, the exact 1,570-float
input and the 29-float action the ONNX returned.

HOW TO USE IT ON THE ROBOT SIDE
-------------------------------
The runner has a ``--policy-input-logfile`` hook that has never been used. Run
it against the same clip and compare element-wise with ``--compare``. A
deployment is parity-checked when the maximum absolute action difference over
the whole trace is at the level of float noise, and NOT before.

The comparison is only as good as the alignment: both sides must start from the
same reference frame and consume the clip at the same rate. Mismatched start
frames look exactly like a broken observation, so the receipt records the reset
convention (root pose and joint state taken from clip frame 0, root velocity
from a one-frame finite difference) for whoever runs the other half.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def sha256(path: Path) -> str:
    d = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            d.update(chunk)
    return d.hexdigest()


def emit(onnx: Path, clip: Path, out: Path, steps: int, seed: int, lam: float) -> dict:
    import mujoco_player as MP

    clip_obj = MP.load_clip(clip) if hasattr(MP, "load_clip") else MP.Clip.load(clip)
    dr = MP.DRConfig(lam=lam, seed=seed)
    player = MP.Player(onnx_path=onnx, clip=clip_obj, dr=dr, video=False)
    player.reset()

    obs_rows: list[np.ndarray] = []
    act_rows: list[np.ndarray] = []
    for _ in range(steps):
        if player.t >= clip_obj.duration:
            break
        obs = player.build_obs().astype(np.float64)
        action = player.session.run(
            None, {player.obs_name: obs.astype(np.float32)[None]}
        )[0][0].astype(np.float64)
        obs_rows.append(obs)
        act_rows.append(action)
        # Advance exactly as the control loop does, so the next observation is
        # the one the runner would face at the same step index.
        target = player.step_policy()
        for _ in range(MP.DECIMATION):
            player.delay.append(target)
            delayed = player.delay[0]
            player._maybe_push()
            player.data.ctrl[:] = player._pd(delayed)
            player.mujoco.mj_step(player.model, player.data)
            player.t += MP.SIM_DT
        player.k += 1

    out.mkdir(parents=True, exist_ok=True)
    obs_arr = np.asarray(obs_rows)
    act_arr = np.asarray(act_rows)
    np.savez_compressed(
        out / "parity_vectors.npz", observations=obs_arr, actions=act_arr
    )
    receipt = {
        "kind": "deploy_parity_vectors",
        "onnx": str(onnx),
        "onnx_sha256": sha256(onnx),
        "clip": str(clip),
        "lam": lam,
        "seed": seed,
        "steps": int(obs_arr.shape[0]),
        "obs_dim": int(obs_arr.shape[1]),
        "action_dim": int(act_arr.shape[1]),
        "control_hz": 1.0 / (MP.SIM_DT * MP.DECIMATION),
        "joint_order": "IsaacLab",
        "reset_convention": (
            "root pose and joint positions from clip frame 0; root linear velocity from a "
            "one-frame finite difference at 50 Hz; root angular velocity from the body-frame "
            "quaternion delta; joint velocities from the clip. Reset WITH reference velocities."
        ),
        "obs_layout": [
            ["motion_joint_positions_10frame_step5", 290],
            ["motion_joint_velocities_10frame_step5", 290],
            ["motion_anchor_orientation_10frame_step5", 60],
            ["his_base_angular_velocity_10frame_step1", 30],
            ["his_body_joint_positions_10frame_step1", 290],
            ["his_body_joint_velocities_10frame_step1", 290],
            ["his_last_actions_10frame_step1", 290],
            ["his_gravity_dir_10frame_step1", 30],
        ],
        "verified": [
            "observations and actions come from the same ONNX session the MuJoCo "
            "reference player uses",
        ],
        "not_verified": [
            "that the C++ runner reproduces these actions -- that is the test these "
            "vectors exist to make possible, and it has never been run",
        ],
    }
    (out / "parity_receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt


def compare(vectors: Path, runner_log: Path, tol: float) -> int:
    """Compare a runner action log against the golden actions."""
    data = np.load(vectors / "parity_vectors.npz")
    golden = data["actions"]
    rows = [
        json.loads(line)
        for line in runner_log.read_text().splitlines()
        if line.strip().startswith("{")
    ]
    if not rows:
        raise SystemExit(f"no JSON action rows found in {runner_log}")
    theirs = np.asarray([r["action"] for r in rows], dtype=np.float64)
    n = min(len(golden), len(theirs))
    if n == 0:
        raise SystemExit("no overlapping steps")
    diff = np.abs(golden[:n] - theirs[:n])
    print(f"compared {n} steps")
    print(f"  max |delta|  : {diff.max():.3e}")
    print(f"  mean |delta| : {diff.mean():.3e}")
    worst = int(np.unravel_index(diff.argmax(), diff.shape)[0])
    print(f"  worst step   : {worst}")
    ok = bool(diff.max() <= tol)
    print("PARITY OK" if ok else f"PARITY FAILED (tolerance {tol:.1e})")
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--onnx", type=Path)
    ap.add_argument("--clip", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--steps", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument(
        "--lam",
        type=float,
        default=0.0,
        help="keep at 0: parity is about the observation pipeline, not robustness",
    )
    ap.add_argument(
        "--compare",
        type=Path,
        default=None,
        help="a runner --policy-input-logfile trace to check against the golden actions",
    )
    ap.add_argument("--tol", type=float, default=1e-4)
    a = ap.parse_args(argv)

    if a.compare is not None:
        return compare(a.out, a.compare, a.tol)
    if not a.onnx or not a.clip:
        raise SystemExit("--onnx and --clip are required unless --compare is given")
    receipt = emit(a.onnx, a.clip, a.out, a.steps, a.seed, a.lam)
    print(
        json.dumps(
            {k: receipt[k] for k in ("steps", "obs_dim", "action_dim", "onnx_sha256")},
            indent=2,
        )
    )
    print(f"wrote {a.out}/parity_vectors.npz and parity_receipt.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
