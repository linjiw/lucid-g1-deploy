"""Select, verify, play and rehearse the September 17 four-motion policy pair."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "config/four_motion_20260917.json"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_joint_order(directory: Path, clip) -> None:
    """Check actual joint values in policy order, not just CSV dimensions."""
    import numpy as np
    from mujoco_player import ISAAC_TO_MJ

    for filename, source in (
        ("joint_pos.csv", clip.dof50),
        ("joint_vel.csv", clip.vel50),
    ):
        actual = np.loadtxt(directory / filename, delimiter=",", skiprows=1)
        expected = source[:, ISAAC_TO_MJ]
        if actual.shape != expected.shape or not np.allclose(
            actual, expected, rtol=0, atol=1e-6
        ):
            raise ValueError(
                f"Reference joint order/values differ: {directory}/{filename}"
            )


def verify(manifest: dict) -> dict:
    """Fail before playback if weights, references, or observation layout changed."""
    import onnxruntime as ort
    from mujoco_player import load_clip
    from verify_deploy_bundle import check_motion

    problems = []
    checked = []
    for arm, policy in manifest["policies"].items():
        path = ROOT / policy["path"]
        if sha(path) != policy["sha256"]:
            raise ValueError(f"Policy hash changed: {path}")
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        session = ort.InferenceSession(
            str(path), sess_options=opts, providers=["CPUExecutionProvider"]
        )
        if [(x.name, x.shape) for x in session.get_inputs()] != [
            ("obs_dict", [1, 1570])
        ]:
            raise ValueError(f"Wrong observation signature: {arm}")
        if [(x.name, x.shape) for x in session.get_outputs()] != [("action", [1, 29])]:
            raise ValueError(f"Wrong action signature: {arm}")
        checked.append(arm)
    for motion in manifest["motions"]:
        if sha(ROOT / motion["clip"]) != motion["sha256"]:
            raise ValueError(f"Source clip hash changed: {motion['alias']}")
        for filename, expected in motion["csv_sha256"].items():
            if sha(ROOT / motion["deploy_motion"] / filename) != expected:
                raise ValueError(f"Reference CSV changed: {motion['alias']}/{filename}")
        check_motion(ROOT / motion["deploy_motion"], problems)
        verify_joint_order(
            ROOT / motion["deploy_motion"], load_clip(ROOT / motion["clip"])
        )
    config = ROOT / "config/observation_config_lucid_g1_1570.yaml"
    if sha(config) != manifest["observation_config_sha256"]:
        raise ValueError("Observation configuration changed")
    if problems:
        raise ValueError("\n".join(problems))
    return {
        "passed": True,
        "policies": checked,
        "motions": [m["alias"] for m in manifest["motions"]],
    }


def command(
    mode: str, arm: str, motion: dict, out: Path, lam: float, seed: int, record: bool
) -> list[str]:
    """Every DDS launch is explicitly a simulator on loopback."""
    policy_name = f"four_motion_{arm}"
    if mode == "play":
        return [
            sys.executable,
            str(ROOT / "tools/mujoco_player.py"),
            "--onnx",
            str(ROOT / "policies" / f"{policy_name}_s8600_g1.onnx"),
            "--clip",
            str(ROOT / motion["clip"]),
            "--out",
            str(out / "playback.mp4"),
            "--full-clip",
            "--lam",
            str(lam),
            "--seed",
            str(seed),
        ]
    if mode == "rehearse":
        result = [
            "bash",
            str(ROOT / "drill.sh"),
            "--policy",
            policy_name,
            "--motion",
            f"four_motion_{motion['alias']}",
            "--iface",
            "lo",
            "--parity",
            "--play",
            "--out",
            str(out),
            "--hold",
            str(motion["duration"] + 0.8),
        ]
        if record:
            result += ["--record", str(out / "deployment_sequence.mp4")]
        return result
    return [
        "bash",
        str(ROOT / "run.sh"),
        "--policy",
        policy_name,
        "--motion",
        f"four_motion_{motion['alias']}",
        "--sim",
        "--iface",
        "lo",
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["list", "check", "play", "rehearse", "run"])
    parser.add_argument("--policy", choices=["fixed_dr", "no_dr"], default="fixed_dr")
    parser.add_argument(
        "--motion",
        choices=["walking", "turning", "crouch_hold", "side_stepping"],
        default="walking",
    )
    parser.add_argument("--out", type=Path)
    parser.add_argument(
        "--lam",
        type=float,
        default=0.0,
        help="play only: MuJoCo DR scale, 0 nominal / 1 approximate training range",
    )
    parser.add_argument("--seed", type=int, default=8700)
    parser.add_argument(
        "--viewer", action="store_true", help="run/rehearse: show the MuJoCo window"
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="rehearse: record the actual deployment sequence",
    )
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    if args.mode == "list":
        print("Policies: fixed_dr, no_dr")
        print("Motions: walking, turning, crouch_hold, side_stepping")
        print(
            "play = reference-initialized MuJoCo video; rehearse = actual DDS sequence; "
            "run = interactive DDS simulation"
        )
        return 0
    checked = verify(manifest)
    if args.mode == "check":
        print(json.dumps(checked, indent=2))
        return 0
    if args.mode != "play" and args.lam != 0:
        parser.error(
            "--lam is supported only by play; DDS rehearsal uses nominal physics"
        )
    if args.mode == "play" and args.viewer:
        parser.error("--viewer is for run/rehearse; play writes an MP4")
    motion = next(m for m in manifest["motions"] if m["alias"] == args.motion)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_id = f"deploy4-play-{stamp}-{uuid.uuid4().hex[:8]}"
    out = (
        args.out
        or ROOT
        / "results/four_motion"
        / f"{args.mode}_{args.policy}_{args.motion}_{stamp}"
    ).resolve()
    if out.exists() and any(out.iterdir()):
        parser.error(f"Output directory is not empty; choose a new --out: {out}")
    out.mkdir(parents=True, exist_ok=True)
    cmd = command(args.mode, args.policy, motion, out, args.lam, args.seed, args.record)
    if args.viewer:
        cmd.append("--viewer")
    import mujoco
    import onnxruntime
    import wandb

    config = {
        "framework": "SONIC",
        "stage": args.mode,
        "arm": args.policy,
        "seed": 8600,
        "evaluation_seed": args.seed,
        "checkpoint_iteration": 4000,
        "motion": args.motion,
        "evaluation_condition": (
            f"MuJoCo lambda={args.lam}"
            if args.mode == "play"
            else "MuJoCo DDS nominal stand-to-playback"
        ),
        "source_commit": manifest["source_commit"],
        "deploy_source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "deploy_worktree_dirty": bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=ROOT, text=True
            ).strip()
        ),
        "deploy_source_sha256": {
            path: sha(ROOT / path)
            for path in (
                "tools/four_motion.py",
                "tools/mujoco_player.py",
                "drill.sh",
                "sim/run_robot_sim.py",
                "sim/gear_sonic_sim/base_sim.py",
                "tools/four_motion_metrics.py",
            )
        },
        "frozen_plan_sha256": sha(MANIFEST),
        "checkpoint_sha256": manifest["policies"][args.policy]["checkpoint_sha256"],
        "onnx_sha256": manifest["policies"][args.policy]["sha256"],
        "motion_sha256": motion["sha256"],
        "python": sys.version,
        "mujoco": mujoco.__version__,
        "onnxruntime": onnxruntime.__version__,
        "command": cmd,
    }
    run = wandb.init(
        entity="16726",
        project="lucid-sonic",
        group=f"four_motion_deploy_{stamp[:8]}",
        id=run_id,
        name=f"SONIC/deploy-{args.mode}/{args.policy}/s8600/h4000/{args.motion}/l{args.lam}/es{args.seed}",
        mode="online",
        dir=str(out),
        config=config,
    )
    (out / "launch.json").write_text(json.dumps(config, indent=2) + "\n")
    started = time.monotonic()
    print("Output:", out, flush=True)
    if args.mode == "run":
        print(
            "Wait for Init Done, then ] starts control, T plays the motion, O stops.",
            flush=True,
        )
    with (out / "console.log").open("w") as log:
        process = subprocess.Popen(
            cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        try:
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
            code = process.wait()
        except KeyboardInterrupt:
            process.terminate()
            code = process.wait(timeout=20)
    metrics = {"returncode": code, "runtime_seconds": time.monotonic() - started}
    if (out / "playback.json").exists():
        result = json.loads((out / "playback.json").read_text())["result"]
        metrics.update(
            {
                "tracking_threshold_exceeded": result["fell"],
                "upright_at_end": result["upright_at_end"],
                "duration_s": result["t_end"],
                "first_tracking_drift_s": result["t_drift"],
            }
        )
    if args.mode == "rehearse" and (out / "sim.log").exists():
        from four_motion_metrics import dds_metrics

        metrics.update(dds_metrics(out))
        expected_frames = motion["parser_audit"]["metadata_timesteps"]
        metrics["expected_reference_rows"] = expected_frames
        metrics["full_reference_observed"] = (
            metrics.get("distinct_reference_rows") == expected_frames
        )
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    run.log({k: v for k, v in metrics.items() if isinstance(v, (int, float, bool))})
    run.summary["metrics_sha256"] = sha(out / "metrics.json")
    url = run.url
    run.finish(exit_code=0 if code == 0 else 1)
    remote = wandb.Api().run(f"16726/lucid-sonic/{run_id}")
    assert remote.summary["metrics_sha256"] == sha(out / "metrics.json")
    (out / "receipt.json").write_text(
        json.dumps(
            {"online_verified": True, "wandb_url": url, "metrics": metrics}, indent=2
        )
        + "\n"
    )
    print("Receipt:", out / "receipt.json")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
