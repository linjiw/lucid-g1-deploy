#!/usr/bin/env python3
"""Convert a LUCID motion clip into the CSV bundle the C++ runner reads.

    convert_clip_for_deploy.py --clip <clip.pkl> --out <dir> [--fps 50]

WHY NOT gear_sonic_deploy/reference/convert_motions.py
------------------------------------------------------
That script expects a pack carrying ``joint_pos``, ``joint_vel``, ``body_pos_w``,
``body_quat_w``, ``body_lin_vel_w`` and ``body_ang_vel_w`` -- a post-retarget
format. A LUCID ``robot_filtered`` clip carries only ``dof``,
``root_trans_offset``, ``root_rot`` and ``fps``, so it raises ``KeyError:
'joint_pos'``. The four body arrays are not stored anywhere; they are forward
kinematics, which is why no LUCID clip had ever been converted.

This computes them, using the same MuJoCo model the runner's robot is described
by and the same clip loader the validated sim2sim player uses, so resampling,
joint ordering and the quaternion convention cannot drift between the two.

THE FOURTEEN BODIES
-------------------
Order is taken from the training config's ``commands.motion.body_names`` and is
load-bearing: the anchor orientation the policy consumes is the FIRST body's.
Each name is resolved against the deploy model by name, never by a hardcoded
index -- the example bundle shipped in the repo carries indices from a
different (43-DoF) model, and reusing those would silently mislabel every
column.

WHAT THIS DOES NOT DO
---------------------
It does not verify that the runner's C++ reader parses these files into the
same numbers. Nothing here has been on a robot. See the bundle README.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

#: Training config `manager_env.commands.motion.body_names`, in order.
BODY_NAMES = [
    "pelvis",
    "left_hip_roll_link",
    "left_knee_link",
    "left_ankle_roll_link",
    "right_hip_roll_link",
    "right_knee_link",
    "right_ankle_roll_link",
    "torso_link",
    "left_shoulder_roll_link",
    "left_elbow_link",
    "left_wrist_yaw_link",
    "right_shoulder_roll_link",
    "right_elbow_link",
    "right_wrist_yaw_link",
]


def isaac_body_indexes(model, names, MP) -> list[int]:
    """The indices the runner's metadata.txt expects, one per tracked body.

    ``ReadMetadata`` fills ``body_part_indexes``, which ``ComputeFK`` maps with
    ``mj_idx = mujoco_to_isaaclab[i-1] + 1``. So these are ISAACLAB joint indices
    offset by one, with 0 reserved for the free-floating root -- not MuJoCo body
    ids. Derived here from the model rather than copied: for each tracked body,
    take the hinge joint that drives it, convert its MuJoCo dof index into the
    IsaacLab dof index, and add one.

    The result reproduces the indices in the repository's own example bundle
    exactly, which is what makes it safe to rely on.
    """
    isaac_to_mj = list(MP.ISAAC_TO_MJ)
    mj_to_isaac = [0] * len(isaac_to_mj)
    for isaac_i, mj_i in enumerate(isaac_to_mj):
        mj_to_isaac[mj_i] = isaac_i
    out = []
    for b in BODY_NAMES:
        bid = names.index(b)
        joint_adr = int(model.body_jntadr[bid])
        if joint_adr <= 0:  # the free root joint, or no joint at all
            out.append(0)
            continue
        out.append(mj_to_isaac[joint_adr - 1] + 1)
    return out


def _velocities(pos, quat, n, nb, fps):
    """Velocities exactly as MotionDataReader::ComputeGlobalVelocities derives them.

    Matching the runner's own convention matters because these columns are only
    meaningful relative to it. Three details, all from
    motion_data_reader.hpp:168-205:

    * linear is a CENTRAL difference, ``(p[f+1] - p[f-1]) * 50 / dt`` with the
      ends clamped and ``dt`` the actual frame span -- not a forward difference;
    * angular is a WORLD-frame delta, ``q[f1] * conj(q[f1-1])``, taken over one
      frame ending at ``f1`` and scaled by 50 with no ``dt`` division -- note it
      is left-multiplied, so it is not the body-frame delta the player uses;
    * both are then smoothed with a Gaussian of sigma 2, which the C++ comment
      describes as replicating ``scipy.ndimage.gaussian_filter1d(sigma=2,
      mode="nearest")``.
    """
    import numpy as np

    lin = np.zeros((n, nb, 3))
    ang = np.zeros((n, nb, 3))
    for f in range(n):
        f0 = max(f - 1, 0)
        f1 = min(n - 1, f + 1)
        dt = max(1, f1 - f0)
        lin[f] = (pos[f1] - pos[f0]) * fps / dt
        fprev = max(0, f1 - 1)
        for j in range(nb):
            q1 = _wxyz_to_xyzw(quat[f1, j])
            q0 = _wxyz_to_xyzw(quat[fprev, j])
            dq = _quat_mul_xyzw(q1, _quat_conj_xyzw(q0))
            w = min(1.0, max(-1.0, dq[3]))
            angle = 2.0 * np.arccos(w)
            sin_half = np.sqrt(max(0.0, 1.0 - w * w))
            axis = dq[:3] / sin_half if sin_half > 1e-9 else np.zeros(3)
            if angle > np.pi:
                angle -= 2.0 * np.pi
            ang[f, j] = axis * angle * fps

    try:
        from scipy.ndimage import gaussian_filter1d

        lin = gaussian_filter1d(lin, sigma=2.0, axis=0, mode="nearest")
        ang = gaussian_filter1d(ang, sigma=2.0, axis=0, mode="nearest")
    except ImportError:  # pragma: no cover - scipy is present in this env
        pass
    return lin, ang


def _wxyz_to_xyzw(q):
    return np.array([q[1], q[2], q[3], q[0]])


def _quat_conj_xyzw(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _quat_mul_xyzw(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def convert(clip_path: Path, out_dir: Path, xml: Path) -> dict:
    import mujoco
    import mujoco_player as MP

    clip = MP.load_clip(clip_path)
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)

    names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        for i in range(model.nbody)
    ]
    missing = [b for b in BODY_NAMES if b not in names]
    if missing:
        raise SystemExit(f"bodies absent from {xml}: {missing}")
    body_ids = [names.index(b) for b in BODY_NAMES]
    metadata_indexes = isaac_body_indexes(model, names, MP)

    n = len(clip.dof50)
    pos = np.zeros((n, len(body_ids), 3))
    # WXYZ. The runner declares body_quaternions_ as [timestep][body_id][wxyz]
    # (motion_data_reader.hpp:272) and its maths reads w = quat[0]
    # (math_utils.hpp:382,411). MuJoCo also stores wxyz, so xquat is written
    # through unchanged -- but the training side and tools/mujoco_player.py use
    # XYZW, so this is the one place the two conventions meet and it must not be
    # "tidied" to match the player.
    quat = np.zeros((n, len(body_ids), 4))
    for t in range(n):
        data.qpos[0:3] = clip.root_pos50[t]
        data.qpos[3:7] = MP.quat_xyzw_to_wxyz(clip.root_quat50_xyzw[t])
        data.qpos[7:] = clip.dof50[t]
        mujoco.mj_kinematics(model, data)
        for j, bid in enumerate(body_ids):
            pos[t, j] = data.xpos[bid]
            quat[t, j] = data.xquat[bid]  # MuJoCo is wxyz; the runner wants wxyz

    fps = MP.REF_FPS
    lin, ang = _velocities(pos, quat, n, len(body_ids), fps)

    out_dir = out_dir / clip.name
    out_dir.mkdir(parents=True, exist_ok=True)

    def write_csv(name: str, arr: np.ndarray, cols: list[str]) -> None:
        flat = arr.reshape(len(arr), -1)
        with (out_dir / name).open("w") as fh:
            fh.write(",".join(cols) + "\n")
            for row in flat:
                fh.write(",".join(f"{v:.6f}" for v in row) + "\n")

    write_csv(
        "joint_pos.csv", clip.dof50, [f"joint_{i}" for i in range(clip.dof50.shape[1])]
    )
    write_csv(
        "joint_vel.csv", clip.vel50, [f"joint_{i}" for i in range(clip.vel50.shape[1])]
    )
    axes3 = [f"{b}_{c}" for b in BODY_NAMES for c in ("x", "y", "z")]
    axes4 = [f"{b}_{c}" for b in BODY_NAMES for c in ("qw", "qx", "qy", "qz")]
    write_csv("body_pos.csv", pos, axes3)
    write_csv("body_quat.csv", quat, axes4)
    write_csv("body_lin_vel.csv", lin, axes3)
    write_csv("body_ang_vel.csv", ang, axes3)

    (out_dir / "metadata.txt").write_text(
        f"Metadata for: {clip.name}\n"
        f"{'=' * (14 + len(clip.name))}\n\n"
        f"Body part indexes:\n"
        f"[{' '.join(f'{i:2d}' for i in metadata_indexes)}]\n\n"
        f"Total timesteps: {n}\n\n"
        f"Body part names, in the same order (index 0 is the anchor):\n"
        f"{BODY_NAMES}\n\n"
        f"MuJoCo body ids resolved by name against {xml.name}:\n{body_ids}\n\n"
        f"Data arrays summary:\n"
        f"  joint_pos: ({n}, {clip.dof50.shape[1]})\n"
        f"  joint_vel: ({n}, {clip.vel50.shape[1]})\n"
        f"  body_pos_w: ({n}, {len(body_ids)}, 3)\n"
        f"  body_quat_w: ({n}, {len(body_ids)}, 4) WXYZ, w first\n"
        f"  body_lin_vel_w: ({n}, {len(body_ids)}, 3)\n"
        f"  body_ang_vel_w: ({n}, {len(body_ids)}, 3)\n\n"
        f"Source clip fps: {clip.src_fps}; resampled to {fps} Hz.\n"
        f"Body arrays are FORWARD KINEMATICS of the source joint angles on this\n"
        f"model, not recorded data. Velocities are one-frame finite differences.\n"
    )
    receipt = {
        "kind": "lucid_deploy_motion_csv",
        "clip": str(clip_path),
        "name": clip.name,
        "frames": int(n),
        "duration_s": round(n / fps, 3),
        "fps": fps,
        "source_fps": int(clip.src_fps),
        "body_names": BODY_NAMES,
        "body_ids_in_model": body_ids,
        "metadata_body_part_indexes": metadata_indexes,
        "model": str(xml),
        "quaternion_convention": "wxyz",
        "quaternion_convention_source": (
            "motion_data_reader.hpp:272 declares [timestep][body_id][wxyz]; "
            "math_utils.hpp:411 reads w = quat[0]"
        ),
        "not_verified": [
            "that the runner's C++ MotionDataReader parses these into the same numbers",
        ],
    }
    (out_dir / "info.txt").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt


def main(argv=None) -> int:
    import mujoco_player as MP

    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--clip", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--xml", type=Path, default=MP.DEFAULT_XML)
    a = ap.parse_args(argv)
    r = convert(a.clip, a.out, a.xml)
    print(
        json.dumps({k: r[k] for k in ("name", "frames", "duration_s", "fps")}, indent=2)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
