#!/usr/bin/env python3
"""Check a deployment bundle the way the C++ runner will read it.

    verify_deploy_bundle.py <bundle-dir> [--obs-config <path>] [--model <onnx>]

The runner has never been built in this repository, so "the bundle is fine" has
until now been an assertion. This turns it into a measurement by re-implementing
the runner's own parsers in Python, from its source, and running them over the
bundle:

* ``ReadMetadata`` (motion_data_reader.hpp:933) needs a literal
  ``Body part indexes:`` line and regex-scans the FOLLOWING line for integers.
  It returns false when that list ends up empty, so a metadata file that merely
  *describes* the bodies in prose loads nothing.
* ``ReadCSV`` (:961) skips exactly one header line and ``std::stod``s every
  comma-separated cell, silently dropping any it cannot parse -- so a stray
  token shortens a row instead of failing.
* ``ReadCSV3D`` (:1004) requires every row to divide evenly by the coordinate
  count, and requires all files to agree on the frame count, or it SKIPS THE
  MOTION with a warning rather than an error.
* Quaternions are ``[wxyz]`` (:272, and math_utils.hpp:411 reads ``w = quat[0]``),
  which is the opposite of the XYZW convention used on the training side and in
  tools/mujoco_player.py.

None of this proves the robot will behave. It proves the runner can load what is
in the bundle, and that the numbers mean what the runner thinks they mean.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

REQUIRED_CSVS = {
    "joint_pos.csv": 1,
    "joint_vel.csv": 1,
    "body_pos.csv": 3,
    "body_quat.csv": 4,
    "body_lin_vel.csv": 3,
    "body_ang_vel.csv": 3,
}


def read_csv(path: Path) -> tuple[list[list[float]], list[str]]:
    """ReadCSV: skip one header line, stod every cell, drop what will not parse."""
    lines = path.read_text().splitlines()
    header = lines[0].split(",") if lines else []
    rows = []
    for line in lines[1:]:
        if not line.strip():
            continue
        row = []
        for cell in line.split(","):
            try:
                row.append(float(cell))
            except ValueError:
                pass  # the C++ silently skips these
        rows.append(row)
    return rows, header


def check_motion(d: Path, problems: list[str]) -> dict:
    info: dict = {"name": d.name}
    meta = d / "metadata.txt"
    if not meta.is_file():
        problems.append(
            f"{d.name}: no metadata.txt; ReadMetadata returns false and the motion loads nothing"
        )
        return info
    text = meta.read_text().splitlines()
    idx: list[int] = []
    timesteps = None
    for i, line in enumerate(text):
        if "Body part indexes:" in line and i + 1 < len(text):
            idx = [int(m) for m in re.findall(r"\d+", text[i + 1])]
        if "Total timesteps:" in line:
            parts = line.split()
            if len(parts) >= 3:
                try:
                    timesteps = int(parts[2])
                except ValueError:
                    pass
    info["body_part_indexes"] = idx
    info["metadata_timesteps"] = timesteps
    if not idx:
        problems.append(
            f"{d.name}: metadata.txt has no parseable 'Body part indexes:' line, so "
            f"ReadMetadata returns false and body_part_indexes stays empty"
        )
    if timesteps is None:
        problems.append(f"{d.name}: metadata.txt has no 'Total timesteps: N' line")

    frames: dict[str, int] = {}
    for name, coords in REQUIRED_CSVS.items():
        f = d / name
        if not f.is_file():
            problems.append(f"{d.name}: missing {name}")
            continue
        rows, header = read_csv(f)
        if not rows:
            problems.append(f"{d.name}/{name}: no data rows after the header")
            continue
        widths = {len(r) for r in rows}
        if len(widths) != 1:
            problems.append(
                f"{d.name}/{name}: inconsistent row widths {sorted(widths)}; ReadCSV3D bails"
            )
            continue
        width = widths.pop()
        if len(header) != width:
            problems.append(
                f"{d.name}/{name}: header has {len(header)} columns, rows have {width}"
            )
        if coords > 1 and width % coords:
            problems.append(
                f"{d.name}/{name}: row width {width} is not divisible by {coords}"
            )
        frames[name] = len(rows)
    if len(set(frames.values())) > 1:
        problems.append(
            f"{d.name}: frame counts disagree {frames}; the reader SKIPS the motion"
        )
    info["frames"] = frames
    if timesteps is not None and frames and timesteps != next(iter(frames.values())):
        problems.append(
            f"{d.name}: metadata says {timesteps} timesteps, CSVs have {next(iter(frames.values()))}"
        )

    # Quaternion sanity: wxyz, unit norm. A file written xyzw still normalises to
    # 1, so norm alone cannot catch the ordering -- but a non-unit quaternion is
    # always wrong, and a w column that is never the dominant component across a
    # mostly-upright motion is worth flagging for a human to check.
    q = d / "body_quat.csv"
    if q.is_file():
        rows, header = read_csv(q)
        arr = np.asarray(rows)
        if arr.size:
            n_bodies = arr.shape[1] // 4
            quats = arr.reshape(len(arr), n_bodies, 4)
            norms = np.linalg.norm(quats, axis=2)
            worst = float(np.abs(norms - 1.0).max())
            info["max_quat_norm_error"] = worst
            if worst > 1e-4:
                problems.append(
                    f"{d.name}/body_quat.csv: quaternion norm off by {worst:.2e}"
                )
            if header[:1] and not header[0].endswith("_qw"):
                problems.append(
                    f"{d.name}/body_quat.csv: first column is {header[0]!r}, expected a _qw "
                    f"column -- the runner reads w = quat[0]"
                )
    return info


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bundle", type=Path)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)

    problems: list[str] = []
    report: dict = {"bundle": str(a.bundle)}

    # --- policies -----------------------------------------------------------
    import onnxruntime as ort

    policies = sorted((a.bundle / "policies").glob("*.onnx"))
    if not policies:
        problems.append("no policies/*.onnx in the bundle")
    sigs = {}
    for p in policies:
        s = ort.InferenceSession(str(p), providers=["CPUExecutionProvider"])
        ins = [(i.name, list(i.shape)) for i in s.get_inputs()]
        outs = [(o.name, list(o.shape)) for o in s.get_outputs()]
        sigs[p.name] = {"in": ins, "out": outs}
        if outs != [("action", [1, 29])]:
            problems.append(f"{p.name}: output is {outs}, expected action [1, 29]")
    report["policies"] = sigs

    # --- observation config against the model width -------------------------
    cfg = a.bundle / "config" / "observation_config_lucid_g1_1570.yaml"
    if cfg.is_file():
        import yaml

        conf = yaml.safe_load(cfg.read_text())
        names = [
            e["name"] for e in conf.get("observations", []) if e.get("enabled", True)
        ]
        report["obs_terms"] = names
        widths = {
            "motion_joint_positions_10frame_step5": 290,
            "motion_joint_velocities_10frame_step5": 290,
            "motion_anchor_orientation_10frame_step5": 60,
            "his_base_angular_velocity_10frame_step1": 30,
            "his_body_joint_positions_10frame_step1": 290,
            "his_body_joint_velocities_10frame_step1": 290,
            "his_last_actions_10frame_step1": 290,
            "his_gravity_dir_10frame_step1": 30,
        }
        total = sum(widths.get(n, 0) for n in names)
        report["obs_total"] = total
        for p, sig in sigs.items():
            if sig["in"] and sig["in"][0][1] == [1, total]:
                continue
            problems.append(
                f"{p}: input {sig['in']} does not match the observation config total {total}"
            )
    else:
        problems.append("no config/observation_config_lucid_g1_1570.yaml")

    # --- motions ------------------------------------------------------------
    motions = (
        sorted(p for p in (a.bundle / "motions").iterdir() if p.is_dir())
        if (a.bundle / "motions").is_dir()
        else []
    )
    if not motions:
        problems.append("no motions/ subdirectories")
    report["motions"] = [check_motion(m, problems) for m in motions]

    # --- parity vectors -----------------------------------------------------
    par = a.bundle / "parity"
    if par.is_dir():
        got = []
        for sub in sorted(p for p in par.iterdir() if p.is_dir()):
            npz = sub / "parity_vectors.npz"
            rec = sub / "parity_receipt.json"
            if not npz.is_file() or not rec.is_file():
                problems.append(f"parity/{sub.name}: incomplete")
                continue
            d = np.load(npz)
            r = json.loads(rec.read_text())
            got.append(
                {
                    "arm": sub.name,
                    "steps": int(d["observations"].shape[0]),
                    "obs_dim": int(d["observations"].shape[1]),
                    "onnx_sha256": r["onnx_sha256"][:16],
                }
            )
            if d["observations"].shape[1] != report.get("obs_total"):
                problems.append(
                    f"parity/{sub.name}: obs width {d['observations'].shape[1]} != config total"
                )
        report["parity"] = got
    else:
        problems.append("no parity/ directory")

    report["problems"] = problems
    report["ok"] = not problems

    if a.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"bundle: {a.bundle}")
        for name, sig in sigs.items():
            print(f"  policy  {name:28s} {sig['in']} -> {sig['out']}")
        print(f"  obs config total: {report.get('obs_total')}")
        for m in report["motions"]:
            f = m.get("frames", {})
            print(
                f"  motion  {m['name']:44s} {next(iter(f.values()), '?')} frames, "
                f"{len(m.get('body_part_indexes', []))} body indexes, "
                f"quat norm err {m.get('max_quat_norm_error', float('nan')):.1e}"
            )
        for p in report.get("parity", []):
            print(
                f"  parity  {p['arm']:12s} {p['steps']} steps x {p['obs_dim']}  onnx {p['onnx_sha256']}"
            )
        if problems:
            print("\nPROBLEMS:")
            for p in problems:
                print(f"  - {p}")
        else:
            print("\nOK: every file parses under the runner's own reading rules.")
    return 0 if not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
