#!/usr/bin/env python3
"""How far is each reference motion's first frame from the pose the runner starts in?

This is a deployment hazard that offline evaluation cannot show you.

The runner's INIT state ramps every joint to `default_angles` and holds there
(WAIT_FOR_CONTROL). When the operator presses ']' the policy takes over and is
immediately asked to track frame 0 of the reference motion. If frame 0 is far
from `default_angles`, the policy's first commanded targets jump, and it has to
recover from a step it never sees in training -- where every episode is reset
ONTO the reference, with the reference's velocities.

That difference is why a clip can score 0/16 falls in `evaluate.sh` and still go
down within seconds in `drill.sh`: the sweep starts the robot on the motion, the
rehearsal starts it in the runner's own standing pose, like the robot will.

    python3 tools/check_motion_start.py                 # every shipped motion
    python3 tools/check_motion_start.py --motion <name>
    python3 tools/check_motion_start.py --max-rms 0.15  # tighten the threshold

Exit code is 0 when every motion is inside the thresholds, 1 otherwise. It is
reported as a WARNING by test.sh rather than a failure, because it is a property
of the clips you chose to ship, not a fault in the bundle.

WHAT TO DO ABOUT A LARGE MISMATCH, in order of preference:

  1. Ship clips that begin near the default stance. This is the only option that
     removes the step rather than managing it.
  2. Give the robot the clip's starting pose before you press ']'. Nothing in
     this runner does that for you -- INIT only ever ramps to `default_angles`.
  3. Deploy with the robot supported, and expect the first second to be rough.

Do NOT silently prepend an interpolated lead-in to the clip: the observation
carries ten future reference frames, so a synthetic lead-in changes what the
policy sees as well as what it tracks, and the policy was not trained on it.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import re
import sys

HERE = Path(__file__).resolve().parent
BUNDLE = HERE.parent

# 29 hardware joints; the first 15 are the ones that carry the robot's weight.
LEG_WAIST = 15
NAMES = [
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ank_pitch", "L_ank_roll",
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ank_pitch", "R_ank_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
]


def default_angles() -> list[float]:
    """Read the ramp target from the runner's own header, not a copy of it."""
    src = (BUNDLE / "runner/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp")
    text = src.read_text()
    block = re.search(r"const std::array<double, 29> default_angles = \{(.*?)\};",
                      text, re.S)
    if not block:
        raise SystemExit(f"could not find default_angles in {src}")
    vals = [float(m) for m in re.findall(r"(-?\d+\.?\d*)\s*,?\s*//", block[1])]
    if len(vals) != 29:
        raise SystemExit(f"parsed {len(vals)} default_angles, expected 29")
    return vals


def first_frame(motion_dir: Path) -> list[float] | None:
    f = motion_dir / "joint_pos.csv"
    if not f.is_file():
        return None
    with open(f) as fh:
        for row in csv.reader(fh):
            cells = [c for c in row if c.strip()]
            try:
                vals = [float(c) for c in cells]
            except ValueError:
                continue  # header
            if len(vals) >= 29:
                return vals[:29]
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--motions", type=Path, default=BUNDLE / "motions")
    ap.add_argument("--motion", help="check just this one")
    ap.add_argument("--max-rms", type=float, default=0.20,
                    help="RMS over 29 joints, radians (default 0.20)")
    ap.add_argument("--max-joint", type=float, default=0.30,
                    help="worst single leg/waist joint, radians (default 0.30)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    da = default_angles()
    dirs = sorted(d for d in args.motions.iterdir() if d.is_dir())
    if args.motion:
        dirs = [d for d in dirs if d.name == args.motion]
        if not dirs:
            raise SystemExit(f"no such motion: {args.motion}")

    bad = 0
    for d in dirs:
        f0 = first_frame(d)
        if f0 is None:
            print(f"  {d.name:<44} no joint_pos.csv -- skipped")
            continue
        delta = [f0[i] - da[i] for i in range(29)]
        rms = math.sqrt(sum(x * x for x in delta) / 29)
        worst_i = max(range(LEG_WAIST), key=lambda i: abs(delta[i]))
        worst = abs(delta[worst_i])
        over = rms > args.max_rms or worst > args.max_joint
        bad += over
        flag = "  <-- JUMP AT ']'" if over else ""
        print(f"  {d.name:<44} RMS {rms:.3f} rad   worst {NAMES[worst_i]} "
              f"{delta[worst_i]:+.3f}{flag}")
        if over and not args.quiet:
            big = [(NAMES[i], delta[i]) for i in range(LEG_WAIST)
                   if abs(delta[i]) > 0.15]
            print(f"       {len(big)}/{LEG_WAIST} leg+waist joints over 0.15 rad: "
                  + ", ".join(f"{n} {v:+.2f}" for n, v in big[:6]))

    if bad:
        print()
        print(f"  {bad} of {len(dirs)} motions start far from the pose the runner")
        print(f"  holds before ']'. The policy will be asked to close that gap in")
        print(f"  one control step, from a standing start, with the robot's weight")
        print(f"  on its feet. Support the robot, and see this file's header for")
        print(f"  what to do about it.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
