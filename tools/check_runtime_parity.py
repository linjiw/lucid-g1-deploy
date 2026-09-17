#!/usr/bin/env python3
"""Compare what the runner's TensorRT engine computed against onnxruntime.

This is the value-level parity check. It answers one question:

    given the observations the RUNNER built, on the machine you are deploying
    from, does its compiled TensorRT engine produce the same actions as the
    ONNX file the policy was exported to?

It needs two artefacts from a live run, which is why it could not be run until
the bundle grew a LowState source (sim/run_robot_sim.py):

    --policy-input-logfile <obs.csv>     one row per control tick, 1570 values,
                                         the exact obs_buffer_ handed to TensorRT
    --enable-csv-logs --logs-dir <dir>   writes action.csv, one row per tick

`action.csv` carries `last_action[i] = floatarr[i]` -- the RAW engine output in
IsaacLab order, before `g1_action_scale` and `default_angles` are applied
(g1_deploy_onnx_ref.cpp, CreatePolicyCommand). So the comparison is direct: no
scaling, no reordering, no inversion of anything.

    python3 tools/check_runtime_parity.py --obs obs.csv --actions logs/action.csv \\
        --onnx policies/deploy_dr_s8600_g1.onnx

MEASURED RESULT
---------------
`deploy_dr`, on an RTX 5080, FP32 requested, 499 control ticks, at the two
TensorRT versions:

                        TensorRT 10.16      TensorRT 10.13  (the pinned version)
    mean |delta|          1.10e-04            1.45e-06
    max  |delta|          5.95e-03            2.37e-03
    median per-tick max   --                  1.79e-06
    99th pct per-tick     --                  4.29e-06

`no_dr`, the OTHER shipped policy, on the SAME correctly pinned 10.13.3, same
GPU, same runner, through `drill.sh --play --parity`, 3 repeats, 648 ticks
(measured in 1ccde55):

                        no_dr               deploy_dr, same series
    mean |delta|          6.88e-05            2.07e-06
    median |delta|        5.71e-05            1.91e-06
    max  |delta|          2.78e-04            6.9e-06
    ticks > 1e-5          648 of 648          0 of 648

**There is no universal pass mark here; a policy's baseline is its own.** no_dr's
CORRECT reading, 2.78e-04, is the same order as the 1.10e-04 that a wrong
TensorRT produced for deploy_dr, so the magnitude alone does not separate "wrong
TensorRT" from "different network". Physically 2.78e-04 on the action is at most
0.15 mrad (0.009 deg) of joint target: `g1_action_scale` in
runner/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp is
`0.25 * effort_limit / stiffness`, whose LARGEST value across the 29 joints is
0.55 rad per unit action (the 7520-14 hip-yaw and waist-yaw joints; the four
4010 wrists are the smallest at 0.07), so the worst joint moves 1.5e-04 rad. It changes
nothing. Take a policy's own first reading as its baseline and compare later
runs against that; do not port deploy_dr's 1e-06 to a policy you just exported.

What the version pin does look like: at 10.16 deploy_dr's disagreement was
systematic -- every tick, mean 1.1e-04, 76x its own 1.45e-06 at the pinned 10.13,
where 498 of 499 ticks sit at ~2e-06. It moved a policy off ITS OWN baseline.
SONIC's `danger` note about using the wrong TensorRT version is not hypothetical;
it is just not a threshold you can apply to a network you have never measured.

Back to deploy_dr's 499-tick run at 10.13: its one outlier (1 tick, 2.37e-03) is
a logging artefact, not an inference difference:

  * all 29 joints are off together, so it is not a torn CSV row;
  * the neighbouring ticks are clean at ~2e-06, so the two logs are not slipped;
  * the observation step into that tick is 0.001, SMALLER than the median 0.004,
    so the engine is not reacting to an unusual input;
  * the offset is the size of ONE control tick of joint motion (that joint moves
    5e-04 to 3e-03 per tick), and the logged value falls between the previous
    and current tick's ONNX output.

`obs_buffer_` is dumped AFTER `Infer()` has already copied it into the policy's
pinned input buffer, so a dump that catches one input-thread update later than
the inference did produces exactly this. It affects the log, not the robot.

Corroborated: runs that happen to contain no such tick come out at max 5.1e-06
and 6.2e-06 over 498 ticks -- three orders tighter than the runs that do. A
numerical property would not come and go between runs of the same engine.

Row alignment is shift -1 (obs.csv is written one tick after the action derived
from it) and is not in doubt: the neighbouring shifts give 2.0 to 4.7.

Precision floor of the test: the runner writes obs.csv with std::ofstream's
default 6 significant digits. Replaying the observations with noise of exactly
that size moves the actions by 3.6e-06 -- at 10.13 that is now the same order as
the signal, so this test cannot resolve finer without raising the log precision.

The verdict this tool prints is `--tol`, default 1e-2, set by TensorRT's default
TF32 matmuls -- not by any figure above. Both shipped policies clear it with
room to spare -- deploy_dr by three orders, no_dr's worst tick (2.78e-04) by a
factor of 36 -- so a PASS here is not a claim that the engine matches some
reference magnitude.

A result MUCH larger than 1e-2 is a different problem: FP16 precision
(`--policy-precision 16`), a TensorRT version that is not the pinned 10.13, or a
policies/*.trt engine cached from a different ONNX file. `setup.sh` deletes
cached engines when the TensorRT version changes, because a stale engine fails
at robot-startup time.

A result well inside 1e-2 but far from that policy's OWN recorded baseline is
the case worth chasing, and the two baselines above are the recorded ones.
"""

from __future__ import annotations

import argparse
import csv
import sys

import numpy as np


def read_csv_rows(path: str, width: int | None) -> np.ndarray:
    """Read a numeric CSV, tolerating a trailing comma and a header line."""
    rows = []
    with open(path, newline="") as fh:
        for row in csv.reader(fh):
            cells = [c for c in row if c.strip() != ""]
            if not cells:
                continue
            try:
                vals = [float(c) for c in cells]
            except ValueError:
                continue  # header
            if width is None or len(vals) >= width:
                rows.append(vals[:width] if width else vals)
    if not rows:
        raise SystemExit(f"no numeric rows in {path}")
    return np.array(rows, dtype=np.float64)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--obs", required=True, help="--policy-input-logfile output")
    ap.add_argument("--actions", required=True, help="action.csv from --enable-csv-logs")
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--tol", type=float, default=1e-2,
                    help="max |delta| treated as a pass (default 1e-2, set by "
                         "TensorRT's default TF32 matmuls -- see the module docstring)")
    ap.add_argument("--max-rows", type=int, default=500)
    args = ap.parse_args()

    import onnxruntime as ort

    obs = read_csv_rows(args.obs, 1570)
    act = read_csv_rows(args.actions, None)

    # action.csv may carry leading timestamp/index columns; the actions are the
    # last 29 numeric columns of each row.
    if act.shape[1] < 29:
        raise SystemExit(f"action.csv has {act.shape[1]} columns, need at least 29")
    act = act[:, -29:]

    print(f"  observations  {obs.shape[0]} rows x {obs.shape[1]}")
    print(f"  engine actions{act.shape[0]:>5} rows x 29")

    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    n = min(len(obs), len(act), args.max_rows)
    if n == 0:
        raise SystemExit("no overlapping rows")
    ref = np.vstack([sess.run(None, {"obs_dict": obs[i:i + 1].astype(np.float32)})[0]
                     for i in range(n)])

    # The two files are written in the same control tick, but a dropped or extra
    # first row would make every comparison meaningless while looking like a
    # numerical failure. Find the alignment before judging the numbers.
    best, best_err = 0, None
    for shift in range(-3, 4):
        a = ref[max(0, shift):n + min(0, shift)]
        b = act[max(0, -shift):n - max(0, shift)]
        m = min(len(a), len(b))
        if m < 5:
            continue
        err = float(np.abs(a[:m] - b[:m]).max())
        if best_err is None or err < best_err:
            best, best_err = shift, err
    if best != 0:
        print(f"  note: files align at row shift {best:+d}")

    a = ref[max(0, best):n + min(0, best)]
    b = act[max(0, -best):n - max(0, best)]
    m = min(len(a), len(b))
    a, b = a[:m], b[:m]
    d = np.abs(a - b)

    print()
    print(f"  compared {m} ticks")
    print(f"  max |delta|   {d.max():.3e}")
    print(f"  mean |delta|  {d.mean():.3e}")
    print(f"  action range  onnx [{a.min():+.3f}, {a.max():+.3f}]  "
          f"engine [{b.min():+.3f}, {b.max():+.3f}]")
    worst = int(np.unravel_index(d.argmax(), d.shape)[1])
    print(f"  worst joint   index {worst}, |delta| {d[:, worst].max():.3e}")

    if d.max() <= args.tol:
        print()
        print(f"  PASS -- the TensorRT engine reproduces the ONNX policy to "
              f"{d.max():.1e} on the runner's own observations.")
        print("  This is a TF32-level agreement, not bit-exactness. See the")
        print("  module docstring for why, and for how to tighten it.")
        return 0
    print()
    print(f"  FAIL -- max |delta| {d.max():.3e} exceeds {args.tol:.1e}.")
    print("  Check, in this order: --policy-precision (FP16 costs ~1e-2),")
    print("  the installed TensorRT version (SONIC pins 10.13 on x86_64), and")
    print("  whether policies/*.trt was cached from a different ONNX file.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
