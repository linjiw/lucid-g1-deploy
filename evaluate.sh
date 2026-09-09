#!/usr/bin/env bash
# Reproduce the DR-perturbation and latency evaluations in MuJoCo.
# source env.sh first.
#
#   bash evaluate.sh                    both sweeps, 16 seeds  (~40 min)
#   bash evaluate.sh --seeds 4          quicker, noisier
#   bash evaluate.sh --clip <name>      a different shipped clip
#   bash evaluate.sh --out <dir>        where results go (default results/)
#
# This is simulator-to-simulator: the exported ONNX policies driven in MuJoCo,
# a different physics engine from the one they trained in, with the 1,570-float
# observation rebuilt from MuJoCo state. It does not touch the C++ runner and
# does not need a robot.
#
# TWO SWEEPS, because they answer different questions:
#
#   perturbation   all randomization channels scaled together by lambda --
#                  friction, per-body mass, torso CoM, joint offsets, pushes and
#                  actuation delay. lambda 1 is the envelope deploy_dr trained
#                  under; 1.5 and 2 are past it, so both policies extrapolate.
#   latency        actuation delay alone, physics nominal. deploy_dr trained at
#                  0-60 ms; 0-80 and 0-120 ms are held out.
#
# Draws come from one seeded stream in fixed order, so at a given (lambda, seed)
# both policies face IDENTICAL physics and identical pushes. The comparison is
# paired, not two independent samples.
#
# --full-clip is used throughout. A rollout normally stops when the pelvis
# passes 0.5 m from the reference, which is a TRACKING criterion and not a fall,
# and it typically fires around 1.4 s -- a policy whose run has already ended
# cannot then be recorded as falling. Counting falls in that window undercounts
# them badly: 3 of 16 against 11 of 16 at heavy randomization. Running to the end
# of the motion is what makes "does it stay on its feet" answerable.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3}"

SEEDS=16
CLIPNAME="walk_arc_cw_stop_001__A047"
OUT="$HERE/results"
JOBS="${JOBS:-5}"
while [ $# -gt 0 ]; do case "$1" in
  --seeds) SEEDS="$2"; shift 2 ;;
  --clip)  CLIPNAME="$2"; shift 2 ;;
  --out)   OUT="$2"; shift 2 ;;
  --jobs)  JOBS="$2"; shift 2 ;;
  --help|-h) sed -n '2,12p' "$0"; exit 0 ;;
  *) echo "unknown option: $1"; exit 2 ;;
esac; done

CLIP="$HERE/clips/${CLIPNAME}.pkl"
[ -f "$CLIP" ] || { echo "no such clip: $CLIP"; echo "available:"; ls "$HERE/clips/" | sed 's/\.pkl$//;s/^/  /'; exit 1; }
$PY -c "import mujoco, onnxruntime, joblib" 2>/dev/null || {
  echo "python deps missing in $PY -- run: bash setup.sh"; exit 1; }

mkdir -p "$OUT"
ARMS="$OUT/arms.json"
$PY - "$HERE" "$ARMS" <<'PY'
import json, sys, glob, os
here, out = sys.argv[1], sys.argv[2]
arms = {}
for p in sorted(glob.glob(f"{here}/policies/*.onnx")):
    arms[os.path.basename(p).replace("_s8600_g1.onnx", "")] = p
json.dump(arms, open(out, "w"), indent=2)
print(f"arms: {', '.join(arms)}")
PY

echo
echo "clip   $CLIPNAME"
echo "seeds  $SEEDS   jobs $JOBS"
echo "out    $OUT"

echo
echo "=================== 1/2  DR perturbation ==================="
$PY "$HERE/tools/mujoco_sweep.py" \
  --out "$OUT/perturbation" --arms-json "$ARMS" --clip "$CLIP" --py "$PY" \
  --lams 0 0.5 1.0 1.5 2.0 --seeds "$SEEDS" --jobs "$JOBS" --full-clip

echo
echo "=================== 2/2  latency only ======================"
$PY "$HERE/tools/mujoco_sweep.py" \
  --out "$OUT/latency" --arms-json "$ARMS" --clip "$CLIP" --py "$PY" \
  --channels delay --lams 0 0.5 1.0 1.5 2.0 3.0 --seeds "$SEEDS" --jobs "$JOBS" --full-clip

cat <<EOF

Results written to $OUT
  perturbation/summary.md   lambda 0 / 0.5 / 1 / 1.5 / 2, all channels
  latency/summary.md        0 / 0-20 / 0-40 / 0-60 / 0-80 / 0-120 ms

Each file has two tables. The first is the scored criterion the Isaac
termination uses. The second separates what actually happened: toppling over
versus walking on but more than 0.5 m off the reference path. Read the second.

For comparison, docs/RESULTS.md carries the numbers measured on the original
machine. Small differences across machines are expected -- MuJoCo and
onnxruntime versions differ -- but the ORDERING should not change. If it does,
something is wrong with the port, not with the policies.
EOF
