#!/usr/bin/env bash
# Verify this bundle end to end.  source env.sh first.
#
#   bash test.sh            run every check that does not need a robot
#   bash test.sh --quick    skip the MuJoCo rollout (the slow one)
#
# Eight checks, in dependency order. Each one is a measurement; none of them is a
# claim about hardware. What they establish, and what they deliberately do not,
# is spelled out in docs/DEPLOY_G1.md section 9.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QUICK=0; [ "${1:-}" = "--quick" ] && QUICK=1
PY="${PYTHON:-python3}"
pass=0; fail=0; skip=0; warn=0
ok()   { echo "  PASS  $*"; pass=$((pass+1)); }
bad()  { echo "  FAIL  $*"; fail=$((fail+1)); }
skipm(){ echo "  SKIP  $*"; skip=$((skip+1)); }
warnm(){ echo "  WARN  $*"; warn=$((warn+1)); }
hdr()  { echo; echo "== $* =="; }

hdr "1/8  bundle parses under the runner's own reading rules"
if $PY "$HERE/tools/verify_deploy_bundle.py" "$HERE" >/tmp/lucid_t1.log 2>&1; then
  ok "metadata, CSVs, joint count + order, quaternion order, frame counts, parity"
else
  bad "see /tmp/lucid_t1.log"; sed 's/^/        /' /tmp/lucid_t1.log | tail -8
fi

hdr "2/8  observation config accepted by the runner's parser"
# Checked against the runner source, not with a YAML library: the runner's
# ExtractValue trims whitespace and quotes only, so an inline '#' comment
# becomes part of the term name and aborts startup. A YAML parser hides that.
if $PY "$HERE/tools/validate_deploy_obs_config.py" \
     "$HERE/config/observation_config_lucid_g1_1570.yaml" \
     --expect-dim 1570 --expect-layout fused_g1_1570 \
     --runner-src "$HERE/runner/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp" \
     >/tmp/lucid_t2.log 2>&1; then
  ok "8 terms, correct order, 1570 total, all in the 76-entry registry"
else
  bad "see /tmp/lucid_t2.log"; sed 's/^/        /' /tmp/lucid_t2.log | tail -8
fi

hdr "3/8  ONNX policies load and are deterministic"
$PY - "$HERE" <<'PY' && ok "signatures and repeatability" || bad "ONNX check"
import sys, glob, os
import numpy as np, onnxruntime as ort
here = sys.argv[1]
rc = 0
for f in sorted(glob.glob(f"{here}/policies/*.onnx")):
    s = ort.InferenceSession(f, providers=["CPUExecutionProvider"])
    i = [(x.name, list(x.shape)) for x in s.get_inputs()]
    o = [(x.name, list(x.shape)) for x in s.get_outputs()]
    name = os.path.basename(f)
    if i != [("obs_dict", [1, 1570])] or o != [("action", [1, 29])]:
        print(f"        {name}: unexpected signature {i} -> {o}"); rc = 1; continue
    rng = np.random.default_rng(0)
    x = rng.standard_normal((1, 1570)).astype(np.float32)
    a1 = s.run(None, {"obs_dict": x})[0]
    a2 = s.run(None, {"obs_dict": x})[0]
    if not np.array_equal(a1, a2):
        print(f"        {name}: not deterministic"); rc = 1; continue
    if not np.isfinite(a1).all():
        print(f"        {name}: non-finite action"); rc = 1; continue
    print(f"        {name}: obs_dict[1,1570] -> action[1,29], |a| max {np.abs(a1).max():.3f}")
sys.exit(rc)
PY

hdr "4/8  golden parity traces match their policies"
$PY - "$HERE" <<'PY' && ok "recorded actions reproduce from the shipped ONNX" || bad "parity trace check"
import sys, json, glob, os
import numpy as np, onnxruntime as ort
here = sys.argv[1]; rc = 0
for d in sorted(glob.glob(f"{here}/parity/*/")):
    rec = json.load(open(os.path.join(d, "parity_receipt.json")))
    npz = np.load(os.path.join(d, "parity_vectors.npz"))
    obs, act = npz["observations"], npz["actions"]
    cand = [p for p in glob.glob(f"{here}/policies/*.onnx")
            if os.path.basename(d.rstrip("/")).split("_s")[0] in os.path.basename(p)]
    if not cand:
        print(f"        {os.path.basename(d.rstrip('/'))}: no matching policy"); rc = 1; continue
    s = ort.InferenceSession(cand[0], providers=["CPUExecutionProvider"])
    got = np.vstack([s.run(None, {"obs_dict": obs[i:i+1].astype(np.float32)})[0] for i in range(len(obs))])
    err = float(np.abs(got - act).max())
    tag = os.path.basename(d.rstrip("/"))
    print(f"        {tag}: {len(obs)} steps, max |delta| {err:.3e}")
    if err > 1e-4: rc = 1
sys.exit(rc)
PY

hdr "5/8  MuJoCo rollout with the reference controller"
if [ "$QUICK" -eq 1 ]; then
  skipm "--quick"
elif ! $PY -c "import mujoco" >/dev/null 2>&1; then
  skipm "mujoco not installed in $PY (pip install mujoco onnxruntime joblib)"
elif [ ! -f "$HERE/tools/mujoco_player.py" ]; then
  skipm "tools/mujoco_player.py not in this bundle"
else
  # The clip the player wants is a LUCID .pkl, which this bundle does not carry
  # (it ships the runner's CSV form instead). Only run if one was supplied.
  # The bundle ships the source .pkl clips so this runs with no extra setup;
  # LUCID_CLIP overrides which one is used.
  CLIP="${LUCID_CLIP:-$HERE/clips/walk_arc_cw_stop_001__A047.pkl}"
  if [ -f "$CLIP" ]; then
    out=$(mktemp -d)
    if MUJOCO_GL=egl PYOPENGL_PLATFORM=egl timeout 300 $PY "$HERE/tools/mujoco_player.py" \
         --onnx "$HERE/policies/deploy_dr_s8600_g1.onnx" --clip "$CLIP" \
         --out "$out/r.mp4" --lam 0 --seed 1 --no-video >/tmp/lucid_t5.log 2>&1; then
      $PY -c "
import json;r=json.load(open('$out/r.json'))['result']
print(f\"        outcome={r['outcome']} t_end={r['t_end']}s pelvis_z={r['pelvis_z_end']}\")"
      ok "rollout completed"
    else
      bad "see /tmp/lucid_t5.log"
    fi
    rm -rf "$out"
  else
    skipm "no clip found (expected clips/ in the bundle, or set LUCID_CLIP)"
  fi
fi

hdr "6/8  C++ runner loads the bundle"
RUNNER="$HERE/runner/target/release/g1_deploy_onnx_ref"
if [ ! -x "$RUNNER" ]; then
  skipm "runner not built -- run: bash build.sh"
else
  # It will sit waiting for LowState with no robot present, which is expected;
  # what is being checked is that it gets that far. Anything earlier is a real
  # failure: a rejected term name, a motion it cannot parse, a shape mismatch.
  timeout 90 "$RUNNER" lo \
    "$HERE/policies/deploy_dr_s8600_g1.onnx" "$HERE/motions/" \
    --obs-config "$HERE/config/observation_config_lucid_g1_1570.yaml" \
    --disable-crc-check >/tmp/lucid_t6.log 2>&1
  if grep -q "Dimension match: Configuration is valid" /tmp/lucid_t6.log \
     && grep -q "Policy engine initialized successfully" /tmp/lucid_t6.log \
     && grep -q "waiting for robot to be ready" /tmp/lucid_t6.log; then
    n=$(grep -c "✓ Loaded" /tmp/lucid_t6.log)
    ok "loaded $n motions, built a TensorRT engine, 1570 dims matched, reached the control loop"
    grep -E "^  [0-9]\. " /tmp/lucid_t6.log | sed 's/^/        /'
  else
    bad "see /tmp/lucid_t6.log"
    grep -iE "error|unknown|throw|abort" /tmp/lucid_t6.log | head -5 | sed 's/^/        /'
  fi
fi

hdr "7/8  DDS robot simulator comes up on the bus"
# The piece that makes a rehearsal possible: a MuJoCo G1 publishing rt/lowstate
# and subscribing rt/lowcmd, so the unmodified runner talks to it exactly as it
# would to hardware. This only checks that it starts, holds its standing pose and
# creates its DDS domain -- the full init/stand/policy/stop sequence is drill.sh.
SIMPY="$HERE/.venv-sim/bin/python"
if [ ! -x "$SIMPY" ]; then
  skipm "no .venv-sim -- run setup.sh (needs python3.10/3.11/3.12 for cyclonedds 0.10.2)"
elif ! PYTHONPATH="$HERE/sim:$HERE/sdk" "$SIMPY" -c \
       "import mujoco, cyclonedds.domain, unitree_sdk2py.core.channel, gear_sonic_sim.simulator_factory" \
       >/tmp/lucid_t7.log 2>&1; then
  bad "sim imports -- see /tmp/lucid_t7.log"; tail -4 /tmp/lucid_t7.log | sed 's/^/        /'
else
  if timeout 90 "$SIMPY" -u "$HERE/sim/run_robot_sim.py" --headless --duration 3 \
       >/tmp/lucid_t7.log 2>&1 && grep -q "EVENT epoch" /tmp/lucid_t7.log; then
    z=$(grep -oE "final pelvis height [0-9.]+" /tmp/lucid_t7.log | awk '{print $4}')
    if [ -n "$z" ] && [ "$(echo "$z > 0.7" | bc -l 2>/dev/null || echo 1)" = "1" ]; then
      ok "publishes rt/lowstate, holds the standing pose at ${z} m with no controller"
    else
      bad "simulator ran but did not hold its pose (pelvis ${z:-?} m)"
    fi
  else
    bad "see /tmp/lucid_t7.log"; tail -5 /tmp/lucid_t7.log | sed 's/^/        /'
  fi
fi

hdr "8/8  reference motions start near the pose the runner holds"
# A hazard that only shows up on the way to a robot. INIT ramps to
# default_angles and WAIT_FOR_CONTROL holds there; pressing ']' hands the policy
# frame 0 of the motion. If frame 0 is far from default_angles the policy has to
# close that gap in one control step, from a standing start, on its feet --
# a step it never sees in training, where every episode is reset ONTO the
# reference. This is why a clip can score 0/16 falls in evaluate.sh and still go
# down within seconds in drill.sh. Reported, not failed: it is a property of the
# clips shipped here, not a fault in the bundle.
if [ ! -f "$HERE/tools/check_motion_start.py" ]; then
  skipm "tools/check_motion_start.py not in this bundle"
elif $PY "$HERE/tools/check_motion_start.py" --quiet >/tmp/lucid_t8.log 2>&1; then
  ok "every shipped motion begins close to default_angles"
else
  warnm "some motions start far from default_angles -- the policy jumps at ']'"
  sed 's/^/        /' /tmp/lucid_t8.log | grep -E "JUMP|motions start far" | head -5
  echo "        full detail: python3 tools/check_motion_start.py"
fi

echo
echo "======================================================================"
echo "  pass $pass   fail $fail   warn $warn   skip $skip"
echo "======================================================================"
if [ "$fail" -eq 0 ]; then
cat <<'EOF'

Everything checked here passed. Note what is still NOT established:

  * anything about how a policy behaves on a robot. This bundle has reached the
    fixed stand on a real G1 once, in a gantry, with the policy never armed --
    see docs/HARDWARE_RUNS.md. No policy has ever been armed on hardware.
  * that the fixed stand holds an unsupported G1. In simulation it does not --
    see docs/DEPLOY_SEQUENCE.md.
  * that a policy survives the step from default_angles onto its reference at
    the moment you press ']'. Check 8 measures that step; on the clips shipped
    here it is large, and in the rehearsal the policy goes down within seconds.

Value-level parity IS established, but not by this script: run
`bash drill.sh --parity`, which drives the runner against the MuJoCo robot and
compares its TensorRT engine against the shipped ONNX.

Next, rehearse the deployment sequence end to end -- init, fixed stand, policy,
emergency stop -- with the real runner driving a MuJoCo G1 over DDS:

    bash drill.sh

Read docs/DEPLOY_SEQUENCE.md before you run it, and again before hardware.
EOF
fi
exit $([ "$fail" -eq 0 ] && echo 0 || echo 1)
