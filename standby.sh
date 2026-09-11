#!/usr/bin/env bash
# The operator loop: stand -> arm -> play -> stop -> back to a stand.
#
#   bash standby.sh                        real robot, auto-detected NIC
#   bash standby.sh --iface enp3s0         explicit NIC
#   bash standby.sh --sim --viewer         rehearse the whole loop, no robot
#   bash standby.sh --policy no_dr         the control policy
#
# WHAT THIS IS, AND WHAT IT IS NOT
#
# The runner has no standby to return to. `operator_state.stop` is set in four
# places and cleared in NONE; `program_state_` is only ever assigned
# WAIT_FOR_CONTROL and CONTROL; and the main loop is
# `while (!operator_state.stop) { sleep(0.02); }`, so a stop does not just end
# the run, it ends the PROCESS. There is no key that takes you back to a stand.
#
# So this script does not add one. It restarts the runner, which re-enters INIT
# and ramps every joint from wherever it is to default_angles over 3 s, then
# holds the fixed stand. That ramp IS the reset to the init position, and it is
# what docs/DEPLOY_SEQUENCE.md prescribes for recovery. Nothing in the SONIC
# runner is modified or bypassed -- each cycle is an ordinary `run.sh`.
#
# THE ONE THING TO UNDERSTAND BEFORE USING IT ON A ROBOT
#
# After a stop the robot is ON THE FLOOR: 'O' writes kp 0, kd 8, tau 0, which
# removes its ability to hold any pose. THE NEXT CYCLE DOES NOT PICK IT BACK UP.
#
# Measured, two cycles against one persistent MuJoCo robot: cycle 1 stopped with
# the pelvis at 0.061 m; cycle 2's INIT ramp then commanded default_angles at
# full PD for 3 s, printed 'Init Done', and the pelvis was at 0.133 m -- still
# flat. The runner went on to arm the policy and play a whole clip with the
# robot lying on the ground, reporting every marker as normal.
#
# 'Init Done' is a TIMER, not a measurement. InitControl() interpolates for
# duration_ = 3 s and declares itself finished; it never checks that the robot
# is upright, that the pose is reachable, or that the feet are under it. So the
# software cannot tell you whether the reset worked, and it will not say so.
#
# A person has to put the robot back on its feet. That is why this script NEVER
# loops on its own -- every cycle waits for someone to confirm it. Do not
# automate past that prompt.
#
# For a scripted, non-interactive rehearsal use drill.sh. For a single run use
# run.sh. This is the loop you drive by hand.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

POLICY=deploy_dr
IFACE=""
PASS=()
SIM=0
while [ $# -gt 0 ]; do case "$1" in
  --policy) POLICY="$2"; shift 2 ;;
  --iface)  IFACE="$2"; shift 2 ;;
  --sim)    SIM=1; PASS+=(--sim); shift ;;
  --viewer) PASS+=(--viewer); shift ;;
  --help|-h) sed -n '2,36p' "$0" | sed 's/^# \?//'; exit 0 ;;
  *) PASS+=("$1"); shift ;;
esac; done

# This script is a conversation with a person. Reading the prompts from /dev/tty
# rather than stdin is the same reasoning run.sh uses for its safety checklist:
# stdin is the runner's keyboard channel, so a `read` on it would be answered by
# whatever keystroke happened to be in the pipe. With no terminal there is no
# one to ask, and the right answer is to refuse rather than to guess.
if ! { [ -e /dev/tty ] && { : >/dev/tty; } 2>/dev/null; }; then
  echo "standby.sh needs a terminal: every cycle is gated on a person confirming"
  echo "the robot is upright and supported. For a scripted rehearsal use drill.sh."
  exit 2
fi

[ -f "$HERE/policies/${POLICY}_s8600_g1.onnx" ] || {
  echo "no such policy: $POLICY"
  echo "available:"; for p in "$HERE"/policies/*.onnx; do
    echo "  $(basename "$p" _s8600_g1.onnx)"; done; exit 1; }

# The clips, with the size of the step the policy is asked to take at ']'.
# MANIFEST.json carries the measured RMS; a clip added later simply has none,
# which is reported as such rather than guessed at.
mapfile -t CLIPS < <(cd "$HERE/motions" && ls -d -- */ 2>/dev/null | sed 's#/$##' | sort)
[ "${#CLIPS[@]}" -gt 0 ] || { echo "no motions in $HERE/motions/"; exit 1; }

clip_note() {  # clip_note <name> -> "0.299 rad RMS, worst L_knee -0.548"
  "${PYTHON:-python3}" - "$HERE/MANIFEST.json" "$1" <<'PY' 2>/dev/null
import json, sys
try:
    m = json.load(open(sys.argv[1]))["start_pose_step"]
    rms = m["rms_rad"].get(sys.argv[2])
    if rms is None: raise KeyError
    worst = m.get("worst_joint", {}).get(sys.argv[2], "")
    print(f"{rms:.3f} rad RMS from the stand" + (f", worst {worst}" if worst else ""))
except Exception:
    print("start-pose step not measured for this clip")
PY
}

echo "======================================================================"
echo "  OPERATOR LOOP -- $POLICY"
echo "======================================================================"
if [ "$SIM" -eq 1 ]; then
  echo "  MODE      SIMULATION -- a MuJoCo G1 on the DDS bus, no robot"
else
  echo "  MODE      REAL ROBOT. run.sh will ask you to confirm the safety"
  echo "            checklist on every cycle. That prompt is not a formality."
fi
echo "  input     keyboard (the runner's default)"
echo "  each cycle restarts the runner: INIT ramps to default_angles over 3 s,"
echo "  then holds the fixed stand. That ramp is the reset to the init position."
echo
echo "  Keys, once the runner is up:"
echo "    ']'  arm the policy      'T'  play the clip     'O'  emergency stop"
echo "    'N'/'P'  next/prev       'R'  clip to frame 0   'I'  reheading"
echo "  ']' and 'T' are two steps. ']' alone runs the policy against a still pose."
echo

choice=""
cycle=0
while :; do
  echo "----------------------------------------------------------------------"
  echo "  clips  (each is run on its own, so motion index 0 is unambiguous)"
  for i in "${!CLIPS[@]}"; do
    printf "    %d) %-44s %s\n" "$((i+1))" "${CLIPS[$i]}" "$(clip_note "${CLIPS[$i]}")"
  done
  echo
  echo "  The number on the right is how far the clip's first frame is from the"
  echo "  pose the runner holds. The policy must close that in ONE control step,"
  echo "  standing, the instant you press ']'. Smallest is least violent."
  echo
  if [ -n "$choice" ]; then
    printf "  clip number, Enter to repeat #%s, or q to quit: " "$choice"
  else
    printf "  clip number, or q to quit: "
  fi
  read -r ans </dev/tty || ans=q
  case "$ans" in
    q|Q) echo "  done."; exit 0 ;;
    "")  [ -n "$choice" ] || { echo "  no previous clip -- pick a number."; continue; } ;;
    *[!0-9]*|"") echo "  not a number."; continue ;;
    *)   if [ "$ans" -lt 1 ] || [ "$ans" -gt "${#CLIPS[@]}" ]; then
           echo "  out of range."; continue
         fi
         choice="$ans" ;;
  esac
  CLIP="${CLIPS[$((choice-1))]}"

  cycle=$((cycle+1))
  echo
  echo "======================================================================"
  echo "  CYCLE $cycle -- $CLIP"
  echo "======================================================================"
  echo "  Wait for 'Init Done', then ']' to arm, 'T' to play, 'O' to stop."
  echo

  run_args=(--policy "$POLICY" --motion "$CLIP" "${PASS[@]}")
  [ -n "$IFACE" ] && run_args+=(--iface "$IFACE")
  bash "$HERE/run.sh" "${run_args[@]}"
  rc=$?

  echo
  echo "----------------------------------------------------------------------"
  if [ "$rc" -eq 0 ]; then
    echo "  cycle $cycle ended. If you pressed 'O', the robot is on the floor:"
    echo "  kp 0, kd 8 is not a stand, it is the removal of one."
  else
    echo "  run.sh exited $rc -- read the output above before going again."
  fi
  echo
  echo "  THE NEXT CYCLE WILL NOT STAND THE ROBOT UP. Measured in simulation:"
  echo "  restarted on a robot lying at 0.06 m, the INIT ramp commanded"
  echo "  default_angles at full PD, printed 'Init Done' on its 3 s timer with"
  echo "  the pelvis still at 0.13 m, and then armed and played a whole clip"
  echo "  with the robot flat on the ground. InitControl() never checks that"
  echo "  the robot is upright, so it cannot warn you."
  echo
  echo "  Put the robot back on its feet yourself and keep it supported. Only"
  echo "  then does the ramp to default_angles mean anything."
  echo
  printf "  reset to the init pose and go again? [y/N] "
  read -r again </dev/tty || again=n
  case "$again" in
    y|Y|yes|YES) echo ;;
    *) echo "  stopping here."; exit 0 ;;
  esac
done
