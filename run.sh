#!/usr/bin/env bash
# Launch a policy from this bundle.  source env.sh first.
#
#   bash run.sh --policy deploy_dr --sim                   bench against MuJoCo
#   bash run.sh --policy deploy_dr --iface eth0            on the robot network
#   bash run.sh --motion walk_arc_cw_stop_001__A047        pick the clip to track
#   bash run.sh --list                                     show what is available
#
# --sim starts the bundled MuJoCo robot on the DDS bus, defaults --iface to lo,
# and adds --disable-crc-check (the simulator computes no CRC). It stops the
# simulator again when the runner exits. Pass --no-auto-sim if you are already
# running sim/run_robot_sim.py yourself -- two robots on one bus both publishing
# rt/lowstate is worse than none.
#
# --disable-crc-check is NEVER added without --sim. On hardware that check is
# what catches a corrupted LowState packet before you act on it.
#
# On a real robot run.sh asks you to confirm the safety checklist, and it reads
# that answer from /dev/tty rather than stdin -- a pipe cannot answer it. With no
# terminal it refuses. --assume-safety-checklist overrides that, deliberately.
#
# For the whole init -> stand -> policy -> stop sequence, scripted, use drill.sh.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POLICY=deploy_dr; IFACE=""; SIM=0; AUTO_SIM=1; ASSUME_SAFE=0; MOTION=""; EXTRA=()
while [ $# -gt 0 ]; do case "$1" in
  --policy) POLICY="$2"; shift 2 ;;
  --iface)  IFACE="$2";  shift 2 ;;
  --sim)    SIM=1; shift ;;
  --no-auto-sim) AUTO_SIM=0; shift ;;
  --assume-safety-checklist) ASSUME_SAFE=1; shift ;;
  --motion) MOTION="$2"; shift 2 ;;
  --list)
    echo "policies:"; for p in "$HERE"/policies/*.onnx; do echo "  $(basename "$p" _s8600_g1.onnx)"; done
    echo "motions:";  for m in "$HERE"/motions/*/; do echo "  $(basename "$m")"; done
    exit 0 ;;
  --help|-h) sed -n '2,21p' "$0" | sed 's/^# \?//'; exit 0 ;;
  *) EXTRA+=("$1"); shift ;;
esac; done

# The runner starts on motion index 0, which is whatever sorts first in the
# directory it is handed -- crouch_idle, for the three clips shipped here. That
# is rarely what you want and it is not the clip the measured results in
# docs/RESULTS.md describe. --motion narrows the directory to one entry, which
# makes that clip index 0. ('N' still cycles motions at runtime.)
MOTIONS_DIR="$HERE/motions"
if [ -n "$MOTION" ]; then
  [ -d "$HERE/motions/$MOTION" ] || {
    echo "no such motion: $MOTION"; echo "available:"
    for m in "$HERE"/motions/*/; do echo "  $(basename "$m")"; done; exit 1; }
  MOTIONS_DIR=$(mktemp -d)
  ln -s "$HERE/motions/$MOTION" "$MOTIONS_DIR/"
fi

RUNNER="$HERE/runner/target/release/g1_deploy_onnx_ref"
[ -x "$RUNNER" ] || { echo "runner not built. Run: bash build.sh"; exit 1; }
ONNX="$HERE/policies/${POLICY}_s8600_g1.onnx"
[ -f "$ONNX" ] || { echo "no such policy: $POLICY  (try --list)"; exit 1; }

# --sim is a bench run: loopback unless told otherwise.
[ "$SIM" -eq 1 ] && [ -z "$IFACE" ] && IFACE=lo

if [ -z "$IFACE" ]; then
  IFACE=$(ip -4 addr show 2>/dev/null | awk '/^[0-9]+:/{gsub(/:$/,"",$2);i=$2}/inet 192\.168\.123\./{print i;exit}')
  [ -n "$IFACE" ] && echo "auto-detected robot interface: $IFACE" \
    || { echo "no 192.168.123.x interface found; pass --iface (use 'lo' for a bench run)"; exit 1; }
fi
SIM_PID=""
cleanup() {
  [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null
  [ -n "$MOTION" ] && [ -d "$MOTIONS_DIR" ] && rm -rf "$MOTIONS_DIR"
  return 0
}
trap cleanup EXIT INT TERM

# Is a MuJoCo robot already on the bus?
#
# `pgrep -f run_robot_sim.py` is NOT good enough: -f matches the whole command
# line, so any shell that merely MENTIONS the script name -- including the one
# that launched this script -- matches too. That false positive would make
# run.sh silently decline to start a robot, and the runner would then wait for a
# LowState that never comes. Count only processes that are actually python.
robot_on_bus() {
  local pid exe
  for pid in $(pgrep -f "run_robot_sim\.py" 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    exe=$(readlink -f "/proc/$pid/exe" 2>/dev/null) || continue
    case "$exe" in *python*) return 0 ;; esac
  done
  return 1
}

if [ "$SIM" -eq 1 ]; then
  EXTRA+=(--disable-crc-check)
  SIMPY="${LUCID_SIM_PYTHON:-$HERE/.venv-sim/bin/python}"
  if [ "$AUTO_SIM" -eq 0 ]; then
    echo "robot    (--no-auto-sim: start sim/run_robot_sim.py yourself)"
  elif robot_on_bus; then
    echo "robot    a MuJoCo robot is already running -- using it"
  elif [ ! -x "$SIMPY" ]; then
    echo "robot    NOT AVAILABLE: no .venv-sim. Run setup.sh, or start a robot"
    echo "         yourself. Without one the runner waits forever for LowState."
  else
    "$SIMPY" -u "$HERE/sim/run_robot_sim.py" --iface "$IFACE" --headless \
      --status-hz 0 >"${TMPDIR:-/tmp}/lucid_run_sim.log" 2>&1 &
    SIM_PID=$!
    sleep 3
    if kill -0 "$SIM_PID" 2>/dev/null; then
      echo "robot    MuJoCo on the DDS bus (pid $SIM_PID), log ${TMPDIR:-/tmp}/lucid_run_sim.log"
    else
      echo "robot    simulator failed to start -- see ${TMPDIR:-/tmp}/lucid_run_sim.log"
      SIM_PID=""
    fi
  fi
fi

echo "policy   $ONNX"
if [ -n "$MOTION" ]; then
  echo "motion   $MOTION"
else
  echo "motions  $HERE/motions/  (starts on $(basename "$(ls -d "$HERE"/motions/*/ | head -1)"); 'N' cycles)"
fi
echo "iface    $IFACE"
[ "$SIM" -eq 1 ] && echo "mode     SIMULATION (CRC check disabled)" || {
  echo "mode     REAL ROBOT"
  echo
  echo "Before you continue: is the emergency stop within reach, the robot on a"
  echo "harness or gantry, and a fallback controller ready? None of those exist"
  echo "in this software. See docs/DEPLOY_G1.md section 8."
  echo
  # Read the confirmation from the TERMINAL, not from stdin.
  #
  # stdin is the runner's keyboard channel, and it is routinely a pipe: drill.sh
  # pipes keystrokes, and anyone scripting a run does the same. A `read` on stdin
  # is then satisfied by whatever byte happens to be in that pipe -- so
  # `printf 'y' | bash run.sh --iface eth0` used to arm a humanoid with nobody in
  # the room. Reading /dev/tty cannot be satisfied by a pipe, and with no
  # controlling terminal there is no one to ask, so it fails closed.
  if [ "$ASSUME_SAFE" -eq 1 ]; then
    echo "--assume-safety-checklist given: proceeding without asking."
    echo "You are asserting the checklist above yourself."
  elif [ -e /dev/tty ] && { : >/dev/tty; } 2>/dev/null; then
    read -r -p "Proceed? [y/N] " a </dev/tty
    [ "$a" = y ] || [ "$a" = Y ] || { echo "cancelled."; exit 0; }
  else
    echo "REFUSING: no controlling terminal, so this prompt cannot be answered"
    echo "by a person. Run it from a terminal. If you really are automating a"
    echo "deployment and have the e-stop, harness and fallback in place, say so"
    echo "explicitly with --assume-safety-checklist."
    exit 1
  fi
}
# Printed in BOTH modes. It used to be simulation-only, which meant a real-robot
# run -- the one where knowing the keys matters most -- got no key list at all.
echo
echo "Keys:"
echo "  ']'  ARM the policy. It starts running at 50 Hz, but the reference stays"
echo "       parked at frame 0: this alone does NOT play the clip."
echo "  'T'  PLAY the clip from the current frame to its end."
echo "  'N' / 'P'  next / previous motion      'R'  reset clip to frame 0 (paused)"
echo "  'O'  EMERGENCY STOP -- kp 0, kd 8, tau 0. Terminal: the robot goes down"
echo "       and the only way back to a stand is restarting this command."
echo "  Ctrl-C  quit"
echo "Lower case works for all of them except ']'."
echo
echo "Sequence: the runner ramps to default_angles and holds the fixed stand until"
echo "you press ']'. Support the robot from 'Init Done' until you are done."

# Not exec: the EXIT trap has to run so the simulator is stopped with the runner.
"$RUNNER" "$IFACE" "$ONNX" "$MOTIONS_DIR/" \
  --obs-config "$HERE/config/observation_config_lucid_g1_1570.yaml" "${EXTRA[@]}"
