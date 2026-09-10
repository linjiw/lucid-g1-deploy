#!/usr/bin/env bash
# Walk the whole deployment sequence against a MuJoCo G1 on the DDS bus.
#
#   bash drill.sh                        deploy_dr, headless, ~45 s
#   bash drill.sh --policy no_dr         the control policy
#   bash drill.sh --viewer               show the MuJoCo window
#   bash drill.sh --play                 ALSO play the clip ('T'), not just arm the policy
#   bash drill.sh --hold 12              stay in CONTROL for 12 s before the stop
#   bash drill.sh --iface eno1           use a real NIC instead of loopback
#
# WHAT THIS IS
#
# The same binary, the same arguments and the same operator keystrokes you would
# use on hardware, with MuJoCo on the other end of the DDS wire instead of a
# robot. Nothing here is a simulation of the deployment; it IS the deployment,
# with the physics substituted. That is the only way to rehearse the sequence
# without a robot in the room.
#
# THE SEQUENCE, and what the runner actually does at each step
#
#   1  INIT               The runner waits for the first LowState, then ramps
#                         every joint from wherever it is to default_angles over
#                         3 s under full PD. Prints "Init Done".
#   2  WAIT_FOR_CONTROL   Holds default_angles under full PD and re-checks
#                         safety at 50 Hz. THIS IS THE FIXED STAND. The policy
#                         is not running. You can leave it here indefinitely.
#   3  CONTROL            ']' sets operator_state.start. The policy runs at
#                         50 Hz. NOTE: this arms the policy, it does not start
#                         the clip -- see 3b.
#   3b PLAYBACK           'T' sets operator_state.play, and nothing else does:
#                         the flag is only ever assigned true in
#                         keyboard_handler.hpp (bound to 'T'), the gamepad
#                         manager and the ZMQ manager. Until it is set,
#                         current_frame_ never increments and the 10-frame
#                         reference stack is ten copies of frame 0 -- the policy
#                         is running, but tracking a still pose. This drill only
#                         sends 'T' with --play. The key is the same on hardware;
#                         docs/DEPLOY_SEQUENCE.md has the full operator table.
#   4  STOP               'O' sets operator_state.stop. The control loop returns
#                         at its first line from then on, the threads are joined
#                         and one damping command is written: kp 0, kd 8, tau 0.
#
# READ THIS BEFORE YOU EXPECT STEP 5
#
# There is no step 5. `operator_state.stop` is set in four places in the runner
# and cleared in none of them, and `program_state_` only ever moves forward. A
# stop is TERMINAL: the process does not go back to a stand, and the damping
# command it leaves behind is not a stand -- it is zero stiffness with a little
# damping, which for a standing humanoid means it goes down. Recovering to a
# stable stand means restarting the runner, which re-enters INIT and ramps back
# to default_angles from wherever the joints ended up. On hardware that restart
# happens while the robot is on the floor or in your harness, not from a stand.
#
# This drill demonstrates that honestly: it shows the pelvis height falling after
# the stop, and then performs the restart-to-stand as a separate, explicit step.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

POLICY=deploy_dr
IFACE=lo
HOLD=10
VIEWER=0
PLAY=0
KEEP_FALLEN=1
BAND=1
PARITY=0
RECORD=""
# Default to the clip every measured number in this bundle is about. The runner
# starts on motion index 0 = whatever sorts first, which is crouch_idle, so
# without this the drill silently rehearses a different motion than
# docs/RESULTS.md describes.
MOTION=walk_arc_cw_stop_001__A047
while [ $# -gt 0 ]; do case "$1" in
  --policy) POLICY="$2"; shift 2 ;;
  --iface)  IFACE="$2"; shift 2 ;;
  --hold)   HOLD="$2"; shift 2 ;;
  --viewer) VIEWER=1; shift ;;
  --play)   PLAY=1; shift ;;
  --reset-on-fall) KEEP_FALLEN=0; shift ;;
  --no-band) BAND=0; shift ;;
  --parity) PARITY=1; shift ;;
  --record) RECORD="$2"; shift 2 ;;
  --motion) MOTION="$2"; shift 2 ;;
  --all-motions) MOTION=""; shift ;;
  --help|-h) sed -n '2,50p' "$0"; exit 0 ;;
  *) echo "unknown option: $1"; exit 2 ;;
esac; done

ONNX="$HERE/policies/${POLICY}_s8600_g1.onnx"
RUNNER="$HERE/runner/target/release/g1_deploy_onnx_ref"
SIMPY="$HERE/.venv-sim/bin/python"
LOGS="$HERE/results/drill"
mkdir -p "$LOGS"

MOTIONS_DIR="$HERE/motions"
if [ -n "$MOTION" ]; then
  [ -d "$HERE/motions/$MOTION" ] || {
    echo "no such motion: $MOTION"; echo "available:"
    for m in "$HERE"/motions/*/; do echo "  $(basename "$m")"; done; exit 1; }
  MOTIONS_DIR=$(mktemp -d)
  ln -s "$HERE/motions/$MOTION" "$MOTIONS_DIR/"
  trap 'rm -rf "$MOTIONS_DIR"' EXIT
fi

for f in "$ONNX" "$RUNNER" "$SIMPY"; do
  [ -e "$f" ] || { echo "missing: $f"; echo "run setup.sh and build.sh first"; exit 1; }
done

SIM_LOG="$LOGS/sim.log"; RUN_LOG="$LOGS/runner.log"
: >"$RUN_LOG"; : >"$SIM_LOG"
STAND_S=6; STOP_S=8   # how long to hold the fixed stand, and to watch after the stop
BOOT_TIMEOUT=180      # the runner loads motions and a TensorRT engine before INIT

echo "======================================================================"
echo "  DEPLOYMENT DRILL -- $POLICY on a MuJoCo G1, over DDS on '$IFACE'"
echo "======================================================================"
echo "  init (wait for the runner) -> stand $STAND_S s -> policy $HOLD s -> stop $STOP_S s"
echo "  motion        ${MOTION:-all three (runner starts on whichever sorts first)}"
echo "  elastic band  $( [ "$BAND" -eq 1 ] && echo 'ON through init and stand, released when the policy starts' || echo 'OFF -- the robot will sit down during INIT, see docs' )"
if [ "$PLAY" -eq 1 ]; then
  echo "  playback      ON -- 'T' sent 1 s after ']', the clip runs to its end"
else
  echo "  playback      OFF -- ']' arms the policy but the reference stays parked at"
  echo "                frame 0. Pass --play to send 'T' and actually track the clip."
fi
echo "  logs  $SIM_LOG"
echo "        $RUN_LOG"
echo

sim_args=(--iface "$IFACE" --status-hz 2)
[ -n "$RECORD" ] && sim_args+=(--record "$RECORD")
[ "$BAND" -eq 1 ] && sim_args+=(--band)
[ "$VIEWER" -eq 1 ] || sim_args+=(--headless)
[ "$KEEP_FALLEN" -eq 1 ] && sim_args+=(--keep-fallen)

echo "[1] starting the robot (MuJoCo on the DDS bus)"
"$SIMPY" -u "$HERE/sim/run_robot_sim.py" "${sim_args[@]}" >"$SIM_LOG" 2>&1 &
SIM_PID=$!
sleep 3
kill -0 $SIM_PID 2>/dev/null || { echo "  simulator died:"; tail -20 "$SIM_LOG"; exit 1; }
echo "    up (pid $SIM_PID)"

# Keystrokes on a timer, into the runner's stdin. The runner puts stdin into
# raw non-blocking mode with termios; that call fails harmlessly on a pipe and
# the non-blocking read() still delivers the bytes, so a pipe works exactly like
# a person typing. ']' = start control, 'O' = emergency stop.
# Wait for a line to appear in the runner's log. The runner spends 10-30 s
# loading motions and building or loading its TensorRT engine before it ever
# looks for a robot, and that time is machine-dependent -- keying the drill to a
# fixed sleep is how you end up pressing ']' at a robot that is still booting.
await_log() {  # await_log <pattern> <timeout-s>
  local n=$(( ${2} * 10 ))
  for _ in $(seq 1 "$n"); do grep -q "$1" "$RUN_LOG" 2>/dev/null && return 0; sleep 0.1; done
  return 1
}

# Value-level parity needs two logs the runner only writes on request: the exact
# 1570-float observations it fed to TensorRT, and the raw engine output.
parity_args=()
if [ "$PARITY" -eq 1 ]; then
  rm -rf "$LOGS/csv"; mkdir -p "$LOGS/csv"
  parity_args=(--policy-input-logfile "$LOGS/obs.csv"
               --enable-csv-logs --logs-dir "$LOGS/csv")
  echo "    parity logging on -> $LOGS/obs.csv and $LOGS/csv/"
fi

echo "[2] starting the runner (loads motions, then the TensorRT engine)"
{
  await_log "Init Done" "$BOOT_TIMEOUT" || true
  sleep "$STAND_S"; printf ']'
  [ "$PLAY" -eq 1 ] && { sleep 1; printf 'T'; }
  sleep "$HOLD";    printf 'O'
  sleep "$STOP_S"
} | "$RUNNER" "$IFACE" "$ONNX" "$MOTIONS_DIR/" \
      --obs-config "$HERE/config/observation_config_lucid_g1_1570.yaml" \
      --disable-crc-check "${parity_args[@]}" 2>&1 \
    | "$SIMPY" -u "$HERE/tools/stamp.py" >"$RUN_LOG" &
RUN_PID=$!

if await_log "Init Done" "$BOOT_TIMEOUT"; then
  echo "[3] INIT ramp complete -> WAIT_FOR_CONTROL. This is the fixed stand."
else
  echo "[3] TIMED OUT waiting for 'Init Done' after ${BOOT_TIMEOUT}s -- see $RUN_LOG"
fi
sleep "$STAND_S"
if [ "$PLAY" -eq 1 ]; then
  echo "[4] ']' sent -> CONTROL. Policy driving at 50 Hz."
  sleep 1; echo "[4b] 'T' sent -> reference playback started."
else
  echo "[4] ']' sent -> CONTROL. Policy at 50 Hz, reference parked at frame 0 (no --play)."
fi
sleep "$HOLD";    echo "[5] 'O' sent -> emergency stop. kp 0, kd 8, tau 0."
sleep "$STOP_S"

wait $RUN_PID 2>/dev/null
kill $SIM_PID 2>/dev/null
wait $SIM_PID 2>/dev/null
# A `pkill -f "sim/run_robot_sim.py"` was here and is a foot-gun: -f matches whole
# command lines, so it kills any shell whose command line merely MENTIONS the
# script -- including the one running this drill, which then exits 144 and
# leaves the simulator behind. The kill above already targets the right PID.
sleep 1

echo
echo "======================================================================"
echo "  WHAT THE RUNNER REPORTED"
echo "======================================================================"
# The keyboard interface prints nothing when 'O' is pressed -- it only sets
# stop_control, which the control loop turns into operator_state.stop. The
# evidence that the stop landed is the shutdown pair, plus kd=8 at the robot.
for pat in "Dimension match" "Init Done" \
           "transitioning to CONTROL" "Stopping G1Deploy" "Stop$"; do
  if grep -qE "$pat" "$RUN_LOG"; then
    printf "  reached   %s\n" "$(grep -ohE "$pat.*" "$RUN_LOG" | head -1 | cut -c1-64)"
  else
    printf "  MISSING   %s\n" "$pat"
  fi
done

if [ "$PLAY" -eq 1 ]; then
  if grep -q "Playing motion" "$RUN_LOG"; then
    printf "  reached   %s\n" "$(grep -ohE "Playing motion.*" "$RUN_LOG" | head -1 | cut -c1-64)"
    if grep -qE "completed\.$" "$RUN_LOG"; then
      printf "  reached   %s\n" "$(grep -ohE "Motion index.*completed\." "$RUN_LOG" | head -1 | cut -c1-64)"
    else
      printf "  PARTIAL   clip did not reach its end before the stop -- raise --hold\n"
    fi
  else
    printf "  MISSING   'T' was sent but the runner never reported playback\n"
  fi
else
  printf "  skipped   reference playback ('T') -- not sent without --play\n"
fi

echo
echo "======================================================================"
echo "  WHAT THE ROBOT DID  (pelvis height and the gains it was commanded)"
echo "======================================================================"
"$SIMPY" - "$SIM_LOG" "$RUN_LOG" <<'PY'
import re, sys

sim_log, run_log = sys.argv[1], sys.argv[2]

# Simulator status lines carry t relative to the simulator's own start, plus one
# EVENT line giving that start as a unix timestamp. The runner log is stamped in
# unix time by tools/stamp.py. Aligning on the epoch is what lets the phases below
# be the runner's ACTUAL transitions rather than the drill's intended schedule.
rows, sim_epoch = [], None
for line in open(sim_log):
    if (m := re.search(r"EVENT epoch ([\d.]+)", line)):
        sim_epoch = float(m[1]); continue
    m = re.search(r"t=\s*([\d.]+)s\s+pelvis=\(([-+\d.]+),([-+\d.]+),([\d.]+)\)m\s+"
                  r"lowcmd=(\w+)\s+kp\[0\]=\s*([\d.]+) kd\[0\]=\s*([\d.]+)", line)
    if m:
        rows.append((float(m[1]), float(m[4]), m[5] == "yes", float(m[6]), float(m[7]),
                     float(m[2]), float(m[3])))

if not rows or sim_epoch is None:
    print("  no usable simulator status -- see the sim log"); raise SystemExit(1)

def when(pattern):
    """Sim-relative time of the first runner log line matching pattern."""
    for line in open(run_log):
        parts = line.split(" ", 1)
        if len(parts) == 2 and re.search(pattern, parts[1]):
            try:
                return float(parts[0]) - sim_epoch
            except ValueError:
                return None
    return None

t_init_done = when(r"Init Done")
t_control   = when(r"transitioning to CONTROL")
t_stop      = when(r"Stopping G1Deploy")
t_first_cmd = next((float(m[1]) for line in open(sim_log)
                    if (m := re.search(r"EVENT first_lowcmd t=([\d.]+)", line))), None)

edges = []
if t_first_cmd is not None:
    edges.append(("held, no cmd", 0.0, t_first_cmd))
    edges.append(("INIT ramp", t_first_cmd, t_init_done if t_init_done else 1e9))
if t_init_done and t_control:
    edges.append(("FIXED STAND", t_init_done, t_control))
if t_control:
    edges.append(("POLICY", t_control, t_stop if t_stop else 1e9))
if t_stop:
    edges.append(("AFTER STOP", t_stop, 1e9))

print(f"  {'phase':<15}{'window':>14}  {'pelvis z':>15}  {'moved':>6}  "
      f"{'kp[0]':>7} {'kd[0]':>6}")
for name, a, b in edges:
    seg = [r for r in rows if a <= r[0] < b]
    if not seg:
        print(f"  {name:<15}{'(no samples)':>14}")
        continue
    # Gains are reported from the FIRST sample in the window: the last sample of
    # the policy window can already carry the damping command that ends it.
    moved = ((seg[-1][5] - seg[0][5]) ** 2 + (seg[-1][6] - seg[0][6]) ** 2) ** 0.5
    print(f"  {name:<15}{seg[0][0]:5.1f}-{seg[-1][0]:5.1f}s  "
          f"{seg[0][1]:6.3f} -> {seg[-1][1]:6.3f}  {moved:6.2f}  "
          f"{seg[0][3]:7.1f} {seg[0][4]:6.1f}")

print()
stand = [r for r in rows if t_init_done and t_control and t_init_done <= r[0] < t_control]
if stand:
    zs = [r[1] for r in stand]
    print(f"  fixed stand held {min(zs):.3f}-{max(zs):.3f} m for "
          f"{stand[-1][0] - stand[0][0]:.1f}s under kp={stand[-1][3]:.0f} kd={stand[-1][4]:.0f}")
pol = [r for r in rows if t_control and t_stop and t_control <= r[0] < t_stop]
if pol:
    trav = ((pol[-1][5] - pol[0][5]) ** 2 + (pol[-1][6] - pol[0][6]) ** 2) ** 0.5
    print(f"  during the policy the robot travelled {trav:.2f} m horizontally, "
          f"ending at ({pol[-1][5]:+.2f}, {pol[-1][6]:+.2f})")
    if trav < 0.30:
        print("  It barely moved. The reference clip does move, so this is the")
        print("  drift in README Limits #1: the observation carries no horizontal")
        print("  position term, so the policy cannot tell it is off the path.")

after = [r for r in rows if t_stop and r[0] >= t_stop]
if after:
    print(f"  after the stop the commanded gains are kp={after[-1][3]:.0f} "
          f"kd={after[-1][4]:.0f} -- zero stiffness")
    print(f"  pelvis went {after[0][1]:.3f} -> {after[-1][1]:.3f} m")
    if after[-1][1] < 0.35:
        print()
        print("  The robot is on the floor. That is what this emergency stop IS:")
        print("  it removes stiffness. Nothing about the policy or the amount of")
        print("  randomization changes it. If the robot must stay up when you hit")
        print("  stop, that has to come from hardware -- a harness or a gantry.")
PY

if [ "$PARITY" -eq 1 ]; then
  echo
  echo "======================================================================"
  echo "  VALUE-LEVEL PARITY  (TensorRT engine vs the shipped ONNX)"
  echo "======================================================================"
  ACT=$(find "$LOGS/csv" -name "action.csv" | head -1)
  if [ -s "$LOGS/obs.csv" ] && [ -n "$ACT" ]; then
    "${PYTHON:-python3}" "$HERE/tools/check_runtime_parity.py" \
      --obs "$LOGS/obs.csv" --actions "$ACT" --onnx "$ONNX" || true
  else
    echo "  no parity logs were written (obs.csv empty or action.csv missing)"
  fi
fi

echo
echo "======================================================================"
echo "  RECOVERY TO A STABLE STAND"
echo "======================================================================"
cat <<'EOF'
  The runner cannot do this without being restarted: operator_state.stop is
  never cleared and program_state_ never moves backwards. Recovery is:

      bash run.sh --policy <p> --iface <if>        # re-enters INIT

  which ramps every joint from wherever it is back to default_angles over 3 s
  and then holds the fixed stand. On hardware, do that with the robot supported
  -- from a harness, a gantry, or hands -- because the ramp assumes the feet can
  take load, and after a stop they usually cannot.

  Full sequence, hardware safety and the wiring: docs/DEPLOY_SEQUENCE.md
EOF
