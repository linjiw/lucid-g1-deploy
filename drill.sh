#!/usr/bin/env bash
# Walk the whole deployment sequence against a MuJoCo G1 on the DDS bus.
#
#   bash drill.sh                        deploy_dr, headless, ~45 s
#   bash drill.sh --policy no_dr         the control policy
#   bash drill.sh --viewer               show the MuJoCo window
#   bash drill.sh --play                 ALSO play the clip ('T'), not just arm the policy
#   bash drill.sh --latency 60           inject 60 ms of actuation latency at the robot
#   bash drill.sh --out results/my-drill  choose a fresh output directory
#   bash drill.sh --stand 2 --stop-wait 2  seconds before control and after stop
#   bash drill.sh --hold 12              stay in CONTROL for 12 s before the stop
#   bash drill.sh --motion <name>        rehearse one clip; it becomes motion index 0
#   bash drill.sh --all-motions          hand the runner all three (index 0 is readdir order)
#   bash drill.sh --parity               log obs and actions, TensorRT engine vs the ONNX
#   bash drill.sh --record run.mp4       write an mp4 of the MuJoCo robot (ffmpeg; setup.sh installs it)
#   bash drill.sh --no-band              no elastic band -- the robot sits down during INIT
#   bash drill.sh --reset-on-fall        let the sim reset below 0.2 m; this HIDES the stop
#   bash drill.sh --iface eno1           a real NIC instead of lo -- REFUSED by default
#   bash drill.sh --robot-is-powered-off the override that refusal names, see below
#
# WHAT THIS IS
#
# The same binary, the same arguments and the same operator keystrokes you would
# use on hardware, with MuJoCo on the other end of the DDS wire instead of a
# robot. Nothing here is a simulation of the deployment; it IS the deployment,
# with the physics substituted. That is the only way to rehearse the sequence
# without a robot in the room.
#
# THE INTERFACE GATE
#
# The loopback default is not a detail. On 'lo' nothing this script does can
# reach a robot; on a real NIC three of its choices go out on the wire with
# nobody asked anything:
#
#   * sim/run_robot_sim.py is started on the SAME interface (below) and
#     publishes rt/lowstate there -- on that bus it IS a robot. A G1 powered on
#     over there makes two of them, both answering the runner.
#   * the runner is launched with --disable-crc-check unconditionally, and has
#     to be: the simulator computes no CRC at all (`grep -ic crc
#     sim/run_robot_sim.py` is 0) and LowStateHandler returns before
#     low_state_buffer_.SetData on a mismatch (g1_deploy_onnx_ref.cpp:2616-2628,
#     read from source, not executed), so with the check on no LowState would
#     reach the runner and it would wait in INIT forever. That same flag also
#     switches off the joint-velocity abort -- g1_deploy_onnx_ref.cpp:2832 reads
#     `if (body_dq[i] > 35 && !disable_crc_check_)`.
#   * ']', optionally 'T', and 'O' are piped in on a timer. run.sh, on anything
#     it is not told is a bench run, first reads a confirmation from /dev/tty --
#     a pipe cannot answer it -- and only --assume-safety-checklist skips that;
#     this script has no prompt at all, deliberately -- it is the scripted
#     rehearsal.
#
# So the gate is on the INTERFACE, not on the CRC flag. A non-loopback --iface
# is refused unless --robot-is-powered-off is passed, which is exactly what you
# are asserting with it: nothing on that wire can move. An --iface the kernel
# has never heard of is refused separately and says so: none of this reasoning
# applies to an interface that is not there, and the fix is the spelling.
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
STAND_S=6
STOP_S=8
OUT=""
VIEWER=0
PLAY=0
LATENCY=0
KEEP_FALLEN=1
BAND=1
PARITY=0
RECORD=""
ROBOT_OFF=0
# Default to the clip every measured number in this bundle is about. The runner
# starts on motion index 0, and which clip that is is NOT DEFINED: motion_data_
# reader.hpp:685 uses an unsorted directory_iterator, so it is readdir order and
# machine-dependent. Without this the drill would silently rehearse whichever
# clip the filesystem happened to hand back first, not the one docs/RESULTS.md
# describes.
MOTION=walk_arc_cw_stop_001__A047
while [ $# -gt 0 ]; do case "$1" in
  --policy) POLICY="$2"; shift 2 ;;
  --iface)  IFACE="$2"; shift 2 ;;
  --hold)   HOLD="$2"; shift 2 ;;
  --out) OUT="$2"; shift 2 ;;
  --stand) STAND_S="$2"; shift 2 ;;
  --stop-wait) STOP_S="$2"; shift 2 ;;
  --viewer) VIEWER=1; shift ;;
  --play)   PLAY=1; shift ;;
  --latency) LATENCY="$2"; shift 2 ;;
  --reset-on-fall) KEEP_FALLEN=0; shift ;;
  --no-band) BAND=0; shift ;;
  --parity) PARITY=1; shift ;;
  --record) RECORD="$2"; shift 2 ;;
  --motion) MOTION="$2"; shift 2 ;;
  --all-motions) MOTION=""; shift ;;
  --robot-is-powered-off) ROBOT_OFF=1; shift ;;
  # The range is derived, not counted. Commit e784134 fixed run.sh printing
  # lines 2..21 of a 22-line header; this line was '2,50p' of a header that had
  # already grown to 56, so --help stopped at "Recovering to a" and never
  # printed the part about restarting on the floor. Print to the first
  # non-comment line and drop it: the help cannot fall behind the header again.
  --help|-h) sed -n '2,/^[^#]/p' "$0" | sed '$d' | sed 's/^# \?//'; exit 0 ;;
  *) echo "unknown option: $1"; exit 2 ;;
esac; done

# THE INTERFACE GATE (the reasoning is in the header, under that name).
iface_is_loopback() {  # 'lo', or any link the kernel flags LOOPBACK
  [ "$1" = lo ] && return 0
  command -v ip >/dev/null 2>&1 || return 1   # cannot tell -- treat it as real
  ip -o link show dev "$1" 2>/dev/null | grep -q LOOPBACK
}
if ! iface_is_loopback "$IFACE"; then
  # A name the kernel does not know is a typo, not the robot network, and
  # iface_is_loopback cannot tell the two apart: `ip -o link show dev nosuchnic`
  # prints nothing on STDOUT (the `Device "nosuchnic" does not exist.` goes to
  # stderr, which the pipeline above discards) and exits 1, exactly as it does
  # for a real NIC that is not loopback. Without this branch `--iface enp3s0` on
  # a box whose NIC is enp130s0 fell through to the rt/lowstate and
  # --disable-crc-check lecture below, with an empty ADDRS, instead of "check the
  # spelling" -- both branches exercised in isolation, the drill itself not run.
  # Refused either way, --robot-is-powered-off included: there is nothing to
  # drill on.
  #
  # All of it goes to stderr, as run.sh's refusals do: drill.sh is run from
  # scripts, and a refusal on stdout is the one line that vanishes under a pipe.
  if command -v ip >/dev/null 2>&1 && ! ip -o link show dev "$IFACE" >/dev/null 2>&1; then
    echo "REFUSING: this machine has no interface named $IFACE." >&2
    echo "  'ip -o link show dev $IFACE' reports no such device, so this is a typo" >&2
    echo "  and none of the robot-network reasoning below applies. 'ip link' lists" >&2
    echo "  what is here; the drill's own default is loopback ('bash drill.sh')." >&2
    exit 2
  fi
  ADDRS=$(ip -4 -o addr show dev "$IFACE" 2>/dev/null | awk '{print $4}' | tr '\n' ' ')
  ADDRS="${ADDRS% }"
  if [ "$ROBOT_OFF" -eq 0 ]; then
    echo "REFUSING: --iface $IFACE is not a loopback interface${ADDRS:+ (it carries $ADDRS)}." >&2
    case "$ADDRS" in *192.168.123.*)
      echo "  192.168.123.x IS the robot network -- docs/ETHERNET_AND_SDK.md." >&2 ;;
    esac
    echo "  This drill would put a MuJoCo robot on that bus publishing rt/lowstate," >&2
    echo "  run the runner with --disable-crc-check (which also disables the 35 rad/s" >&2
    echo "  joint-velocity abort) and send ']' on a timer, with no prompt to anyone." >&2
    echo "  Rehearse on loopback -- 'bash drill.sh' already defaults to --iface lo." >&2
    echo "  If the robot on that wire is powered off and unplugged, say so:" >&2
    echo "    bash drill.sh --iface $IFACE --robot-is-powered-off" >&2
    exit 2
  fi
  echo "WARNING: drilling on $IFACE${ADDRS:+ ($ADDRS)}, which is not loopback." >&2
  echo "  --robot-is-powered-off given: you are asserting that nothing on that wire" >&2
  echo "  can move. The runner still runs with no CRC check and no 35 rad/s abort," >&2
  echo "  and MuJoCo publishes rt/lowstate there for anything else listening." >&2
fi

ONNX="$HERE/policies/${POLICY}_s8600_g1.onnx"
RUNNER="$HERE/runner/target/release/g1_deploy_onnx_ref"
SIMPY="$HERE/.venv-sim/bin/python"
# One directory per drill, like run.sh's results/run/<timestamp>-... . This used
# to be a fixed results/drill/ whose two logs were truncated on every
# invocation, so the rehearsal docs/DEPLOY_G1.md asks for before EVERY hardware
# session (its "**Bench first.**" paragraph, end of section 4, which asks for it
# before every run on the robot) left exactly one record and nothing to compare
# it against. 'latest' is added because the paths printed below are how the
# parity output and the sim log are found.
LOGS="${OUT:-$HERE/results/drill/$(date +%Y%m%d-%H%M%S)-$POLICY${MOTION:+-$MOTION}}"
mkdir -p "$LOGS" "$HERE/results/drill"
LOGS="$(cd "$LOGS" && pwd)"
ln -sfn "$LOGS" "$HERE/results/drill/latest"

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
BOOT_TIMEOUT=180      # the runner loads motions and a TensorRT engine before INIT

echo "======================================================================"
echo "  DEPLOYMENT DRILL -- $POLICY on a MuJoCo G1, over DDS on '$IFACE'"
echo "======================================================================"
echo "  init (wait for the runner) -> stand $STAND_S s -> policy $HOLD s -> stop $STOP_S s"
echo "  motion        ${MOTION:-all three (index 0 is readdir order -- unpredictable)}"
echo "  elastic band  $( [ "$BAND" -eq 1 ] && echo 'ON through init and stand, released when the policy starts' || echo 'OFF -- the robot will sit down during INIT, see docs' )"
if [ "$PLAY" -eq 1 ]; then
  echo "  playback      ON -- 'T' sent after CONTROL is confirmed, the clip runs to its end"
else
  echo "  playback      OFF -- ']' arms the policy but the reference stays parked at"
  echo "                frame 0. Pass --play to send 'T' and actually track the clip."
fi
if [ "$LATENCY" != "0" ]; then
  echo "  latency       ${LATENCY} ms actuation delay, injected at the robot"
else
  echo "  latency       none (--latency <ms> injects actuation delay)"
fi
echo "  logs  $SIM_LOG"
echo "        $RUN_LOG"
echo "        results/drill/latest -> $(basename "$LOGS")"
echo

sim_args=(--iface "$IFACE" --status-hz 10 --latency-ms "$LATENCY")
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
               --target-motion-logfile "$LOGS/target.csv"
               --enable-csv-logs --logs-dir "$LOGS/csv")
  echo "    parity logging on -> $LOGS/obs.csv and $LOGS/csv/"
fi

echo "[2] starting the runner (loads motions, then the TensorRT engine)"
{
  await_log "Init Done" "$BOOT_TIMEOUT" || exit 1
  sleep "$STAND_S"; printf ']'
  await_log "transitioning to CONTROL" 10 || exit 1
  [ "$PLAY" -eq 1 ] && printf 'T'
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
MISSING=0
for pat in "Dimension match" "Init Done" \
           "transitioning to CONTROL" "Stopping G1Deploy" "Stop$"; do
  if grep -qE "$pat" "$RUN_LOG"; then
    printf "  reached   %s\n" "$(grep -ohE "$pat.*" "$RUN_LOG" | head -1 | cut -c1-64)"
  else
    printf "  MISSING   %s\n" "$pat"
    MISSING=$((MISSING + 1))
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
    MISSING=$((MISSING + 1))
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
      --obs "$LOGS/obs.csv" --actions "$ACT" --onnx "$ONNX" || MISSING=$((MISSING + 1))
  else
    echo "  no parity logs were written (obs.csv empty or action.csv missing)"
    MISSING=$((MISSING + 1))
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

[ "$MISSING" -eq 0 ]
