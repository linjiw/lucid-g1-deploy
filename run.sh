#!/usr/bin/env bash
# Launch a policy from this bundle.  source env.sh first.
#
#   bash run.sh --policy deploy_dr --sim                   bench against MuJoCo
#   bash run.sh --policy deploy_dr --sim --viewer          ...and watch the robot
#   bash run.sh --policy deploy_dr --iface eth0            on the robot network
#   bash run.sh --motion walk_arc_cw_stop_001__A047        pick the clip to track
#   bash run.sh --list                                     show what is available
#   bash run.sh --help                                     this text, in full
#
# --sim starts the bundled MuJoCo robot on the DDS bus, defaults --iface to lo,
# and adds --disable-crc-check (the simulator computes no CRC). It stops the
# simulator again when the runner exits. Pass --no-auto-sim if you are already
# running sim/run_robot_sim.py yourself -- two robots on one bus both publishing
# rt/lowstate is worse than none.
#
# The simulated robot gets the elastic band and keeps its fall, the same robot
# drill.sh rehearses against. Neither is cosmetic. Without the band it collapses
# during the INIT ramp -- measured, 0.791 -> 0.131 m -- so ']' would arm the
# policy on a robot already on the floor. Without --keep-fallen the vendor's
# check_fall() resets the simulation below 0.2 m, snapping the robot back upright
# at exactly the moment an emergency stop is meant to show it going down. The
# band releases when the policy takes over, so it never helps the policy;
# --no-band turns it off. See docs/DEPLOY_SEQUENCE.md.
#
# --disable-crc-check is NEVER added without --sim. On hardware that check is
# what catches a corrupted LowState packet before you act on it. Passing it by
# hand without --sim is REFUSED rather than obeyed: the same flag also gates the
# 35 rad/s joint-velocity abort (g1_deploy_onnx_ref.cpp:2832,
# `if (body_dq[i] > 35 && !disable_crc_check_)`), so it removes two guards, not
# the one its name describes.
#
# Any other unrecognised argument is forwarded to the runner verbatim (e.g.
# --input-type gamepad), and run.sh echoes each one as it does. That echo is not
# decoration: the runner's parser is an if/else-if chain with no final else (its
# `for (int i = 4; i < argc; i++)` argv loop, g1_deploy_onnx_ref.cpp:4183-4413),
# so it drops an argument it does not know without a word -- a typo like
# --no-bnd would run with the band on and look exactly like a working flag.
#
# On a real robot run.sh asks you to confirm the safety checklist, and it reads
# that answer from /dev/tty rather than stdin -- a pipe cannot answer it. With no
# terminal it refuses. --assume-safety-checklist overrides that, deliberately.
#
# EVERY run is recorded, under results/run/<date>-<policy>[-<motion>]/:
# console.log (the whole console, written while it happens), run_info.txt
# (machine, GPU, commit, the runner binary that actually ran, the full argv, the
# exit code) and, on hardware, csv/ -- the runner's 22 per-tick CSVs plus
# metadata.json (21 `FileSink sink_*` members, state_logger.hpp:231-251, plus
# the motion_name.csv opened in state_logger.cpp:68). CSV logs default ON for a
# real robot and OFF for --sim, because on hardware every tick is data that
# cannot be reproduced; --csv-logs and --no-csv-logs override that. --log-dir
# <dir> moves the run directory to another disk and still gives each run its own
# timestamped subdirectory under it, because the runner's CSV sinks open
# append-only and suppress the header on a non-empty file (FileSink::open,
# file_sink.cpp:51-55), so two runs sharing a directory would silently
# concatenate with a time column that restarts.
#
# Those CSVs are raw capture, not a report. The only reader in this bundle is
# tools/check_runtime_parity.py, it reads exactly one of them (csv/action.csv),
# and it also needs the obs.csv the runner writes only under
# --policy-input-logfile <path> -- which run.sh does not add. Pass it by hand
# and it is forwarded; `drill.sh --parity` is the wired-up version. Nothing here
# reads the other 21, on hardware or in sim.
#
# results/ is gitignored (.gitignore:13): a run's data exists on the machine
# that produced it and nowhere else until someone copies it off.
#
# For the whole init -> stand -> policy -> stop sequence, scripted, use drill.sh.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POLICY=deploy_dr; IFACE=""; SIM=0; AUTO_SIM=1; ASSUME_SAFE=0; MOTION=""; EXTRA=()
VIEWER=0; BAND=1; CSV=auto; LOGDIR=""; CRC_OFF_BY_HAND=0
# Kept whole for run_info.txt: the loop below consumes "$@", and the argv is the
# one field that says what a run actually was rather than what run.sh defaulted
# to. The first hardware run left no record of any of this.
ARGV=("$@")
while [ $# -gt 0 ]; do case "$1" in
  --policy) POLICY="$2"; shift 2 ;;
  --iface)  IFACE="$2";  shift 2 ;;
  --sim)    SIM=1; shift ;;
  --no-auto-sim) AUTO_SIM=0; shift ;;
  --viewer) VIEWER=1; shift ;;
  --no-band) BAND=0; shift ;;
  --csv-logs) CSV=1; shift ;;
  --no-csv-logs) CSV=0; shift ;;
  --log-dir) LOGDIR="$2"; shift 2 ;;
  --assume-safety-checklist) ASSUME_SAFE=1; shift ;;
  # Caught here rather than passed through, so it can be refused on hardware
  # below. The check runs after the loop, not in this case, because --sim may
  # still be coming: `--disable-crc-check --sim` and `--sim --disable-crc-check`
  # have to mean the same thing.
  --disable-crc-check) CRC_OFF_BY_HAND=1; shift ;;
  --motion) MOTION="$2"; shift 2 ;;
  --list)
    echo "policies:"; for p in "$HERE"/policies/*.onnx; do echo "  $(basename "$p" _s8600_g1.onnx)"; done
    echo "motions:";  for m in "$HERE"/motions/*/; do echo "  $(basename "$m")"; done
    exit 0 ;;
  # Derived range, not a counted one: print to the first non-comment line and
  # drop it -- the idiom drill.sh, standby.sh and test.sh already use. The old
  # 'sed -n 2,32p' had to be bumped by hand every time the header grew, which is
  # how run.sh once printed a help text one line short (fixed in e784134 by
  # bumping the number, i.e. by rearming the same trap), and help that is short
  # by a paragraph is indistinguishable from a flag that does not exist.
  --help|-h) sed -n '2,/^[^#]/p' "$0" | sed '$d' | sed 's/^# \?//'; exit 0 ;;
  # Unrecognised arguments still go to the runner verbatim -- standby.sh forwards
  # everything it does not parse through here (its `*) PASS+=("$1")` catch-all,
  # replayed into its `bash "$HERE/run.sh" "${run_args[@]}"`) -- but each one is
  # announced, because the runner accepts them in silence:
  # its argv loop is an if/else-if chain with no final else
  # (g1_deploy_onnx_ref.cpp:4183-4413), so '--no-bnd' would run with the band on
  # and print nothing at all.
  *) echo "passing through to the runner: $1" >&2; EXTRA+=("$1"); shift ;;
esac; done

# Checked after the loop, not in the case above, so that --disable-crc-check and
# --sim mean the same thing in either order. run.sh adds this flag itself for
# --sim (the simulator computes no CRC); by hand on hardware it is refused, not
# warned about, because it disables two guards and names one: the LowState CRC
# check, and the 35 rad/s joint-velocity abort that shares its variable
# (g1_deploy_onnx_ref.cpp:2832).
if [ "$CRC_OFF_BY_HAND" -eq 1 ] && [ "$SIM" -eq 0 ]; then
  echo "REFUSING: --disable-crc-check without --sim." >&2
  echo "On hardware that check is what catches a corrupted LowState packet before" >&2
  echo "you act on it, and the same flag also disables the 35 rad/s joint-velocity" >&2
  echo "abort. Add --sim for a bench run; there is no hardware case for it." >&2
  exit 1
fi

# The runner starts on motion index 0 -- and which clip that is, is NOT DEFINED.
# motion_data_reader.hpp:685 walks the directory with a bare
# std::filesystem::directory_iterator and never sorts, so index 0 is readdir
# order: filename-hash order on ext4, and it can differ between the machine you
# bench on and the machine you deploy from. Measured here, the three shipped
# clips come back walk_ff, walk_arc, crouch_idle -- NOT alphabetical.
#
# So never hand the runner a directory of clips and assume you know which one
# 'T' will play. --motion narrows the directory to one entry, which is the only
# way to make index 0 deterministic. Use it for every hardware run.
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
RUNDIR=""        # set once the run directory exists, below
RUNNER_RC=""     # set only if the runner actually returned
RUN_EXIT_LOGGED=""   # "" | signalled | rc
RUN_PATH_SHOWN=0

# The end-of-run record is written from here, not from after the pipeline,
# because run.sh runs under `set -e` (above) and the runner exits non-zero on
# every path except the clean 'O' quit. Reproduced in this session:
#   bash -c 'set -euo pipefail; trap "echo TRAP" EXIT;
#            sh -c "exit 3" | tee /dev/null; rc=${PIPESTATUS[0]}; echo REACHED'
# printed TRAP and exited 3 -- REACHED never ran. So a crash, a failed init, or
# the Ctrl-C the key list itself offers used to leave run_info.txt with no exit
# line and the operator with no "log saved" path on screen: exactly the runs
# whose record matters most. The runner installs no signal handler (no `signal(`,
# `SIGINT` or `sigaction` anywhere in g1_deploy_onnx_ref.cpp), so Ctrl-C is
# non-zero too.
record_run_end() {
  [ -n "$RUNDIR" ] || return 0
  if [ -n "$RUNNER_RC" ] && [ "$RUN_EXIT_LOGGED" != rc ]; then
    echo "# exit     $RUNNER_RC at $(date -Is)" >> "$RUNDIR/run_info.txt"
    RUN_EXIT_LOGGED=rc
  elif [ -z "$RUNNER_RC" ] && [ -z "$RUN_EXIT_LOGGED" ]; then
    # No number is invented here. $? in a signal trap belongs to whatever ran
    # last, not to the runner, and a made-up exit code in the only record a
    # hardware run leaves is worse than an admitted gap. If the runner does
    # return afterwards, its real code is APPENDED under this line rather than
    # replacing it -- both lines are then true, and the second explains the first.
    echo "# exit     unknown at $(date -Is) -- run.sh was signalled or aborted" \
         "before the runner returned" >> "$RUNDIR/run_info.txt"
    RUN_EXIT_LOGGED=signalled
  fi
  if [ "$RUN_PATH_SHOWN" -eq 0 ]; then
    RUN_PATH_SHOWN=1
    echo
    echo "log saved: $RUNDIR"
    # Only true of the default location. --log-dir can put the run anywhere,
    # including external media, where "results/ is gitignored" is false and says
    # the wrong thing about where the data now lives. The else arm claims no
    # more than that: LOGDIR is used verbatim (`--log-dir) LOGDIR="$2"` above),
    # so a relative `--log-dir results/x` IS covered by .gitignore and still
    # lands here, because it does not match the absolute pattern.
    case "$RUNDIR" in
      "$HERE/results/"*)
        echo "           results/ is gitignored (.gitignore:13) -- copy this off the machine." ;;
      *)
        echo "           --log-dir: not the default results/ path -- check .gitignore covers it." ;;
    esac
  fi
  return 0
}
cleanup() {
  [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null
  [ -n "$MOTION" ] && [ -d "$MOTIONS_DIR" ] && rm -rf "$MOTIONS_DIR"
  record_run_end
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

if [ "$SIM" -eq 0 ] && { [ "$VIEWER" -eq 1 ] || [ "$BAND" -eq 0 ]; }; then
  echo "--viewer and --no-band configure the bundled simulator; without --sim there" >&2
  echo "is no simulator to configure. Ignoring them." >&2
fi

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
    # The same robot drill.sh rehearses against -- see the note at the top of
    # this file for what the band and --keep-fallen are each doing.
    SIM_ARGS=(--iface "$IFACE" --status-hz 0 --keep-fallen)
    if [ "$VIEWER" -eq 0 ]; then SIM_ARGS+=(--headless); fi
    if [ "$BAND"   -eq 1 ]; then SIM_ARGS+=(--band); fi
    "$SIMPY" -u "$HERE/sim/run_robot_sim.py" "${SIM_ARGS[@]}" \
      >"${TMPDIR:-/tmp}/lucid_run_sim.log" 2>&1 &
    SIM_PID=$!
    sleep 3
    if kill -0 "$SIM_PID" 2>/dev/null; then
      echo "robot    MuJoCo on the DDS bus (pid $SIM_PID), log ${TMPDIR:-/tmp}/lucid_run_sim.log"
      echo "         viewer $( [ "$VIEWER" -eq 1 ] && echo "on" || echo "off (--viewer)" )" \
           "  band $( [ "$BAND" -eq 1 ] && echo "on (released at policy start)" || echo "off (--no-band)" )" \
           "  fall kept"
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
  echo "motions  $HERE/motions/  ('N' cycles)"
  echo "         WARNING: which clip is index 0 is readdir order, not alphabetical,"
  echo "         and it is not predictable. Watch the 'Started with motion:' line"
  echo "         below, or pass --motion <name> to pin it. See --help."
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
# EVERY run records itself. The first hardware run of this bundle was lost
# because run.sh wrote nothing to disk and the operator's scrollback was the only
# copy -- of the only hardware data the project had ever produced. A console log
# costs nothing and there is no version of "we'll turn logging on next time" that
# survives contact with a robot.
#
# stdbuf -oL matters: teeing makes the runner's stdout a pipe, and C++ stdio
# switches from line- to fully-buffered on a pipe, so without it the operator
# watching the console during an INIT ramp would see output in 4 KB lumps.
#
# --log-dir names the PARENT, not the run directory. Each run still gets its own
# timestamped subdirectory under it, because the runner's CSV sinks open with
# std::ios::app and set header_written from whether the file is already
# non-empty (FileSink::open, file_sink.cpp:51-55), while StateLogger::getLogsDir
# uses the path it is given as-is (state_logger.cpp:378-390). Two runs pointed at
# one directory would therefore append into the same q.csv with no header and no
# separator, and index/time_ms restart at 0 each run -- a time column that
# silently goes backwards mid-file is worse than two directories.
RUNSTAMP="$(date +%Y%m%d-%H%M%S)-$POLICY${MOTION:+-$MOTION}"
if [ -n "$LOGDIR" ]; then RUNBASE="$LOGDIR/$RUNSTAMP"; else RUNBASE="$HERE/results/run/$RUNSTAMP"; fi
# The stamp is `date +%Y%m%d-%H%M%S`, one-second resolution, so two launches
# inside one second build the same RUNBASE. Read from source, not run:
# FileSink::open would then append into the existing q.csv with no header
# (file_sink.cpp:51-55), and the run_info.txt block below redirects with `>`, so
# the second run's header would replace the first's. Take the next free name
# instead. Not atomic (two shells in the same second could still race), but one
# robot has one operator, and the alternative is a silently merged CSV.
RUNDIR="$RUNBASE"; RUNSEQ=2
while [ -e "$RUNDIR" ]; do RUNDIR="$RUNBASE-$RUNSEQ"; RUNSEQ=$((RUNSEQ+1)); done
mkdir -p "$RUNDIR"
CONSOLE_LOG="$RUNDIR/console.log"
# CSV logs default ON for a real robot and OFF for a bench run: on hardware every
# tick is data that cannot be reproduced, in sim it is regenerable noise.
[ "$CSV" = auto ] && { [ "$SIM" -eq 1 ] && CSV=0 || CSV=1; }
if [ "$CSV" = 1 ]; then
  EXTRA+=(--enable-csv-logs --logs-dir "$RUNDIR/csv")
  mkdir -p "$RUNDIR/csv"
fi
# The CUDA device NAME is load-bearing, not trivia: InferenceEngine.cpp:136
# hashes GetCudaDeviceName() into the TensorRT engine cache key, so the same
# bundle on a different card silently rebuilds the engine -- minutes, at a
# powered robot. memory.free is the deploy-day gate (docs/DEPLOY_DAY.md, section
# "0.4 GPU headroom" -- 183 MiB free aborted reading the engine, 168 MiB
# segfaulted inside CUDA init), and only its value AT LAUNCH says
# anything. Collected outside the block below and guarded twice, because this
# runs on every launch under `set -e` (with pipefail, so a missing nvidia-smi
# would take the pipeline, and the script, down with it).
GPU_INFO=$(nvidia-smi --query-gpu=name,driver_version,memory.free,memory.total \
             --format=csv,noheader 2>/dev/null | head -1) || GPU_INFO=""
if [ -n "$GPU_INFO" ]; then
  GPU_INFO="$GPU_INFO   [name, driver, free, total]"
else
  GPU_INFO="unavailable (no nvidia-smi, or it failed)"
fi
{
  echo "# lucid-g1-deploy run"
  echo "# date     $(date -Is)"
  echo "# policy   $POLICY"
  echo "# motion   ${MOTION:-<all, index 0 is readdir order>}"
  echo "# iface    $IFACE"
  echo "# mode     $([ "$SIM" -eq 1 ] && echo SIMULATION || echo 'REAL ROBOT')"
  # describe --dirty: a bare hash describes the tree only if the tree is clean,
  # and the tree at a robot usually is not. There are no tags in this repo, so
  # --always prints the abbreviated hash; the fallback covers a copy with no
  # .git at all.
  echo "# commit   $(git -C "$HERE" describe --always --dirty 2>/dev/null || echo unknown)"
  # The binary that actually ran, not the source it is supposed to correspond
  # to: build.sh is a separate manual step, so the runner can be older than the
  # commit above and nothing else would show it. mtime and size rather than a
  # checksum, and not for cost: this binary is 5,469,840 bytes and sha256 of it
  # measured 0.01 s here. A hash only says whether two records name the same
  # bytes; the mtime is the field you hold against the commit date to see that
  # the runner predates the source that is checked out.
  echo "# runner   $RUNNER"
  echo "#          $(stat -c '%y  %s bytes' "$RUNNER" 2>/dev/null || echo 'stat failed')"
  # Both argvs. The first is what a person typed; the second is what the runner
  # was actually handed, which is not derivable from the first -- run.sh adds
  # flags of its own (--disable-crc-check under --sim, --enable-csv-logs) and
  # passes unrecognised ones straight through, so a hand-passed --input-type or
  # --disable-crc-check has to be readable here rather than inferred.
  echo "# argv     bash run.sh ${ARGV[*]:-}"
  echo "# exec     $RUNNER $IFACE $ONNX $MOTIONS_DIR/ --obs-config" \
       "$HERE/config/observation_config_lucid_g1_1570.yaml ${EXTRA[*]:-}"
  echo "# tensorrt $(cat "$HERE/policies/.trt_built_with" 2>/dev/null || echo unknown)"
  echo "# gpu      $GPU_INFO"
  echo "# kernel   $(uname -sr)"
  echo "# host     $(uname -n)"
} > "$RUNDIR/run_info.txt"
echo "log      $RUNDIR"
echo

echo "Keys:"
echo "  ']'  ARM the policy. It starts running at 50 Hz, but the reference stays"
echo "       parked at frame 0: this alone does NOT play the clip."
echo "  'T'  PLAY the clip from the current frame to its end."
echo "  'N' / 'P'  next / previous motion      'R'  reset clip to frame 0 (paused)"
echo "  Enter  toggles planner mode -- and NO planner is loaded in this bundle,"
echo "       so it prints 'Planner not loaded - cannot enable' and in the same"
echo "       tick stops playback and snaps the reference back to frame 0, with"
echo "       the policy still armed (keyboard_handler.hpp:462-472). 'R' is that"
echo "       same jump asked for on purpose. Do not press Enter out of habit."
echo "  'O'  EMERGENCY STOP -- kp 0, kd 8, tau 0. Terminal: the robot goes down"
echo "       and the only way back to a stand is restarting this command."
echo "  Ctrl-C  kills the process -- NOT a stop. No damping command is sent, the"
echo "       last command stays latched on the wire, and the terminal is left"
echo "       raw (run 'stty sane'). Use 'O'."
echo "Lower case works for all of them except ']'."
echo
echo "Sequence: the runner ramps to default_angles and holds the fixed stand until"
echo "you press ']'. Support the robot from 'Init Done' until you are done."

# Not exec: the EXIT trap has to run so the simulator is stopped with the runner.
#
# set +e for the pipeline itself. Under `set -e` a non-zero runner -- which is
# every exit but the clean 'O' quit -- aborts run.sh on this line, before the
# exit code is captured or the log path is printed. A trailing `|| true` is not
# a substitute: it resets PIPESTATUS.
set +e
stdbuf -oL -eL "$RUNNER" "$IFACE" "$ONNX" "$MOTIONS_DIR/" \
  --obs-config "$HERE/config/observation_config_lucid_g1_1570.yaml" "${EXTRA[@]}" \
  2>&1 | tee -a "$CONSOLE_LOG"
# PIPESTATUS, not $?, because $? here is tee's.
RUNNER_RC=${PIPESTATUS[0]}
set -e
record_run_end
exit "$RUNNER_RC"
