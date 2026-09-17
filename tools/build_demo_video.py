#!/usr/bin/env python3
"""Build the guided demo video for this bundle.

Composes title cards and real drill footage into one mp4. Nothing here is a
mock-up: the footage is a recorded run of `drill.sh`, and the numbers on the
cards are the measured ones. Regenerate the footage with

    MUJOCO_GL=egl bash drill.sh --hold 14 --record results/demo/drill_raw.mp4

then

    python3 tools/build_demo_video.py --footage results/demo/drill_raw.mp4 \\
        --out results/demo/lucid-g1-deploy-demo.mp4

Only ffmpeg and a DejaVu font are needed -- no python video libraries. Text is
passed through `drawtext=textfile=` rather than inline, so nothing has to be
escaped and the card content can contain the characters a terminal actually
prints.

The footage cut points come from the EVENT lines the simulator writes, so if you
re-record with different timings, pass the new ones with --events.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap

W, H, FPS = 1280, 720, 30
# Text column is W - 2*84 = 1112 px. DejaVu Sans Mono advances 0.602 em, so a
# 22 px caption fits 84 columns and a 23 px body fits 80. Cards are hand-set to
# stay inside that; captions are wrapped because they read as prose.
MONO_COLS = 84
BODY_COLS = 80
BG = "0x11151c"
ACCENT = "0x7fb2ff"
INK = "0xdfe6f0"
DIM = "0x8d9bb0"
WARN = "0xffb454"

FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_M = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"


def run(cmd: list[str]) -> None:
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        sys.stderr.write(" ".join(cmd[:12]) + " ...\n" + p.stderr[-2500:] + "\n")
        raise SystemExit(f"ffmpeg failed ({p.returncode})")


def card(tmp: Path, idx: int, seconds: float, title: str, body: str,
         kicker: str = "", accent: str = ACCENT) -> Path:
    """A text card: kicker, title, monospaced body."""
    out = tmp / f"seg{idx:02d}.mp4"
    over = [l for l in body.splitlines() if len(l) > BODY_COLS]
    if over:
        raise SystemExit(f"card {idx} '{title}': {len(over)} line(s) exceed "
                         f"{BODY_COLS} columns and would run off the frame:\n  "
                         + "\n  ".join(over[:3]))
    bodyf = tmp / f"body{idx:02d}.txt"
    bodyf.write_text(body.rstrip() + "\n")
    filters = [
        f"drawbox=x=0:y=0:w={W}:h=6:color={accent}@1:t=fill",
    ]
    y = 78
    if kicker:
        kf = tmp / f"kick{idx:02d}.txt"
        kf.write_text(kicker + "\n")
        filters.append(
            f"drawtext=fontfile={FONT_M}:textfile={kf}:fontcolor={DIM}:"
            f"fontsize=22:x=84:y={y}")
        y += 40
    tf = tmp / f"title{idx:02d}.txt"
    tf.write_text(title + "\n")
    filters.append(
        f"drawtext=fontfile={FONT_B}:textfile={tf}:fontcolor={INK}:"
        f"fontsize=42:x=84:y={y}")
    filters.append(
        f"drawtext=fontfile={FONT_M}:textfile={bodyf}:fontcolor={INK}:"
        f"fontsize=23:line_spacing=11:x=84:y={y + 86}")
    run(["ffmpeg", "-y", "-loglevel", "error",
         "-f", "lavfi", "-i", f"color=c={BG}:s={W}x{H}:d={seconds}:r={FPS}",
         "-vf", ",".join(filters),
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
         "-preset", "medium", str(out)])
    return out


def clip(tmp: Path, idx: int, footage: Path, start: float, seconds: float,
         label: str, sub: str, accent: str = ACCENT) -> Path:
    """A slice of the recorded drill, with a caption burned in."""
    out = tmp / f"seg{idx:02d}.mp4"
    lf = tmp / f"lab{idx:02d}.txt"
    lf.write_text(label + "\n")
    sf = tmp / f"sub{idx:02d}.txt"
    sf.write_text("\n".join(textwrap.wrap(sub, MONO_COLS)) + "\n")
    filters = [
        f"scale={W}:{H}",
        f"drawbox=x=0:y=0:w={W}:h=6:color={accent}@1:t=fill",
        f"drawbox=x=0:y={H-150}:w={W}:h=150:color={BG}@0.86:t=fill",
        f"drawtext=fontfile={FONT_B}:textfile={lf}:fontcolor={INK}:"
        f"fontsize=34:x=84:y={H-132}",
        f"drawtext=fontfile={FONT_M}:textfile={sf}:fontcolor={DIM}:"
        f"fontsize=22:line_spacing=9:x=84:y={H-82}",
    ]
    run(["ffmpeg", "-y", "-loglevel", "error",
         "-ss", str(start), "-t", str(seconds), "-i", str(footage),
         "-vf", ",".join(filters), "-r", str(FPS),
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
         "-preset", "medium", "-an", str(out)])
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--footage", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--t-init", type=float, default=19.4,
                    help="footage time of the first LowCmd (EVENT first_lowcmd)")
    ap.add_argument("--t-stand", type=float, default=23.9,
                    help="EVENT fixed_stand")
    ap.add_argument("--t-policy", type=float, default=29.0,
                    help="EVENT policy_moving")
    ap.add_argument("--t-stop", type=float, default=44.7,
                    help="when the damping command lands")
    ap.add_argument("--keep-temp", action="store_true")
    args = ap.parse_args()

    for f in (FONT_B, FONT_M):
        if not os.path.exists(f):
            raise SystemExit(f"font not found: {f}  (apt install fonts-dejavu)")
    if not shutil.which("ffmpeg"):
        raise SystemExit("ffmpeg not found")
    if not args.footage.is_file():
        raise SystemExit(f"footage not found: {args.footage}\n"
                         f"record it with:  MUJOCO_GL=egl bash drill.sh "
                         f"--hold 14 --record {args.footage}")

    tmp = Path(tempfile.mkdtemp(prefix="lucid_demo_"))
    segs: list[Path] = []
    n = 0

    def add(p: Path) -> None:
        segs.append(p)

    add(card(tmp, (n := n + 1), 5.5,
             "lucid-g1-deploy",
             "Two motion-tracking policies for the Unitree G1, and everything\n"
             "needed to put them on one: the SONIC C++ runner, the Unitree SDK,\n"
             "and a MuJoCo robot you can rehearse the whole deployment against.\n\n"
             "Copy the directory to an Ubuntu machine. Four commands to running.",
             kicker="A STANDALONE DEPLOYMENT BUNDLE"))

    add(card(tmp, (n := n + 1), 10,
             "The two policies",
             "Trained from scratch on the same three motions, same budget, same\n"
             "seed. The ONLY difference is the randomization they trained under.\n\n"
             "                    no_dr         deploy_dr\n"
             "  mass/CoM/friction nominal       lambda 1.0 envelope\n"
             "  actuation latency none          0-60 ms\n"
             "  push              none          +/-1.5 m/s planar, every 1-3 s\n\n"
             "  MuJoCo falls, 16 seeds, robot reset ONTO the reference:\n"
             "  training envelope   7/16          0/16\n"
             "  0-80 ms latency    10/16          0/16\n"
             "  Started from a stand instead, both go down in seconds -- see below.",
             kicker="WHAT YOU ARE DEPLOYING"))

    add(card(tmp, (n := n + 1), 9.5,
             "What is in the box",
             "  policies/   two fused ONNX heads, obs_dict[1,1570] -> action[1,29]\n"
             "  runner/     the SONIC C++ deployment runner, buildable as-is,\n"
             "              with unitree_sdk2 + CycloneDDS for x86_64 and aarch64\n"
             "  sdk/        unitree_sdk2py, vendored\n"
             "  sim/        a MuJoCo G1 that speaks the robot's own DDS protocol\n"
             "  config/     the observation config the runner needs\n"
             "  motions/    reference clips in the runner's CSV format\n"
             "  docs/       the sequence, the wiring, the measured results",
             kicker="LAYOUT"))

    add(card(tmp, (n := n + 1), 10,
             "Install",
             "  $ scp -r lucid-g1-deploy/ user@newbox:~/\n"
             "  $ ssh user@newbox && cd ~/lucid-g1-deploy\n\n"
             "  $ bash setup.sh        # a real terminal: sudo needs a tty\n"
             "  $ source env.sh        # every new shell\n"
             "  $ bash build.sh        # compiles the runner\n"
             "  $ bash test.sh         # seven checks\n\n"
             "setup.sh pins TensorRT 10.13 -- the version SONIC requires on\n"
             "x86_64. It is a safety pin, not a preference. See the last card.",
             kicker="FOUR COMMANDS"))

    add(card(tmp, (n := n + 1), 10,
             "Verify, before anything moves",
             "  == 1/7  bundle parses under the runner's own reading rules   PASS\n"
             "  == 2/7  observation config accepted by the runner's parser   PASS\n"
             "  == 3/7  ONNX policies load and are deterministic             PASS\n"
             "  == 4/7  golden parity traces match their policies            PASS\n"
             "  == 5/7  MuJoCo rollout with the reference controller         PASS\n"
             "  == 6/7  C++ runner loads the bundle                          PASS\n"
             "  == 7/7  DDS robot simulator comes up on the bus              PASS\n\n"
             "  pass 7   fail 0   skip 0",
             kicker="bash test.sh"))

    add(card(tmp, (n := n + 1), 11,
             "How it connects",
             "The runner does not know whether MuJoCo or a robot is on the other\n"
             "end. Same binary, same arguments, same keys -- only the far end of\n"
             "the DDS wire changes.\n\n"
             "  rt/lowcmd        -> 29 motor commands: q dq kp kd tau\n"
             "  rt/lowstate      <- 29 motor states, IMU quaternion, gyro, accel\n"
             "  rt/secondary_imu <- torso IMU\n"
             "  rt/dex3/*/cmd    -> hands\n\n"
             "  robot network:  192.168.123.x     rehearsal:  lo",
             kicker="DDS, THE SAME EITHER WAY"))

    add(card(tmp, (n := n + 1), 9,
             "Rehearse the deployment",
             "  $ python3 sim/run_robot_sim.py --iface lo     # the robot\n"
             "  $ bash run.sh --policy deploy_dr --iface lo   # the controller\n\n"
             "or both at once, scripted end to end:\n\n"
             "  $ bash drill.sh\n\n"
             "What follows is a real recording of that command.",
             kicker="NO ROBOT REQUIRED"))

    add(clip(tmp, (n := n + 1), args.footage, args.t_init - 1.2, 6.0,
             "1. INIT  -  the 3-second ramp",
             "The runner waits for the first LowState, then interpolates every joint "
             "to default_angles under full PD. Prints \"Init Done\"."))

    add(clip(tmp, (n := n + 1), args.footage, args.t_stand, 5.0,
             "2. WAIT_FOR_CONTROL  -  the fixed stand",
             "Holds default_angles, re-checks safety at 50 Hz. The policy is not "
             "running. Stay here as long as you like. Press ']' to go on."))

    add(clip(tmp, (n := n + 1), args.footage, args.t_policy, 6.5,
             "3. CONTROL  -  the policy takes over",
             "50 Hz: observations rebuilt from LowState, TensorRT inference, motor "
             "commands out. The support strap is released the instant it starts -- "
             "and on this clip the robot goes down within seconds. That is not a "
             "bug in the rehearsal. The next card explains it."))

    add(card(tmp, (n := n + 1), 12,
             "Why it goes down, and why the sweep disagrees",
             "INIT ramps to default_angles and the stand holds there. Pressing ']'\n"
             "hands the policy frame 0 of the reference -- and every clip shipped\n"
             "here starts far from that pose:\n\n"
             "  crouch_idle          0.409 rad RMS   R_hip_pitch  -0.816\n"
             "  walk_arc_cw          0.299 rad RMS   L_knee       -0.548\n"
             "  walk_ff_stop_270_R   0.341 rad RMS   R_knee       -0.546\n\n"
             "The policy must close that in ONE control step, standing, on its\n"
             "feet. It never sees that in training, where every episode is reset\n"
             "ONTO the reference. evaluate.sh resets that way too, and reports\n"
             "0/16 falls. drill.sh starts it the way a robot starts. Believe the\n"
             "drill.  $ bash test.sh   check 8 measures this for your own clips.",
             kicker="THE STEP AT ']'", accent=WARN))

    add(clip(tmp, (n := n + 1), args.footage, args.t_stop - 1.5, 6.5,
             "4. 'O'  -  emergency stop",
             "kp 0, kd 8, tau 0. Zero stiffness. Here it lands on a robot that is "
             "already down; on a standing one it puts it there. That is what this "
             "stop IS -- it removes the ability to hold a pose.", accent=WARN))

    add(card(tmp, (n := n + 1), 11,
             "There is no step 5",
             "operator_state.stop is set in four places and cleared in none.\n"
             "program_state_ only ever moves forward. A stop is TERMINAL.\n\n"
             "Recovery to a stable stand means restarting the runner, which\n"
             "re-enters INIT and ramps back to default_angles from wherever the\n"
             "joints ended up.\n\n"
             "Do that with the robot SUPPORTED. The ramp assumes the feet can\n"
             "take load, and after a stop they are usually not under the robot.",
             kicker="RECOVERY", accent=WARN))

    add(card(tmp, (n := n + 1), 10,
             "Does the engine compute the right thing?",
             "The MuJoCo robot gives the control loop a LowState, so the runner's\n"
             "TensorRT engine can be checked against the shipped ONNX on the\n"
             "runner's OWN observations.  $ bash drill.sh --parity\n\n"
             "  499 control ticks, FP32          TensorRT 10.13   TensorRT 10.16\n"
             "  mean |delta|                       1.45e-06        1.10e-04\n"
             "  median per-tick max                1.79e-06           --\n\n"
             "The version pin is visible in the numbers: at 10.16 the engine was\n"
             "systematically off on every tick. Use 10.13 on x86_64, 10.7 on Orin.",
             kicker="VALUE-LEVEL PARITY"))

    add(card(tmp, (n := n + 1), 10,
             "Reproduce the measurements",
             "  $ bash evaluate.sh          # ~40 min, no robot, 16 seeds\n"
             "  $ bash drill.sh             # the deployment, ~90 s\n\n"
             "Two sweeps: all randomization channels scaled together, and\n"
             "actuation latency alone. Draws come from one seeded stream in fixed\n"
             "order, so at a given (lambda, seed) both policies face IDENTICAL\n"
             "physics. The comparison is paired.\n\n"
             "Compare results/*/summary.md against docs/RESULTS.md. Small\n"
             "differences across machines are expected. The ORDERING should not\n"
             "change. If it does, the port is wrong, not the policies.",
             kicker="SIM2SIM"))

    add(card(tmp, (n := n + 1), 12,
             "Before you go near hardware",
             "  1  These policies CANNOT perceive their own horizontal position.\n"
             "     In the run you just watched the robot travelled 0.22 m while\n"
             "     the reference walked an arc. More DR does not fix it.\n"
             "  2  The policy jumps at ']'. 0.30-0.41 rad RMS on these clips.\n"
             "  3  The fixed stand does not hold an unsupported G1 in simulation.\n"
             "     Ankle-pitch stiffness is 28.5 N.m/rad. Support the robot.\n"
             "  4  A hardwired e-stop, a harness or gantry, and a clear floor are\n"
             "     not in this software. Provide them.\n"
             "  5  NO POLICY HAS EVER BEEN ARMED ON A ROBOT. Every number is simulation.",
             kicker="LIMITS", accent=WARN))

    add(card(tmp, (n := n + 1), 7.5,
             "Start here",
             "  README.md                  the four steps, and the limits\n"
             "  docs/DEPLOY_SEQUENCE.md    the state machine, keys, what stop does\n"
             "  docs/ETHERNET_AND_SDK.md   wiring, the SDK, DDS, troubleshooting\n"
             "  docs/RESULTS.md            every measured number\n\n"
             "  $ bash drill.sh --help",
             kicker="DOCS"))

    lst = tmp / "concat.txt"
    lst.write_text("".join(f"file '{p}'\n" for p in segs))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0",
         "-i", str(lst), "-c", "copy", str(args.out)])

    dur = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration",
         "-of", "csv=p=0", str(args.out)], capture_output=True, text=True).stdout.strip()
    size = args.out.stat().st_size / 1e6
    print(f"  {args.out}")
    print(f"  {len(segs)} segments, {float(dur):.1f}s, {size:.1f} MB, {W}x{H}@{FPS}")
    if not args.keep_temp:
        shutil.rmtree(tmp, ignore_errors=True)
    else:
        print(f"  temp kept: {tmp}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
