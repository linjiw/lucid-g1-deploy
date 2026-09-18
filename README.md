# lucid-g1-deploy

A self-contained bundle for running LUCID motion-tracking policies on a Unitree
G1 — the 29-DoF one — through SONIC's C++ deployment runner. Copy it to another
Ubuntu machine and follow the four steps below.

This bundle has been on a G1 once — `INIT` and the fixed stand, in a gantry
harness, on 2026-09-11. The policy was never armed, and that run saved nothing.
Every behavioural number here is still simulation. See
[`docs/HARDWARE_RUNS.md`](docs/HARDWARE_RUNS.md), and read **Limits** before you
get near a robot.

## Latest four-motion policies (September 17 training)

**[Start here: install, verify, play, and rehearse both policies](docs/FOUR_MOTION_DEPLOY.md).**
This repository now includes the fixed-DR and no-DR ONNX models trained on
**walking, turning, crouch hold, and side-stepping** (seed 8600, iteration 4000),
plus all four reference clips and runner-format motions. A normal Git clone
includes the weights; there is no separate model download or Git LFS step.

```bash
git clone https://github.com/linjiw/lucid-g1-deploy.git
cd lucid-g1-deploy
bash setup.sh
source env.sh
bash build.sh
bash four_motion.sh check
"$PYTHON" -m wandb login     # account with access to 16726/lucid-sonic
bash four_motion.sh play --policy fixed_dr --motion walking
bash four_motion.sh rehearse --policy no_dr --motion crouch_hold
```

`play` records reference-initialized MuJoCo playback. `rehearse` tests the actual
C++/TensorRT/DDS deployment sequence. Their measured outcomes differ; read the
[validation results and limitations](docs/FOUR_MOTION_DEPLOY.md#validation-and-limitations).
All four exported references use the corrected IsaacLab joint order, checked
against every source joint position and velocity before launching.

The rest of this README's historical measurements and `deploy_dr` / `no_dr`
examples describe the older three-motion, 8,000-iteration pair. The latest
models have explicit `four_motion_...` filenames and the dedicated launcher above.

## Original three-motion bundle and deployment background

**[Watch the 2½-minute guided demo →](docs/demo/lucid-g1-deploy-demo.mp4)** —
what it is, how to install it, how the DDS interface works, and a real recording
of the deployment sequence: init → fixed stand → policy → emergency stop.

```bash
git clone https://github.com/linjiw/lucid-g1-deploy.git
cd lucid-g1-deploy
bash setup.sh && source env.sh && bash build.sh && bash test.sh
bash drill.sh --play       # rehearse the whole deployment, no robot needed
bash run.sh --policy deploy_dr --sim   # or drive it yourself, one command
```

`drill.sh` defaults to `walk_arc_cw_stop_001__A047`, the clip every measured
number here is about. `--motion <name>` picks another; `bash run.sh --list`
shows what is available.

```
setup.sh    install the toolchain on a fresh machine (asks for sudo once)
env.sh      source in every shell
build.sh    compile the runner
test.sh     verify the bundle end to end -- eight checks
evaluate.sh reproduce the DR-perturbation and latency measurements in MuJoCo
drill.sh    rehearse the deployment sequence against a MuJoCo G1 over DDS
run.sh      launch a policy
standby.sh  the operator loop: pick a clip, run it, reset to the stand, repeat
```

## Four steps on a new machine

```bash
rsync -a --exclude .git --exclude .venv --exclude .venv-sim --exclude .build \
      --exclude runner/target --exclude runner/build --exclude results \
      --exclude 'policies/*.trt' --exclude policies/.trt_built_with \
      lucid-g1-deploy/ user@newbox:~/lucid-g1-deploy/    # ~286 MiB, 1,419 files
ssh user@newbox
cd ~/lucid-g1-deploy

bash setup.sh          # in a REAL TERMINAL: sudo needs a tty for the password
source env.sh          # every new shell
bash build.sh          # a few minutes the first time
bash test.sh           # eight checks: seven pass, one warns (the jump at ']')
```

`build.sh` builds the deployment binary and nothing else. The runner's gtest
unit tests are opt-in — `cmake -S runner -B .build -DG1_BUILD_TESTS=ON`, which
needs `libgtest-dev` — and the configure step reaches the network nowhere: no
live `FetchContent`, `ExternalProject` or `file(DOWNLOAD` call is left in any
CMake file under `runner/` outside `thirdparty/`, grepped here. The mentions
that remain are all inside one comment in
`runner/src/g1/g1_deploy_onnx_ref/CMakeLists.txt` — the block opening
`# test executable: OFF by default, and not because the tests are unwanted.` —
which records the googletest `FetchContent` that used to run at configure time
and was removed.

Copy the source tree, not the working directory. `scp -r lucid-g1-deploy/` is
1.5 GB here — it carries everything `.gitignore:1-13` refuses to commit (`.venv`
434M, `.venv-sim` 349M, `results/` 96M, `policies/*.trt` 112M, `.build` 16M),
plus 152M of `.git` history the new machine can re-clone — and the bytes are not
the worst of it. `.build/CMakeCache.txt:607,686` records the absolute paths it
was configured in, and `build.sh:6` reuses `$HERE/.build`, so under a different
home cmake has a cache it will not accept. And `scp -r` dereferences symlinks,
so `.venv/bin/python` — a symlink to `/usr/bin/python3` here — arrives as a real
copy of this box's x86_64 interpreter, which an Orin cannot execute at all. That
one repairs itself: `setup.sh` gates the venv rebuild on the imports working
rather than on `bin/python` existing — its `venv_ok` runs
`import numpy, onnxruntime, yaml, joblib, scipy, mujoco` through that
interpreter — so a copied `.venv` fails the import and is deleted and rebuilt.
The cost is 434 MB of transfer wasted, not a machine you cannot repair. The
`rsync` above, measured here with `rsync -an --stats`, is about 286 MiB across
1,419 files. Where the new machine has network, the `git clone` at the top of
this README is simpler still.

Target: **Ubuntu 22.04, x86_64, NVIDIA GPU**, and a **29-DoF G1**:
`G1_NUM_MOTOR = 29` (`robot_parameters.hpp:32`), policies are
`obs_dict [1,1570]` → `action [1,29]`, and the runner initialises
Dex3 hands unconditionally (`g1_deploy_onnx_ref.cpp:2188`) and publishes hand
commands every writer cycle (`:2682`). The hand *state* read is null-guarded
(`:2909`), so a G1 without hands would log zeros there — but nothing in this
bundle has ever been run on a 23-DoF or hand-less G1.

The **arm64/Orin path is not `setup.sh`-equivalent** — see
`docs/ETHERNET_AND_SDK.md`, "Deploying on the robot's own Orin".
`runner/scripts/install_deps.sh` installs the arm64 apt packages, `just` and a
**CPU** onnxruntime 1.16.3 into `/opt/onnxruntime` (`:380`, `:383`, `:390`); it
installs no TensorRT — its own closing advice (`:856`, `:884`) tells you to make
sure TensorRT is there yourself — and it creates neither venv. So on an Orin you
must additionally pin TensorRT 10.7 under JetPack 6 and create
`.venv`/`.venv-sim` yourself, or `test.sh` falls back to a system `python3` with
none of the packages (`PY="${PYTHON:-python3}"`) and check 7, the DDS robot
simulator, skips for want of `.venv-sim`. `onnxruntime_ROOT` resolves without
help: the x64 GPU glob misses on arm64, so `env.sh` tries a widened
`onnxruntime-linux-*` glob under the toolchain directory and then
`/opt/onnxruntime`, taking either only if it carries `lib/cmake/onnxruntime`
(`[ -d /opt/onnxruntime/lib/cmake/onnxruntime ]`). `/opt/onnxruntime` is where
`install_deps.sh` puts it: `ONNX_INSTALL_PATH` defaults to it (`:380`) and the
unpacked tree is moved there at `:416`. Export `onnxruntime_ROOT` by hand only
if that directory is absent — `build.sh:5` aborts with
`run 'source env.sh' first` when nothing set it. `env.sh`'s TensorRT probe
follows `$(uname -m)`, so on an Orin it reads
`/usr/include/aarch64-linux-gnu/NvInferVersion.h` — JetPack's multiarch path —
and when nothing is there it names `docs/ETHERNET_AND_SDK.md` rather than
`setup.sh`, because `setup.sh` is the one thing that will not run on aarch64 (it
exits on `[ "$(uname -m)" = "x86_64" ] ||`). All of this is read from those
files, not executed on an Orin.

If `setup.sh` stops on `apt-get update`, a broken third-party repo elsewhere on
the machine is failing and `set -e` is doing its job — fix or disable that repo
and re-run. And if the box already has a **CUDA 13** toolkit, install
`cuda-nvcc-12-9` as well: without an `nvcc` under `/usr/local/cuda-12.9`,
`find_package(CUDAToolkit)` falls through to a path in `runner/CMakeLists.txt`
that hardcodes `/usr/local/cuda`, and you get a binary built against CUDA 13
under a TensorRT built for 12.9. Confirm afterwards:

```bash
ldd runner/target/release/g1_deploy_onnx_ref | grep cudart   # must say .so.12
```

## Deploy in MuJoCo (sim2sim)

`sim/` puts a MuJoCo G1 on the DDS bus speaking the robot's own protocol, so the
**unmodified runner binary** drives it with the same arguments and the same
operator keys you would use on hardware. What is substituted is the physics, not
the software. Two ways in.

**Scripted — `drill.sh`.** Walks the whole sequence and reports what the robot
did:

```bash
bash drill.sh                     # init -> fixed stand -> arm -> stop  (~45 s)
bash drill.sh --play              # ...and press 'T', so the clip actually plays
bash drill.sh --play --parity     # ...and check the TensorRT engine vs the ONNX
bash drill.sh --viewer            # show the MuJoCo window (needs a desktop session)
bash drill.sh --motion <name>     # another clip; `bash run.sh --list` shows them
bash drill.sh --hold 12           # longer in CONTROL before the stop
bash drill.sh --latency 60        # 60 ms of actuation delay at the robot
```

`--latency` delays every LowCmd inside the simulated robot before it reaches the
joints, and in the viewer `=` / `-` move it by 5 ms and `0` clears it (`drill.sh`
passes it through as `sim/run_robot_sim.py --latency-ms`). It perturbs the plant
the runner is driving over DDS — not the same harness as `evaluate.sh`'s latency
ladder, which runs its own MuJoCo rollouts through `tools/mujoco_sweep.py` with
no runner and no DDS, so the two sets of numbers are not comparable.

A healthy `--play` run reaches every marker and ends with the robot on the
floor, which is what an emergency stop *is*:

```
reached   Dimension match: Configuration is valid!
reached   Init Done
reached   transitioning to CONTROL state
reached   Playing motion 0 from frame 0 to end (432 total frames)
reached   Motion index: 0 : walk_arc_cw_stop_001__A047 completed.
reached   Stopping G1Deploy...

phase                  window         pelvis z   moved    kp[0]  kd[0]
FIXED STAND     24.6- 30.1s   0.759 ->  0.771    0.06     99.1    6.3
POLICY          30.6- 43.2s   0.766 ->  0.282    0.33     99.1    6.3
AFTER STOP      43.7- 49.7s   0.064 ->  0.069    0.04      0.0    8.0
```

**Interactive — `run.sh --sim`.** Starts the same MuJoCo robot and hands you the
keyboard, which is the closer rehearsal of a real run:

```bash
bash run.sh --policy deploy_dr --iface lo --sim --viewer
```

Wait for `Init Done`, then `]` to arm, `T` to play, `O` to stop. `run.sh` prints
the whole key list at startup. `--sim` also defaults `--iface` to `lo`, adds
`--disable-crc-check`, and stops the simulator when the runner exits; pass
`--no-auto-sim` if you are already running the robot simulator yourself — start
it as `"$LUCID_SIM_PYTHON" sim/run_robot_sim.py`, not under `python3`: only
`.venv-sim` carries the cyclonedds 0.10.2 bindings, and `env.sh` exports both
that interpreter and the `PYTHONPATH` the vendored SDK needs (the script's USAGE
block says so).
`--viewer` shows the MuJoCo window (needs a desktop session); without it the
robot is there on the bus but you cannot see it.

The simulated robot gets the same support `drill.sh` uses: the elastic band
through init and the fixed stand, released the instant the policy takes over,
and a fall left where it lands. Both matter. Unsupported, the robot collapses
during the INIT ramp — 0.791 → 0.131 m, measured — so `]` would arm the policy
on a robot already on the floor; and the vendor's `check_fall()` otherwise
resets the simulation below 0.2 m, snapping the robot upright at exactly the
moment the emergency stop is meant to show it going down. `--no-band` turns the
band off if you want to see that.

**The operator loop — `standby.sh`.** One clip at a time, keyboard, with a reset
to the init pose between runs:

```bash
bash standby.sh --sim --viewer            # rehearse the loop, no robot
bash standby.sh                           # on the robot network, NIC auto-detected
```

It lists the clips with the size of the step each one asks for at `]`, runs the
one you pick through `run.sh`, and then waits for a person before going again.
Each cycle is an ordinary `run.sh`, so nothing in the SONIC runner is modified
or bypassed — the restart is what re-enters INIT and ramps to `default_angles`.

**It will not stand a fallen robot up.** Measured across two cycles on one
persistent MuJoCo robot: cycle 1 stopped with the pelvis at 0.061 m, cycle 2's
INIT ramp commanded `default_angles` at full PD, printed `Init Done` on its 3 s
timer with the pelvis at 0.133 m, then armed and played a whole clip with the
robot flat on the ground — every marker reported normally. `Init Done` is a
timer, not a measurement; `InitControl()` never checks that the robot is
upright. Put it back on its feet yourself. `docs/DEPLOY_SEQUENCE.md` §5 has the
trace.

**Value-level parity** — the TensorRT engine against the shipped ONNX, on the
runner's own observations. This is the check that a version-mismatched TensorRT
would fail:

```bash
bash drill.sh --play --parity
```

```
compared 499 ticks
max |delta|   5.722e-06      mean |delta|  4.204e-07
PASS -- the TensorRT engine reproduces the ONNX policy to 5.7e-06
```

That is one run, and **Limits** #2 quotes a different one (mean 1.45e-06, max
2.37e-03). Both pass: the mean turns on whether the run happens to contain the
single logging-artefact tick, and the run above does not — its max is 5.722e-06,
not 2.37e-03. The evidence is in `tools/check_runtime_parity.py`, under "runs
that happen to contain no such tick come out at max 5.1e-06 and 6.2e-06";
`docs/DEPLOY_DAY.md` says it again under "Two different means are both correct".
Record your own first reading rather than dividing one against the other.

**Policy comparison**, no robot needed (~40 min):

```bash
bash evaluate.sh                 # both sweeps, 16 seeds
bash evaluate.sh --seeds 4       # ~5 min, noisier, ordering still shows
```

Compare `results/*/summary.md` against `docs/RESULTS.md`. Small differences
across machines are expected (MuJoCo and onnxruntime versions differ); the
**ordering** should not change. If it does, the port is wrong, not the policies.

## Deploy on a real G1

**One hardware run has happened: `INIT` and the fixed stand, under a gantry, on
2026-09-11. No policy has ever been armed on a robot.** Every behavioural number
in this bundle is simulation. `docs/HARDWARE_RUNS.md` is the log of what has
actually been on a G1 and what it established.

Read, in this order: `docs/DEPLOY_SEQUENCE.md` for what the runner does between
power-on and the policy, `docs/DEPLOY_G1.md` section 8 for the bundle-level
preconditions, and `docs/DEPLOY_DAY.md` for the procedure to actually follow on
the day — it is the newest and most complete of the three, and every step in it
is marked with what it is worth. `docs/deploy-day-checklist.html` is the same
procedure as a tick-through page for the lab.

**1. Rehearse first, on the machine you will deploy from.** `bash test.sh` with
no failures, then `bash drill.sh --play --parity`. If parity does not pass, stop
— the engine on this machine does not reproduce the policy you validated.

**2. Wire it.** The G1 is on `192.168.123.0/24`, the robot at
`192.168.123.161`. Give your machine a static address on that subnet and confirm
DDS traffic before running anything that moves:

```bash
ip link                                                 # enp3s0 is an example
sudo ip addr add 192.168.123.222/24 dev enp3s0
sudo ip link set enp3s0 up
ping -c3 192.168.123.161
sudo tcpdump -i enp3s0 -c 20 udp portrange 7400-7500    # silence here = no DDS
```

Full detail, including releasing Unitree's own controller: `docs/ETHERNET_AND_SDK.md`.

**3. Provide the safety this software does not have.** A hardwired emergency
stop, a fall-arrest harness or gantry, and a fallback controller. The runner's
stop path is a software boolean plus a 35 rad/s joint-velocity abort and a
motor-temperature cutoff; none of those is an emergency stop. For a humanoid,
cutting power is itself a hazard, because the robot falls.

**4. Support the robot from `Init Done` onward.** Two measured reasons, both in
**Limits** below: the fixed stand does not hold an unsupported G1 in simulation
(it sits down in about 1.4 s), and all three shipped clips start 0.30–0.41 rad
RMS away from the pose the runner holds, so the policy is asked to close that
gap in one control step the moment you press `]`.

**5. Launch.** Pass `--motion` on every hardware run. The runner starts on
motion index 0, and which clip that is comes from `readdir` order, not
alphabetical order — `motion_data_reader.hpp:685` walks the directory with an
unsorted `std::filesystem::directory_iterator`, and `run.sh` says so in the
comment above its `MOTIONS_DIR` — so without it you do not know what `T` will
play; `run.sh` prints a three-line warning when it is missing.

```bash
CLIP=walk_arc_cw_stop_001__A047                 # bash run.sh --list shows them
bash run.sh --policy deploy_dr --motion "$CLIP"            # NIC auto-detected
bash run.sh --policy deploy_dr --motion "$CLIP" --iface enp3s0   # explicit NIC
```

`run.sh` refuses to guess if it cannot find a `192.168.123.x` interface, and
asks you to confirm the safety checklist from `/dev/tty` — a pipe cannot answer
it. It never adds `--disable-crc-check` without `--sim`, and a hand-passed one
is now refused rather than obeyed: that flag also gates the 35 rad/s
joint-velocity abort (`g1_deploy_onnx_ref.cpp:2832`,
`if (body_dq[i] > 35 && !disable_crc_check_)`), so it removes two guards and
names one.

Every run records itself, under `results/run/<timestamp>-<policy>[-<motion>]/`:
`console.log`, `run_info.txt` (date, policy, motion, iface, commit, the runner
binary that actually ran, the full argv, GPU, host and the exit code) and, on
hardware, `csv/` — the runner's per-tick CSV logs, on by default for a real
robot and off for `--sim` (`--csv-logs` / `--no-csv-logs` override that;
`--log-dir <dir>` puts the run directories on another disk). `run.sh` prints the
path when the run starts and again, as `log saved:`, when it ends. `results/` is
gitignored (`.gitignore:13`), so that directory is the only copy of a hardware
run until someone moves it off the machine.

**6. Drive it.** These are the runner's keys, the same ones `drill.sh` sends:

| key | what it does |
|---|---|
| `]` | **arm** — WAIT_FOR_CONTROL → CONTROL. The policy runs at 50 Hz, reference parked at frame 0. |
| `T` | **play** — run the clip from the current frame to its end |
| `N` / `P` | next / previous motion |
| `R` | reset the clip to frame 0, paused |
| `O` | **emergency stop** — kp 0, kd 8, tau 0 |

Lower case works for all of them except `]`. With `--motion` the runner sees a
directory holding that one clip (`run.sh` symlinks it into a fresh temporary
`MOTIONS_DIR`), so `N` / `P` have nothing to cycle to — they are for bench runs
with more than one clip loaded.

**7. After a stop, there is no step 8.** `O` is terminal:
`operator_state.stop` is never cleared and `program_state_` never moves
backwards, so the process does not return to a stand. Recovery is restarting
`run.sh`, which re-enters INIT and ramps to `default_angles` from wherever the
joints ended up — do that with the robot supported, because the ramp assumes the
feet can take load and after a stop they usually cannot.


## What is in here

| | |
|---|---|
| `policies/` | two ONNX policies, fused single-file form: `obs_dict [1,1570]` → `action [1,29]` |
| `config/` | the observation config the runner needs, and joint order / gains / action scale metadata |
| `motions/` | three reference clips already converted to the runner's CSV format |
| `parity/` | golden (observation, action) traces for a value-level parity test |
| `clips/` | the three source clips, so the MuJoCo evaluation runs with no extra setup |
| `tools/` | validator, bundle verifier, clip converter, parity harness, MuJoCo player and sweep |
| `runner/` | the C++ deployment runner source, buildable as-is, with `unitree_sdk2` and CycloneDDS 0.10.2 vendored for x86_64 and aarch64; it also carries upstream SONIC's own `deploy.sh`, kept only so `runner/` stays diffable — it is **not** this bundle's entry point and must not be run |
| `sdk/` | `unitree_sdk2py`, vendored verbatim |
| `sim/` | a MuJoCo G1 that speaks the robot's own DDS protocol, so the runner can be rehearsed with no robot |
| `docs/` | the deploy-day procedure (`DEPLOY_DAY.md`, and `deploy-day-checklist.html` as its tick-through page), the deployment guide, the sequence, the wiring, the measured results, the hardware-run log and the demo video |
| `MANIFEST.json` | the bundle's own receipt: file hashes, the policy/config table, and the start-pose step per clip that `standby.sh` prints beside every clip (its `clip_note()` reads `start_pose_step` out of this file) |
| `NOTICE` | what came from where — the vendored SONIC, Unitree and CycloneDDS code and their licences |
| `results/` | sweep, drill and per-run output. Written by `evaluate.sh`, `drill.sh` and every `run.sh` (`results/run/<timestamp>-<policy>[-<motion>]/`), never committed (`.gitignore:13`); `docs/RESULTS.md` is the version of record, and a hardware run's directory has to be copied off by hand |

## The two policies

Trained from scratch on the same three motions, same budget, same seed. The
**only** difference between them is the randomization they trained under.

| | `no_dr` | `deploy_dr` |
|---|---|---|
| mass / CoM / joint / friction | nominal | λ 1.0 envelope |
| actuation latency | none | **0–60 ms** |
| push | none | **±1.5 m/s** planar, every 1–3 s |

Measured in MuJoCo, 16 seeds, full 8.6 s motion — **falls**:

| perturbation | `no_dr` | `deploy_dr` |
|---|---|---|
| none | 0/16 | 0/16 |
| training envelope | **7/16** | **0/16** |
| heavy | **11/16** | **3/16** |
| 0–80 ms latency | **10/16** | **0/16** |

And in Isaac at 0–60 ms latency, `no_dr` scores **0.000** success on all three
training panels while `deploy_dr` scores 0.90 / 0.98 / 0.21.

**Use `deploy_dr`** for anything involving contact, disturbance or real
actuation delay. `no_dr` exists as the control — it tracks the reference path
about four times longer when nothing is perturbing it, and falls over as soon as
something is.

Neither policy tracked a full clip in any condition. See **Limits**.

## Deploying a different policy

Export the fused g1 head from a training checkpoint, then:

```bash
# 1. export  (in the training repo, needs the GPU)
bash scripts/practice_utility/export_arm_onnx.sh <arm-artifact-dir>

# 2. drop it in
cp <arm>/exported/model_step_XXXXXX_g1.onnx policies/myPolicy_s8600_g1.onnx

# 3. convert any new reference clips
python tools/convert_clip_for_deploy.py --clip <clip.pkl> --out motions/

# 4. re-verify, then run
bash test.sh
bash run.sh --policy myPolicy --iface lo --sim
```

If the new policy has a different observation width, `config/` needs a matching
config; validate it with `tools/validate_deploy_obs_config.py` before building.
`test.sh` will catch a mismatch, but only if you run it.

## Limits — read before hardware

1. **The policy cannot see its own horizontal position.** The observation has
   joint angles and velocities, base angular velocity, projected gravity, past
   actions and the reference motion — and no term at all for where the robot is
   relative to the path. Across 1,536 base states and 6,144 horizontal
   translations of ±0.25 m, every actor-side observation and every action mean
   changed by exactly 0.0. Both policies drift off the reference within seconds,
   and this is why. Treat any deployment as **open-loop in the horizontal
   plane**. More randomization does not fix it.
2. **Value-level parity has been run, and it passes.** This was the bundle's one
   unclosed verification; `sim/` closed it by giving the runner the `LowState`
   its control loop needs, so its TensorRT engine can be compared against the
   shipped ONNX on the runner's own observations. At the pinned TensorRT 10.13,
   FP32, 499 control ticks: **mean |delta| 1.45e-06, median per-tick max
   1.79e-06, 99th percentile 4.29e-06** — ordinary FP32 agreement. One tick in
   499 reads 2.37e-03; all 29 joints move together, the neighbours are clean and
   the offset is one control tick of joint motion, which is the runner's
   observation dump racing its own inference, not an inference difference.

   The version pin is visible here. Under TensorRT **10.16** the same test gave
   mean **1.10e-04** — systematically off on every tick, 76× worse. SONIC's
   `danger` note about using anything other than 10.13 is not hypothetical.
   Reproduce with `bash drill.sh --play --parity`: without `--play`, `drill.sh`
   never sends `T` — the one line that sends it is
   `[ "$PLAY" -eq 1 ] && { sleep 1; printf 'T'; }` — and the comparison is
   against a reference parked at frame 0, a much narrower input distribution
   than the ticks above.
   The reasoning and the ruled-out alternatives are in
   `tools/check_runtime_parity.py`.

   **That 1e-06 is a property of this network, not a pass mark for any policy.**
   Both figures above are `deploy_dr`. Running the same test on `no_dr` — same
   pinned TensorRT 10.13.3, same GPU, same runner, 3 repeats — gives **mean
   6.88e-05, median 5.71e-05, max 2.78e-04**, and it is systematic rather than
   an outlier: **every one of 648 ticks** exceeds 1e-05, where `deploy_dr` has
   none. That is the same order as the 1.10e-04 above, which is the evidence for
   the version pin — so the magnitude alone does not distinguish "wrong TensorRT"
   from "different network". What separates them is the shape: a version mismatch
   moves a policy off its own baseline, and 2.78e-04 is `no_dr`'s correct
   baseline. Physically it is 0.15 mrad (0.009°) of joint target and changes
   nothing. Take a policy's own baseline first and compare against that; do not
   port `deploy_dr`'s number to a policy you just exported.

3. **The policy jumps at the moment you press `]`.** INIT ramps to
   `default_angles` and the stand holds there; pressing `]` hands the policy
   frame 0 of the reference. All three shipped clips start far from that pose —
   **0.30–0.41 rad RMS**, with both knees 0.55 rad off on the walk clip and hip
   pitch 0.82 rad off on the crouch. The policy is asked to close that in one
   control step, from a standing start, on its feet — a step it never sees in
   training, where every episode is reset *onto* the reference with its
   velocities. This is why the same clip scores 0/16 falls in `evaluate.sh` and
   still goes down within seconds in `drill.sh`. `bash test.sh` check 8 measures
   it; `tools/check_motion_start.py` explains the options.

4. **The fixed stand does not hold a free-standing G1 in simulation.** Holding
   `default_angles` with the runner's own gains, unsupported, the robot sits down
   in about 1.4 s — the ankle-pitch stiffness (28.5 N·m/rad, derived from rotor
   inertia rather than from the balance moment) loses the battle and it pitches
   onto its toes. The vendor's own simulator config ships with
   `ENABLE_ELASTIC_BAND: True` for this reason. Support the robot between
   `Init Done` and `]`. Measured in simulation only — see
   `docs/DEPLOY_SEQUENCE.md` for the trace and the caveat.
5. **An emergency stop puts the robot on the floor.** `O` writes kp 0, kd 8,
   tau 0 and joins the threads. It is terminal: `operator_state.stop` is never
   cleared and `program_state_` never moves backwards, so recovery to a stable
   stand means restarting the runner, with the robot supported.
6. **No policy has ever been armed on a robot.** This bundle has been on a G1
   once -- `INIT` and the fixed stand, in a gantry harness, on 2026-09-11 --
   and `]` was never pressed. Every behavioural number here is simulation.
   `docs/HARDWARE_RUNS.md` is the log of what has actually been on a robot.
7. **No calibrated recovery.** No band, dwell or horizon has ever been frozen in
   this project, so no policy has passed or failed a recovery test. "0/16 falls"
   is a measurement on one clip, not a safety property.
8. **The pushes are not physical.** Root velocity increments in m/s written into
   simulator state — no mass, no duration, no contact point. They name no
   impulse in newton-seconds.
9. **One training seed.** The measured between-seed effect on absolute
   capability in this project is 7.8 points, so the comparison above is
   descriptive, not a general claim about randomization.
10. **Safety hardware is not in this software.** The runner's stop path is a
   software boolean plus a 35 rad/s joint-velocity abort and a motor-temperature
   cutoff. Provide a hardwired emergency stop, a fall-arrest harness or gantry,
   and a fallback controller independently. For a humanoid, cutting power is
   itself a hazard, because the robot falls.

## Three traps that do not fail loudly

Each of these was found by running something, after static checks had passed.

* **No inline `#` comments on a `- name:` line** in the observation config. The
  runner does not use a YAML parser; it trims whitespace and quotes and nothing
  else, so a trailing comment becomes part of the term name and startup aborts.
  A real YAML parser hides this completely.
* **Term ORDER in that config is load-bearing** and is checked by nothing at
  runtime. Offsets are assigned top to bottom, so a reordered config loads
  cleanly and feeds the policy shuffled input. Gravity goes **last**.
* **Motion quaternions are WXYZ**, the opposite of the XYZW used on the training
  side and by the MuJoCo player. `tools/convert_clip_for_deploy.py` gets this
  right; hand-edited CSVs are where it goes wrong.

`bash test.sh` catches all three.

## Licence

Apache 2.0 (`LICENSE`). This bundle redistributes NVIDIA's GR00T-WholeBodyControl
(Apache 2.0), Unitree's `unitree_sdk2` and `unitree_sdk2_python` (BSD 3-Clause)
and Eclipse Cyclone DDS — see `NOTICE` for what came from where, and
`sim/VENDOR_PATCHES.md` for the six places vendored code was changed.

No upstream model weights are redistributed. Both policies were trained from
scratch in this project.
