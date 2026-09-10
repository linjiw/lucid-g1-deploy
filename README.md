# lucid-g1-deploy

A self-contained bundle for running LUCID motion-tracking policies on a Unitree
G1 through SONIC's C++ deployment runner. Copy the whole directory to another
Ubuntu machine and follow the four steps below.

Nothing here has been on a robot. Read **Limits** before you get near one.

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
```

## Four steps on a new machine

```bash
scp -r lucid-g1-deploy/ user@newbox:~/          # ~272 MB
ssh user@newbox
cd ~/lucid-g1-deploy

bash setup.sh          # in a REAL TERMINAL: sudo needs a tty for the password
source env.sh          # every new shell
bash build.sh          # a few minutes the first time
bash test.sh           # eight checks: seven pass, one warns (the jump at ']')
```

Target: **Ubuntu 22.04, x86_64, NVIDIA GPU.** On a Jetson (arm64) skip
`setup.sh` and use `runner/scripts/install_deps.sh`, which handles the arm64
packages; everything else is the same.

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
```

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
bash run.sh --policy deploy_dr --iface lo --sim
```

Wait for `Init Done`, then `]` to arm, `T` to play, `O` to stop. `run.sh` prints
the whole key list at startup. `--sim` also defaults `--iface` to `lo`, adds
`--disable-crc-check`, and stops the simulator when the runner exits; pass
`--no-auto-sim` if you are already running `sim/run_robot_sim.py` yourself.

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

**Policy comparison**, no robot needed (~40 min):

```bash
bash evaluate.sh                 # both sweeps, 16 seeds
bash evaluate.sh --seeds 4       # ~5 min, noisier, ordering still shows
```

Compare `results/*/summary.md` against `docs/RESULTS.md`. Small differences
across machines are expected (MuJoCo and onnxruntime versions differ); the
**ordering** should not change. If it does, the port is wrong, not the policies.

## Deploy on a real G1

**Nothing in this bundle has been on a robot.** Every number in it is
simulation. Read `docs/DEPLOY_SEQUENCE.md` and `docs/DEPLOY_G1.md` section 8
before you start.

**1. Rehearse first, on the machine you will deploy from.** `bash test.sh` with
no failures, then `bash drill.sh --play --parity`. If parity does not pass, stop
— the engine on this machine does not reproduce the policy you validated.

**2. Wire it.** The G1 is on `192.168.123.0/24`, the robot at
`192.168.123.161`. Give your machine a static address on that subnet and confirm
DDS traffic before running anything that moves:

```bash
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

**5. Launch.**

```bash
bash run.sh --policy deploy_dr                  # auto-detects a 192.168.123.x NIC
bash run.sh --policy deploy_dr --iface enp3s0   # explicit
```

`run.sh` refuses to guess if it cannot find a `192.168.123.x` interface, never
adds `--disable-crc-check` without `--sim`, and asks you to confirm the safety
checklist from `/dev/tty` — a pipe cannot answer it.

**6. Drive it.** These are the runner's keys, the same ones `drill.sh` sends:

| key | what it does |
|---|---|
| `]` | **arm** — WAIT_FOR_CONTROL → CONTROL. The policy runs at 50 Hz, reference parked at frame 0. |
| `T` | **play** — run the clip from the current frame to its end |
| `N` / `P` | next / previous motion |
| `R` | reset the clip to frame 0, paused |
| `O` | **emergency stop** — kp 0, kd 8, tau 0 |

Lower case works for all of them except `]`.

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
| `runner/` | the C++ deployment runner source, buildable as-is, with `unitree_sdk2` and CycloneDDS 0.10.2 vendored for x86_64 and aarch64 |
| `sdk/` | `unitree_sdk2py`, vendored verbatim |
| `sim/` | a MuJoCo G1 that speaks the robot's own DDS protocol, so the runner can be rehearsed with no robot |
| `docs/` | the deployment guide, the sequence, the wiring, the measured results, and the demo video |

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
   Reproduce with `bash drill.sh --parity`; the reasoning and the ruled-out
   alternatives are in `tools/check_runtime_parity.py`.

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

3. **The fixed stand does not hold a free-standing G1 in simulation.** Holding
   `default_angles` with the runner's own gains, unsupported, the robot sits down
   in about 1.4 s — the ankle-pitch stiffness (28.5 N·m/rad, derived from rotor
   inertia rather than from the balance moment) loses the battle and it pitches
   onto its toes. The vendor's own simulator config ships with
   `ENABLE_ELASTIC_BAND: True` for this reason. Support the robot between
   `Init Done` and `]`. Measured in simulation only — see
   `docs/DEPLOY_SEQUENCE.md` for the trace and the caveat.
4. **An emergency stop puts the robot on the floor.** `O` writes kp 0, kd 8,
   tau 0 and joins the threads. It is terminal: `operator_state.stop` is never
   cleared and `program_state_` never moves backwards, so recovery to a stable
   stand means restarting the runner, with the robot supported.
5. **No hardware result of any kind exists.** Every number here is simulation.
6. **No calibrated recovery.** No band, dwell or horizon has ever been frozen in
   this project, so no policy has passed or failed a recovery test. "0/16 falls"
   is a measurement on one clip, not a safety property.
7. **The pushes are not physical.** Root velocity increments in m/s written into
   simulator state — no mass, no duration, no contact point. They name no
   impulse in newton-seconds.
8. **One training seed.** The measured between-seed effect on absolute
   capability in this project is 7.8 points, so the comparison above is
   descriptive, not a general claim about randomization.
9. **Safety hardware is not in this software.** The runner's stop path is a
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
`sim/VENDOR_PATCHES.md` for the four places vendored code was changed.

No upstream model weights are redistributed. Both policies were trained from
scratch in this project.
