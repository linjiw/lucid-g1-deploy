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
bash drill.sh              # rehearse the whole deployment, no robot needed
bash run.sh --policy deploy_dr --sim   # or drive it yourself, one command
```

`drill.sh` defaults to `walk_arc_cw_stop_001__A047`, the clip every measured
number here is about. `--motion <name>` picks another; `bash run.sh --list`
shows what is available.

```
setup.sh    install the toolchain on a fresh machine (asks for sudo once)
env.sh      source in every shell
build.sh    compile the runner
test.sh     verify the bundle end to end -- seven checks
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
bash test.sh           # seven checks, all should pass
```

Then rehearse the deployment without a robot. `drill.sh` puts a MuJoCo G1 on the
DDS bus and drives it with the **unmodified runner binary**, the same arguments
and the same operator keystrokes you would use on hardware:

```bash
bash drill.sh                    # init -> fixed stand -> policy -> emergency stop
bash drill.sh --viewer           # watch it
bash drill.sh --parity           # also check the TensorRT engine against the ONNX
```

Read **`docs/DEPLOY_SEQUENCE.md`** before you run it. It has the state machine,
the operator keys, what an emergency stop actually does, and why the robot has
to be supported during bring-up.

Reproduce the measurements on the new machine — no robot needed, ~40 min:

```bash
bash evaluate.sh                 # both sweeps, 16 seeds
bash evaluate.sh --seeds 4       # ~5 min, noisier, ordering still shows
```

Compare `results/*/summary.md` against `docs/RESULTS.md`. Small differences
across machines are expected (MuJoCo and onnxruntime versions differ); the
**ordering** should not change. If it does, the port is wrong, not the policies.

Then bench it on loopback before anything else:

```bash
bash run.sh --policy deploy_dr --iface lo --sim
```

and on the robot network:

```bash
bash run.sh --policy deploy_dr            # auto-detects 192.168.123.x
```

Target: **Ubuntu 22.04, x86_64, NVIDIA GPU.** On a Jetson (arm64) skip
`setup.sh` and use `runner/scripts/install_deps.sh`, which handles the arm64
packages; everything else is the same.

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
