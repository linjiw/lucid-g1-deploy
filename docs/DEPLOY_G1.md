# Deploying a LUCID policy on a real Unitree G1

What this is: the SONIC deployment path as it actually exists in this
repository, checked against the runner's own source and then against the runner
itself. As of 2026-09-09 the C++ runner has been **built and run** here for the
first time, on the x86 workstation, and it loads this bundle: three motions, a
TensorRT engine compiled from the policy, and its own printed observation layout
matching the expected 1570 terms. See section 9 for exactly which claims that
does and does not settle.

What it is not: a result about policy behaviour on a robot. This bundle has been
on a G1 once — `INIT` and the fixed stand, in a gantry, on 2026-09-11, with the
policy never armed — and that run saved no data. `docs/HARDWARE_RUNS.md` is the
log.

Bundle: `$LUCID_ROOT/analysis/deploy_dr_ab_20260909/deploy_bundle`.

---

## 1. Which artifact goes on the robot

The export produces five ONNX heads. They are not interchangeable:

| head | input | output | what it is |
|---|---:|---|---|
| `..._g1.onnx` | 1570 | action [29] | **fused** — reference terms and proprioception in one model |
| `..._encoder.onnx` | 1751 | tokens [64] | reference terms → latent motion token |
| `..._decoder.onnx` | 994 | action [29] | 64 tokens + 930 proprioception → action |
| `..._smpl.onnx` | 1770 | action [29] | SMPL-reference variant |
| `..._teleop.onnx` | 1197 | action [29] | VR 3-point teleop variant |

SONIC's own `deploy.sh` — vendored here as `runner/deploy.sh`, and **not** this
bundle's entry point — defaults to the **split** path (`--cp <prefix>` resolves
`<prefix>_encoder.onnx` and `<prefix>_decoder.onnx`) because that is how the
released SONIC controller ships. For a LUCID policy the **fused** path is
simpler and is what this bundle is built for:

* one TensorRT engine instead of two, so one less inference in the 50 Hz loop;
* no encoder-mode selection logic, no `token_state`, no encoder observation set;
* the runner supports it — `g1_deploy_onnx_ref.cpp:2344` loads an encoder only
  when the observation config declares `encoder.dimension > 0`, and the parser
  explicitly ignores an encoder section when `token_state` is disabled
  (`observation_config.hpp:327-333`, the `// token_state disabled: encoder
  section is IGNORED` branch). The runner's own usage line takes a single
  `model.onnx`.

`--planner-file` is **optional** (`:4105`, `:2396`). The planner produces
locomotion targets for the 10 Hz replanning thread; a motion-tracking policy
replaying a fixed clip does not need one.

**`runner/deploy.sh` is present, executable and must not be run.** It is
upstream's launcher, kept so `runner/` stays diffable against upstream; nothing
in this bundle invokes it. It defaults to `INTERFACE_MODE="real"` — the robot,
not the simulator — points at `policy/release/model`, a planner and a reference
directory that are not in this bundle, prints "Some files are missing" and
continues anyway, then `apt`-installs its own dependencies and rebuilds through
`just` into `runner/build/`, which is not the tree `build.sh` produces. Its
"Proceed with deployment?" prompt reads stdin and treats an empty line as yes.
The file's own banner comment enumerates all of that with line numbers; read it
there rather than trusting this paragraph. The entry point is `run.sh`, with
`drill.sh` to rehearse the same sequence against the simulator first.

## 2. The observation config

Use `config/observation_config_lucid_g1_1570.yaml`. Validate it against the
runner you are about to build, every time:

```bash
python tools/validate_deploy_obs_config.py \
  config/observation_config_lucid_g1_1570.yaml \
  --expect-dim 1570 --expect-layout fused_g1_1570
```

**Do not use `policy/release/observation_config_sonic_release.yaml`**, despite
its header naming the exact export command LUCID uses. It lists eight encoder
term names that are absent from the runner's 76-entry registry
(`encoder_index`, `command_multi_future_nonflat`,
`motion_anchor_ori_b_mf_nonflat`, `command_multi_future_lower_body`,
`motion_anchor_ori_b`, `smpl_joints_multi_future_local_nonflat`,
`smpl_root_ori_b_multi_future`, `joint_pos_multi_future_wrist_for_smpl`), so
`InitializeObservationFunctions` throws on load. The validator reports all eight.

Term ORDER is load-bearing and is not checked by anything at runtime: offsets
are assigned by walking the file top to bottom, so a reordered config loads
cleanly and feeds the policy shuffled input. Note in particular that gravity
comes **last**, even though the training YAML lists `gravity_dir` first — the
concatenation follows the `PolicyCfg` attribute order, not the YAML order.

## 3. Reference motions

The runner reads a directory per motion containing `joint_pos.csv`,
`joint_vel.csv`, `body_pos.csv`, `body_quat.csv`, `body_lin_vel.csv`,
`body_ang_vel.csv` and `metadata.txt`.

The vendored `gear_sonic_deploy/reference/convert_motions.py` **cannot read
LUCID clips**: it expects a post-retarget pack with `joint_pos`, `body_pos_w`,
`body_quat_w` and friends, while a `robot_filtered` clip carries only `dof`,
`root_trans_offset`, `root_rot` and `fps`. The body arrays are not stored
anywhere — they are forward kinematics. Use instead:

```bash
python tools/convert_clip_for_deploy.py --clip <clip.pkl> --out <motions-dir>
```

Two conventions in that data are easy to get wrong and neither fails loudly:

* **Quaternions are WXYZ.** `motion_data_reader.hpp:272` declares
  `[timestep][body_id][wxyz]` and `math_utils.hpp:411` reads `w = quat[0]`. The
  training side and `tools/mujoco_player.py` use XYZW. This is the one boundary
  where they meet.
* **`metadata.txt` must contain a literal `Body part indexes:` line**, with the
  integers on the FOLLOWING line. `ReadMetadata` regex-scans that next line and
  returns false if the list comes out empty. Prose describing the bodies loads
  nothing.

Verify the whole bundle under the runner's own parsing rules before you go near
a robot:

```bash
python tools/verify_deploy_bundle.py <bundle-dir>
```

## 4. What to set up

**On the robot (Jetson, arm64).** This is where the build belongs. An x86
workstation build is not the deployable artifact even if it compiles.

`thirdparty/roboticsservice_1.0.0.0_arm64.deb` is **not** redistributed in this
repository: it is a proprietary third-party package (maintained by
`roboticsservice.support@bytedance.com`) with no license this repository can
point to, and it is arm64-only. Copy it from the upstream SONIC repository's
`gear_sonic_deploy/thirdparty/` if you need it on the Orin.

1. TensorRT — `find_package(TensorRT REQUIRED)` in `runner/CMakeLists.txt:28`.
   (Not `runner/src/g1/g1_deploy_onnx_ref/CMakeLists.txt`; there are two.)
   Install it, then `export TensorRT_ROOT=$HOME/TensorRT` in `~/.bashrc`;
   `scripts/setup_env.sh` reads it from there and prepends `$TensorRT_ROOT/lib`
   to `LD_LIBRARY_PATH`.
2. onnxruntime with its CMake package (`find_package(onnxruntime REQUIRED)`,
   `runner/CMakeLists.txt:30`), CUDA Toolkit ≥ 10.2
   (`runner/CMakeLists.txt:42`, `find_package(CUDAToolkit 10.2 QUIET)`).
3. `scripts/install_deps.sh` installs the rest: `just`, `clang`, `cmake`, `git`,
   `git-lfs`, `pkg-config`, `patchelf`, `zlib1g-dev`, `libgtest-dev`.
4. ROS 2 is optional. `setup_env.sh` sets `HAS_ROS2=1` and
   `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` when it finds it, otherwise `HAS_ROS2=0`
   and a bundled FastRTPS profile. Only needed for `--output-type ros2`.
5. Network: the robot is on `192.168.123.x`. Both SONIC's `deploy.sh real` and
   this bundle's `run.sh` auto-detect the interface with that prefix — check
   which one it picked before trusting it. `run.sh` refuses to guess if it
   cannot find a `192.168.123.x` NIC, rather than falling back to something
   else.

**Launch, fused path:**

```bash
cd gear_sonic_deploy
just run g1_deploy_onnx_ref <iface> \
    <bundle>/policies/deploy_dr_s8600_g1.onnx \
    <bundle>/motions/ \
    --obs-config <bundle>/config/observation_config_lucid_g1_1570.yaml \
    --input-type manager --output-type all
```

Add `--disable-crc-check` for MuJoCo/loopback simulation; both `deploy.sh sim`
and this bundle's `run.sh --sim` add it automatically, and neither adds it
without the sim flag. Input types are `keyboard`, `gamepad`, `gamepad_manager`,
`manager`, `zmq`, `zmq_manager`, `ros2`.

In this bundle the equivalent launch is one command, and it pins the clip:

```bash
bash run.sh --policy deploy_dr --iface <iface> --motion <clip>
```

**Bench first.** `bash run.sh --policy deploy_dr --sim --viewer` runs the same
binary against a MuJoCo G1 over loopback, and `bash drill.sh --play --parity`
scripts the whole sequence and checks the engine. Do that before any run on the
robot, every time.

## 5. Timing

Four real-time threads (`g1_deploy_onnx_ref.cpp:14-21`): Input 100 Hz, **Control
50 Hz**, Planner 10 Hz, Command Writer 500 Hz. The control rate matches training
exactly (`decimation 4 × sim_dt 0.005` = 50 Hz).

Two things to watch. The four threads are created with `UT_CPU_ID_NONE`, so
**none is pinned to a CPU**, and SCHED_FIFO is applied to the main thread only,
*after* the workers spawn — on a Jetson also running TensorRT inference the
50 Hz loop's timing is not bounded. A missed deadline degrades silently: the
500 Hz writer republishes the last target. Measure the realised control period
before drawing any conclusion from the robot's behaviour.

## 6. How the randomization carries over — and how it does not

**You do not reproduce domain randomization at deployment, and the runner has no
facility to.** Randomization is a training-time device for producing a policy
whose competence covers a range of robots. At deployment the real robot *is* one
sample from that range. The deployment question is therefore not "how do I
re-create the DR" but **"is this robot inside the envelope the policy trained
on"**, which is a measurement on your hardware:

| training channel | envelope trained | what to measure on the robot |
|---|---|---|
| actuation latency | **0–60 ms** | end-to-end sensor→torque delay, inference included. Our MuJoCo ladder shows 0 falls of 16 through 0–80 ms and 6 of 16 at 0–120 ms, so 60 ms is the trained ceiling and ~80 ms is where the measured margin runs out. |
| push | ±1.5 m/s planar root velocity increment, every 1–3 s | not directly measurable. See the caveat below. |
| friction | λ 1.0 envelope around nominal | floor material; the sim friction floor clamps near λ 1.385, so wet or polished surfaces are outside what was trained. |
| body mass | λ 1.0 envelope, wrists and torso | any payload, tools or added hardware. |
| torso centre of mass | λ 1.0 envelope | battery or backpack placement. |
| joint zero offsets | λ 1.0 envelope | encoder calibration error. |

**The push channel does not convert.** It is a root velocity increment written
directly into simulator state, with no mass, no duration and no contact point,
so it names no impulse in newton-seconds. Multiplying by the robot's ~35 kg to
get ~52 N·s is arithmetic, not a measurement: a real shove has a contact point
and a duration, and produces angular momentum the simulator's increment does
not. Do not put a newton-second number on this policy's push tolerance without a
standardized physical protocol.

To decide whether the policy is adequate for a given environment, use the
**MuJoCo severity ladder as the acceptance test** rather than trying to
re-create DR on hardware: pick the λ that matches how disturbed your environment
is, and read the fall rate off the measured table in the bundle README. That is
what those sweeps are for.

## 7. The limit that randomization cannot fix

The deployed observation contains joint positions and velocities, base angular
velocity from the IMU, projected gravity, past actions, and the reference
motion. It contains **no horizontal position term at all** — the runner's
76-entry registry has no xy, no path error, no odometry; the only
position-bearing reference term is `motion_root_z_position`, which is vertical.

In a paired-state experiment over 1,536 base states and 6,144 horizontal
translations of ±0.25 m, every actor-side observation and every action mean
changed by exactly 0.0. **The policy cannot perceive that it has drifted off the
path.** Both LUCID arms leave the reference path within seconds in MuJoCo, the
randomized one sooner, and this is why — not the randomization.

The repair drafted in this project appends the reference-minus-robot horizontal
displacement, which in simulation is a motion-library lookup indexed by the
integer simulator clock minus the simulator's own body position: absolute,
drift-free, and unavailable on a real G1. A proprioceptive-only legged state
estimator is reported at ~83 cm of relative pose error over 10 m on ANYmal.
Until a real position source is named and characterized, treat any deployment of
these policies as **open-loop in the horizontal plane** and expect the drift the
sweeps measure.

## 8. Before any hardware run

1. `verify_deploy_bundle.py` passes.
2. `validate_deploy_obs_config.py` passes against the runner you built.
3. **Value-level parity — and it is not the `parity/` test.** Two separate
   checks, on different data, that must not be collapsed into one claim
   (`docs/RESULTS.md`, "Running it" item 2, has both):

   - `bash drill.sh --play --parity` drives the runner against the MuJoCo robot
     and compares **its TensorRT engine against the shipped ONNX, on the
     runner's own observations**. It never reads `parity/`. This is the check
     that a mismatched TensorRT fails, and the one to re-run on the machine you
     will deploy from. It has been run and it passes — README **Limits** #2 has
     the figures and the version-pin argument.
   - `parity/` holds golden (observation, action) traces from the reference
     implementation, and is consumed by `bash test.sh`, check 4 of 8 — ONNX
     against the golden actions, no runner and no GPU.

   Passing the first says nothing about the runner's **observation pipeline**:
   it feeds the engine and the ONNX the same runner-built observation, so a term
   the runner transposes is invisible to it. That question is still open.

   Record the baseline **per policy**, and compare policies only inside one
   measurement series. The one series that measured both — 3 repeats, 648 ticks,
   same GPU, same runner, same pinned 10.13.3 — reads `deploy_dr` mean |delta|
   **2.07e-06** against `no_dr` **6.88e-05**, a **~33×** spread that is a
   property of the networks, not of the install
   (`tools/check_runtime_parity.py`, MEASURED RESULT, the table whose columns
   are `no_dr` and `deploy_dr, same series`). `deploy_dr`'s **1.45e-06** is a
   *different* run — the separate 499-tick one in the table above it — and a
   run of that length that happens to miss the known logging-artefact tick reads
   mean ~4.2e-07 instead, and also passes. Do not divide figures from two
   different series against each other. Judge a new export against its own first
   reading, not against `deploy_dr`'s.
4. Bench first: `bash drill.sh --play --parity`, then `bash run.sh --sim
   --viewer` to drive it by hand.
5. Measure the realised control period and end-to-end latency.
6. Safety, which does not exist in this software and must be provided
   independently: a hardwired emergency stop, a fall-arrest harness or gantry,
   and a fallback controller. The runner's stop path is a software boolean
   (`:2686-2700`) plus a 35 rad/s joint-velocity abort (`:2833-2837`) and a
   motor-temperature hysteresis entering at 90 °C and clearing at 85 °C
   (`:2846-2853`). None of those is an emergency stop. For a humanoid, cutting
   power is itself a hazard, because the robot falls.

## 9. Status of each claim here

Updated after `sim/run_robot_sim.py` landed and after run 001; the 2026-09-09
revision predated both, which is why the engine-parity row below used to say it
needed a robot state source. Run 001 on hardware was 2026-09-11
(`docs/HARDWARE_RUNS.md`).

| | |
|---|---|
| the runner builds | **VERIFIED** — configures and compiles 100%, 0 errors, with the toolchain from `setup_deploy_toolchain.sh`. It had never been built in this repository before. |
| the runner loads this bundle | **VERIFIED** — it read all three motions (406 / 457 / 432 timesteps), compiled the policy to a TensorRT engine, and reported `INPUT obs_dict [1, 1570]` / `OUTPUT action [1, 29]` |
| the observation layout | **VERIFIED by the runner itself**, which printed the eight terms at offsets 0, 290, 580, 640, 670, 960, 1250, 1540, total 1570, "Dimension match: Configuration is valid!" — identical to the layout derived from `tools/mujoco_player.py` |
| ONNX signatures | measured with onnxruntime |
| motion CSV format, quaternion order, metadata format | verified by re-implementing the runner's parsers, then confirmed by the runner loading them |
| body index list | verified — independently derived, reproduces the repository's example bundle exactly |
| joint order, kp/kd, default pose, action scale, control rate | verified — byte-identical between `robots/g1.py` and `policy_parameters.hpp` |
| the runner's TensorRT engine reproduces the shipped ONNX on the runner's own observations | **VERIFIED** — `bash drill.sh --play --parity`, TensorRT 10.13.3, FP32, 499 ticks, mean \|delta\| 1.45e-06 (README **Limits** #2). `sim/run_robot_sim.py` supplies the robot state source this row once said was missing. |
| the runner's observation pipeline agrees with the one that produced `parity/` | **NOT answered** — the check above feeds the engine and the ONNX the *same* runner-built observation, so a term the runner transposes is invisible to it. `docs/RESULTS.md`, "Running it" item 2, has the detail and why `--compare` and `--policy-input-logfile` do not yet meet. |
| init + fixed stand on hardware | verified once — run 001, 2026-09-11, in a gantry harness; see `HARDWARE_RUNS.md` |
| POLICY behaviour on hardware | **NOT verified** — `]` has never been pressed on a robot |

### What running it actually found

Three defects that static checking had passed:

1. **Inline YAML comments break the runner.** It does not use a YAML parser;
   `ExtractValue` (observation_config.hpp:452-470) trims whitespace and quotes
   and nothing else, so `- name: "x"  # note` yields the term name `x"  # note`
   and startup aborts with "Unknown observation function". Reading the file with
   PyYAML hides this entirely. `validate_deploy_obs_config.py` now reproduces
   ExtractValue instead of trusting `yaml.safe_load`.
2. **Velocity convention.** `ComputeGlobalVelocities` uses a central difference
   for linear velocity, a WORLD-frame one-frame quaternion delta for angular,
   and a Gaussian filter of sigma 2. The converter now matches all three; it
   previously used forward differences and a body-frame delta.
3. **`crt/` headers.** `cuda_runtime_api.h` includes four headers from `crt/`,
   which ship with the compiler headers rather than the runtime, so
   `cuda-cudart-dev` alone does not compile. `cuda-crt` is 82 kB.

### One discrepancy left open, deliberately

The runner's `test_fk` compares its own forward kinematics against the body
arrays in a bundle. Against ours it reports mean position error 0.128 m on the
non-anchor bodies. **Run the same test against the repository's own shipped
example data and it fails too**, and worse on orientation: quaternion mean error
0.87 against our 0.21, angular velocity 0.49 against our 0.075. The test's own
author left the comment "I couldn't get this quite right ... Should get to the
bottom of this and fix it though."

This does not affect a policy using the config in this bundle. The only body
array the 1570-term observation reads is the ANCHOR quaternion,
`BodyQuaternions(frame)[0]`, and body 0 is exact by construction: `RobotFK::DoFK`
assigns `positions_world[0] = root_translation; rotations_world[0] =
root_rotation` before descending the chain, and our anchor was separately checked
against the clip's own root pose at several frames.

It would matter for any config that reads non-anchor bodies. Before using one,
settle whose forward kinematics is right.
