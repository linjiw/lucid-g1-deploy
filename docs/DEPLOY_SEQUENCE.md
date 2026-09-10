# The deployment sequence

What the runner does, in order, and what you do at each step. Everything here
was read out of `runner/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp`
and then executed against a MuJoCo G1 on the DDS bus — see **Rehearsing it**.

---

## The state machine

`ProgramState { INIT, WAIT_FOR_CONTROL, CONTROL }` — three states, and it only
ever moves forward.

### Before INIT — taking low-level control

At construction the runner releases whatever Unitree's own high-level controller
is doing, then opens its DDS channels:

```cpp
msc_ = std::make_unique<MotionSwitcherClient>();
msc_->SetTimeout(5.0f);
msc_->Init();
while (msc_->CheckMode(form, name), !name.empty()) {
  if (msc_->ReleaseMode()) std::cout << "Failed to switch to Release Mode\n";
  sleep(5);
}
ChannelFactory::Instance()->Init(0, networkInterface);
```

With no robot on the network `CheckMode` times out, `name` stays empty and the
loop falls through immediately. On a real G1 this is the step that stops the
built-in sport mode from fighting you, and it can take several seconds.

Then it loads the reference motions, loads the observation config, checks the
config's dimensions against the policy's input, and builds or loads a TensorRT
engine. **This is the slow part** — tens of seconds on a first run, because the
engine is compiled. Nothing has moved yet.

### 1. INIT — the ramp to the default pose

`InitControl()`, at 50 Hz. It returns false and prints
`LowState is not available, waiting for robot to be ready` until the first
LowState arrives. Once it does:

```cpp
double ratio = std::clamp(time_ / duration_, 0.0, 1.0);   // duration_ = 3 s
q_target[i] = current_pos * (1.0 - ratio) + default_angles[i] * ratio;
kp[i] = kps[i];   kd[i] = kds[i];   tau_ff[i] = 0;
```

A 3-second linear interpolation from wherever each joint is to
`default_angles`, at full PD. Prints `Init Done`. The Dex3 hands are held closed
during the ramp and opened at the end.

**The ramp starts from wherever the joints are.** It does not check that the
starting pose is safe, reachable, or that the robot is upright.

### 2. WAIT_FOR_CONTROL — the fixed stand

Holds `default_angles` under the same PD gains and re-runs `CheckSafety()` at
50 Hz. **This is the "stand fixed" state.** The policy is not running; nothing
is being inferred; you can sit here as long as you like.

`CheckSafety()` fails, and forces a stop, if LowState is missing or older than
`LOW_STATE_ABSENT_THRESHOLD`. A pulled cable ends the run here.

It leaves this state only when `operator_state.start` becomes true — the `]`
key.

### 3. CONTROL — the policy runs

The four threads do their work: Input at 100 Hz, Control at 50 Hz, Planner at
10 Hz, Command Writer at 500 Hz. `CheckSafety()` still runs every control tick.

### 4. Stop

`O` sets `stop_control`, which the input handler turns into
`operator_state.stop`. Three things then happen:

```cpp
void Control() { if (operator_state.stop) { return; } ... }   // every tick, forever
```

```cpp
void Stop() {
  operator_state.stop = true;
  /* join input, control, command-writer and planner threads */
  CreateDampingCommand();
  LowCommandWriter();
}
```

```cpp
void CreateDampingCommand() {
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    tau_ff[i] = 0; q_target[i] = 0; dq_target[i] = 0;
    kp[i] = 0;   kd[i] = 8;
  }
}
```

Zero stiffness, a little damping, no feed-forward torque. **For a standing
humanoid this means it goes down.** That is what this emergency stop is: it
removes the ability to hold a pose. It is the right thing to do when something
has gone wrong, and it is not a controlled sit.

`operator_state.stop` is set in four places — the `O` key, a failed safety
check, a failed state-gather, and `Stop()` — and is **cleared in none of them**.
`program_state_` is assigned `WAIT_FOR_CONTROL` and `CONTROL` and never anything
earlier. A stop is terminal.

### 5. Recovery to a stable stand

There is no in-process recovery. Recovery is restarting the runner:

```bash
bash run.sh --policy deploy_dr --iface <iface>
```

which re-enters INIT and ramps from wherever the joints ended up back to
`default_angles`. **Do that with the robot supported** — a harness, a gantry, or
hands. The ramp assumes the feet can take load, and after a stop they are
usually not under the robot.

If you need a stop that holds the robot up, that is a hardware property, not a
software one. No amount of policy training changes what `kp = 0` does.

---

## Operator keys

The default input interface is `keyboard` (`--input-type keyboard`). Keys are
read from stdin in raw non-blocking mode, so a pipe works exactly like typing —
which is how `drill.sh` scripts the whole sequence.

| key | effect |
|---|---|
| `]` | start control (WAIT_FOR_CONTROL → CONTROL) — **arms the policy only** |
| `O` | **emergency stop** (uppercase; the manager interface also takes `o`) |
| `N` | next motion |
| `T` | play / resume playback |
| `R` | restart the current motion at frame 0, paused |
| `I` | reinitialise heading from the current IMU |
| `Q` / `E` | delta heading ∓ π/12 |
| `F` | print motor temperatures |
| Enter | toggle planner mode |

`]` and `T` are two separate steps and both are required to track a motion.
`]` moves the state machine into CONTROL, so the policy starts running at 50 Hz —
but `operator_state.play` is still false, `current_frame_` never increments, and
every one of the ten reference frames in the observation is frame 0. `T` is the
only key that sets `play`; it is also set by the gamepad and ZMQ interfaces, and
nowhere else. A policy left armed but unplayed holds a still pose and looks like
it is failing to track.

In planner mode `W`/`S`/`A`/`D` drive, `1`–`8` pick a locomotion mode, and
``R``/`` ` `` is the planner's own emergency stop (momentum reset), which is
**not** the same thing as `O`.

---

## Rehearsing it without a robot

`sim/run_robot_sim.py` puts a MuJoCo G1 on the DDS bus, publishing and
subscribing the same topics with the same IDL the robot uses. The runner
binary, its arguments and the keystrokes are unchanged; only the far end of the
wire is different.

```bash
source env.sh
bash drill.sh                      # deploy_dr, ~60 s, headless
bash drill.sh --policy no_dr       # the control policy
bash drill.sh --viewer --hold 20   # watch it
```

Torque is applied exactly as the motor controller applies it, from the fields
in the LowCmd the runner actually sent:

```
tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq),   clipped to the motor limit
```

so `kp = 0, kd = 8` goes limp here for the same reason it does on the robot.

A run looks like this — measured, `deploy_dr`, TensorRT 10.13, on
`walk_arc_cw_stop_001__A047`, which is the clip `drill.sh` defaults to and the
one every number in `docs/RESULTS.md` is about:

| phase | pelvis z | horizontal travel | commanded kp[0] / kd[0] |
|---|---|---|---|
| held, no command | 0.793 m | 0.00 m | 0 / 0 |
| INIT ramp | 0.791 → 0.759 | 0.00 m | 99.1 / 6.3 |
| **fixed stand** | 0.759 → 0.772, held 5.5 s | 0.06 m | 99.1 / 6.3 |
| policy | 0.771 → 0.081 | **0.22 m** | 99.1 / 6.3 |
| after stop | 0.069 | 0.07 m | 0 / 8 |

Two things to read off that table. The policy travelled **0.22 m** while the
reference walked an arc — that is Limits #1, the missing horizontal-position
term, visible directly. And it went **down within seconds**, which is the step at
`]` described below, not a failure of the rehearsal.

Pass `--motion <name>` to rehearse a different shipped clip, or `--all-motions`
to hand the runner all of them (the runner then starts on whichever sorts
first — `crouch_idle` — and `N` cycles).

The drill reads the runner's phase transitions off its own log, and detects the
WAIT_FOR_CONTROL → CONTROL edge from the wire alone: in the pre-policy phases
the commanded targets are constant at `default_angles`, and when the policy
takes over they start moving again.

---

## The step at `]`, measured

`INIT` ramps to `default_angles` and `WAIT_FOR_CONTROL` holds there. The instant
you press `]`, the policy is handed frame 0 of the reference motion. If frame 0
is far from `default_angles`, the policy has to close that gap in one control
step — standing, with its weight on its feet.

Measured on the three clips shipped here (`tools/check_motion_start.py`):

| clip | RMS over 29 joints | worst leg joint |
|---|---|---|
| `crouch_idle_004__A246` | 0.409 rad | R_hip_pitch −0.816 |
| `walk_arc_cw_stop_001__A047` | 0.299 rad | L_knee −0.548 |
| `walk_ff_stop_270_R_very_slow_001__A445_M` | 0.341 rad | R_knee −0.546 |

On the walk clip both knees are bent 0.669 rad by the ramp and the reference
wants 0.121 — a 0.55 rad step, on both legs, at once.

**This is why `evaluate.sh` and `drill.sh` disagree.** The sweep resets the
robot *onto* the reference with the reference's velocities, so it never sees the
step and reports 0/16 falls at λ0. The rehearsal starts the robot in the
runner's own standing pose, the way a robot actually starts, and the policy goes
down within seconds. The rehearsal is the one that resembles deployment.

Nothing in the runner closes this gap for you: `InitControl()` only ever ramps
to `default_angles`. Options, in order of preference:

1. Ship clips that begin near the default stance. The only fix that removes the
   step instead of managing it.
2. Put the robot in the clip's starting pose before pressing `]`.
3. Deploy supported, and expect the first second to be rough.

Do **not** quietly prepend an interpolated lead-in to the clip. The observation
carries ten future reference frames, so a synthetic lead-in changes what the
policy sees as well as what it tracks, and it was not trained on it.

---

## The support problem, measured

**The fixed stand does not hold a free-standing G1 in simulation.** Holding
`default_angles` with the runner's own `kps`/`kds` and no external support, in
the vendor's own MuJoCo model, the robot sits down in about 1.4 s:

```
   t     pelvis   ankle_pitch error
 0.00     0.759      +0.000
 0.40     0.734      +0.160
 0.80     0.721      +0.309
 1.00     0.686      +0.468
 1.20     0.593      +0.504     <- rolling onto the toes, contacts 16 -> 4
 1.40     0.275
 2.00     0.095      +0.010     <- sitting, holding the commanded pose
```

Hip and knee errors stay under 0.18 rad throughout. The joint that runs away is
**ankle pitch**, whose stiffness is `2 × STIFFNESS_5020 = 28.5 N·m/rad`. These
gains are derived from rotor inertia and a 10 Hz natural frequency
(`stiffness = armature × ω²`), not from the moment needed to balance a body — so
the ankle cannot hold the balance torque, the robot pitches forward, and it goes
over.

This is corroborated by the vendor's own configuration:
`sim/wbc_configs/g1_29dof_sonic_model12.yaml` ships with
`ENABLE_ELASTIC_BAND: True`. The band is on by default because the pre-policy
phases need support.

**What this means for hardware.** Do not assume the robot will stand on its own
between `Init Done` and `]`. Support it. `drill.sh` models this with the elastic
band, anchored at the standing pose and released the instant the policy takes
over, so the policy is never helped by it.

*One caveat, stated plainly:* this is a simulation result about a simulation
model. It says the stand is not statically stable at these gains in MuJoCo. It
does not prove a real G1 falls over — the real machine has gearbox friction,
series elasticity and a firmware-level current loop that MuJoCo does not model,
any of which could hold it. Treat it as a reason to have the harness on, not as
a measured fact about hardware. Nothing in this bundle has been on a robot.

---

## Checklist before hardware

1. A hardwired emergency stop that cuts motor power, within reach, tested.
2. A harness or gantry taking the robot's weight, with slack for the motion.
3. Clear floor, no people inside the reachable volume.
4. Robot network verified: `docs/ETHERNET_AND_SDK.md`.
5. `bash test.sh` green on the machine you will deploy from.
6. `bash drill.sh` run at least once, so you have seen the sequence.
7. `bash test.sh` check 8 read, and the step at `]` understood for the clip you
   are about to run.
8. TensorRT version confirmed — 10.13 on x86_64, 10.7 on the onboard Orin.
   `source env.sh` prints it. This is not a formality: measured on this bundle,
   the engine-vs-ONNX mean disagreement is **1.45e-06 at 10.13** and
   **1.10e-04 at 10.16** — systematically 76× worse at the wrong version. Run
   `bash drill.sh --parity` on the machine you will deploy from.
9. Read `../README.md` **Limits**. In particular: these policies cannot perceive
   their own horizontal position, and no value-level parity against the runner
   has ever been run.
