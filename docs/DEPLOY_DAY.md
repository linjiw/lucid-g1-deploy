# Deployment day

The order to do things in, and what each step is actually worth. Written against
this bundle, executed against the MuJoCo robot on the DDS bus.

**This bundle has been on a G1 once — `INIT` and the fixed stand, in a gantry
harness, on 2026-09-11 — and the policy was never armed.** Every number below is
simulation. `docs/HARDWARE_RUNS.md` records that run and what it established.
This procedure cannot make an armed hardware run safe; it can only stop you from
discovering a software problem while the robot is powered.

A tick-through version of this page, for use in the lab, is published at
<https://claude.ai/code/artifact/0775dd15-78a8-4b64-bcaa-f78f4f1f411a>
and its source is `docs/deploy-day-checklist.html` — republish that file to the
same URL rather than creating a second artifact.

Each step carries what it is worth:

| | |
|---|---|
| **[rehearsed]** | executed end to end against the MuJoCo G1 over DDS, result quoted |
| **[read]** | established by reading the runner source, not executed |
| **[hardware]** | cannot be rehearsed in software at all |

---

## Phase 0 — the machine, the day before

Do this on the machine you will deploy *from*, not on some other machine that
"has the same setup". The TensorRT engine is built per-machine and cached.

**0.1 Toolchain.** **[rehearsed]**

```bash
source env.sh          # must print TensorRT 10.13
ldd runner/target/release/g1_deploy_onnx_ref | grep cudart   # must say .so.12
```

10.13 on x86_64, 10.7 on the robot's own Orin. This is not a formality: measured
on this bundle, the engine-vs-ONNX mean disagreement is **1.45e-06 at 10.13** and
**1.10e-04 at 10.16**, systematically 76× worse at the wrong version.

**0.2 Bundle self-test.** **[rehearsed]** — 7 pass, 0 fail, 1 warn

```bash
bash test.sh
```

The one WARN is check 8, the start-pose step, and it is expected — see 4.2. Any
FAIL stops the day.

**0.3 Engine parity, and write the number down.** **[rehearsed]**

```bash
bash drill.sh --play --parity
```

Measured here on `deploy_dr`, TensorRT 10.13.3, FP32, 499 ticks:
**max |delta| 6.2e-06, mean 4.2e-07.**

**Record the baseline per policy.** `deploy_dr` sits near 2e-06 mean and `no_dr`
near 6.9e-05 on the *same* correctly-pinned install — a 33× spread that is a
property of the networks, not of the machine. Judge a newly exported policy
against its own first reading, never against `deploy_dr`'s.

**0.4 GPU headroom.** **[rehearsed]** — and found the hard way

```bash
nvidia-smi --query-gpu=memory.free --format=csv
```

The runner loads a 55 MiB engine and needs a CUDA context on top. Attempted on a
32 GB card with training jobs holding it, this failed **two different ways**:

At **183 MiB free**, it failed reading the engine and threw:

```
Requested amount of GPU memory (57662592 bytes) could not be allocated.
Error Code 2: OutOfMemory
terminate called after throwing an instance of 'std::runtime_error'    -> SIGABRT
```

At **168 MiB free**, it failed earlier still, inside CUDA init, and **segfaulted**:

```
createInferRuntime: Error Code 6: API Usage Error
  (CUDA initialization failure with error: 2 ...)
Unable to set GPU device index to 0. Current device has 1 CUDA - capable GPU(s).
                                                                       -> SIGSEGV
```

Both happen in the G1Deploy constructor at `:2300`, before the input interface
is constructed at `:2510` and long before any DDS command is written. The
simulator confirmed it: `lowcmd=no` for the entire run in both cases — the robot
was never driven. So the failure is fail-safe, but one of the two paths is a
segfault rather than a clean abort, and neither tells you "the GPU is busy" in
those words.

**Do not share the deploy machine's GPU with training jobs.** Check free memory
before you power the robot, and check it again on the robot's own Orin, where it
is shared with the system.

**0.5 Rehearse the sequence at least once.** **[rehearsed]**

```bash
bash drill.sh --play --viewer      # watch it, needs a desktop session
```

You should see: INIT ramp, ~5.5 s of fixed stand, the policy take over, the
robot go down, the stop. If you have not watched this happen in simulation, do
not watch it happen for the first time on a robot.

---

## Phase 1 — the network

**1.1 Address and link.** **[hardware]**

```bash
sudo ip addr add 192.168.123.222/24 dev enp3s0
sudo ip link set enp3s0 up
ping -c3 192.168.123.161
sudo tcpdump -i enp3s0 -c 20 udp portrange 7400-7500   # silence here = no DDS
```

Silence on the tcpdump means DDS is not flowing. Fix that before anything else;
the runner will otherwise sit forever printing `LowState is not available,
waiting for robot to be ready`, which looks like a hang and is actually correct
behaviour.

**1.2 `run.sh` will not guess.** **[rehearsed]**

With no `192.168.123.x` interface on this machine, `run.sh` refuses rather than
falling back to some other NIC:

```
no 192.168.123.x interface found; pass --iface (use 'lo' for a bench run)
```

Releasing Unitree's own controller happens inside the runner at construction
(`MotionSwitcherClient` → `ReleaseMode`, looping until the mode name is empty).
With no robot on the network it times out and falls through immediately; on a
real G1 it can take several seconds. Full wiring detail:
`docs/ETHERNET_AND_SDK.md`.

---

## Phase 2 — safety **[hardware]**

None of this is in the software, and none of it can be rehearsed.

1. A hardwired emergency stop that cuts motor power, within reach, tested.
2. A harness or gantry taking the robot's weight, with slack for the motion.
3. A fallback controller.
4. Clear floor, nobody inside the reachable volume.

The runner's stop path is a software boolean plus a 35 rad/s joint-velocity
abort and a motor-temperature cutoff entering at 90 °C. **None of those is an
emergency stop.** For a humanoid, cutting power is itself a hazard, because the
robot falls.

One thing here *is* rehearsed: `run.sh` reads its safety checklist from
`/dev/tty`, not stdin, so a pipe cannot answer it. **[rehearsed]** — with no
controlling terminal it refuses and exits rather than arming a robot with nobody
in the room.

---

## Phase 3 — the dry run: power on, stand, do not arm

The first run with the robot powered should never reach the policy.

**3.1 Launch, with the clip pinned.** **[rehearsed]**

```bash
bash run.sh --policy deploy_dr --iface enp130s0 --motion walk_arc_cw_stop_001__A047
```

**Always pass `--motion`.** Which clip is motion index 0 is *not defined*:
`motion_data_reader.hpp:685` enumerates the directory with an unsorted
`std::filesystem::directory_iterator`, so it is readdir order and
machine-dependent. Measured here the three clips come back `walk_ff`,
`walk_arc`, `crouch_idle` — not alphabetical, and not the order any listing
shows you. `--motion` narrows the directory to one entry, which is the only way
to know what `T` will play.

**3.2 Support the robot from `Init Done` onward.** **[rehearsed, simulation]**

Holding `default_angles` at the runner's own gains, unsupported, the robot sits
down in about 1.4 s in MuJoCo — ankle pitch (28.5 N·m/rad, derived from rotor
inertia, not from the balance moment) cannot hold the balance torque. The
vendor's own sim config ships `ENABLE_ELASTIC_BAND: True` for this reason.

This is a simulation result about a simulation model. It does not prove a real
G1 falls over. Treat it as a reason to have the harness on.

**3.3 Health check — and there isn't one.** **[rehearsed]**

This is the step the procedure wanted and the runner does not have. Pressing
`F` during the fixed stand does **nothing**: the temperature handler sits inside
`case ProgramState::CONTROL` (`:3859`), and so does the `Loop timing` print
(`:4049`). Rehearsed here — `F` held down through the whole stand produced not
one line of output.

So in WAIT_FOR_CONTROL the runner reports **no motor temperatures, no LowState
age, no loop timing**. The only things a dry run actually establishes are that
the engine loaded, that LowState is arriving (otherwise `Init Done` never
prints), and that nothing has errored.

**The consequence for sequencing:** `DEPLOY_G1.md` §8 item 5 asks you to measure
the realised control period and end-to-end latency. You cannot do that before
arming. Those numbers only exist once the policy is running, so the first time
you see them is the first time the robot is under policy control. Plan for that
— have someone watching the console whose only job is those lines.

Measured on the bench once armed: `LowState age` 2.2–23.0 ms and policy
inference 2.1–11.2 ms, the wide end of both being the first ticks after
start-up on a shared GPU. Note that 11.2 ms is over half of the 20 ms budget a
50 Hz control loop has.

**3.4 End without arming — and know what each exit does.** **[rehearsed]**

This matters more than it looks, because the two exits are not equivalent:

| exit | what it does | leaves the robot |
|---|---|---|
| **`O`** | `Stop()`: joins threads, writes one damping command kp 0, kd 8, tau 0 | **limp — it goes down** |
| **Ctrl-C** | SIGINT. The runner installs **no signal handler**, so the process dies immediately | **holding the last command, with nothing driving it** |

`ISIG` is not cleared (the keyboard handler clears only `ICANON|ECHO`), so
Ctrl-C really does raise SIGINT — and with no handler, `Stop()` never runs, no
damping command is ever written, and the `SimpleKeyboard` destructor that
restores your terminal never runs either. After a Ctrl-C your shell is left with
echo and line-buffering off and stdin non-blocking. Run `stty sane` to get it
back.

Rehearsed: SIGINT was sent to the runner during the fixed stand. The runner's
log ends at `Init Done` — no `Stopping G1Deploy`, no `Stop`, nothing — and the
simulator went on reporting `lowcmd=yes kp[0]=99.1 kd[0]=6.3` with the pelvis at
0.759 m for as long as it was watched. **The last full-PD command stayed latched
with no process behind it.**

Read that carefully before assuming it is benign. In MuJoCo the robot kept
standing because the simulator applies the last LowCmd forever and the elastic
band was on. A real G1's motor controller may have its own watchdog that does
something different when commands stop arriving — this bundle cannot tell you
which. The one hardware run to date never exercised it (see
`HARDWARE_RUNS.md`).

For a dry run on a supported robot, **`O` is the exit to use**: it is the one
that leaves a defined command on the wire, and it is the one the runner actually
has a shutdown path for. If you do hit Ctrl-C, run `stty sane` to get your
terminal back.

---

## Phase 4 — the first armed run

**4.1 Everything from Phase 3 still true:** harness loaded, e-stop in hand.

**4.2 Understand the step you are about to take.** **[rehearsed]**

`]` hands the policy frame 0 of the reference, and all three shipped clips start
far from the pose the runner holds:

| clip | RMS over 29 joints | worst leg joint |
|---|---|---|
| `walk_arc_cw_stop_001__A047` | **0.299 rad** | L_knee −0.548 |
| `walk_ff_stop_270_R_very_slow_001__A445_M` | 0.341 rad | R_knee −0.546 |
| `crouch_idle_004__A246` | 0.409 rad | R_hip_pitch −0.816 |

The policy must close that in one control step, standing, on its feet — a step
it never sees in training, where every episode resets *onto* the reference with
its velocities. Start with the smallest: `walk_arc`.

**4.3 Two keys, not one.** **[rehearsed]**

```
']'   arm    WAIT_FOR_CONTROL -> CONTROL. Policy runs at 50 Hz,
             reference parked at frame 0. THIS ALONE DOES NOT PLAY THE CLIP.
'T'   play   run the clip from the current frame to its end.
```

A policy left armed but unplayed tracks a still pose and looks like it is
failing. `T` is the only key that sets `operator_state.play`.

**4.4 Expect it to go down.** **[rehearsed]** Measured in the rehearsal,
`deploy_dr` on `walk_arc`:

```
FIXED STAND   0.759 -> 0.769 m   held 5.5 s   kp 99.1  kd 6.3
POLICY        0.768 -> 0.373 m   travelled 0.27 m      kp 99.1  kd 6.3
AFTER STOP    0.063 -> 0.064 m                         kp 0.0   kd 8.0
```

The reference clip walks an arc; the robot travelled 0.27 m. That gap is
**Limits #1** — the observation carries no horizontal-position term at all, so
the policy cannot tell it is off the path. Treat every deployment as **open-loop
in the horizontal plane**.

**4.5 If the robot falls off the bus, the runner catches it.** **[rehearsed]**

The one safety property in here that works the way you would want. The simulator
was SIGKILLed mid-policy — the robot simply gone from the bus, which on hardware
is a pulled cable, a switch reboot, or the robot powering off under you:

```
[ERROR] Lost LowState data connection from robot!
[ERROR] Safety check failed, stopping control.
[DEBUG] Stopping G1Deploy...
Stop
```

`CheckSafety()` runs every control tick, sees LowState older than
`LOW_STATE_ABSENT_THRESHOLD`, and forces a stop — and it goes through `Stop()`,
so the damping command *is* written. Note what that means physically: the
correct response to losing the robot is to make it go limp. If the cable pulls
mid-motion, the robot goes down. That is the right call and it is still a fall.

---

## Phase 5 — stop, and what recovery actually is

**5.1 `O` is terminal.** **[read, confirmed in every rehearsal]**
`operator_state.stop` is set in four places and cleared in none;
`program_state_` only ever moves forward; and `main` is
`while (!operator_state.stop) { sleep(0.02); }`, so a stop ends the **process**,
not just the run. There is no key that goes back to a stand.

**5.2 The restart does not pick the robot up.** **[rehearsed]**

Recovery is restarting the runner, which re-enters INIT and ramps to
`default_angles`. Measured across two cycles against one persistent robot:

| t | pelvis | kp[0] | |
|---|---|---|---|
| 44.7 s | 0.061 m | 0 | cycle 1 stop — on the floor |
| 68.8 s | 0.064 m | 99.1 | **cycle 2 INIT ramp begins, on a fallen robot** |
| 78.9 s | 0.133 m | 99.1 | still down — and the runner printed `Init Done` |

It then transitioned to CONTROL and played a whole clip to completion, reporting
every marker normally, with the robot flat on the ground.

**`Init Done` is a timer, not a measurement.** `InitControl()` interpolates for
`duration_ = 3 s` and declares itself finished. Nothing in it checks that the
robot is upright, that `default_angles` is reachable from where the joints are,
or that the feet are under the body. The software cannot tell a recovered stand
from a robot lying on its side, and it will report the second as success.

**A person has to put the robot back on its feet.** Then restart, supported.

**5.3 Repeating runs.** `standby.sh` is the loop: it pins one clip per launch,
runs it through `run.sh`, and gates every cycle on someone confirming the robot
is upright and supported. It adds nothing to the runner — each cycle is an
ordinary `run.sh`.

```bash
bash standby.sh --iface enp3s0
```

---

## The operator keys

| key | effect |
|---|---|
| `]` | **arm** — WAIT_FOR_CONTROL → CONTROL (arms only; does not play) |
| `T` | **play** the clip from the current frame to its end |
| `O` | **emergency stop** — kp 0, kd 8, tau 0. Terminal. |
| `N` / `P` | next / previous motion |
| `R` | reset the clip to frame 0, paused |
| `I` | reinitialise heading from the current IMU |
| `Q` / `E` | delta heading ∓ π/12 |
| `F` | report motor temperatures — **only while armed**; silent in the stand |
| Enter | toggle planner mode |
| Ctrl-C | kill the process — **no damping command, terminal left raw** |

Lower case works for all of them except `]`.

The Unitree wireless remote is also supported, unmodified, via
`--input-type gamepad` (or `manager` for keyboard *and* remote together, switched
with `Shift+1`/`Shift+2`): **Start** = `]`, **Select** = `O`, **A** = `T`,
**B** = `R`, **L1/R1** = `P`/`N`, **X/Y** = `I`. Note that the bundled simulator
does not publish `wireless_remote`, so **the remote cannot be rehearsed here** —
the first button press would be on the robot.

---

## What this procedure still does not establish

1. Anything about how a POLICY behaves on a robot. The one hardware run reached
   the fixed stand under a gantry and stopped there; `]` was never pressed. See
   `HARDWARE_RUNS.md`.
2. That the fixed stand holds an unsupported G1. In simulation it does not.
3. That a policy survives the step at `]`. In simulation it does not.
4. That the wireless remote works — it cannot be rehearsed in this bundle.
5. Recovery has no calibrated band, dwell or horizon, so no policy has passed or
   failed a recovery test. "0/16 falls" is a measurement on one clip, not a
   safety property.
