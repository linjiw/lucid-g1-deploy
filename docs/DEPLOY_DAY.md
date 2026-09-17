# Deployment day

The order to do things in, and what each step is actually worth. Written against
this bundle, executed against the MuJoCo robot on the DDS bus.

**This bundle has been on a G1 once — `INIT` and the fixed stand, in a gantry
harness, on 2026-09-11 — and the policy was never armed.** Every number below is
simulation. `docs/HARDWARE_RUNS.md` records that run and what it established.
This procedure cannot make an armed hardware run safe; it can only stop you from
discovering a software problem while the robot is powered.

A tick-through version of this page, for use in the lab, is
`docs/deploy-day-checklist.html`. **Open that file from disk — it is the source
of record.** It is also published at
<https://claude.ai/code/artifact/0775dd15-78a8-4b64-bcaa-f78f4f1f411a> for
convenience; republish the local file to that same URL rather than creating a
second artifact, and never edit the hosted copy. Do not plan the day around the
URL: the deploy machine is wired to `192.168.123.0/24` for the robot and may
have no route off that subnet.

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

Measured here on `deploy_dr`, TensorRT 10.13.3, FP32.
Write down **both** numbers. The tool prints them on **two separate lines** —
`tools/check_runtime_parity.py` emits `  max |delta|   ...` and then
`  mean |delta|  ...` after `compared <n> ticks` — so do not go looking for one
line with both on it. The reading on record for this bundle is README's
"Value-level parity" block: `compared 499 ticks`, **max |delta| 5.722e-06**,
**mean |delta| 4.204e-07**. The README compresses the pair onto one line; that
is the README's formatting, not the tool's.

**Two different means are both correct, and you may get either.** A run that
contains the known logging artefact tick reads **mean ~1.45e-06, max 2.37e-03**
(`tools/check_runtime_parity.py`, MEASURED RESULT, its 499-tick table); the
artefact-free runs the same file records as corroboration read **max 5.1e-06 and
6.2e-06 over 498 ticks**. So the tick count does not tell the two apart — the
499-tick run quoted just above and the 499-tick run quoted here are different
runs. The outlier is one tick in 499 where `obs_buffer_` is dumped after
`Infer()` has already consumed it, so the log catches one input update later
than the inference did — it affects the log, not the robot. The four checks that
establish that are in `tools/check_runtime_parity.py`, under "its one outlier
(1 tick, 2.37e-03) is" — the four bullets that follow it. Both readings pass;
neither is a regression.

**Record the baseline per policy — and compare policies only inside one
series.** The one series that measured both is 3 repeats, 648 ticks, same GPU,
same runner, same pinned 10.13.3: `deploy_dr` mean |delta| **2.07e-06** against
`no_dr` **6.88e-05**, a **~33×** spread that is a property of the networks, not
of the machine (`tools/check_runtime_parity.py`, MEASURED RESULT, the table whose
columns are `no_dr` and `deploy_dr, same series`). `deploy_dr`'s **1.45e-06**
above is a *different* run — the separate 499-tick one — so do not divide the
two against each other; that gives a 47× that no single measurement supports.
Judge a newly exported policy against its own first reading, never against
`deploy_dr`'s.

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

**The NIC name below is an example, not a constant.** It is `enp3s0` on the
bench machine and was `enp130s0` on the machine that did run 001
(`docs/HARDWARE_RUNS.md`). Find yours first and substitute it everywhere in this
block:

```bash
ip link                                                # find your NIC
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
bash run.sh --policy deploy_dr --motion walk_arc_cw_stop_001__A047
```

**No `--iface` here on purpose.** With none given, `run.sh` scans for the
interface holding a `192.168.123.x` address, prints
`auto-detected robot interface: <name>`, and refuses outright rather than
guessing if it finds none. Read that printed name back and check it is the NIC
you wired in Phase 1. Pass `--iface <name>` only to override a wrong pick —
hard-coding a NIC name in this procedure is how the two halves of it came to
disagree.

`run.sh` also prints `log      <dir>` before the runner starts — but not next:
it comes after the `policy` / `motion` / `iface` banner and, on a real robot,
after the safety confirmation. That directory is where this run is being
recorded; see 5.4.

**The four flags that decide what is kept.** `--csv-logs` / `--no-csv-logs`
force the runner's CSV set on or off (default: on for a real robot, off for
`--sim`). `--log-dir <dir>` moves the **parent** of the run directory, not the
run directory itself. `--assume-safety-checklist` skips the `Proceed? [y/N]`
question — which exists in that form because `run.sh` reads the answer from
`/dev/tty`, so a pipe cannot answer it for you. Use that last flag only when you
are asserting the harness, e-stop and fallback yourself. `bash run.sh --help`
prints all of them from the script's own header.

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

**3.3 Health check — and there isn't one.** **[read]**

This is the step the procedure wanted and the runner does not have. Pressing the
temperature key during the fixed stand does **nothing**, whichever key it is in
your mode: the temperature handler sits inside `case ProgramState::CONTROL`
(`g1_deploy_onnx_ref.cpp:3859`; that case opens at `:3839`), and so does the
`Loop timing` print (`:4049`).

**Which key that is depends on `--input-type`.** In the default `keyboard`
interface, motion-tracking branch (`use_planner = false`,
`keyboard_handler.hpp:85`), the temperature request is bound at
`keyboard_handler.hpp:318-319` `case 'h': case 'H'`; `F` is bound only in the
planner branch, `:256-257`, which Enter switches to. Under
`--input-type manager` or `gamepad_manager` it is the other way round — the
manager reads stdin first and takes `F` for itself. "The operator keys" below
has both. The runner's own comment at `g1_deploy_onnx_ref.cpp:3858` says
"(F key)"; that is right for the manager interfaces and the planner branch and
wrong for the default one, so do not take it as confirmation. Earlier revisions
of this page named `F` for the default mode, and the rehearsal that "confirmed"
the silence held `F` down through a whole stand: it produced no output, but `F`
is unbound in that branch, so that rehearsal established nothing either way.
**`H` has never been pressed — not in a stand, not while armed.** The claim
above is read from source, not executed.

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
bash standby.sh          # add --iface <name> only if the auto-detect is wrong
```

**5.4 Copy the run directory off the machine. Before the next run.** **[read]**

This is the step run 001 did not have, and run 001 is why it is here: `run.sh`
wrote no files then, the terminal scrollback was the only copy, and it is gone
(`docs/HARDWARE_RUNS.md`). `17491f9` made every run record itself.

`run.sh` creates `results/run/<timestamp>-<policy>[-<motion>]/` and prints its path
as `log      <dir>` at launch, before the runner starts. The stamp has one-second
resolution, so a relaunch inside the same second gets `-2`, `-3` appended
(`run.sh`, the `while [ -e "$RUNDIR" ]; do RUNDIR="$RUNBASE-$RUNSEQ"` loop) —
read the printed path, do not reconstruct it. It holds:

| | |
|---|---|
| `console.log` | the runner's stdout and stderr, line-buffered through `stdbuf -oL` so it is written as it happens, not in 4 KB lumps |
| `run_info.txt` | date, policy, motion, iface, real-robot-vs-sim, commit, TensorRT version, GPU, kernel, host, the exact exec line — and an exit line written from `run.sh`'s EXIT trap, so a crash or a Ctrl-C is recorded too |
| `csv/` | the runner's full CSV set. **On by default for a real robot**, off for `--sim`; `--csv-logs` / `--no-csv-logs` override that |

`--log-dir <dir>` names the **parent**, not the run directory: each run still
gets its own timestamped subdirectory under it. Two runs are never put in one
directory, because the runner's CSV sinks open with `std::ios::app`: they would
append into the same `q.csv` with `time_ms` restarting mid-file. That is what the
`-2` suffix above prevents.

**`.gitignore:13` excludes `results/`, so nothing here leaves the machine through
git.** `scp` or `rsync` the directory somewhere durable before you launch again.
`run.sh` says so itself on the way out, under `log saved: <dir>`.

---

## The operator keys

| key | effect |
|---|---|
| `]` | **arm** — WAIT_FOR_CONTROL → CONTROL (arms only; does not play) |
| `T` | **play** the clip from the current frame to its end |
| `O` | **emergency stop** — kp 0, kd 8, tau 0. Terminal. |
| `N` / `P` | next / previous motion |
| `R` | reset the clip to frame 0, paused — a **reference discontinuity** of the same kind if the policy is armed; its size depends on where in the clip you press it, and step 4.2 has the frame-0 distances that bound it |
| `I` | reinitialise heading from the current IMU |
| `Q` / `E` | delta heading ∓ π/12 |
| `H` | report motor temperatures — default `keyboard` interface only, and **only while armed**; silent in the stand. Under `manager` it is `F`; see below. |
| Enter | toggle planner mode — **no planner is loaded in this bundle**, so it prints `Planner not loaded - cannot enable` and, in the same tick, clears `play` and snaps the reference back to frame 0 with the policy still armed (`keyboard_handler.hpp:462-472`). That is the `R` discontinuity, caused by accident. Do not press Enter out of habit. |
| Ctrl-C | kill the process — **no damping command, terminal left raw** |

Lower case works for all of them except `]`.

**Which key reports temperatures depends on `--input-type`. State the mode
before you state the key.** This table is the default `keyboard` interface:
`H` is bound at `keyboard_handler.hpp:318-319`, and `F` only inside the planner
branch (`:256-257`), so under plain `keyboard` — no planner loaded — `F` does
nothing.

Under `--input-type manager` it is reversed, because `InterfaceManager` consumes
stdin before the keyboard handler sees it: `F` is the temperature key there
(`interface_manager.hpp:155-160`), and `H` is taken as **decrease left-hand
compliance** (`:124-129`) and never forwarded. `gamepad_manager` binds `F` the
same way (`gamepad_manager.hpp:136-140`). With this bundle's 1570-term
observation config the policy does not observe `vr_3point_compliance`, so the
runner prints a `Compliance control: DISABLED` banner at startup and that
compliance change is ignored for control (`g1_deploy_onnx_ref.cpp:2532-2538`) —
but you still lose the temperature reading you asked for. The runner's own
comment at `g1_deploy_onnx_ref.cpp:3858` reads "(F key)" and is wrong for the
default interface. Read from source, not executed.

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
