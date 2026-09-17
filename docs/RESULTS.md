# LUCID G1 deployment bundle — 2026-09-09

Two policies trained from scratch on the same three motions, same budget, same
seed. **The only difference between them is the randomization they trained
under.** Everything needed to run either one is here; everything that is *not*
yet true about running them on hardware is in **Limits**, which you should read
before the rest.

| | `no_dr_s8600_g1.onnx` | `deploy_dr_s8600_g1.onnx` |
|---|---|---|
| mass / CoM / joint / friction | nominal | λ 1.0 envelope |
| actuation latency | none | **0–60 ms** |
| push | none | **±1.5 m/s** planar, every 1–3 s |
| sha256 | `580bcb87…` | `c10804f3…` |

Both are the fused single-file export: input `obs_dict [1, 1570]`, output
`action [1, 29]`. No encoder, no separate decoder.

## Contents

```
policies/    the two ONNX policies
config/      observation_config_lucid_g1_1570.yaml   <- validated, USE THIS ONE
             deploy_metadata.json                    joint order, gains, action scale, rates
motions/     the three training clips as the runner's CSV bundle
parity/      golden (observation, action) traces for a value-level parity test
```

## Which policy for which environment

Measured in MuJoCo, 16 seeds per cell, on `walk_arc_cw_stop_001__A047` (8.6 s).
The **two falls tables** below are the full-clip window: each rollout run to the
end of the motion, so a policy that drifts early still has the chance to fall.
They report two distinct events separately: toppling over, and walking on but
more than 0.5 m off the reference path. The **time-on-path table** under them is
the *scored* window, which ends the rollout at 0.5 m -- see the note beneath it.

Reproduce the falls tables with `bash evaluate.sh` (~40 min). It passes
`--full-clip` to both sweeps (evaluate.sh:77,83) and writes
`results/perturbation/summary.md` and `results/latency/summary.md` -- the two
tables in `docs/measured_*_sweep_fullclip.md`, under shorter row labels
(evaluate.sh:63 keys the arms off the policy filenames; those files give the
exact invocation that reproduces the labels as committed). `evaluate.sh` does
not produce the scored-window files; `docs/measured_*_sweep.md` each carry their
own invocation.

These ladders are **not the same plant as `bash drill.sh --latency`**. The sweep
drives `runner/g1/g1_29dof.xml` (31 bodies / 30 joints / 29 actuators, 35.112 kg)
through `tools/mujoco_player.py`; the DDS rehearsal drives
`sim/models/g1/scene_43dof.xml` (45 / 44 / 43, 36.165 kg) with 14 Dex3 hand
joints the runner never commands. The control is identical -- the same 29 body
joints -- so what differs is 1.05 kg of hand mass, inertia and collision
geometry. The measurement is in `sim/VENDOR_PATCHES.md` section 3
(`base_sim.py` — `GEAR_SONIC_ROOT`); do not read a `drill.sh --latency`
number and a number from these tables as the same robot.

### Falls, out of 16

| perturbation | no DR | deployment DR |
|---|---|---|
| none (λ 0) | 0 | 0 |
| light (λ 0.5) | 0 | 0 |
| training envelope (λ 1) | **7** | **0** |
| heavy (λ 1.5) | **11** | **3** |
| extreme (λ 2) | **14** | **5** |

| latency alone | no DR | deployment DR |
|---|---|---|
| none | 0 | 0 |
| 0-20 ms | 0 | 0 |
| 0-40 ms (training envelope) | 0 | 0 |
| 0-60 ms (deploy_dr's ceiling) | **6** | **0** |
| 0-80 ms (held out) | **10** | **0** |
| 0-120 ms (held out) | **11** | **6** |

### Time on the reference path, mean

| perturbation | no DR | deployment DR |
|---|---|---|
| none (λ 0) | 5.5 s | 1.3 s |
| light (λ 0.5) | 3.4 s | 1.3 s |
| training envelope (λ 1) | 3.1 s | 1.4 s |
| heavy (λ 1.5) | 2.2 s | 1.4 s |
| extreme (λ 2) | 1.7 s | 1.5 s |

Latency alone: no DR decays 5.5 -> 2.8 s across the ladder; deployment DR sits
at 1.3 s at **every** level.

Every cell in this table is the `stop` figure from the **scored-window** files
(`docs/measured_perturbation_sweep.md`, `docs/measured_latency_sweep.md`) -- not
from the full-clip files the falls tables above come from, where every `stop`
cell reads 8.6s by construction. `scored()` in `tools/mujoco_sweep.py` computes
it as `sum(tf) / len(tf)` over `tf = [r["t_end"] for r in rs if r["fell"]]`:
the **mean** `t_end` over the runs the sweep marked failed, with runs that
tracked to the end excluded (`ok = sum(1 for r in rs if not r["fell"])`).
`fell` is set by a topple as well as by a band exit, so it is not a pure
time-to-leave-the-band: the split table in
`docs/measured_perturbation_sweep.md` shows `fell 3 · drifted 13` for `no_dr` at
λ 1.5, so 3 of the 16 values averaged into that cell's 2.2 s are topple times.
This heading said "median" until 2026-09-17; no number changed, the statistic was
mislabelled.

### Reading it

- **DR is what keeps the robot standing.** At the training envelope the no-DR
  policy is already on the floor in 7 of 16 runs and the DR policy in none. On
  latency the DR policy is perfect through 0-80 ms -- past its own 0-60 ms
  training ceiling -- while the no-DR policy is at 10 of 16.
- **DR costs path accuracy, and charges even when nothing perturbs it.** At
  λ 0 the no-DR policy holds the path four times longer.
- **DR's drift is flat.** 1.3 s to 1.5 s across every perturbation and every
  latency. It has traded path-following for a robustness that no longer depends
  on how hard you push it.
- **Neither policy tracked a full clip in any condition.** The three
  completions on record are all no-DR at light perturbation or mild latency,
  which at 16 seeds is not a real effect.

**So:** for contact, disturbance or real actuation delay, use `deploy_dr`. For
path accuracy in a controlled space neither is adequate and `no_dr` is merely
less bad. See limit 2 for why -- it is not a randomization problem.

### A note on the numbers

An earlier version of this table counted falls in the *scored* window, which
ends the rollout the moment the pelvis passes 0.5 m from the reference -- often
around 1.4 s. A run that has already ended cannot fall, so that window
undercounted badly: 3 of 16 at λ 1.5 against the 11 of 16 above. Both windows
are in the bundle -- `docs/measured_*_sweep.md` is the scored one,
`docs/measured_*_sweep_fullclip.md` the full one. (There is no `sweep_fullclip/`
directory, and nothing writes one: evaluate.sh:75-83 writes `$OUT/perturbation`
and `$OUT/latency`.) The tables here are the full-clip window, which is the one
that answers whether the robot stays on its feet.

## Running it

1. **Validate the observation config against the runner you are about to build.**
   ```bash
   python tools/validate_deploy_obs_config.py \
     config/observation_config_lucid_g1_1570.yaml \
     --expect-dim 1570 --expect-layout fused_g1_1570
   ```
   Do not skip this. The config shipped in `policy/release/` whose header names
   this exact export command lists **eight term names the runner's registry does
   not contain**, and orders its encoder terms differently from the exporter; it
   throws on load, and if the names were patched it would still place 66 floats
   in the wrong slots. Term order is assigned by file order, so a reordering
   does not fail — it silently feeds the policy shuffled input.

2. **Three parity questions; two of them are answered.** They use different
   data and different commands, so do not collapse them into one claim.

   - *Does the shipped ONNX reproduce the golden traces in `parity/`?* **Yes**,
     max |delta| 0.0 (MANIFEST.json:29). Checked by `bash test.sh`, check 4 of 8
     -- grep it for `hdr "4/8  golden parity traces match their policies"` -- and
     failed by `if err > 1e-4: rc = 1` in that check's heredoc. No runner, no GPU.
   - *Does the runner's TensorRT engine reproduce that ONNX, on the observations
     the runner itself built?* **Yes.** At the pinned TensorRT 10.13, FP32, 499
     control ticks, `deploy_dr` agrees to **mean |delta| 1.45e-06**. Reproduce
     with `bash drill.sh --play --parity`, which adds `--policy-input-logfile`
     and `--enable-csv-logs` to the runner and hands both logs to
     `tools/check_runtime_parity.py`. `--play` is not optional: `drill.sh` sends
     `T` only when it is given -- its sole `printf 'T'` is gated on
     `[ "$PLAY" -eq 1 ]` -- so bare `--parity` compares the engine against a
     reference parked at frame 0, a far narrower input distribution than the
     499 ticks this figure was measured over. That figure is a property of
     *that* network, not a pass mark: `no_dr`'s own correct baseline is **mean
     6.88e-05**. Take a policy's own baseline before comparing anything to it.
   - *Does the runner's observation pipeline agree with the one that produced
     `parity/`?* **Not answered.** The check above feeds the engine and the ONNX
     the same runner-built observation, so a term the runner transposes is
     invisible to it -- which is the failure class
     `tools/emit_parity_vectors.py:9-16` was written for. Closing it means
     comparing a runner action trace against the golden actions with
     `python tools/emit_parity_vectors.py --out parity/deploy_dr --compare
     <trace>`, and the two halves do not meet yet: `--compare` parses JSON rows
     carrying `action` (tools/emit_parity_vectors.py:140-147), while the
     runner's `--policy-input-logfile` writes one CSV row of 1570 floats per
     tick (the format `tools/check_runtime_parity.py` documents and consumes).
     Read from the source, not executed.

3. Convert any further clips with `tools/convert_clip_for_deploy.py`. The
   vendored `convert_motions.py` cannot read LUCID clips — it wants a
   post-retarget pack with body arrays that a `robot_filtered` clip does not
   carry.

## Limits — read before hardware

1. **No policy here has ever run on a robot.** The bundle has been on a G1 once
   — `INIT` and the fixed stand, in a gantry harness, 2026-09-11 — and `]` was
   never pressed. Every behavioural number in this file is simulation. See
   `HARDWARE_RUNS.md`.
2. **The policy cannot see its own horizontal position.** The observation
   contains joint angles, velocities, IMU angular velocity, projected gravity,
   past actions, and the reference motion — and *no* term for where the robot
   is relative to the path. The repaired input that would supply it is a
   motion-library lookup minus simulator ground truth: absolute, drift-free and
   unavailable on a real G1. **This, not randomization, is why both policies
   leave the path.** Deploying either without an external position source means
   accepting open-loop drift of the kind the table above measures.
3. **No recovery guarantee.** No band, dwell or horizon has been frozen in this
   project, so no policy has passed or failed a recovery test. "Fell 0/16" is a
   MuJoCo measurement on one clip, not a safety property.
4. **The pushes are simulator velocity increments**, in m/s written into state.
   They name no impulse in newton-seconds — no mass, no duration, no contact
   point — so they cannot be compared to a physical shove without a conversion
   nobody here has made.
5. **One training seed.** The measured between-seed effect on absolute
   capability in this project is 7.8 points, so the DR-vs-no-DR contrast is
   descriptive. It is not licensed as "DR is better" until it replicates on
   seeds 8601 and 8602.
6. **Safety hardware does not exist here.** The runner's stop path is a software
   boolean plus a joint-velocity abort and a motor-temperature cutoff. There is
   no hardwired e-stop hook, no fall-arrest rig and no fallback controller.
   Provide all three independently of this software. For a humanoid, cutting
   power is itself a hazard — the robot falls.
7. The MuJoCo XML re-clamps hip pitch and roll to ±88 N·m while training gave
   those joints 139 N·m, so the sim2sim numbers above are from a robot running
   four leg joints at 63% of the training torque ceiling.
