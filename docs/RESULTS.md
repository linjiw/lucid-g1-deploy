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

Measured in MuJoCo, 16 seeds per cell, on `walk_arc_cw_stop_001__A047` (8.6 s),
each rollout run to the **end of the motion** so a policy that drifts early
still has the chance to fall. Two distinct events are reported separately:
toppling over, and walking on but more than 0.5 m off the reference path.

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

### Time on the reference path, median

| perturbation | no DR | deployment DR |
|---|---|---|
| none (λ 0) | 5.5 s | 1.3 s |
| light (λ 0.5) | 3.4 s | 1.3 s |
| training envelope (λ 1) | 3.1 s | 1.4 s |
| heavy (λ 1.5) | 2.2 s | 1.4 s |
| extreme (λ 2) | 1.7 s | 1.5 s |

Latency alone: no DR decays 5.5 -> 2.8 s across the ladder; deployment DR sits
at 1.3 s at **every** level.

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
less bad. See limit 3 for why -- it is not a randomization problem.

### A note on the numbers

An earlier version of this table counted falls in the *scored* window, which
ends the rollout the moment the pelvis passes 0.5 m from the reference -- often
around 1.4 s. A run that has already ended cannot fall, so that window
undercounted badly: 3 of 16 at λ 1.5 against the 11 of 16 above. Both windows
are in the bundle (`measured_*_sweep.md` is the scored one, `sweep_fullclip/`
the full one); the tables here are the full-clip window, which is the one that
answers whether the robot stays on its feet.

## Running it

1. **Validate the observation config against the runner you are about to build.**
   ```bash
   python scripts/practice_utility/validate_deploy_obs_config.py \
     config/observation_config_lucid_g1_1570.yaml \
     --expect-dim 1570 --expect-layout fused_g1_1570
   ```
   Do not skip this. The config shipped in `policy/release/` whose header names
   this exact export command lists **eight term names the runner's registry does
   not contain**, and orders its encoder terms differently from the exporter; it
   throws on load, and if the names were patched it would still place 66 floats
   in the wrong slots. Term order is assigned by file order, so a reordering
   does not fail — it silently feeds the policy shuffled input.

2. **Run the value-level parity test.** `parity/` holds golden
   `(observation, action)` pairs from the reference implementation. Point the
   runner's `--policy-input-logfile` at the same clip and compare:
   ```bash
   python tools/emit_parity_vectors.py --out parity/deploy_dr --compare <runner.log>
   ```
   Dimensional agreement is verified. **Numerical agreement is not**, and only
   this test can establish it.

3. Convert any further clips with `tools/convert_clip_for_deploy.py`. The
   vendored `convert_motions.py` cannot read LUCID clips — it wants a
   post-retarget pack with body arrays that a `robot_filtered` clip does not
   carry.

## Limits — read before hardware

1. **The C++ runner has never been built or run in this repository.** No
   `build/`, no `CMakeCache.txt`, no logs. Everything stated about its
   behaviour is read from source.
2. **No policy here has ever run on a robot**, and no ONNX-versus-Isaac value
   parity test has ever been executed. That is what `parity/` is for.
3. **The policy cannot see its own horizontal position.** The observation
   contains joint angles, velocities, IMU angular velocity, projected gravity,
   past actions, and the reference motion — and *no* term for where the robot
   is relative to the path. The repaired input that would supply it is a
   motion-library lookup minus simulator ground truth: absolute, drift-free and
   unavailable on a real G1. **This, not randomization, is why both policies
   leave the path.** Deploying either without an external position source means
   accepting open-loop drift of the kind the table above measures.
4. **No recovery guarantee.** No band, dwell or horizon has been frozen in this
   project, so no policy has passed or failed a recovery test. "Fell 0/16" is a
   MuJoCo measurement on one clip, not a safety property.
5. **The pushes are simulator velocity increments**, in m/s written into state.
   They name no impulse in newton-seconds — no mass, no duration, no contact
   point — so they cannot be compared to a physical shove without a conversion
   nobody here has made.
6. **One training seed.** The measured between-seed effect on absolute
   capability in this project is 7.8 points, so the DR-vs-no-DR contrast is
   descriptive. It is not licensed as "DR is better" until it replicates on
   seeds 8601 and 8602.
7. **Safety hardware does not exist here.** The runner's stop path is a software
   boolean plus a joint-velocity abort and a motor-temperature cutoff. There is
   no hardwired e-stop hook, no fall-arrest rig and no fallback controller.
   Provide all three independently of this software. For a humanoid, cutting
   power is itself a hazard — the robot falls.
8. The MuJoCo XML re-clamps hip pitch and roll to ±88 N·m while training gave
   those joints 139 N·m, so the sim2sim numbers above are from a robot running
   four leg joints at 63% of the training torque ceiling.
