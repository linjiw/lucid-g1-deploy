Perturbation sweep (all six randomization channels scaled together by λ),
**scored window**: the rollout ends the moment pelvis-to-reference passes 0.5 m
— 1.3 s to 1.5 s for `deploy_dr`, 1.7 s to 5.5 s for `no_dr`, per the `stop`
column below — so a run that has already ended cannot then be recorded as
falling. That is why `no_dr` at λ 1.5 reports 3 falls of 16 here against 11 in
`docs/measured_perturbation_sweep_fullclip.md`; `docs/RESULTS.md` quotes the
full-clip file for falls and this one for the `stop` times.

`bash evaluate.sh` does **not** reproduce this file: it passes `--full-clip` to
both sweeps (evaluate.sh:77). From the bundle root:

    python3 tools/mujoco_sweep.py --out <dir> \
      --arm no_dr_s8600=policies/no_dr_s8600_g1.onnx \
      --arm deploy_dr_s8600=policies/deploy_dr_s8600_g1.onnx \
      --clip clips/walk_arc_cw_stop_001__A047.pkl \
      --lams 0 0.5 1.0 1.5 2.0 --seeds 16

The `--arm NAME=PATH` spellings are what keep the row labels below: the arm key
*is* the row label -- `row()` in tools/mujoco_sweep.py returns
`f"| {LABEL.get(arm, arm)} | "` -- while evaluate.sh:63 instead derives the
shorter keys `no_dr` / `deploy_dr` by stripping `_s8600_g1.onnx` off the policy
filenames. Same policies, same numbers, different row names.

The plant here is `runner/g1/g1_29dof.xml` via `tools/mujoco_player.py`, not the
43-DoF Dex3 model `bash drill.sh` drives -- 1.05 kg of hand mass apart, measured
in `sim/VENDOR_PATCHES.md` section 3 (`base_sim.py` — `GEAR_SONIC_ROOT`). One of
the six channels λ scales here is actuation delay (evaluate.sh:17-20), so these
numbers are not comparable with a `bash drill.sh --latency` rehearsal either.

### Scored criterion (pelvis-to-reference > 0.5 m ends the rollout)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 |
|---|---|---|---|---|---|
| no_dr_s8600 | 0/16 (0%, stop 5.5s) | 3/16 (19%, stop 3.4s) | 0/16 (0%, stop 3.1s) | 0/16 (0%, stop 2.2s) | 0/16 (0%, stop 1.7s) |
| deploy_dr_s8600 | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.4s) | 0/16 (0%, stop 1.4s) | 0/16 (0%, stop 1.5s) |

### What actually happened (fell over vs drifted off the path while upright)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 |
|---|---|---|---|---|---|
| no_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 13 · tracked 3 | fell 0 · drifted 16 · tracked 0 | fell 3 · drifted 13 · tracked 0 | fell 5 · drifted 11 · tracked 0 |
| deploy_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 1 · drifted 15 · tracked 0 |
