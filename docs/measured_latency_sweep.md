Latency sweep (actuation delay alone, physics nominal), **scored window**: the
rollout ends the moment pelvis-to-reference passes 0.5 m — 1.3 s flat for
`deploy_dr`, 2.8 s to 6.1 s for `no_dr`, per the `stop` column below — so a run
that has already ended cannot then be recorded as falling.
`docs/measured_latency_sweep_fullclip.md` is the same grid run to the end of
the 8.6 s clip, and `docs/RESULTS.md` quotes that one for falls and this one
for the `stop` times.

`bash evaluate.sh` does **not** reproduce this file: it passes `--full-clip` to
both sweeps (evaluate.sh:83). From the bundle root:

    python3 tools/mujoco_sweep.py --out <dir> \
      --arm no_dr_s8600=policies/no_dr_s8600_g1.onnx \
      --arm deploy_dr_s8600=policies/deploy_dr_s8600_g1.onnx \
      --clip clips/walk_arc_cw_stop_001__A047.pkl \
      --channels delay --lams 0 0.5 1.0 1.5 2.0 3.0 --seeds 16

The `--arm NAME=PATH` spellings are what keep the row labels below: the arm key
*is* the row label -- `row()` in tools/mujoco_sweep.py returns
`f"| {LABEL.get(arm, arm)} | "` -- while evaluate.sh:63 instead derives the
shorter keys `no_dr` / `deploy_dr` by stripping `_s8600_g1.onnx` off the policy
filenames. Same policies, same numbers, different row names.

The plant here is `runner/g1/g1_29dof.xml` via `tools/mujoco_player.py`, not the
43-DoF Dex3 model `bash drill.sh --latency` drives -- 1.05 kg apart, measured in
`sim/VENDOR_PATCHES.md` section 3 (`base_sim.py` — `GEAR_SONIC_ROOT`). The two
latency numbers are not comparable.

### Scored criterion (pelvis-to-reference > 0.5 m ends the rollout)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 | λ 3 |
|---|---|---|---|---|---|---|
| no_dr_s8600 | 0/16 (0%, stop 5.5s) | 0/16 (0%, stop 6.1s) | 2/16 (12%, stop 5.2s) | 1/16 (6%, stop 4.2s) | 0/16 (0%, stop 3.9s) | 1/16 (6%, stop 2.8s) |
| deploy_dr_s8600 | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) |

### What actually happened (fell over vs drifted off the path while upright)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 | λ 3 |
|---|---|---|---|---|---|---|
| no_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 14 · tracked 2 | fell 0 · drifted 15 · tracked 1 | fell 0 · drifted 16 · tracked 0 | fell 4 · drifted 11 · tracked 1 |
| deploy_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 |
