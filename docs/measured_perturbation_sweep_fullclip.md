Perturbation sweep (all six randomization channels scaled together by λ),
**full-clip window**: every rollout runs to the end of the 8.6 s motion, which
is why every `stop` cell below reads 8.6s. The 0.5 m criterion still marks a run
failed, it just no longer ends it (tools/mujoco_sweep.py `--full-clip`).
`docs/RESULTS.md` quotes this file's fall counts;
`docs/measured_perturbation_sweep.md` is the scored window, where `no_dr` at
λ 1.5 reports 3 falls of 16 against the 11 below.

Reproduce with `bash evaluate.sh` and read `results/perturbation/summary.md`
(evaluate.sh:77 already passes `--full-clip`), or directly from the bundle root:

    python3 tools/mujoco_sweep.py --out <dir> \
      --arm no_dr_s8600=policies/no_dr_s8600_g1.onnx \
      --arm deploy_dr_s8600=policies/deploy_dr_s8600_g1.onnx \
      --clip clips/walk_arc_cw_stop_001__A047.pkl \
      --lams 0 0.5 1.0 1.5 2.0 --seeds 16 --full-clip

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

The heading below was corrected by hand: the `mujoco_sweep.py` that produced
this table printed `### Scored criterion ...` whatever the window, which is how
this file came to be headed for a criterion that ends nothing in it. That is
fixed -- `main()` now picks the heading off `a.full_clip` (the
`### Full-clip window ...` / `### Scored criterion ...` ternary in
tools/mujoco_sweep.py) and takes the clip length from the receipts' own
`duration` field, so another clip reports its own length. Formatting that
f-string against `results/eval16/perturbation/sweep.json` (`duration` 8.62, so
the length renders `8.6 s`) reproduces the heading below byte for byte; the
sweep itself was not re-run.

### Full-clip window (rollout runs to the end of the 8.6 s motion; pelvis-to-reference > 0.5 m still counts as a failure)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 |
|---|---|---|---|---|---|
| no_dr_s8600 | 0/16 (0%, stop 8.6s) | 3/16 (19%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) |
| deploy_dr_s8600 | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) |

### What actually happened (fell over vs drifted off the path while upright)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 |
|---|---|---|---|---|---|
| no_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 13 · tracked 3 | fell 7 · drifted 9 · tracked 0 | fell 11 · drifted 5 · tracked 0 | fell 14 · drifted 2 · tracked 0 |
| deploy_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 3 · drifted 13 · tracked 0 | fell 5 · drifted 11 · tracked 0 |
