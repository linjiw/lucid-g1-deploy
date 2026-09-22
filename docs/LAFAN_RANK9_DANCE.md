# New LAFAN dance motion and fixed-DR policy

This release adds catalog rank **#9**, `dance2_subject5` source seconds **20–30**,
and its SONIC policy trained from scratch using **4096 environments**, **1000
PPO learning iterations**, seed 8600, fixed lambda=1 over six standard DR
channels (including 0–40 ms actuator delay). Checkpoints were saved at 500 and 1000;
the bundled ONNX is the final 1000-iteration model. Training completion does not
establish convergence.

## Included files

- `policies/lafan_rank9_ne4096_h1000_fixed_s8600_g1.onnx`
- `clips/lafan_rank9_dance2_subject5_20to30.pkl`
- `motions/lafan_rank9_dance2_subject5_20to30/`: all six runner CSVs and metadata
- `config/lafan_rank9_20260922.json`: source/checkpoint/model/motion hashes and
  simulation results, including online W&B URLs

The source contains 301 samples at 30 Hz spanning 10 seconds. The runner export has
501 samples at 50 Hz, including both endpoints. Joint positions/velocities are
in **IsaacLab joint order**; body quaternions are **WXYZ**.

## Verify and play

After the repository's normal setup:

```bash
source env.sh
"$PYTHON" tools/verify_deploy_bundle.py . --json
"$PYTHON" -m wandb login
"$PYTHON" tools/lafan_rank9.py --lam 0 --seed 8700
"$PYTHON" tools/lafan_rank9.py --lam 1 --seed 8700
```

Playback writes a full-clip video, trajectory and receipt into a timestamped
`results/lafan_rank9_*` directory and logs online to `16726/lucid-sonic`.
`--out <directory>` selects another output directory; `--no-video` skips rendering.
It starts on the reference pose **with reference velocities**, not from a
stationary stand. Videos continue after path drift so final posture is visible.

For a subsequent C++/TensorRT/DDS **simulation** rehearsal, the installed names are:

```bash
bash run.sh --sim \
  --policy lafan_rank9_ne4096_h1000_fixed \
  --motion lafan_rank9_dance2_subject5_20to30
```

That rehearsal command is supplied for future validation; it was not run as part
of this release. Follow the existing operator procedure; direct MuJoCo playback
is not evidence for the C++ deployment handover or real hardware.

## Measured verification and limits

Passed ONNX full graph checking, the `obs_dict [1,1570] -> action [1,29]`
signature, 20 deterministic finite-inference probes, bundle parser verification,
and all 501 joint position/velocity CSV frames compared against source resampling
and the required ordering (absolute tolerance 5.01e-7).

| Physics | Evaluation seed | First 0.5 m path error | Upright at clip end |
|---|---:|---:|---|
| nominal |8700|3.36s|yes|
| randomized lambda=1 |8700|2.32s|yes|
| randomized lambda=1 |8701|2.68s|yes|
| randomized lambda=1 |8702|3.16s|yes|

[Watch the nominal policy rollout](media/lafan_rank9_nominal.mp4).

**No rollout passed the complete-clip path criterion.** Upright is a final
pelvis-height proxy, not a contact-based fall diagnosis. The player's legacy
`fell` flag means crossing the path threshold; it must not be read as a literal
fall. Randomized MuJoCo parameters approximate the training envelope; this is
one training seed and three randomized evaluation seeds, not a robustness
certification. Checkpoint-to-ONNX numerical parity, this policy's TensorRT/DDS
execution, stationary-stand handover and hardware tracking remain unverified.
The reference's initial standing joint mismatch is approximately 0.327 rad RMS.

[Training run](https://wandb.ai/16726/lucid-sonic/runs/lafan_rank9_fixed_ne4096_h1000_20260922-train-fixed).
Evaluation URLs and metric hashes are in the release JSON; all four online runs
were verified finished. The checkpoint is retained in the research artifact
store; only the deployment ONNX and selected motion are bundled here.

## Motion provenance

G1-retargeted LAFAN1, `lvhaidong/LAFAN1_Retargeting_Dataset`, revision
`ce1572906efe6157840e8474d5a0d7aa87481e74`, `g1/dance2_subject5.csv`, frames 600–900
inclusive. No retiming, synthetic transition, or coordinate modification was
applied. See the upstream [retargeting dataset](https://huggingface.co/datasets/lvhaidong/LAFAN1_Retargeting_Dataset)
and [LAFAN1 source](https://github.com/ubisoft/ubisoft-laforge-animation-dataset).
The upstream dataset card identifies the source motion-data license as
CC BY-NC-ND 4.0; this repository's code license does not replace that attribution.
