# Latest four-motion policies

The September 17 SONIC training run produced **two policies, each trained on all
four motions**: walking, turning, crouch hold, and side-stepping. Both are seed
8600, iteration 4000. `fixed_dr` used the standard six-channel fixed DR setting;
`no_dr` used zero DR. These are the latest four-motion video models.

The weights and references are included in Git. Clone the repository normally;
no Git LFS, training repository, dataset download, or ZIP installer is required.

## Install on a new machine

The tested deployment target is Ubuntu 22.04, x86_64, an NVIDIA GPU, and the
29-DoF G1 model. Run setup in a terminal where you can answer its sudo prompt.
For the separate Orin path, see [ETHERNET_AND_SDK.md](ETHERNET_AND_SDK.md).

```bash
git clone https://github.com/linjiw/lucid-g1-deploy.git
cd lucid-g1-deploy
bash setup.sh
source env.sh
bash build.sh
bash four_motion.sh check
bash test.sh --quick
"$PYTHON" -m unittest discover -s tools -p test_four_motion.py
"$PYTHON" -m wandb login
```

`setup.sh` installs video-export and W&B dependencies as well as the deployment
toolchain. `build.sh` compiles the C++ runner. TensorRT engines are generated on
the target machine; copying `.trt` files or Python environments is unnecessary.
`four_motion.sh check` verifies both model hashes, ONNX signatures, observation
configuration, reference-file hashes, CSV shapes, and every source joint
position/velocity after conversion into IsaacLab order.

`play`, `rehearse`, and `run` log online to **`16726/lucid-sonic`**. Use an
account with access to that W&B project. Authentication is stored by W&B outside
this repository. `list` and `check` do not require W&B authentication. A logging
or verification failure exits with an error; it is not silently treated as an
online result.

## Choose a policy and motion

| Launcher policy | ONNX file | Training |
|---|---|---|
| `fixed_dr` | `policies/four_motion_fixed_dr_s8600_g1.onnx` | Fixed DR, seed 8600, iteration 4000 |
| `no_dr` | `policies/four_motion_no_dr_s8600_g1.onnx` | No DR, seed 8600, iteration 4000 |

| `--motion` | Reference directory | Frames at 50 Hz |
|---|---|---:|
| `walking` | `motions/four_motion_walking/` | 484 |
| `turning` | `motions/four_motion_turning/` | 432 |
| `crouch_hold` | `motions/four_motion_crouch_hold/` | 406 |
| `side_stepping` | `motions/four_motion_side_stepping/` | 422 |

Crouch hold is a sustained crouch, not repeated squatting. Source clips are in
`clips/four_motion_<motion>.pkl`; the runner consumes the corresponding six CSV
files and `metadata.txt`. [The manifest](../config/four_motion_20260917.json)
records model/checkpoint hashes, source training commit, reference hashes, and
conversion provenance. Each model accepts `obs_dict [1,1570]` and returns
`action [1,29]`.

The older `policies/deploy_dr_s8600_g1.onnx` and `policies/no_dr_s8600_g1.onnx`
remain the **three-motion, iteration-8000** models. In the new launcher,
`--policy no_dr` selects `four_motion_no_dr_s8600_g1.onnx`; in the general
`run.sh` or `drill.sh`, specify **`--policy four_motion_no_dr`** instead.

## Record reference-initialized MuJoCo playback

```bash
bash four_motion.sh play --policy fixed_dr --motion walking
bash four_motion.sh play --policy no_dr --motion walking
bash four_motion.sh play --policy fixed_dr --motion turning --lam 1 --seed 8700
```

`play` starts the simulated robot at the reference pose, runs the full clip,
and writes `playback.mp4`, `playback.json`, `metrics.json`, `launch.json`, and an
online-verified `receipt.json`. Outputs go under `results/four_motion/` with a
unique timestamp. Use `--out results/my-new-run` to choose a new empty directory.

`--lam 0` is nominal physics. `--lam 1` requests the player's approximation to
the training DR range: friction, mass, CoM, joint-default bias, velocity-change
pushes, and a shared episode actuation delay. This is not identical to Isaac
training's independent actuator-group delays. Seeded paired comparisons should
use the same `--lam` and `--seed` for both policies. Direct playback bypasses
DDS and the C++ stand-to-policy transition.

## Rehearse the actual deployment sequence in MuJoCo

```bash
bash four_motion.sh rehearse --policy fixed_dr --motion crouch_hold
bash four_motion.sh rehearse --policy no_dr --motion crouch_hold
bash four_motion.sh rehearse --policy fixed_dr --motion turning --record
```

This runs the actual C++/TensorRT runner against the MuJoCo robot over DDS on
loopback (`lo`). It loads the selected model and exactly one reference, waits
for INIT, holds the fixed stand, sends `]`, confirms CONTROL, sends `T` to play
the reference, and sends `O` after the clip. Initialization support is released
when control starts. Automatic fall resets are disabled. Rehearsals use nominal
physics; `--lam` is available only in `play`.

The output includes runner/simulator logs, observations, target-reference
traces, action CSVs, TensorRT-versus-ONNX parity, and phase/height metrics.
`--record` also writes `deployment_sequence.mp4`; `--viewer` shows the live
window when a desktop display is available. Playback completion, tracking,
and staying upright are separate measurements. A zero process exit code alone
does not establish successful motion tracking.

Run one DDS rehearsal at a time: simultaneous runners on `lo` would share the
robot's DDS topics. To repeat all eight combinations sequentially:

```bash
for policy in fixed_dr no_dr; do
  for motion in walking turning crouch_hold side_stepping; do
    bash four_motion.sh rehearse --policy "$policy" --motion "$motion" || break 2
  done
done
```

## Interactive simulation and the hardware process

```bash
bash four_motion.sh run --policy fixed_dr --motion crouch_hold --viewer
```

Wait for **Init Done**. `]` starts policy control; **`T` starts reference
playback**; `O` stops. Stopping removes stiffness, so collapse after `O` is
reported separately from a failure during policy playback. Restart the process
to return through INIT. The new launcher always selects simulation and `lo`.
The underlying `run.sh` also records its own run files under `results/run/`.

For a physical G1, follow [DEPLOY_DAY.md](DEPLOY_DAY.md) and
[DEPLOY_SEQUENCE.md](DEPLOY_SEQUENCE.md), including their support and operator
checks. The corresponding model/motion selectors for that documented process
are `--policy four_motion_fixed_dr` (or `four_motion_no_dr`) and
`--motion four_motion_crouch_hold` (or one of the other aliases prefixed with
`four_motion_`). The simulation launcher does not connect to hardware. These
new policies have not been armed on a physical robot.

## Validation and limitations

The September 18 publication check regenerated all four joint-position and
joint-velocity references using the upstream IsaacLab-order correction
(`04c56d2`). It checks every value against the source clips, not only file
shape. Five regression tests cover joint order, explicit playback selection,
and separating post-stop collapse from low height during playback.

**The earlier September 17 DDS results are superseded.** Those CSVs used
MuJoCo joint order, so their successful model loading, balance, and engine
parity did not establish playback of the intended reference. The old ZIP's
CSV references and DDS readiness table should not be used; use this Git
version. The ONNX weights and source clips are unchanged.

Corrected-reference rehearsal results are recorded in
[validation/four_motion_20260918.json](validation/four_motion_20260918.json),
including each online W&B run URL and source/configuration hashes.

The corrected-reference run completed the full deployment workflow and observed
every reference frame in **8/8** cases. **8/8** stayed above the defined
low-pelvis failure criterion during playback. Maximum TensorRT-versus-ONNX
action difference was **5.72e-06**. The runner's logged reference values also
matched every intended frame within **6e-6**, the CSV logging precision.

| Policy | Motion | Reference frames | Minimum playback pelvis height | Low-height check |
|---|---|---:|---:|---|
| fixed_dr | walking | 484 | 0.742 m | pass |
| fixed_dr | turning | 432 | 0.731 m | pass |
| fixed_dr | crouch_hold | 406 | 0.629 m | pass |
| fixed_dr | side_stepping | 422 | 0.740 m | pass |
| no_dr | walking | 484 | 0.753 m | pass |
| no_dr | turning | 432 | 0.744 m | pass |
| no_dr | crouch_hold | 406 | 0.625 m | pass |
| no_dr | side_stepping | 422 | 0.751 m | pass |

Reference-initialized direct playback is unaffected by the CSV correction.
The earlier eight nominal direct playbacks completed without their configured
low-pelvis failure. Walking, turning, and side-stepping nevertheless crossed the
0.5 m root-distance threshold after 1.12–2.48 s. Only crouch hold stayed within
that threshold. The approximate DR checks passed the height criterion in 12/12
fixed-DR rollouts and 8/12 no-DR rollouts (three seeds per motion). These small
counts are descriptive, not a general success rate. The individual direct
results and W&B links are in
[validation/four_motion_direct_20260917.csv](validation/four_motion_direct_20260917.csv).

Direct low height means five consecutive 50 Hz samples below 60% of reference
pelvis height. DDS low height means five consecutive 10 Hz wall-clock samples
below 0.35 m during policy playback. Neither criterion measures accurate path
tracking. The direct runtime was Python 3.13.15, MuJoCo 3.13.0, ONNX Runtime
1.29.0; DDS used Python 3.10.12, MuJoCo 3.13.0 and TensorRT 10.13.3 on RTX 5080.
