# What was changed in the vendored simulator

`sim/gear_sonic_sim/` is `gear_sonic/utils/mujoco_sim/` from
GR00T-WholeBodyControl, plus `gear_sonic/utils/network/network_utils.py`, copied
verbatim and then changed in the six places below — sections 1 to 6, of which
four are in `base_sim.py`. `sdk/unitree_sdk2py/` is copied verbatim with **no**
changes.

Regenerate the diff against the upstream tree with:

    U=<repo>/gear_sonic
    diff -ru "$U/utils/mujoco_sim" sim/gear_sonic_sim -x __pycache__ -x network_utils.py
    diff -u  "$U/utils/network/network_utils.py"  sim/gear_sonic_sim/network_utils.py
    diff -u  "$U/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml" \
             sim/wbc_configs/g1_29dof_sonic_model12.yaml

This file used to name `base_sim.py` alone, which misses most of what changed:
section 1 is a sweep over every file in the package, section 2 is in
`configs.py`, and the `ROBOT_SCENE` line in section 3 is in a yaml that no
longer sits inside the package. (Written from the layout sections 1–3 describe —
the upstream tree is not in this bundle, so these commands were not run here.)
Section 5 is also visible without upstream, as
`git show 823e404 -- sim/gear_sonic_sim/base_sim.py`.

## 1. Package name (all files)

    from gear_sonic.utils.mujoco_sim.X   ->  from gear_sonic_sim.X
    from gear_sonic.utils.network.network_utils  ->  from gear_sonic_sim.network_utils

Mechanical. The bundle does not carry `gear_sonic`, only the simulator.

## 2. `configs.py` — where the wbc configs live

`load_wbc_yaml()` did `import gear_sonic` and resolved
`gear_sonic/utils/mujoco_sim/wbc_configs`. In the bundle they are at
`sim/wbc_configs/`, beside the package.

## 3. `base_sim.py` — `GEAR_SONIC_ROOT`

Four `.parent`s became two. `ROBOT_SCENE` is resolved against this root, and the
models are at `sim/models/`, so the root is `sim/`. The vendored
`wbc_configs/g1_29dof_sonic_model12.yaml` has its `ROBOT_SCENE` repointed to
`models/g1/scene_43dof.xml` to match — the model file itself is unmodified.

That one line also decides **which G1 the DDS rehearsal drives, and it is not
the one the ONNX ladder drives.** Loaded through MuJoCo and measured here:

| | `sim/models/g1/scene_43dof.xml` (drill.sh) | `runner/g1/g1_29dof.xml` (evaluate.sh) |
|---|---|---|
| bodies / joints / actuators | 45 / 44 / 43 | 31 / 30 / 29 |
| hand joints | 14 (Dex3, 7 per side) | 0 |
| robot mass | 36.165 kg | 35.112 kg |

`scene_43dof.xml:2` includes `g1_29dof_with_hand.xml`, while
`tools/mujoco_player.py:43-49` resolves `runner/g1/g1_29dof.xml`, whose hands are
one rigid `*_rubber_hand` mesh per arm. What bounds the difference: the runner
commands the same 29 body joints in both, and the 14 Dex3 DoF are never
commanded, so what diverges is the 1.05 kg of extra hand mass and inertia and the
hand collision geometry — not the control. Do not read a `drill.sh --latency`
number and an `evaluate.sh` latency number as the same plant.

## 4. `base_sim.py` — `self.elastic_band` initialisation  (a real bug)

    self.elastic_band = None          # added before the enabling branch

`sim_step()` and `handle_keyboard_button()` both read `self.elastic_band`
unconditionally, but it was only ever assigned inside
`if config["ENABLE_ELASTIC_BAND"] and self.use_floating_root_link:`. The vendor's
own config sets `ENABLE_ELASTIC_BAND: True`, so upstream never reaches the else.
Running with the band off raises `AttributeError` on the first physics step.

## 5. `base_sim.py` — actuation latency injection  (added capability)

Added by commit 823e404 so the DDS rehearsal can be run at a latency, the way
the ONNX ladder in `evaluate.sh` can. Running

    git log --oneline -- sim/gear_sonic_sim/base_sim.py

returns exactly two commits — a30501f (the import) and 823e404 — so that second
commit's diff *is* this section. Six touch points:

* `import collections` (:19).
* `DefaultEnv.__init__` (:53–61) — `actuation_delay_steps`, `actuation_delay_ms`,
  and `_cmd_delay_buf`, one `LowCmd` snapshot per PHYSICS step.
* `set_actuation_delay_ms()` (:278–288) — rounds ms to whole physics steps and
  resizes the deque, keeping whatever history still fits so a change mid-run does
  not snap the robot with a stale or empty command.
* `viewer_key_callback()` (:290–307) — `=` +5 ms, `-` −5 ms, `0` reset; anything
  else falls through to the vendor's `ElasticBand.MujuocoKeyCallback` (7/8/9).
* the callback is attached on **both** viewer branches (:216–222 and :228–232).
  Upstream passed `key_callback=self.elastic_band.MujuocoKeyCallback` on the
  band branch and **no** `key_callback` at all on the other, so with
  `ENABLE_ELASTIC_BAND: False` the viewer had no keyboard.
* `compute_body_torques()` (:309–331) — reads the whole `LowCmd` in ONE pass into
  an array, pushes it, and computes torque from `_cmd_delay_buf[0]`, the oldest
  entry. At `maxlen == 1` that is the snapshot just pushed, so zero latency is
  the upstream arithmetic unchanged. The single-pass read also makes the command
  atomic against the DDS callback thread, which the previous field-by-field read
  was not.

Driven by `sim/run_robot_sim.py --latency-ms` and `drill.sh --latency`.

## 6. `base_sim.py` — `BaseSimulator` builds only the `default` env

    # base_sim.py:600-603
    raise ValueError(
        f"Invalid environment name: {env_name}. "
        f"Only 'default' is supported in this minimal build."
    )

`DefaultEnv` is the only env class in the package — `grep -n '^class'
sim/gear_sonic_sim/*.py` lists thirteen classes and `DefaultEnv` is the only one
an `env_name` could name (the rest are the bridge, the factory, the configs and
the sensor/image helpers) — so the dispatch has nothing else to pick. This came
in with the import commit a30501f, not with 823e404. Which env classes upstream offered is not recorded here — the upstream
tree is not in the bundle — so the diff at the top is the way to see it.

## Not patched — handled in the launcher instead

These are worked around in `sim/run_robot_sim.py`, leaving the vendor code alone:

* **Double DDS init.** `SimWrapper` in the vendor's `run_sim_loop.py` calls
  `init_channel()` and then `BaseSimulator.__init__` calls
  `ChannelFactoryInitialize` again. `ChannelFactory` is a singleton, so the
  second call fails and prints `create domain error`. The launcher calls neither
  and lets `BaseSimulator` do it once.
* **`ElasticBand.point` defaults to `[0, 0, 1]`.** The G1's `torso_link` stands
  at z = 0.847, so the default anchor pulls up with `kp_pos × 0.153 ≈ 1500 N`,
  about four times body weight. The launcher re-anchors the band at the attached
  link's actual standing position.
* **`check_fall()` resets the simulation** when the pelvis drops below 0.2 m.
  For an emergency-stop rehearsal that erases the thing being observed.
  `--keep-fallen` replaces it with a report.
* **`BaseSimulator.start()`** is an unbounded loop with no status output and no
  duration bound. The launcher runs the same body — same order, same rate
  limiter — with a status line, phase detection and `--duration`.
