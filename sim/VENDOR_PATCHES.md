# What was changed in the vendored simulator

`sim/gear_sonic_sim/` is `gear_sonic/utils/mujoco_sim/` from
GR00T-WholeBodyControl, plus `gear_sonic/utils/network/network_utils.py`, copied
verbatim and then changed in the four places below. `sdk/unitree_sdk2py/` is
copied verbatim with **no** changes.

Regenerate the diff against the upstream tree with:

    diff -u <repo>/gear_sonic/utils/mujoco_sim/base_sim.py sim/gear_sonic_sim/base_sim.py

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

## 4. `base_sim.py` — `self.elastic_band` initialisation  (a real bug)

    self.elastic_band = None          # added before the enabling branch

`sim_step()` and `handle_keyboard_button()` both read `self.elastic_band`
unconditionally, but it was only ever assigned inside
`if config["ENABLE_ELASTIC_BAND"] and self.use_floating_root_link:`. The vendor's
own config sets `ENABLE_ELASTIC_BAND: True`, so upstream never reaches the else.
Running with the band off raises `AttributeError` on the first physics step.

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
