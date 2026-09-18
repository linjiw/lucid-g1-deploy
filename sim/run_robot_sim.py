#!/usr/bin/env python3
"""Stand in for a Unitree G1 on the DDS bus, using MuJoCo for the physics.

This is not a policy evaluator. It is the ROBOT side of the deployment: it
speaks the same DDS topics, with the same IDL, that the real G1 speaks, so the
C++ runner (`runner/target/release/g1_deploy_onnx_ref`) connects to it with no
change of any kind -- same binary, same arguments, same startup sequence. The
only difference between this and hardware is which end of the wire the physics
comes from.

    rt/lowcmd        <- the runner   29 motor commands, each q dq kp kd tau
    rt/lowstate      -> the runner   29 motor states (q dq ddq tau_est),
                                     IMU quaternion (w x y z), gyro, accel
    rt/secondary_imu -> the runner   torso IMU
    rt/odostate      -> the runner   ground-truth base pose and velocity
    rt/dex3/*/cmd    <- the runner   Dex3 hand commands, 7 per side
    rt/dex3/*/state  -> the runner   Dex3 hand states

Torque is computed exactly as the motor controller computes it, from the fields
in the LowCmd the runner actually sent:

    tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq)          (clipped)

which is why an emergency stop is visible here: the runner's stop path writes
kp = 0, kd = 8, q = 0, tau = 0, and this simulator applies exactly that.

USAGE  -- after `source env.sh`, and NOT under `python3`:

    "$LUCID_SIM_PYTHON" sim/run_robot_sim.py               # loopback, viewer if a display
    "$LUCID_SIM_PYTHON" sim/run_robot_sim.py --headless    # no viewer (servers, CI, drill)
    "$LUCID_SIM_PYTHON" sim/run_robot_sim.py --iface eth0  # a real NIC, to another host
    "$LUCID_SIM_PYTHON" sim/run_robot_sim.py --band        # elastic band: hold the robot up

`python3` is the wrong interpreter and fails before the model loads. env.sh puts
.venv/bin first on PATH, and .venv carries no cyclonedds; only .venv-sim does
(cyclonedds 0.10.2, which unitree_sdk2py pins and which does not import on 3.13
-- checked: `ls .venv/lib/python3.10/site-packages | grep -i cyclone` is empty,
.venv-sim has cyclonedds-0.10.2). The import chain here is run_robot_sim ->
gear_sonic_sim.base_sim -> unitree_sdk2py.core.channel -> cyclonedds, so under
.venv it raises ModuleNotFoundError. env.sh exports LUCID_SIM_PYTHON and the
PYTHONPATH (sim/ and sdk/) that makes the vendored SDK and simulator importable
at all. Every launcher in the bundle does the same -- drill.sh, run.sh and
test.sh each set SIMPY to .venv-sim/bin/python.

By default a fall RESETS the simulation, which is the vendor behaviour and is
wrong for watching an emergency stop: the robot would snap back upright at the
moment you wanted to see it go down. --keep-fallen leaves it where it lands.

The simulator carries no policy and no reference motion. Start it first, then
start the runner against the same interface.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import sys
import time

HERE = Path(__file__).resolve().parent
BUNDLE = HERE.parent

# The bundle is self-contained: the vendored Unitree python SDK lives in sdk/,
# the vendored simulator in sim/gear_sonic_sim/. Neither is pip-installed.
sys.path.insert(0, str(BUNDLE / "sdk"))
sys.path.insert(0, str(HERE))


class _Recorder:
    """Offscreen video of the run, piped straight into ffmpeg.

    Frames go to ffmpeg's stdin as raw RGB rather than being accumulated, so a
    long run costs no memory. The camera tracks the pelvis, which matters here:
    the robot walks away from where it started, and falls over at the end.
    """

    def __init__(self, env, path, width, height, fps, sim_dt):
        import subprocess
        import mujoco
        self._mujoco = mujoco
        self.env = env
        self.every = max(1, int(round(1.0 / (fps * sim_dt))))
        self.renderer = mujoco.Renderer(env.mj_model, height=height, width=width)
        self.cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.cam)
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 135.0, -12.0, 3.4
        self.proc = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error",
             "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{width}x{height}", "-r", f"{fps:g}", "-i", "-",
             "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p",
             "-crf", "20", "-preset", "medium", path],
            stdin=subprocess.PIPE)
        self.frames = 0

    def maybe_capture(self, step):
        if step % self.every:
            return
        d = self.env.mj_data
        # Follow the pelvis in x and y, but hold the height: tracking z as well
        # makes the camera dive at the moment the robot goes down, which is the
        # one moment a viewer needs a stable frame of reference.
        self.cam.lookat[0] = d.qpos[0]
        self.cam.lookat[1] = d.qpos[1]
        self.cam.lookat[2] = 0.75
        self.renderer.update_scene(d, camera=self.cam)
        try:
            self.proc.stdin.write(self.renderer.render().tobytes())
            self.frames += 1
        except BrokenPipeError:
            pass

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=60)
        except Exception:
            pass
        try:
            self.renderer.close()
        except Exception:
            pass
        print(f"  [sim] wrote {self.frames} frames", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Run a MuJoCo G1 that speaks the robot's own DDS protocol.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--iface", default="lo",
                    help="network interface for DDS. 'lo' for same-machine, or the "
                         "NIC on the 192.168.123.x robot network. Default: lo")
    ap.add_argument("--domain", type=int, default=None,
                    help="DDS domain id (default: whatever the wbc config says, 0)")
    ap.add_argument("--headless", action="store_true",
                    help="no MuJoCo viewer window")
    ap.add_argument("--band", action="store_true",
                    help="enable the elastic band that holds the robot up. The "
                         "vendor's own wbc config ships with this ON, because the "
                         "runner's pre-policy phases cannot hold a free stand "
                         "(see --help notes and docs/DEPLOY_SEQUENCE.md)")
    ap.add_argument("--hold-band-through-policy", action="store_true",
                    help="keep the band on once the policy starts. Default is to "
                         "release it the moment the commanded joint targets start "
                         "moving, so the policy balances on its own.")
    ap.add_argument("--keep-fallen", action="store_true",
                    help="do NOT reset the sim when the robot falls (watch the fall)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many seconds (0 = run until interrupted)")
    ap.add_argument("--record", metavar="PATH",
                    help="write an mp4 of the run. With --headless it sets "
                         "MUJOCO_GL=egl for itself and renders offscreen with no "
                         "display; with the viewer on it makes its own offscreen "
                         "context on the default GLFW backend, which needs a "
                         "display. Needs ffmpeg on PATH.")
    ap.add_argument("--record-hz", type=float, default=30.0)
    ap.add_argument("--record-size", default="1280x720")
    ap.add_argument("--status-hz", type=float, default=1.0,
                    help="how often to print a status line (0 to silence)")
    ap.add_argument("--latency-ms", type=float, default=0.0,
                    help="actuation latency in ms, applied to every LowCmd before "
                         "it becomes torque. Rounded to whole physics steps (5 ms). "
                         "Adjust live in the viewer with '=' / '-' / '0'.")
    ap.add_argument("--no-hold", action="store_true",
                    help="do NOT hold the robot upright before the first LowCmd; "
                         "let it collapse under zero torque (see --help notes)")
    args = ap.parse_args()

    # Both guards run HERE, before the imports below: base_sim imports mujoco at
    # module scope, and both failures are otherwise raised after the model is
    # loaded and the DDS domain is taken -- i.e. with a robot already publishing
    # on the bus.
    #
    # MUJOCO_GL is read exactly once, at `import mujoco`: mujoco/__init__.py:76
    # pulls in rendering/classic/gl_context.py, which reads it at :24 and, when it
    # is empty, runs the whole backend dispatch (:37-48) down to its final
    # `else:` / `from mujoco.glfw import GLContext as _GLContext` (:46-48). There
    # is no auto-fallback to EGL: the ONLY branch in that dispatch that reaches
    # mujoco.egl is `_MUJOCO_GL == 'egl'` (:40-42), and an empty value falls to
    # GLFW. So headless --record without this dies inside mujoco.Renderer with no
    # display.
    # Only when headless: forcing EGL with the viewer on would break
    # mujoco.viewer.launch_passive (base_sim.py:216-232), which needs GLFW.
    # setdefault, so an explicit MUJOCO_GL=osmesa from the caller still wins --
    # tools/build_demo_video.py's recipe still sets it by hand ahead of drill.sh
    # (`MUJOCO_GL=egl bash drill.sh --hold 14 --record ...`, its module docstring).
    if args.record and args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    # _Recorder pipes raw frames into ffmpeg through an unguarded Popen in
    # _Recorder.__init__, so a missing ffmpeg surfaces as a bare FileNotFoundError
    # naming neither the binary's purpose nor how to get it. setup.sh does install
    # it (an `ffmpeg` entry in the `sudo apt-get install -y` list of its
    # build-tools step) and does report it (its `chk ffmpeg` line), so
    # this guard only fires on a box where that step was skipped or refused --
    # which is exactly when the bare traceback is least readable. Same check as
    # tools/build_demo_video.py (`if not shutil.which("ffmpeg")`, in main() just
    # after the font check); the message here adds the install hint.
    if args.record and not shutil.which("ffmpeg"):
        raise SystemExit("ffmpeg not found  (--record pipes frames into it; "
                         "apt install ffmpeg)")

    from gear_sonic_sim.configs import SimLoopConfig
    from gear_sonic_sim.simulator_factory import SimulatorFactory
    import gear_sonic_sim.base_sim as base_sim
    import mujoco

    cfg = SimLoopConfig()
    wbc = cfg.load_wbc_yaml()
    wbc["ENV_NAME"] = "default"
    wbc["INTERFACE"] = args.iface
    wbc["ENABLE_ONSCREEN"] = not args.headless
    wbc["ENABLE_OFFSCREEN"] = False
    wbc["ENABLE_ELASTIC_BAND"] = bool(args.band)
    wbc["USE_JOYSTICK"] = 0
    if args.domain is not None:
        wbc["DOMAIN_ID"] = args.domain

    if args.record:
        # MuJoCo bakes the offscreen framebuffer size at compile time and the
        # vendored scene takes the 640x480 default, which is too small for an HD
        # recording. Write a variant of the scene with a bigger <global
        # offwidth/offheight> NEXT TO the original -- same directory, so meshdir
        # and <include> still resolve -- and load that.
        #
        # It has to be a real file loaded through from_xml_path, not an MjSpec
        # compile: DefaultEnv._get_dof_indices_by_class() calls
        # mujoco.mj_saveLastXML(), which reads MuJoCo's last-parsed-XML global
        # and raises "No XML model loaded" for a spec-compiled model.
        _rec_w, _rec_h = (int(v) for v in args.record_size.split("x"))
        _orig_from_path = mujoco.MjModel.from_xml_path
        _tmp_scene = []

        def _from_path_bigger_fb(path, *a, **kw):
            import xml.etree.ElementTree as ET
            tree = ET.parse(path)
            root = tree.getroot()
            visual = root.find("visual") or ET.SubElement(root, "visual")
            if root.find("visual") is None:
                root.append(visual)
            g = visual.find("global")
            if g is None:
                g = ET.SubElement(visual, "global")
            g.set("offwidth", str(_rec_w))
            g.set("offheight", str(_rec_h))
            out = Path(path).with_name(f".record_{Path(path).name}")
            tree.write(out)
            _tmp_scene.append(out)
            return _orig_from_path(str(out), *a, **kw)

        mujoco.MjModel.from_xml_path = _from_path_bigger_fb

    scene = BUNDLE / "sim" / wbc["ROBOT_SCENE"]
    if not scene.is_file():
        print(f"model not found: {scene}", file=sys.stderr)
        return 1

    if args.keep_fallen:
        # The vendor DefaultEnv.check_fall() calls reset() as soon as the pelvis
        # drops below 0.2 m. For an emergency-stop drill that is exactly wrong:
        # the point is to see where the robot ends up. Report, do not reset.
        def _report_only(self):
            fallen = self.mj_data.qpos[2] < 0.2
            if fallen and not getattr(self, "_announced_fall", False):
                print(f"  [sim] robot is down: pelvis at {self.mj_data.qpos[2]:.3f} m "
                      f"(not resetting; --keep-fallen)", flush=True)
                self._announced_fall = True
            self.fall = fallen
        base_sim.DefaultEnv.check_fall = _report_only

    print("=" * 70)
    print("  MuJoCo G1 on the DDS bus -- the robot side of the deployment")
    print("=" * 70)
    print(f"  interface      {args.iface}"
          f"{'   (same machine only)' if args.iface == 'lo' else ''}")
    print(f"  DDS domain     {wbc['DOMAIN_ID']}")
    print(f"  model          {scene.relative_to(BUNDLE)}")
    print(f"  motors         {wbc['NUM_MOTORS']} body + {wbc.get('NUM_HAND_MOTORS', 0)} per hand")
    print(f"  physics step   {wbc['SIMULATE_DT'] * 1000:.0f} ms")
    print(f"  viewer         {'off' if args.headless else 'on'}"
          f"     elastic band {'on' if args.band else 'off'}"
          f"{'' if not args.band or args.hold_band_through_policy else ' (released at policy start)'}"
          f"     fall {'kept' if args.keep_fallen else 'resets sim'}")
    print()
    print("  subscribing  rt/lowcmd  rt/dex3/left/cmd  rt/dex3/right/cmd")
    print("  publishing   rt/lowstate  rt/secondary_imu  rt/odostate  rt/dex3/*/state")
    print()
    print("  torque applied per motor, from the LowCmd the runner sends:")
    print("      tau = tau_ff + kp*(q_des - q) + kd*(dq_des - dq),  clipped to the")
    print("      motor effort limit. kp and kd come from the command, not from here,")
    print("      so a stop that sets kp=0 kd=8 goes limp here as it would on the robot.")
    print()
    if args.no_hold:
        print("  --no-hold: before the first LowCmd the robot gets zero torque and")
        print("  collapses. The runner's INIT ramp then starts from the floor.")
    else:
        print("  Until the first LowCmd arrives the robot is HELD at its standing")
        print("  pose. That stands in for the gantry, harness or hands that must")
        print("  hold a real G1 during bring-up -- it is not a physics result.")
    print("=" * 70)
    # NOTE: do NOT call init_channel() here. BaseSimulator.__init__ calls
    # ChannelFactoryInitialize itself, and ChannelFactory is a singleton: a second
    # Init on the same domain id fails and prints "create domain error", which
    # looks alarming and is purely self-inflicted. The vendor's own
    # scripts/run_sim_loop.py has exactly this double-init.
    sim = SimulatorFactory.create_simulator(
        config=wbc, env_name="default",
        onscreen=wbc["ENABLE_ONSCREEN"], offscreen=False, enable_image_publish=False,
    )

    if args.record:
        mujoco.MjModel.from_xml_path = _orig_from_path
        for _f in _tmp_scene:
            _f.unlink(missing_ok=True)

    env = sim.sim_env
    bridge = sim.unitree_bridge

    applied = env.set_actuation_delay_ms(args.latency_ms)
    if args.latency_ms and abs(applied - args.latency_ms) > 1e-9:
        print(f"  latency        {applied:.0f} ms  (rounded from {args.latency_ms:.1f} to a "
              f"whole {wbc['SIMULATE_DT'] * 1000:.0f} ms physics step)")
    else:
        print(f"  latency        {applied:.0f} ms actuation delay"
              f"{'' if applied else '  (none)'}")
    if not args.headless:
        print("  viewer keys    '='  +5 ms latency    '-'  -5 ms    '0'  reset to 0")
        print("                 '7'/'8' band length   '9' band on/off")
    print()

    recorder = None
    if args.record:
        recorder = _Recorder(env, args.record, _rec_w, _rec_h, args.record_hz,
                             wbc["SIMULATE_DT"])
        print(f"  [sim] recording {_rec_w}x{_rec_h} @ {args.record_hz:g} fps "
              f"-> {args.record}", flush=True)

    if env.elastic_band is not None:
        # ElasticBand.point defaults to [0, 0, 1]. On a G1 the attached link
        # (torso_link) stands at z = 0.847, so the default anchor pulls upward with
        # kp_pos * 0.153 = ~1500 N, four times body weight -- it does not support
        # the robot, it launches it. Anchor the band where the robot actually is,
        # which is what a gantry strap does.
        env.elastic_band.point = env.mj_data.xpos[env.band_attached_link].copy()
        print(f"  [sim] elastic band anchored at "
              f"{env.elastic_band.point.round(3).tolist()} (the standing pose)",
              flush=True)
    t0 = time.monotonic()
    # Match the origin of the status-line clock. Printing this before simulator
    # construction shifts every phase by its startup time and can incorrectly
    # count the post-stop collapse as a policy failure.
    print(f"  [sim] EVENT epoch {time.time():.6f}", flush=True)
    last_status = 0.0
    seen_cmd = False
    policy_running = False
    stand_detected = False
    q_ref = None
    quiet_since = 0.0
    STAND_QUIET_S = 1.0

    try:
        # The vendor BaseSimulator.start() is an unbounded loop with no status
        # output, which is unusable for a scripted drill. Same body, same order,
        # same rate limiter -- plus a status line and a duration bound.
        sim_cnt = 0
        viewer_every = max(1, int(sim.viewer_dt / sim.sim_dt))
        reward_every = max(1, int(sim.reward_dt / sim.sim_dt))
        qpos0 = env.mj_data.qpos.copy()
        while sim._running and (env.viewer is None or env.viewer.is_running()):
            step_start = time.monotonic()
            if not seen_cmd and not args.no_hold:
                # Hold the standing pose until the controller takes over. On
                # hardware this is the harness or the operator; here it keeps the
                # INIT ramp from starting on a robot that is already on the floor,
                # which would make the whole drill meaningless.
                env.mj_data.qpos[:] = qpos0
                env.mj_data.qvel[:] = 0.0
                mujoco.mj_forward(env.mj_model, env.mj_data)
            env.sim_step()
            if sim_cnt % viewer_every == 0:
                env.update_viewer()
            if sim_cnt % reward_every == 0:
                env.update_reward()
            if recorder is not None:
                recorder.maybe_capture(sim_cnt)

            now = time.monotonic() - t0
            if not seen_cmd and bridge.cmd_received():
                seen_cmd = True
                print(f"  [sim] EVENT first_lowcmd t={now:.3f}s -- the runner is "
                      f"driving the motors", flush=True)

            # Read the runner's state machine off the wire, with no cross-process
            # signal. Three regimes, in order, distinguished only by how the
            # commanded joint targets behave:
            #
            #   INIT              targets interpolate towards default_angles, so
            #                     they move every step for about 3 s
            #   WAIT_FOR_CONTROL  targets are CONSTANT at default_angles
            #   CONTROL           targets move again, now driven by the policy
            #
            # Watching only for "movement" therefore fires during INIT. The
            # transition that matters is movement AFTER a quiet period, so the
            # detector waits for the stand first.
            if seen_cmd and not policy_running:
                q = [bridge.low_cmd.motor_cmd[i].q for i in range(29)]
                moved = q_ref is not None and max(abs(a - b) for a, b in zip(q, q_ref)) > 0.01
                if q_ref is None or moved:
                    q_ref, quiet_since = q, now
                elif not stand_detected and now - quiet_since > STAND_QUIET_S:
                    stand_detected = True
                    print(f"  [sim] EVENT fixed_stand t={now:.3f}s -- targets have "
                          f"been constant for {STAND_QUIET_S:.1f}s", flush=True)
                if stand_detected and moved:
                    policy_running = True
                    print(f"  [sim] EVENT policy_moving t={now:.3f}s -- targets are "
                          f"moving again; the policy has taken over", flush=True)
                    if env.elastic_band is not None and not args.hold_band_through_policy:
                        env.elastic_band.enable = False
                        print(f"  [sim] EVENT band_released t={now:.3f}s -- the "
                              f"policy is balancing unaided from here", flush=True)
            if args.status_hz > 0 and now - last_status >= 1.0 / args.status_hz:
                last_status = now
                got = seen_cmd
                kp0 = bridge.low_cmd.motor_cmd[0].kp if got else 0.0
                kd0 = bridge.low_cmd.motor_cmd[0].kd if got else 0.0
                # x and y matter as much as height. These policies carry no
                # horizontal-position term in their observation, so the failure
                # that actually shows up is walking off the reference path while
                # staying upright -- invisible if you only watch pelvis_z.
                print(f"  [sim] t={now:6.1f}s  pelvis=({env.mj_data.qpos[0]:+.2f},"
                      f"{env.mj_data.qpos[1]:+.2f},{env.mj_data.qpos[2]:.3f})m  "
                      f"lowcmd={'yes' if got else 'no '}  "
                      f"kp[0]={kp0:6.1f} kd[0]={kd0:5.1f}"
                      f"{f'  lat={env.actuation_delay_ms:.0f}ms' if env.actuation_delay_ms else ''}",
                      flush=True)

            if args.duration and now >= args.duration:
                print(f"  [sim] --duration {args.duration}s reached", flush=True)
                break

            elapsed = time.monotonic() - step_start
            if (sleep_for := sim.sim_dt - elapsed) > 0:
                time.sleep(sleep_for)
            sim_cnt += 1
    except KeyboardInterrupt:
        print("\n  [sim] interrupted", flush=True)
    finally:
        print(f"  [sim] final pelvis height {env.mj_data.qpos[2]:.3f} m", flush=True)
        if recorder is not None:
            recorder.close()
        sim.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
