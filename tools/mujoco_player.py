#!/usr/bin/env python3
"""Play an exported SONIC policy in MuJoCo, under chosen physics, and record it.

Single process, no DDS, no C++ runner. Loads the fused ``*_g1.onnx`` exported
by ``eval_agent_trl.py +export_onnx_only=true``, rebuilds its 1,570-float
observation from MuJoCo state and the reference clip using the training-side
definitions, runs PD torque control at 200 Hz with a 20 ms policy step, and
writes an mp4 through an EGL offscreen renderer.

Why the parity gate exists
--------------------------
The observation pipeline is re-implemented here, not shared with training, so
a wrong term would show a broken policy rather than a bad one. The gate: the
fixed-DR policy scores 100% success on this clip at nominal physics in Isaac.
If it cannot track the clip at nominal physics here, the pipeline is wrong and
no comparison footage is produced.

Domain-randomization knobs mirror the six training channels as closely as
MuJoCo allows: friction scale, link-mass scale, pelvis centre-of-mass offset,
joint-default bias, periodic pushes, and an actuation delay buffer. They are
labelled by the lambda they approximate; they are not the Isaac events.

usage: mujoco_player.py --onnx <..._g1.onnx> --clip <motion.pkl> --out <mp4>
          [--lam 0|1.5|2.0] [--seed N] [--width W --height H] [--no-video]
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys

import numpy as np

# Where the G1 MuJoCo model lives. Inside the standalone deploy bundle the model
# and its meshes ship at runner/g1/; that is checked FIRST, so the bundle is
# self-contained on a machine that has never seen the training repo. The absolute
# path is only the fallback for running this file in place inside that repo.
_BUNDLE_G1 = Path(__file__).resolve().parent.parent / "runner" / "g1"
if (_BUNDLE_G1 / "g1_29dof.xml").is_file():
    _G1 = _BUNDLE_G1
else:
    REPO = Path("/home/linjiw/lucid/GR00T-WholeBodyControl")
    _G1 = REPO / "gear_sonic_deploy" / "g1"
DEFAULT_XML = _G1 / "g1_29dof.xml"
DEFAULT_SCENE = _G1 / "scene_empty.xml"

SIM_DT = 0.005  # Isaac sim_dt
DECIMATION = 4  # 20 ms policy step
CLIP_FPS = 30
NUM_FUTURE = 10  # num_future_frames
DT_FUTURE = 0.1  # dt_future_ref_frames, seconds
HISTORY = 10  # actor_prop_history_length

# MuJoCo joint order (from g1_29dof.xml), index = actuator index.
MJ_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

# ---------------------------------------------------------------------------
# Filled from the training-side source (see docstring). Each entry is verified
# against file:line in the commit message that lands it.
# ---------------------------------------------------------------------------
#: Isaac (training) joint order -> MuJoCo index.  isaac_to_mj[i] = mj index of Isaac joint i
# Verified: gear_sonic_deploy/visualize_motion.py:65 gives isaaclab_to_mujoco as the
# MuJoCo-indexed table [0,3,6,9,13,17,...]; inverting it yields IsaacLab's
# breadth-first joint order (hip_pitch L/R, waist_yaw, hip_roll L/R, waist_roll,
# hip_yaw L/R, waist_pitch, knee L/R, shoulder_pitch L/R, ankle_pitch L/R, ...).
ISAAC_TO_MJ: list[int] | None = [
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26,
    20, 27, 21, 28,
]
ISAAC_JOINTS = [MJ_JOINTS[j] for j in ISAAC_TO_MJ]
#: Training-side constants, all verified against gear_sonic/envs/manager_env/robots/g1.py
#: (init_state.joint_pos :223-235; gain formulas :14-27 and groups :238-357;
#: effort limits :238-357; action scale :363-374) and policy_parameters.hpp:107-208.
_OMEGA = 2.0 * np.pi * 10.0
_ZETA = 2.0


def _group(name: str) -> str:
    n = name.replace("_joint", "")
    if any(k in n for k in ("hip_pitch", "hip_roll", "knee")):
        return "7520_22"
    if any(k in n for k in ("hip_yaw", "waist_yaw")):
        return "7520_14"
    if any(k in n for k in ("ankle", "waist_roll", "waist_pitch")):
        return "2x5020"
    if any(k in n for k in ("wrist_pitch", "wrist_yaw")):
        return "4010"
    return "5020"  # shoulders, elbow, wrist_roll


_KP = {"7520_22": 99.13, "7520_14": 40.18, "2x5020": 28.51, "5020": 14.25, "4010": 16.78}
_KD = {"7520_22": 6.31, "7520_14": 2.56, "2x5020": 1.81, "5020": 0.907, "4010": 1.068}


def _effort(name: str) -> float:
    n = name.replace("_joint", "")
    if "hip_yaw" in n or "waist_yaw" in n:
        return 88.0
    if any(k in n for k in ("hip_roll", "hip_pitch", "knee")):
        return 139.0
    if any(k in n for k in ("ankle", "waist_roll", "waist_pitch")):
        return 50.0
    if any(k in n for k in ("wrist_pitch", "wrist_yaw")):
        return 5.0
    return 25.0


def _default(name: str) -> float:
    n = name.replace("_joint", "")
    if n.endswith("hip_pitch"):
        return -0.312
    if n.endswith("knee"):
        return 0.669
    if n.endswith("ankle_pitch"):
        return -0.363
    if n.endswith("elbow"):
        return 0.6
    if n == "left_shoulder_roll":
        return 0.2
    if n == "right_shoulder_roll":
        return -0.2
    if n.endswith("shoulder_pitch"):
        return 0.2
    return 0.0


_isaac_names = [MJ_JOINTS[j] for j in ISAAC_TO_MJ]
DEFAULT_JOINT_POS_ISAAC: np.ndarray | None = np.array([_default(n) for n in _isaac_names])
KP_ISAAC: np.ndarray | None = np.array([_KP[_group(n)] for n in _isaac_names])
KD_ISAAC: np.ndarray | None = np.array([_KD[_group(n)] for n in _isaac_names])
EFFORT_ISAAC: np.ndarray | None = np.array([_effort(n) for n in _isaac_names])
#: Per-joint action scale: q_des = default + 0.25 * effort / kp * action.
ACTION_SCALE_ISAAC: np.ndarray = 0.25 * EFFORT_ISAAC / KP_ISAAC
#: Isaac sets joint armature = kp / omega^2 (the gains are derived from it).
ARMATURE_ISAAC: np.ndarray = KP_ISAAC / (_OMEGA**2)
ACTION_SCALE: float = 1.0  # kept for the CLI; the per-joint vector above is what is applied
REF_FPS = 50  # motion.yaml target_fps: the clip is resampled to 50 Hz at load
FRAME_SKIP = int(round(DT_FUTURE * REF_FPS))  # 5 frames = 0.1 s


@dataclass
class DRConfig:
    """The six training channels at a scalar lambda, with Isaac's exact ranges.

    Ranges are the lambda = 1 event configs under
    gear_sonic/config/manager_env/events/terms/, widened affinely about their
    nominal the way dr_scaling.scale_params does (mass about 1.0, additive
    terms about 0, friction about the range midpoint), then physically clamped
    like dr_scaling.clamp_physical. Sampling mirrors the Isaac terms: mass is
    drawn PER BODY, the CoM shift applies to torso_link only, and a push SETS
    the base velocity rather than adding to it.
    """

    lam: float = 0.0
    seed: int = 0
    static_friction: tuple[float, float] = (0.3, 1.6)  # physics_material, midpoint nominal
    mass_scale: tuple[float, float] = (0.8, 1.2)  # randomize_rigid_body_mass, per body
    com_range: tuple[tuple[float, float], ...] = ((-0.025, 0.025), (-0.05, 0.05), (-0.05, 0.05))
    joint_bias: tuple[float, float] = (-0.01, 0.01)  # add_joint_default_pos
    push_lin: tuple[float, float, float] = (0.5, 0.5, 0.2)  # push_robot velocity_range xyz
    push_ang: tuple[float, float, float] = (0.52, 0.52, 0.78)  # roll pitch yaw
    push_interval_s: tuple[float, float] = (1.0, 3.0)
    delay_range: tuple[float, float] = (0.0, 8.0)  # physics steps
    #: Channels to enable; None = all six. For ablations.
    channels: tuple[str, ...] | None = None

    def _widen(self, lo: float, hi: float, centre: float) -> tuple[float, float]:
        return centre + (lo - centre) * self.lam, centre + (hi - centre) * self.lam

    def sample(self, num_bodies: int) -> dict:
        rng = np.random.default_rng(self.seed)
        if self.lam <= 0:
            return {"friction": 1.0, "mass_scale": [1.0] * num_bodies, "com_offset": [0.0, 0.0, 0.0],
                    "joint_bias": [0.0] * 29, "push_lin": [0.0, 0.0, 0.0], "push_ang": [0.0, 0.0, 0.0],
                    "delay_steps": 0, "seed": int(self.seed)}
        f_lo, f_hi = self._widen(*self.static_friction, centre=sum(self.static_friction) / 2)
        f_lo = max(f_lo, 0.05)
        m_lo, m_hi = self._widen(*self.mass_scale, centre=1.0)
        m_lo = max(m_lo, 0.1)
        com = [float(rng.uniform(*self._widen(lo, hi, 0.0))) for lo, hi in self.com_range]
        jb_lo, jb_hi = self._widen(*self.joint_bias, 0.0)
        d_lo, d_hi = self._widen(*self.delay_range, 0.0)
        out = {
            "friction": float(rng.uniform(f_lo, f_hi)),
            "mass_scale": rng.uniform(m_lo, m_hi, num_bodies).tolist(),
            "com_offset": com,
            "joint_bias": rng.uniform(jb_lo, jb_hi, 29).tolist(),
            "push_lin": [v * self.lam for v in self.push_lin],
            "push_ang": [v * self.lam for v in self.push_ang],
            "delay_steps": int(round(rng.uniform(max(0.0, d_lo), d_hi))),
            "seed": int(self.seed),
        }
        if self.channels is not None:
            on = set(self.channels)
            if "friction" not in on: out["friction"] = 1.0
            if "mass" not in on: out["mass_scale"] = [1.0] * num_bodies
            if "com" not in on: out["com_offset"] = [0.0, 0.0, 0.0]
            if "joint" not in on: out["joint_bias"] = [0.0] * 29
            if "push" not in on: out["push_lin"] = [0.0, 0.0, 0.0]; out["push_ang"] = [0.0, 0.0, 0.0]
            if "delay" not in on: out["delay_steps"] = 0
        out["channels"] = list(self.channels) if self.channels else "all"
        return out


@dataclass
class Clip:
    """Reference motion resampled to REF_FPS, as motion_lib does at load.

    ``dof50`` is (T50, 29) in MuJoCo order. Velocities are the forward finite
    difference at the resampled dt with the last row duplicated
    (torch_humanoid_batch.py:449-450). Root quaternion is xyzw here; the
    training code converts the same xyzw source to wxyz once and uses it as
    the pelvis reference orientation.
    """

    dof50: np.ndarray
    vel50: np.ndarray
    root_pos50: np.ndarray
    root_quat50_xyzw: np.ndarray
    src_fps: int
    name: str

    @property
    def num_frames(self) -> int:
        return len(self.dof50)

    @property
    def duration(self) -> float:
        return (self.num_frames - 1) / REF_FPS

    def frame(self, k: int) -> int:
        return int(np.clip(k, 0, self.num_frames - 1))


def _resample(x: np.ndarray, src_fps: int, dst_fps: int) -> np.ndarray:
    t_src = np.arange(len(x)) / src_fps
    t_dst = np.arange(0, t_src[-1] + 1e-9, 1.0 / dst_fps)
    out = np.stack([np.interp(t_dst, t_src, x[:, j]) for j in range(x.shape[1])], axis=1)
    return out


def _resample_quat(q: np.ndarray, src_fps: int, dst_fps: int) -> np.ndarray:
    t_src = np.arange(len(q)) / src_fps
    t_dst = np.arange(0, t_src[-1] + 1e-9, 1.0 / dst_fps)
    out = np.zeros((len(t_dst), 4))
    for i, t in enumerate(t_dst):
        f = min(t * src_fps, len(q) - 1)
        i0 = int(np.floor(f)); i1 = min(i0 + 1, len(q) - 1); a = f - i0
        out[i] = quat_slerp(q[i0], q[i1], a)
    return out


def load_clip(path: Path) -> Clip:
    import joblib

    data = joblib.load(path)
    name, mo = next(iter(data.items()))
    fps = int(mo.get("fps", CLIP_FPS))
    dof = _resample(np.asarray(mo["dof"], dtype=np.float64), fps, REF_FPS)
    vel = np.diff(dof, axis=0) * REF_FPS
    vel = np.concatenate([vel, vel[-1:]], axis=0)
    return Clip(
        dof50=dof,
        vel50=vel,
        root_pos50=_resample(np.asarray(mo["root_trans_offset"], dtype=np.float64), fps, REF_FPS),
        root_quat50_xyzw=_resample_quat(np.asarray(mo["root_rot"], dtype=np.float64), fps, REF_FPS),
        src_fps=fps,
        name=name,
    )


# ------------------------------------------------------------- quaternions --
# All quaternions here are xyzw unless the name says otherwise. MuJoCo stores
# wxyz; conversions are explicit at the boundary.


def quat_wxyz_to_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([q[1], q[2], q[3], q[0]])


def quat_xyzw_to_wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[3], q[0], q[1], q[2]])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_rotate_inv(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate v by the inverse of q (world -> body)."""
    vq = np.array([v[0], v[1], v[2], 0.0])
    r = quat_mul(quat_mul(quat_conj(q), vq), q)
    return r[:3]


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > 0.9995:
        r = a + t * (b - a)
        return r / np.linalg.norm(r)
    th = np.arccos(d)
    return (np.sin((1 - t) * th) * a + np.sin(t * th) * b) / np.sin(th)


# --------------------------------------------------------------- the loop --


class Player:
    def __init__(self, onnx_path: Path, clip: Clip, dr: DRConfig, xml: Path = DEFAULT_XML,
                 scene: Path = DEFAULT_SCENE, width: int = 1280, height: int = 720,
                 video: bool = True):
        import mujoco
        import onnxruntime as ort

        if ISAAC_TO_MJ is None or KP_ISAAC is None or DEFAULT_JOINT_POS_ISAAC is None:
            raise RuntimeError("training-side constants not filled in; see module docstring")
        self.mujoco = mujoco
        self.model = self._build_model(xml, scene)
        self.model.opt.timestep = SIM_DT
        self.data = mujoco.MjData(self.model)
        self.clip = clip
        self.dr = dr
        self.dr_sample = dr.sample(self.model.nbody)
        self._apply_dr()
        self.session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
        self.obs_name = self.session.get_inputs()[0].name
        self.obs_dim = int(self.session.get_inputs()[0].shape[1])
        self.pelvis = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.mj_to_isaac = np.argsort(np.asarray(ISAAC_TO_MJ))
        self.isaac_to_mj = np.asarray(ISAAC_TO_MJ)
        self.default_mj = np.asarray(DEFAULT_JOINT_POS_ISAAC)[self.mj_to_isaac] + np.asarray(
            self.dr_sample["joint_bias"]
        )
        self.kp_mj = np.asarray(KP_ISAAC)[self.mj_to_isaac]
        self.kd_mj = np.asarray(KD_ISAAC)[self.mj_to_isaac]
        self.effort_mj = None if EFFORT_ISAAC is None else np.asarray(EFFORT_ISAAC)[self.mj_to_isaac]
        self.scale_mj = ACTION_SCALE_ISAAC[self.mj_to_isaac]
        # Isaac's actuator model adds armature = kp / omega^2 to every joint.
        self.model.dof_armature[6:] = ARMATURE_ISAAC[self.mj_to_isaac]
        self.hist: dict[str, deque] = {
            k: deque(maxlen=HISTORY) for k in ("gravity", "ang_vel", "joint_pos", "joint_vel", "action")
        }
        self.last_action_isaac = np.zeros(29)
        # randomize_action_delay is in PHYSICS steps (0..8 at lambda 1 = 0..40 ms),
        # so the buffer holds one target per 5 ms substep and the applied target
        # is the one pushed delay_steps substeps ago.
        self.delay = deque(maxlen=max(1, self.dr_sample["delay_steps"] + 1))
        self.rng = np.random.default_rng(dr.seed + 1)
        self.next_push_t = self.rng.uniform(*dr.push_interval_s) if max(self.dr_sample['push_lin']) > 0 else np.inf
        self.video = video
        self.frames: list[np.ndarray] = []
        if video:
            # MuJoCo's offscreen framebuffer defaults to 640x480; the renderer
            # refuses anything larger unless the model's limits are raised.
            self.model.vis.global_.offwidth = max(int(self.model.vis.global_.offwidth), width)
            self.model.vis.global_.offheight = max(int(self.model.vis.global_.offheight), height)
        self.renderer = mujoco.Renderer(self.model, height, width) if video else None
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.cam.trackbodyid = self.pelvis
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 3.0, 135.0, -15.0
        self.log: list[dict] = []

    def _build_model(self, xml: Path, scene: Path):  # noqa: ARG002
        """The deploy robot file already carries its floor, light and skybox.

        Composing it into scene_empty.xml duplicates the 'groundplane' texture
        and MuJoCo refuses the model, so the robot file is loaded on its own.
        """
        return self.mujoco.MjModel.from_xml_path(str(xml))

    def _apply_dr(self):
        s = self.dr_sample
        m = self.model
        # Friction: MuJoCo pairs take the max of the two geoms' coefficients, so
        # every geom (floor included, all 1.0 in the XML) is set to the draw.
        m.geom_friction[:, 0] = np.clip(s["friction"], 0.05, 4.0)
        scale = np.asarray(s["mass_scale"])
        scale[0] = 1.0  # world body
        m.body_mass[:] = m.body_mass * scale
        m.body_inertia[:] = m.body_inertia * scale[:, None]
        torso = self.mujoco.mj_name2id(m, self.mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        m.body_ipos[torso] = m.body_ipos[torso] + np.asarray(s["com_offset"])
        self.mujoco.mj_setConst(m, self.data)

    # ----------------------------------------------------------- state --

    def reset(self):
        d, m = self.data, self.model
        self.mujoco.mj_resetData(m, d)
        d.qpos[0:3] = self.clip.root_pos50[0]
        d.qpos[3:7] = quat_xyzw_to_wxyz(self.clip.root_quat50_xyzw[0])
        d.qpos[7:] = self.clip.dof50[0]
        # Isaac resets the robot WITH the reference velocities (root linear and
        # angular, and joint), not at rest. The clip is already walking at
        # ~0.8 m/s at frame 0; starting from rest opened a 0.3 m lag in the
        # first half second that a policy without position feedback never
        # recovered. Free-joint qvel: linear is world-frame, angular is body-frame.
        d.qvel[:] = 0
        d.qvel[0:3] = (self.clip.root_pos50[1] - self.clip.root_pos50[0]) * REF_FPS
        q0, q1 = self.clip.root_quat50_xyzw[0], self.clip.root_quat50_xyzw[1]
        dq = quat_mul(quat_conj(q0), q1)  # body-frame relative rotation over one frame
        ang = 2.0 * dq[:3] * (1.0 if dq[3] >= 0 else -1.0) * REF_FPS  # small-angle
        d.qvel[3:6] = ang
        d.qvel[6:] = self.clip.vel50[0]
        self.k = 0  # reference frame index; one control step == one 50 Hz frame
        self.mujoco.mj_forward(m, d)
        for h in self.hist.values():
            h.clear()
        self.last_action_isaac[:] = 0
        self.delay.clear()
        self.t = 0.0
        self.frames.clear()

    def _base_quat_xyzw(self) -> np.ndarray:
        return quat_wxyz_to_xyzw(self.data.qpos[3:7])

    def _proprio_isaac(self) -> dict[str, np.ndarray]:
        d = self.data
        q = self._base_quat_xyzw()
        grav = quat_rotate_inv(q, np.array([0, 0, -1.0]))
        ang_w = d.qvel[3:6].copy()  # free-joint qvel[3:6] is BODY-frame angular velocity (verified vs mj_objectVelocity flg_local=1)
        jp_mj = d.qpos[7:].copy()
        jv_mj = d.qvel[6:].copy()
        return {
            "gravity": grav,
            "ang_vel": ang_w,
            "joint_pos": (jp_mj - self.default_mj)[self.isaac_to_mj],
            "joint_vel": jv_mj[self.isaac_to_mj],
            "action": self.last_action_isaac.copy(),
        }

    # ----------------------------------------------------- observations --

    def _push_history(self, cur: dict[str, np.ndarray]) -> None:
        """CircularBuffer semantics: the first push after reset fills every slot."""
        for k, v in cur.items():
            h = self.hist[k]
            if len(h) == 0:
                for _ in range(HISTORY):
                    h.append(v.copy())
            else:
                h.append(v.copy())

    def build_obs(self) -> np.ndarray:
        """The fused g1 ONNX input, 1,570 floats.

        Layout (inference_helpers.py:120-127, TokenizerCfg attribute order,
        PolicyCfg attribute order, CircularBuffer oldest-first):

            [ ref qpos f0..f9 (290) | ref qvel f0..f9 (290) | ref pelvis ori 6D f0..f9 (60)
            | angvel hist (30) | qpos_rel hist (290) | qvel hist (290) | action hist (290)
            | gravity hist (30) ]

        Reference frames are k, k+5, ..., k+45 at 50 Hz, clamped to the clip;
        joint values are ABSOLUTE and in Isaac order. The 6D orientation is the
        first two columns of R(quat_inv(pelvis_now) * ref_pelvis_quat[f]),
        row-major [R00, R01, R10, R11, R20, R21].
        """
        cur = self._proprio_isaac()
        self._push_history(cur)
        frames = [self.clip.frame(self.k + FRAME_SKIP * j) for j in range(NUM_FUTURE)]
        qpos_ref = np.concatenate([self.clip.dof50[f][self.isaac_to_mj] for f in frames])
        qvel_ref = np.concatenate([self.clip.vel50[f][self.isaac_to_mj] for f in frames])
        q_now = self._base_quat_xyzw()
        ori = []
        for f in frames:
            rel = quat_mul(quat_conj(q_now), self.clip.root_quat50_xyzw[f])
            R = quat_to_mat(rel)
            ori.extend([R[0, 0], R[0, 1], R[1, 0], R[1, 1], R[2, 0], R[2, 1]])
        proprio = np.concatenate(
            [np.concatenate(list(self.hist[k])) for k in ("ang_vel", "joint_pos", "joint_vel", "action", "gravity")]
        )
        obs = np.concatenate([qpos_ref, qvel_ref, np.asarray(ori), proprio])
        assert obs.shape == (1570,), obs.shape
        return obs

    # ---------------------------------------------------------- control --

    def step_policy(self):
        obs = self.build_obs().astype(np.float32)[None]
        assert obs.shape[1] == self.obs_dim, (obs.shape, self.obs_dim)
        action = self.session.run(None, {self.obs_name: obs})[0][0].astype(np.float64)
        self.last_action_isaac = action
        target_mj = self.default_mj + self.scale_mj * action[self.mj_to_isaac]
        return target_mj

    def _pd(self, target_mj: np.ndarray) -> np.ndarray:
        d = self.data
        tau = self.kp_mj * (target_mj - d.qpos[7:]) - self.kd_mj * d.qvel[6:]
        if self.effort_mj is not None:
            tau = np.clip(tau, -self.effort_mj, self.effort_mj)
        return tau

    def _maybe_push(self):
        """push_by_setting_velocity: ADD a world-frame draw to the root velocity.

        IsaacLab (envs/mdp/events.py) does ``vel_w += sample_uniform(range)`` on
        the world-frame root velocity, so the walking momentum is kept and the
        push is an impulse on top of it. MuJoCo's free joint stores linear
        velocity in the world frame and angular velocity in the body frame, so
        the angular part of the draw is rotated into the body frame first.
        """
        if self.t >= self.next_push_t:
            lin = self.rng.uniform(-1, 1, 3) * np.asarray(self.dr_sample["push_lin"])
            ang_w = self.rng.uniform(-1, 1, 3) * np.asarray(self.dr_sample["push_ang"])
            R = quat_to_mat(self._base_quat_xyzw())
            self.data.qvel[0:3] += lin
            self.data.qvel[3:6] += R.T @ ang_w
            self.next_push_t = self.t + self.rng.uniform(*self.dr.push_interval_s)
            return True
        return False

    def run(self, max_time: float | None = None, full_clip: bool = False) -> dict:
        max_time = self.clip.duration if max_time is None else min(max_time, self.clip.duration)
        self.reset()
        fell = False
        t_drift: float | None = None
        n_ctrl = 0
        while self.t < max_time:
            target = self.step_policy()
            for _ in range(DECIMATION):
                self.delay.append(target)
                delayed = self.delay[0]  # oldest = delay_steps substeps ago once the buffer is full
                self._maybe_push()
                self.data.ctrl[:] = self._pd(delayed)
                self.mujoco.mj_step(self.model, self.data)
                self.t += SIM_DT
            n_ctrl += 1
            self.k += 1  # one 50 Hz reference frame per 20 ms control step
            if self.video:
                self.renderer.update_scene(self.data, camera=self.cam)
                self.frames.append(self.renderer.render().copy())
            pelvis_z = float(self.data.xpos[self.pelvis][2])
            ref_pos = self.clip.root_pos50[self.clip.frame(self.k)]
            err = float(np.linalg.norm(self.data.xpos[self.pelvis] - ref_pos))
            self.log.append({"t": round(self.t, 3), "k": self.k, "pelvis_z": round(pelvis_z, 3), "anchor_err": round(err, 3)})
            if err > 0.5:  # anchor_pos termination threshold used in every scored cell
                fell = True
                if t_drift is None:
                    t_drift = self.t
                # Breaking here is right for SCORING: it is the same event the
                # Isaac anchor_pos termination fires on. It is wrong for
                # LOOKING -- a policy that drifts at 1.3 s yields a 1.3 s video
                # in which nothing can be seen. full_clip keeps stepping so the
                # whole motion is on screen with the drift moment recorded.
                # `fell` is set either way, so the scored number is identical.
                if not full_clip:
                    break
        # `fell` is a TRACKING criterion -- pelvis-to-reference distance past
        # 0.5 m -- and not a fall. A policy that walks steadily but drifts off
        # the reference path trips it while fully upright, which is exactly what
        # a policy trained under heavy push randomization does. Record the
        # pelvis height against the reference's own height at the same frame so
        # a reader (and any caption) can tell the two events apart instead of
        # calling every termination a fall.
        pelvis_z = float(self.data.xpos[self.pelvis][2])
        ref_z = float(self.clip.root_pos50[self.clip.frame(self.k)][2])
        upright = pelvis_z > 0.6 * ref_z
        return {"fell": fell, "t_end": round(self.t, 3), "duration": round(max_time, 3),
                "pelvis_z_end": round(pelvis_z, 3), "ref_z_end": round(ref_z, 3),
                "upright_at_end": bool(upright),
                "outcome": ("completed" if not fell else ("drifted" if upright else "fell")),
                "t_drift": (round(t_drift, 3) if t_drift is not None else None),
                "full_clip": bool(full_clip),
                "dr": self.dr_sample, "lam": self.dr.lam}

    def save_video(self, out: Path, fps: float = 1 / (SIM_DT * DECIMATION)):
        import imageio

        out.parent.mkdir(parents=True, exist_ok=True)
        with imageio.get_writer(str(out), fps=fps, codec="libx264", quality=8) as w:
            for f in self.frames:
                w.append_data(f)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--onnx", type=Path, required=True)
    p.add_argument("--full-clip", action="store_true",
                   help="keep stepping past the 0.5 m drift threshold so the whole motion is "
                        "rendered; the scored 'fell' flag and t_drift are unchanged")
    p.add_argument("--clip", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--lam", type=float, default=0.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--max-time", type=float, default=None)
    p.add_argument("--channels", type=str, default=None, help="comma list of friction,mass,com,joint,push,delay")
    a = p.parse_args(argv)
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

    clip = load_clip(a.clip)
    channels = tuple(c.strip() for c in a.channels.split(',')) if a.channels else None
    player = Player(a.onnx, clip, DRConfig(lam=a.lam, seed=a.seed, channels=channels), width=a.width, height=a.height,
                    video=not a.no_video)
    result = player.run(a.max_time, full_clip=a.full_clip)
    if not a.no_video:
        player.save_video(a.out)
        result["video"] = str(a.out)
    a.out.with_suffix(".json").write_text(json.dumps({"result": result, "log": player.log}, indent=1))
    print(json.dumps(result))
    sys.stdout.flush()
    if player.renderer is not None:
        player.renderer.close()
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
