"""Regression checks for reference order and deployment-window measurements.

Run: python -m unittest discover -s tools -p test_four_motion.py
"""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from four_motion import command, verify_joint_order
from four_motion_metrics import dds_metrics
from mujoco_player import ISAAC_TO_MJ


class FourMotionTests(unittest.TestCase):
    def test_all_reference_joint_values_must_be_in_isaac_order(self):
        source = np.arange(87).reshape(3, 29) / 100
        clip = SimpleNamespace(dof50=source, vel50=source * 2)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for filename, values in (
                ("joint_pos.csv", clip.dof50),
                ("joint_vel.csv", clip.vel50),
            ):
                np.savetxt(
                    root / filename,
                    values[:, ISAAC_TO_MJ],
                    delimiter=",",
                    header="joints",
                )
            verify_joint_order(root, clip)
            # The historical bug has valid CSV widths but wrong values.
            np.savetxt(root / "joint_pos.csv", source, delimiter=",", header="joints")
            with self.assertRaisesRegex(ValueError, "joint order/values"):
                verify_joint_order(root, clip)

    def test_velocity_permutation_is_checked_independently(self):
        source = np.arange(58).reshape(2, 29) / 100
        clip = SimpleNamespace(dof50=source, vel50=-source)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            np.savetxt(
                root / "joint_pos.csv",
                source[:, ISAAC_TO_MJ],
                delimiter=",",
                header="joints",
            )
            np.savetxt(root / "joint_vel.csv", -source, delimiter=",", header="joints")
            with self.assertRaisesRegex(ValueError, "joint_vel.csv"):
                verify_joint_order(root, clip)

    def test_rehearsal_starts_reference_and_uses_loopback(self):
        cmd = command(
            "rehearse",
            "fixed_dr",
            {"alias": "walking", "duration": 9.7},
            Path("out"),
            0,
            0,
            False,
        )
        self.assertIn("--play", cmd)
        self.assertEqual(cmd[cmd.index("--iface") + 1], "lo")
        self.assertEqual(cmd[cmd.index("--policy") + 1], "four_motion_fixed_dr")

    def measure(self, heights):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "runner.log").write_text(
                "1002.0 transitioning to CONTROL\n1002.1 Playing motion\n"
                "1003.0 Motion index: 0 completed\n1004.0 Stopping G1Deploy\n"
            )
            lines = ["EVENT epoch 1000.0", "EVENT band_released t=2.02"]
            lines += [f"t={t:.1f}s pelvis=(0.0,0.0,{z:.3f})m" for t, z in heights]
            (root / "sim.log").write_text("\n".join(lines))
            return dds_metrics(root)

    def test_post_stop_collapse_does_not_fail_playback(self):
        heights = [(t / 10, 0.75 if t < 40 else 0.10) for t in range(20, 46)]
        result = self.measure(heights)
        self.assertTrue(result["no_low_pelvis_during_policy"])
        self.assertTrue(result["no_low_pelvis_during_playback"])
        self.assertAlmostEqual(result["support_release_after_control_s"], 0.02)

    def test_sustained_low_height_during_playback_is_failure(self):
        heights = [(t / 10, 0.10) for t in range(20, 30)]
        result = self.measure(heights)
        self.assertFalse(result["no_low_pelvis_during_playback"])
        self.assertAlmostEqual(result["first_low_pelvis_during_playback_s"], 0.4)


if __name__ == "__main__":
    unittest.main()
