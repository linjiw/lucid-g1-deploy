"""Measured deployment phases and height checks from stamped DDS logs."""

import re
from pathlib import Path

import numpy as np


def dds_metrics(out: Path) -> dict:
    runner = (out / "runner.log").read_text(errors="replace")
    sim = (out / "sim.log").read_text(errors="replace")
    phases = {
        key: bool(re.search(pattern, runner))
        for key, pattern in {
            "dimensions_valid": "Dimension match",
            "engine_ready": "Policy engine initialized successfully",
            "init_done": "Init Done",
            "controller_started": "transitioning to CONTROL",
            "reference_playback_started": "Playing motion",
            "reference_playback_completed": "Motion index:.*completed",
            "stop_observed": "Stopping G1Deploy",
        }.items()
    }
    epoch = re.search(r"EVENT epoch ([\d.]+)", sim)

    def when(pattern):
        match = next(
            (line for line in runner.splitlines() if re.search(pattern, line)), ""
        )
        return float(match.split()[0]) - float(epoch[1]) if match and epoch else None

    start, end = when("transitioning to CONTROL"), when("Stopping G1Deploy")
    rows = []
    for line in sim.splitlines():
        match = re.search(
            r"t=\s*([\d.]+)s\s+pelvis=\(([-+\d.]+),([-+\d.]+),([\d.]+)\)m", line
        )
        if match:
            rows.append(tuple(map(float, match.groups())))
    segment = [
        r
        for r in rows
        if start is not None and start <= r[0] and (end is None or r[0] < end)
    ]
    count = 0
    low = None
    for row in segment:
        count = count + 1 if row[3] < 0.35 else 0
        if count >= 5 and low is None:
            low = round(row[0] - start, 3)
    phases.update(
        {
            "policy_samples": len(segment),
            "first_low_pelvis_after_start_s": low,
            "no_low_pelvis_during_policy": bool(segment) and low is None,
            "min_policy_pelvis_height_m": min((r[3] for r in segment), default=None),
            "end_policy_pelvis_height_m": segment[-1][3] if segment else None,
            "support_released": "EVENT band_released" in sim,
        }
    )
    release = re.search(r"EVENT band_released t=([\d.]+)", sim)
    phases["support_release_after_control_s"] = (
        round(float(release[1]) - start, 3) if release and start is not None else None
    )
    complete = when("Motion index:.*completed")
    playback_segment = [r for r in segment if complete is None or r[0] < complete]
    phases["min_playback_pelvis_height_m"] = min(
        (r[3] for r in playback_segment), default=None
    )
    count = 0
    low_playback = None
    for row in playback_segment:
        count = count + 1 if row[3] < 0.35 else 0
        if count >= 5 and low_playback is None:
            low_playback = round(row[0] - start, 3)
    phases["first_low_pelvis_during_playback_s"] = low_playback
    phases["no_low_pelvis_during_playback"] = (
        bool(playback_segment) and low_playback is None
    )
    console = (
        (out / "console.log").read_text(errors="replace")
        if (out / "console.log").exists()
        else ""
    )
    parity = re.search(r"max \|delta\|\s+([\deE.+-]+)", console)
    phases["tensorrt_onnx_max_action_difference"] = float(parity[1]) if parity else None
    if (out / "target.csv").exists():
        target = np.genfromtxt(out / "target.csv", delimiter=",")
        phases["distinct_reference_rows"] = (
            len(np.unique(target[:, :-1], axis=0)) if target.ndim == 2 else 0
        )
    return phases
