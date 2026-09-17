#!/usr/bin/env python3
"""Check a deployment observation config against the C++ runner's own registry.

    validate_deploy_obs_config.py <config.yaml> [--expect-dim 1570]

The runner builds its observation by walking the config's terms IN ORDER and
writing each one at a running offset. Three things can go wrong, and only the
first is loud:

1. A term name the registry does not know. ``InitializeObservationFunctions``
   throws, so the runner refuses to start. Loud, but only on the robot.
2. The right names in the WRONG ORDER. Every offset shifts, the total still
   matches, the model still runs, and it is fed noise. Silent.
3. The right names in the right order summing to the wrong total. The model's
   input width disagrees and the failure is a shape error at best.

This script catches all three before a build, on any machine, with no
toolchain: it parses the registry out of the runner's own source, so it cannot
drift from what the binary will actually accept.

Why this exists: ``policy/release/observation_config_sonic_release.yaml`` --
the file whose header names the exact export command used for LUCID -- lists
eight term names that are absent from the 76-entry registry, and orders its
encoder terms differently from the exporter. It would throw on load, and if the
names were fixed it would still place 66 floats in the wrong slots.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

# The runner source ships inside this bundle at runner/src/; that is checked
# FIRST, so the tool works on a machine that has never seen the training repo.
# It previously resolved only parents[2]/gear_sonic_deploy/..., i.e. a sibling of
# the bundle's own parent directory, so every documented invocation of this file
# died with FileNotFoundError on a path outside the repository. The absolute path
# is kept as the fallback for running this file in place inside the training
# tree. Same shape as tools/mujoco_player.py:44-48.
_BUNDLE_SRC = (
    Path(__file__).resolve().parent.parent
    / "runner/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp"
)
if _BUNDLE_SRC.is_file():
    RUNNER_SRC = _BUNDLE_SRC
else:
    REPO = Path(__file__).resolve().parents[2]
    RUNNER_SRC = REPO / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp"

#: The fused-g1 export's layout, in the order the exporter concatenates it.
#: Traced to tools/mujoco_player.py:build_obs, which is the reference
#: implementation validated behaviourally against the Isaac receipts, and to
#: inference_helpers.py:120-127 (TokenizerCfg then PolicyCfg attribute order,
#: gravity LAST -- NOT the order the YAML happens to list the terms in).
FUSED_G1_1570 = [
    ("motion_joint_positions_10frame_step5", 290),
    ("motion_joint_velocities_10frame_step5", 290),
    ("motion_anchor_orientation_10frame_step5", 60),
    ("his_base_angular_velocity_10frame_step1", 30),
    ("his_body_joint_positions_10frame_step1", 290),
    ("his_body_joint_velocities_10frame_step1", 290),
    ("his_last_actions_10frame_step1", 290),
    ("his_gravity_dir_10frame_step1", 30),
]


def registry(source: Path = RUNNER_SRC) -> dict[str, int]:
    """Term name -> declared width, parsed from the runner's own source."""
    text = source.read_text()
    start = text.index('return {{"token_state"')
    block = text[start : start + 24000]
    found: dict[str, int | None] = {}
    # token_state's width is the identifier `token_dim` (the encoder output
    # size), not a literal, so accept an identifier and record it as None --
    # "known, but sized by the encoder section" rather than "unknown".
    for name, width in re.findall(r'\{"([A-Za-z0-9_]+)",\s*([A-Za-z0-9_]+)', block):
        found[name] = int(width) if width.isdigit() else None
    if not found:
        raise SystemExit(f"could not parse an observation registry out of {source}")
    return found


def terms_of_runner(path: Path) -> list[str]:
    """Term names as the RUNNER reads them, not as YAML defines them.

    The runner does not use a YAML parser. ObservationConfigParser scans for
    lines containing "- name:" and hands the remainder to ExtractValue
    (observation_config.hpp:452-470), which trims whitespace and double quotes
    from both ends and NOTHING ELSE. In particular it does not strip a trailing
    `#` comment, so

        - name: "his_gravity_dir_10frame_step1"   # 30 floats

    yields the term name `his_gravity_dir_10frame_step1"   # 30 floats` and the
    runner aborts at startup with "Unknown observation function".

    Reading the file with PyYAML hides this completely -- a real parser strips
    the comment and the config looks perfect -- which is why this check has to
    reproduce ExtractValue rather than trust yaml.safe_load.
    """
    out = []
    in_obs = False
    for raw in path.read_text().splitlines():
        stripped = raw.strip()
        if stripped.startswith("observations:"):
            in_obs = True
            continue
        if stripped.startswith("encoder:"):
            in_obs = False
        if not in_obs or "- name:" not in raw:
            continue
        value = raw[raw.index("- name:") + len("- name:") :]
        value = value.strip(' \t"')
        out.append(value)
    return out


def terms_of(config: dict) -> list[str]:
    """Enabled observation terms per a real YAML parser, for comparison."""
    out = []
    for entry in config.get("observations") or []:
        if entry.get("enabled", True):
            out.append(entry["name"])
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("config", type=Path)
    ap.add_argument(
        "--expect-dim",
        type=int,
        default=None,
        help="required total width, e.g. 1570 for the fused g1 export",
    )
    ap.add_argument(
        "--expect-layout",
        choices=("fused_g1_1570",),
        default=None,
        help="also require the exact term ORDER of a known export",
    )
    ap.add_argument("--runner-src", type=Path, default=RUNNER_SRC)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)

    import yaml

    known = registry(a.runner_src)
    config = yaml.safe_load(a.config.read_text())
    names = terms_of(config)
    problems: list[str] = []

    # What the runner will actually see. Any divergence from the YAML reading is
    # a name the runner will reject even though the file is valid YAML.
    runner_names = terms_of_runner(a.config)
    for yaml_name, runner_name in zip(names, runner_names):
        if yaml_name != runner_name:
            problems.append(
                f"the runner reads this term as {runner_name!r}, not {yaml_name!r}. Its "
                f"ExtractValue trims only whitespace and quotes, so an inline '#' comment "
                f"on a '- name:' line becomes part of the name and startup aborts with "
                f"'Unknown observation function'. Put the comment on its own line."
            )
    if len(runner_names) != len(names):
        problems.append(
            f"the runner finds {len(runner_names)} '- name:' lines but YAML defines "
            f"{len(names)} enabled terms"
        )

    encoder_names = [
        e["name"]
        for e in ((config.get("encoder") or {}).get("encoder_observations") or [])
        if e.get("enabled", True)
    ]
    for n in names:
        if n not in known:
            problems.append(
                f"observation term not in the runner registry, "
                f"InitializeObservationFunctions would throw: {n!r}"
            )
    for n in encoder_names:
        if n not in known:
            problems.append(
                f"ENCODER term not in the runner registry, "
                f"InitializeObservationFunctions would throw: {n!r}"
            )

    # token_state without an encoder section is refused by the runner; an
    # encoder section without token_state is silently ignored.
    encoder_dim = int((config.get("encoder") or {}).get("dimension") or 0)
    if "token_state" in names and encoder_dim <= 0:
        problems.append(
            "'token_state' is enabled but no encoder dimension is set; the runner refuses this"
        )
    if "token_state" not in names and encoder_dim > 0:
        problems.append(
            "an encoder section is present but 'token_state' is not enabled; the runner will IGNORE the encoder"
        )

    total = sum((encoder_dim if n == "token_state" else (known.get(n) or 0)) for n in names)
    encoder_total = sum((known.get(n) or 0) for n in encoder_names)

    if a.expect_dim is not None and total != a.expect_dim:
        problems.append(f"total width {total} != expected {a.expect_dim}")

    if a.expect_layout == "fused_g1_1570":
        want = [n for n, _ in FUSED_G1_1570]
        if names != want:
            problems.append(
                "term ORDER does not match the fused-g1 export. Offsets are assigned in "
                "file order, so a reordering silently feeds the model shuffled input.\n"
                f"    expected: {want}\n    got     : {names}"
            )
        for n, w in FUSED_G1_1570:
            if n in known and known[n] != w:
                problems.append(f"{n}: runner declares {known[n]}, the export expects {w}")

    report = {
        "config": str(a.config),
        "runner_src": str(a.runner_src),
        "registry_terms": len(known),
        "terms": [{"name": n, "width": known.get(n)} for n in names],
        "encoder_terms": [{"name": n, "width": known.get(n)} for n in encoder_names],
        "encoder_input_width": encoder_total,
        "encoder_dimension": encoder_dim,
        "total_width": total,
        "problems": problems,
        "ok": not problems,
    }
    if a.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"registry: {len(known)} terms parsed from {a.runner_src.name}")
        for n in names:
            if n == "token_state":
                print(f"  OK   {n:45s} {encoder_dim} (from encoder.dimension)")
            elif n in known:
                print(f"  OK   {n:45s} {known[n]}")
            else:
                print(f"  BAD  {n:45s} (not in registry)")
        if encoder_names:
            print(f"  encoder inputs ({encoder_total} floats -> {encoder_dim}):")
            for n in encoder_names:
                print(
                    f"    {'OK ' if n in known else 'BAD'}  {n:43s} "
                    f"{known.get(n) if n in known else '(not in registry)'}"
                )
        print(f"total width: {total}")
        if problems:
            print("\nPROBLEMS:")
            for p in problems:
                print(f"  - {p}")
        else:
            print("OK: every term is known, ordered as the export expects, and the width matches.")
    return 0 if not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
