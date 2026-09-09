#!/usr/bin/env python3
"""Parallel MuJoCo survival sweep over arms x lambdas x seeds, with fall times.

Runs tools/mujoco_player.py without video across a grid and writes one table:

    <out>/sweep.json   {arm: {lam: {seed: {"fell": bool, "t_end": s, ...}}}}
    <out>/summary.md   pass rates per arm x lambda, and mean time-to-fall

Seeds are shared across arms at each lambda, so every arm faces the identical
sequence of physics draws and pushes. That makes per-seed comparison fair and
lets a video later show the same draw side by side.

Arms default to the five historical checkpoints below, so existing invocations
and ``mujoco_story.py`` are unchanged. A NEW campaign supplies its own:

    --arms-json  {"name": "/abs/path/model_step_008000_g1.onnx", ...}
    --arm        name=/abs/path.onnx            (repeatable)
    --clip       the reference clip to track    (default: the hob002 testbed)

Each run records the sha256 of the ONNX that produced it. A cached result whose
recorded hash differs from the file on disk is DISCARDED and recomputed: the
arm keys are stable names, so re-pointing one at a retrained checkpoint and
reusing --out previously returned the old policy's numbers in silence.

usage: mujoco_sweep.py --out DIR [--lams 0 0.5 1.0 1.5 2.0] [--seeds 32] [--jobs 6]
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

PY = "/home/linjiw/isaaclab-install/env_isaaclab/bin/python"
PLAYER = Path(__file__).resolve().parent / "mujoco_player.py"
CLIP = "/home/linjiw/lucid-sonic/pools/debug512/robot_filtered/walk_hands_on_back_loop_002__A066_M.pkl"
A = Path("/home/linjiw/lucid-sonic/artifacts/curriculum_comparison")

ARMS = {
    "off_s8600": A
    / "curriculum_comparison_ne1024_20260829_000249/seed_8600/off/exported/model_step_008000_g1.onnx",
    "lucid_collapsed_s8601": A
    / "curriculum_comparison_ne1024_20260829_000249/seed_8601/lucid_rg/exported/model_step_008000_g1.onnx",
    "fixed_s8600": A
    / "curriculum_comparison_ne1024_20260829_000249/seed_8600/fixed/exported/model_step_008000_g1.onnx",
    "ratchet_s8601": A
    / "curriculum_comparison_ne1024_20260831_144022/seed_8601/lucid_ratchet_rg/exported/model_step_008000_g1.onnx",
    "fixed_s8601": A
    / "curriculum_comparison_ne1024_20260829_000249/seed_8601/fixed/exported/model_step_008000_g1.onnx",
}
LABEL = {
    "off_s8600": "no randomization",
    "lucid_collapsed_s8601": "feedback curriculum, unconstrained (collapsed to λ 0.06)",
    "fixed_s8600": "fixed full DR",
    "ratchet_s8601": "feedback curriculum + monotone ratchet (ours)",
    "fixed_s8601": "fixed full DR (paired seed 8601)",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_arms(a) -> dict[str, Path]:
    """Arm name -> ONNX path, from --arms-json / --arm, else the historical five.

    --arms and the two new flags are mutually exclusive per arm name; a name
    given twice is an error rather than a silent last-one-wins, because the
    whole point of the hash guard is that an arm key means one checkpoint.
    """
    if a.arms_json or a.arm:
        arms: dict[str, Path] = {}
        if a.arms_json:
            for name, path in json.loads(Path(a.arms_json).read_text()).items():
                arms[str(name)] = Path(path)
        for spec in a.arm or []:
            if "=" not in spec:
                raise SystemExit(f"--arm expects NAME=/path/to.onnx, got {spec!r}")
            name, _, path = spec.partition("=")
            if name in arms:
                raise SystemExit(f"arm {name!r} given twice")
            arms[name] = Path(path)
        return arms
    unknown = [k for k in a.arms if k not in ARMS]
    if unknown:
        raise SystemExit(
            f"unknown arm(s) {unknown}; known: {sorted(ARMS)}. Use --arms-json for new ones."
        )
    return {k: ARMS[k] for k in a.arms}


def one(
    arm: str,
    onnx: Path,
    lam: float,
    seed: int,
    out: Path,
    channels: str | None = None,
    clip: str = CLIP,
    digest: str | None = None,
    py: str = PY,
    full_clip: bool = False,
) -> tuple[str, float, int, dict]:
    d = out / "runs" / arm / f"lam{lam:g}"
    d.mkdir(parents=True, exist_ok=True)
    js = d / f"seed{seed}.json"
    stamp = d / f"seed{seed}.onnx_sha256"
    if js.is_file():
        # Reuse only when the cached result provably came from THIS checkpoint
        # and THIS clip. A missing stamp is a pre-guard receipt: recompute.
        cached = stamp.read_text().strip() if stamp.is_file() else None
        if (
            digest is not None
            and cached
            == f"{digest} {clip} {channels or 'all'} {'full' if full_clip else 'scored'}"
        ):
            return arm, lam, seed, json.loads(js.read_text())["result"]
        js.unlink()
    env = dict(os.environ, MUJOCO_GL="egl", PYOPENGL_PLATFORM="egl")
    cmd = (
        [
            py,
            str(PLAYER),
            "--onnx",
            str(onnx),
            "--clip",
            clip,
            "--out",
            str(d / f"seed{seed}.mp4"),
            "--lam",
            f"{lam:g}",
            "--seed",
            str(seed),
            "--no-video",
        ]
        + (["--full-clip"] if full_clip else [])
        + (["--channels", channels] if channels else [])
    )
    proc = subprocess.run(
        cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
    )
    if proc.returncode != 0 or not js.is_file():
        return arm, lam, seed, {"fell": None, "t_end": None, "error": True}
    if digest is not None:
        # The channel mask belongs in the stamp: it zeroes whole physics
        # channels at the same lambda, and it appears nowhere in the path.
        stamp.write_text(
            f"{digest} {clip} {channels or 'all'} {'full' if full_clip else 'scored'}\n"
        )
    return arm, lam, seed, json.loads(js.read_text())["result"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--lams", type=float, nargs="+", default=[0, 0.5, 1.0, 1.5, 2.0])
    ap.add_argument("--seeds", type=int, default=32)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument(
        "--arms", nargs="+", default=list(ARMS), help="names from the built-in table"
    )
    ap.add_argument(
        "--arms-json",
        type=str,
        default=None,
        help='JSON file mapping {"arm name": "/abs/path/..._g1.onnx"}',
    )
    ap.add_argument(
        "--arm",
        action="append",
        default=None,
        metavar="NAME=PATH",
        help="repeatable; overrides/extends --arms-json",
    )
    ap.add_argument(
        "--clip", type=str, default=CLIP, help="reference clip .pkl to track"
    )
    ap.add_argument(
        "--py", type=str, default=PY, help="interpreter that runs the player"
    )
    ap.add_argument(
        "--channels", type=str, default=None, help="comma list; default all six"
    )
    ap.add_argument(
        "--full-clip",
        action="store_true",
        help="keep stepping past the 0.5 m drift threshold to the end of the motion. "
        "The scored `fell` flag is unchanged, but a rollout that is no longer cut "
        "short at ~1.4 s can go on to actually topple, so this is the window that "
        "answers 'does it stay on its feet for the whole clip'",
    )
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    arms = resolve_arms(a)
    if not arms:
        print("no arms selected", file=sys.stderr)
        return 1
    if not Path(a.clip).is_file():
        print(f"missing clip: {a.clip}", file=sys.stderr)
        return 1
    for k, path in arms.items():
        if not path.is_file():
            print(f"missing onnx for {k}: {path}", file=sys.stderr)
            return 1
    digests = {k: sha256(path) for k, path in arms.items()}
    jobs = [
        (arm, onnx, lam, seed)
        for lam in a.lams
        for arm in arms
        for seed in range(1, a.seeds + 1)
        for onnx in [arms[arm]]
    ]
    table: dict = {arm: {f"{lam:g}": {} for lam in a.lams} for arm in arms}
    done = 0
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [
            ex.submit(
                one,
                arm,
                onnx,
                lam,
                seed,
                a.out,
                a.channels,
                a.clip,
                digests[arm],
                a.py,
                a.full_clip,
            )
            for arm, onnx, lam, seed in jobs
        ]
        for f in as_completed(futs):
            arm, lam, seed, res = f.result()
            table[arm][f"{lam:g}"][str(seed)] = res
            done += 1
            if done % 50 == 0:
                print(f"  {done}/{len(jobs)}", file=sys.stderr, flush=True)
    (a.out / "sweep.json").write_text(
        json.dumps(
            {
                "arms": {k: str(v) for k, v in arms.items()},
                "arm_sha256": digests,
                "clip": a.clip,
                "full_clip": bool(a.full_clip),
                "labels": LABEL,
                "channels": a.channels or "all",
                "lams": a.lams,
                "seeds": a.seeds,
                "table": table,
            },
            indent=1,
        )
    )

    # Two tables, because `fell` conflates two different events: a rollout ends
    # when pelvis-to-reference distance passes 0.5 m, which a policy trips while
    # walking perfectly well if it has drifted off the path. The first table is
    # the scored criterion; the second separates toppling from drifting using
    # the pelvis height recorded at termination.
    def row(arm, fmt):
        cells = []
        for lam in a.lams:
            rs = [
                r for r in table[arm][f"{lam:g}"].values() if r.get("fell") is not None
            ]
            cells.append(fmt(rs))
        return f"| {LABEL.get(arm, arm)} | " + " | ".join(cells) + " |"

    def scored(rs):
        ok = sum(1 for r in rs if not r["fell"])
        tf = [r["t_end"] for r in rs if r["fell"]]
        mt = f", stop {sum(tf) / len(tf):.1f}s" if tf else ""
        return f"{ok}/{len(rs)} ({100 * ok / max(1, len(rs)):.0f}%{mt})"

    def split(rs):
        toppled = sum(1 for r in rs if r.get("outcome") == "fell")
        drifted = sum(1 for r in rs if r.get("outcome") == "drifted")
        done = sum(1 for r in rs if r.get("outcome") == "completed")
        if toppled + drifted + done != len(rs):
            return "n/a (pre-outcome receipts)"
        return f"fell {toppled} · drifted {drifted} · tracked {done}"

    head = [
        "| arm | " + " | ".join(f"λ {lam:g}" for lam in a.lams) + " |",
        "|---|" + "---|" * len(a.lams),
    ]
    lines = [
        "### Scored criterion (pelvis-to-reference > 0.5 m ends the rollout)",
        "",
        *head,
    ]
    lines += [row(arm, scored) for arm in arms]
    lines += [
        "",
        "### What actually happened (fell over vs drifted off the path while upright)",
        "",
        *head,
    ]
    lines += [row(arm, split) for arm in arms]
    (a.out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
