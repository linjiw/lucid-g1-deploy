"""Evaluate the rank-9 LAFAN dance policy in MuJoCo and log to W&B online."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import wandb

ROOT = Path(__file__).resolve().parents[1]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lam", type=float, choices=(0, 1), default=0)
    parser.add_argument("--seed", type=int, default=8700)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    config_path = ROOT / "config/lafan_rank9_20260922.json"
    config = json.loads(config_path.read_text())
    model, clip = ROOT / config["policy"], ROOT / config["clip"]
    assert sha(model) == config["onnx_sha256"], "Policy hash mismatch"
    assert sha(clip) == config["clip_sha256"], "Motion hash mismatch"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")
    out = (args.out or ROOT / "results" / f"lafan_rank9_{stamp}").resolve()
    out.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        str(ROOT / "tools/mujoco_player.py"),
        "--onnx",
        str(model),
        "--clip",
        str(clip),
        "--out",
        str(out / "rollout.mp4"),
        "--full-clip",
        "--lam",
        str(args.lam),
        "--seed",
        str(args.seed),
        "--width",
        "640",
        "--height",
        "480",
    ]
    if args.no_video:
        command.append("--no-video")
    run = wandb.init(
        entity="16726",
        project="lucid-sonic",
        group=config["group"],
        name=f"SONIC/eval/fixed/s8600/h1000/rank9/lam{args.lam:g}/s{args.seed}",
        mode="online",
        dir=str(out),
        config={
            **config,
            "stage": "eval",
            "arm": "fixed",
            "evaluation_seed": args.seed,
            "evaluation_condition": f"reference-init MuJoCo lambda={args.lam:g}",
            "evaluator_sha256": sha(Path(__file__)),
            "player_sha256": sha(ROOT / "tools/mujoco_player.py"),
        },
    )
    started = time.monotonic()
    try:
        with (out / "player.log").open("w") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        report = json.loads((out / "rollout.json").read_text())
        result = report["result"]
        run.log({k: v for k, v in result.items() if isinstance(v, (int, float, bool))})
        run.summary["outcome"] = result["outcome"]
        run.summary["runtime_seconds"] = time.monotonic() - started
        run.summary["metrics_sha256"] = sha(out / "rollout.json")
        if not args.no_video:
            run.log(
                {
                    "simulation_video": wandb.Video(
                        str(out / "rollout.mp4"), format="mp4"
                    )
                }
            )
        receipt = {
            "result": result,
            "wandb_url": run.url,
            "command": command,
            "runtime_seconds": time.monotonic() - started,
            "metrics_sha256": sha(out / "rollout.json"),
        }
        (out / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
        print(json.dumps(receipt, indent=2))
    except BaseException:
        run.finish(exit_code=1)
        raise
    else:
        run.finish()


if __name__ == "__main__":
    main()
