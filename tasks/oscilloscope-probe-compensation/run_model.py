#!/usr/bin/env python3
"""Evaluate one frozen task with Dream-1 and retain the Inspect recording."""

import argparse
import hashlib
import importlib.metadata as metadata
import json
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time


TASK = Path(__file__).resolve().parent
MODEL = "dreamlabs/dream-1-beta"
CONTROL_HZ = 20
CAMERAS = ("top_cam", "left_cam", "right_cam")


def registry():
    entries = metadata.entry_points(group="inspect_robots.embodiments")
    for name in ("oscilloscope_native", "resist_isaac"):
        matches = [entry for entry in entries if entry.name == name]
        if matches:
            if len(matches) != 1 or matches[0].value != "resist_isaac:embodiment":
                raise RuntimeError(f"Ambiguous or unexpected embodiment entry point: {name}")
            break
    else:
        raise RuntimeError("Register the task in the prepared environment with pip install --no-deps.")
    for group, key in (("policies", "dream"), ("scorers", "task_score")):
        registered = metadata.entry_points(group=f"inspect_robots.{group}", name=key)
        if len(registered) != 1:
            raise RuntimeError(f"Expected one registered Inspect {group} entry for {key}.")
    return name


def command(args, name, config):
    return [
        sys.executable, "-m", "inspect_robots.cli", "run",
        "--embodiment", name, "--policy", "dream",
        "-E", f"task={TASK}", "-E", "condition=recoverable",
        "-E", f"evidence_directory={args.output / 'native-evidence'}",
        "-E", f"device={args.device}", "-E", "headless=true",
        "-E", "object_jitter=0", "-E", "scene=station", "-E", f"control_hz={CONTROL_HZ}",
        "-E", f"cam_width={args.width}", "-E", f"cam_height={args.height}",
        "-E", "top_cam=calibrated", "-E", "wrist_cam_mount=link",
        "-P", f"model={MODEL}", "-P", "preroll_steps=32", "-P", "rows_per_request=48",
        "-P", "dry_run=false", "-P", "record=false",
        "--instruction", config["instruction"], "--max-steps", str(config["max_steps"]),
        "--scorer", "task_score", "--epochs", "1", "--seed", "0", "--grader", "none",
        "--no-prompt", "--store-frames", "--no-rerun", "--no-rerun-save",
        "--fail-on-error", "1", "--config", os.devnull, "--log-dir", str(args.output / "inspect"),
    ]


def terminal_result(output, config):
    for path in sorted((output / "inspect").glob("*.json")):
        data = json.loads(path.read_text())
        if (path.name.endswith(".live.json") or data.get("status") != "success"
                or data.get("results", {}).get("errored_trials") != 0):
            continue
        run, stats, samples = data["eval"], data["stats"], data["samples"]
        if (run["policy"] != "dream" or run["policy_checkpoint"] != MODEL
                or run["embodiment_info"]["control_hz"] != CONTROL_HZ or run["seed"] != 0
                or run["max_steps"] != config["max_steps"] or len(samples) != 1
                or samples[0]["instruction"] != config["instruction"]
                or len(samples[0]["epochs"]) != 1 or not stats["completed_at"]):
            raise RuntimeError(f"Unexpected trial metadata in {path}.")
        score = samples[0]["epochs"][0]["task_score"]
        steps = stats["total_steps"]
        if (not math.isfinite(score) or not 0 <= score <= 1
                or data["results"]["metrics"]["task_score"] != score
                or not 0 < steps <= config["max_steps"]):
            raise RuntimeError(f"Incomplete or inconsistent score in {path}.")
        frame_dir = Path(stats["frames_dir"])
        if not frame_dir.is_absolute():
            frame_dir = path.parent / frame_dir
        for camera in CAMERAS:
            frames = sorted(int(p.stem.rsplit("_", 1)[-1]) for p in frame_dir.glob(f"*_{camera}_*.npy"))
            if frames != list(range(steps + 1)):
                raise RuntimeError(f"Incomplete {camera} recording in {frame_dir}.")
        return {
            "inspect_log": str(path),
            "inspect_log_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "frames_dir": str(frame_dir),
            "frames_per_camera": steps + 1,
            "actual_model_score": score,
            "actual_steps": steps,
            "termination_reasons": samples[0]["termination_reasons"],
        }
    raise RuntimeError("No successful terminal Inspect JSON log. Check model-stdout.log.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Fresh run directory outside the frozen task.")
    parser.add_argument("--expected-hostname", default="visnu")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.output == TASK or TASK in args.output.parents:
        parser.error("The run directory must be outside the task.")
    if args.width <= 0 or args.height <= 0:
        parser.error("Camera dimensions must be positive.")
    hostname = socket.gethostname()
    if hostname != args.expected_hostname:
        raise RuntimeError(f"Expected {args.expected_hostname!r}, got {hostname!r}.")
    if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"YES", "Y", "TRUE", "1"}:
        raise RuntimeError("Inherit the operator's existing NVIDIA terms acceptance setting.")

    sys.path.insert(0, str(TASK / "native"))
    from resist_isaac.runner import git_metadata, source_files, source_snapshot

    name = registry()
    config = json.loads((TASK / "pilot.json").read_text())
    if not config["instruction"].strip() or type(config["max_steps"]) is not int or config["max_steps"] <= 0:
        raise ValueError("Pilot must supply the trial instruction and positive integer action limit.")
    versions = {package: metadata.version(package) for package in (
        "isaacsim", "isaaclab", "isaaclab-arena", "yam-arena",
        "inspect-robots", "inspect-robots-dream", "dreamlabs-client",
    )}
    gpu = subprocess.check_output([
        "nvidia-smi", "--query-gpu=index,name,uuid,driver_version",
        "--format=csv,noheader",
    ], text=True, timeout=15)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frame_bytes = (config["max_steps"] + 1) * len(CAMERAS) * args.width * args.height * 3
    if shutil.disk_usage(args.output.parent).free < frame_bytes + 1_000_000_000:
        raise RuntimeError(f"Need room for up to {frame_bytes / 1e9:.1f} GB of RGB frames plus run data.")
    args.output.mkdir(exist_ok=False)
    source_hashes = source_snapshot(TASK, args.output)
    current_hashes = lambda: {
        p.relative_to(TASK).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in source_files(TASK)
    }
    if current_hashes() != source_hashes:
        raise RuntimeError("Task changed during source capture. Use a frozen source folder.")
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1")
    environment["PYTHONPATH"] = os.pathsep.join(
        value for value in (str(TASK / "native"), environment.get("PYTHONPATH")) if value
    )
    environment["DREAM_LOG_DIR"] = str(args.output / "dream-logs")
    Path(environment["DREAM_LOG_DIR"]).mkdir()
    argv = command(args, name, config)
    receipt = {
        "status": "starting", "hostname": hostname, "versions": versions, "gpu": gpu,
        "policy": "dream", "model": MODEL, "command": argv, "source_files_sha256": source_hashes,
        **git_metadata(TASK, args.output),
    }
    receipt_path = args.output / "launch-receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    started = time.monotonic()
    try:
        with (args.output / "model-stdout.log").open("wb") as log:
            process = subprocess.run(
                argv, cwd=TASK, env=environment, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT,
            )
        receipt["exit_code"] = process.returncode
        if process.returncode != 0:
            raise RuntimeError(f"Inspect exited with status {process.returncode}. Check model-stdout.log.")
        receipt.update(terminal_result(args.output, config))
        receipt["status"] = "completed"
    except (Exception, KeyboardInterrupt) as error:
        receipt.update(status="error", wrapper_error=f"{type(error).__name__}: {error}")
        raise
    finally:
        receipt["elapsed_wall_s"] = time.monotonic() - started
        receipt["source_unchanged"] = current_hashes() == source_hashes
        if not receipt["source_unchanged"]:
            receipt["status"] = "source_changed"
        receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    if not receipt["source_unchanged"]:
        raise RuntimeError("The frozen task source changed during evaluation.")
    print(receipt_path)


if __name__ == "__main__":
    main()
