"""Headless scene/camera/reset check; no model API or benchmark score.

Inside a compatible YAM runtime:
  /isaac-sim/python.sh run.py --out /outputs/scene-check
"""

import argparse
from dataclasses import asdict
import fcntl
import json
import os
from pathlib import Path
import sys
import subprocess
import threading
import time
import traceback

import task


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--mode", choices=("check", "demo"), default="check")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--interactive", action="store_true", help="Development: accept next trial via output-folder command.json")
    parser.add_argument("--physics-only", action="store_true", help="Development demo check without cameras or video")
    parser.add_argument("--expected-gpu-uuid", help="Host launcher's verified single-GPU UUID")
    args = parser.parse_args()
    if args.steps < 40:
        parser.error("--steps must be at least 40 for the stability check")
    if args.physics_only and args.mode != "demo":
        parser.error("--physics-only applies to demo development")
    if not args.out.resolve().is_relative_to(Path("/outputs")):
        parser.error("Write runtime results below the personal /outputs mount")
    visible = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=uuid", "--format=csv,noheader"], text=True
    ).strip().splitlines()
    if len(visible) != 1 or (args.expected_gpu_uuid and visible != [args.expected_gpu_uuid]):
        parser.error("Exactly the expected single GPU must be visible before starting Isaac Sim")
    return args


def check_scene(env, specs, args, report):
    import cv2
    import numpy as np
    import torch
    from PIL import Image, ImageDraw
    from yam_arena.constants import READY
    from yam_arena.scene import object_states

    base = env.unwrapped
    import omni.usd
    from pxr import UsdLux

    stage = omni.usd.get_context().get_stage()
    report["lights"] = [
        {
            "path": str(prim.GetPath()), "type": prim.GetTypeName(),
            "intensity": float(UsdLux.LightAPI(prim).GetIntensityAttr().Get()),
            "color": list(UsdLux.LightAPI(prim).GetColorAttr().Get()),
        }
        for prim in stage.Traverse() if prim.HasAPI(UsdLux.LightAPI)
    ]
    ready = torch.tensor([[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=base.device)
    names = tuple(s.name for s in specs)
    movable = tuple(s.name for s in specs if s.movable)
    collider_vertices = {s.name: task.collision_vertices(s) for s in specs if s.movable}
    checks = report["checks"]

    def numpy(value):
        value = value.torch if hasattr(value, "torch") else value
        return value.detach().cpu().numpy()

    def states():
        values = object_states(env, names)
        result = {}
        for name, state in values.items():
            result[name] = {
                "center": state.pos.tolist(), "rotation_xyzw": state.quat_xyzw.tolist(),
                "linear_velocity": state.lin_vel.tolist(), "angular_velocity": state.ang_vel.tolist(),
                "bounds_low": state.low.tolist(), "bounds_high": state.high.tolist(),
            }
            if name in movable:
                result[name]["root_pose"] = numpy(base.scene[name].data.root_link_pose_w)[0].tolist()
        return result

    def capture(label):
        frames = {}
        for name in task.POLICY_CAMERAS:
            frame = numpy(base.scene[name].data.output["rgb"])[0, ..., :3].astype(np.uint8)
            if frame.ndim != 3 or frame.shape[-1] != 3 or frame.std() <= 5:
                raise ValueError(f"{name}: invalid or blank RGB camera frame")
            Image.fromarray(frame).save(args.out / f"{label}_{name}.png")
            policy = cv2.resize(frame, task.POLICY_SIZE)
            Image.fromarray(policy).save(args.out / f"{label}_{name}_policy.png")
            frames[name] = frame
            report.setdefault("cameras", {})[name] = {
                "native_shape": list(frame.shape), "policy_shape": list(policy.shape),
                "std": float(frame.std()), "mean": float(frame.mean()),
                "prim_path": base.scene[name].cfg.prim_path,
                "position_world": numpy(base.scene[name].data.pos_w)[0].tolist(),
                "rotation_ros_xyzw": numpy(base.scene[name].data.quat_w_ros)[0].tolist(),
            }
        checks[f"{label}_camera_views_differ"] = all(
            np.abs(frames[a].astype(float) - frames[b].astype(float)).mean() > 5
            for a, b in (("top_cam", "left_cam"), ("top_cam", "right_cam"), ("left_cam", "right_cam"))
        )
        if label == "settled":
            width, height = frames["top_cam"].shape[1], frames["top_cam"].shape[0]
            sheet = Image.new("RGB", (width * 2, (height + 26) * 2), "#20252b")
            draw = ImageDraw.Draw(sheet)
            for name, x, y in (("top_cam", 0, 0), ("left_cam", 0, height + 26), ("right_cam", width, height + 26)):
                draw.text((x + 8, y + 6), name, fill="white")
                sheet.paste(Image.fromarray(frames[name]), (x, y + 26))
            draw.multiline_text((width + 18, 40),
                "Isaac Sim: local YAM top-camera view\n\nInstruction: Put the knife into the holder.\n\n"
                f"Arms held at READY.\n{args.steps}-step scene check; no model policy.\n\n"
                "Native renders + 320 x 176 policy images.", fill="white", spacing=8)
            sheet.save(args.out / "camera_views.png")

    def reset_and_settle():
        env.reset()
        for _ in range(round(0.5 * task.CONTROL_HZ)):
            env.step(ready)
        for _ in range(4):
            base.sim.render()
        return states()

    report["first_reset"] = reset_and_settle()
    capture("initial")
    midway = None
    for index in range(args.steps):
        env.step(ready)
        if index + 1 == args.steps // 2:
            midway = states()
        if (index + 1) % 40 == 0:
            print(f"SCENE_CHECK_STEP {index + 1}/{args.steps}", flush=True)
    report["settled"] = settled = states()
    capture("settled")
    table_top = json.loads((task.ROOT / "layout.json").read_text())["table_top_z"]
    from yam_arena.urdf import quat_xyzw_to_matrix

    for name in movable:
        value = settled[name]
        drift = float(np.linalg.norm(np.array(value["root_pose"][:3]) - midway[name]["root_pose"][:3]))
        speed = float(np.linalg.norm(value["linear_velocity"]))
        spin = float(np.linalg.norm(value["angular_velocity"]))
        report.setdefault("stability", {})[name] = {"drift_m": drift, "speed_m_s": speed, "spin_rad_s": spin}
        checks[f"{name}_stable"] = drift < 0.002 and speed < 0.01 and spin < 0.5
        pose = np.array(value["root_pose"])
        vertices = collider_vertices[name] @ quat_xyzw_to_matrix(pose[3:]).T + pose[:3]
        value["collision_bounds_low"] = vertices.min(axis=0).tolist()
        value["collision_bounds_high"] = vertices.max(axis=0).tolist()
        checks[f"{name}_above_table"] = float(vertices[:, 2].min()) >= table_top - 0.003
        checks[f"{name}_finite"] = bool(np.isfinite(value["root_pose"]).all())

    q = np.array(settled["hamster"]["root_pose"][3:])
    checks["hamster_upright"] = float(quat_xyzw_to_matrix(q)[2, 2]) > 0.95
    report["second_reset"] = second = reset_and_settle()
    capture("reset")
    for name in movable:
        first = report["first_reset"][name]["root_pose"]
        after = second[name]["root_pose"]
        translation_error = float(np.linalg.norm(np.array(first[:3]) - after[:3]))
        angle_error = float(2 * np.arccos(np.clip(abs(np.dot(first[3:], after[3:])), 0.0, 1.0)))
        report.setdefault("reset_repeatability", {})[name] = {
            "position_error_m": translation_error, "rotation_error_rad": angle_error,
        }
        checks[f"{name}_repeatable_reset"] = translation_error < 0.002 and angle_error < 0.035
    checks["native_camera_size"] = all(c["native_shape"] == [360, 640, 3] for c in report["cameras"].values())
    checks["policy_camera_size"] = all(c["policy_shape"] == [176, 320, 3] for c in report["cameras"].values())
    checks["all_seven_task_objects_loaded"] = len(settled) == 7
    report["passed"] = all(checks.values())


def main():
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    # Serialize this runner across all docker exec sessions in the personal container.
    lock = Path("/outputs/.scene-run.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit("A scene runner is already active in this container")
    (args.out / "runner.pid").write_text(str(os.getpid()) + "\n")
    report = {
        "passed": False, "checks": {}, "instruction": task.INSTRUCTION,
        "mode": args.mode, "control_hz": task.CONTROL_HZ, "physics_hz": 240,
        "control_steps": args.steps if args.mode == "check" else 0,
        "physical_gpu": int(os.environ["YAM_HOST_GPU_INDEX"]) if "YAM_HOST_GPU_INDEX" in os.environ else None,
        "policy": "hold_ready", "dream_api_calls": 0,
        "camera_rig": "station/holder-visibility", "camera_view": task.CAMERA_VIEW, "object_jitter_m": 0.0,
        "lighting": task.LIGHTING,
        "rendered": not args.physics_only,
    }
    app = env = None
    code = 1
    started = time.monotonic()
    try:
        # All scene assets are local. Avoid starting an unnecessary cache daemon.
        os.environ["OMNICLIENT_HUB_MODE"] = "disabled"
        from isaaclab.app import AppLauncher

        print("SCENE_CHECK_START: starting Isaac Sim with calibrated cameras", flush=True)
        app = AppLauncher(
            headless=True, enable_cameras=not args.physics_only, device="cuda:0", multi_gpu=False,
            kit_args=f"--portable-root {Path.home()/'kit'} "
            "--/renderer/multiGpu/enabled=false --/renderer/multiGpu/autoEnable=false "
            "--/renderer/multiGpu/maxGpuCount=1 --/renderer/activeGpu=0 "
            "--/plugins/carb.tasking.plugin/threadCount=8",
        ).app
        print("SCENE_CHECK_BUILD: loading local assets", flush=True)
        env, specs = task.build_environment(enable_cameras=not args.physics_only)
        report["objects"] = [{**asdict(s), "asset": str(s.asset)} for s in specs]
        if args.mode == "demo":
            run_demonstrations(env, specs, args, report)
        else:
            check_scene(env, specs, args, report)
        code = 0 if report["passed"] else 2
    except BaseException:
        report["error"] = traceback.format_exc()
        traceback.print_exc()
    finally:
        report["elapsed_s"] = round(time.monotonic() - started, 2)
        filename = "demo_report.json" if args.mode == "demo" else "scene_check.json"
        (args.out / filename).write_text(json.dumps(report, indent=2) + "\n")
        print("SCENE_CHECK_RESULT", json.dumps({"passed": report["passed"], "checks": report["checks"]}), flush=True)

        def force_exit():
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(code)

        # Kit shutdown can stall; bound it without affecting the idle container.
        timer = threading.Timer(25, force_exit)
        timer.daemon = True
        timer.start()
        try:
            if env is not None:
                env.close()
            if app is not None:
                app.close()
        finally:
            timer.cancel()
    return code


def run_demonstrations(env, specs, args, report):
    import importlib
    import numpy as np
    import torch
    from PIL import Image
    from yam_arena.constants import READY
    from demo_runtime import StateReader, VideoRecorder
    import demo

    reader = StateReader(env, specs)
    report["scoring_rules"] = reader.scorer.configuration()
    report["episodes"] = []
    report["policy"] = "local_scripted_physical_demo"
    command = {"parameters": {}, "record": True}
    index = 0
    while True:
        folder = args.out / f"episode-{index:02d}"
        folder.mkdir(exist_ok=True)
        env.reset()
        reader.scorer.reset()
        ready = torch.tensor([[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=reader.base.device)
        for _ in range(20):
            env.step(ready)
        if not args.physics_only:
            for _ in range(4):
                reader.base.sim.render()
            Image.fromarray(reader.rgb("top_cam")).save(folder / "setup.png")
        recorder = VideoRecorder(reader, folder) if command.get("record", True) and not args.physics_only else None
        policy = demo.Demo(env, reader, recorder=recorder, output=folder, parameters=command.get("parameters"))
        result = {"passed": False, "score": 0.0, "episode": index}
        try:
            if recorder:
                recorder.capture()
            outcome = policy.run()
            result.update(outcome)
            result["passed"] = outcome["score"] == 1.0
        except Exception:
            result["error"] = traceback.format_exc()
            traceback.print_exc()
            result.update(reader.score())
        finally:
            if recorder:
                recorder.close()
                result["video_frames"] = recorder.frames
            result.update(policy.report())
            if not args.physics_only:
                Image.fromarray(reader.rgb("top_cam")).save(folder / "final.png")
            (folder / "episode.json").write_text(json.dumps(result, indent=2) + "\n")
        report["episodes"].append({k: v for k, v in result.items() if k not in ("trajectory", "phases")})
        report["control_steps"] = max(e["control_steps"] for e in report["episodes"])
        report["control_steps_total"] = sum(e["control_steps"] for e in report["episodes"])
        report["passed"] = any(e["passed"] for e in report["episodes"])
        report["checks"] = {"full_score_demonstrated": report["passed"]}
        (args.out / "demo_report.json").write_text(json.dumps(report, indent=2) + "\n")
        print("DEMO_TRIAL_RESULT", json.dumps({k: result.get(k) for k in ("episode", "passed", "score", "control_steps", "error")}), flush=True)
        index += 1
        if args.interactive:
            request = args.out / "command.json"
            print("DEMO_WAITING_FOR_COMMAND", str(request), flush=True)
            while not request.exists():
                time.sleep(0.5)
            command = json.loads(request.read_text())
            request.unlink()
            if command.get("command") == "stop":
                break
            demo = importlib.reload(demo)
        elif index >= args.episodes:
            break


if __name__ == "__main__":
    raise SystemExit(main())
