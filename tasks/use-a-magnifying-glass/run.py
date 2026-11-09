"""Run the identical magnifier scene with a scripted demo or ordinary Dream-1.

The demo alone uses measured tool pose to correct robot joint commands. Dream
receives only the normal YAM RGB/joint observation and the instruction below.
Lens-hidden reference images are collected after control with physics frozen.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import os
from pathlib import Path
import shutil
import time

import numpy as np

if __package__:
    from .common import array, author_card, review_camera, table_clearance, tool_vertices
else:
    from common import array, author_card, review_camera, table_clearance, tool_vertices

HERE = Path(__file__).resolve().parent
CONTROL_HZ = 20
RUN_LIMIT = 1200
ALIGN_STEPS = 60
HOLD_STEPS = 100
INSTRUCTION = (
    "Pick up the magnifying glass by its handle. Position and orient it relative "
    "to the left wrist camera and the printed target until both groups of five "
    "bars are enlarged and clearly separated. Finish by holding that view for "
    "three seconds."
)
RUNTIME_FILES = (
    "run.py", "common.py", "dream_policy.py", "pickup_scene.py", "tool.py", "lens.py",
    "align.py", "inspection_target.py", "inspection_measure.py", "measure.py", "target.py", "score_hold.py",
    "scoring.py", "test_runner.py", "JUDGING.md", "README.md", "task.json",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_plan(path):
    plan = json.loads(Path(path).read_text())
    if plan.get("control_hz") != CONTROL_HZ:
        raise ValueError("The frozen demo plan must use 20 Hz")
    frames = plan.get("frames", [])
    if len(frames) < 2:
        raise ValueError("Demo plan must include an initial state and actions")
    for frame in frames:
        action = np.asarray(frame["action14"], dtype=float)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise ValueError("Every frozen action must have 14 finite targets")
    if len(frames)-1+ALIGN_STEPS+HOLD_STEPS > RUN_LIMIT:
        raise ValueError("Demo exceeds the task run limit")
    return plan


def freeze_source(output, plan_path):
    destination = output/"task-source"
    destination.mkdir()
    hashes = {}
    for name in RUNTIME_FILES:
        source = HERE/name
        if source.is_file():
            shutil.copyfile(source, destination/name)
            hashes[name] = sha256(source)
    shutil.copyfile(plan_path, destination/"motion-plan.json")
    hashes["motion-plan.json"] = sha256(plan_path)
    return hashes


def package_versions():
    result = {}
    for name in ("isaaclab", "isaacsim", "inspect-robots", "inspect-robots-yam",
                 "inspect-robots-dream", "dreamlabs-client", "numpy", "opencv-python",
                 "Pillow", "imageio", "imageio-ffmpeg"):
        try:
            result[name] = version(name)
        except PackageNotFoundError:
            result[name] = None
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("demo", "dream"), default="demo")
    parser.add_argument("--out", "--log-dir", type=Path, required=True)
    parser.add_argument("--target-dir", type=Path,
                        help="Optional pre-generated V2 chart; default: generate in the run directory")
    parser.add_argument("--plan", type=Path, default=HERE/"motion-plan.json")
    parser.add_argument("--max-steps", type=int, default=RUN_LIMIT)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--dream-rows-per-request", type=int, default=8, choices=range(1, 49))
    args = parser.parse_args()
    if not 1 <= args.max_steps <= RUN_LIMIT:
        parser.error("max-steps must be between 1 and 1200")
    if not 0 <= args.seed < 2**32:
        parser.error("seed must be a 32-bit unsigned integer")
    plan = load_plan(args.plan)
    args.out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    request_audit, session_audit = [], {"session_count": 0}
    record = {
        "schema": "magnifier-trial-v1", "policy": args.policy, "complete": False,
        "instruction": INSTRUCTION, "seed": args.seed, "control_hz": CONTROL_HZ,
        "run_limit": RUN_LIMIT, "max_steps": args.max_steps, "steps": 0,
        "source_sha256": freeze_source(args.out, args.plan), "plan_sha256": sha256(args.plan),
        "versions": package_versions(), "dream_requests": request_audit,
        "dream_sessions": session_audit,
        "policy_observation": {
            "cameras": ["top_cam", "left_cam", "right_cam"],
            "native_rgb_shape": [360, 640, 3], "wrapper_rgb_shape": [224, 224, 3],
            "state": "14 measured joint/gripper values; no object poses or assessor state",
            "dream_adapter_encoding": "Normal adapter: 224x224 observations resized to 320x176 JPEG quality 85",
        },
        "demo_feedback": "Demo-only measured tool pose corrects right-arm joint targets; no tool motion is imposed.",
        "table_top_z_m": .7475,
    }

    def save():
        record["wall_seconds"] = time.monotonic()-started
        temporary = args.out/"pickup.json.tmp"
        temporary.write_text(json.dumps(record, indent=2)+"\n")
        temporary.replace(args.out/"pickup.json")

    save()
    target_dir = args.out/"target"
    target_dir.mkdir()
    if args.target_dir is None:
        if __package__:
            from .inspection_target import generate
        else:
            from inspection_target import generate
        generate(target_dir)
    else:
        for name in ("target.json", "target.png"):
            shutil.copyfile(args.target_dir/name, target_dir/name)
    target = json.loads((target_dir/"target.json").read_text())
    if target.get("schema") != "magnifier-inspection-target-v2":
        raise ValueError("Use the validated V2 inspection chart")
    if target.get("sha256") != sha256(target_dir/"target.png"):
        raise ValueError("Chart texture differs from its metadata")
    record["target_sha256"] = {name: sha256(target_dir/name) for name in ("target.json", "target.png")}
    save()

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True).app
    video = wrist_video = trace = dream_policy = None
    hold_frames = deque(maxlen=61)
    phase = "scene initialization"
    frames_saved = False
    try:
        import carb
        import cv2
        import imageio.v2 as imageio
        import omni.usd
        from PIL import Image
        import torch
        from pxr import Usd, UsdGeom, UsdPhysics
        from isaaclab_physx.renderers import IsaacRtxRendererCfg, IsaacRtxRendererGlobalSettingsCfg
        from yam_arena import embodiment as env_module, inspect_embodiment as wrapper, scene
        from yam_arena.analysis import pose7_to_matrix
        from yam_arena.constants import TCP_OFFSET
        from yam_arena.urdf import quat_xyzw_to_matrix
        if __package__:
            from .tool import attach_tool
        else:
            from tool import attach_tool

        np.random.seed(args.seed)
        torch.manual_seed(args.seed)
        settings = carb.settings.get_settings()
        settings.set_bool("/isaaclab/render/deterministic", False)
        overrides = {"/rtx/rendermode": "RealTimePathTracing", "/rtx/rtpt/maxBounces": 6,
                     "/rtx/rtpt/maxSpecularAndTransmissionBounces": 6,
                     "/rtx/translucency/enabled": True, "/rtx/translucency/maxRefractionBounces": 6}
        env_module.RENDERER = IsaacRtxRendererCfg(global_settings=IsaacRtxRendererGlobalSettingsCfg(
            ambient_light_intensity=0., antialiasing_mode=None, enable_dlssg=False, carb_settings=overrides))
        wrapper._APP = app
        original_object, original_env = scene.task_object, scene.make_env

        def task_object(spec):
            obj = original_object(spec)
            if spec.name == "magnifier":
                spawn = obj.spawner_cfg.func

                def physical_spawn(prim_path, cfg, *a, **kw):
                    prim = spawn(prim_path, cfg, *a, **kw)
                    record["tool"] = attach_tool(omni.usd.get_context().get_stage(), str(prim.GetPath()))
                    record["tool_root_path"] = str(prim.GetPath())
                    return prim

                obj.spawner_cfg.func = physical_spawn
                obj.object_cfg.spawn.func = physical_spawn
            return obj

        def make_env(embodiment, **kw):
            review_camera(embodiment)
            return original_env(embodiment, **kw)

        scene.task_object, scene.make_env = task_object, make_env
        emb = wrapper.YamArenaEmbodiment(task=str(HERE/"pickup_scene.py"), object_jitter=0.,
                                         control_hz=CONTROL_HZ, cam_width=224, cam_height=224)
        try:
            emb._ensure_sim(None)
        finally:
            scene.task_object, scene.make_env = original_object, original_env
        base = emb._base
        stage = omni.usd.get_context().get_stage()
        author_card(stage, "/World/InspectionPrint", target_dir/"target.png", .120, .100,
                    (.35, .05, .7551), np.diag([-1., -1., 1.]))
        emb._instruction = INSTRUCTION
        # Preserve pickup-005's reset/ten settling actions exactly in both modes.
        emb._env.reset()
        for _ in range(10):
            emb._env.step(emb._ready)
        emb._t = 0.
        root = stage.GetPrimAtPath(record["tool_root_path"])
        record["rigid_bodies"] = [str(p.GetPath()) for p in Usd.PrimRange(root)
                                  if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        record["joints_in_tool"] = [str(p.GetPath()) for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]
        if len(record["rigid_bodies"]) != 1 or record["joints_in_tool"]:
            raise RuntimeError("The magnifier must be one ordinary rigid body without attachment joints")
        record["renderer_settings"] = {k: settings.get(k) for k in overrides}
        record["renderer_flags"] = {"isaaclab_deterministic": settings.get("/isaaclab/render/deterministic"),
                                    "antialiasing_mode": None, "enable_dlssg": False,
                                    "ambient_light_intensity": 0.}
        if settings.get("/rtx/rendermode") != "RealTimePathTracing":
            raise RuntimeError("Unexpected renderer mode")
        if not np.allclose(array(base.arena_world.get_pose_w("magnifier")),
                           array(base.scene["magnifier"].data.root_pose_w)):
            raise RuntimeError("Tool pose source disagrees with the rigid body")
        record["left_camera_mount"] = asdict(emb.rig.wrist_cams["left"])
        record["joint_names"] = {name: list(base.scene[name].joint_names)
                                  for name in ("left_robot", "right_robot")}
        vertices = tool_vertices()
        record["tool_clearance_geometry"] = {"vertex_count": len(vertices),
            "source": "All handle, rim and glass collision-solid vertices in the tool root frame"}
        for name, filename in (("review", "demo.mp4" if args.policy == "demo" else "dream.mp4"),
                               ("wrist", "wrist.mp4")):
            writer = imageio.get_writer(args.out/filename, fps=CONTROL_HZ, codec="libx264", pixelformat="yuv420p",
                                       ffmpeg_params=["-movflags", "+faststart"], macro_block_size=1)
            if name == "review":
                video = writer
            else:
                wrist_video = writer
        trace = (args.out/"trace.jsonl").open("w")

        def snapshot(step, label, action):
            pose = array(base.arena_world.get_pose_w("magnifier"))[0]
            target_pose = array(base.arena_world.get_pose_w("target_plate"))[0]
            links, tcp, joints = {}, {}, {}
            for name in ("left_robot", "right_robot"):
                robot = base.scene[name]
                measured = array(robot.data.body_link_pose_w)[0]
                links[name] = {n: p.tolist() for n, p in zip(robot.body_names, measured, strict=True)}
                joints[name] = array(robot.data.joint_pos)[0].tolist()
                gripper_pose = measured[robot.body_names.index("gripper")]
                g = pose7_to_matrix(gripper_pose)
                tcp[name] = [*(g[:3, :3]@np.asarray(TCP_OFFSET)+g[:3, 3]).tolist(),
                             *gripper_pose[3:].tolist()]
            g = pose7_to_matrix(np.asarray(links["left_robot"]["gripper"]))
            c = emb.rig.wrist_cams["left"]
            rotation = g[:3, :3]@quat_xyzw_to_matrix(np.asarray(c.rot_xyzw))
            eye = g[:3, :3]@np.asarray(c.pos)+g[:3, 3]
            observation = emb._observation()
            state = {"step": step, "time_s": step/CONTROL_HZ, "phase": label, "action": list(action),
                     "tool_pose_xyzw": pose.tolist(), "target_pose_xyzw": target_pose.tolist(),
                     "tool_table_clearance_m": table_clearance(vertices, pose), "joints": joints,
                     "measured_action14": np.asarray(observation.state["joint_pos"]).tolist(),
                     "robot_link_poses_xyzw": links, "robot_tcp_pose_xyzw": tcp,
                     "camera_eye": eye.tolist(), "camera_rotation": rotation.tolist(),
                     "physics_step_count": int(base.sim._physics_step_count)}
            raw = array(base.scene["left_cam"].data.output["rgb"])[0, :, :, :3].astype(np.uint8)
            policy_rgb = observation.images["left_cam"]
            if raw.shape != (360, 640, 3) or policy_rgb.shape != (224, 224, 3):
                raise RuntimeError("Unexpected native or policy camera resolution")
            if not np.array_equal(policy_rgb, cv2.resize(raw, (224, 224))):
                raise RuntimeError("Saved wrist RGB differs from the actual policy preprocessing")
            hold_frames.append((step, raw.copy(), policy_rgb.copy()))
            review = array(base.scene["review_cam"].data.output["rgb"])[0, :, :, :3].astype(np.uint8)
            video.append_data(review)
            wrist_video.append_data(policy_rgb)
            trace.write(json.dumps(state)+"\n")
            trace.flush()
            if step == 0:
                Image.fromarray(review).save(args.out/"setup.png")
                record["initial_state"] = state
            return state, observation

        state, observation = snapshot(0, "settled", emb._ready[0].cpu().tolist())
        save()
        if args.policy == "dream":
            from inspect_robots import Scene
            if __package__:
                from .dream_policy import make_policy
            else:
                from dream_policy import make_policy
            phase = "Dream session initialization"
            dream_policy = make_policy(request_audit, session_audit, rows_per_request=args.dream_rows_per_request)
            dream_policy.bind(emb.info)
            dream_policy.reset(Scene(id="magnifier", instruction=INSTRUCTION))
            record["dream_model"] = dream_policy.info.checkpoint
        else:
            if __package__:
                from .align import action14 as align_action
            else:
                from align import action14 as align_action

        planned = plan["frames"][1:]
        demo_steps = len(planned)+ALIGN_STEPS+HOLD_STEPS
        total_steps = min(args.max_steps, demo_steps) if args.policy == "demo" else args.max_steps
        action = state["action"]
        phase = "control"
        for step in range(1, total_steps+1):
            if dream_policy is not None:
                chunk = dream_policy.act(observation)
                if len(chunk.actions) != 1:
                    raise RuntimeError("Dream adapter must return one control action")
                requested = np.asarray(chunk.actions[0].data, dtype=float)
                label = "dream"
            elif step <= len(planned):
                requested = np.asarray(planned[step-1]["action14"], dtype=float)
                label = planned[step-1]["phase"]
            elif step <= len(planned)+ALIGN_STEPS:
                requested = np.asarray(align_action(action, state["tool_pose_xyzw"]), dtype=float)
                label = "centre measured lens"
            else:
                requested = np.asarray(action, dtype=float)
                label = "hold centred view"
            if requested.shape != (14,) or not np.isfinite(requested).all():
                raise RuntimeError("Policy returned an invalid action")
            # Exactly the wrapper's ordinary URDF-bound clipping, in both modes.
            action = np.clip(requested, emb._low, emb._high).tolist()
            emb._env.step(torch.tensor([action], device=base.device, dtype=torch.float32))
            emb._t = step/CONTROL_HZ
            state, observation = snapshot(step, label, action)
            record.update(steps=step, final_state=state)
            if step % 20 == 0:
                save()
                print(json.dumps({"step": step, "phase": label, "requests": len(request_audit)}), flush=True)

        phase = "evidence finalization"
        for step, raw, policy_rgb in hold_frames:
            Image.fromarray(raw).save(args.out/f"hold-{step:04d}-raw.png")
            Image.fromarray(policy_rgb).save(args.out/f"hold-{step:04d}-policy.png")
        record["saved_hold_steps"] = [row[0] for row in hold_frames]
        frames_saved = True
        video.close(); video = None
        wrist_video.close(); wrist_video = None
        Image.fromarray(array(base.scene["review_cam"].data.output["rgb"])[0, :, :, :3].astype(np.uint8)).save(args.out/"final-review.png")

        # Optical controls are post-episode only; nothing moves or reaches Dream.
        physics_before = int(base.sim._physics_step_count)
        frozen_joints = {name: array(base.scene[name].data.joint_pos).copy()
                         for name in ("left_robot", "right_robot")}
        frozen_objects = {name: array(base.arena_world.get_pose_w(name)).copy()
                          for name in ("magnifier", "target_plate")}
        lens = stage.GetPrimAtPath(record["tool_root_path"]+"/MagnifierHead/Lens")
        try:
            for label, visible in (("final", True), ("baseline", False)):
                (UsdGeom.Imageable(lens).MakeVisible if visible else UsdGeom.Imageable(lens).MakeInvisible)()
                for _ in range(32):
                    base.sim.render()
                    base.scene["left_cam"].update(.05, force_recompute=True)
                raw = array(base.scene["left_cam"].data.output["rgb"])[0, :, :, :3].astype(np.uint8)
                policy_rgb = emb._observation().images["left_cam"]
                if not np.array_equal(policy_rgb, cv2.resize(raw, (224, 224))):
                    raise RuntimeError("Frozen reference does not match actual policy RGB")
                if int(base.sim._physics_step_count) != physics_before:
                    raise RuntimeError("Physics advanced during the reference comparison")
                for name, before in frozen_joints.items():
                    if not np.array_equal(before, array(base.scene[name].data.joint_pos)):
                        raise RuntimeError("Robot moved during the reference comparison")
                for name, before in frozen_objects.items():
                    if not np.array_equal(before, array(base.arena_world.get_pose_w(name))):
                        raise RuntimeError("Object moved during the reference comparison")
                Image.fromarray(raw).save(args.out/f"{label}-raw.png")
                Image.fromarray(policy_rgb).save(args.out/f"{label}-policy.png")
                np.savez_compressed(args.out/f"{label}.npz", raw=raw, policy=policy_rgb)
        finally:
            UsdGeom.Imageable(lens).MakeVisible()
        record["optical_comparison"] = {
            "same_camera_pose": True, "same_physics_step": physics_before,
            "same_tool_and_target_pose": True, "actual_policy_resize_verified": True,
            "counterfactual": "Only glass hidden after control; frame, handle and all physics stay fixed.",
        }
        record.update(complete=True, termination="demo_finished" if args.policy == "demo" and
                      total_steps == demo_steps else "action_limit", final_state=state)
    except BaseException as error:
        # Remote/credential exception messages are deliberately not serialized.
        record.update(complete=False, error_type=type(error).__name__, error_phase=phase)
        print(json.dumps({"complete": False, "error_type": type(error).__name__, "phase": phase}), flush=True)
        raise RuntimeError(f"Trial failed during {phase}: {type(error).__name__}") from None
    finally:
        if hold_frames and not frames_saved:
            try:
                for step, raw, policy_rgb in hold_frames:
                    Image.fromarray(raw).save(args.out/f"hold-{step:04d}-raw.png")
                    Image.fromarray(policy_rgb).save(args.out/f"hold-{step:04d}-policy.png")
                record["saved_hold_steps"] = [row[0] for row in hold_frames]
            except Exception as error:
                record.update(complete=False, frame_finalization_error=type(error).__name__)
        if dream_policy is not None:
            if __package__:
                from .dream_policy import public_transcript
            else:
                from dream_policy import public_transcript
            try:
                record["dream_transcript"] = public_transcript(dream_policy)
                dream_policy.close()
            except Exception as error:
                record.update(complete=False, dream_finalization_error=type(error).__name__)
        for writer in (video, wrist_video):
            if writer is not None:
                try:
                    writer.close()
                except Exception as error:
                    record.update(complete=False, video_finalization_error=type(error).__name__)
        if trace is not None:
            trace.close()
        save()
        app.close()


if __name__ == "__main__":
    main()
