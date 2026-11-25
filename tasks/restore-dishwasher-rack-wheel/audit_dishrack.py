"""Save delivered RGB, native RGB, and a matched-seed reset check."""
import json
import os
from pathlib import Path
import sys

import numpy as np
from inspect_robots.cli import main
from yam_arena import inspect_embodiment


CAMERAS = ("top_cam", "left_cam", "right_cam")
ARTICULATIONS = ("left_robot", "right_robot", "basket", "replacement_roller")
ASSETS = (*ARTICULATIONS, "press_block")
STEPS = (17, 21, 26, 42, 62, 64, 80, 94, 104, 116, 126, 136, 146, 160, 179, 182, 201, 220, 223, 235, 242, 257, 258, 280, 299, 321, 329, 349)
output = Path(os.environ["DISHRACK_AUDIT_DIR"])
camera_output = output / "camera-audit"
camera_output.mkdir()
make_arena = inspect_embodiment.yam_arena_embodiment


def as_array(value):
    return getattr(value, "torch", value).detach().cpu().numpy().copy()


def scorer_state(scorer):
    return {name: sorted(value) if isinstance(value, frozenset) else value
            for name, value in vars(scorer).items()}


def snapshot(arena, observation):
    task = arena.task.module
    state = {}
    frame_numbers = {}
    for name in ASSETS:
        data = arena._base.scene[name].data
        for field in ("root_link_pose_w", "root_vel_w"):
            state[f"{name}/{field}"] = as_array(getattr(data, field))
        if name in ARTICULATIONS:
            for field in ("joint_pos", "joint_vel"):
                state[f"{name}/{field}"] = as_array(getattr(data, field))
    for side in CAMERAS:
        camera = arena._base.scene[side]
        state[f"{side}/pos_w"] = as_array(camera.data.pos_w)
        state[f"{side}/quat_w_world"] = as_array(camera.data.quat_w_world)
        frame_numbers[side] = int(as_array(camera.frame)[0])
    info = {
        "score": scorer_state(task.scorer),
        "image_times": observation.image_times, "state_time": observation.state_time,
        "camera_frames": frame_numbers,
        "physics_hz": round(1. / arena._base.sim.cfg.dt), "control_hz": round(1. / arena._base.step_dt),
    }
    return state, info, {name: frame.copy() for name, frame in observation.images.items()}


def record(arena, label, observation):
    assert set(observation.images) == set(CAMERAS)
    for name in CAMERAS:
        img = observation.images[name]
        frame = as_array(arena._base.scene[name].data.output["rgb"])[0, ..., :3]
        assert img.shape == (176, 320, 3) and frame.shape == (360, 640, 3)
        np.save(camera_output / f"{label}-delivered_{name}.npy", img)
        np.save(camera_output / f"{label}_{name}.npy", frame)


def gradient_alignment(before, after, passes=0):
    vectors = []
    for frame in (before, after):
        gray = frame.astype(np.float32).mean(axis=2)
        for _ in range(passes):
            gray = (gray[:-1,:-1]+gray[1:,:-1]+gray[:-1,1:]+gray[1:,1:])*.25
        gradient = np.concatenate([np.diff(gray, axis=axis).ravel() for axis in (0, 1)])
        vectors.append(gradient)
    norms = np.linalg.norm(vectors[0]) * np.linalg.norm(vectors[1])
    return float(vectors[0] @ vectors[1] / norms) if norms > 0. else 0.


def audit_factory(**kwargs):
    arena = make_arena(**kwargs)
    reset_arena, step_arena, close_arena = arena.reset, arena.step, arena.close
    initial = {}
    steps = 0
    last_observation = None

    def reset(scene, *, seed=None):
        obs = reset_arena(scene, seed=seed)
        initial.update(scene=scene, seed=seed, snapshot=snapshot(arena, obs))
        record(arena, "start", obs)
        return obs

    def step(action):
        nonlocal steps, last_observation
        result = step_arena(action)
        steps += 1
        last_observation = result.observation
        if steps in STEPS:
            record(arena, f"t{steps:04d}", result.observation)
        return result

    def close():
        if not initial or not steps:
            close_arena()
            return
        before, counters, frames = initial["snapshot"]
        _, terminal_counters, terminal_frames = snapshot(arena, last_observation)
        terminal = scorer_state(arena.task.module.scorer)
        trace_path = os.environ.pop("DISHRACK_TRACE", None)
        try:
            obs = reset_arena(initial["scene"], seed=initial["seed"])
            after, after_counters, after_frames = snapshot(arena, obs)
            record(arena, "reset", obs)
        finally:
            if trace_path is not None:
                os.environ["DISHRACK_TRACE"] = trace_path
            close_arena()
        errors = {name: float(np.abs(value - after[name]).max()) for name, value in before.items()}
        rgb = {}
        for name in CAMERAS:
            delta = np.abs(frames[name].astype(np.float32) - after_frames[name])
            initial_alignment = gradient_alignment(frames[name], after_frames[name], passes=3)
            stale_alignment = gradient_alignment(terminal_frames[name], after_frames[name], passes=3)
            rgb[name] = {
                "mean_delta": float(delta.mean()),
                "fraction_over_10": float((delta > 10.).mean()),
                "raw_gradient_alignment": gradient_alignment(frames[name], after_frames[name]),
                "reset_edge_alignment": initial_alignment,
                "stale_edge_alignment": stale_alignment,
            }
        checks = {
            "physical_state": max(err for name,err in errors.items() if name.split("/")[0] in ASSETS) <= 2.e-4,
            "counters_and_timestamps": counters == after_counters,
            "rates": after_counters["physics_hz"] == 480 and after_counters["control_hz"] == 20,
            "cameras": (all(row["reset_edge_alignment"] >= .92
                            and row["reset_edge_alignment"] > row["stale_edge_alignment"] + .02
                            for row in rgb.values())
                        and all(after_counters["camera_frames"][name] < terminal_counters["camera_frames"][name]
                                for name in CAMERAS)
                        and max(err for name,err in errors.items() if name.split("/")[0] in CAMERAS) <= 1.e-5),
        }
        result = dict(checks=checks, steps=steps, terminal=terminal, reset=after_counters, max_state_errors=errors, rgb=rgb)
        (output / "reset-audit.json").write_text(json.dumps(result, indent=2) + "\n")
        assert all(checks.values()), "Post-trial dishwasher reset audit failed"

    arena.reset, arena.step, arena.close = reset, step, close
    return arena


inspect_embodiment.yam_arena_embodiment = audit_factory
sys.exit(main())
