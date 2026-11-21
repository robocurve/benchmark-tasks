"""Run the official Inspect Robots Dream policy on a contact simulation."""

import json
import os
from pathlib import Path
import re
import time

import numpy as np

from review_media import CONTROL_HZ, POLICY_CAMERAS, StationViews, policy_images, review_writer, save_setup


def _safe_error(error):
    message = f"{type(error).__name__}: {error}"
    key = os.environ.get("DREAM_API_KEY")
    if key:
        message = message.replace(key, "[redacted]")
    return re.sub(r"(?i)(bearer\s+)[A-Za-z0-9._~-]+", r"\1[redacted]", message)


def _joint_contract(model, data):
    bounds = []
    indices = []
    for side in ("left", "right"):
        joints = [model.joint(f"{side}_joint{index}") for index in range(1, 7)]
        candidates = [
            (f"{side}_finger_-1", f"{side}_finger_1"),
            (f"{side}_left_finger", f"{side}_right_finger"),
        ]
        names = {model.joint(i).name for i in range(model.njnt)}
        fingers = [model.joint(name) for name in next(pair for pair in candidates if set(pair) <= names)]
        limits = np.array([finger.range[np.argmax(abs(finger.range))] for finger in fingers])
        fingers_at = [finger.qposadr[0] for finger in fingers]
        joints_at = [joint.qposadr[0] for joint in joints]
        indices.append((joints_at, fingers_at, limits))
        bounds.extend([joint.range.copy() for joint in joints] + [(0., 1.)])

    def observe():
        state = []
        for joints_at, fingers_at, limits in indices:
            state.extend(data.qpos[joints_at])
            state.append(float(np.clip(np.mean(data.qpos[fingers_at] / limits), 0., 1.)))
        return np.asarray(state, dtype=np.float32)

    limits = np.asarray(bounds)
    return observe, limits[:, 0], limits[:, 1]


def run_episode(model, data, step, score, *, instruction, task_id, action_limit, out, cameras,
                observe=None, dry_run=False):
    """Use normalized 14-value joint commands, a fresh scene, and the 20 Hz control clock.

    The offline check holds the measured pose without contacting the model service.
    Production calls use the same three camera keys, action format and eight-row
    chunks as the accepted candy run. Review footage includes a separate detail view.
    """
    from inspect_robots import EmbodimentInfo, Observation, Scene
    from inspect_robots_dream import DreamPolicy
    from inspect_robots_yam.config import action_box, observation_space

    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    summary = {
        "task": task_id, "instruction": instruction, "planned_actions": action_limit,
        "rate_hz": CONTROL_HZ, "actions": 0, "requests": 0, "status": "starting",
        "engine": "MuJoCo", "offline_check": dry_run, "model_attempted": False,
    }

    def save():
        temp = out / "summary.tmp"
        temp.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        temp.replace(out / "summary.json")

    save()
    policy = renderer = None
    executed = []
    initial_views = None
    try:
        default_observe, low, high = _joint_contract(model, data)
        observe = observe if observe is not None else default_observe
        info = EmbodimentInfo(
            name="yam_mujoco",
            action_space=action_box(low, high),
            observation_space=observation_space(176, 320, POLICY_CAMERAS),
            is_simulated=True,
            control_hz=CONTROL_HZ,
            environment_id=f"yam-calibrated-{task_id}",
        )
        renderer = StationViews(model, cameras)
        initial_views = renderer.capture(data)
        save_setup(initial_views, out / "setup.png")
        if any(image.std() < 3. for image in initial_views.values()):
            raise RuntimeError("At least one initial camera view is blank")
        home = np.clip(observe(), low, high)
        if not dry_run:
            policy = DreamPolicy(rows_per_request=8, preroll_steps=100, timeout_s=120.)
            policy.bind(info)
            policy.reset(Scene(id=task_id, instruction=instruction))
        summary.update(status="running", model_attempted=not dry_run)
        save()
        with review_writer(out / "dream.mp4") as write:
            views = initial_views
            for index in range(action_limit):
                write(views)
                if policy is None:
                    action = home
                else:
                    now = index / CONTROL_HZ
                    result = policy.act(Observation(
                        images=policy_images(views), state={"joint_pos": observe()},
                        instruction=instruction, state_time=now,
                        image_times={name: now for name in POLICY_CAMERAS},
                    ))
                    if len(result.actions) != 1:
                        raise ValueError("Dream did not return a single joint target")
                    action = np.asarray(result.actions[0].data, dtype=np.float32)
                    if action.shape != (14,) or not np.isfinite(action).all():
                        raise ValueError("Dream returned a non-finite or malformed target")
                    action = np.clip(action, low, high)
                step(action.copy())
                executed.append(action.copy())
                views = renderer.capture(data)
                if len(executed) % 20 == 0:
                    summary["actions"] = len(executed)
                    summary["requests"] = len(policy.transcript()["requests"]) if policy else 0
                    save()
            write(views)
        summary.update(status="complete", grading=score())
    except BaseException as error:
        summary.update(status="failed", error=_safe_error(error), error_type=type(error).__name__)
        # Preserve the server's typed refusal/error reason without credentials.
        reason = getattr(error, "reason", None)
        if isinstance(reason, str):
            summary["reason"] = reason
    finally:
        summary.update(actions=len(executed), simulated_s=len(executed)/CONTROL_HZ,
                       wall_seconds=time.monotonic()-started)
        np.save(out / "actions.npy", np.asarray(executed, dtype=np.float32).reshape((-1, 14)))
        if policy is not None:
            transcript = policy.transcript()
            transcript.pop("server_url", None)
            for request in transcript["requests"]:
                request.pop("session_id", None)
            (out / "dream-transcript.json").write_text(json.dumps(transcript, indent=2) + "\n")
            summary["requests"] = len(transcript["requests"])
        save()
        if policy is not None:
            policy.close()
        if renderer is not None:
            renderer.close()
    return summary
