"""Reset, demonstrate, and independently re-score the fixed kitchen-utensil task."""

import argparse
import json
import time
from contextlib import ExitStack
from itertools import groupby
from pathlib import Path

import numpy as np
from PIL import Image

from task import HZ, INSTRUCTION, LIMIT, KitchenUtensils
from inspect_robots import Action, Scene
from controller import motions
from planner import Planner
from scoring import engagement, evaluate_saved, markers, snapshot, state_id, whisk_shape

TASK = "untangle-kitchen-utensils"
EXPECTED = {"untouched": 0.0, "one_free": 0.5, "completed": 1.0}


def reset(env):
    return env.reset(Scene(id=TASK, instruction=INSTRUCTION), seed=0)


def save_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def scores(model, states):
    return {name: evaluate_saved(model, states[name]) for name in states}


def save_images(env, out):
    for cam, file in (("setup_view", "setup.png"), ("tangle_view", "starting-tangle.png")):
        Image.fromarray(env.render(cam, 960, 540)).save(out / file)


def demonstrate(env, out, record):
    observation = reset(env)
    planner = Planner(observation.state["joint_pos"])
    states = {"untouched": snapshot(env.model, env.data)}
    actions = []
    started = time.monotonic()
    with ExitStack() as stack:
        writer = None
        if record:
            import imageio.v2 as imageio

            writer = stack.enter_context(
                imageio.get_writer(
                    str(out / "demo.mp4"),
                    fps=HZ,
                    codec="libx264",
                    macro_block_size=1,
                    pixelformat="yuv420p",
                    output_params=["-crf", "19", "-movflags", "+faststart"],
                )
            )
            save_images(env, out)
            writer.append_data(env.render("setup_view", 960, 540))
        path = motions(planner, lambda: observation.state["joint_pos"])
        for label, group in groupby(path, key=lambda step: step[0]):
            for _, target in group:
                if env.steps >= LIMIT:
                    raise RuntimeError("Demo exhausted the action budget")
                observation = env.step(Action(data=target)).observation
                actions.append(target)
                if writer:
                    writer.append_data(env.render("setup_view", 960, 540))
            if label == "clear spatula":
                states["one_free"] = snapshot(env.model, env.data)
    states["completed"] = snapshot(env.model, env.data)
    np.savez_compressed(out / "states.npz", **states)
    np.savez_compressed(out / "actions.npz", targets=np.asarray(actions))
    results = scores(env.model, states)
    measurements = dict(
        seed=0,
        action_limit=LIMIT,
        steps=env.steps,
        simulated_seconds=env.steps / HZ,
        wall_seconds=round(time.monotonic() - started, 2),
        checkpoints=results,
    )
    save_json(out / "scores.json", measurements)
    measured = {k: r["score"] for k, r in results.items()}
    print(json.dumps(measured), flush=True)
    print(f"{env.steps} actions, {env.steps / HZ:.2f} simulated seconds")
    if measured != EXPECTED:
        raise SystemExit(1)


def force_on_spoon(model, data):
    """Reaction from silicone on wood, projected opposite the +y withdrawal."""
    import mujoco

    force = np.zeros(6)
    resisting, normal = 0.0, 0.0
    hand_on_whisk = False
    for i, c in enumerate(data.contact):
        names = [model.geom(int(g)).name or "" for g in c.geom]
        primary = [n.partition("_")[0] for n in names]
        if "whisk" in primary and "left" in primary and c.dist <= 0:
            hand_on_whisk = True
        if "spoon" not in primary or not any(n.startswith("spatula_blade") for n in names):
            continue
        mujoco.mj_contactForce(model, data, i, force)
        on_second = c.frame.reshape(3, 3).T @ force[:3]
        on_spoon = on_second if primary[1] == "spoon" else -on_second
        resisting -= on_spoon[1]
        normal += force[0]
    return max(0.0, float(resisting)), float(normal), hand_on_whisk


def check_tangle(env, trial):
    observation = reset(env)
    reset_hash = state_id(snapshot(env.model, env.data))
    planner = Planner(observation.state["joint_pos"])
    active = "lift spoon alone" if trial == "lift" else "pull past blade"
    before = None
    peaks = dict(
        crown_rise_mm=0.0,
        ferrule_rise_mm=0.0,
        blade_tip_motion_mm=0.0,
        blade_on_spoon_opposing_N=0.0,
        blade_on_spoon_normal_N=0.0,
        wire_bend_deg=0.0,
        blade_bend_deg=0.0,
    )
    contacted = False
    for label, target in motions(planner, lambda: observation.state["joint_pos"], trial):
        if label == active and before is None:
            before = {n: markers(env.data, n) for n in ("whisk", "spatula", "spoon")}
            start_tcp = env.data.site("left_tcp").xpos.copy()
        observation = env.step(Action(data=target)).observation
        if label != active:
            continue
        whisk = 1000 * (markers(env.data, "whisk") - before["whisk"])
        blade = 1000 * (markers(env.data, "spatula") - before["spatula"])
        force, normal, hand_on_whisk = force_on_spoon(env.model, env.data)
        contacted |= hand_on_whisk
        shape = whisk_shape(env.data)
        current = dict(
            crown_rise_mm=whisk[1, 2],
            ferrule_rise_mm=whisk[2, 2],
            blade_tip_motion_mm=np.linalg.norm(blade[1]),
            blade_on_spoon_opposing_N=force,
            blade_on_spoon_normal_N=normal,
            wire_bend_deg=shape["max_wire_bend_deg"],
            blade_bend_deg=shape["max_blade_bend_deg"],
        )
        for key, value in current.items():
            peaks[key] = max(peaks[key], value)
    return dict(
        reset_sha256=reset_hash,
        steps=env.steps,
        left_pad_touched_whisk=bool(contacted),
        left_tcp_displacement_mm=np.round(
            1000 * (env.data.site("left_tcp").xpos - start_tcp), 2
        ).tolist(),
        peaks={k: round(float(v), 3) for k, v in peaks.items()},
        engagement_after=engagement(env.data),
        score_after=evaluate_saved(env.model, snapshot(env.model, env.data))["score"],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    for cmd in ("reset", "demo", "check-tangle"):
        group = sub.add_parser(cmd)
        group.add_argument("--out", required=True, type=Path, help="fresh output directory")
        if cmd == "demo":
            group.add_argument(
                "--record", action="store_true", help="write H.264 video and setup images"
            )
    group = sub.add_parser("score")
    group.add_argument("--states", type=Path, default=Path(__file__).parent / "evidence" / "states.npz")
    args = parser.parse_args()
    env = KitchenUtensils(cameras=False)
    try:
        if args.mode == "score":
            with np.load(args.states, allow_pickle=False) as saved:
                result = scores(env.model, saved)
            print(json.dumps(result, indent=2))
            return
        args.out.mkdir(parents=True, exist_ok=False)
        if args.mode == "demo":
            demonstrate(env, args.out, args.record)
        elif args.mode == "reset":
            reset(env)
            states = {"untouched": snapshot(env.model, env.data)}
            np.savez_compressed(args.out / "states.npz", **states)
            save_images(env, args.out)
            result = scores(env.model, states)
            save_json(args.out / "scores.json", result)
            print(json.dumps(result["untouched"]))
        else:
            checks = {trial: check_tangle(env, trial) for trial in ("lift", "pull")}
            save_json(args.out / "coupling.json", checks)
            print(json.dumps(checks, indent=2))
            lift, pull = checks["lift"], checks["pull"]
            if (
                lift["reset_sha256"] != pull["reset_sha256"]
                or lift["left_pad_touched_whisk"]
                or lift["peaks"]["crown_rise_mm"] < 10
                or pull["peaks"]["blade_on_spoon_opposing_N"] <= 0.02
            ):
                raise SystemExit(1)
    finally:
        env.close()


if __name__ == "__main__":
    main()
