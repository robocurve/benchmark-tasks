"""Start a separating zipper, zip the blue baby jacket, and release it."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import traceback

from zipper.mechanism import MAX_STEPS
from zipper.scoring import Reading, zip_complete, zip_score

TASK = "textile-13-start-and-zip-jacket"
INSTRUCTION = (
    "Hold the blue jacket steady and start its zipper. Grasp the reinforced blue tape behind the loose "
    "silver insertion pin at the hem. Align the pin with the opening beside the orange slider and insert "
    "it fully into the retaining box. Then grasp the orange pull and zip the jacket all the way to the collar. "
    "Regrasp if needed, and release the jacket and zipper when finished."
)


def save_view(robot, destination, *, detail=False):
    import numpy as np
    from PIL import Image
    from yam_arena.station import TABLE_TOP

    camera = robot._base.scene["top_cam"]
    if detail:
        camera.set_world_poses_from_view(
            np.array([[0.100, -0.095, TABLE_TOP + 0.19]]), np.array([[0.162, -0.013, TABLE_TOP + 0.025]]),
        )
        for _ in range(3):
            robot._env.step(robot._ready)
    image = camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy()
    Image.fromarray(image).save(destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--dream", action="store_true")
    group.add_argument("--expert", action="store_true")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--max-steps", type=int, default=1200)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.max_steps <= MAX_STEPS:
        parser.error(f"--max-steps must be 1 through {MAX_STEPS}")
    args.output.mkdir(parents=True, exist_ok=True)

    from inspect_robots import Scene, Task, eval
    from inspect_robots.scorer import Score
    from zipper.yam import ZipperEmbodiment

    robot = ZipperEmbodiment(TASK, INSTRUCTION, 0.0, live=args.live, auxiliary_grips=False)
    scene = Scene(id="pin-disengaged", instruction=INSTRUCTION)

    class ZipperScorer:
        name = "start_and_zip"

        def __call__(self, record, target):
            readings = [Reading(**entry.result.info["zipper"]) for entry in record.steps]
            if not readings:
                return Score(value=None, explanation="No actions were recorded")
            result = {
                "score": zip_score(readings), "goal_met": zip_complete(readings[-1]),
                "final": asdict(readings[-1]), "steps": len(readings), "control_hz": 20,
            }
            (args.output / "dream-score.json").write_text(json.dumps(result, indent=2) + "\n")
            (args.output / "states.json").write_text(json.dumps([asdict(r) for r in readings]) + "\n")
            return Score(value=result["score"], metadata=result)

    status = 1
    try:
        if args.dream:
            experiment = Task(name=TASK, scenes=[scene], scorer=ZipperScorer(), max_steps=args.max_steps)
            [log] = eval(experiment, "dream", robot, seed=0, log_dir=str(args.output), fail_on_error=True,
                         store_frames=True, guard=["clamp", "delta-limit"])
            for sample in log.samples:
                print(sample.status, sample.reduced, flush=True)
            status = 0 if log.status == "success" else 1
        elif args.expert:
            from zipper.expert import record

            status = record(robot, scene, args.output, args.max_steps)
        else:
            try:
                robot.preview(scene, hold_open=0)
            except RuntimeError:
                save_view(robot, args.output / "failed-setup.png")
                save_view(robot, args.output / "failed-detail.png", detail=True)
                raise
            save_view(robot, args.output / "setup.png")
            save_view(robot, args.output / "pin-detail.png", detail=True)
            (args.output / "initial-state.json").write_text(json.dumps(asdict(robot.reading()), indent=2) + "\n")
            status = 0
    except Exception:
        traceback.print_exc()
    finally:
        if robot.app is not None:
            robot.app.close(exit_code=status)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
