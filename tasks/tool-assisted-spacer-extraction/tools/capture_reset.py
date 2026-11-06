"""Capture the fixed reset at native camera resolution, without running a policy.

Save the native 640 x 360 render before the policy's 320 x 176 resize. Poses,
field of view and physics retain the supplied task calibration. This
inspection is not a completed trial.
"""
from __future__ import annotations

import argparse
from pathlib import Path

from inspect_robots import Scene

from spacer_extraction.runtime import SpacerExtractionEmbodiment


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    embodiment = SpacerExtractionEmbodiment(
        trace_path=str(args.out / "reset-inspection.jsonl"), headless=True
    )
    try:
        embodiment.reset(
            Scene(id="reset-inspection", instruction="Inspect the fixed starting layout."),
            seed=1415158438,
        )
        import cv2

        for name in ("top_cam", "left_cam", "right_cam"):
            output = embodiment._base.scene[name].data.output["rgb"]
            output = output.torch if hasattr(output, "torch") else output
            rgb = output[0, ..., :3].cpu().numpy()
            path = args.out / ("setup.png" if name == "top_cam" else f"{name}.png")
            if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise OSError(f"could not save {path}")
            print(path, flush=True)
    finally:
        embodiment.close()


if __name__ == "__main__":
    main()
