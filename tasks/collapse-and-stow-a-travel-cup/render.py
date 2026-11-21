"""Render every recorded control state in the accepted four-camera review layout."""

import argparse
from pathlib import Path

from review_media import StationViews, review_writer, save_setup

import mujoco
import numpy as np

from scene import CAMERAS, build


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    model = build()
    data = mujoco.MjData(model)
    states = np.load(args.run / "replay.npz", allow_pickle=False)["qpos"]
    views = StationViews(model, CAMERAS)
    try:
        with review_writer(args.run / "review.mp4") as write:
            for index, qpos in enumerate(states):
                data.qpos[:] = qpos
                mujoco.mj_forward(model, data)
                image = views.capture(data)
                if index == 0:
                    save_setup(image, args.run / "setup.png")
                write(image)
    finally:
        views.close()


if __name__ == "__main__":
    main()
