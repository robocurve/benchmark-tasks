"""Render all control states in the standard four-view layout at 20 Hz."""

import argparse
from pathlib import Path

from review_media import StationViews, review_writer, save_setup
from run import Trial
from scene import CAMERAS

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    trial = Trial()
    states = np.load(args.run / "states.npz", allow_pickle=False)["qpos"]
    views = StationViews(trial.m, CAMERAS)
    try:
        with review_writer(args.run / "review.mp4") as write:
            for index, qpos in enumerate(states):
                trial.d.qpos[:] = qpos
                mujoco.mj_forward(trial.m, trial.d)
                images = views.capture(trial.d)
                if index == 0:
                    save_setup(images, args.run / "setup.png")
                write(images)
    finally:
        views.close()


if __name__ == "__main__":
    main()
