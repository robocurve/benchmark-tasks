"""Prepare the shallow starting fill; never called during an evaluation.

Run from the repository root:
    python tasks/dig-out-a-buried-pipe-with-a-spade/prepare_soil.py

Keep the lower part of the original settled fill, retaining positions and
orientations. The original deep-fill file remains unchanged for provenance.
"""

from pathlib import Path

import digplan as d
import numpy as np
from scoring import exposed_length, grain_mass


def main():
    folder = Path(__file__).resolve().parent
    with np.load(folder / "soil_angular_start.npz") as original:
        keep = original["pos"][:, 2] + original["radius"] <= d.SOIL_TOP
        layout = {key: original[key][keep] for key in ("pos", "radius", "quat")}
    exposure = exposed_length(layout["pos"], layout["radius"], layout["quat"])
    if exposure != 0:
        raise ValueError("Prepared fill must initially cover the marked crown")
    np.savez(folder / "soil_shallow_start.npz", **layout)
    print(
        f"{len(layout['pos'])} grains; {grain_mass(layout['radius']).sum():.6f} kg; "
        f"initial exposure {exposure:.3f} m"
    )


if __name__ == "__main__":
    main()
