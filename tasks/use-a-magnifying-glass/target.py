"""Generate a dimensioned RGB inspection card; this is not an optical renderer."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def generate(output_dir: str | Path, pixels_per_mm: int = 40) -> dict:
    """Write target.png and target.json, with metric coordinates in millimetres.

    The card is 120 x 100 mm. Texture top-left is (-60, +50) mm in
    the card's local XY plane. Do not flip the texture relative to the metadata.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    width_mm, height_mm = 120.0, 100.0
    width, height = round(width_mm * pixels_per_mm), round(height_mm * pixels_per_mm)
    image = Image.new("RGB", (width, height), (242, 242, 242))
    draw = ImageDraw.Draw(image)

    def pixel(x: float, y: float) -> tuple[float, float]:
        return (x + width_mm / 2) * pixels_per_mm, (height_mm / 2 - y) * pixels_per_mm

    def rectangle(center, size, fill):
        x, y = center
        w, h = size
        draw.rectangle([pixel(x-w/2, y+h/2), pixel(x+w/2, y-h/2)], fill=fill)

    fiducials = [
        {"name": "red", "xy_mm": [-9.5, 9.5], "rgb": [225, 25, 30]},
        {"name": "green", "xy_mm": [9.5, 9.5], "rgb": [25, 190, 40]},
        {"name": "blue", "xy_mm": [-9.5, -9.5], "rgb": [30, 70, 230]},
        {"name": "magenta", "xy_mm": [9.5, -9.5], "rgb": [220, 30, 205]},
    ]
    for fiducial in fiducials:
        x, y = fiducial["xy_mm"]
        radius = fiducial["radius_mm"] = 1.1
        draw.ellipse([pixel(x-radius, y+radius), pixel(x+radius, y-radius)], fill=tuple(fiducial["rgb"]))

    patches = []
    for row, orientation in enumerate(("vertical", "horizontal")):
        for column, pitch in enumerate((0.6, 1.2, 2.4)):
            x, y = (-5.5, 0.0, 5.5)[column], (3.3, -3.3)[row]
            patch = {"name": f"{orientation}_{pitch:g}mm", "center_mm": [x, y],
                     "size_mm": [5.0, 5.0], "orientation": orientation,
                     "pitch_mm": pitch, "line_width_mm": pitch / 2}
            patches.append(patch)
            # A mathematical square wave avoids cumulative rectangle rounding.
            left, top = map(round, pixel(x-2.5, y+2.5))
            patch_size = round(5.0 * pixels_per_mm)
            yy, xx = np.indices((patch_size, patch_size), dtype=float)
            position = (xx + .5) / pixels_per_mm if orientation == "vertical" else (yy + .5) / pixels_per_mm
            gray = np.where(np.mod(position, pitch) < pitch / 2, 12, 242).astype(np.uint8)
            image.paste(Image.fromarray(gray).convert("RGB"), (left, top))

    references = [{"name": "black", "center_mm": [-2.8, 0.0], "size_mm": [4.4, 0.8], "rgb": [12]*3},
                  {"name": "white", "center_mm": [2.8, 0.0], "size_mm": [4.4, 0.8], "rgb": [242]*3}]
    for reference in references:
        rectangle(reference["center_mm"], reference["size_mm"], tuple(reference["rgb"]))
    image.save(output_dir / "target.png")
    metadata = {"schema": "magnifier-target-v1", "card_size_mm": [width_mm, height_mm],
                "pixels_per_mm": pixels_per_mm, "texture_size": [width, height],
                "coordinates": "Card local +X right, +Y up; PNG top-left=(-60,+50) mm.",
                "fiducials": fiducials, "patches": patches, "references": references,
                "sha256": hashlib.sha256((output_dir / "target.png").read_bytes()).hexdigest()}
    (output_dir / "target.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--pixels-per-mm", type=int, default=40)
    args = parser.parse_args()
    print(json.dumps(generate(args.output_dir, args.pixels_per_mm), indent=2))
