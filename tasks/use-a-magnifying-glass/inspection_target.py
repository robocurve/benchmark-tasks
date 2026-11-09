"""Generate the validated five-bar inspection card."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image, ImageDraw


def generate(output_dir: str | Path, pixels_per_mm: int = 40) -> dict:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    card_size = [120.0, 100.0]
    width, height = [round(v*pixels_per_mm) for v in card_size]
    image = Image.new("RGB", (width, height), (242, 242, 242))
    draw = ImageDraw.Draw(image)

    def pixel(x, y):
        return ((x+60)*pixels_per_mm, (50-y)*pixels_per_mm)

    def box(center, size, fill):
        x, y = center
        w, h = size
        # Pillow's rectangle includes its last pixel; use exclusive right/bottom.
        left, top = pixel(x-w/2, y+h/2)
        right, bottom = pixel(x+w/2, y-h/2)
        draw.rectangle((round(left), round(top), round(right)-1, round(bottom)-1), fill=fill)

    fiducials = [
        {"name": "red", "xy_mm": [-9.5, 9.5], "rgb": [225, 25, 30]},
        {"name": "green", "xy_mm": [9.5, 9.5], "rgb": [25, 190, 40]},
        {"name": "blue", "xy_mm": [-9.5, -9.5], "rgb": [30, 70, 230]},
        {"name": "magenta", "xy_mm": [9.5, -9.5], "rgb": [220, 30, 205]},
    ]
    for fid in fiducials:
        x, y = fid["xy_mm"]
        radius = fid["radius_mm"] = 1.1
        draw.ellipse([pixel(x-radius, y+radius), pixel(x+radius, y-radius)], fill=tuple(fid["rgb"]))

    groups = [
        {"name": "vertical", "orientation": "vertical", "center_mm": [0.0, 4.3], "size_mm": [10.8, 3.2]},
        {"name": "horizontal", "orientation": "horizontal", "center_mm": [0.0, -4.3], "size_mm": [3.2, 10.8]},
    ]
    for group in groups:
        group.update(bar_count=5, gap_count=4, bar_width_mm=1.2, gap_width_mm=1.2,
                     stripe_length_mm=3.2, evaluated_length_fraction=.5)
        axis = 0 if group["orientation"] == "vertical" else 1
        for n in range(5):
            center = group["center_mm"].copy()
            center[axis] += -4.8+n*2.4
            size = [1.2, 3.2] if axis == 0 else [3.2, 1.2]
            box(center, size, (12, 12, 12))

    references = [
        {"name": "black", "center_mm": [-5.5, -4.5], "size_mm": [3.0, 3.0], "rgb": [12]*3},
        {"name": "white", "center_mm": [5.5, -4.5], "size_mm": [3.0, 3.0], "rgb": [242]*3},
    ]
    for ref in references:
        box(ref["center_mm"], ref["size_mm"], tuple(ref["rgb"]))
    registration = []
    # V1's (40,30) mm radius was outside the glass but behind the opaque rim.
    # (50,40) mm leaves a visible margin beyond the rim's projected silhouette.
    for x in (-50.0, 50.0):
        for y in (-40.0, 40.0):
            marker = {"name": f"{'left' if x < 0 else 'right'}_{'bottom' if y < 0 else 'top'}",
                      "xy_mm": [x, y], "size_mm": [1.5, 1.5], "rgb": [12]*3}
            registration.append(marker)
            box(marker["xy_mm"], marker["size_mm"], (12, 12, 12))

    image.save(output_dir / "target.png")
    metadata = {"schema": "magnifier-inspection-target-v2", "card_size_mm": card_size,
                "texture_size": [width, height], "pixels_per_mm": pixels_per_mm,
                "coordinates": "Card local +X right, +Y up; PNG top-left=(-60,+50) mm.",
                "fiducial_square_side_mm": 19.0, "fiducials": fiducials,
                "bar_groups": groups, "references": references,
                "registration_markers": registration,
                "revision": "Moved only the outer registration markers from (+/-40,+/-30) to (+/-50,+/-40) mm to clear the physical opaque rim; validated in the recorded final trials.",
                "status": "Validated inspection chart; see final trial evidence.",
                "sha256": hashlib.sha256((output_dir / "target.png").read_bytes()).hexdigest()}
    (output_dir / "target.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--pixels-per-mm", type=int, default=40)
    args = parser.parse_args()
    print(json.dumps(generate(args.output_dir, args.pixels_per_mm), indent=2))
