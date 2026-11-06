"""Export a dimensioned plan and object inventory from the task scene source."""
from __future__ import annotations

import dataclasses
import html
import json
from pathlib import Path

from spacer_extraction import task


def main() -> None:
    out = Path(__file__).resolve().parents[1] / "assets"
    out.mkdir(exist_ok=True)
    objects = []
    names = set()
    for obj in task.OBJECTS:
        if obj.name in names:
            raise ValueError(f"duplicate task object name: {obj.name}")
        names.add(obj.name)
        if len(obj.size) != 3 or any(float(dimension) <= 0 for dimension in obj.size):
            raise ValueError(f"invalid dimensions for {obj.name}")
        row = dataclasses.asdict(obj)
        row["type"] = type(obj).__name__
        row["material"] = "aluminium" if obj.name == "spacer" else "PETG"
        row["fixed"] = obj.mass is None
        row["height_above_table_m"] = obj.pos[2] - task.TOP
        if obj.name == "pusher":
            row["mass_basis"] = "60 g effective printed-part mass; PETG infill and wall geometry are not modeled"
        elif obj.name == "spacer":
            row["mass_basis"] = "solid aluminium at 2.70 g/cm^3"
        elif obj.mass is None:
            row["mass_basis"] = "fixed fixture geometry; dynamic mass is not simulated"
        objects.append(row)

    if len(objects) != 14:
        raise ValueError(f"expected 14 task objects, found {len(objects)}")

    setup = {
        "schema_version": 2,
        "coordinate_frame": "x forward from arm-base line; y left; z up; SI metres",
        "table_top_z_m": task.TOP,
        "control_hz": 20,
        "physics_hz": 240,
        "layout_randomized": False,
        "object_jitter_m": 0,
        "left_base_m": [0, 0.305, 0.75],
        "right_base_m": [0, -0.305, 0.75],
        "shared_static_dynamic_friction": [1.2, 1.0],
        "objects": objects,
    }
    (out / "setup.json").write_text(json.dumps(setup, indent=2) + "\n", encoding="utf-8")

    scale = 1700

    def screen(x: float, y: float) -> tuple[float, float]:
        return 120 + scale * x, 700 - scale * y

    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="850" viewBox="0 0 1100 850">',
        '<rect width="1100" height="850" fill="white"/>',
        '<g font-family="sans-serif" fill="#182330">',
        '<text x="40" y="42" font-size="26">Tool-assisted spacer extraction</text>',
        f'<text x="40" y="70" font-size="16">Top plan · dimensions in mm · table surface z = {task.TOP:.4f} m</text>',
    ]
    for x in range(0, 451, 50):
        sx, _ = screen(x / 1000, 0)
        parts.append(f'<line x1="{sx}" y1="90" x2="{sx}" y2="700" stroke="#e2e7eb"/>')
        parts.append(f'<text x="{sx - 10}" y="725" font-size="13">{x}</text>')
    for y in range(0, 401, 50):
        _, sy = screen(0, y / 1000)
        parts.append(f'<line x1="120" y1="{sy}" x2="885" y2="{sy}" stroke="#e2e7eb"/>')
        parts.append(f'<text x="75" y="{sy + 4}" font-size="13">{y}</text>')

    ordered = sorted(task.OBJECTS, key=lambda obj: ("roof" in obj.name, obj.name))
    for obj in ordered:
        x, y = screen(obj.pos[0] - obj.size[0] / 2, obj.pos[1] + obj.size[1] / 2)
        color = "#" + "".join(f"{round(component * 255):02x}" for component in obj.color)
        translucent = ' fill-opacity="0.32" stroke-dasharray="4 3"' if "roof" in obj.name else ""
        parts.append(
            f'<rect x="{x}" y="{y}" width="{obj.size[0] * scale}" '
            f'height="{obj.size[1] * scale}" fill="{color}" stroke="#273646"{translucent}/>'
        )

    labels = [
        (0.270, 0.300, "Pusher: 260 × 16 × 16 · rear grip x = 160"),
        (0.350, 0.285, "Recovery nest · centre (350, 260)"),
        (0.350, 0.154, "Covered channel: 40 long × 28 wide · 30 clear"),
        (0.386, 0.052, "Spacer: 28 × 24 × 24"),
        (0.200, 0.020, "Push direction → (+x) · extraction marks at x = 330 and 370"),
    ]
    for x, y, label in labels:
        sx, sy = screen(x, y)
        parts.append(f'<text x="{sx}" y="{sy}" font-size="14" text-anchor="middle">{html.escape(label)}</text>')
    parts.extend([
        '<text x="120" y="760" font-size="15">x forward →    y left ↑ · roof is translucent in this plan to reveal the spacer.</text>',
        '<text x="120" y="782" font-size="13">Pusher park and extraction marks are 0.5 mm high printed guides; they do not retain objects.</text>',
        f'<text x="40" y="824" font-size="14">{len(objects)} task objects. Exact centres, heights, materials and masses are in setup.json.</text>',
        '</g></svg>',
    ])
    (out / "setup.svg").write_text("\n".join(parts), encoding="utf-8")
    print(f"Exported {len(objects)} objects to {out}")


if __name__ == "__main__":
    main()
