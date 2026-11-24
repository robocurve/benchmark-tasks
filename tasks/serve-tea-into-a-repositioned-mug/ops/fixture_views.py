"""Record the displacement witness before contact, at trip and after withdrawal."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tea.model import Kitchen
from tea.reference import Server
from tea.render import Camera
from tea.run import revision


def capture(out):
    out.mkdir(parents=True, exist_ok=True)
    e = Kitchen("steady")
    motion = Server(e, "contact_probe")
    home = e.state().copy()
    cam = Camera(e, 480, 360)
    # External review camera. Policy observations continue to use the three task cameras.
    eye = mj.MjvCamera()
    eye.type = mj.mjtCamera.mjCAMERA_FREE
    eye.lookat[:] = [0.543, 0.305, 0.794]
    eye.distance = 0.45
    eye.azimuth = -100
    eye.elevation = -48
    frames, readings = [], []

    def snap(label):
        frames.append(Image.fromarray(cam.frame(eye)))
        readings.append(
            dict(
                label=label,
                t_s=round(e.d.time, 4),
                flag=e.flag_source,
                xy_mm=(1000 * e.d.qpos[e.witness_q]).round(3).tolist(),
            )
        )

    try:
        snap("Before contact")
        for _ in range(160):
            e.action(motion.command())
            if e.flag_source:
                snap("Displacement latched")
                break
        else:
            raise RuntimeError("Displacement control did not trip the witness")
        tripped = e.flag_source
        start = e.state().copy()
        for i in range(110):
            frac = min((i + 1) / 40, 1)
            e.action(start + frac * (home - start))
        if np.max(np.abs(e.d.qpos[e.witness_q])) >= 0.0005 or e.flag_source != tripped:
            raise RuntimeError(
                "Witness failed to retain its indication after unloading"
            )
        snap("Unloaded with flag retained")
    finally:
        cam.close()
    image = Image.new("RGB", (1440, 425), "#192526")
    draw = ImageDraw.Draw(image)
    try:
        heading = ImageFont.truetype("DejaVuSans.ttf", 22)
        small = ImageFont.truetype("DejaVuSans.ttf", 16)
    except OSError:
        heading = small = ImageFont.load_default()
    for i, (picture, record) in enumerate(zip(frames, readings)):
        x = i * 480
        image.paste(picture, (x, 65))
        draw.text((x + 12, 4), record["label"], font=heading, fill="white")
        xy = record["xy_mm"]
        draw.text(
            (x + 12, 36),
            f"{record['t_s']:.2f} s    X {xy[0]:+.2f} mm    Y {xy[1]:+.2f} mm",
            font=small,
            fill="#bcd4d1",
        )
    target = out / "WITNESS_SIMULATION.png"
    image.save(target)
    (out / "WITNESS_SIMULATION.json").write_text(
        json.dumps(
            dict(
                source_sha256=revision(),
                image_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                scene="steady",
                controller="contact_probe then measured reset pose",
                camera="supplementary oblique inspection view",
                frames=readings,
            ),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    capture(parser.parse_args().output)
