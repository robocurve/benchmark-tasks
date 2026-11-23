"""Render the recorded commands through the same plant and verify reporting parity."""

import argparse
import hashlib
import json
import subprocess
from contextlib import ExitStack
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from .model import Kitchen
from .render import Camera


def font(n):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", n)
    except OSError:
        return ImageFont.load_default()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("suite", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    cases = [
        ("steady", "Steady mug", "Recorded serving motion"),
        ("same_actions_shift", "Mug moves", "Identical robot commands"),
        ("reference", "Mug moves", "Responsive serving motion"),
    ]
    width, height = 400, 264
    with ExitStack() as stack:
        plants = []
        for name, title, subtitle in cases:
            folder = a.suite / name
            r = json.loads((folder / "result.json").read_text())
            arr = np.load(folder / "actions.npy", allow_pickle=False)
            if (
                hashlib.sha256(arr.astype("<f8").tobytes()).hexdigest()
                != r["actions_sha256"]
            ):
                raise ValueError("Command hash differs")
            e = Kitchen(r["profile"])
            c = Camera(e, width, height)
            stack.callback(c.close)
            plants.append((e, c, arr, r, title, subtitle))
        view = plants[0][1]
        Image.fromarray(view.frame("top_cam")).save(a.out / "setup.png")
        video = a.out / "serving_comparison.mp4"
        p = subprocess.Popen(
            [
                "ffmpeg",
                "-v",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                "1200x388",
                "-r",
                "10",
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-threads",
                "2",
                "-preset",
                "veryfast",
                "-crf",
                "19",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(video),
            ],
            stdin=subprocess.PIPE,
        )
        try:
            for tick in range(len(plants[0][2])):
                for env, c, arr, r, title, subtitle in plants:
                    env.action(arr[tick])
                if tick % 2:
                    continue
                canvas = Image.new("RGB", (1200, 388), "#192526")
                draw = ImageDraw.Draw(canvas)
                for col, (env, c, arr, r, title, subtitle) in enumerate(plants):
                    x = width * col
                    report = env.report()
                    canvas.paste(Image.fromarray(c.frame("detail")), (x, 51))
                    draw.text((x + 12, 4), title, font=font(19), fill="white")
                    draw.text((x + 12, 29), subtitle, font=font(14), fill="#bfdad6")
                    draw.text(
                        (x + 12, 324),
                        f"Mug  {env.cup:5.1f} mL      On person  {env.skin:4.1f} mL",
                        font=font(16),
                        fill="white",
                    )
                    status = (
                        "Exposure latched"
                        if env.first_fault
                        else "Pitcher returned"
                        if env.returned and env.lifted
                        else "Pour in progress"
                        if env.rate > 0.1
                        else "Flow stopped"
                    )
                    draw.text(
                        (x + 12, 352),
                        f"{env.d.time:4.1f} s    {status}",
                        font=font(14),
                        fill="#ffb998" if env.first_fault else "#c1e4d7",
                    )
                p.stdin.write(canvas.tobytes())
                if tick == 210:
                    canvas.save(a.out / "comparison.png")
        finally:
            p.stdin.close()
            if p.wait():
                raise RuntimeError("Video encoding failed")
        for env, c, arr, r, title, subtitle in plants:
            replay = env.report()
            for key in ("score", "received_ml", "on_person_ml", "released_on_counter"):
                if replay[key] != r[key]:
                    raise RuntimeError(f"Replay mismatch: {key}")


if __name__ == "__main__":
    main()
