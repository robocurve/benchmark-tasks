"""Record native simulation pixels and live scorer values in an H.264 MP4."""

from pathlib import Path
import subprocess

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 800, 452, 10
BG, INK, MUTED, ACCENT = "#111a20", "#f2f7f8", "#b7c9cc", "#8ce4d2"


def camera(target, distance, azimuth, elevation):
    cam = mj.MjvCamera()
    cam.lookat[:] = target
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    return cam


def font(size, bold=False):
    return ImageFont.truetype("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf", size)


class Recorder:
    def __init__(self, directory):
        self.path = Path(directory)
        self.frames = 0
        self.final_pixels = None
        self.proc = None
        self.renderers = []
        self.heading, self.body, self.small = font(21, True), font(17), font(13)

    def attach(self, trial):
        self.trial = trial
        self.scene_option = mj.MjvOption()
        self.scene_option.geomgroup[5] = 0
        self.renderers = [
            mj.Renderer(trial.model, height=h, width=w, max_geom=3500)
            for w, h in [(528, 348), (256, 244)]
        ]
        x, y, z = trial.fixture.focus
        self.cameras = [
            camera((x, y + 0.005, z), 0.285, 8, -12),
            camera((x, y - 0.012, z + 0.095), 0.94, 290, -31),
        ]
        argv = [
            "ffmpeg",
            "-nostdin",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{W}x{H}",
            "-framerate",
            str(FPS),
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
            "25",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(self.path / "demo.mp4"),
        ]
        self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE)

    def draw(self):
        canvas = Image.new("RGB", (W, H), BG)
        trial = self.trial
        for i, (renderer, cam) in enumerate(zip(self.renderers, self.cameras)):
            renderer.update_scene(
                trial.data, camera=cam, scene_option=self.scene_option
            )
            renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = True
            view = Image.fromarray(renderer.render())
            if not self.frames and i == 0:
                view.save(self.path / "setup.png")
            canvas.paste(view, (8, 51) if i == 0 else (540, 51))
        draw = ImageDraw.Draw(canvas)
        draw.text((12, 13), trial.fixture.title, font=self.heading, fill=INK)
        read = trial.reading()
        draw.text(
            (550, 305),
            f"Native score {read['score']:.2f}   |   {read['seconds']:.1f} s",
            font=self.body,
            fill=ACCENT,
        )
        readable = [
            (key, value)
            for key, value in read.items()
            if key not in {"score", "step", "phase", "seconds"} and np.isscalar(value)
        ]
        for i, (key, value) in enumerate(readable[:4]):
            text = key.replace("_", " ").capitalize()
            if isinstance(value, bool):
                value = "yes" if value else "no"
            draw.text(
                (550, 334 + 16 * i), f"{text}  {value}", font=self.small, fill=MUTED
            )
        draw.text((12, 417), trial.phase, font=self.body, fill=INK)
        self.final_pixels = canvas.tobytes()
        self.proc.stdin.write(self.final_pixels)
        self.frames += 1

    def close(self):
        if self.proc:
            for _ in range(8):
                if self.final_pixels:
                    self.proc.stdin.write(self.final_pixels)
            self.proc.stdin.close()
            if self.proc.wait() != 0:
                raise RuntimeError("MP4 encoding failed")
        for renderer in self.renderers:
            renderer.close()
