"""Capture actual frames on the candy reference layout at the control rate."""

from contextlib import contextmanager
import os
import subprocess

import cv2
import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("EGL_PLATFORM", "surfaceless")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("MESA_SHADER_CACHE_DISABLE", "true")

import mujoco


POLICY_CAMERAS = ("top_cam", "left_cam", "right_cam")
CAMERAS = (*POLICY_CAMERAS, "judging_front")
CONTROL_HZ = 20


class StationViews:
    def __init__(self, model, cameras):
        if set(cameras) != set(CAMERAS):
            raise ValueError(f"Supply the four standard view roles {CAMERAS}")
        for camera in cameras.values():
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera) < 0:
                raise ValueError(f"Missing camera {camera}")
        self.cameras = cameras
        self.renderer = mujoco.Renderer(model, height=480, width=640)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3:] = 0

    def capture(self, data):
        views = {}
        for name in CAMERAS:
            self.renderer.update_scene(data, camera=self.cameras[name], scene_option=self.options)
            pixels = self.renderer.render()
            views[name] = (pixels if name == "judging_front" else pixels[60:420]).copy()
        return views

    def close(self):
        self.renderer.close()


def policy_images(views):
    return {
        name: cv2.resize(views[name], (320, 176), interpolation=cv2.INTER_AREA)
        for name in POLICY_CAMERAS
    }


def _label(frame, label, origin):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (width, height), baseline = cv2.getTextSize(label, font, .45, 1)
    x, y = origin
    cv2.rectangle(frame, (x - 3, y - height - 3), (x + width + 3, y + baseline + 2), (0, 0, 0), -1)
    cv2.putText(frame, label, origin, font, .45, (255, 255, 255), 1, cv2.LINE_AA)


def compose_review(views):
    """One large fixed detail view, then overhead and both wrists as in the accepted candy clip."""
    frame = np.zeros((720, 1360, 3), dtype=np.uint8)
    frame[:, :960] = cv2.resize(views["judging_front"], (960, 720), interpolation=cv2.INTER_LINEAR)
    _label(frame, "Detail camera", (16, 23))
    for i, (camera, label) in enumerate(zip(POLICY_CAMERAS, ("Overhead", "Left wrist", "Right wrist"))):
        y = 240 * i
        frame[y+15:y+240, 960:] = cv2.resize(views[camera], (400, 225), interpolation=cv2.INTER_AREA)
        _label(frame, label, (968, y + 12))
    return frame


def save_setup(views, path):
    frame = np.zeros((480, 1280, 3), dtype=np.uint8)
    frame[:, :640] = views["judging_front"]
    frame[60:420, 640:] = views["top_cam"]
    if not cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"Unable to save setup image to {path}")


@contextmanager
def review_writer(path):
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-n",
        "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "1360x720",
        "-framerate", str(CONTROL_HZ), "-i", "-", "-an", "-c:v", "libx264",
        "-threads", "2", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(path),
    ]
    with subprocess.Popen(command, stdin=subprocess.PIPE) as process:
        try:
            yield lambda views: process.stdin.write(compose_review(views).tobytes())
        finally:
            process.stdin.close()
            if process.wait():
                raise RuntimeError(f"Review video encoding failed for {path}")
