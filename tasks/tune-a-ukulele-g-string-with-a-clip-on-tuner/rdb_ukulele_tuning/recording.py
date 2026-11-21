"""Anonymous RGB evidence. No renderer settings or review images enter policy state.

The overhead view and a review-only side camera (``fixed_scene.side_camera``) are recorded every second control step,
with the first overhead frame as ``setup.png`` and the last frame of each view as an end-of-trial still.
"""

from contextlib import contextmanager
import json
from pathlib import Path

import numpy as np

from .fixed_scene import CONTROL_HZ, side_camera

RT2 = {
    "/rtx/rendermode": "RealTimePathTracing", "/rtx/post/aa/op": 4,
    "/rtx-transient/dlssg/enabled": False, "/rtx-transient/dldenoiser/enabled": True,
    "/rtx/rtpt/cached/enabled": True, "/rtx/rtpt/lightcache/cached/enabled": True,
}
CAMERAS = ("top_cam", "side_cam")
REVIEW_SIZE = (1280, 704)


@contextmanager
def review_camera(headless=True):
    """Start Kit with the validated RT2 profile and add a review-only camera. With ``headless`` False the Isaac Sim
    window opens on DISPLAY or the runtime's browser desktop, as the runtime does for -E headless=false
    (yam_arena/inspect_embodiment.py, _ensure_sim); the trial is the same."""
    import ctypes
    import atexit
    import os
    import yam_arena.inspect_embodiment as inspect_yam

    if inspect_yam._APP is None:
        ctypes.CDLL("libnvidia-ngx.so.1")
        from isaaclab.app import AppLauncher

        os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        window = {}
        if not headless:
            window["visualizer"] = ["kit"]
            os.environ["DISPLAY"] = inspect_yam.window_display(os.environ)
            if os.environ["DISPLAY"] == inspect_yam.DESKTOP_DISPLAY:
                window["window_width"], window["window_height"] = inspect_yam.DESKTOP_SIZE
        inspect_yam._APP = AppLauncher(headless=headless, device="cuda:0", enable_cameras=True,
            renderer="RealTimePathTracing", anti_aliasing=4,
            kit_args="--/persistent/rtx/modes/rt2/enabled=true --/persistent/rtx/modes/rt/enabled=false "
                     "--/persistent/rtx/modes/pt/enabled=false", **window).app
        atexit.register(inspect_yam._close_app_at_exit)

    import yam_arena.embodiment as native
    from isaaclab_physx.renderers import IsaacRtxRendererCfg, IsaacRtxRendererGlobalSettingsCfg

    original = native.BimanualYamEmbodiment
    old_renderer = native.RENDERER
    native.RENDERER = IsaacRtxRendererCfg(global_settings=IsaacRtxRendererGlobalSettingsCfg(
        ambient_light_intensity=0., carb_settings=RT2))
    camera = side_camera()

    class ReviewYam(original):
        def _camera_cfg(self, height, width, overview):
            stock = super()._camera_cfg(height, width, overview)
            # Review only, never a policy input: sharp enough to read the tuner and see the button turn.
            review_width, review_height = REVIEW_SIZE
            side = native.CameraCfg(prim_path="{ENV_REGEX_NS}/SideReviewCam",
                height=review_height, width=review_width, data_types=["rgb"],
                spawn=native._pinhole(camera, review_height, review_width, near=.03),
                offset=native._offset(camera), renderer_cfg=native.RENDERER)
            return native.make_configclass("UkuleleReviewCameras",
                [("side_cam", native.CameraCfg, side)], bases=(type(stock),))()

    native.BimanualYamEmbodiment = ReviewYam
    try:
        yield
    finally:
        native.BimanualYamEmbodiment = original
        native.RENDERER = old_renderer


class Recorder:
    def __init__(self, rig, directory):
        self.rig = rig
        self.directory = Path(directory)
        self.writers = {}
        self.trace = None
        self.frames = 0

    def rgb(self, name):
        raw = self.rig._base.scene[name].data.output["rgb"]
        raw = raw.torch if hasattr(raw, "torch") else raw
        return np.ascontiguousarray(raw[0, ..., :3].cpu().numpy(), dtype=np.uint8)

    def start(self):
        import imageio.v2 as imageio
        from PIL import Image

        # One official trial per output directory. Refuse to overwrite evidence.
        self.directory.mkdir(parents=True, exist_ok=False)
        for name in CAMERAS:
            self.writers[name] = imageio.get_writer(str(self.directory / f"{name}.mp4"), fps=CONTROL_HZ / 2,
                codec="libx264", pixelformat="yuv420p", macro_block_size=1, output_params=["-movflags", "+faststart"])
        self.trace = (self.directory / "grader-trace.jsonl").open("x")
        Image.fromarray(self.rgb("top_cam")).save(self.directory / "setup.png")
        self.frame()

    def frame(self):
        for name, writer in self.writers.items():
            writer.append_data(self.rgb(name))
        self.frames += 1

    def step(self, tick, observation, grade, *, terminal):
        self.trace.write(json.dumps(dict(tick=tick, time_s=observation.state_time, **grade)) + "\n")
        if tick % 2 == 0 or terminal:
            self.frame()

    def close(self):
        if self.writers:
            try:                       # the last frame of each view as a still, for the end-of-trial check
                from PIL import Image

                for name in CAMERAS:
                    Image.fromarray(self.rgb(name)).save(self.directory / f"end_{name}.png")
            except Exception:
                pass
        for writer in self.writers.values():
            writer.close()
        self.writers.clear()
        if self.trace is not None:
            self.trace.close()
            self.trace = None
