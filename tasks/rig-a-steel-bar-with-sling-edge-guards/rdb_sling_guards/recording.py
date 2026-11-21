"""Anonymous RGB evidence. No renderer settings or review images enter policy state.

Copied from the team's packages (``rdb_spark_plug/recording.py``, itself ``rdb_ukulele_tuning/recording.py``); only the
names, the review camera's pose (``fixed_scene.side_camera``) and a second, low review camera (``low_camera``) differ.
The policy's images are yam_arena's top_cam, left_cam and right_cam only (``inspect_embodiment.DEFAULT_CAMERAS``).
"""

from contextlib import contextmanager
import json
from pathlib import Path

import numpy as np

from . import layout as L
from .fixed_scene import CONTROL_HZ, TOP, side_camera

RT2 = {
    "/rtx/rendermode": "RealTimePathTracing", "/rtx/post/aa/op": 4,
    "/rtx-transient/dlssg/enabled": False, "/rtx-transient/dldenoiser/enabled": True,
    "/rtx/rtpt/cached/enabled": True, "/rtx/rtpt/lightcache/cached/enabled": True,
}
CAMERAS = ("top_cam", "side_cam", "low_cam")
REVIEW_SIZE = (1280, 704)


def low_camera():
    """Review camera on the robot's left, a little above the bar's underside, looking straight across the sling at
    the bar's left side face: the bottom-left edge (under the bar, out of the overhead and side views), the top-left
    edge and the web's left leg show. Not a policy input. Defined here, not in fixed_scene.py, so that adding it
    changed no hashed file."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((L.XS, .42, TOP + .05))
    look = np.array((L.XS, L.HALF, TOP + .07))
    back = eye - look
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(18.))


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
    camera, low = side_camera(), low_camera()

    fast = os.environ.get("RDB_RIGGING_FAST") == "1"      # development only: no review camera (never for evidence)

    class ReviewYam(original):
        def _camera_cfg(self, height, width, overview):
            stock = super()._camera_cfg(height, width, overview)
            if fast:
                return stock
            # Review only, never a policy input: sharp enough to see the guards on the edges and the latches.
            review_width, review_height = REVIEW_SIZE
            views = []
            for name, prim, cam in (("side_cam", "SideReviewCam", camera), ("low_cam", "LowReviewCam", low)):
                views.append((name, native.CameraCfg, native.CameraCfg(prim_path="{ENV_REGEX_NS}/" + prim,
                    height=review_height, width=review_width, data_types=["rgb"],
                    spawn=native._pinhole(cam, review_height, review_width, near=.03),
                    offset=native._offset(cam), renderer_cfg=native.RENDERER)))
            return native.make_configclass("RiggingReviewCameras", views, bases=(type(stock),))()

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
