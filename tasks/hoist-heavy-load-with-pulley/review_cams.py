"""Extra fixed cameras for reviewing a trial: the ratchet block, the rope and the board from the side and from
further back. They only record video; their images never reach the policy and they don't change the physics.

Enabled by setting ``HOIST_REVIEW_CAMS`` to an output directory (``run_demo.py`` does this unless
``--no-review-cams``); each view is written there as an H.264 MP4 at the control rate (fragmented, so it stays playable if the simulator exits hard;
``run_demo.py`` remuxes it with faststart).
"""

from __future__ import annotations

import atexit
import os
from pathlib import Path

import numpy as np

# name, eye (m, world), target (m, world), focal length (mm, 20.955 mm aperture)
VIEWS = (
    # from the arms' left, a little above the block: the rope over the sheave, the hands and the kettlebell
    ("side_left", (0.30, 1.20, 1.55), (0.56, 0.0, 1.17), 13.0),
    # from the right and in front, looking at the board's face: the tape lines and the kettlebell against them
    ("front_right", (0.0, -1.20, 1.45), (0.64, 0.0, 1.18), 14.0),
    # far back and high over the arms: the whole rig
    ("wide_back", (-0.60, 0.95, 1.90), (0.55, 0.0, 1.12), 14.0),
)
SIZE = (640, 360)
FPS = 20

_STATE: dict = {}


def enabled() -> bool:
    return bool(os.environ.get("HOIST_REVIEW_CAMS"))


def _setup() -> None:
    import imageio_ffmpeg
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    out = Path(os.environ["HOIST_REVIEW_CAMS"]).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    stage = omni.usd.get_context().get_stage()
    views = []
    for name, eye, target, focal in VIEWS:
        cam = UsdGeom.Camera.Define(stage, f"/World/review_{name}")
        cam.GetPrim().CreateAttribute("omni:scenePartition", Sdf.ValueTypeNames.Token).Set("env_0")
        cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())
        cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))
        product = rep.create.render_product(str(cam.GetPath()), SIZE)
        rgb = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
        rgb.attach([product])
        writer = imageio_ffmpeg.write_frames(
            str(out / f"{name}.mp4"), SIZE, fps=FPS, codec="libx264", pix_fmt_out="yuv420p", macro_block_size=8,
            output_params=["-an", "-crf", "20", "-preset", "fast", "-movflags", "+frag_keyframe+empty_moov"],
        )
        writer.send(None)
        views.append((name, rgb, writer))
    _STATE["views"] = views
    atexit.register(close)


def capture() -> None:
    """Grab one frame from each review camera (called once per control step)."""
    if not enabled():
        return
    if "views" not in _STATE:
        _setup()
    for _name, rgb, writer in _STATE["views"]:
        data = rgb.get_data()
        if data is None or getattr(data, "size", 0) == 0 or data.ndim != 3:
            continue  # the first render after creation can be empty
        writer.send(np.ascontiguousarray(data[:, :, :3], dtype=np.uint8))


def close() -> None:
    for _name, _rgb, writer in _STATE.pop("views", ()):
        writer.close()
