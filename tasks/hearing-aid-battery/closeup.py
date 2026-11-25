"""A close-up camera for the demo's video only (TASK_CLOSEUP=<folder>): it follows the tweezers' tips from the side and
saves a 640 x 360 picture every other control step. It is not one of the policy's cameras and changes nothing in the
trial. Isaac Sim's own render product and annotator; imported only when asked for."""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

PATH = "/World/closeup_cam"
_STATE: dict = {}


def _look_at(eye, target):
    """A USD camera looks along its -z with +y up: the rotation (as a 4x4, row vectors) placing it at `eye`."""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(f, up)
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    m = np.eye(4)
    m[0, :3], m[1, :3], m[2, :3], m[3, :3] = x, y, -f, eye
    return m


def start(origin) -> None:
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, UsdGeom

    if "annot" in _STATE:
        return
    stage = omni.usd.get_context().get_stage()
    cam = UsdGeom.Camera.Define(stage, PATH)
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.005, 10.0))
    _STATE["xf"] = UsdGeom.Xformable(cam.GetPrim())
    _STATE["op"] = _STATE["xf"].AddTransformOp()
    rp = rep.create.render_product(PATH, (640, 360))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    _STATE.update(annot=annot, origin=np.asarray(origin, float), k=0, out=Path(os.environ["TASK_CLOSEUP"]))
    _STATE["out"].mkdir(parents=True, exist_ok=True)


def step(target, side=(0.075, -0.045, 0.035)) -> None:
    """Point the camera at `target` (station frame) and save the last rendered picture."""
    from pxr import Gf
    from PIL import Image

    k = _STATE["k"] = _STATE["k"] + 1
    eye = np.asarray(target) + np.asarray(side)
    m = _look_at(eye + _STATE["origin"], np.asarray(target) + _STATE["origin"])
    _STATE["op"].Set(Gf.Matrix4d(*[float(v) for v in m.reshape(-1)]))
    if k % 2:
        return
    data = _STATE["annot"].get_data()
    if data is None or getattr(data, "size", 0) == 0:
        return
    img = np.asarray(data)[..., :3].astype(np.uint8)
    Image.fromarray(img).save(_STATE["out"] / f"closeup_{k:05d}.png")
