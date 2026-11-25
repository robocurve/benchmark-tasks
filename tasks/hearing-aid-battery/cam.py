"""A close-up camera for the demo's video only (TASK_CLOSEUP=<folder>): a fixed oblique view of the hearing aid's battery
door, saving a 640 x 360 picture each control step. It is not a policy camera and changes nothing in the trial. Isaac
Sim's own render product and annotator; imported only when asked for."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

_STATE: dict = {}


def _look_at(eye, target):
    """A USD camera looks along its -z with +y up: the 4x4 (row vectors) placing it at `eye`, looking at `target`."""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    x = np.cross(f, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    m = np.eye(4)
    m[0, :3], m[1, :3], m[2, :3], m[3, :3] = x, y, -f, eye
    return m


def start(env_path: str, origin, eye, target, focal: float = 40.0, size=(640, 360)) -> None:
    if _STATE.get("started"):
        return
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    path = f"{env_path}/closeup_cam"
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(float(focal))
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955 * size[1] / size[0])
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.005, 10.0))
    cam.GetPrim().CreateAttribute("omni:scenePartition", Sdf.ValueTypeNames.Token, True, Sdf.VariabilityUniform).Set(env_path.rsplit("/", 1)[-1])
    m = _look_at(np.asarray(eye, float), np.asarray(target, float))
    UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(Gf.Matrix4d(*[float(v) for v in m.reshape(-1)]))
    rp = rep.create.render_product(path, tuple(size))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    out = Path(os.environ["TASK_CLOSEUP"])
    out.mkdir(parents=True, exist_ok=True)
    _STATE.update(started=True, annot=annot, rp=rp, out=out, k=0, blank=0)
    print(f"[closeup] camera at {np.round(eye, 3).tolist()} looking at {np.round(target, 3).tolist()}, focal {focal}", flush=True)


def step() -> None:
    from PIL import Image

    k = _STATE["k"] = _STATE["k"] + 1
    data = _STATE["annot"].get_data()
    if data is None or getattr(data, "size", 0) == 0 or np.asarray(data).ndim != 3:
        _STATE["blank"] += 1
        return
    img = np.asarray(data)[..., :3].astype(np.uint8)
    Image.fromarray(img).save(_STATE["out"] / f"closeup_{k:05d}.png")
