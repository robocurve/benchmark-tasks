"""An overhead view of the can for the demo's video only (TASK_INSET=<folder>): a camera where the phone hangs, straight
down over the mould, framing 30 mm, saved as a 360 x 360 picture each control step. It is not one of the policy's
cameras and changes nothing in the trial. Isaac Sim's own render product and annotator; imported only when asked for."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

NAME = "inset_cam"
SIZE = 360
FIELD = 0.030  # m framed across at the can's rim: about 12 pixels per mm
_STATE: dict = {}


def start(env_path: str, xy, z_cam: float, z_target: float) -> None:
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    if "annot" in _STATE:
        return
    stage = omni.usd.get_context().get_stage()
    path = f"{env_path}/{NAME}"
    cam = UsdGeom.Camera.Define(stage, path)
    aperture = 20.955
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateVerticalApertureAttr(aperture)
    cam.CreateFocalLengthAttr(float(aperture * (z_cam - z_target) / FIELD))  # a USD camera looks down its -z: straight down
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))
    # the renderer culls by environment: a camera sees only its env's partition, so it carries the env's token
    cam.GetPrim().CreateAttribute("omni:scenePartition", Sdf.ValueTypeNames.Token, True, Sdf.VariabilityUniform).Set(env_path.rsplit("/", 1)[-1])
    UsdGeom.Xformable(cam.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), float(z_cam)))
    world = UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(0)
    print(f"[inset] camera {path} at {list(world.ExtractTranslation())}, focal {aperture * (z_cam - z_target) / FIELD:.1f} mm", flush=True)
    rp = rep.create.render_product(path, (SIZE, SIZE))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    out = Path(os.environ["TASK_INSET"])
    out.mkdir(parents=True, exist_ok=True)
    _STATE.update(annot=annot, rp=rp, k=0, out=out, blank=0)


def step() -> None:
    """Save the latest rendered picture."""
    from PIL import Image

    k = _STATE["k"] = _STATE["k"] + 1
    data = _STATE["annot"].get_data()
    if data is None or getattr(data, "size", 0) == 0 or np.asarray(data).ndim != 3:
        _STATE["blank"] += 1
        return
    img = np.asarray(data)[..., :3].astype(np.uint8)
    if img.max() == 0:
        _STATE["blank"] += 1
    Image.fromarray(img).save(_STATE["out"] / f"inset_{k:05d}.png")
    if k in (1, 2, 5, 20, 100):
        print(f"[inset] step {k}: shape {img.shape}, mean {img.mean():.1f}, blank so far {_STATE['blank']}", flush=True)
