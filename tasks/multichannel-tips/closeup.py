"""A close-up of the nozzle row and column 4 for the demo's video only (TASK_CLOSEUP=<folder>): a fixed camera on the
arms' side of the box, just above its rim, looking along +x across the empty columns 1-3 at column 4, so that the eight
nozzles can be seen going into their tips and the tips coming up on them. It saves a 640 x 360 picture each control
step. It is not one of the policy's cameras and changes nothing in the trial: no body, collider or light is added, and
the policy's cameras and observations are as without it. Isaac Sim's own render product and annotator; imported only
when asked for."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

NAME = "closeup_cam"
SIZE = (640, 360)
TOP = 0.7475  # the station table's top (tipkit.TOP)
TARGET = (0.265, 0.0, TOP + 0.064)  # column 4, a little above the tips' mouths (58 mm up)
EYE = (0.150, -0.012, TOP + 0.082)  # over the bench on the arms' side, 30 mm above the box's rim
FOCAL = 21.0  # mm, with a 20.955 mm aperture: about 130 mm across at column 4
_STATE: dict = {}


def _look_at(eye, target) -> np.ndarray:
    """A USD camera looks along its -z with +y up: its local-to-parent matrix (row vectors) at `eye`, aimed at `target`."""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    x = np.cross(f, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    m = np.eye(4)
    m[0, :3], m[1, :3], m[2, :3], m[3, :3] = x, y, -f, eye
    return m


def start(env_path: str) -> None:
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    if "annot" in _STATE:
        return
    stage = omni.usd.get_context().get_stage()
    path = f"{env_path}/{NAME}"
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955 * SIZE[1] / SIZE[0])
    cam.CreateFocalLengthAttr(FOCAL)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.005, 10.0))
    # the renderer culls by environment: a camera sees only its env's partition, so it carries the env's token
    cam.GetPrim().CreateAttribute("omni:scenePartition", Sdf.ValueTypeNames.Token, True, Sdf.VariabilityUniform).Set(env_path.rsplit("/", 1)[-1])
    m = _look_at(EYE, TARGET)
    UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(Gf.Matrix4d(*[float(v) for v in m.reshape(-1)]))
    world = UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(0)
    print(f"[closeup] camera {path} at {list(world.ExtractTranslation())}", flush=True)
    rp = rep.create.render_product(path, SIZE)
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    out = Path(os.environ["TASK_CLOSEUP"])
    out.mkdir(parents=True, exist_ok=True)
    _STATE.update(annot=annot, rp=rp, k=0, out=out, blank=0)


def step() -> None:
    """Save the latest rendered picture, named by control step."""
    from PIL import Image

    k = _STATE["k"] = _STATE["k"] + 1
    data = _STATE["annot"].get_data()
    if data is None or getattr(data, "size", 0) == 0 or np.asarray(data).ndim != 3:
        _STATE["blank"] += 1
        return
    img = np.asarray(data)[..., :3].astype(np.uint8)
    if img.max() == 0:
        _STATE["blank"] += 1
    Image.fromarray(img).save(_STATE["out"] / f"closeup_{k:05d}.png")
    if k in (1, 2, 5, 20, 100):
        print(f"[closeup] step {k}: shape {img.shape}, mean {img.mean():.1f}, blank so far {_STATE['blank']}", flush=True)
