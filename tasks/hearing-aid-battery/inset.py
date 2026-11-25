"""Overhead views for the demo's video only (TASK_INSET=<folder>), not policy cameras; they change nothing in the trial.

- `can`: where phone A hangs, straight down over the mould, framing 30 mm, a 360 x 360 picture each control step
  (12 pixels per mm).
- `dishes`: where phone C hangs, 45 cm straight above the middle of the two dishes, framing 200 x 140 mm at
  1200 x 840 pixels: 0.167 mm per pixel, so 1 mm of part movement on a dish is 6 pixels. A JPEG each control step.

Each step also logs where the tweezers' tips are (`tips.csv`), which the video uses to zoom on the dish view at each pick.
Isaac Sim's own render products and annotators; imported only when asked for."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

CAN = dict(name="inset_cam", size=(360, 360), field=0.030, prefix="inset", ext="png")
DISHES = dict(name="dish_cam", size=(1200, 840), field=0.200, prefix="dish", ext="jpg")
_STATE: dict = {"cams": {}}


def _camera(env_path: str, spec: dict, xy, z_cam: float, z_target: float) -> None:
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    path = f"{env_path}/{spec['name']}"
    w, h = spec["size"]
    cam = UsdGeom.Camera.Define(stage, path)
    aperture = 20.955
    focal = aperture * (z_cam - z_target) / spec["field"]
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateVerticalApertureAttr(aperture * h / w)  # square pixels
    cam.CreateFocalLengthAttr(float(focal))  # a USD camera looks down its -z: straight down
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))
    # the renderer culls by environment: a camera sees only its env's partition, so it carries the env's token
    cam.GetPrim().CreateAttribute("omni:scenePartition", Sdf.ValueTypeNames.Token, True, Sdf.VariabilityUniform).Set(env_path.rsplit("/", 1)[-1])
    UsdGeom.Xformable(cam.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), float(z_cam)))
    print(f"[inset] camera {path} over {list(xy)} at z {z_cam:.3f}, focal {focal:.1f} mm, field {spec['field'] * 1000:.0f} mm across "
          f"{w} px: {spec['field'] * 1000 / w:.3f} mm per pixel at z {z_target:.4f}", flush=True)
    rp = rep.create.render_product(path, (w, h))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    _STATE["cams"][spec["name"]] = dict(spec, annot=annot, rp=rp)


def start(env_path: str, can_xy, can_z_cam: float, can_z: float, dish_xy, dish_z_cam: float, dish_z: float) -> None:
    if _STATE.get("started"):
        return
    out = Path(os.environ["TASK_INSET"])
    out.mkdir(parents=True, exist_ok=True)
    _camera(env_path, CAN, can_xy, can_z_cam, can_z)
    _camera(env_path, DISHES, dish_xy, dish_z_cam, dish_z)
    (out / "tips.csv").write_text("step,t,tip_x,tip_y,tip_z\n")
    _STATE.update(started=True, k=0, out=out, blank=0)


def step(t: float, tips_xyz) -> None:
    """Save the latest rendered pictures, and where the tweezers' tips are."""
    from PIL import Image

    k = _STATE["k"] = _STATE["k"] + 1
    out = _STATE["out"]
    with open(out / "tips.csv", "a") as f:
        f.write(f"{k},{t:.3f},{tips_xyz[0]:.5f},{tips_xyz[1]:.5f},{tips_xyz[2]:.5f}\n")
    for c in _STATE["cams"].values():
        data = c["annot"].get_data()
        if data is None or getattr(data, "size", 0) == 0 or np.asarray(data).ndim != 3:
            _STATE["blank"] += 1
            continue
        img = np.asarray(data)[..., :3].astype(np.uint8)
        if img.max() == 0:
            _STATE["blank"] += 1
        path = out / f"{c['prefix']}_{k:05d}.{c['ext']}"
        Image.fromarray(img).save(path, quality=92) if c["ext"] == "jpg" else Image.fromarray(img).save(path)
        if k in (1, 2, 5, 20, 100):
            print(f"[inset] {c['name']} step {k}: shape {img.shape}, mean {img.mean():.1f}, blank so far {_STATE['blank']}", flush=True)
