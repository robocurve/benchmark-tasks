"""Measure the library scissors (scissors_ycb_robolab) for scissors_handoff.py. Run it once, offline from the
simulator, in the IsaacLab-Arena environment:

    source ~/yam/IsaacLab-Arena/yam.env
    python tasks/scissors-handoff/measure_scissors.py OUT_DIR

It downloads the USD and the layers it composes (not the textures) into OUT_DIR. Then it writes
OUT_DIR/scissors_vertices.npy: every mesh vertex, unscaled, in the frame that the arena bounding-box helper uses.
It also prints the bounds, the long axis, and the width profile along that axis, so that we can find the blades,
the pivot and the finger rings.
"""

import sys
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
from isaaclab_arena.utils.usd.helpers import _read_default_prim_scale, compute_local_bounding_box_from_usd
from pxr import Sdf, Usd, UsdGeom

URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/"
    "Arena/assets/object_library/srl_robolab_assets/objects/ycb/scissors.usd"
)
HEADERS = {"User-Agent": "robocurve-measure/1.0"}


def fetch(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not dest.exists():
        with urllib.request.urlopen(urllib.request.Request(url, headers=HEADERS), timeout=120) as res:
            dest.write_bytes(res.read())


def fetch_layers(url: str, dest: Path, seen: set[str]) -> None:
    """Download a layer and, recursively, its sublayers, references and payloads."""
    if url in seen:
        return
    seen.add(url)
    fetch(url, dest)
    layer = Sdf.Layer.FindOrOpen(str(dest))
    for dep in layer.GetCompositionAssetDependencies():
        if "://" in dep or dep.startswith("/"):
            print(f"skipped a non-relative dependency: {dep}")
            continue
        fetch_layers(urllib.parse.urljoin(url, dep), (dest.parent / dep).resolve(), seen)


def vertices(stage: Usd.Stage) -> np.ndarray:
    """All mesh points relative to the default prim's origin, with its root scale removed (as
    isaaclab_arena.utils.usd.helpers.compute_local_bounding_box_from_usd does)."""
    root = stage.GetDefaultPrim()
    cache = UsdGeom.XformCache()
    root_tf = cache.GetLocalToWorldTransform(root)
    origin = np.array(root_tf.ExtractTranslation())
    scale = np.array(_read_default_prim_scale(root))
    out = []
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            tf = cache.GetLocalToWorldTransform(prim)
            pts = [tf.Transform(p) for p in UsdGeom.Mesh(prim).GetPointsAttr().Get()]
            out.append((np.array(pts) - origin) / scale)
            print(f"mesh {prim.GetPath()}: {len(pts)} points")
    return np.vstack(out)


def main() -> None:
    out_dir = Path(sys.argv[1]).resolve()
    usd = out_dir / "scissors.usd"
    fetch_layers(URL, usd, set())
    stage = Usd.Stage.Open(str(usd))
    v = vertices(stage)
    np.save(out_dir / "scissors_vertices.npy", v)

    lo, hi = v.min(0), v.max(0)
    ref = compute_local_bounding_box_from_usd(str(usd))
    print(f"arena helper bounds: lo {np.round(ref.min_point[0].tolist(), 4)} hi {np.round(ref.max_point[0].tolist(), 4)}")
    print(f"up axis {UsdGeom.GetStageUpAxis(stage)}, meters/unit {UsdGeom.GetStageMetersPerUnit(stage)}")
    print(f"bounds lo {np.round(lo, 4)} hi {np.round(hi, 4)} size {np.round(hi - lo, 4)}")
    centred = v[:, :2] - v[:, :2].mean(0)
    axis = np.linalg.svd(centred, full_matrices=False)[2][0]
    print(f"long axis in xy: {np.round(axis, 4)}")
    along = v[:, :2] @ axis
    across = v[:, :2] @ np.array([-axis[1], axis[0]])
    edges = np.linspace(along.min(), along.max(), 21)
    print("width profile along the long axis (start, width across, height):")
    for a, b in zip(edges[:-1], edges[1:]):
        sel = (along >= a) & (along < b)
        if sel.any():
            print(f"  {a:+.4f}  {np.ptp(across[sel]):.4f}  {np.ptp(v[sel, 2]):.4f}")


if __name__ == "__main__":
    main()
