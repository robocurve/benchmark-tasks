"""Boolean-union printable STL from the same primitives as the simulator.

Run from the repository root:
    .venv/bin/python -m brick_workbench.cad.export_stl

Requires numpy, manifold3d and trimesh. The STL is in millimetres and retains
the body-centre origin used by the matching SCAD/USD source. No OpenSCAD binary
is needed. Existing SCAD files are not modified.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import itertools
import json
import math
from pathlib import Path

import manifold3d
import numpy as np
import trimesh

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import geometry


CYLINDER_FACETS = 96  # Same as generate_brick_scad(); analytic cylinders in USD.
SNAP_DECIMALS_MM = 8  # 10^-8 mm only reconciles coincident floating-point faces.
PLA_DENSITY_G_CM3 = 1.24


def _part_points_mm(part: dict) -> np.ndarray:
    """Exactly the source convex piece, with printable cylinder tessellation."""
    if part["kind"] == "box":
        signs = np.array(list(itertools.product((-1, 1), repeat=3)))
        points = signs * np.asarray(part["size"]) / 2 + part["centre"]
    elif part["kind"] == "cylinder":
        angles = np.arange(CYLINDER_FACETS) * 2 * np.pi / CYLINDER_FACETS
        points = np.array([[part["radius"] * math.cos(angle), part["radius"] * math.sin(angle),
                            sign * part["height"] / 2]
                           for sign in (-1, 1) for angle in angles]) + part["centre"]
    elif part["kind"] == "mesh":
        points = np.asarray(part["points"])
    else:
        raise ValueError(f"unsupported source primitive: {part['kind']}")
    # Shared planes such as roof_bottom must coincide exactly in the Boolean
    # input. Unrounded arithmetic can create ~10^-14 mm zero-volume slivers.
    # This is not a fit adjustment, mesh dilation, repair or added material.
    return np.round(points * 1000, SNAP_DECIMALS_MM)


def export_stl(path: str | Path, shape: tuple[int, int], *,
               interference: float = geometry.DEFAULT_INTERFERENCE,
               segments: int = geometry.DEFAULT_SEGMENTS) -> dict:
    """Export one unioned, single-component solid, then validate the STL bytes."""
    spec = geometry.brick_geometry_specs(shape, interference=interference, segments=segments)
    components = []
    source_points = []
    for part in spec["parts"]:
        points = _part_points_mm(part)
        source_points.append(points)
        # The source mesh pieces are convex and USD explicitly uses convexHull.
        solid = manifold3d.Manifold.hull_points(points)
        if solid.status() != manifold3d.Error.NoError or solid.volume() <= 0:
            raise ValueError(f"invalid source convex piece: {part['name']}")
        components.append(solid)
    solid = manifold3d.Manifold.batch_boolean(components, manifold3d.OpType.Add)
    if solid.status() != manifold3d.Error.NoError or solid.volume() <= 0 or len(solid.decompose()) != 1:
        raise ValueError("Boolean union did not produce one positive-volume connected solid")
    mesh_data = solid.to_mesh64()
    mesh = trimesh.Trimesh(vertices=mesh_data.vert_properties[:, :3], faces=mesh_data.tri_verts, process=False)
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        raise ValueError("Boolean union has inconsistent or open surface topology")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(mesh.export(file_type="stl"))

    # STL has float32, unindexed triangles. Reimporting/welding the actual
    # written bytes catches cracks or collapsed triangles hidden by mesh IDs.
    reread = trimesh.load(path, file_type="stl", force="mesh", process=True)
    imported = manifold3d.Manifold(manifold3d.Mesh(
        np.asarray(reread.vertices, dtype=np.float32), np.asarray(reread.faces, dtype=np.uint32)))
    watertight = bool(reread.is_watertight)
    consistent = bool(reread.is_winding_consistent)
    volume = float(reread.volume)
    connected = len(imported.decompose())
    all_points = np.concatenate(source_points)
    expected_bounds = np.array([all_points.min(axis=0), all_points.max(axis=0)])
    bounds_error = float(np.max(np.abs(reread.bounds - expected_bounds)))
    if (not watertight or not consistent or volume <= 0 or connected != 1
            or imported.status() != manifold3d.Error.NoError or bounds_error > 1e-5
            or not math.isclose(volume, solid.volume(), rel_tol=1e-6)):
        raise ValueError("STL round-trip validation failed")
    return {
        "file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "units": "millimetres", "origin": "body centre; slicer may translate the base to Z=0",
        "shape": spec["shape"], "source_primitives": len(spec["parts"]),
        "source_spec_sha256": hashlib.sha256(json.dumps(spec, sort_keys=True).encode()).hexdigest(),
        "radial_interference_mm": interference * 1000,
        "socket_segments": segments, "stud_facets": CYLINDER_FACETS,
        "input_coordinate_rounding_mm": 10 ** -SNAP_DECIMALS_MM,
        "nominal_bounds_mm": expected_bounds.tolist(), "stl_bounds_mm": reread.bounds.tolist(),
        "size_mm": (expected_bounds[1] - expected_bounds[0]).tolist(),
        "maximum_bounds_error_mm": bounds_error,
        "vertices_after_stl_weld": len(reread.vertices), "triangles": len(reread.faces),
        "watertight": watertight, "winding_consistent": consistent,
        "connected_solid_components": connected, "manifold_status": str(imported.status()),
        "volume_mm3": volume, "volume_cm3": volume / 1000,
        "solid_material_density_g_cm3": PLA_DENSITY_G_CM3,
        "solid_material_mass_g": volume / 1000 * PLA_DENSITY_G_CM3,
        "fit_or_clutch_calibrated": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--interference", type=float, default=geometry.DEFAULT_INTERFERENCE,
                        help="radial interference in metres, matching geometry.py")
    args = parser.parse_args()
    outputs = [export_stl(args.output_dir / f"brick_{n}x{n}.stl", (n, n), interference=args.interference)
               for n in (1, 2)]
    record = {
        "geometry_source": "brick_workbench/geometry.py",
        "geometry_source_sha256": hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest(),
        "exporter_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "dependencies": {name: importlib.metadata.version(name) for name in ("numpy", "manifold3d", "trimesh")},
        "construction": "Boolean union of source convex primitives; no mesh concatenation or added joining material",
        "print_orientation": "open socket mouths toward build plate, studs upward; slicer translates base to Z=0",
        "manufacturing_note": "Roof bridges across socket bores require print testing. Fit, insertion force and retention are uncalibrated.",
        "mass_note": "Geometric solid-material estimate at 1.24 g/cm^3 PLA; actual sliced infill, supports and print affect mass.",
        "files": outputs,
    }
    (args.output_dir / "stl_validation.json").write_text(json.dumps(record, indent=2) + "\n")
    for record in outputs:
        print(f"{record['file']}: {record['triangles']} triangles, watertight single solid, "
              f"{record['volume_cm3']:.4f} cm^3, {record['solid_material_mass_g']:.3f} g solid PLA")


if __name__ == "__main__":
    main()
