"""Build the task's geometry asset from the official SO-101 CAD (offline; not needed to run the task).

    python make_assets.py <robotstudio_so101 dir (so101.xml + assets/*.stl)> ../assets/so101_joint1.npz

Input: TheRobotStudio SO-101 description (Apache-2.0), as packaged in the Hugging Face repo
``lerobot/so101-tabletop-mujoco`` folder ``robotstudio_so101/`` (so101.xml + STL meshes, metres).
Needs numpy, trimesh, shapely, mapbox_earcut, fast_simplification.

Output (all in the servo frame S of ``scoring.py``: origin on the joint-1 axis at the top horn's outer face, z up,
x = the arm's front; metres):
* ``vis_<part>_v/_f``: decimated visual meshes of base, base motor holder, servo case, top horn (+ screw), bottom
  horn and the shoulder bracket (bracket in its frame B, which equals S when home at yaw 0);
* ``static_v/_f``: one triangle mesh for the clamped base + servo case collider, horn regions cut out (the horns are
  separate bodies);
* ``piece_<i>``: convex pieces (vertex arrays) of the bracket collider, made by slicing the official bracket mesh
  into z slabs at its feature levels and splitting each cross-section into convex polygons (no hand-drawn shapes);
* ``meta``: JSON with source SHA-256s, slab table and the CAD measurements quoted in task.json.
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union

# Bracket slab boundaries in the bracket frame (mm), from the CAD's cross-section profile:
# bottom cheek, its boss slot, bottom nose, web, top lip, top slot, top cheek, upright, motor-2 holder.
SLABS_MM = [(-46.2, -45.0), (-45.0, -37.4), (-37.4, -36.4), (-36.4, -35.0), (-35.0, -32.0), (-32.0, -24.0),
            (-24.0, -4.0), (-4.0, -1.2), (-1.2, 0.0), (0.0, 1.5), (1.5, 9.5), (9.5, 16.5), (16.5, 19.0),
            (19.0, 38.0)]
# where each slab's cross-section is taken (fraction of the slab height); critical fit slabs use the section that
# has LESS material (no invented interference): the slots and the lip at their middle.
DECIMATE = {'base': 6000, 'base_motor_holder': 6000, 'servo_case': 5000, 'horn_top': 1200, 'horn_bottom': 1200,
            'bracket': 9000}
HORN_R_MM = 10.0
# The CAD lip arc is r 10.11 mm about the bracket's hole-pattern axis (LIP_AXIS_MM, 0.20 mm from the MJCF joint anchor,
# frame B's origin; the slots are centred on it too), 0.11 mm clear of the 20.0 mm horn; the lip slab's section is cut
# back to r 10.10 mm about that axis (0.10 mm radial clearance) with chunky convex sectors (lip_pieces).
LIP_AXIS_MM = (-.201, .022)
LIP_SLAB_MM = (-1.2, 0.0)
NOTCH_SLAB_MM = (-37.4, -36.4)   # bottom cheek's inner-face slab: hole chamfers notch the slot walls here
LIP_CLEAR_R_MM = 10.10


def quat_wxyz(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def vec(s, n=3, default=None):
    return np.array([float(v) for v in s.split()]) if s is not None else default


def geoms(xml_path):
    """{mesh name: [(R, t) world transforms at the zero pose]} for every visual mesh geom (bodies chained)."""
    root = ET.parse(xml_path).getroot()
    out = {}

    def walk(body, R, t):
        for el in body:
            if el.tag == 'geom' and el.get('type') == 'mesh' and el.get('class') == 'visual':
                gp = vec(el.get('pos'), default=np.zeros(3))
                gr = quat_wxyz(vec(el.get('quat'), default=np.array([1., 0, 0, 0])))
                out.setdefault(el.get('mesh'), []).append((R @ gr, R @ gp + t, body.get('name')))
            elif el.tag == 'body':
                bp = vec(el.get('pos'), default=np.zeros(3))
                br = quat_wxyz(vec(el.get('quat'), default=np.array([1., 0, 0, 0])))
                walk(el, R @ br, R @ bp + t)
    walk(root.find('worldbody'), np.eye(3), np.zeros(3))
    return out


def load(stl_dir, name, R, t):
    m = trimesh.load_mesh(stl_dir / f'{name}.stl', process=True)
    m.apply_transform(np.block([[R, t[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]))
    return m


def decimate(m, faces):
    if len(m.faces) <= faces:
        return m
    import fast_simplification
    v, f = fast_simplification.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32),
                                        target_reduction=1 - faces / len(m.faces))
    return trimesh.Trimesh(v, f, process=True)


MAX_EDGE_MM = 1.0  # boundary resampling before triangulation


def hertel_mehlhorn(poly: Polygon):
    """Convex partition of a polygon (with holes): constrained Delaunay triangulation of the polygon with its
    boundary resampled to <= MAX_EDGE_MM, then greedy merging of neighbouring pieces while the union stays convex.
    (An ear-clipping triangulation fans long needle triangles out of single vertices; PhysX's convex cooking of such
    needle prisms stood up to about 0.2 mm proud of the authored faces and lifted / dragged the bracket on the horns,
    found with isaac/probe_fit.py. The Delaunay triangles of the resampled boundary are well shaped.)"""
    import shapely
    dense = shapely.segmentize(poly, MAX_EDGE_MM)
    tris = shapely.constrained_delaunay_triangles(dense)
    pieces = [Polygon(np.asarray(t.exterior.coords)[:-1]) for t in tris.geoms if t.area > 1e-6]
    merged = True
    while merged:
        merged = False
        for i in range(len(pieces)):
            for j in range(i + 1, len(pieces)):
                a, b = pieces[i], pieces[j]
                if a.distance(b) > 1e-9 or a.intersection(b).length < 1e-6:
                    continue
                u = unary_union([a, b])
                if isinstance(u, Polygon) and abs(u.convex_hull.area - u.area) < 1e-6 * max(u.area, 1.):
                    pieces[i] = u.convex_hull
                    pieces.pop(j)
                    merged = True
                    break
            if merged:
                break
    return pieces


def section_polygons(mesh_mm, z, close=False):
    s = mesh_mm.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if s is None:
        return []
    p, _ = s.to_planar(to_2D=np.eye(4))
    polys = []
    for pf in p.polygons_full:
        # screw holes and the motor-2 pocket never touch anything in this task: filled (fewer convex pieces).
        # In the slab at the bottom cheek's inner face (close=True) the holes' edge chamfers break into the lower
        # slot's walls (notches about 0.9 mm wide at the wall); they are filled too, by a 0.6 mm closing (dilate,
        # erode) before taking the outer boundary (fills gaps narrower than 1.2 mm; that slab's slot width, slot end
        # and cheek end are unchanged, checked against the unclosed section)
        g = Polygon(pf.exterior)
        if close:
            g = g.buffer(.6, 32).buffer(-.6, 32)
        rings = list(g.geoms) if isinstance(g, MultiPolygon) else [g]
        for r in rings:
            r = Polygon(r.exterior).simplify(.05, preserve_topology=True).buffer(0)
            polys += list(r.geoms) if isinstance(r, MultiPolygon) else [r]
    return [g for g in polys if g.area > .5]


def lip_pieces(poly):
    """Convex pieces of the lip slab. Near the horn (r < 12.5 mm) one chunky sector per edge of the 64-gon cut
    (inner edge = that chord, two radial sides, outer chord at 12.5 mm), so that no piece is a long sliver: PhysX's
    convex-hull cooking (tolerances set for metre-scale parts) re-fitted the slivers that earcut leaves along the
    cut and stopped the horn about 0.5 mm short of the lip (found with isaac/probe_fit.py). Farther out: as usual."""
    from shapely.geometry import Point
    cx, cy = LIP_AXIS_MM
    near_disk = Point(cx, cy).buffer(12.5, 16)  # 64-gon with the same vertex angles as the cut
    out = []
    near = poly.intersection(near_disk)
    far = poly.difference(near_disk)
    a = np.linspace(0, 2 * np.pi, 65)
    for k in range(64):
        sector = Polygon([(cx + r * np.cos(a[j]), cy + r * np.sin(a[j]))
                          for r, j in ((LIP_CLEAR_R_MM, k), (12.5, k), (12.5, k + 1), (LIP_CLEAR_R_MM, k + 1))])
        cell = near.intersection(sector)
        for g in (list(cell.geoms) if hasattr(cell, 'geoms') else [cell]):
            if isinstance(g, Polygon) and g.area > .05:
                # a cell within 0.05 mm2 of convex (the lip's end corners) is taken as its convex hull: the hull stays
                # inside the cell's sector, behind its chord, so nothing comes nearer the horn, and no sliver piece is
                # left (a 0.1 mm wide sliver at the lip's end was cooked about 0.5 mm proud of its authored face and
                # pushed the seated bracket 0.44 mm sideways: found with a free bracket placed at home, no arm)
                out += [g.convex_hull] if g.convex_hull.area - g.area < .05 else hertel_mehlhorn(g)
    for g in (list(far.geoms) if hasattr(far, 'geoms') else [far]):
        if isinstance(g, Polygon) and g.area > .01:
            out += hertel_mehlhorn(g)
    return out


def bracket_pieces(mesh_mm):
    pieces, table = [], []
    for z0, z1 in SLABS_MM:
        zc = .5 * (z0 + z1)
        n0 = len(pieces)
        polys = section_polygons(mesh_mm, zc, close=(z0, z1) == NOTCH_SLAB_MM)
        if (z0, z1) == LIP_SLAB_MM:
            from shapely.geometry import Point
            cut = Point(*LIP_AXIS_MM).buffer(LIP_CLEAR_R_MM, 16)  # 64-gon: edges at r >= 10.088 mm
            # the CAD arc (r 10.09-10.12 mm) weaves across the 64-gon: fill the sliver gaps between them (within 0.1 mm
            # of the lip), so that the lip's inner face is the 64-gon's chords (convex sectors, no slivers)
            band = Point(*LIP_AXIS_MM).buffer(10.2, 16).difference(cut)
            polys = [p.union(band.intersection(p.buffer(.1))) for p in polys]
            polys = [g for p in polys for g in ([p.difference(cut)] if isinstance(p.difference(cut), Polygon)
                                                 else list(p.difference(cut).geoms)) if g.area > .5]
        convex = []
        for poly in polys:
            if (z0, z1) == LIP_SLAB_MM:
                convex += lip_pieces(poly)
            else:
                convex += hertel_mehlhorn(poly)
        for cp in convex:
            if cp.area >= .02:  # specks (< 0.02 mm2, from clipping) are left out
                xy = np.asarray(cp.exterior.coords)[:-1]
                v = np.concatenate([np.c_[xy, np.full(len(xy), z0)], np.c_[xy, np.full(len(xy), z1)]])
                pieces.append(v / 1000.)
        table.append(dict(z_mm=[z0, z1], section_at_mm=zc, pieces=len(pieces) - n0))
    return pieces, table


def split_servo(m_mm):
    """Servo mesh -> case, top horn (+ its screw), bottom horn, by face centroid (visual use only)."""
    c = m_mm.triangles_center
    r = np.hypot(c[:, 0], c[:, 1])
    top = (r <= HORN_R_MM + .05) & (c[:, 2] > -2.6)
    bottom = (r <= HORN_R_MM + .05) & (c[:, 2] < -34.35)
    case = ~(top | bottom)
    return {k: m_mm.submesh([np.nonzero(sel)[0]], append=True) for k, sel in
            [('servo_case', case), ('horn_top', top), ('horn_bottom', bottom)]}


def main(src_dir, out_path):
    src = Path(src_dir)
    stl = src / 'assets'
    g = geoms(src / 'so101.xml')
    anchor = g['rotation_pitch_so101_v1'][0]  # bracket transform at zero pose (shoulder body)
    # servo frame S: joint-1 anchor = shoulder body origin (MJCF body "shoulder" pos), axes = base axes (z up)
    root = ET.parse(src / 'so101.xml').getroot()
    sh = root.find(".//body[@name='shoulder']")
    o_s = vec(sh.get('pos'))
    to_s = lambda m: m.apply_translation(-o_s)  # noqa: E731
    parts = {}
    for name in ('base_so101_v2', 'base_motor_holder_so101_v1', 'sts3215_03a_v1'):
        R, t, body = [x for x in g[name] if x[2] == 'base'][0]
        parts[name] = to_s(load(stl, name, R, t))
    R, t, _ = anchor
    bracket = to_s(load(stl, 'rotation_pitch_so101_v1', R, t))
    mm = lambda m: m.copy().apply_scale(1000.)  # noqa: E731
    servo = split_servo(mm(parts['sts3215_03a_v1']))
    out, meta = {}, dict(source={}, units='m', frame='servo frame S (see scoring.py)')
    for f in sorted(stl.glob('*.stl')):
        if f.stem in ('base_so101_v2', 'base_motor_holder_so101_v1', 'sts3215_03a_v1', 'rotation_pitch_so101_v1'):
            meta['source'][f.name] = hashlib.sha256(f.read_bytes()).hexdigest()
    meta['source']['so101.xml'] = hashlib.sha256((src / 'so101.xml').read_bytes()).hexdigest()
    vis = {'base': parts['base_so101_v2'], 'base_motor_holder': parts['base_motor_holder_so101_v1'],
           'bracket': bracket}
    vis.update({k: v.copy().apply_scale(.001) for k, v in servo.items()})
    for k, m in vis.items():
        d = decimate(m, DECIMATE[k])
        d.update_faces(d.nondegenerate_faces(height=1e-7))  # no zero-area triangles (NaN normals in the renderer)
        d.update_faces(d.unique_faces())
        d.remove_unreferenced_vertices()
        out[f'vis_{k}_n'] = d.vertex_normals.astype(np.float32)
        out[f'vis_{k}_v'] = d.vertices.astype(np.float32)
        out[f'vis_{k}_f'] = d.faces.astype(np.int32)
        meta.setdefault('visual_faces', {})[k] = int(len(d.faces))
    # static collider: base + holder + servo case without the horn cylinders' space (r < 10.3 mm near the horns)
    static = trimesh.util.concatenate([parts['base_so101_v2'], parts['base_motor_holder_so101_v1'],
                                       servo['servo_case'].copy().apply_scale(.001)])
    tri = static.triangles * 1000
    r = np.hypot(tri[..., 0], tri[..., 1])
    zone = (r < 10.4) & ((tri[..., 2] > -3.2) | (tri[..., 2] < -33.9))  # any vertex in a horn's space
    keep = ~zone.any(axis=1)
    static = decimate(static.submesh([np.nonzero(keep)[0]], append=True), 16000)
    out['static_v'] = static.vertices.astype(np.float32)
    out['static_f'] = static.faces.astype(np.int32)
    pieces, table = bracket_pieces(mm(bracket))
    for i, p in enumerate(pieces):
        out[f'piece_{i}'] = p.astype(np.float64)
    meta['pieces'] = len(pieces)
    meta['slabs'] = table
    bb = mm(bracket).bounds
    meta['bracket_bounds_mm'] = np.round(bb, 2).tolist()
    meta['bracket_volume_cm3'] = round(float(bracket.volume) * 1e6, 2)
    meta['base_bottom_z_mm'] = round(float(min(parts['base_so101_v2'].bounds[0][2],
                                               parts['base_motor_holder_so101_v1'].bounds[0][2]) * 1000), 2)
    out['meta'] = np.array(json.dumps(meta))
    np.savez_compressed(out_path, **out)
    print(json.dumps({k: v for k, v in meta.items() if k != 'slabs'}, indent=1))
    for row in table:
        print(row)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
