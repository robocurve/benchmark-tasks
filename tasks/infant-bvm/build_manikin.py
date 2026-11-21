"""Build the infant manikin's body from the MakeHuman base mesh (hm08, CC0) and MakeHuman's own baby targets (CC0).

    mkdir -p build/dl_makehuman && cd build/dl_makehuman
    curl -L -O https://raw.githubusercontent.com/makehumancommunity/makehuman/master/makehuman/data/3dobjs/base.obj
    for e in african asian caucasian; do for s in female male; do curl -L -O \\
      https://raw.githubusercontent.com/makehumancommunity/makehuman/master/makehuman/data/targets/macrodetails/$e-$s-baby.target
    done; done
    cd ../.. && python build_manikin.py

The six ethnic/sex baby targets are mixed equally (MakeHuman's defaults at age 0), the A-pose arms are lowered 25
degrees toward the sides, and the body is scaled 1.08 (650 mm long, 140 mm high: the Little Baby QCPR's published
size). It lies face up, head toward the robot, eyes looking straight up (the maker's neutral).

Writes build/manikin.npz (manikin frame: station axes, origin on the mat top under the crown, on the midline):
- the torso (visual mesh, its convex pieces from CoACD) and the head (visual mesh, eyes, the face collider: the head
  closed at the neck and cut off below 25 mm, so only the occiput sphere touches the mat), the head's frame being the
  neck hinge (the centre of the occiput's curve);
- the chest patch round the sternum (visual, rises with the lungs) and the landmarks: nasion, chin point, eyes, the
  nominal mask pose on the face, the rim's face heights (the anatomical cushion), the sternum point.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import trimesh

import geometry as g

HERE = Path(__file__).resolve().parent
DL = HERE / "build" / "dl_makehuman"
MM = 1e-3


def read_obj():
    V, groups, grp = [], {}, None
    with open(DL / "base.obj") as f:
        for line in f:
            if line.startswith("v "):
                V.append([float(t) for t in line.split()[1:4]])
            elif line.startswith("g "):
                grp = line.split()[1]
            elif line.startswith("f "):
                idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
                groups.setdefault(grp, []).extend([idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1))
    return np.array(V), {k: np.array(v) for k, v in groups.items()}


def baby(V):
    off = np.zeros_like(V)
    for e in ("african", "asian", "caucasian"):
        for s in ("female", "male"):
            for line in open(DL / f"{e}-{s}-baby.target"):
                if line[0].isdigit():
                    t = line.split()
                    off[int(t[0])] += np.array([float(x) for x in t[1:4]]) / 6.0
    return (V + off) * 100.0            # MakeHuman decimetres -> mm


def lower_feet(P, groups):
    """Point the feet 50 degrees down (about each ankle's x axis), blended over 20 mm above the ankle, then lift each
    foot back so its heel is no lower than before: lying face up, the toes no longer stand 180 mm high."""
    P = P.copy()
    for side in ("l", "r"):
        an = P[np.unique(groups[f"joint-{side}-ankle"])].mean(0)
        near = (np.abs(P[:, 0] - an[0]) < 40.0) & (P[:, 0] * an[0] > 0)
        u = (an[1] + 10.0 - P[:, 1]) / 20.0
        w = np.clip(u, 0, 1)
        w = w * w * (3 - 2 * w) * near
        foot = near & (P[:, 1] < an[1] + 10.0)
        z_heel = P[foot, 2].min()
        a = math.radians(50.0) * w
        c, s = np.cos(a), np.sin(a)
        dy, dz = P[:, 1] - an[1], P[:, 2] - an[2]
        P[:, 1], P[:, 2] = an[1] + c * dy - s * dz, an[2] + s * dy + c * dz
        P[:, 2] += w * max(0.0, z_heel - P[foot, 2].min())
    return P


def lower_arms(P, groups):
    """Rotate each arm about its shoulder joint toward the body (about MakeHuman's z, forward) and back (about x), so
    it lies along the side, blended over 30 mm."""
    P = P.copy()
    for side, sgn in (("l", 1.0), ("r", -1.0)):
        sh = P[np.unique(groups[f"joint-{side}-shoulder"])].mean(0)
        hand = P[np.unique(groups[f"joint-{side}-hand"])].mean(0)
        ax = (hand - sh) / np.linalg.norm(hand - sh)
        t = (P - sh) @ ax
        off = np.linalg.norm((P - sh) - np.outer(t, ax), axis=1)
        w = np.clip((t + 10.0) / 30.0, 0, 1)
        f = np.clip((70.0 - off) / 30.0, 0, 1)
        w = w * w * (3 - 2 * w) * f * f * (3 - 2 * f)
        a = -sgn * g.ARM_DROP * w
        c, s = np.cos(a), np.sin(a)
        dx, dy = P[:, 0] - sh[0], P[:, 1] - sh[1]
        P[:, 0], P[:, 1] = sh[0] + c * dx - s * dy, sh[1] + s * dx + c * dy
        b = g.ARM_BACK * w                              # and back (about x), so the arms lie along the sides
        c, s = np.cos(b), np.sin(b)
        dy, dz = P[:, 1] - sh[1], P[:, 2] - sh[2]
        P[:, 1], P[:, 2] = sh[1] + c * dy - s * dz, sh[2] + s * dy + c * dz
    return P


def fan_cap(m: trimesh.Trimesh) -> trimesh.Trimesh:
    """Close each boundary loop with a fan round its centroid, oriented outward."""
    import networkx as nx
    edges = m.edges[trimesh.grouping.group_rows(m.edges_sorted, require_count=1)]
    G = nx.Graph()
    G.add_edges_from(edges)
    V, F = m.vertices, [m.faces]
    extra = []
    for comp in nx.connected_components(G):
        comp = set(comp)
        loop = [e for e in edges if e[0] in comp]
        c = V[list(comp)].mean(0)
        ci = len(V) + len(extra)
        extra.append(c)
        # the boundary edges as stored in the faces run one way round the hole; the cap runs the other way
        F.append(np.array([[b, a, ci] for a, b in loop]))
    m2 = trimesh.Trimesh(np.vstack([V, extra]), np.vstack(F), process=True)
    return m2


def fit_sphere(P):
    A = np.c_[2 * P, np.ones(len(P))]
    b = (P ** 2).sum(1)
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    c = x[:3]
    return c, math.sqrt(x[3] + c @ c)


def main():
    V0, groups = read_obj()
    P = lower_feet(lower_arms(baby(V0), groups), groups)                     # MakeHuman frame, mm: x left, y up, z forward
    body = groups["body"]
    y_crown = P[np.unique(body)][:, 1].max()

    # the neck cut: a plane through the neck below the jaw (front) and the nape (back); the head is above it
    p0, p1 = np.array([-356.0, 33.0]), np.array([-338.0, -28.0])
    d = p1 - p0
    n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
    n = n if n[0] > 0 else -n
    side = (P[:, 1] - p0[0]) * n[0] + (P[:, 2] - p0[1]) * n[1]
    in_head = (side[body] > 0).all(1)

    def to_frame(Q, z_low):
        """MakeHuman mm -> manikin frame m (x toward the feet from the crown, y the manikin's left, z up)."""
        s = g.MH_SCALE * MM
        return np.c_[(y_crown - Q[:, 1]) * s, Q[:, 0] * s, (Q[:, 2] - z_low) * s]

    # -- the head: the occiput's curve -> the neck hinge -------------------------------------------------------------
    head = trimesh.Trimesh(P, body[in_head], process=True)
    head.remove_unreferenced_vertices()
    back = head.vertices[(head.vertices[:, 2] < -25) & (head.vertices[:, 1] > -320) & (head.vertices[:, 1] < -235)
                         & (np.abs(head.vertices[:, 0]) < 35)]
    c_mh, r_mh = fit_sphere(back)
    z_low_head = c_mh[2] - r_mh                          # the sphere's bottom on the mat
    head = trimesh.Trimesh(*trimesh.remesh.subdivide_loop(head.vertices, head.faces, iterations=1), process=True)
    head = fan_cap(head)
    print("head watertight", head.is_watertight, "faces", len(head.faces), "occiput r", round(r_mh * g.MH_SCALE, 1),
          "mm")
    Hv = to_frame(head.vertices, z_low_head)
    hinge = to_frame(c_mh[None], z_low_head)[0]
    hinge[1] = 0.0
    R_occ = r_mh * g.MH_SCALE * MM
    head_w = trimesh.Trimesh(Hv - hinge, head.faces, process=False)
    head_w.fix_normals()
    # the face collider: the closed head cut off below 25 mm above the mat (the occiput sphere carries it there)
    cut = head_w.slice_plane(plane_origin=[0, 0, 0.025 - hinge[2]], plane_normal=[0, 0, 1])
    cut.merge_vertices()
    col = fan_cap(cut)
    col.fix_normals()
    if not col.is_watertight:
        bad = col.edges_sorted[trimesh.grouping.group_rows(col.edges_sorted, require_count=1)]
        print("face collider open edges", len(bad), np.round(col.vertices[np.unique(bad)].mean(0) * 1000, 1))
    print("face collider watertight", col.is_watertight, "faces", len(col.faces), "lowest of the visual head",
          round(float(Hv[:, 2].min()) * 1000, 2), "mm")
    eyes = []
    for k in ("helper-l-eye", "helper-r-eye"):
        F = groups[k]
        used = np.unique(F)
        remap = -np.ones(len(P), int)
        remap[used] = np.arange(len(used))
        eyes.append((to_frame(P[used], z_low_head) - hinge, remap[F]))
    eye_c = np.array([e[0].mean(0) for e in eyes])

    # -- the torso ------------------------------------------------------------------------------------------------------
    torso = trimesh.Trimesh(P, body[~in_head], process=True)
    torso.remove_unreferenced_vertices()
    torso = fan_cap(torso)
    z_low_torso = torso.vertices[:, 2].min()
    Tv = to_frame(torso.vertices, z_low_torso)
    # the torso as it rests by itself on the flat mat: pitch it about y until its centre of mass lies over its support
    com = trimesh.Trimesh(Tv, torso.faces, process=False).center_mass
    best = None
    for phi in np.radians(np.arange(-8.0, 8.01, 0.05)):
        c, s_ = math.cos(phi), math.sin(phi)
        Rp = np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
        W = Tv @ Rp.T
        W[:, 2] -= W[:, 2].min()
        sup = W[W[:, 2] < 0.0008]
        cm = Rp @ com
        ok = sup[:, 0].min() + 0.01 < cm[0] < sup[:, 0].max() - 0.01 and sup[:, 1].min() < cm[1] < sup[:, 1].max()
        if ok and (best is None or abs(phi) < abs(best[0])):
            best = (phi, Rp, sup[:, 0].min(), sup[:, 0].max())
    phi, Rp = best[0], best[1]
    T_rest = Tv @ Rp.T
    dz_rest = float(T_rest[:, 2].min())
    Tv = T_rest - np.array([0.0, 0.0, dz_rest])
    print(f"torso rests pitched {math.degrees(phi):.2f} deg on x {best[2] * 1000:.0f}..{best[3] * 1000:.0f} mm, "
          f"com x {(Rp @ com)[0] * 1000:.0f}")
    torso_w = trimesh.Trimesh(Tv, torso.faces, process=False)
    torso_w.fix_normals()
    print("torso watertight", torso_w.is_watertight, "faces", len(torso_w.faces), "bbox mm",
          np.round(torso_w.bounds * 1000, 1).tolist(), "head over torso at the neck mm",
          round((z_low_torso - z_low_head) * g.MH_SCALE, 1))
    import coacd
    coacd.set_log_level("error")
    parts = coacd.run_coacd(coacd.Mesh(torso_w.vertices, torso_w.faces), threshold=0.06, max_convex_hull=28,
                            preprocess_resolution=40, resolution=1500, seed=1)
    hulls = [(np.asarray(v), np.asarray(f)) for v, f in parts]
    # the torso rests on its convex pieces, which reach a little below the mesh: lift it so they meet the mat at z 0
    hz = min(float(v[:, 2].min()) for v, _ in hulls)
    hulls = [(v - np.array([0.0, 0.0, hz]), f) for v, f in hulls]
    Tv = Tv - np.array([0.0, 0.0, hz])
    torso_w = trimesh.Trimesh(Tv, torso.faces, process=False)
    torso_w.fix_normals()
    dz_rest += hz
    print("torso hulls", len(hulls), "lifted", round(-hz * 1000, 2), "mm")

    # -- landmarks (manikin frame) -----------------------------------------------------------------------------------
    sec = head_w.section(plane_origin=[0, 0, 0], plane_normal=[0, 1, 0])
    prof = np.vstack([e for e in sec.discrete]) + hinge          # the midline profile
    front = prof[prof[:, 2] > hinge[2] + 0.02]
    xs = np.arange(front[:, 0].min() + 0.002, front[:, 0].max() - 0.002, 0.0005)
    top = np.array([front[np.abs(front[:, 0] - x) < 0.0015][:, 2].max(initial=-1.0) for x in xs])
    ok = top > -0.5
    xs, top = xs[ok], top[ok]
    # nasion: the deepest point of the profile between the brow and the nose tip
    tip_i = int(np.argmax(np.where((xs > 0.090) & (xs < 0.115), top, -1)))
    brow = (xs > 0.040) & (xs < xs[tip_i])
    nas_i = int(np.argmin(np.where(brow & (xs > xs[tip_i] - 0.025), top, 9)))
    nasion = np.array([xs[nas_i], 0.0, top[nas_i]])
    nose_tip = np.array([xs[tip_i], 0.0, top[tip_i]])
    # chin point (gnathion): along the chin's profile, where the outward normal is 45 deg between up and +x
    zs = top
    dz = np.gradient(zs, xs)
    chin_zone = (xs > xs[tip_i] + 0.030) & (xs < xs[tip_i] + 0.070)
    gn_i = int(np.argmin(np.where(chin_zone, np.abs(dz + 1.0), 9)))
    gnathion = np.array([xs[gn_i], 0.0, zs[gn_i]])
    # the nominal mask pose: origin midway between nasion and chin point, x along that line, z out of the face
    xm = (gnathion - nasion) / np.linalg.norm(gnathion - nasion)
    zm = np.array([0, 0, 1.0]) - xm * xm[2]
    zm /= np.linalg.norm(zm)
    ym = np.cross(zm, xm)
    R_nom = np.column_stack([xm, ym, zm])
    o_nom = (nasion + gnathion) / 2
    # the face's height (along the mask's z) under each cushion point: the anatomical cushion
    cp, tan = g.contact_points()
    origins = o_nom + (np.c_[cp, np.full(len(cp), 0.05)]) @ R_nom.T
    loc, ri, _ = trimesh.Trimesh(Hv, head.faces, process=False).ray.intersects_location(
        origins, np.tile(-zm, (len(cp), 1)), multiple_hits=False)
    face_h = np.full(len(cp), np.nan)
    face_h[ri] = (loc - o_nom) @ zm
    # the capsule's own first touch: lowered along the mask's -z, a capsule (radius SEG_R, cylinder SEG_LEN along the
    # rim's tangent) meets the face where it rises highest under its footprint (on a slope, its side touches first);
    # face_h is the height of the capsule's bottom at that touch
    Q = (Hv - o_nom) @ R_nom                                # head points in the nominal mask frame
    for i, (p, t) in enumerate(zip(cp, tan)):
        d = Q[:, :2] - p
        along = d @ t
        perp = d @ np.array([-t[1], t[0]])
        hl = g.SEG_LEN / 2
        dd = np.where(np.abs(along) <= hl, np.abs(perp), np.hypot(np.abs(along) - hl, perp))
        m = dd < g.SEG_R
        if m.any():
            face_h[i] = max(face_h[i], float((Q[m, 2] + np.sqrt(g.SEG_R ** 2 - dd[m] ** 2)).max() - g.SEG_R))
    # outline (outer edge) heights, for the eye check and the dome
    out = g.mask_outline(96)
    origins = o_nom + (np.c_[out, np.full(len(out), 0.05)]) @ R_nom.T
    loc, ri2, _ = trimesh.Trimesh(Hv, head.faces, process=False).ray.intersects_location(
        origins, np.tile(-zm, (len(out), 1)), multiple_hits=False)
    out_h = np.full(len(out), np.nan)
    out_h[ri2] = (loc - o_nom) @ zm
    # the nose tip's height over the mask plane
    nose_h = float((nose_tip - o_nom) @ zm)
    # the sternum at the nipple line (MakeHuman y -415)
    yn = -415.0
    q = torso.vertices[(np.abs(torso.vertices[:, 0]) < 4) & (np.abs(torso.vertices[:, 1] - yn) < 6)]
    stern = to_frame(q[np.argmax(q[:, 2])][None], z_low_torso)[0] @ Rp.T - np.array([0.0, 0.0, dz_rest])
    stern[1] = 0.0
    sel = (np.abs(Tv[:, 0] - stern[0]) < 0.040) & (np.abs(Tv[:, 1]) < 0.045) & (Tv[:, 2] > stern[2] - 0.030)
    fsel = sel[torso_w.faces].all(1) & (torso_w.face_normals[:, 2] > 0.3)
    patch = trimesh.Trimesh(Tv, torso_w.faces[fsel], process=True)
    patch.remove_unreferenced_vertices()
    patch_v = patch.vertices - stern + patch.vertex_normals * 0.0006

    np.savez(HERE / "build" / "manikin.npz",
             head_V=head_w.vertices, head_F=head_w.faces, col_V=col.vertices, col_F=col.faces,
             eye0_V=eyes[0][0], eye0_F=eyes[0][1], eye1_V=eyes[1][0], eye1_F=eyes[1][1],
             torso_V=torso_w.vertices, torso_F=torso_w.faces,
             hull_V=np.array([h[0] for h in hulls], dtype=object), hull_F=np.array([h[1] for h in hulls], dtype=object),
             patch_V=patch_v, patch_F=patch.faces, allow_pickle=True)
    marks = dict(hinge=hinge, r_occ=R_occ, nasion=nasion, gnathion=gnathion, nose_tip=nose_tip, eyes=eye_c + hinge,
                 mask_o=o_nom, mask_R=R_nom, face_h=face_h, out_h=out_h, nose_h=nose_h, sternum=stern,
                 torso_low=Tv[:, 2].min(), torso_bounds=torso_w.bounds, head_bounds=head_w.bounds + hinge,
                 back_pts=Tv[Tv[:, 2] < Tv[:, 2].min() + 0.003][::2], head_pts=head_w.vertices[::2])
    np.savez(HERE / "assets" / "landmarks.npz", **marks)
    for k, v in marks.items():
        if k in ("back_pts", "head_pts"):
            print(k, len(v))
            continue
        print(k, np.round(np.asarray(v, float) * (1000 if k not in ("mask_R",) else 1), 1).tolist())
    print("nasion-chin mm", round(float(np.linalg.norm(gnathion - nasion)) * 1000, 1))


if __name__ == "__main__":
    main()
