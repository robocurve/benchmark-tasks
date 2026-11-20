"""corded_pole: shock-corded tube sections (tent pole, folding cane, collapsible mast) for YAM Arena task files.

What it models
    Rigid tube sections at true size, each a free body built from convex pieces:
      * the tube wall as a 12-sided prism (``R_OUT`` across the corners), solid where nothing enters it;
      * a receiving end ("mouth"): a bore of ``BORE_DEPTH`` made of 12 convex wall pieces (inner flats at ``R_BORE``);
      * an insert ("ferrule") fixed in the other end: a 12-sided rod of ``R_FERRULE`` with a tapered tip, sticking
        out ``FERRULE_OUT``.
    The shock cord is NOT a body. It is an elastic2_kit line (massless Hooke line, recomputed before every 240 Hz
    physics step) from a knot in the first section's free end, through each insert's tip and each mouth, to a knot
    in the last section's free end. A mouth point is on the cord only while the insert's tip is outside that bore
    (or beside it): once the tip is in the bore the cord runs straight from the tip into the tube, so it pulls the
    insert along the bore toward its seat. With a joint open, the cord pulls the insert's tip toward the centre of
    the mouth (the real cord leaves the tip and enters the next tube there). The line has no contact, weight or
    friction and cannot be pinched; it is drawn only where it shows, at an open joint, as short black pieces that
    part as it stretches (inside a closed joint they are hidden in the tube).

Use (module level of a task file)::

    import corded_pole as cp
    A = cp.section("tube_a", pos, yaw, mass, ferrule=True, mouth=False)
    M = cp.section("tube_m", pos, yaw, mass, ferrule=True, mouth=True)
    B = cp.section("tube_b", pos, yaw, mass, ferrule=False, mouth=True)
    CORD = cp.cord("cord", [A, M, B], rest=0.787, stiffness=8.5, damping=0.2)
    OBJECTS = [A, M, B, *CORD.segments]
    cp.joints(states)   # per joint: depth of the insert tip past the mouth, side offset, gap at the seat

Frame of a section: origin at the middle of the tube on its axis, +y along the axis toward the end that carries
the insert (the assembled pole runs +y), z up; flats face +-x and +-z, so a section lies on a flat.

Tuned numbers (Nov 20)
    Cord 8.5 N/m, damping 0.2 N s/m, 28 g sections: k dt^2 / m = 0.005 and c dt / m = 0.03 (elastic2 limits
    0.3 and 0.45). A change of the cord's route (a joint opening or closing) restarts the damping term for one
    step so no spike is applied. Section bodies: 32/4 solver iterations, speculative CCD, 1 mm contact offset,
    damping (0.3, 1.0) 1/s.
"""
from __future__ import annotations

import math

import numpy as np
import yam_mesh as ymesh
import yam_parts as parts
import elastic2_kit as e2

N_SIDES = 12
PHASE = math.pi / N_SIDES  # corner angles 15 + 30 k deg in the section's x-z plane: flats face +-x and +-z
FLAT = math.cos(math.pi / N_SIDES)  # inradius / corner radius

# Six Moon Designs 45 in aluminium pole: 3/8 in (listed "3/8" - 10 mm") tube, 114 cm in 3 sections, folds to 43 cm.
R_OUT = 0.00475  # tube outer radius at the corners (9.5 mm across)
TUBE_L = 0.380  # each section's tube (114 / 3 cm)
FERRULE_OUT = 0.050  # insert showing past the tube end (43 - 38 cm folded)
R_BORE = 0.00405  # bore radius to the inner flats (8.1 mm bore: estimate, 0.7 mm wall)
BORE_DEPTH = 0.052  # modelled hollow this deep (the insert is 50 mm); deeper the tube is modelled solid
R_FERRULE = 0.00395  # insert radius at the corners (7.9 mm: estimate)
TIP_TAPER = (0.0035, 0.0029)  # the insert's tip tapers over its last 3.5 mm to a 2.9 mm radius (estimate)
GUNMETAL = (0.24, 0.25, 0.27)  # dark grey anodised aluminium (maker's photo)
ALU = (0.60, 0.62, 0.64)  # bare aluminium insert
CORD_BLACK = (0.04, 0.04, 0.05)
FRICTION = (0.35, 0.30)  # anodised aluminium (estimate)

HALF = TUBE_L / 2
TIP_LOCAL = (0.0, HALF + FERRULE_OUT, 0.0)  # insert tip, on the axis
END_LOCAL = (0.0, HALF, 0.0)  # the insert end's tube face (the seat)
MOUTH_LOCAL = (0.0, -HALF, 0.0)  # receiving end's face, on the axis
FREE_LOCAL = {"ferrule": (0.0, -HALF, 0.0), "mouth": (0.0, HALF, 0.0)}  # a free end (knot) for each kind of end
_TO_Y = np.array(((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0)))  # revolve's z axis -> the section's y


def _rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def solid(name, profile_ry, color, proxy=True):
    """Convex 12-sided solid of revolution about the section's y axis; profile [(r, y), ...] round the section."""
    m = ymesh.revolve(list(profile_ry), samples=N_SIDES, name=name, collision="convex")
    p = m.points @ _rotz(PHASE).T @ _TO_Y.T
    rmax = max(r for r, _ in profile_ry)
    ys = [y for _, y in profile_ry]
    px = (parts.box((0.0, (min(ys) + max(ys)) / 2, 0.0), (2 * rmax, max(ys) - min(ys), 2 * rmax)),) if proxy else ()
    return ymesh.Mesh(name, p, m.faces, collision="convex", color=color, proxy=px)


def bore(prefix, y0, y1, color):
    """12 convex wall pieces between the inner flats R_BORE and the outer corners R_OUT, from y0 to y1."""
    ri, ro = R_BORE / FLAT, R_OUT
    out = []
    for k in range(N_SIDES):
        a0, a1 = PHASE + 2 * math.pi * k / N_SIDES, PHASE + 2 * math.pi * (k + 1) / N_SIDES
        corners = [(ri * math.cos(a0), ri * math.sin(a0)), (ri * math.cos(a1), ri * math.sin(a1)),
                   (ro * math.cos(a1), ro * math.sin(a1)), (ro * math.cos(a0), ro * math.sin(a0))]
        m = ymesh.prism([(x, -z) for x, z in corners], y1 - y0, axis="y", name=f"{prefix}{k:02d}",
                        collision="convex", proxy=())
        out.append(ymesh.Mesh(m.name, m.points + np.array((0.0, (y0 + y1) / 2, 0.0)), m.faces, collision="convex",
                              color=color, proxy=()))
    return out


def section_meshes(ferrule, mouth):
    """Meshes of one section: tube (solid part), bore pieces at -y if ``mouth``, insert at +y if ``ferrule``."""
    y0 = -HALF + (BORE_DEPTH if mouth else 0.0)
    meshes = [solid("tube", [(0.0, y0), (R_OUT, y0), (R_OUT, HALF), (0.0, HALF)], GUNMETAL, proxy=False)]
    if mouth:
        meshes += bore("bore", -HALF, -HALF + BORE_DEPTH, GUNMETAL)
    if ferrule:
        ya, yb = HALF - 0.004, HALF + FERRULE_OUT  # 4 mm of the insert inside the tube's solid end (one body)
        meshes.append(solid("ferrule", [(0.0, ya), (R_FERRULE, ya), (R_FERRULE, yb - TIP_TAPER[0]),
                                        (TIP_TAPER[1], yb), (0.0, yb)], ALU, proxy=False))
    return meshes


def proxies(ferrule, mouth, seg=0.010):
    """Offline stand-ins for the checker (never colliders): the tube in 10 mm boxes, the bore as four walls round
    an 8.1 mm square hole, the insert as one box."""
    out = []
    y0 = -HALF + (BORE_DEPTH if mouth else 0.0)
    n = int(round((HALF - y0) / seg))
    for k in range(n):
        a, b = y0 + (HALF - y0) * k / n, y0 + (HALF - y0) * (k + 1) / n
        out.append(parts.box((0.0, (a + b) / 2, 0.0), (2 * R_OUT, b - a, 2 * R_OUT)))
    if mouth:
        t, ym = R_OUT - R_BORE, -HALF + BORE_DEPTH / 2
        for sx in (-1, 1):
            out.append(parts.box((sx * (R_BORE + t / 2), ym, 0.0), (t, BORE_DEPTH, 2 * R_OUT)))
        for sz in (-1, 1):
            out.append(parts.box((0.0, ym, sz * (R_BORE + t / 2)), (2 * R_BORE, BORE_DEPTH, t)))
    if ferrule:
        out.append(parts.box((0.0, HALF + FERRULE_OUT / 2, 0.0), (2 * R_FERRULE * FLAT, FERRULE_OUT, 2 * R_FERRULE * FLAT)))
    return tuple(out)


def section(name, pos, yaw, mass, ferrule, mouth, color=GUNMETAL):
    """One free tube section (an elastic2_kit EPart: the cord's kit hooks need it)."""
    proxy = proxies(ferrule, mouth)
    meshes = section_meshes(ferrule, mouth)
    meshes[0] = ymesh.Mesh(meshes[0].name, meshes[0].points, meshes[0].faces, collision="convex",
                           color=color, proxy=proxy)
    part = e2.EPart(name, pos, (), mass=mass, color=color, yaw=yaw, friction=FRICTION, contact_offset=0.001,
                    meshes=tuple(meshes), iterations=(32, 4), damping=(0.3, 1.0), max_depen=1.0, ccd=True)
    return part


class CordLine(e2.Line):
    """The shock cord: an elastic2_kit line whose mouth points drop out while an insert's tip is in its bore, drawn
    only at open joints."""

    def __init__(self, name, a, b, rest, stiffness, damping, guides, mouths, piece=0.004, per_joint=2,
                 thickness=0.0025, color=CORD_BLACK, spawn=None):
        self.mouths = dict(mouths)  # mouth guide index -> (tip guide index, receiving axis (local), bore radius)
        self._ids, self._prev = None, None
        self.piece, self.per_joint = float(piece), int(per_joint)
        super().__init__(name, a, b, rest=rest, stiffness=stiffness, damping=damping, guides=guides,
                         max_force=25.0, color=color, thickness=thickness, n_draw=1, floor=None)
        for seg in self.segments:  # the kit's pieces (sized for the whole line) are replaced by short joint pieces
            e2.SPECS.pop(seg.name, None)
            e2.OPTS.pop(seg.name, None)
        frames = {n: e2.spec_frame(e2.SPECS[n]) for n in self.bodies()}
        poses = [p for p, _ in self.draw_poses(self.measure_frames(frames)["points"])]
        if spawn is not None:  # spawn the drawn pieces out of the way: the kit moves them onto the cord at every
            poses = [np.asarray(spawn, float) + np.array((0.0, 0.012 * i, 0.0)) for i in range(len(poses))]  # reset
        self.segments = tuple(
            e2.EPart(f"{name}_{i:02d}", tuple(float(v) for v in p),
                     (parts.box((0, 0, 0), (self.piece, thickness, thickness)),), mass=0.001, color=color, ghost=True)
            for i, p in enumerate(poses))

    def open_joints(self, frames):
        """Per mouth guide: True while the insert's tip is outside (or beside) the bore."""
        gp = [self._pt(frames, g.body, g.local) for g in self.guides]
        out = {}
        for i, (j, axis, r) in self.mouths.items():
            R = frames[self.guides[i].body][1]
            u = R @ np.asarray(axis, float)
            d = gp[j] - gp[i]
            s = float(d @ u)
            lat = float(np.linalg.norm(d - s * u))
            out[i] = not (s > 0.0 and lat < r)
        return out

    def path(self, frames):
        pa, pb = self._pt(frames, *self.a), self._pt(frames, *self.b)
        is_open = self.open_joints(frames)
        act = tuple(i for i in range(len(self.guides)) if is_open.get(i, True))
        pts = [pa, *(self._pt(frames, self.guides[i].body, self.guides[i].local) for i in act), pb]
        names = [self.a[0], *(self.guides[i].body for i in act), self.b[0]]
        self._ids = (-1, *act, -2)
        return pts, names, act

    def step(self, frames, dt=e2.DT):
        act = self.path(frames)[2]
        if act != self._prev:  # the route changed: restart the damping term (no one-step spike)
            self._last = None
            self._prev = act
        return super().step(frames, dt)

    def draw_poses(self, pts):
        pts = [np.asarray(p, float) for p in pts]
        ids = list(self._ids) if self._ids is not None and len(self._ids) == len(pts) else None
        out = []
        for i, (j, _, _) in sorted(self.mouths.items()):
            if ids is not None and i in ids and j in ids:
                p, q = pts[ids.index(j)], pts[ids.index(i)]
            elif ids is not None and j in ids:  # closed: hide the pieces at the tip, inside the tube
                k = ids.index(j)
                p = pts[k]
                q = pts[k + 1] if k + 1 < len(pts) else pts[k - 1]
                q = p + 1e-4 * (q - p) / max(np.linalg.norm(q - p), 1e-9)
            else:
                p = q = pts[0]
            d = q - p
            R = e2._along(d if np.linalg.norm(d) > 1e-6 else np.array((0.0, 1.0, 0.0)))
            for k in range(self.per_joint):
                out.append((p + (k + 0.5) / self.per_joint * d, R))
        return out


def cord(name, sections, rest, stiffness, damping, spawn=None):
    """The cord through a chain of sections (first: insert only; middle: mouth and insert; last: mouth only)."""
    guides, mouths = [], {}
    for k, s in enumerate(sections):
        if k > 0:
            mouths[len(guides)] = (len(guides) - 1, (0.0, 1.0, 0.0), R_BORE)
            guides.append(e2.Guide(s.name, MOUTH_LOCAL))
        if k < len(sections) - 1:
            guides.append(e2.Guide(s.name, TIP_LOCAL))
    a = (sections[0], FREE_LOCAL["ferrule"])
    b = (sections[-1], FREE_LOCAL["mouth"])
    line = CordLine(name, a, b, rest, stiffness, damping, guides, mouths, spawn=spawn)
    e2.group("pole", sections)
    return line


def joint(ins_frame, rec_frame):
    """One joint from the inserting section's frame and the receiving one's (origin, R): tip depth past the mouth
    face along the receiving axis (m), the tip's side offset from that axis (m), the gap left at the seat
    (tube end face to mouth face, m), and the angle between the two axes (deg)."""
    oi, Ri = ins_frame
    orr, Rr = rec_frame
    u = Rr[:, 1]
    mouth = orr + Rr @ np.asarray(MOUTH_LOCAL)
    tip = oi + Ri @ np.asarray(TIP_LOCAL)
    end = oi + Ri @ np.asarray(END_LOCAL)
    d = tip - mouth
    s = float(d @ u)
    lat = float(np.linalg.norm(d - s * u))
    gap = float((mouth - end) @ u)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(Ri[:, 1] @ u)))))
    return {"depth": s, "side": lat, "gap": gap, "angle": ang}
