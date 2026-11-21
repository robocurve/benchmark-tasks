"""cloth_kit: true cloth (a PhysX surface deformable) for YAM Arena task files, with our own generated meshes.

What it models
    A thin fabric sheet, cap or tube whose every mesh node moves under the native PhysX deformable solver (gravity,
    contact with rigid parts and the robot pads, friction, self-contact). It is not a chain of rigid pieces. The
    kit declares the cloth, builds its mesh, gives the simulator an Arena DeformableObject and reads the node
    positions back; it never writes a cloth pose or force while a run is going. The only thing it switches at run time
    is the contact-gated pad attachment, and only if the task asks for it (:func:`use_grip_rule`).

Declare (module level of a task file, next to the yam_parts objects)::

    import cloth_kit as ck
    OBJECTS = [ ..., ck.Cloth.sheet("napkin", pos=(0.2, 0.1, TOP + 0.0425), width=0.235, length=0.280,
                                    thickness=0.0025, mass_kg=0.068),
               ck.Cloth.cap("beanie", pos=(0.1, 0.0, ZC), quat_xyzw=ck.axis_to_x(), radius=0.078, wall=0.06,
                            crown=0.08, thickness=0.0028, mass_kg=0.070, youngs=1.0e5) ]
    # score(objects) / demo code: objects["beanie"].nodes  ->  (n, 3) world positions, metres

Parameters of ``Cloth`` (all lengths metres, mass kg, stress Pa)
    kind           "sheet" | "cap" | "tube" (a cap is the same generator with a shorter wall; "tube" = closed at one end
                   with a round end, for a sock)
    pos            sheet: the middle of the sheet. cap/tube: the middle of the rim circle (the open end), part frame.
    quat_xyzw      orientation of the part frame about ``pos`` (default identity). Part frame: sheet lies in x-y with
                   z up; cap/tube axis is +z from the open rim (z = 0) to the closed end. ``yaw`` also applies, last.
    width, length  sheet size in x and y (before yaw).
    radius         cap/tube: relaxed rim radius (circumference 2 pi radius).
    wall           cap/tube: straight length above the rim before the round end starts.
    crown          cap/tube: height of the round end (an ellipse quarter, radius x crown; crown = radius is a
                   hemisphere). The apex is at z = wall + crown.
    edge           target spacing between nodes (smaller bends rounder and costs time: nodes ~ area / edge^2).
    thickness      collision/physical thickness of the fabric (PhysX surface thickness).
    mass_kg        total cloth mass; the kit derives ``density`` from it and the mesh area, so the number is the real one.
    youngs, poisson   membrane stiffness (Pa) and Poisson ratio.
    bend           surface bend stiffness; bend_damping, damping: bending and elastic damping.
    friction       static = dynamic friction of the cloth material (PhysX combines it with the pad's).
    rest_offset, contact_offset   PhysX collision rest offset (half the gap kept to other surfaces and to itself)
                   and contact offset (distance at which contacts are generated).
    iterations     solver position iterations of the deformable (24 as the accepted napkin used).
    self_collide   cloth collides with itself (needed for anything folded, rolled or doubled).
    keep_shape     take rest bend angles from the spawn shape ("restShapeDefault"), so a dome or a crease stays as
                   built instead of springing flat (needed for a cap; a flat sheet does not need it).
    color          sRGB.

Reading nodes: ``ClothState`` (what ``objects[name]`` is in score()): ``nodes`` (n, 3) world positions, ``node_vel``,
``pos`` (mean of the nodes), ``low``/``high`` (their bounds), ``lin_vel`` (the fastest node), ``origin`` (= pos).
``Cloth.rim_ring()`` gives the node indices of the open rim in order, ``rim_length(nodes)`` its perimeter,
``Cloth.edge_nodes(...)``/``nodes_near(...)`` pick nodes by their starting position.

Holding cloth in the pads (grip study, Nov 19)
    Friction alone does NOT hold cloth in this harness. PhysX surface cloth has dynamic friction only, and the finger
    drive (5000 N/m, target clamped at closed) presses a pinched 2.5 mm cloth with only ~1 N at 24 solver iterations
    (the pads crush it to a 0.3-0.5 mm gap). With PR 2194's own grip and cloth one napkin dropped mid-lift and the other
    crept 10-11 mm in a 3 s hold before it was lost; at 128 iterations (which apply to the whole scene, 2-5x slower) the
    pinch rose to 7 N but the edge was squeezed out of the pads 22-35 mm into the lift. 0 of 6 hand-trials passed.
    What works: the contact-gated attachment. One line at task module level::

        ck.use_grip_rule("napkin")                     # or several cloths; sides=("left", "right") by default

    Then every control step, for each hand, :class:`GripRule` attaches the cloth nodes between the pads to that gripper
    (native ``OmniPhysicsVtxXformAttachment``) only while BOTH pads are stopped on the cloth with a stated squeeze
    (:class:`GripGate` defaults: drive pinch >= 0.5 N, fingers <= 3 mm/s, gap 0.2 mm .. 3 x thickness + 2 mm, >= 3
    nodes between the faces and >= 1 within thickness/2 + 1.5 mm of each face, 2 checks in a row), and releases when
    the policy opens the hand (commanded opening > reached + 1 mm), lets the squeeze off (< 0.25 N), the pads part
    wider than the cloth, or a node drifts 20 mm (failed binding). Nothing else triggers it (no demo phase, no score),
    so a Dream policy gets the same rule. Proven on both hands (probe A run 2): attached at the close, 80 mm lift,
    3 s hold, 0.0-0.2 mm creep, released on opening and the napkins fell. :func:`edge_pinch_ways` is the proven pick of
    a cloth edge hanging over a table edge. Say it in the fidelity note, e.g. "Cloth is held by a contact-gated
    constraint: while both pads press the cloth it is pinned to the gripper, opening releases it (PhysX cloth has no
    static friction). No demo phase or score triggers it."
    Reading the hand: :func:`read_hands` (TCP pose, finger joints, commanded target; ``HandState.pinch`` = mean drive
    force, the fingers' weight cancels), :func:`pad_contacts` (cloth nodes in the pad window / between / touching).
    The scene has no contact sensors (``pad_forces`` returns an error).

Pitfalls (all found on the GPU)
    * Import order no longer matters: cloth_kit imports yam_parts / yam_joints / yam_mesh / yam_linkage (when present)
      before installing its import hook, puts its hook first again whenever a Cloth is declared, and delegates to the
      hooks behind it. (The first version answered alone and silently turned every Part into a plain box.)
    * Arena resets a deformable by moving its default nodes about the prim origin, so the kit sets the origin to the mesh
      centroid and bakes orientation into the nodes. Reset was checked over two episodes: identical start, no drift.
    * A cloth has no mass (``mass`` stays None), so yam_arena never jitters it and the rc probe's loose-object log ignores it:
      log your own numbers from ``objects[name].nodes`` in score().
    * ``demo()`` is built before the episode; the cloth has not settled then. Read the settled nodes inside a lazily evaluated
      waypoint (cache it per episode) and give an offline fallback.
    * "Deformable body view is not valid ... check PhysX logs" appears once per cloth at start; the cloth works.
    * Stiffness: with the accepted napkin's numbers (E 1e6, bend 500) a 6 cm overhang hangs straight down the board face and
      its last 2 cm lie on the table; bend 5e4 droops the tip 30 mm in 52 mm; bend 5e5 droops 7 mm (a stiff cantilever).
      A dome-shaped cap (2.8 mm, 70 g, E 1e5) collapses at bend 100, keeps its shape at 3e3 to 3e4 when it lies on the table;
      a narrow prop under its wall made it fold. The first deformable declared in a scene collapsed in two runs where later
      ones did not (not explained; the probe lists a tiny dummy sheet first).
    * Speed: 6.6 to 7.9 control steps per second (12 physics steps each, 240 Hz) with 1.4 to 2.4 thousand nodes, two arms and
      three cameras; 400 steps take about 50 s after the start.
    * The pad is 13 mm thick: a pad pair needs about 31 mm of clear height under a cloth it has to slip under. A level hand
      (fingers one above the other) is NOT reachable 4-12 cm above the station table, but is 13-15 cm up: put the cloth
      on a side table 20 cm high (as PR 2194 did; :func:`edge_pinch_ways`). Pinching straight down on cloth lying flat
      on a table does not work (the lower pad cannot get under it).
    * Two top-down hands pinching 15 cm apart collide at the wrist links (link4): lean each hand outward (0 deg: hit,
      10 deg: 7 mm apart, 15 deg: 34 mm). With a leaning hand the pad width axis must point back (-x) on both arms;
      the other sign puts the left wrist at a joint limit past x ~0.13.
    * Python closures in a demo loop bind late: ``lambda: g()`` inside ``for arm in ...`` calls the LAST arm's ``g``;
      bind it (``lambda g=g: g()``).
    * What the gripper can pinch (URDF hulls, TCP frame): behind the flat pads the two finger carriages fill the whole
      closing range (-48..+48 mm) from 65 to 81 mm behind the TCP (68 mm wide), and the housing (+-51 mm) from 70 mm
      on. So cloth can be pinched only where it ENDS (an edge, or a fold that leaves the gap) within ~65 mm of the pads
      towards the palm. A lying cap's side wall does not: its wall curves round to the cap's top inside that distance
      and the carriage meets it (probe B: the rim was pushed back 16-36 mm in 4 of 5 runs). A cap stiff enough to keep
      its opening on its side (bend 1.5e5; 2.8 mm, 70 g, R 78 mm, wall 9 cm) cannot be dented out of the way; at 1.5e4 it
      lies flat. Tipping the fingertips into the cap moves the carriage past the rim only if 65 sin(tip) - 34 cos(tip)
      exceeds the bite (tip >~ 32 deg for 5 mm); the wrist allows ~18-22 deg with the crown towards the robot.
    * Solver iterations of deformables act scene-wide (the largest wins): one 128-iteration cloth made every cloth in
      the scene behave so and slowed the run 2-5x.
    * Under a stretched cap (14-27 %) cloth nodes sank up to 6.6 mm into a rigid head at 24 iterations; check
      penetration in score() rather than trusting contact.
"""
from __future__ import annotations

import importlib.abc
import importlib.machinery
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

TAU = 2.0 * math.pi


# ------------------------------------------------------------------------------------------- orientation helpers
def quat_from_matrix(m) -> tuple[float, float, float, float]:
    m = np.asarray(m, float)
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return (float(x), float(y), float(z), float(w))


def matrix_from_quat(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def axis_to_direction(direction) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) that turns the part's +z axis into ``direction`` (cap/tube axis -> world direction)."""
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, d)
    c = float(z @ d)
    if np.linalg.norm(v) < 1e-9:
        return (0.0, 0.0, 0.0, 1.0) if c > 0 else (1.0, 0.0, 0.0, 0.0)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return quat_from_matrix(np.eye(3) + k + k @ k / (1.0 + c))


# -------------------------------------------------------------------------------------------------- mesh generators
def sheet_mesh(width: float, length: float, edge: float):
    """Flat rectangle in x-y centred on the origin. Returns (nodes (n,3), faces (m,3), cells (cu, cv)).
    Node (i, j) is row ``i * (cv + 1) + j``; i runs along x, j along y."""
    cu, cv = max(2, round(width / edge)), max(2, round(length / edge))
    u = np.linspace(-width / 2, width / 2, cu + 1)
    v = np.linspace(-length / 2, length / 2, cv + 1)
    x, y = np.meshgrid(u, v, indexing="ij")
    nodes = np.stack([x, y, np.zeros_like(x)], -1).reshape(-1, 3)
    idx = np.arange((cu + 1) * (cv + 1)).reshape(cu + 1, cv + 1)
    p, q, r, t = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
    faces = np.concatenate([np.stack([p, q, r], 1), np.stack([p, r, t], 1)])
    return nodes, faces, (cu, cv)


def hang_over_edge(nodes, side: str, length: float, radius: float, extent: float):
    """Bend the last ``length`` (arc length) of a flat sheet on ``side`` ('+x', '-x', '+y', '-y', part frame) down over an
    edge: a quarter circle of ``radius`` and then straight down, as a napkin hangs over a table edge. Lengths are kept
    (the bend is isometric), so the membrane starts unstrained. ``extent`` is the sheet's size along that axis."""
    out = np.array(nodes, float)
    if length <= 0:
        return out
    axis, sign = "xy".index(side[1]), 1.0 if side[0] == "+" else -1.0
    fold = sign * (extent / 2 - length)            # the edge line, in the part frame
    s = sign * (out[:, axis] - fold)               # arc length past the edge line (> 0 hangs)
    hang = s > 0
    quarter = math.pi * radius / 2
    on_arc = hang & (s <= quarter)
    phi = s[on_arc] / radius
    out[on_arc, axis] = fold + sign * radius * np.sin(phi)
    out[on_arc, 2] = -radius * (1 - np.cos(phi))
    down = hang & (s > quarter)
    out[down, axis] = fold + sign * radius
    out[down, 2] = -radius - (s[down] - quarter)
    return out


def capped_tube_profile(radius: float, wall: float, crown: float, edge: float):
    """(r, z) points from the rim (r = radius, z = 0) up the straight wall and round the elliptical end to the apex
    (r = 0, z = wall + crown), spaced about ``edge`` apart along the surface."""
    pts = []
    n_wall = max(1, math.ceil(wall / edge - 1e-9)) if wall > 1e-9 else 0
    for i in range(n_wall + 1):
        pts.append((radius, wall * i / n_wall if n_wall else 0.0))
    phi = np.linspace(0.0, math.pi / 2, 400)
    arc = np.stack([radius * np.cos(phi), wall + crown * np.sin(phi)], 1)
    seg = np.linalg.norm(np.diff(arc, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    n_arc = max(2, round(s[-1] / edge))
    for k in range(1, n_arc + 1):
        target = s[-1] * k / n_arc
        pts.append(tuple(np.array([np.interp(target, s, arc[:, 0]), np.interp(target, s, arc[:, 1])])))
    pts[-1] = (0.0, wall + crown)  # apex exactly on the axis
    return pts


def revolve_surface(profile, edge: float, min_ring: int = 6):
    """Open surface of revolution about +z from a (r, z) profile; ring k has about 2 pi r / edge nodes. Returns
    (nodes, faces, rings) where ``rings`` lists each ring's node indices in angle order (the apex is a ring of one)."""
    nodes, rings = [], []
    for r, z in profile:
        if r < 1e-9:
            rings.append([len(nodes)])
            nodes.append((0.0, 0.0, z))
            continue
        m = max(min_ring, round(TAU * r / edge))
        a = np.arange(m) * TAU / m
        rings.append(list(range(len(nodes), len(nodes) + m)))
        nodes.extend(zip(r * np.cos(a), r * np.sin(a), np.full(m, z)))
    faces = []
    for ra, rb in zip(rings[:-1], rings[1:]):
        faces.extend(_strip(ra, rb))
    return np.asarray(nodes, float), np.asarray(faces, int), rings


def _strip(ra, rb):
    """Triangles between ring ``ra`` (below) and ring ``rb`` (above), counter-clockwise seen from outside."""
    na, nb = len(ra), len(rb)
    out = []
    if nb == 1:  # fan to the apex
        for i in range(na):
            out.append((ra[i], ra[(i + 1) % na], rb[0]))
        return out
    i = j = 0
    while i < na or j < nb:
        if j >= nb or (i < na and (i + 1) * nb <= (j + 1) * na):
            out.append((ra[i % na], ra[(i + 1) % na], rb[j % nb]))
            i += 1
        else:
            out.append((ra[i % na], rb[(j + 1) % nb], rb[j % nb]))
            j += 1
    return out


def mesh_area(nodes, faces) -> float:
    t = nodes[faces]
    return float(0.5 * np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1).sum())


def perimeter(nodes, ring) -> float:
    p = nodes[list(ring)]
    return float(np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1).sum())


# ------------------------------------------------------------------------------------------------------------- spec
@dataclass(frozen=True)
class Cloth(Box):
    """A cloth declared in a task file's OBJECTS (see the module docstring for every field). Build with
    :meth:`sheet`, :meth:`cap` or :meth:`tube`."""

    size: tuple[float, float, float] = (0.0, 0.0, 0.0)  # filled from the shape; Box wants it
    kind: str = "sheet"
    quat_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    width: float = 0.235
    length: float = 0.280
    radius: float = 0.078
    wall: float = 0.06
    crown: float = 0.08
    edge: float = 0.007
    thickness: float = 0.0025
    mass_kg: float = 0.068
    youngs: float = 1.0e6
    poisson: float = 0.25
    bend: float = 500.0
    damping: float = 1.0
    bend_damping: float = 5.0
    friction: float = 1.5
    rest_offset: float = 0.00125
    contact_offset: float = 0.00525
    iterations: int = 24
    self_collide: bool = True
    keep_shape: bool = False
    hang_side: str = ""        # sheet only: this side's last hang_length hangs down over an edge at the start
    hang_length: float = 0.0
    hang_radius: float = 0.005

    @staticmethod
    def sheet(name, pos, width=0.235, length=0.280, yaw=0.0, quat_xyzw=(0, 0, 0, 1), color=(0.95, 0.95, 0.93), **kw):
        """A flat sheet; ``pos`` is the middle of the flat sheet (before any hang). ``hang_side='-x', hang_length=L``
        starts it with its last L (arc length) on that side bent down over an edge (radius ``hang_radius``): the edge
        line is at local x = -(width/2 - L), and the sheet's flat part lies at pos z."""
        return Cloth(name=name, pos=tuple(float(v) for v in pos), color=color, yaw=float(yaw), kind="sheet",
                     quat_xyzw=tuple(float(v) for v in quat_xyzw), width=width, length=length, **kw)

    @staticmethod
    def cap(name, pos, radius=0.078, wall=0.06, crown=0.08, yaw=0.0, quat_xyzw=(0, 0, 0, 1), color=(0.75, 0.2, 0.2), **kw):
        kw.setdefault("keep_shape", True)
        return Cloth(name=name, pos=tuple(float(v) for v in pos), color=color, yaw=float(yaw), kind="cap",
                     quat_xyzw=tuple(float(v) for v in quat_xyzw), radius=radius, wall=wall, crown=crown, **kw)

    @staticmethod
    def tube(name, pos, radius=0.04, wall=0.25, crown=0.04, yaw=0.0, quat_xyzw=(0, 0, 0, 1), color=(0.2, 0.3, 0.7), **kw):
        """A tube closed at one end by a round end (a sock: radius = leg radius, crown = the round end's height)."""
        kw.setdefault("keep_shape", True)
        return Cloth(name=name, pos=tuple(float(v) for v in pos), color=color, yaw=float(yaw), kind="tube",
                     quat_xyzw=tuple(float(v) for v in quat_xyzw), radius=radius, wall=wall, crown=crown, **kw)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.kind not in ("sheet", "cap", "tube"):
            raise ValueError(f"{self.name}: kind must be sheet, cap or tube")
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Cloth takes no mass; give mass_kg")
        if not 0.002 <= self.edge <= 0.03:
            raise ValueError(f"{self.name}: edge {self.edge} m is outside 2 to 30 mm")
        if not (self.thickness > 0 and self.mass_kg > 0 and self.youngs > 0):
            raise ValueError(f"{self.name}: thickness, mass_kg and youngs must be positive")
        if self.kind != "sheet" and not (self.radius > 0 and self.crown > 0 and self.wall >= 0):
            raise ValueError(f"{self.name}: radius and crown must be positive, wall not negative")
        if self.rest_offset < 0 or self.contact_offset <= self.rest_offset:
            raise ValueError(f"{self.name}: need 0 <= rest_offset < contact_offset")
        if self.hang_length and (self.kind != "sheet" or self.hang_side not in ("+x", "-x", "+y", "-y")):
            raise ValueError(f"{self.name}: hang_side must be one of +x -x +y -y on a sheet")
        install()  # re-assert the scene hook's place in front of any kit imported after this one (see install)

    # -- geometry --------------------------------------------------------------------------------------------
    def _local(self):
        if self.kind == "sheet":
            nodes, faces, cells = sheet_mesh(self.width, self.length, self.edge)
            if self.hang_length:
                extent = self.width if self.hang_side[1] == "x" else self.length
                nodes = hang_over_edge(nodes, self.hang_side, self.hang_length, self.hang_radius, extent)
            return nodes, faces, None, cells
        prof = capped_tube_profile(self.radius, self.wall, self.crown, self.edge)
        nodes, faces, rings = revolve_surface(prof, self.edge)
        return nodes, faces, rings, None

    def mesh(self, world: bool = True):
        """(nodes (n,3), faces (m,3)) as spawned. world=False: part frame (before quat_xyzw, yaw and pos)."""
        nodes, faces, _, _ = self._local()
        if world:
            nodes = self.to_world(nodes)
        return nodes, faces

    def to_world(self, local) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        yaw_m = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        rot = yaw_m @ matrix_from_quat(self.quat_xyzw)
        return np.asarray(local, float) @ rot.T + np.asarray(self.pos, float)

    @property
    def rotation(self) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ matrix_from_quat(self.quat_xyzw)

    @property
    def node_count(self) -> int:
        return len(self._local()[0])

    @property
    def area(self) -> float:
        nodes, faces, _, _ = self._local()
        return mesh_area(nodes, faces)

    @property
    def density(self) -> float:
        """kg/m^3 given to PhysX so that area x thickness x density = mass_kg."""
        return self.mass_kg / (self.area * self.thickness)

    def rim_ring(self) -> list[int]:
        """Node indices of the open rim (cap/tube) in angle order, as built."""
        if self.kind == "sheet":
            raise ValueError("a sheet has no rim ring; use edge_nodes")
        return list(self._local()[2][0])

    def rim_length(self, nodes) -> float:
        """Perimeter of the rim ring through the given (n,3) node positions."""
        return perimeter(np.asarray(nodes, float), self.rim_ring())

    def relaxed_rim_length(self) -> float:
        return self.rim_length(self._local()[0])

    def nodes_near(self, point_world, radius: float) -> np.ndarray:
        """Indices of the nodes that START within ``radius`` of a world point."""
        d = np.linalg.norm(self.mesh()[0] - np.asarray(point_world, float), axis=1)
        return np.flatnonzero(d <= radius)

    def edge_nodes(self, side: str) -> np.ndarray:
        """Sheet: node indices along one edge of the flat sheet, side in '+x', '-x', '+y', '-y' (part frame, before any
        hang)."""
        nodes = sheet_mesh(self.width, self.length, self.edge)[0]
        axis, sign = "xy".index(side[1]), 1.0 if side[0] == "+" else -1.0
        half = (self.width, self.length)[axis] / 2
        return np.flatnonzero(sign * nodes[:, axis] > half - 0.5 * self.edge)

    def state_at(self, nodes=None, node_vel=None) -> "ClothState":
        pts = self.mesh()[0] if nodes is None else np.asarray(nodes, float).reshape(-1, 3)
        vel = np.zeros_like(pts) if node_vel is None else np.asarray(node_vel, float).reshape(-1, 3)
        fastest = vel[np.argmax(np.linalg.norm(vel, axis=1))]
        centre = pts.mean(0)
        return ClothState(pos=centre, quat_xyzw=np.asarray(self.quat_xyzw, float), lin_vel=fastest, ang_vel=np.zeros(3),
                          low=pts.min(0), high=pts.max(0), nodes=pts, node_vel=vel, origin=centre)


@dataclass(frozen=True)
class ClothState(ObjectState):
    """``objects[name]`` for a Cloth: nodes (n,3) and node_vel (n,3) in the world frame (env 0)."""

    nodes: np.ndarray = None
    node_vel: np.ndarray = None
    origin: np.ndarray = None


# ---------------------------------------------------------------------------------------------- simulator side
_SPECS: dict[str, Cloth] = {}
_NAMES: set[str] = set()
_PATCH_TOKEN = object()
_SEEN: dict[str, int] = {}


def _cloth_object(spec: Cloth):
    """An Arena DeformableObject for ``spec`` (GPU host only). The prim origin is the mesh centroid and its
    orientation identity, because Arena resets a deformable by moving its default nodes about the prim origin."""
    import isaaclab.sim as sim_utils
    import trimesh
    from isaaclab.sim.spawners.meshes import meshes as mesh_spawners
    from isaaclab.sim.utils import clone, get_current_stage
    from isaaclab_arena.assets.deformable_object import DeformableObject
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
    from yam_arena.station import linear_rgb

    nodes, faces = spec.mesh(world=True)
    centre = nodes.mean(0)
    local_nodes = nodes - centre

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        mesh = trimesh.Trimesh(vertices=local_nodes, faces=faces, process=False)
        stage = get_current_stage()
        mesh_spawners._spawn_mesh_geom_from_mesh(prim_path, cfg, mesh, translation, orientation, None, stage=stage)
        if spec.keep_shape:
            from pxr import Sdf, Usd

            done = 0
            for prim in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
                if prim.GetAttribute("omniphysics:restShapePoints").IsValid():
                    attr = prim.GetAttribute("omniphysics:restBendAnglesDefault")
                    if not attr.IsValid():
                        attr = prim.CreateAttribute("omniphysics:restBendAnglesDefault", Sdf.ValueTypeNames.Token, False,
                                                    Sdf.VariabilityUniform)
                    attr.Set("restShapeDefault")
                    done += 1
            print(f"[cloth_kit] {spec.name}: rest bend angles taken from the spawn shape on {done} prim(s)", flush=True)
        return stage.GetPrimAtPath(prim_path)

    material = PhysxSurfaceDeformableBodyMaterialCfg(
        density=float(spec.density), surface_thickness=float(spec.thickness), poissons_ratio=float(spec.poisson),
        youngs_modulus=float(spec.youngs), surface_bend_stiffness=float(spec.bend), elasticity_damping=float(spec.damping),
        bend_damping=float(spec.bend_damping), static_friction=float(spec.friction), dynamic_friction=float(spec.friction))
    cfg = sim_utils.MeshRectangleCfg(
        func=spawn, size=(1.0, 1.0), edge_refinement=1.0, physics_material=material,
        deformable_props=PhysxDeformableBodyPropertiesCfg(
            solver_position_iteration_count=int(spec.iterations), linear_damping=0.0,
            self_collision=True if spec.self_collide else None),
        collision_props=[PhysxCollisionCfg(rest_offset=float(spec.rest_offset), contact_offset=float(spec.contact_offset))],
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.8))
    return DeformableObject(name=spec.name, spawner_cfg=cfg,
                            initial_pose=Pose(position_xyz=tuple(float(v) for v in centre),
                                              rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _patch_scene(scene) -> None:
    if getattr(scene, "_cloth_kit_patch_token", None) is _PATCH_TOKEN:
        return
    inner_object = getattr(scene.task_object, "_cloth_kit_inner", scene.task_object)
    inner_states = getattr(scene.object_states, "_cloth_kit_inner", scene.object_states)

    def task_object(spec):
        if isinstance(spec, Cloth):
            _SPECS[spec.name] = spec
            _NAMES.add(spec.name)
            return _cloth_object(spec)
        return inner_object(spec)

    def object_states(env, names):
        _ensure_grip_rule(env)
        cloth = [n for n in names if n in _NAMES]
        out = inner_states(env, [n for n in names if n not in _NAMES]) if len(cloth) != len(names) else {}
        if cloth:
            base = env.unwrapped
            world = base.arena_world
            origin = base.scene.env_origins[0]

            def arr(t):
                return t.detach().cpu().numpy().astype(np.float64)

            for n in cloth:
                nodes = arr(world.get_nodal_positions_w(n)[0] - origin)
                vel = arr(world.get_nodal_velocities_w(n)[0])
                if n not in _SEEN:
                    _SEEN[n] = len(nodes)
                    print(f"[cloth_kit] {n}: {len(nodes)} nodes in the simulator, {_SPECS[n].node_count} in the mesh",
                          flush=True)
                out[n] = _SPECS[n].state_at(nodes, vel)
        return out

    task_object._cloth_kit_inner = inner_object
    object_states._cloth_kit_inner = inner_states
    scene.task_object = task_object
    scene.object_states = object_states
    scene._cloth_kit_patch_token = _PATCH_TOKEN


class _Loader(importlib.abc.Loader):
    def __init__(self, loader):
        self.loader = loader

    def create_module(self, spec):
        create = getattr(self.loader, "create_module", None)
        return None if create is None else create(spec)

    def exec_module(self, module) -> None:
        self.loader.exec_module(module)
        _patch_scene(module)


class _Finder(importlib.abc.MetaPathFinder):
    """Sits first on sys.meta_path but does NOT answer on its own: it asks the finders behind it (the yam_parts / yam_mesh /
    yam_joints ones) for the spec, so their patches still apply, then wraps the loader it gets so that this module's patch is
    applied last (outermost). Returning a bare spec here would silently switch the other kits' patches off: every Part
    would then be built by the stock scene code as a plain box of its bounding size."""

    _cloth_kit_finder = True

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        for finder in sys.meta_path:
            if finder is self or getattr(finder, "_cloth_kit_finder", False):
                continue
            find = getattr(finder, "find_spec", None)
            spec = find(fullname, path, target) if find is not None else None
            if spec is not None and spec.loader is not None:
                spec.loader = _Loader(spec.loader)
                return spec
        return None


_SIBLING_KITS = ("yam_parts", "yam_joints", "yam_mesh", "yam_linkage")


def _import_sibling_kits() -> None:
    """Import the other kits that sit next to this file BEFORE cloth_kit's hook goes in, so their finders are already
    on sys.meta_path behind ours (a kit imported later would put its own finder in front, answer alone and skip ours).
    A kit that is not on the path is skipped; a later ``import yam_parts`` in the task file is then a no-op."""
    import importlib

    for kit in _SIBLING_KITS:
        if kit in sys.modules:
            continue
        try:
            importlib.import_module(kit)
        except ImportError:
            pass


def install() -> None:
    """Idempotent: patch yam_arena.scene (now, or when it is first imported) so Cloth objects become deformables.
    Our finder is (re)placed FIRST on sys.meta_path every time this runs (at import and again from every
    ``Cloth._check()``, i.e. when the task file is loaded), and it delegates to the finders behind it, so the order
    in which a task file imports cloth_kit and the yam_* kits no longer matters."""
    mine = [f for f in sys.meta_path if getattr(f, "_cloth_kit_finder", False)]
    for f in mine:
        sys.meta_path.remove(f)
    sys.meta_path.insert(0, mine[0] if mine else _Finder())
    scene = sys.modules.get("yam_arena.scene")
    if scene is not None:
        _patch_scene(scene)


_import_sibling_kits()
install()

# ======================================================================================= grip: cloth in the YAM pads
# Pad geometry (URDF collision hulls, gripper link frame = TCP frame: x the closing axis, tip_left's face at x = -|q7|,
# tip_right's at +|q8|; y across the pad; z the approach): the inner faces are flat from 20.7 mm behind the TCP to the
# tip 12.8 mm past it, 26 mm wide at the tip end (40 mm further back); each finger is 6.4 mm thick at the tip end.
K_FINGER = 5000.0            # N/m per finger (harness GRIPPER stiffness); squeeze = K x how far the target leads the finger
FINGER_TRAVEL = 0.04695      # each finger's travel from closed (0) to wide open (joint value -0.04695)
PAD_Z = (-0.0205, 0.0125)    # flat pad faces along the approach, from the TCP (m)
PAD_HALF_WIDTH = 0.013       # half width of the pad faces at the fingertip end (m)
TCP_IN_GRIPPER = (0.0, 0.0, 0.134)


def _np(t) -> np.ndarray:
    t = getattr(t, "torch", t)
    return t.detach().cpu().numpy().astype(np.float64) if hasattr(t, "detach") else np.asarray(t, np.float64)


@dataclass
class HandState:
    """One gripper as simulated (env 0, world frame relative to the env origin): ``tcp`` (3,), ``rot`` (3,3) with
    columns (closing axis, pad width axis, approach), finger joints ``q``/``qd`` (joint7 = tip_left, joint8 =
    tip_right; 0 closed, -0.04695 open) and the commanded finger target (m, <= 0; None if it cannot be read)."""

    side: str
    tcp: np.ndarray
    rot: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    target: float | None

    @property
    def opening(self) -> np.ndarray:
        """Each finger's distance from closed (m): its pad face is this far from the TCP along the closing axis."""
        return np.abs(self.q)

    @property
    def gap(self) -> float:
        return float(np.abs(self.q).sum())

    @property
    def squeeze(self) -> np.ndarray:
        """Each finger's drive push toward closed (N): K x (finger opening - commanded opening), clipped at 0. It
        includes the finger's own weight (~0.68 N when the fingers are stacked) and damping lag while moving; use
        :attr:`pinch` for the force on what is between the pads."""
        if self.target is None:
            return np.zeros(2)
        return np.clip(K_FINGER * (self.opening - abs(self.target)), 0.0, 50.0)

    @property
    def pinch(self) -> float:
        """Force (N) the pads press on what is between them, from the drives: the mean over both fingers of K x
        (reached opening - commanded opening). The fingers' weight cancels in the mean (it opens one finger and closes
        the other by the same amount); damping lag does not, so trust it only with the fingers still."""
        if self.target is None:
            return 0.0
        return float(K_FINGER * (self.opening - abs(self.target)).mean())

    def local(self, points) -> np.ndarray:
        return (np.asarray(points, float) - self.tcp) @ self.rot


def read_hands(env, sides=("left", "right")) -> dict:
    """{side: HandState} read from the simulator: the gripper link's simulated pose, the finger joints and the
    gripper action the policy sent this step."""
    from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN

    base = env.unwrapped
    origin = _np(base.scene.env_origins)[0]
    out = {}
    for side in sides:
        robot = base.scene[side + "_robot"]
        body = list(robot.body_names).index("gripper")
        pose = _np(robot.data.body_link_pose_w)[0, body]
        rot = matrix_from_quat(pose[3:7])
        tcp = pose[:3] - origin + rot @ np.asarray(TCP_IN_GRIPPER)
        ids = robot.find_joints(list(FINGER_JOINTS), preserve_order=True)[0]
        q, qd = _np(robot.data.joint_pos)[0, ids], _np(robot.data.joint_vel)[0, ids]
        target = None
        try:
            g = float(_np(base.action_manager.get_term(side + "_gripper").processed_actions).reshape(-1)[0])
            target = g * FINGER_OPEN
        except Exception:  # noqa: BLE001
            pass
        out[side] = HandState(side, tcp, rot, q, qd, target)
    return out


def pad_contacts(hand: HandState, nodes, thickness: float, tol: float = 0.0015) -> dict:
    """Which cloth nodes one hand's pads hold: ``window`` (in front of the flat pad faces), ``between`` (also between
    the two faces), ``touch_left``/``touch_right`` (between and within thickness/2 + tol of that pad's face: in
    contact with it), plus ``local`` (all nodes in the TCP frame) and ``dist`` (each pad face to its nearest node in
    the window, m)."""
    loc = hand.local(nodes)
    a, b = hand.opening
    win = (np.abs(loc[:, 1]) <= PAD_HALF_WIDTH) & (loc[:, 2] >= PAD_Z[0]) & (loc[:, 2] <= PAD_Z[1])
    between = win & (loc[:, 0] > -a - tol) & (loc[:, 0] < b + tol)
    reach = thickness / 2 + tol
    touch_l = between & (loc[:, 0] + a <= reach)
    touch_r = between & (b - loc[:, 0] <= reach)
    dist = (float(np.min(np.abs(loc[win, 0] + a))) if win.any() else None,
            float(np.min(np.abs(b - loc[win, 0]))) if win.any() else None)
    return dict(window=win, between=between, touch_left=touch_l, touch_right=touch_r, local=loc, dist=dist)


def pad_forces(env, side: str) -> dict:
    """Net contact force (N, world vector) on each finger link from the arm's contact sensor (``<side>_contacts``,
    every link). Whether PhysX reports cloth contacts here is what the grip study checked (see the module notes)."""
    try:
        sensor = env.unwrapped.scene[side + "_contacts"]
        f = _np(sensor.data.net_forces_w)[0]
        names = list(sensor.body_names)
        return {n: f[names.index(n)] for n in ("tip_left", "tip_right") if n in names}
    except Exception as e:  # noqa: BLE001
        return {"error": repr(e)}


class PadAttachment:
    """Native PhysX vertex attachment (``OmniPhysicsVtxXformAttachment``) between a cloth and one gripper link, created
    switched off. ``attach(indices, nodes_world)`` pins those cloth vertices to where they are now, in the gripper
    link's frame, and switches it on; ``release()`` switches it off. It is a hard hold (no slip): use it only through
    :class:`GripRule`, which ties it to measured pad contact and squeeze. GPU host only.

    ``base`` is ``env.unwrapped``; ``side`` is 'left' or 'right'."""

    def __init__(self, base, cloth_name: str, side: str):
        import omni.usd
        from pxr import Sdf, UsdPhysics
        from yam_arena.embodiment import SIDES

        self.base, self.side, self.cloth_name = base, side, cloth_name
        stage = omni.usd.get_context().get_stage()
        cloth = [str(p.GetPath()) for p in stage.Traverse()
                 if str(p.GetPath()).endswith("/" + cloth_name) and "OmniPhysicsDeformableBodyAPI" in p.GetAppliedSchemas()]
        if len(cloth) != 1:
            raise RuntimeError(f"expected one deformable body named {cloth_name}, found {cloth}")
        arm = SIDES[side]["prim"]
        link = [str(p.GetPath()) for p in stage.Traverse() if "/" + arm + "/" in str(p.GetPath())
                and str(p.GetPath()).endswith("/gripper") and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(link) != 1:
            raise RuntimeError(f"expected one gripper link for {side}, found {link}")
        self.robot = base.scene[side + "_robot"]
        self.body = list(self.robot.body_names).index("gripper")
        prim = stage.DefinePrim(cloth[0] + "_" + side + "_pad_attachment", "OmniPhysicsVtxXformAttachment")
        prim.CreateRelationship("omniphysics:src0").SetTargets([Sdf.Path(cloth[0])])
        prim.CreateRelationship("omniphysics:src1").SetTargets([Sdf.Path(link[0])])
        self._enabled = prim.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool)
        self._enabled.Set(False)
        self._indices = prim.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray)
        self._indices.Set([])
        self._local = prim.CreateAttribute("omniphysics:localPositionsSrc1", Sdf.ValueTypeNames.Point3fArray)
        self._local.Set([])
        self.on = False
        self.indices = np.empty(0, int)
        self.anchor = None

    def link_pose(self):
        pose = _np(self.robot.data.body_link_pose_w)[0, self.body]
        return pose[:3] - _np(self.base.scene.env_origins)[0], matrix_from_quat(pose[3:7])

    def attach(self, indices, nodes_world) -> None:
        from pxr import Vt

        pos, rot = self.link_pose()
        local = (np.asarray(nodes_world, float)[np.asarray(indices)] - pos) @ rot  # world -> link frame (rot^T applied)
        self._indices.Set([int(i) for i in indices])
        self._local.Set(Vt.Vec3fArray([tuple(float(v) for v in row) for row in local]))
        self._enabled.Set(True)
        self.on, self.indices, self.anchor = True, np.asarray(indices, int), local

    def residual(self, nodes_world) -> float:
        """How far (m) the attached nodes are from where the attachment holds them (the worst one)."""
        if not self.on or not len(self.indices):
            return 0.0
        pos, rot = self.link_pose()
        held = self.anchor @ rot.T + pos
        return float(np.linalg.norm(np.asarray(nodes_world, float)[self.indices] - held, axis=1).max())

    def release(self) -> None:
        if self.on:
            self._enabled.Set(False)
        self.on = False


@dataclass
class GripGate:
    """When a hand counts as holding cloth (the contact gate of :class:`GripRule`). Defaults are the values the grip
    study ran with (Nov 19)."""

    pinch_min: float = 0.5       # N, HandState.pinch (fully closed on 2.5 mm cloth: 0.9-1.3 N; closed on air: <= 0.35 N)
    pinch_hold: float = 0.25     # N, release if the pinch stays below this for `dwell` checks (squeeze let off)
    still: float = 0.003         # m/s: both fingers stopped (a free sweep runs at 5-11 mm/s)
    gap_min: float = 0.0002      # m: the pads stopped apart (closed on air: 0-0.14 mm; on 2.5 mm cloth: 0.3-0.6 mm)
    min_nodes: int = 3           # cloth nodes between the faces in the pad window
    touch_nodes: int = 1         # of them, at least this many within thickness/2 + 1.5 mm of EACH face
    layers_max: int = 3          # the reached gap may be at most layers_max x thickness + 2 mm (cloth, not something else)
    dwell: int = 2               # consecutive checks the above must hold
    patch: float = 0.012         # attach the between-nodes within this of their middle (m)
    open_margin: float = 0.001   # release once the commanded opening exceeds the reached one by this (m)
    drift_max: float = 0.020     # release if an attached node is this far from its anchor (failed binding)


class GripRule:
    """Contact-gated hold, the same rule for every policy (scripted demo or Dream): every control step, for each hand
    and each listed cloth, if BOTH pads are stopped on cloth (the fingers are still and short of closed, the gap fits
    the cloth's thickness, cloth nodes lie between the faces and touch each one) and the drives press it with at least
    ``gate.pinch_min`` (:attr:`HandState.pinch`), for ``gate.dwell`` steps, the cloth nodes between the pads are
    attached to that gripper where they are. Opening the hand, letting the squeeze off, the pads parting wider than
    the cloth, or a failed binding releases them. Nothing else moves the cloth; no score or demo phase triggers it.

    Why it exists (grip study, Nov 19): PhysX surface cloth has dynamic friction only and the finger drive can press
    a pinched cloth with only ~1 N (the target cannot pass closed; the cloth is crushed to 0.3-0.6 mm), so a friction
    pinch creeps 10+ mm in 3 s or drops the cloth.

    Installed by :func:`use_grip_rule` (task module level); runs from the scene's per-physics-step write hook, once
    per control step. ``events`` lists (t, side, cloth, 'attach'|'release', detail); ``status[(cloth, side)]`` is the
    last check (pinch, gap, still, nodes between, touching each face, attached)."""

    def __init__(self, env, cloths: dict, sides=("left", "right"), gate: GripGate | None = None, log: bool = True):
        self.base = env.unwrapped
        self.env = env
        self.cloths = dict(cloths)          # name -> Cloth spec
        self.sides = tuple(sides)
        self.gate = gate or GripGate()
        self.log = log
        self.active = True
        self.events = []
        self.status = {}
        self.t = 0.0
        self._att = {}
        self._count = {}
        self._slack = {}
        self._sub = 0
        self._every = int(getattr(getattr(self.base, "cfg", None), "decimation", 12) or 12)
        self._dt = float(getattr(self.base, "physics_dt", 1 / 240))
        self._origin = _np(self.base.scene.env_origins)[0]
        scene = self.base.scene
        write, reset = scene.write_data_to_sim, scene.reset

        def write_hook(*a, **k):
            out = write(*a, **k)
            self._sub += 1
            self.t += self._dt
            if self._sub % self._every == 1 or self._every == 1:
                try:
                    self.tick()
                except Exception as e:  # noqa: BLE001
                    if not getattr(self, "_warned", False):
                        print(f"[cloth_kit] grip rule error: {e!r}", flush=True)
                        self._warned = True
            return out

        def reset_hook(*a, **k):
            self.release_all("reset")
            self.t, self._sub = 0.0, 0
            return reset(*a, **k)

        scene.write_data_to_sim, scene.reset = write_hook, reset_hook

    def _nodes(self, name):
        return _np(self.base.arena_world.get_nodal_positions_w(name))[0] - self._origin

    def attachment(self, name, side) -> PadAttachment:
        key = (name, side)
        if key not in self._att:
            self._att[key] = PadAttachment(self.base, name, side)
        return self._att[key]

    def release_all(self, why: str) -> None:
        for (name, side), att in self._att.items():
            if att.on:
                att.release()
                self._event(side, name, "release", why)
        self._count.clear()
        self._slack.clear()

    def _event(self, side, name, what, detail) -> None:
        self.events.append((round(self.t, 3), side, name, what, detail))
        if self.log:
            print(f"[cloth_kit] GRIP {what} {side} {name} t={self.t:.2f} {detail}", flush=True)

    def tick(self) -> None:
        if not self.active:
            self.release_all("rule off")
            return
        hands = read_hands(self.env, self.sides)
        g = self.gate
        for name, spec in self.cloths.items():
            nodes = self._nodes(name)
            for side, hand in hands.items():
                key = (name, side)
                att = self._att.get(key)
                pinch, still = hand.pinch, float(np.abs(hand.qd).max())
                if att is not None and att.on:
                    opened = hand.target is not None and abs(hand.target) > hand.opening.max() + g.open_margin
                    too_wide = hand.gap > g.layers_max * spec.thickness + 0.004
                    drift = att.residual(nodes)
                    self._slack[key] = self._slack.get(key, 0) + 1 if pinch < g.pinch_hold else 0
                    slack = self._slack[key] >= g.dwell
                    self.status[key] = dict(pinch=pinch, gap=hand.gap, qd=still, attached=True, drift=drift)
                    if opened or too_wide or slack or drift > g.drift_max:
                        att.release()
                        why = ("opened" if opened else "gap %.1f mm" % (hand.gap * 1e3) if too_wide
                               else "pinch %.2f N" % pinch if slack else "drift %.1f mm" % (drift * 1e3))
                        self._event(side, name, "release", why)
                        self._slack[key] = 0
                    continue
                pc = pad_contacts(hand, nodes, spec.thickness)
                n_btw, n_l, n_r = int(pc["between"].sum()), int(pc["touch_left"].sum()), int(pc["touch_right"].sum())
                ok = (still <= g.still and pinch >= g.pinch_min
                      and g.gap_min <= hand.gap <= g.layers_max * spec.thickness + 0.002
                      and n_btw >= g.min_nodes and n_l >= g.touch_nodes and n_r >= g.touch_nodes)
                self.status[key] = dict(pinch=pinch, gap=hand.gap, qd=still, between=n_btw, touch=(n_l, n_r), ok=ok,
                                        attached=False)
                self._count[key] = self._count.get(key, 0) + 1 if ok else 0
                if self._count[key] >= g.dwell:
                    ids = np.flatnonzero(pc["between"])
                    mid = nodes[ids].mean(0)
                    ids = ids[np.linalg.norm(nodes[ids] - mid, axis=1) <= g.patch]
                    if len(ids) >= g.min_nodes:
                        self.attachment(name, side).attach(ids, nodes)
                        self._event(side, name, "attach", f"{len(ids)} nodes, gap {hand.gap * 1e3:.2f} mm, pinch "
                                    f"{pinch:.2f} N, fingers {still * 1e3:.1f} mm/s, touching {n_l}/{n_r}")
                        self._count[key] = 0


_GRIP = {"spec": None, "rule": None}


def use_grip_rule(*cloth_names, sides=("left", "right"), gate: GripGate | None = None) -> None:
    """Task module level: hold these cloths by the contact-gated attachment (:class:`GripRule`) instead of by pad
    friction alone. The rule is built the first time the harness reads the cloth's state (the first control step) and
    then runs every control step on its own. ``grip_rule()`` returns it (None offline)."""
    _GRIP["spec"] = (tuple(cloth_names), tuple(sides), gate)


def grip_rule():
    return _GRIP["rule"]


def _ensure_grip_rule(env) -> None:
    if _GRIP["spec"] is None or _GRIP["rule"] is not None:
        return
    names, sides, gate = _GRIP["spec"]
    _GRIP["rule"] = GripRule(env, {n: _SPECS[n] for n in names}, sides, gate)
    print(f"[cloth_kit] grip rule on for {names}, hands {sides}", flush=True)


# ------------------------------------------------------------------------------- waypoints with a full TCP rotation
def grip_matrix(approach, close) -> np.ndarray:
    """TCP rotation with the approach axis ``approach`` and the closing axis ``close`` (made perpendicular): columns
    (closing axis = tip_right's side, approach x closing, approach). A straight-down grip closing along x is
    diag(1, -1, -1)."""
    a = np.asarray(approach, float)
    a = a / np.linalg.norm(a)
    j = np.asarray(close, float)
    j = j - (j @ a) * a
    j = j / np.linalg.norm(j)
    return np.column_stack([j, np.cross(a, j), a])


class _Rot(float):
    """A waypoint ``yaw`` that carries a full rotation matrix to yam_arena.demo.grip_rotation (patched below)."""

    def __new__(cls, matrix):
        value = super().__new__(cls, 0.0)
        value.grip_matrix = np.array(matrix, float)
        return value


def install_grip_rotation() -> None:
    """Let a waypoint's ``yaw`` carry a full TCP rotation (see :class:`Way`); idempotent."""
    from yam_arena import demo as arena_demo

    original = arena_demo.grip_rotation
    if getattr(original, "_grip_matrix", False):
        return

    def grip_rotation(yaw=0.0):
        m = getattr(yaw, "grip_matrix", None)
        return m.copy() if m is not None else original(float(yaw))

    grip_rotation._grip_matrix = True
    grip_rotation.__wrapped__ = original
    arena_demo.grip_rotation = grip_rotation


def _waypoint_base():
    from yam_arena.demo import Waypoint

    return Waypoint


class Way(_waypoint_base()):
    """A scripted-demo waypoint with a full TCP rotation ``rot`` (3x3) and a position that may be a callable,
    evaluated lazily once per episode (``episode`` is a callable returning the current episode number, so a waypoint
    can read the settled cloth). ``finger`` is the finger joint target (0 closed .. -0.04695 open)."""

    def __init__(self, duration, pos, finger, label="", rot=None, episode=None):
        self.duration, self.finger, self.label = float(duration), float(finger), label
        self._pos, self._rot, self._episode, self._cache = pos, rot, episode, (None, None)
        install_grip_rotation()

    @property
    def pos(self):
        if not callable(self._pos):
            return tuple(float(v) for v in self._pos)
        ep = self._episode() if self._episode else None
        if self._cache[0] != ep or self._cache[1] is None:
            self._cache = (ep, tuple(float(v) for v in self._pos()))
        return self._cache[1]

    @property
    def yaw(self):
        return 0.0 if self._rot is None else _Rot(self._rot)


# --------------------------------------------------------------------- the proven pick: a hanging edge, level hand
EDGE_PINCH_PHASES = (("in front", 2.5), ("down", 1.3), ("stack", 1.4), ("under edge", 1.2), ("edge on pad", 0.6),
                     ("back", 0.7), ("close", 1.4), ("settle", 0.3), ("lift", 2.0))


def edge_pinch_ways(side: str, edge_x: float, y: float, top_z: float, flap: float, lift: float = 0.08,
                    durations: dict | None = None, episode=None) -> list:
    """Waypoints for one arm that pick a cloth by its edge hanging over a table's near edge (PR 2194's grip, re-derived;
    grip study probe A, Nov 19): the hand comes level, pointing +x, fingers side by side, turns the fingers one above
    the other (wide open), puts the lower fingertip 8 mm under the table edge and its pad 7 mm under the hanging cloth,
    rises 10 mm so the edge rests on the lower pad, draws back 16 mm and up 10 mm (upper finger clear of the table),
    closes over 1.4 s while descending 47 mm (the lower pad stays put, the upper pad comes down onto the cloth),
    settles 0.3 s and lifts ``lift`` straight up. Returns the list of :class:`Way` ending at the lifted pose (append
    your own after it).

    side: 'left' | 'right'; edge_x: the table's near edge (x); y: where along the edge; top_z: the table top (world
    z); flap: how far the hanging edge reaches below the top (m, 0.061 in 2194 and probe A; build the cloth with
    ``Cloth.sheet(..., hang_side="-x", hang_length=...)``). Checked: side table top 0.20 m above the station table,
    edge x 0.22, y +-0.125..+-0.185 (IK <= 0.3 mm; joint 5 sits at its -90 deg limit while the fingers are stacked).
    Friction alone does not hold the cloth in our harness (see :class:`GripRule`): use with :func:`use_grip_rule`."""
    s = 1.0 if side == "left" else -1.0
    dur = dict(EDGE_PINCH_PHASES, **(durations or {}))
    travel = FINGER_TRAVEL
    low = flap + 0.007                     # the lower pad starts 7 mm under the hanging edge
    z0 = top_z - low + travel              # TCP height with the lower (tip_left / tip_right) face there, wide open
    x_in, back = edge_x - 0.005, edge_x - 0.005 - 0.016
    z_close = top_z - low + 0.020
    side_by_side = grip_matrix((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
    stack = grip_matrix((1.0, 0.0, 0.0), (0.0, 0.0, s))
    pts = [("in front", (edge_x - 0.045, y, top_z + 0.04), -travel, side_by_side),
           ("down", (edge_x - 0.050, y, z0), -travel, side_by_side),
           ("stack", (edge_x - 0.050, y, z0), -travel, stack),
           ("under edge", (x_in, y, z0), -travel, stack),
           ("edge on pad", (x_in, y, z0 + 0.010), -travel, stack),
           ("back", (back, y, z0 + 0.020), -travel, stack),
           ("close", (back, y, z_close), 0.0, stack),
           ("settle", (back, y, z_close), 0.0, stack),
           ("lift", (back, y, z_close + lift), 0.0, stack)]
    return [Way(dur[n], p, f, n, rot=r, episode=episode) for n, p, f, r in pts]


__all__ = ["Cloth", "ClothState", "EDGE_PINCH_PHASES", "GripGate", "GripRule", "HandState", "PadAttachment", "Way",
           "axis_to_direction", "capped_tube_profile", "edge_pinch_ways", "grip_matrix", "grip_rule", "hang_over_edge", "install", "install_grip_rotation",
           "matrix_from_quat", "mesh_area", "pad_contacts", "pad_forces", "perimeter", "quat_from_matrix", "read_hands",
           "revolve_surface", "sheet_mesh", "use_grip_rule"]
