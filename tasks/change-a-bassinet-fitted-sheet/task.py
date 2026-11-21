"""Strip a used fitted sheet into a laundry zone and fit a clean one."""
from itertools import product
from math import pi

import os

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from geom import Geometry

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MAX_STEPS = 1400
INSTRUCTION = (
    "Change the bassinet sheet. Take the used coral sheet off the mattress and put it in the "
    "laundry zone. Fit the clean blue sheet over all four corners, then release it."
)
CASES = ("nominal", "miss-one-pocket", "leave-used-sheet")
CASE = os.environ.get("ROBOCURVE_FIXTURE_CASE", "nominal")

X0, X1, Y = .206, .481, .330
ZTOP, ZBOTTOM = TOP + .059, TOP + .015
# The four pocket hems run around the underside of the mattress.
SIGNS = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
CORNERS = np.array([[X0, -Y], [X1, -Y], [X1, Y], [X0, Y]])
SEAT = np.column_stack((CORNERS + SIGNS * .008, np.full(4, ZBOTTOM + .004)))
SHELF_X = .518
SHELF_SIZE = (.045, .526, .020)
FOLD_START = SHELF_X - .009
CLEAN_FOLD = np.array([[FOLD_START+.015, -.186, TOP + .022], [FOLD_START, -.235, TOP + .029],
                       [FOLD_START, .235, TOP + .029], [FOLD_START+.015, .186, TOP + .022]])
HOVER = TOP + .109
PAD = .013
TAB_OFFSETS = np.column_stack((np.zeros(4), SIGNS[:, 1] * .037, np.zeros(4)))
GRID_X, GRID_Y = 7, 13
CORNER_I = [0, (GRID_X - 1) * GRID_Y, GRID_X * GRID_Y - 1, GRID_Y - 1]
LAUNDRY_END = .196
POCKET_ESCAPE_M = .029
POCKET_CONSTRAINT_GAIN = .43
PHOTO_SAMPLE_M = .005
PHOTO_FIT_MIN = .95
PHOTO_LAUNDRY_MIN = .90

OBJECTS = [
    Box("bassinet_foam_mattress", ((X0 + X1) / 2, 0, (ZTOP + ZBOTTOM) / 2),
        size=(X1-X0, 2*Y, ZTOP-ZBOTTOM), color=(.81, .82, .82)),
    Box("clean_sheet_shelf", (SHELF_X, 0, TOP + .010), size=SHELF_SIZE,
        color=(.17, .31, .32)),
    Box("laundry_zone_pad", (.090, 0, TOP + .003), size=(.207, .600, .006),
        color=(.31, .34, .27)),
]


def configure_demo(case):
    global CASE
    CASE = case


def demo():
    c, o = FINGER_CLOSED, FINGER_OPEN

    def w(duration, point, grip, name, yaw=0.):
        return Waypoint(duration, tuple(point), grip, name, yaw)

    def approach(path, point, title, yaw=0., hover=HOVER):
        opening = .55*o if yaw else o
        path.extend([
            w(1.35, (*point[:2], hover), opening, "approach " + title, yaw),
            w(1.20, point + (0, 0, PAD), opening, "lower to " + title, yaw),
            w(.72, point + (0, 0, PAD), c, "pinch " + title, yaw),
        ])

    def unhook(path, index):
        pocket = SEAT[index] + TAB_OFFSETS[index]
        dx, dy = SIGNS[index] * .030
        outside = pocket + (dx, dy, PAD)
        yaw = float(SIGNS[index,1] * pi / 2) if SIGNS[index,0] > 0 else 0.
        approach(path, pocket, "used pull tab", yaw)
        path.extend([
            w(.88, outside, c, "stretch elastic hem out", yaw),
            w(.87, (*outside[:2], HOVER), c, "lift used hem above mattress", yaw),
        ])
        return outside

    def install(path, index, skip=False):
        pocket = SEAT[index] + TAB_OFFSETS[index]
        dx, dy = SIGNS[index] * .025
        pickup = CLEAN_FOLD[index] + TAB_OFFSETS[index]
        yaw = float(SIGNS[index,1] * pi / 2) if SIGNS[index,0] > 0 else 0.
        shelf_hover = TOP + .074 if index in (1,2) else TOP + .080
        approach(path, pickup, "clean pull tab", hover=shelf_hover)
        path.append(w(.92, (*pickup[:2], shelf_hover), c, "lift clean pocket"))
        if index in (0,3):
            path.append(w(1.22, (.360, SIGNS[index,1]*.270, HOVER+.016), c,
                          "draw clean fabric above the mattress"))
        path.extend([
            w(1.55, (pocket[0]+dx, pocket[1]+dy, HOVER), c, "stretch clean pocket over corner", yaw),
            w(.94, (pocket[0]+dx, pocket[1]+dy, pocket[2]+PAD), c, "pull hem below mattress top", yaw),
            w(.77, pocket+(0, 0, PAD), c, "draw hem inward under corner", yaw) if not skip else
            w(.77, (pocket[0]+dx, pocket[1]+dy, HOVER), c, "leave pocket loose", yaw),
            w(.69, pocket+(0, 0, PAD), o, "let elastic pocket hold", yaw) if not skip else
            w(.69, (pocket[0]+dx, pocket[1]+dy, HOVER), o, "release loose pocket", yaw),
            w(.87, (pocket[0]+dx, pocket[1]+dy, HOVER), o, "clear seated pocket", yaw),
        ])

    result = {}
    for side, near, far, sign in (("right", 0, 1, -1.), ("left", 3, 2, 1.)):
        path = []
        park = (.18, sign * .274, HOVER)
        if CASE != "leave-used-sheet":
            p = unhook(path, near)
            path += [
                w(.90, (.295, p[1] - sign * .055, HOVER), c, "fold released near pocket onto sheet"),
                w(.72, (.295, p[1] - sign * .055, HOVER), o, "release used near pocket"),
            ]
            unhook(path, far)
            path += [
                w(1.80, (0.044, sign*.289, HOVER), c, "pull soiled sheet entirely off mattress"),
                w(1.02, (0.043, sign*.272, TOP + .046), c, "lower used sheet onto laundry pad"),
                w(.90, (0.043, sign*.272, TOP + .046), o, "release used sheet into laundry zone"),
                w(.90, (0.043, sign*.272, HOVER), o, "clear laundry zone"),
            ]
            trailing = np.array([.202, sign*.258, TOP+.021])
            approach(path, trailing, "trailing used corners")
            path += [
                w(.72, (*trailing[:2], HOVER), c, "lift trailing fabric"),
                w(1.45, (0.044, sign*.245, HOVER), c, "clear all used fabric from mattress"),
                w(.92, (0.043, sign*.227, TOP+.030), c, "lay the trailing fabric on the laundry pad"),
                w(.75, (0.043, sign*.227, TOP+.030), o, "release used sheet completely"),
                w(.92, (0.043, sign*.227, HOVER), o, "clear used sheet"),
            ]
            drape = np.array([.216, sign*.114, ZTOP+.003])
            approach(path, drape, "remaining used fabric at near edge")
            path += [
                w(.70, (*drape[:2], HOVER), c, "lift remaining used fabric"),
                w(1.35, (0.025, sign*.130, HOVER), c, "clear mattress edge"),
                w(.92, (0.025, sign*.130, TOP+.031), c, "lay remaining fabric on pad"),
                w(.70, (0.025, sign*.130, TOP+.031), o, "release remaining fabric"),
                w(.80, (0.025, sign*.130, HOVER), o, "clear laundry pad"),
            ]
        install(path, far)
        install(path, near, skip=CASE == "miss-one-pocket" and side == "right")
        path += [w(1.3, park, o, "release and clear mattress"),
                 w(1.1, park, o, "settle sheet")]
        result[side] = path
    return result


def grid_faces(nx, ny):
    return [(x*ny+y, (x+1)*ny+y, (x+1)*ny+y+1, x*ny+y+1)
            for x in range(nx-1) for y in range(ny-1)]


def make_positions():
    return np.array([(x, y, ZTOP+.003) for x in np.linspace(X0-.012, X1+.012, GRID_X)
                     for y in np.linspace(-Y-.012, Y+.012, GRID_Y)])


def make_links():
    links = []
    for x, y in product(range(GRID_X), range(GRID_Y)):
        i = x*GRID_Y+y
        for dx, dy, stiff in ((1, 0, .90), (0, 1, .90), (1, 1, .38), (1, -1, .38),
                              (2, 0, .14), (0, 2, .14)):
            xx, yy = x+dx, y+dy
            if 0 <= xx < GRID_X and 0 <= yy < GRID_Y:
                links.append((i, xx*GRID_Y+yy, stiff))
    return zip(*links)


A, B, STIFFNESS = (np.array(values, dtype=dtype)
                   for values, dtype in zip(make_links(), (int, int, float)))
REST = np.linalg.norm(make_positions()[B] - make_positions()[A], axis=1)
# The sewn skirt contributes material at the outside rows, while nominal
# panel dimensions and all interior rest lengths stay fixed.
_relaxed = make_positions()
_border = np.array([(i in (0, GRID_X-1), j in (0, GRID_Y-1))
                    for i,j in product(range(GRID_X),range(GRID_Y))])
_relaxed[:, 0] += _border[:, 0] * np.sign(_relaxed[:, 0] - (X0+X1)/2) * .021
_relaxed[:, 1] += _border[:, 1] * np.sign(_relaxed[:, 1]) * .018
CLEAN_REST = np.linalg.norm(_relaxed[B] - _relaxed[A], axis=1)

DEGREE = np.bincount(np.concatenate((A, B)), minlength=GRID_X*GRID_Y)[:, None]


def project_box(points, before, lower, upper, margin):
    """Keep samples on the side of the solid they approached from."""
    low, high = np.asarray(lower) - margin, np.asarray(upper) + margin
    delta = points - before
    active = np.abs(delta) > 1e-10
    safe_delta = np.where(active, delta, 1.)
    near = np.where(delta > 0, low, high)
    far = np.where(delta > 0, high, low)
    enter = np.where(active, (near - before) / safe_delta, -np.inf)
    exit_ = np.where(active, (far - before) / safe_delta, np.inf)
    entry = enter.max(axis=1)
    exit_ = exit_.min(axis=1)
    parallel_inside = np.all(active | ((before > low) & (before < high)), axis=1)
    hit = (entry >= -1e-8) & (entry <= 1.) & (entry < exit_) & parallel_inside
    rows = np.flatnonzero(hit)
    axes = np.argmax(enter[rows], axis=1)
    points[rows, axes] = near[rows, axes]

    # Stretch constraints can move a resting point off a corner within one
    # iteration. Repair overlap even when its previous position is inside.
    inside = np.all((points > low) & (points < high), axis=1)
    rows = np.flatnonzero(inside)
    distances = np.column_stack((points[rows] - low, high - points[rows]))
    face = np.argmin(distances, axis=1)
    axis = face % 3
    points[rows, axis] = np.where(face < 3, low[axis], high[axis])


SOLIDS = (
    ((X0, -Y, ZBOTTOM), (X1, Y, ZTOP)),
    ((X0-.012, -Y-.010, TOP), (X1+.012, Y+.010, TOP+.014)),
    ((SHELF_X-SHELF_SIZE[0]/2, -.263, TOP),
     (SHELF_X+SHELF_SIZE[0]/2, .263, TOP+SHELF_SIZE[2])),
    ((-.0135, -.300, TOP), (.1935, .300, TOP+.006)),
)
FOLDED_CLEAN = ((FOLD_START-.004, -.244, TOP+.019), (FOLD_START+.022, .244, TOP+.045))
THICKNESS = .002
FACES = np.array(grid_faces(GRID_X, GRID_Y))
TRIANGLES = FACES[:, (0, 1, 2, 0, 2, 3)].reshape(-1, 3)
GRID_COORDINATES = np.column_stack(np.unravel_index(np.arange(GRID_X*GRID_Y), (GRID_X, GRID_Y)))
TOPOLOGICAL_GAPS = np.max(np.abs(GRID_COORDINATES[:, None, None, :]
                                 - GRID_COORDINATES[TRIANGLES][None, :, :, :]), axis=3)
SELF_CONTACT = np.all(TOPOLOGICAL_GAPS > 1, axis=2)
# Edge midpoints and the quad centre participate in rigid contact.
CONTACT_WEIGHTS = np.array([[.5,.5,0,0], [0,.5,.5,0], [0,0,.5,.5],
                            [.5,0,0,.5], [.25,.25,.25,.25]])
CONTACT_IDS = np.repeat(FACES, len(CONTACT_WEIGHTS), axis=0)
SAMPLE_WEIGHTS = np.tile(CONTACT_WEIGHTS, (len(FACES), 1))


def fixture_contact(points, before, folded_clean=False):
    # Horizontal support surfaces use unilateral height constraints.
    points[:, 2] = np.maximum(points[:, 2], TOP + THICKNESS)
    for lower, upper in SOLIDS:
        project_box(points, before, lower, upper, THICKNESS)
    if folded_clean:
        project_box(points, before, *FOLDED_CLEAN, THICKNESS)


def face_contact(points, before, vertices, previous, search=None):
    """Project cloth vertices against the other layer's swept triangles."""
    tri = vertices[TRIANGLES]
    old_tri = previous[TRIANGLES]
    lower = np.minimum(tri.min(axis=1), old_tri.min(axis=1)) - THICKNESS
    upper = np.maximum(tri.max(axis=1), old_tri.max(axis=1)) + THICKNESS
    points_lo, points_hi = np.minimum(points, before), np.maximum(points, before)
    candidates = np.flatnonzero(np.all((points_hi > lower.min(axis=0))
                                       & (points_lo < upper.max(axis=0)), axis=1))
    bounds = ((points_hi[candidates, None, :] > lower)
              & (points_lo[candidates, None, :] < upper)).all(axis=2)
    if search is not None:
        bounds &= search[candidates]
    rows, j = np.nonzero(bounds)
    i = candidates[rows]
    if not len(i):
        return
    p, q, r = tri[j].transpose(1, 0, 2)
    old_p, old_q, old_r = old_tri[j].transpose(1, 0, 2)
    u, v = q - p, r - p
    normals = np.cross(u, v)
    magnitude = np.linalg.norm(normals, axis=1)
    n = normals / np.maximum(magnitude[:, None], 1e-10)
    old_n = np.cross(old_q-old_p, old_r-old_p)
    old_n /= np.maximum(np.linalg.norm(old_n, axis=1, keepdims=True), 1e-10)
    old_n *= np.where(np.sum(n*old_n, axis=1, keepdims=True) >= 0, 1., -1.)
    d0 = np.sum((before[i] - old_p)*old_n, axis=1)
    relative = points[i] - p
    d1 = np.sum(relative*n, axis=1)
    side = np.where(d0 >= 0, 1., -1.)
    near = np.abs(d1) < THICKNESS
    crossing = ((d0*d1 < 0) & (np.abs(d0) > 1e-5))
    tau = np.where(crossing, np.clip(d0 / np.where(crossing, d0-d1, 1.), 0., 1.), 1.)
    test = np.where(crossing[:, None],
                    before[i] + tau[:, None]*(points[i]-before[i]), points[i])
    origin = old_p + tau[:, None]*(p-old_p)
    edge1 = old_q-old_p + tau[:, None]*(u-(old_q-old_p))
    edge2 = old_r-old_p + tau[:, None]*(v-(old_r-old_p))
    rel = test-origin
    aa, ab, bb = (np.sum(edge1*edge1, axis=1), np.sum(edge1*edge2, axis=1), np.sum(edge2*edge2, axis=1))
    ar, br = np.sum(edge1*rel, axis=1), np.sum(edge2*rel, axis=1)
    det = np.maximum(aa*bb-ab*ab, 1e-16)
    alpha, beta = (bb*ar-ab*br)/det, (aa*br-ab*ar)/det
    hit = ((near | crossing) & (magnitude > 1e-7)
           & (alpha >= -.025) & (beta >= -.025) & (alpha+beta <= 1.025))
    delta = np.maximum(0., THICKNESS-side*d1)
    if search is not None:
        # Limit corrections between folds of the same moving mesh. Large
        # stale-plane offsets otherwise pull wrinkles through nearby layers.
        hit &= np.abs(d0) < .018
        delta = np.minimum(delta, THICKNESS)
    hits = np.flatnonzero(hit)
    # Resolve the strongest plane for each sample instead of accumulating
    # equivalent contacts on adjacent triangles.
    hits = hits[np.argsort(-delta[hits])]
    _, first = np.unique(i[hits], return_index=True)
    selected = hits[first]
    points[i[selected]] += (side[selected]*delta[selected])[:, None] * n[selected]


class Sheet:
    def __init__(self, clean):
        self.clean = clean
        self.xyz = make_positions()
        if clean:
            for i in range(GRID_X):
                s = i / (GRID_X-1)
                width = .186 + .049*s
                for j in range(GRID_Y):
                    y = (j/(GRID_Y-1)*2 - 1)*width
                    self.xyz[i*GRID_Y+j] = (FOLD_START+.015*(1-s)+(.004 if i%2 else 0), y,
                                            TOP+.022+.007*s+.006*(i%2))
            self.xyz[CORNER_I] = CLEAN_FOLD
        else:
            self.xyz[CORNER_I] = SEAT
        self.velocity = np.zeros_like(self.xyz)
        self.latched = np.full(4, not clean, dtype=bool)
        self.folded = np.full(4, clean, dtype=bool)
        self.grasps = [None] * 4
        self.threaded = [False] * 4
        self.started = not clean
        self.body_grasps = {}

    def grip(self, hands, occupied):
        anchors = {}
        for i, vertex in enumerate(CORNER_I):
            owner = self.grasps[i]
            if owner is None:
                for name, hand in hands.items():
                    tip = hand.position - (0, 0, PAD) - TAB_OFFSETS[i]
                    if (name not in occupied and hand.upright and hand.open_fraction < .31
                            and np.linalg.norm(tip-self.xyz[vertex]) < .015):
                        owner = self.grasps[i] = name
                        occupied.add(name)
                        self.folded[i] = False
                        self.started = True
                        break
            if owner is None:
                continue
            hand = hands[owner]
            tip = hand.position - (0, 0, PAD) - TAB_OFFSETS[i]
            clearance = (tip[:2] - CORNERS[i])*SIGNS[i]
            outside = bool(np.all(clearance > .017))
            if self.latched[i] and outside and tip[2] < ZTOP + .012:
                self.latched[i] = False
                self.threaded[i] = False
            if not self.latched[i]:
                if outside and ZBOTTOM -.008 < tip[2] < ZTOP -.011:
                    self.threaded[i] = True
                if (self.threaded[i] and np.all(clearance > -.008) and np.all(clearance < .020)
                        and ZBOTTOM -.008 < tip[2] < ZBOTTOM + .021):
                    self.latched[i] = True
                    self.threaded[i] = False
            if hand.open_fraction >= .58:
                self.grasps[i] = None
                self.threaded[i] = False
            elif self.clean or not self.latched[i]:
                anchors[vertex] = tip
        for i, vertex in enumerate(CORNER_I):
            if self.latched[i] and not self.clean:
                anchors[vertex] = SEAT[i]
            elif self.folded[i]:
                anchors[vertex] = CLEAN_FOLD[i]
        if not self.clean and self.started and not bool(np.any(self.latched)):
            candidates = np.array([i for i in range(len(self.xyz)) if i not in CORNER_I], dtype=int)
            for name, hand in hands.items():
                tip = hand.position - (0, 0, PAD)
                if name not in occupied and name not in self.body_grasps and hand.open_fraction < .31 and hand.upright:
                    gaps = np.linalg.norm(self.xyz[candidates] - tip, axis=1)
                    best = int(np.argmin(gaps))
                    if gaps[best] < .020 and int(candidates[best]) not in self.body_grasps.values():
                        self.body_grasps[name] = int(candidates[best])
                if name not in self.body_grasps:
                    continue
                vertex = self.body_grasps[name]
                occupied.add(name)
                if hand.open_fraction >= .58:
                    del self.body_grasps[name]
                else:
                    anchors[vertex] = tip
        return anchors

    def solve(self, anchors, other, previous_other, dt):
        if not self.started:
            return
        indices = np.array(list(anchors), dtype=int)
        values = np.array(list(anchors.values())).reshape(-1, 3)
        free = np.ones(len(self.xyz), dtype=bool)
        free[indices] = False
        sample_free = SAMPLE_WEIGHTS * free[CONTACT_IDS]
        weights_sq = np.sum(sample_free**2, axis=1)
        vertices = self.xyz
        folded_clean = other.clean and not other.started
        before = vertices.copy()
        vertices += self.velocity * dt
        vertices[:, 2] -= 7.0*dt*dt
        before_samples = np.einsum("ni,nij->nj", SAMPLE_WEIGHTS, before[CONTACT_IDS])
        for iteration in range(10):
            diffs = vertices[B] - vertices[A]
            length = np.linalg.norm(diffs, axis=1)
            strain = length-(CLEAN_REST if self.clean else REST)
            strain = np.maximum(0., strain) + .28*np.minimum(0., strain)
            correction = (STIFFNESS * strain/np.maximum(length, 1e-8))[:, None]*diffs
            fixes = np.zeros_like(vertices)
            np.add.at(fixes, A, correction)
            np.add.at(fixes, B, -correction)
            vertices += 1.5*fixes/DEGREE
            # Only the actual elastic hem has a retaining force. Any adjacent
            # sheet vertex must get there through cloth tension and contact.
            for i, vertex in enumerate(CORNER_I):
                if not self.clean or not self.latched[i] or vertex in anchors:
                    continue
                displacement = vertices[vertex] - SEAT[i]
                opening = np.r_[np.maximum(displacement[:2] * SIGNS[i], 0.),
                                max(displacement[2], 0.)]
                if np.linalg.norm(opening) > POCKET_ESCAPE_M:
                    self.latched[i] = self.threaded[i] = False
                else:
                    vertices[vertex] -= POCKET_CONSTRAINT_GAIN * displacement
            vertices[indices] = values
            fixture_contact(vertices, before, folded_clean)
            if iteration % 2 == 1:
                samples = np.einsum("ni,nij->nj", SAMPLE_WEIGHTS, vertices[CONTACT_IDS])
                projected = samples.copy()
                fixture_contact(projected, before_samples, folded_clean)
                if iteration == 9:
                    cloth_projected = projected.copy()
                    face_contact(cloth_projected, before_samples, other.xyz, previous_other)
                    projected += np.clip(cloth_projected-projected, -.006, .006)
                change = (projected-samples)/np.maximum(weights_sq[:, None], 1e-8)
                hit = np.any(change != 0., axis=1) & (weights_sq > 0.)
                ids = CONTACT_IDS[hit]
                delta = sample_free[hit, :, None]*change[hit, None, :]
                fixes.fill(0.)
                np.add.at(fixes, ids, delta)
                count = np.bincount(ids.ravel(), minlength=len(vertices))
                vertices += fixes / np.maximum(count[:, None], 1)
        free_vertices = vertices[free]
        face_contact(free_vertices, before[free], other.xyz, previous_other)
        face_contact(free_vertices, before[free], vertices, before, SELF_CONTACT[free])
        vertices[free] = free_vertices
        fixture_contact(vertices, before, folded_clean)
        self.velocity = (vertices-before) / dt * .84
        self.velocity[indices] = 0.


def silhouette(vertices, bounds, minimum_height=None):
    """Rasterize the overhead projection of the actual cloth triangles."""
    xmin, xmax, ymin, ymax = bounds
    nx = round((xmax-xmin) / PHOTO_SAMPLE_M)
    ny = round((ymax-ymin) / PHOTO_SAMPLE_M)
    pixels = np.zeros((nx, ny), dtype=bool)
    for triangle in vertices[TRIANGLES]:
        projected = (triangle[:, :2] - (xmin, ymin)) / PHOTO_SAMPLE_M - .5
        lo = np.maximum(np.ceil(projected.min(axis=0)).astype(int), 0)
        hi = np.minimum(np.floor(projected.max(axis=0)).astype(int) + 1, (nx, ny))
        if np.any(hi <= lo):
            continue
        edge1, edge2 = projected[1:] - projected[0]
        det = edge1[0] * edge2[1] - edge1[1] * edge2[0]
        if abs(det) < 1e-9:
            continue
        xx, yy = np.ogrid[lo[0]:hi[0], lo[1]:hi[1]]
        xx, yy = xx - projected[0, 0], yy - projected[0, 1]
        b = (xx * edge2[1] - yy * edge2[0]) / det
        c = (yy * edge1[0] - xx * edge1[1]) / det
        visible = (b >= -1e-8) & (c >= -1e-8) & (b+c <= 1 + 1e-8)
        if minimum_height is not None:
            z0, z1, z2 = triangle[:, 2]
            visible &= (z0 + b*(z1-z0) + c*(z2-z0) >= minimum_height)
        pixels[lo[0]:hi[0], lo[1]:hi[1]] |= visible
    return pixels


class State:
    def __init__(self, case="nominal"):
        self.old = Sheet(clean=False)
        self.clean = Sheet(clean=True)

    def advance(self, hands, dt):
        occupied = set(self.old.body_grasps) | {owner for owner in self.old.grasps + self.clean.grasps if owner is not None}
        clean_anchors = self.clean.grip(hands, occupied)
        old_anchors = self.old.grip(hands, occupied)
        for _ in range(3):
            before = self.clean.xyz.copy()
            self.clean.solve(clean_anchors, self.old, self.old.xyz, dt/3)
            self.old.solve(old_anchors, self.clean, before, dt/3)
        # The used fabric must have cleared the sleeping surface. Bulk of the
        # fabric must rest in the laundry area, with both arms released.
        old = self.old.xyz
        surface_samples = np.einsum("ni,nij->nj", SAMPLE_WEIGHTS, old[CONTACT_IDS])
        cloth_samples = np.concatenate((old, surface_samples))
        ox, oy, oz = cloth_samples.T
        old_mattress = ((ox >= X0) & (ox <= X1)
                        & (np.abs(oy) <= Y) & (oz >= ZBOTTOM-.003))
        off = not bool(np.any(old_mattress) or np.any(self.old.latched) or any(self.old.grasps) or self.old.body_grasps)
        laundry_fraction = 0.
        if off:
            xmin, ymin = np.floor(old[:, :2].min(axis=0) / PHOTO_SAMPLE_M) * PHOTO_SAMPLE_M
            xmax, ymax = np.ceil(old[:, :2].max(axis=0) / PHOTO_SAMPLE_M) * PHOTO_SAMPLE_M
            mask = silhouette(old, (xmin, xmax, ymin, ymax))
            xx = xmin + (np.arange(mask.shape[0])[:, None] + .5) * PHOTO_SAMPLE_M
            yy = ymin + (np.arange(mask.shape[1])[None, :] + .5) * PHOTO_SAMPLE_M
            pad = (xx >= -.0135) & (xx <= .1935) & (np.abs(yy) <= .300)
            laundry_fraction = float(np.count_nonzero(mask & pad) / max(1, np.count_nonzero(mask)))
        sorted_old = bool(off and laundry_fraction >= PHOTO_LAUNDRY_MIN)
        fit = int(np.sum(self.clean.latched))
        released = not any(self.clean.grasps)
        cloth = self.clean.xyz
        coverage = float(silhouette(cloth, (X0, X1, -Y, Y), ZTOP-.002).mean()) if fit else 0.
        score = 0.
        if off:
            score += .17
        if sorted_old:
            score += .08
        if off and released:
            score += .155 * fit
        if sorted_old and fit == 4 and coverage >= PHOTO_FIT_MIN and released:
            score = 1.
        return {
            "score": round(score, 4), "used_pockets": int(np.sum(self.old.latched)),
            "clean_pockets": fit, "used_removed": off, "laundry": sorted_old,
            "laundry_percent": round(laundry_fraction * 100, 1),
            "old_centroid": [round(float(v),4) for v in old.mean(axis=0)],
            "old_x_max": round(float(old[:,0].max()),4),
            "old_outliers": [[round(float(v),4) for v in pt] for pt in old[np.argsort(-old[:,0])[:4]]],
            "old_body_grasps": self.old.body_grasps.copy(),
            "surface_percent": round(coverage*100, 1),
            "clean_corner_xyz": [[round(float(v),4) for v in pt] for pt in cloth[CORNER_I]],
            "old_corner_xyz": [[round(float(v),4) for v in pt] for pt in old[CORNER_I]],
            "clean_grasps": self.clean.grasps.copy(), "used_grasps": self.old.grasps.copy(),
            "caption": f"{fit} of 4 clean pockets seated" if off else
                       f"{int(np.sum(self.old.latched))} used pockets still on mattress",
        }


# Render a denser fabric surface by interpolating the physical triangles.
RENDER_X, RENDER_Y = 25, 49
U, V = np.meshgrid(np.linspace(0, 1, RENDER_X), np.linspace(0, 1, RENDER_Y), indexing="ij")
HEM = np.r_[np.arange(RENDER_Y), np.arange(1, RENDER_X) * RENDER_Y + RENDER_Y - 1,
            np.arange(RENDER_Y - 2, -1, -1) + (RENDER_X - 1) * RENDER_Y,
            np.arange(RENDER_X - 2, 0, -1) * RENDER_Y]
# Move each perimeter sample one grid point inward, including the four corners.
HEM_INNER = (np.clip(HEM // RENDER_Y, 1, RENDER_X - 2) * RENDER_Y
             + np.clip(HEM % RENDER_Y, 1, RENDER_Y - 2))
HEM_FACES = [(i, (i + 1) % len(HEM), (i + 1) % len(HEM) + len(HEM), i + len(HEM))
             for i in range(len(HEM))]


def fabric_surface(sheet):
    # Subdivide the same triangles used by collision. A spline can bow
    # through the mattress or another layer even if its control grid clears it.
    x, y = U*(GRID_X-1), V*(GRID_Y-1)
    i, j = np.minimum(x.astype(int), GRID_X-2), np.minimum(y.astype(int), GRID_Y-2)
    dx, dy = (x-i)[..., None], (y-j)[..., None]
    xyz = sheet.xyz.reshape(GRID_X, GRID_Y, 3)
    origin, opposite = xyz[i, j], xyz[i+1, j+1]
    dense = np.where(dx >= dy,
                     origin*(1-dx) + xyz[i+1,j]*(dx-dy) + opposite*dy,
                     origin*(1-dy) + xyz[i,j+1]*(dy-dx) + opposite*dx)
    return dense.reshape(-1, 3)


class View:
    def __init__(self, state):
        g = self.g = Geometry()
        xmid = (X0 + X1)/2
        g.box("WoodSupport", (xmid, 0, TOP+.007), (X1-X0+.024, 2*Y+.020, .014), (.54,.37,.24))
        g.box("LaundryBackStripe", (.015, 0, TOP+.0058), (.007,.596,.0014), (.69,.65,.23))
        for j, y in enumerate((-.292, .292)):
            g.box(f"LaundryEdge{j}", (.093,y,TOP+.0058), (.187,.007,.0014), (.69,.65,.23))
        self.sheets = []
        self.hems = []
        self.tags = []
        for i, (sheet, color, seam_color) in enumerate((
            (state.old,(.63,.25,.20),(.52,.18,.15)),
            (state.clean,(.18,.46,.63),(.13,.35,.52)),
        )):
            points = fabric_surface(sheet)
            mesh = g.mesh(f"Fabric{i}", points, grid_faces(RENDER_X, RENDER_Y), color)
            self.sheets.append(mesh)
            hem = g.mesh(f"ElasticHem{i}", self.hem_points(points), HEM_FACES, seam_color)
            self.hems.append(hem)
            tabs = [g.mesh(f"PullTab{i}_{j}", self.tab_points(sheet, j),
                           [(k,k+1,k+6,k+5) for k in range(4)], color)
                    for j in range(4)]
            self.tags.append(tabs)
        self.update(state)

    @staticmethod
    def hem_points(surface):
        exterior = surface[HEM]
        interior = surface[HEM_INNER]
        offset = np.array([0, 0, .001])
        return np.concatenate((exterior + offset, exterior + .47*(interior-exterior) + offset))

    @staticmethod
    def tab_points(sheet, i):
        base = sheet.xyz[CORNER_I[i]]
        lengths = np.linspace(0, 1, 5)
        center = base + lengths[:, None] * TAB_OFFSETS[i]
        # The free fabric tab droops under its own weight. A pinch straightens it.
        drop = .009 if sheet.grasps[i] is None else .001
        center[:, 2] += .002 - drop * lengths**2
        width = .008 - .002 * lengths**2
        cross = width[:, None] * (1., 0., .12)
        return np.concatenate((center-cross, center+cross))

    def update(self,state):
        for i,sheet in enumerate((state.old,state.clean)):
            points = fabric_surface(sheet)
            self.g.points(self.sheets[i], points)
            self.g.points(self.hems[i], self.hem_points(points))
            for j, tab in enumerate(self.tags[i]):
                self.g.points(tab, self.tab_points(sheet, j))
