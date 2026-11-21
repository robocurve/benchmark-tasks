"""Construct the original full-size cookbook and spring page holders."""

from math import cos, sin

import numpy as np

from . import config as C
from .motion import rotation
from .pages import print_face
from .usd_scene import Writer

WOOD = (.69, .43, .23)
EDGE = (.57, .33, .15)
PAPER = (.93, .86, .67)
COVER = (.21, .33, .23)
BLACK = (.12, .13, .14)
STEEL = (.51, .56, .59)


def quat(w, matrix):
    from yam_arena.ik import matrix_to_quat_xyzw

    q = matrix_to_quat_xyzw(matrix)
    return w.Gf.Quatf(float(q[3]), w.Gf.Vec3f(*map(float, q[:3])))


def orient(w, s, path, matrix):
    q = quat(w, matrix)
    v = q.GetImaginary()
    s.GetPrimAtPath(path).GetAttribute("xformOp:orient").Set(
        w.Gf.Quatd(q.GetReal(), w.Gf.Vec3d(*v))
    )


def body(w, s, path, mass, pos, matrix, **kwargs):
    path = w.body(s, path, mass, position=pos, ccd=True, **kwargs)
    orient(w, s, path, matrix)
    return path


def joint(w, s, path, a, b, loc, rot, axis, lower, upper, *, linear=False):
    cls = w.Phy.PrismaticJoint if linear else w.Phy.RevoluteJoint
    j = cls.Define(s, path)
    j.CreateBody0Rel().SetTargets([a])
    j.CreateBody1Rel().SetTargets([b])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, loc)))
    j.CreateLocalRot0Attr(quat(w, rot))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(lower)
    j.CreateUpperLimitAttr(upper)
    return j


def drive(w, j, stiffness, damping, maximum, target=0., *, linear=False):
    d = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear" if linear else "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(stiffness)
    d.CreateDampingAttr(damping)
    d.CreateMaxForceAttr(maximum)
    d.CreateTargetPositionAttr(target)


def furniture(w, s):
    w.box(s, "/Asset/Worktop", (1.08, 1.25, .045), (.30, 0, C.TABLE - .0225),
          (.66, .61, .52))
    for i, x in enumerate((-.18, .77)):
        for j, y in enumerate((-.55, .55)):
            w.box(s, f"/Asset/Leg{i}{j}", (.051, .051, C.TABLE - .045),
                  (x, y, (C.TABLE - .045) / 2), (.21, .23, .24))
    w.box(s, "/Asset/Floor", (8., 8., .02), (0, 0, -.01), (.34, .38, .39))


def stand(w):
    s = w.stage("stand")
    root = body(w, s, "/Asset/Bamboo", .91, np.zeros(3), np.eye(3),
                fixed=True, articulation=True)

    def block(label, size, loc, color=WOOD, *, extra=None, collide=True):
        frame = C.FRAME if extra is None else C.FRAME @ extra
        w.box(s, root + "/" + label, size, C.world(loc), color, frame, collide=collide)

    block("Back", (.012, .335, .240), (.006, 0., .111))
    block("Ledge", (.030, .335, .015), (-.015, 0., -.012))
    for i in range(10):
        tone = WOOD if i % 2 else (.72, .47, .27)
        block(f"BambooLamella{i}", (.00013, .335, .005),
              (-.0001, 0., .005 + .022 * i), tone, collide=False)
    for i, side in enumerate(("left", "right")):
        sigma = C.sign(side)
        y = sigma * .115
        w.box(s, root + "/Foot" + side, (.133, .018, .015),
              (.478, y, C.TABLE + .0075), EDGE)
        w.box(s, root + "/Prop" + side, (.015, .018, .170),
              (.501, y, C.TABLE + .080), WOOD, rotation("Y", -.30))
        block(f"Hinge{i}", (.008, .047, .006), (-.013, y, -.0035), STEEL, collide=False)

        pivot = C.pivot(side)
        r = rotation("X", np.radians(-sigma * C.CLIP_START))
        clip = body(w, s, f"/Asset/Clip{i}", .012, C.world(pivot), C.FRAME @ r)
        w.cylinder(s, clip + "/Pivot", C.PIVOT_RADIUS, C.PIVOT_DEPTH, (0, 0, 0), BLACK, "X")
        # Raised spring strip gives fingers access to the metal and leaves room
        # for the shoe to yield toward the reader as it passes the paper edge.
        w.box(s, clip + "/Strip", C.STRIP_SIZE, C.strip_center(side), BLACK)
        w.box(s, clip + "/Bend", C.BEND_SIZE, C.bend_center(side), BLACK)
        lower, upper = (-C.CLIP_END + C.CLIP_START, 5.) if sigma == 1 else (
            -5., C.CLIP_END - C.CLIP_START)
        pivot_joint = joint(w, s, f"/Asset/Pivot{i}", root, clip, C.world(pivot),
                            C.FRAME @ r, "X", lower, upper)
        w.Px.PhysxJointAPI.Apply(pivot_joint.GetPrim()).CreateJointFrictionAttr(.26)
        drive(w, pivot_joint, .0, .00016, .006)

        offset = np.array([0., -sigma * C.CLIP_LENGTH, 0.])
        shoe = body(w, s, f"/Asset/Shoe{i}", .003, C.world(pivot + r @ offset),
                    C.FRAME @ r)
        w.box(s, shoe + "/Return", C.RETURN_SIZE, C.RETURN_CENTER, BLACK)
        w.box(s, shoe + "/Rubber", C.RUBBER_SIZE, C.RUBBER_CENTER, (.31, .34, .29))
        spring = joint(w, s, f"/Asset/Spring{i}", clip, shoe, offset, np.eye(3),
                       "X", -.029, .015, linear=True)
        drive(w, spring, 190., .72, 6., target=.003, linear=True)
    return s


def draw_spread(w, s, path, sigma):
    y = sigma * C.PAGE_CENTER_Y
    w.box(s, path + "/PageBlock", (.0125, C.PAGE_WIDTH, C.BOOK_HEIGHT - .004),
          (-.0075, y, (C.BOOK_HEIGHT + .002) / 2), PAPER)
    w.box(s, path + "/ClothBoard", (.0024, C.PAGE_WIDTH + .007, C.BOOK_HEIGHT),
          (-.00095, y, C.BOOK_HEIGHT / 2), COVER)
    for i in range(10):
        w.box(s, f"{path}/LeafEdge{i}", (.00012, .0003, C.BOOK_HEIGHT - .0045),
              (-.00215 - .00112 * i, sigma * (.074 + C.PAGE_WIDTH / 2 + .0003),
               (C.BOOK_HEIGHT + .002) / 2), (.79, .72, .57), collide=False)
    print_face(w, s, path, center_y=y, page_number=24 if sigma == 1 else 25)


def book(w):
    s = w.stage("cookbook")
    root = body(w, s, "/Asset/Spine", .032, np.zeros(3), np.eye(3), articulation=True)
    w.box(s, root + "/BoundSpine", (.017, .012, C.BOOK_HEIGHT),
          (-.003, 0, C.BOOK_HEIGHT / 2), COVER)
    for i, side in enumerate(("left", "right")):
        sigma = C.sign(side)
        loc = np.array([0., sigma * .0064, 0.])
        r = rotation("Z", np.radians(sigma * C.BOOK_START))
        page = body(w, s, f"/Asset/Leaves{i}", .148, loc, r)
        draw_spread(w, s, page, sigma)
        lo, hi = ((-C.BOOK_START - 1, 68.) if sigma == 1 else
                  (-68., C.BOOK_START + 1))
        crease = joint(w, s, f"/Asset/Binding{i}", root, page, loc, r, "Z", lo, hi)
        drive(w, crease, .014, .001, .30, target=sigma * 3.)
    return s


def create_assets(directory):
    w = Writer(directory)
    room = w.stage("room")
    furniture(w, room)
    return {"assets": [
        {"name": "table", "type": "base", "file": w.save(room)},
        {"name": "stand", "type": "articulation", "file": w.save(stand(w))},
        {"name": "book", "type": "articulation", "file": w.save(book(w)),
         "position": C.world(C.BOOK_ORIGIN).tolist(),
         "quaternion": [0., sin(np.pi / 12), 0., cos(np.pi / 12)]},
    ], "settle_actions": 15}
