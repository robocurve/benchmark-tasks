"""The hand fits the handle: checked on the YAM URDF collision hulls (the same hulls Isaac collides).

Isaac run 2 put the grasp point 80 mm behind the board's front edge; with the jaws open the finger roots, 61.5 mm
behind the fingertip centre, span the board's thickness, so the hands stopped 18.5 mm short and never reached the
paddles. These checks would have caught that.
"""

import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest

pytest.importorskip("yam_arena")
from yam_arena import YAM_URDF, ik  # noqa: E402
from yam_arena.constants import ALL_JOINTS, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.urdf import forward_kinematics, rpy_matrix  # noqa: E402

from rdb_desk_converter import layout as L  # noqa: E402
from rdb_desk_converter.fixed_scene import BASES, START_THETA  # noqa: E402
from rdb_desk_converter.scripted import GRIP_ROT, OPEN_V, SEEDS  # noqa: E402

HAND = ("gripper", "tip_left", "tip_right")


def _stl(path):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    rows = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return np.unique(rows["v"].reshape(-1, 3).astype(float).round(5), axis=0)


def _hulls():
    out = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        if link.get("name") not in HAND:
            continue
        clouds = []
        for col in link.findall("collision"):
            origin = col.find("origin")
            xyz = np.array(origin.get("xyz", "0 0 0").split(), float) if origin is not None else np.zeros(3)
            rpy = np.array(origin.get("rpy", "0 0 0").split(), float) if origin is not None else np.zeros(3)
            mesh = col.find("geometry/mesh")
            if mesh is not None:
                clouds.append(_stl(YAM_URDF.parent / mesh.get("filename")) @ rpy_matrix(rpy).T + xyz)
        out[link.get("name")] = np.concatenate(clouds)
    return out


HULLS = _hulls()


def hand_cloud(q6, jaw, base):
    q = dict(zip(ALL_JOINTS[:6], q6, strict=True))
    q["joint7"] = q["joint8"] = float(jaw) * FINGER_OPEN
    poses = forward_kinematics(YAM_URDF, q)
    return np.concatenate([HULLS[n] @ poses[n][:3, :3].T + poses[n][:3, 3] + np.asarray(base) for n in HAND])


def box_gap(cloud, lo, hi):
    c, h = (np.asarray(lo) + hi) / 2, (np.asarray(hi) - lo) / 2
    d = np.abs(cloud - c) - h
    return float((np.linalg.norm(np.maximum(d, 0.), axis=1) + np.minimum(d.max(axis=1), 0.)).min())


def top_box(name, angle, lift=0.):
    t = L.top_origin(angle)
    if name == "paddle":
        c = np.array((L.HANDLE_X, L.HANDLE_Y, L.PADDLE_TOP_REST - L.PADDLE_HALF[2] + lift))
        return t + c - L.PADDLE_HALF, t + c + L.PADDLE_HALF
    (x0, x1), (y0, y1), (z0, z1) = L.SLAB if name == "slab" else L.TRAY
    return t + (x0, y0, z0), t + (x1, y1, z1)


def solve(point):
    q, err, rerr = ik.solve(point, ik.matrix_to_quat_xyzw(GRIP_ROT), SEEDS["left"], BASES["left"], iters=400)
    assert err < 5e-4 and rerr < 5e-3
    return q


def test_finger_root_depth_is_what_layout_says():
    q = dict.fromkeys(ALL_JOINTS[:6], 0.)
    q["joint7"] = q["joint8"] = OPEN_V * FINGER_OPEN
    poses = forward_kinematics(YAM_URDF, q)
    g = np.linalg.inv(poses["gripper"])
    deepest = -1.
    for n in HAND:
        pts = HULLS[n] @ poses[n][:3, :3].T + poses[n][:3, 3]
        local = (np.c_[pts, np.ones(len(pts))] @ g.T)[:, :3]
        band = (local[:, 0] > -.004) & (local[:, 0] < .0185)      # the board's thickness about the grasp line
        deepest = max(deepest, float(local[band, 2].max()) - TCP_OFFSET[2])
    assert deepest == pytest.approx(-L.FINGER_ROOT, abs=.0015)


@pytest.mark.parametrize("setting", [4, 9])
def test_open_hand_at_the_grasp_clears_board_paddle_and_tray(setting):
    angle = START_THETA if setting == 4 else L.seated(9)
    q = solve(L.grasp_point("left", angle))
    cloud = hand_cloud(q, OPEN_V, BASES["left"])
    assert box_gap(cloud, *top_box("slab", angle)) > .002
    assert box_gap(cloud, *top_box("paddle", angle)) > .002
    assert box_gap(cloud, *top_box("tray", angle)) > .003


def test_closed_hand_touches_board_and_squeezed_paddle_only_with_its_pads():
    angle = START_THETA
    point = L.grasp_point("left", angle)
    q = solve(point)
    stack = -L.PADDLE_TOP_REST + 2 * L.PADDLE_HALF[2] - L.PADDLE_TRAVEL          # 33 mm
    cloud = hand_cloud(q, stack / .0939, BASES["left"])
    roots = cloud[cloud[:, 0] < point[0] - .045]                                 # finger roots and housing
    lo, hi = top_box("slab", angle)
    in_band = roots[(roots[:, 2] > lo[2] - .001) & (roots[:, 2] < hi[2] + .001)]
    assert in_band[:, 0].max() < lo[0] - .005                                   # roots stay in front of the edge
    assert box_gap(roots, *top_box("paddle", angle, L.PADDLE_TRAVEL)) > .005
    assert box_gap(cloud, *top_box("slab", angle)) > -.0015 and box_gap(cloud, *top_box("tray", angle)) > .005
    pads = cloud[cloud[:, 0] >= point[0] - .025]
    assert abs(box_gap(pads, *top_box("slab", angle))) < .003                    # upper pad on the board
    assert abs(box_gap(pads, *top_box("paddle", angle, L.PADDLE_TRAVEL))) < .003  # lower pad under the paddle


def test_the_logged_run_2_hand_was_blocked_by_the_board_edge():
    """Geometry of run 2 (grasp 80 mm in): the open hand at the old grasp point penetrates the board."""
    angle = START_THETA
    old = L.grasp_point("left", angle) + (.080 - L.GRASP_DEPTH, 0., 0.)
    cloud = hand_cloud(solve(old), OPEN_V, BASES["left"])
    assert box_gap(cloud, *top_box("slab", angle)) < -.005
