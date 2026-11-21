"""Check opening and turnover targets against the stock YAM URDF, without simulating object contact."""

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def dry_run():
    from yam_arena import ik
    from yam_arena.constants import READY

    from rdb_tablets_cap import fixed_scene as f, layout as L, scripted as s

    joints = {arm: np.array(READY) for arm in f.ARMS}
    error = []
    station = np.asarray(s.OPENING_STATION)
    z = np.array((0., 0., 1.))
    cap_offset = .007 * z
    landing = np.array((*s.CAP_DOWN_XY, .145))

    def probe(arm, p, r):
        q, dp, dr = ik.solve(p, ik.matrix_to_quat_xyzw(r), joints[arm], f.BASES[arm], iters=100)
        joints[arm] = q
        margin = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
        error.append((1000 * dp, np.degrees(dr), np.degrees(margin)))

    probe("right", station + s.BOTTLE_GRIP_Z * z, s.BOTTLE_GRIP)
    for delta in np.linspace(.050, -.0035, 17):
        probe("left", station + (L.CLOSED_Z - .007 + delta) * z, s.OPENING_GRIP)
    for degrees in np.linspace(0., s.OPENING_TURN_DEG, 33):
        turn = Rotation.from_euler("z", degrees, degrees=True).as_matrix()
        approach = turn @ s.OPENING_GRIP
        probe("left", station + (L.CLOSED_Z - .007 - .0035) * z, approach)
    for delta in np.linspace(-.0035, .042, 9):
        probe("left", station + (L.CLOSED_Z - .007 + delta) * z, approach)
    for a in np.linspace(0., 1., 9):
        probe("right", station * (1-a) + np.asarray(s.HOLD_AT) * a + s.BOTTLE_GRIP_Z*z, s.BOTTLE_GRIP)
    for a in np.linspace(0., 1., 9):
        origin = (station + (L.CLOSED_Z + .042)*z) * (1-a) + landing*a
        probe("left", origin - cap_offset, approach)
    for degrees in np.linspace(0., s.TURNOVER_DEG, 33):
        roll = s.cap_rotation(np.radians(degrees), s.TIP_AXIS_CUP)
        probe("left", landing - roll @ cap_offset, roll @ approach)
    for height in np.linspace(landing[2], s.TURNOVER_RELEASE_Z, 15):
        probe("left", np.array((*s.CAP_DOWN_XY, height)) - roll @ cap_offset, roll @ approach)
    for height in np.linspace(.050, 0., 10):
        probe("left", np.array((*s.CAP_DOWN_XY, s.CAP_GRIP_Z + height)), s.CAP_GRIP)
    measured = np.asarray(error)
    return dict(waypoints=len(error), max_position_error_mm=float(measured[:,0].max()),
                max_orientation_error_deg=float(measured[:,1].max()),
                min_joint_limit_margin_deg=float(measured[:,2].min()))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-arena", help="local checkout of the stock YAM runtime")
    args = parser.parse_args()
    sys.path[:0] = [str(ROOT), str(ROOT / "tools/stubs")] + ([args.yam_arena] if args.yam_arena else [])
    print(dry_run())
