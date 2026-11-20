"""Scorer tests for steamtable.py, run without the simulator:  python test_steamtable.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import steamtable as S  # noqa: E402

AWAY_L, AWAY_R = (0.18, 0.27, S.TOP + 0.20), (0.18, -0.27, S.TOP + 0.20)
Z_HALF = (S.PLATE_T + S.POST_H + S.STRAP_T) / 2  # the cover's box: plate underside to strap top
REST = 25
LEFT, MID, RIGHT = S.PAN_Y


def _obj(pos, q=(0.0, 0.0, 0.0, 1.0), lo=None, hi=None):
    pos = np.asarray(pos, float)
    lo = pos - 0.01 if lo is None else np.asarray(lo, float)
    hi = pos + 0.01 if hi is None else np.asarray(hi, float)
    return ObjectState(pos, np.asarray(q, float), np.zeros(3), np.zeros(3), lo, hi)


def cover_state(xy, bottom_z, tilt_deg=0.0, yaw_deg=0.0, origin_off=(0.0, 0.0)):
    """A cover with its underside at bottom_z (absolute), its plate centre over xy. Returns (body, welded marker).
    origin_off is the body origin's offset from the plate centre in the cover's own frame (0.03 for a notched cover)."""
    pos = np.array([xy[0], xy[1], bottom_z + Z_HALF])
    ay, az = math.radians(tilt_deg) / 2, math.radians(yaw_deg) / 2
    qy = np.array([0, math.sin(ay), 0, math.cos(ay)])
    qz = np.array([0, 0, math.sin(az), math.cos(az)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qy
    q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    mark = _obj([xy[0], xy[1], bottom_z + S.PLATE_T / 2], q)
    r = S._rot(q)
    body_pos = pos + r @ np.array([origin_off[0], origin_off[1], 0.0])
    return _obj(body_pos, q, pos - [0.16, 0.09, Z_HALF], pos + [0.16, 0.09, Z_HALF]), mark


def ladle_state(name, dx=0.0, dz=0.0, turn_deg=0.0):
    cy = S.LADLE_PANS[name]
    pos = np.array([S.CX + S.LADLE_BOWL_X + 0.1 + dx, cy, S.TOP + 0.1 + dz])
    a = math.radians(turn_deg) / 2
    return _obj(pos, (0.0, math.sin(a), 0.0, math.cos(a)))


START = {"cover_solid": (S.CX, 0.370, S.TOP), "cover_a": (S.CX, 0.554, S.TOP), "cover_b": (S.CX, -0.370, S.TOP)}


def frame(covers=None, ladles=None, left=AWAY_L, right=AWAY_R):
    c = dict(START)
    c.update(covers or {})
    f = {}
    for n, v in c.items():
        body, mark = cover_state((v[0], v[1]), v[2]) if len(v) == 3 else v
        f[n], f[n + "_mark"] = body, mark
    for n in S.LADLE_PANS:
        f[n] = (ladles or {}).get(n, ladle_state(n))
    for arm, p in (("left", left), ("right", right)):  # fingers 1.3 cm either side of the TCP (along y)
        f[f"{arm}_tcp"] = _obj(p)
        for k, dy in (("l", 0.013), ("r", -0.013)):
            f[f"{arm}_finger_{k}"] = _obj((p[0], p[1] + dy, p[2]))
    return f


def run(frames):
    S._state.clear()
    S._state.update(S._fresh())
    s = 0.0
    for f in frames:
        s = S.score(f)
    return s


SEAT = S.TOP + S.RIM
ON = {"cover_solid": (S.CX, MID, SEAT), "cover_a": (S.CX, LEFT, SEAT), "cover_b": (S.CX, RIGHT, SEAT)}


def seated(**kw):
    c = {k: v for k, v in ON.items()}
    c.update(kw)
    return c


def test_doing_nothing_scores_zero():
    assert run([frame()] * 40) == 0.0


def test_all_three_covered_scores_one():
    assert run([frame(seated())] * REST) == 1.0


def test_needs_a_second_still():
    assert run([frame(seated())] * 10) == 0.0


def test_one_second_exactly_counts():
    assert run([frame(seated())] * 20) == 1.0


def test_still_holding_a_cover_scores_zero():
    held = frame(seated(), right=(S.CX, RIGHT, S.TOP + 0.14))
    assert run([frame()] * 3 + [held] * REST) == 0.0


def test_moving_covers_are_not_still():
    f = [frame(seated(cover_solid=(S.CX + 0.01 * (i % 2), MID, SEAT))) for i in range(REST)]
    assert run(f) == 0.0


def test_empty_pan_solid_cover_counts_a_third():
    assert abs(run([frame({"cover_solid": (S.CX, MID, SEAT)})] * REST) - 1 / 3) < 1e-3


def test_notched_cover_on_the_empty_pan_counts_a_sixth():
    f = frame({"cover_a": (S.CX, MID, SEAT)})
    assert abs(run([f] * REST) - 1 / 6) < 1e-3


def test_only_the_ladle_pan_covers_score_two_thirds():
    f = frame({"cover_a": (S.CX, LEFT, SEAT), "cover_b": (S.CX, RIGHT, SEAT)})
    assert abs(run([f] * REST) - 2 / 3) < 1e-3


def test_solid_cover_resting_on_a_handle_scores_nothing_for_that_pan():
    """Sitting proud on the ladle handle (about 12 mm above the rim)."""
    f = frame(seated(cover_solid=(S.CX, LEFT, SEAT + 0.012), cover_a=(S.CX, MID, SEAT)))
    s = run([f] * REST)
    assert abs(s - (1 / 6 + 1 / 3)) < 1e-3  # right pan: cover_b; middle: notched cover (half); left: nothing


def test_notch_away_from_the_handle_sits_proud_and_scores_nothing():
    f = frame(seated(cover_a=(S.CX, LEFT, SEAT + 0.012)))
    assert abs(run([f] * REST) - 2 / 3) < 1e-3


def test_cover_tilted_is_not_seated():
    f = frame(seated(cover_solid=cover_state((S.CX, MID), SEAT, tilt_deg=6.0)))
    assert abs(run([f] * REST) - 2 / 3) < 1e-3


def test_cover_turned_ninety_degrees_is_not_seated():
    f = frame(seated(cover_solid=cover_state((S.CX, MID), SEAT, yaw_deg=90.0)))
    assert abs(run([f] * REST) - 2 / 3) < 1e-3


def test_cover_off_centre_is_not_seated():
    f = frame(seated(cover_solid=(S.CX, MID + 0.03, SEAT)))
    assert abs(run([f] * REST) - 2 / 3) < 1e-3


def test_cover_straddling_two_pans_scores_nothing():
    f = frame({"cover_solid": (S.CX, (MID + LEFT) / 2, SEAT)})
    assert run([f] * REST) == 0.0


def test_ladle_taken_out_scores_zero_for_that_pan_even_if_put_back():
    out = frame(seated(), ladles={"ladle_l": ladle_state("ladle_l", dz=0.2)})
    back = frame(seated())
    assert abs(run([frame()] * 3 + [out] * 5 + [back] * REST) - 2 / 3) < 1e-3


def test_ladle_pushed_into_the_pan_scores_zero_for_that_pan():
    f = frame(seated(), ladles={"ladle_r": ladle_state("ladle_r", dx=-0.05)})
    assert abs(run([frame()] * 3 + [f] * REST) - 2 / 3) < 1e-3


def test_ladle_turned_scores_zero_for_that_pan():
    f = frame(seated(), ladles={"ladle_r": ladle_state("ladle_r", turn_deg=14.0)})
    assert abs(run([frame()] * 3 + [f] * REST) - 2 / 3) < 1e-3


def test_small_ladle_nudge_is_tolerated():
    f = frame(seated(), ladles={"ladle_l": ladle_state("ladle_l", dx=0.01, turn_deg=4.0)})
    assert run([frame()] * 3 + [f] * REST) == 1.0


def test_ladle_removal_does_not_cost_the_other_pans():
    f = frame(seated(), ladles={"ladle_l": ladle_state("ladle_l", dz=0.2)})
    assert abs(run([frame()] * 3 + [f] * REST) - 2 / 3) < 1e-3


def test_cover_knocked_off_the_table_scores_zero():
    f = frame({"cover_solid": (S.CX, 0.9, S.TOP - 0.5)})
    assert run([f] * REST) == 0.0


def test_centred_solid_cover_scores():
    f = frame({"cover_solid": (S.CX, MID, SEAT)})
    assert abs(run([f] * REST) - 1 / 3) < 1e-3


def test_notched_cover_centred_with_offset_origin_scores():
    """Plate centred on the left pan, body origin 3 cm off (as the real body); the body position must not matter."""
    c = cover_state((S.CX, LEFT), SEAT, origin_off=(0.03, 0.0))
    f = frame({"cover_a": c})
    assert abs(run([f] * REST) - 1 / 3) < 1e-3
    c = cover_state((S.CX, LEFT), SEAT, yaw_deg=180.0, origin_off=(0.03, 0.0))  # turned half a turn
    assert abs(run([frame({"cover_a": c})] * REST) - 1 / 3) < 1e-3


def test_notched_cover_origin_near_pan_centre_but_plate_off_fails():
    """The body origin is on the pan's centre (old rule: seated) but the plate is 3 cm off, so it does not count."""
    c = cover_state((S.CX - 0.03, LEFT), SEAT, origin_off=(0.03, 0.0))
    assert c[0].pos[0] > S.CX - 0.001 and abs(c[0].pos[0] - S.CX) < 0.001
    assert run([frame({"cover_a": c})] * REST) == 0.0


def test_gripper_holding_a_cover_by_its_edge_is_not_released():
    edge = (S.CX + S.PAN_L / 2 - 0.01, MID, SEAT + 0.001)  # far from the origin, at the plate's short edge
    f = frame(seated(), right=edge)
    assert run([frame()] * 3 + [f] * REST) == 0.0
    f = frame(seated(), right=(S.CX, MID + S.PAN_W / 2 - 0.005, SEAT + 0.001))  # long edge
    assert run([f] * REST) == 0.0


def test_finger_touching_the_strap_is_not_released():
    f = frame(seated(), left=(S.CX, MID, SEAT + S.BAR_DZ + S.PLATE_T / 2))
    assert run([f] * REST) == 0.0


def test_finger_just_clear_of_the_cover_is_released():
    f = frame(seated(), left=(S.CX + 0.25, 0.0, SEAT + 0.05))
    assert run([f] * REST) == 1.0


def test_demo_waypoints_are_reachable_values():
    d = S.demo()
    assert set(d) == {"left", "right"}
    assert abs(sum(w.duration for w in d["left"]) - sum(w.duration for w in d["right"])) < 1e-6


def test_start_geometry_fits():
    assert S.COVER_Y["cover_a"] + S.PAN_W / 2 < 0.65  # inside the table's width
    assert 0.001 < S.HANDLE_ALPHA < 0.5
    # handle clears the notch: rod top crosses the rim before the notch's inner edge
    x_cross = S.WALL_IN_X - (S.HANDLE_T / math.cos(S.HANDLE_ALPHA)) / math.tan(S.HANDLE_ALPHA)
    assert S.PAN_L / 2 - S.NOTCH_D < x_cross - 0.005
    assert S.NOTCH_W / 2 > S.HANDLE_W / 2 + 0.01


if __name__ == "__main__":
    names = [n for n in dir() if n.startswith("test_")]
    for n in names:
        globals()[n]()
    print(f"{len(names)} tests passed")
