"""Pure-Python tests of sink_stopper's rules without the simulator: the thread's catch, turn-back and snug, every score
tier, the hold, the whole-trial zeros, and the figures the rules rest on."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import scene as S  # noqa: E402
import task as T  # noqa: E402

CLEAR = {"left": (np.array([0.10, 0.30, T.TOP + 0.25]), 1.0), "right": (np.array([0.10, -0.30, T.TOP + 0.25]), 1.0)}
EYE = np.eye(3)


def rod_seated(pitch_deg: float = 0.0, dx: float = 0.0):
    """The ball on its seat, the rear pressed ``pitch_deg`` down (the tip rises)."""
    return (np.array([P.X_DRAIN + P.SEAT_OFFSET_X + dx, P.Y_DRAIN, T.TOP + P.PORT_Z]),
            mk.axis_angle((0.0, 1.0, 0.0), math.radians(pitch_deg)))


def stopper_at(gap: float = 0.0, yaw_deg: float = 0.0, tilt_deg: float = 0.0, dx: float = 0.0):
    return (np.array([P.X_DRAIN + dx, P.Y_DRAIN, T.TOP + P.FLANGE_Z + gap]),
            mk.axis_angle((0.0, 0.0, 1.0), math.radians(yaw_deg)) @ mk.axis_angle((1.0, 0.0, 0.0), math.radians(tilt_deg)))


def nut_at(front_u: float, off: float = 0.0, tilt_deg: float = 0.0, z: float | None = None):
    """The nut's front face ``front_u`` behind the drain's axis, ``off`` m above the port's axis, its axis turned
    ``tilt_deg`` in the level plane."""
    return (np.array([P.X_DRAIN + front_u, P.Y_DRAIN, T.TOP + P.PORT_Z + off if z is None else z]),
            mk.axis_angle((0.0, 0.0, 1.0), math.radians(tilt_deg)))


def turns(cw_deg: float) -> dict:
    return {j: math.radians(cw_deg) / len(S.TURN_JOINTS) for j in S.TURN_JOINTS}


CLOCK = [1.0]
LAST: dict = {}


def put(rod=None, stopper=None, nut=None, cw_deg: float | None = None, grippers=None) -> None:
    """One control step's state (the clock moves on 0.05 s a call)."""
    LAST.update(rod=rod, stopper=stopper, nut=nut, cw_deg=cw_deg, grippers=grippers)
    CLOCK[0] += 0.05
    poses = {"pivot_rod": rod or rod_seated(), "stopper": stopper or stopper_at(), "pivot_nut": nut or nut_at(P.U_START + 0.010)}
    mk.set_state(poses=poses, joints=None if cw_deg is None else turns(cw_deg), grippers=CLEAR if grippers is None else grippers,
                 time=CLOCK[0])


def still(steps: int) -> None:
    """``steps`` more control steps with nothing moving."""
    for _ in range(steps):
        put(**LAST)
        T.step()


def fresh() -> None:
    mk.mend_all()
    T.begin(0)


def caught(cw_deg: float = 0.0, rod=None, stopper=None) -> None:
    """A nut set square on the thread start and caught, then turned on ``cw_deg``."""
    fresh()
    put(rod=rod, stopper=stopper, nut=nut_at(P.U_START - 0.0001))
    T.step()
    assert T.thread_engaged()
    put(rod=rod, stopper=stopper, nut=nut_at(P.U_START - P.gap_for_cw_deg(0.0) + P.gap_for_cw_deg(cw_deg)), cw_deg=cw_deg)
    T.step()


def test_figures_and_thread_geometry() -> None:
    s = P.summary()
    assert sum(s["counts"].values()) == len(P.FIGURES) >= 40
    assert abs((P.U_START - P.U_SNUG) - P.THREAD_PITCH) < 1e-9  # the stand-in thread is one turn
    assert 0.7e-3 < P.WASHER_INTERFERENCE < 0.8e-3
    for cw in (0.0, 90.0, 359.0):
        assert abs(P.cw_for_gap_deg(P.gap_for_cw_deg(cw)) - cw) < 1e-9
    assert P.GAP_OPEN_MIN <= P.open_gap_for_pitch(22.0) <= P.GAP_OPEN_MAX
    assert P.NUT_FACETS >= 36 and P.UNTHREAD_DEG < 0.0 < P.CATCH_GAP_BAND


def test_instruction_names_every_scored_condition() -> None:
    text = T.INSTRUCTION.lower()
    for words in ("pivot rod through the stopper's hole", "drain port", "pivot nut", "until snug", "press the rod down",
                  "stopper lifts open", "let go", "both hands clear"):
        assert words in text, words
    assert T.INSTRUCTION.count(".") == 1


def test_score_text_names_each_tier_and_threshold() -> None:
    doc = " ".join((T.score.__doc__ or "").split())
    for words in ("0 for the whole trial", "30 mm over the bench", "coaxial within 0.5 mm", "square within 1.5 deg", "1.0 when",
                  "within 3 mm of its seat", "3 mm or more past", "within 20 deg", "8 to 22 mm", "within 4 mm", "8 deg of upright",
                  "1.0 s", "2 mm drift", "30 mm clear", "25 mm/s", "0.5 when", "0.25 when", "0 else"):
        assert words in doc, words
    assert P.HOLD_STEPS == 20 and abs(P.HANDS_CLEAR - 0.030) < 1e-12 and abs(P.TIP_PAST - 0.003) < 1e-12


def test_start_scores_zero() -> None:
    fresh()
    put(rod=(np.array([P.X_DRAIN + P.U_BALL_START, P.Y_DRAIN, T.TOP + P.PORT_Z - P.ROD_START_DROP]), EYE))
    T.step()
    assert not T.rod_in_port() and not T.thread_engaged() and T.score(None) == 0.0 and T.STATE["zero"] is None


def test_rod_in_port_through_hole_is_quarter() -> None:
    fresh()
    put()
    T.step()
    inside, through = T.hole_crossing()
    assert inside and through >= P.TIP_PAST and T.score(None) == 0.25


def test_rod_seated_but_hole_turned_away_or_short_scores_zero() -> None:
    fresh()
    put(stopper=stopper_at(yaw_deg=90.0))
    T.step()
    assert T.score(None) == 0.0
    put(rod=rod_seated(dx=0.004))  # ball 4 mm off its seat
    T.step()
    assert T.score(None) == 0.0


def test_square_start_catches_where_the_nut_stands() -> None:
    fresh()
    put(nut=nut_at(P.U_START - 0.00015))
    T.step()
    assert T.thread_engaged()
    c = T.STATE["catch"]
    assert abs(c["cw_deg"] - 0.15 / 1.6 * 360.0) < 0.05 and abs(c["land_gap_mm"] + 0.15) < 1e-6
    assert np.allclose(mk.pose("pivot_nut")[0], nut_at(P.U_START - 0.00015)[0])  # the catch moves nothing


def test_crooked_off_axis_or_far_start_never_catches() -> None:
    for nut in (nut_at(P.U_START - 0.0001, tilt_deg=2.0), nut_at(P.U_START - 0.0001, off=0.0006),
                nut_at(P.U_START + 0.0003), nut_at(P.U_START - 0.0005)):
        fresh()
        put(nut=nut)
        for _ in range(5):
            T.step()
        assert not T.thread_engaged()
        assert T.score(None) == 0.25


def test_snug_is_half_and_short_of_snug_is_quarter() -> None:
    caught(cw_deg=345.0)
    assert T.nut_snug() and T.score(None) == 0.5
    caught(cw_deg=330.0)
    assert not T.nut_snug() and T.score(None) == 0.25


def test_open_and_held_reaches_full_only_after_the_hold() -> None:
    rod, stopper = rod_seated(22.0), stopper_at(gap=P.open_gap_for_pitch(22.0))
    caught(cw_deg=355.0, rod=rod, stopper=stopper)
    assert T.stopper_open() and T.rod_in_port()
    for k in range(P.HOLD_STEPS - 1):  # the step after the turn on is the first still one
        still(1)
        assert T.score(None) == 0.5, k
    still(1)
    assert T.score(None) == 1.0


def test_hold_resets_on_drift_near_hand_or_motion() -> None:
    rod, stopper = rod_seated(22.0), stopper_at(gap=P.open_gap_for_pitch(22.0))
    caught(cw_deg=355.0, rod=rod, stopper=stopper)
    still(5)
    assert T.STATE["hold"] >= 5
    put(rod=rod, stopper=stopper_at(gap=P.open_gap_for_pitch(22.0) + 0.0025), nut=nut_at(P.U_SNUG), cw_deg=355.0)
    T.step()
    assert T.STATE["hold"] == 0
    near = {**CLEAR, "left": (mk.pose("pivot_nut")[0] + np.array([0.0, 0.025, 0.0]), 0.0)}
    put(rod=rod, stopper=stopper, nut=nut_at(P.U_SNUG), cw_deg=355.0, grippers=near)
    T.step()
    assert T.STATE["hold"] == 0 and T.score(None) == 0.5
    for _ in range(3):
        put(rod=rod, stopper=stopper, nut=nut_at(P.U_SNUG), cw_deg=355.0)
        T.step()
    assert T.STATE["hold"] >= 2
    moved = stopper_at(gap=P.open_gap_for_pitch(22.0) + 0.0015)  # 1.5 mm in one 0.05 s step: 30 mm/s, under the 2 mm drift
    put(rod=rod, stopper=moved, nut=nut_at(P.U_SNUG), cw_deg=355.0)
    T.step()
    assert not T.at_rest() and T.STATE["hold"] == 0 and T.score(None) == 0.5


def test_open_limits() -> None:
    for gap, tilt in ((0.007, 0.0), (0.024, 0.0), (0.012, 10.0)):
        rod, stopper = rod_seated(22.0), stopper_at(gap=gap, tilt_deg=tilt)
        caught(cw_deg=355.0, rod=rod, stopper=stopper)
        assert not T.stopper_open(), (gap, tilt)


def test_turning_back_releases_and_rearms_only_once_backed_off() -> None:
    caught(cw_deg=10.0)
    put(nut=nut_at(P.U_START + P.gap_for_cw_deg(-4.0) - P.gap_for_cw_deg(0.0)), cw_deg=-4.0)
    T.step()
    assert not T.thread_engaged() and T.STATE["unthread_wait"]
    put(nut=nut_at(P.U_START - 0.0001))
    T.step()
    assert not T.thread_engaged()  # set straight back on: no catch until it has come off
    put(nut=nut_at(P.U_START + P.REARM_GAP + 0.0005))
    T.step()
    assert not T.STATE["unthread_wait"]
    put(nut=nut_at(P.U_START - 0.0001))
    T.step()
    assert T.thread_engaged()


def test_falls_zero_the_whole_trial() -> None:
    for name in ("pivot_nut", "pivot_rod"):
        fresh()
        low = {"pivot_nut": dict(nut=nut_at(0.10, z=T.TOP + 0.010)),
               "pivot_rod": dict(rod=(np.array([0.40, 0.05, T.TOP + 0.012]), EYE))}[name]
        put(**low)
        T.step()
        assert T.STATE["zero"] is not None and T.score(None) == 0.0
        rod, stopper = rod_seated(22.0), stopper_at(gap=P.open_gap_for_pitch(22.0))
        for _ in range(P.HOLD_STEPS + 2):
            put(rod=rod, stopper=stopper, nut=nut_at(P.U_SNUG), cw_deg=355.0)
            T.step()
        assert T.score(None) == 0.0


def test_lifting_the_stopper_out_is_no_zero() -> None:
    fresh()
    put(stopper=stopper_at(gap=0.060, dx=0.020))
    T.step()
    assert T.STATE["zero"] is None and T.score(None) == 0.0
    put()
    T.step()
    assert T.score(None) == 0.25
