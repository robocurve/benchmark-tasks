from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest
from yam_arena.task_file import ObjectState
from geometry import FILM_NAMES, N_BONDED, PITCH, CARRIER_POS, SIZES, TABLE_Z, TAB_POS, TAB_QUAT, corners, strip_pos
from scoring import Trial
from tool_geometry import tool_objects,TOOL_NAMES,ALL_TOOL_NAMES,TOOL_SIZE,returned

TOOL_SPECS={o.name:o for o in tool_objects()}


def state(name, pos, quat=(0., 0., 0., 1.)):
    s = ObjectState(np.array(pos), np.array(quat), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
    size=SIZES[name] if name in SIZES else TOOL_SIZE if name in ALL_TOOL_NAMES else TOOL_SPECS[name].size
    c = corners(s, size)
    return ObjectState(s.pos, s.quat_xyzw, s.lin_vel, s.ang_vel, c.min(0), c.max(0))


def initial():
    tools={n:state(n,np.array(o.pos)+([0,-.0023,0] if n in ALL_TOOL_NAMES and n.endswith('_a') else
                                    [0,.0023,0] if n in ALL_TOOL_NAMES and n.endswith('_b') else [0,0,0]))
           for n,o in TOOL_SPECS.items()}
    return dict(carrier=state('carrier', CARRIER_POS),**tools,
                pull_tab=state('pull_tab', TAB_POS, TAB_QUAT),
                **{n: state(n, strip_pos(i)) for i, n in enumerate(FILM_NAMES[1:])})


def completed():
    o = initial()
    for i, name in enumerate(FILM_NAMES):
        o[name] = state(name, [.28, -.28-i*.01, TABLE_Z+SIZES[name][2]/2])
    return o


def settle(t, o, steps=20):
    for _ in range(steps):
        t.observe(o, hands_clear=True)
        t.finish_step(o, .05)


@pytest.mark.parametrize('count,expected', [(0,0), (5,0), (6,.25), (11,.25), (12,.5), (17,.5), (18,.75), (22,.75), (23,.75)])
def test_partial_credit(count, expected):
    t = Trial(); o = initial()
    t.observe(o, hands_clear=True); t.peeled = count
    assert t.score(o) == expected


def test_full_score_requires_disposal_and_withdrawal():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    t.observe(o, hands_clear=False)
    assert t.score(o) == .75
    t.observe(o, hands_clear=True)
    settle(t, o)
    assert t.score(o) == 1.


def test_hovering_is_not_disposal():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    for name in FILM_NAMES:
        o[name] = state(name, o[name].pos + [0,0,.02])
    t.observe(o, hands_clear=True)
    assert t.score(o) == .75


def test_precreased_tab_may_remain_raised_on_disposed_sheet():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    o['pull_tab'] = state('pull_tab', [.28, -.28, TABLE_Z+.011], TAB_QUAT)
    t.observe(o, hands_clear=True)
    settle(t, o)
    assert t.score(o) == 1.


def test_centre_outside_but_edge_overlaps():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    o['film_00'] = state('film_00', [.35, -.10, TABLE_Z+.0005])
    t.observe(o, hands_clear=True)
    assert t.score(o) == .75


@pytest.mark.parametrize('shift', [[.031,0,0], [0,0,.006]])
def test_plate_excursion_cannot_be_undone(shift):
    t = Trial(); o = completed(); t.peeled = N_BONDED
    o['carrier'] = state('carrier', CARRIER_POS + shift)
    t.observe(o, hands_clear=True)
    o['carrier'] = state('carrier', CARRIER_POS)
    t.observe(o, hands_clear=True)
    assert t.score(o) == 0.
    t.reset(); t.peeled = N_BONDED; t.observe(o, hands_clear=True)
    settle(t, o)
    assert t.score(o) == 1.


def test_timeout_before_one_second_does_not_receive_full_credit():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    settle(t, o, 19)
    assert t.score(o) == .75
    # Reading the scorer repeatedly must not advance the hold timer.
    assert t.score(o) == .75
    settle(t, o, 1)
    assert t.score(o) == 1.


def test_disrupted_settling_restarts_full_second():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    settle(t, o, 19)
    t.observe(o, hands_clear=False); t.finish_step(o, .05)
    settle(t, o, 19)
    assert t.score(o) == .75
    settle(t, o, 1)
    assert t.score(o) == 1.
    t.reset()
    assert t.settled_seconds == 0.


def test_invalid_state_fails_closed():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    o['film_00'].pos[0] = float('nan')
    t.observe(o, hands_clear=True)
    assert t.score(o) == 0.


def test_invalid_quaternion_fails_closed():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    o['film_00'].quat_xyzw[:] = 0
    t.observe(o, hands_clear=True)
    assert t.score(o) == 0.


@pytest.mark.parametrize('axis', [0, 2])
def test_plate_rotation_excursion_is_latched(axis):
    t = Trial(); o = initial()
    q = np.zeros(4); q[axis] = np.sin(np.deg2rad(11)/2); q[3] = np.cos(np.deg2rad(11)/2)
    o['carrier'] = state('carrier', CARRIER_POS, q)
    t.observe(o, hands_clear=True)
    t.peeled = 7; t.observe(initial(), hands_clear=True)
    assert t.score(initial()) == 0.


def test_exact_translation_limit_is_allowed():
    t = Trial(); o = initial(); t.peeled = round(.070 / PITCH)  # 70 mm
    o['carrier'] = state('carrier', CARRIER_POS + [.030,0,0])
    t.observe(o, hands_clear=True)
    assert t.score(o) == .5


def test_released_count_does_not_depend_on_current_position():
    t = Trial(); t.peeled = round(.070 / PITCH)  # 70 mm
    t.observe(initial(), hands_clear=True)
    assert t.score(initial()) == .5


def test_film_below_table_rejected():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    o['film_00'] = state('film_00', [.2, -.3, TABLE_Z-.01])
    t.observe(o, hands_clear=True)
    assert t.score(o) == .75


@pytest.mark.parametrize('shift', [[.006,0,0],[0,0,.006]])
def test_tool_return_is_required_for_full_score(shift):
    t=Trial();o=completed();t.peeled=N_BONDED
    for name in TOOL_NAMES:o[name]=state(name,o[name].pos+shift)
    assert not returned(o)
    settle(t,o)
    assert t.score(o)==.75


def test_tool_must_be_open_released_and_stationary():
    t=Trial();o=completed();t.peeled=N_BONDED
    assert returned(o)
    o['tweezer_b']=state('tweezer_b',o['tweezer_b'].pos+[0,-.004,0])
    assert not returned(o)
    settle(t,o)
    assert t.score(o)==.75
    o=completed();o['tweezer_a'].lin_vel[0]=.011
    settle(t,o)
    assert t.score(o)==.75


def test_invalid_tool_state_fails_closed():
    t=Trial();o=completed();t.peeled=N_BONDED
    o['tweezer_a'].pos[0]=np.nan
    t.observe(o,hands_clear=True)
    assert t.score(o)==0


def test_decoy_tool_must_also_be_back_in_its_stand():
    t=Trial();o=completed();t.peeled=N_BONDED
    assert returned(o)
    for name in ('paddle_a','paddle_b'):o[name]=state(name,o[name].pos+[-.06,0,0])
    assert not returned(o)
    settle(t,o)
    assert t.score(o)==.75


def test_only_fine_tips_pass_the_tab_notch():
    from fixture_geometry import tab_parts
    from tool_geometry import TOOLS
    bridges=[p for p in tab_parts() if p.name.startswith('bridge')]
    notch=min(abs(p.center[0])-p.size[0]/2 for p in bridges)*2
    assert TOOLS['fine']['blade'][0] < notch < TOOLS['paddle']['blade'][0]


def test_solver_velocity_on_a_still_folded_sheet_does_not_block_credit():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    # Measured on the folded 5 mm strips: constant velocity readings with fixed poses.
    o['film_03'].lin_vel[:] = [0, .0123, 0]; o['film_04'].ang_vel[:] = [2.8, 0, 0]
    settle(t, o)
    assert t.score(o) == 1.


def test_film_moving_during_the_hold_restarts_the_full_second():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    settle(t, o, 10)
    o['film_05'] = state('film_05', o['film_05'].pos + [0, -.003, 0])
    settle(t, o, 19)
    assert t.score(o) == .75
    settle(t, o, 1)
    assert t.score(o) == 1.


def test_turning_film_piece_restarts_the_full_second():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    settle(t, o, 10)
    a = np.deg2rad(6)
    o['film_07'] = state('film_07', o['film_07'].pos, (np.sin(a/2), 0, 0, np.cos(a/2)))
    settle(t, o, 19)
    assert t.score(o) == .75


def test_slowly_creeping_film_never_settles():
    t = Trial(); o = completed(); t.peeled = N_BONDED
    for _ in range(60):  # 3 mm/s for 3 s
        o['film_09'] = state('film_09', o['film_09'].pos + [0, -.00015, 0])
        settle(t, o, 1)
        assert t.score(o) == .75
