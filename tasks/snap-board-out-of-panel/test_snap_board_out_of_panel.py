"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest snap-board-out-of-panel    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.modules.pop("yam_parts", None)
import yam_parts

TASK = load(Path(__file__).with_name("snap_board_out_of_panel.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1800


def _rot(yaw_deg, tilt_deg, axis):
    y, t = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    c, s = math.cos(t), math.sin(t)
    rt = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) if axis == "x" else np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return rz @ rt


def _half(o, r):
    if hasattr(o, "size"):
        return np.abs(r) @ (np.asarray(o.size) / 2)
    if hasattr(o, "height"):
        a = r[:, 2]
        return o.height / 2 * np.abs(a) + o.radius * np.sqrt(np.clip(1 - a * a, 0, 1))
    return np.full(3, o.radius)


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0), quat=None):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = (_rot(yaw_deg, tilt_deg, tilt_axis) if quat is None else
         ik.quat_xyzw_to_matrix(np.asarray(quat, float) / np.linalg.norm(quat)))
    if isinstance(OBJ[name], yam_parts.Part):
        origin = np.asarray(pos, float)
        low, high = OBJ[name].aabb(origin, r)
        return yam_parts.PartState((low + high) / 2, np.asarray(ik.matrix_to_quat_xyzw(r), float),
                                   np.asarray(vel, float), np.zeros(3), low, high, origin)
    pos, half = np.asarray(pos, float), _half(OBJ[name], r)
    q = ik.matrix_to_quat_xyzw(r)
    return ObjectState(pos, np.asarray(q, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def layout(poses):
    """Every object's state: the loose ones as given, the fixed ones where the task file puts them."""
    out = {o.name: place(o.name, o.pos, math.degrees(o.yaw)) for o in T.OBJECTS if o.mass is None or o.name in ('frame', 'board')}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
    'frame': {'pos': (0.205, 0.025, 0.7893)},
    'board': {'pos': (0.205, -0.02, 0.7893)},
}
GOAL_POSES = {
    'frame': {'pos': (0.205, 0, 0.7883)},
    'board': {'pos': (0.205, -0.215, 0.7523)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_HISTORY': 'VALID_HISTORY'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == set()
    assert TASK.docs and TASK.docs.strip()


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


def test_the_goal_scores_full_and_succeeds(monkeypatch):
    module_values = {}
    for name, expr in GOAL_MODULE.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    assert T.score(GOAL) == 1.0
    assert T.success(GOAL)


def test_success_waits_for_the_objects_to_settle(monkeypatch):
    module_values = {}
    for name, expr in GOAL_MODULE.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    moving = layout({n: {**p, "vel": (0.05, 0.0, 0.0)} for n, p in GOAL_POSES.items()})
    assert not T.success(moving)

# (name, the layout it starts from, what's moved, the score it should get, module overrides)
CASES = [
    ('untouched start', START_POSES, {
    }, 0, {}),
    ('separated geometry without a witnessed break', GOAL_POSES, {
    }, 0, {}),
    ('valid break board in landing nest', GOAL_POSES, {
        'board': {'pos': (0.205, -0.048, 0.7503)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('correct tray pose moving', GOAL_POSES, {
        'board': {'pos': (0.205, -0.215, 0.7523), 'vel': (0.2, 0.0, 0.0)},
    }, 0.75, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('held 20 mm above the tray floor', GOAL_POSES, {
        'board': {'pos': (0.205, -0.215, 0.7723)},
    }, 0.75, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('tray pose still held by terminal', GOAL_POSES, {
    }, 0.75, {'TEST_HISTORY': 'VALID_HISTORY', 'TEST_GRIPPERS': "{'left': (np.array([.10,.27,PARK_Z]),1.), 'right': (np.array([TRAY_X,TRAY_Y+TERMINAL_Y,TRAY_Z+.015]),0.)}"}),
    ('board drops flat 8 mm over the edge with no attached bend', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'fixture_history("no pre bend")'}),  # A flat downward departure over the edge is downward loading; the post-break window rules out twist, pull and lift.
    ('0.2 degree attached twist, then 6 degree post-break twist with a downward drop', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'fixture_history("twist")'}),  # Attached twist under the visible 1 degree limit; post-break twist 6 degrees is within the 12 degree cap and the anchor drops 3 then 6 mm.
    ('0.3 degree attached bend and twist, then a clean snap', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'fixture_history("pre twist")'}),
    ('frame resting with 1.9 mm overhang', GOAL_POSES, {
        'frame': {'pos': (0.205, -0.0019, 0.7883)},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('frame footprint 3 mm past block edge at top height', GOAL_POSES, {
        'frame': {'pos': (0.205, -0.003, 0.7883)},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('frame resting with 7 mm overhang', GOAL_POSES, {
        'frame': {'pos': (0.205, -0.007, 0.7883)},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('frame CoM 21 mm inside block', GOAL_POSES, {
        'frame': {'pos': (0.249, 0, 0.7883)},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('frame CoM only 19 mm inside block', GOAL_POSES, {
        'frame': {'pos': (0.251, 0, 0.7883)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('snap with 6 degree bend and twist', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'fixture_history("bend and twist")'}),
    ('downward attached bend then nearly flat drop', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'fixture_history("flat drop")'}),
    ('snap registered before 7 mm frame recoil', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'fixture_history("recoil")'}),
    ('frame lifted 60 mm above block', GOAL_POSES, {
        'frame': {'pos': (0.205, 0, 0.8483)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('frame off block but less than 5 mm below top', GOAL_POSES, {
        'frame': {'pos': (0.205, 0.2, 0.7834)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('red team: valid snap after a 0.02 degree yaw transient during registration', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': '(lambda z,r: [history_sample(0,0.,z[0],_state(BOARD,z[0].origin+np.array([.045*math.sin(math.radians(.02)),-.045*math.cos(math.radians(.02)),0.]),(0.,0.,math.sin(math.radians(.01)),math.cos(math.radians(.01)))))]+[history_sample(i+1,(i+1)*CONTROL_DT,r[i][0],r[i][1]) for i in (0,1,2)])(bend_states(0.),[bend_states(.3),bend_states(8.,0.,(0,0,-.003)),bend_states(18.,0.,(0,0,-.006))])'}),  # A 0.02 degree relative yaw (0.016 mm at the board tip) while the panel slides is handling noise, not torsional loading; the break itself is a clean downward snap and the board is trayed, so it deserves full credit.
    ('red team: valid snap after a 0.02 degree upward sag while the panel is handled', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': '(lambda u,r: [history_sample(0,0.,u[0],u[1])]+[history_sample(i+1,(i+1)*CONTROL_DT,r[i][0],r[i][1]) for i in (0,1,2)])(bend_states(-.02),[bend_states(.3),bend_states(8.,0.,(0,0,-.003)),bend_states(18.,0.,(0,0,-.006))])'}),  # 0.02 degrees of upward relative bend (0.016 mm at the tip) earlier in handling can't be seen or measured and doesn't load the V-score; the break is downward over the edge and the board is trayed.
    ('red team: frame set down flat on the bench beside the block, board perfectly trayed', GOAL_POSES, {
        'frame': {'pos': (0.08, 0.02, 0.7483)},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),  # The instruction and context only ask to separate the board and tray it; the spent frame is scrap, set down flat and still on the bench. Holding it in the air instead scores 0.5, so setting it down neatly scores less than leaving it held.
    ('frame set flat on the bench but still sliding', GOAL_POSES, {
        'frame': {'pos': (0.08, 0.02, 0.7483), 'vel': (0.2, 0.0, 0.0)},
    }, 0.75, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('valid snap after a 0.9 degree upward sag while attached', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': '(lambda u,r: [history_sample(0,0.,u[0],u[1])]+[history_sample(i+1,(i+1)*CONTROL_DT,r[i][0],r[i][1]) for i in (0,1,2)])(bend_states(-.9),[bend_states(.3),bend_states(8.,0.,(0,0,-.003)),bend_states(18.,0.,(0,0,-.006))])'}),
    ('frame held in the air beside the block 5.1 mm below its top', GOAL_POSES, {
        'frame': {'pos': (0.205, 0.2, 0.7832)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),  # The frame is neither flat on the block nor flat on the bench, so the trayed board earns only the separation tier.
    ('frame held tilted 16 degrees above the block', GOAL_POSES, {
        'frame': {'pos': (0.205, 0, 0.8183), 'tilt_deg': 16, 'tilt_axis': 'x'},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),  # Frame not resting flat on a surface; separation credit only.
    ('frame put in the tray, board left on the block', START_POSES, {
        'frame': {'pos': (0.205, -0.215, 0.7523)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),  # The witnessed valid break earns 0.5; the frame in the tray earns nothing (it is not the board, and a frame over the tray outline doesn't count as set down).
    ('red team: scrap frame flat on the bench at 45 degrees yaw, 33 mm clear of the tray corner (variant A)', GOAL_POSES, {
        'frame': {'pos': (0.1117, -0.1197, 0.7483), 'yaw_deg': 45},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),  # The scrap frame lies flat and still on the bench and no part of it is over the tray's 140 x 144 mm outline (nearest point about 33 mm from the tray corner); only its world-aligned corner box overlaps the outline. The text allows a frame laid flat anywhere on the bench outside the tray outline.
    ("red team: scrap frame flat on the bench at 30 degrees yaw beside the tray's -x side, 6 mm clear of the outline", GOAL_POSES, {
        'frame': {'pos': (0.11, -0.15, 0.7483), 'yaw_deg': 30},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),  # Frame flat on the bench, its footprint clear of the tray outline; only its world-aligned corner box crosses it.
    ('scrap frame flat on the bench at -45 degrees yaw, 33 mm clear of the other tray corner', GOAL_POSES, {
        'frame': {'pos': (0.2983, -0.1197, 0.7483), 'yaw_deg': -45},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('frame still clamped on the block by the left gripper, board seated and released', GOAL_POSES, {
    }, 0.75, {'TEST_HISTORY': 'VALID_HISTORY', 'TEST_GRIPPERS': "{'left': (np.array([X-.010,.130,BLOCK_TOP+.030]),0.), 'right': (np.array([.10,-.30,PARK_Z]),1.)}"}),  # A frame still in a gripper but inside the 0.75 limits (flat on the block) earns 0.75, as the scoring text states.
    ('frame held 3 mm above the bench by the left gripper, board seated and released', GOAL_POSES, {
        'frame': {'pos': (0.08, 0.02, 0.7513)},
    }, 0.75, {'TEST_HISTORY': 'VALID_HISTORY', 'TEST_GRIPPERS': "{'left': (np.array([.08,.15,TOP+.035]),0.), 'right': (np.array([.10,-.30,PARK_Z]),1.)}"}),  # Every frame corner is within the 4 mm bench band, so the held frame meets the 0.75 limits; not released, so not 1.0.
    ('red team: clean snap whose last attached sample shows 0.10 deg bend and 0.10 deg twist', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': '(lambda h,f: (lambda q: [history_sample(0,0.,f,_state(BOARD,f.origin+_state(BOARD,(0,0,0),q).rotation()@np.array([0.,-BOARD_D/2,0.]),q))]+[history_sample(i,i*CONTROL_DT,*bend_states(a,0.,(0,0,-.003*i))) for i,a in ((1,8.),(2,18.))])((math.sin(h)*math.cos(h),-math.sin(h)**2,math.cos(h)*math.sin(h),math.cos(h)**2)))(math.radians(.05),bend_states(0.)[0])'}),  # A 0.1 degree twist is 0.16 mm at the board's tip, invisible on video and far under the 1 degree limit the text calls visible; the break itself is a clean 8-18 degree downward snap over the edge, and the board is trayed.
    ('red team: clean downward snap completed between two 20 Hz samples (last attached sample at rest)', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': '[history_sample(0,0.,*bend_states(.003))]+[history_sample(i,i*CONTROL_DT,*bend_states(a,0.,(0,0,-.003*i))) for i,a in ((1,8.),(2,18.))]'}),  # A firm press that fractures the web within one 50 ms control step leaves the last attached sample at rest; the break is downward (8 then 18 degrees, anchor 3-6 mm below the score line) over the edge with no twist or pull. Scoring it 0 depends on sampling phase, not on the skill.
    ('frame sunk 3 mm into the block top, board seated', GOAL_POSES, {
        'frame': {'pos': (0.205, 0, 0.7853)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),  # The supported underside is 3 mm below the block top, outside the -2 to +4 mm band; a real frame cannot sink into steel.
    ('frame 1.5 mm into the block top (resting contact), board seated', GOAL_POSES, {
        'frame': {'pos': (0.205, 0, 0.7868)},
    }, 1, {'TEST_HISTORY': 'VALID_HISTORY'}),  # Within the 2 mm resting-contact penetration allowed at the block top.
    ('red team: snap made in mid-air 120 mm beside the block, then frame returned and board trayed', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': '(lambda s,r: [history_sample(i,i*CONTROL_DT,_state(FRAME,r[i][0].origin+s),_state(BOARD,r[i][1].origin+s,r[i][1].quat_xyzw)) for i in (0,1,2)])(np.array([-.12,0.,0.]),[bend_states(.3),bend_states(8.,0.,(0,0,-.003)),bend_states(18.,0.,(0,0,-.006))])'}),  # The score line (x 45-125 mm) is beside the block's edge (x 140-270 mm).
    ('red team: snap with 65 mm of the score line hanging past the block side', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': '(lambda s,r: [history_sample(i,i*CONTROL_DT,_state(FRAME,r[i][0].origin+s),_state(BOARD,r[i][1].origin+s,r[i][1].quat_xyzw)) for i in (0,1,2)])(np.array([-.090,0.,0.]),[bend_states(.3),bend_states(8.,0.,(0,0,-.003)),bend_states(18.,0.,(0,0,-.006))])'}),  # One score-line endpoint (x 75 mm) is off the block's edge (x 140-270 mm).
    ('upward bend of 1.2 degrees while attached, then a clean snap', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': '(lambda u,r: [history_sample(0,0.,u[0],u[1])]+[history_sample(i+1,(i+1)*CONTROL_DT,r[i][0],r[i][1]) for i in (0,1,2)])(bend_states(-1.2),[bend_states(.3),bend_states(8.,0.,(0,0,-.003)),bend_states(18.,0.,(0,0,-.006))])'}),  # Visible upward bend (over 1 degree) loads the V-score the wrong way; sticky.
    ('twist of 1.2 degrees while attached, then a clean snap', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': '(lambda z,r: [history_sample(0,0.,z[0],_state(BOARD,z[0].origin+np.array([.045*math.sin(math.radians(1.2)),-.045*math.cos(math.radians(1.2)),0.]),(0.,0.,math.sin(math.radians(.6)),math.cos(math.radians(.6)))))]+[history_sample(i+1,(i+1)*CONTROL_DT,r[i][0],r[i][1]) for i in (0,1,2)])(bend_states(0.),[bend_states(.3),bend_states(8.,0.,(0,0,-.003)),bend_states(18.,0.,(0,0,-.006))])'}),  # Visible twist (over 1 degree) while attached; sticky.
    ('frame dropped on the floor, board trayed', GOAL_POSES, {
        'frame': {'pos': (0.08, 0.02, 0.02)},
    }, 0, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('valid downward load but score line 2.8 mm past edge', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'fixture_history("2.8 mm registration")'}),
    ('downward departure stops short of 2 mm anchor drop', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'fixture_history("no anchor drop")'}),
    ('panel trayed whole', START_POSES, {
        'frame': {'pos': (0.205, -0.17, 0.7523)},
        'board': {'pos': (0.205, -0.215, 0.7523)},
    }, 0, {}),  # No break and the frame is off the block.
    ('board flipped in tray', GOAL_POSES, {
        'board': {'pos': (0.205, -0.215, 0.7673), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('board stood on end in tray', GOAL_POSES, {
        'board': {'pos': (0.205, -0.215, 0.7965), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('board on tray rim', GOAL_POSES, {
        'board': {'pos': (0.271, -0.215, 0.7643)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('board stacked above floor', GOAL_POSES, {
        'board': {'pos': (0.205, -0.215, 0.7873)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('yawed corner overhangs tray', GOAL_POSES, {
        'board': {'pos': (0.235, -0.215, 0.7523), 'yaw_deg': 45},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('pull break then perfect tray', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'fixture_history("pull")'}),
    ('twist 13 degrees exceeds absolute cap', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'fixture_history("twist cap")'}),
    ('registered only after fracture', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'fixture_history("late registration")'}),
    ('bend without downward board motion', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'fixture_history("no downward motion")'}),
    ('upward bend then perfect tray', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'fixture_history("up")'}),
    ('score line 5 mm off edge then perfect tray', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'fixture_history("off edge")'}),
    ('board on floor', GOAL_POSES, {
        'board': {'pos': (0.205, -0.215, 0.02)},
    }, 0, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('undone separation', START_POSES, {
        'frame': {'pos': (0.205, 0, 0.7883)},
        'board': {'pos': (0.205, -0.045, 0.7883)},
    }, 0, {'TEST_HISTORY': 'VALID_HISTORY'}),
    ('frame hovering 20 mm above block', GOAL_POSES, {
        'frame': {'pos': (0.205, 0, 0.8083)},
    }, 0.5, {'TEST_HISTORY': 'VALID_HISTORY'}),
]


@pytest.mark.parametrize(("name", "base", "moved", "expected", "module"), CASES, ids=[c[0] for c in CASES])
def test_named_layouts(name, base, moved, expected, module, monkeypatch):
    module_values = {}
    for name, expr in module.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    objects = layout({**base, **moved})
    assert T.score(objects) == pytest.approx(expected), name
    assert bool(T.success(objects)) == (expected == 1.0), name


def test_the_demo_is_reachable_within_the_joint_limits():
    plan = TASK.demo()
    assert set(plan) <= {"left", "right"}
    for side, waypoints in plan.items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = round(arm.duration * 20) + 5
        actions = np.array([arm.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(actions[:, :6] >= ik.LOWER - 1e-6) and np.all(actions[:, :6] <= ik.UPPER + 1e-6), side


def test_the_demo_fits_in_the_run_limit():
    longest = max(ArmPlan(w, station.BASES[s]).duration for s, w in TASK.demo().items())
    assert (longest + TASK.success_hold_s) * 20 < RUN_LIMIT_STEPS * 0.6
