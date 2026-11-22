"""The exposed sterile interfaces are checked against every reachable nonsterile surface, and the rack clip is physical."""

import numpy as np

import task
from asepsis import AsepsisMonitor, BoxObstacle, CylinderObstacle, segment_distance

OBSTACLES = task.FIXED_OBSTACLES + task._bag_obstacles(task.BAG_AT, task.BAG_R0)
MOUTH = task.BAG_AT + task.BAG_R0 @ task.MOUTH
INTO_BAG = task.BAG_R0[:, 1]
SPIKE_LEN = task.SPIKE_TIP - 0.039


def _observe(base, tip, port_center=MOUTH, port_open=True, spike_exposed=True, locked=False):
    m = AsepsisMonitor()
    m.observe(port_open=port_open, spike_exposed=spike_exposed, spike_locked=locked, port_center=port_center,
              port_radius=task.PORT_R, spike_base=base, spike_tip=tip, spike_radius=0.0032, gripper_surfaces=(),
              tray_top=task.TT, obstacles=OBSTACLES)
    return m


def test_every_rack_support_and_bag_surface_is_an_obstacle():
    names = {o.name.split("[")[0] for o in OBSTACLES}
    assert names == {"sterile_cradle", "iv_support", "bag"}
    rack = [o for o in OBSTACLES if o.name.startswith("sterile_cradle")]
    assert len(rack) == len([s for s in task._SPEC["sterile_cradle"].shapes if s.collide])
    support = [o for o in OBSTACLES if o.name.startswith("iv_support")]
    assert len(support) == len([s for s in task._SPEC["iv_support"].shapes if s.collide])


def test_spike_port_itself_is_not_an_obstacle():
    for d in np.linspace(0.0, task.SEAT, 11):
        tip = MOUTH + d * INTO_BAG
        m = _observe(tip - SPIKE_LEN * INTO_BAG, tip)
        assert not m.contaminated, (d, m.cause)


def test_spike_touching_the_rack_contaminates():
    post = next(o for o in OBSTACLES if o.name == "sterile_cradle[1]")
    face = post.center + post.rot @ np.array([post.half[0] + 0.003, 0.0, 0.0])
    m = _observe(face, face + np.array([0.0, 0.0, SPIKE_LEN]))
    assert m.contaminated and m.cause.startswith("spike on sterile_cradle")


def test_spike_clear_of_the_rack_is_clean():
    post = next(o for o in OBSTACLES if o.name == "sterile_cradle[1]")
    face = post.center + post.rot @ np.array([post.half[0] + 0.010, 0.0, 0.0])
    assert not _observe(face, face + np.array([0.0, 0.0, SPIKE_LEN])).contaminated


def test_spike_touching_the_iv_pole_contaminates():
    pole = next(o for o in OBSTACLES if isinstance(o, CylinderObstacle) and o.name.startswith("iv_support"))
    side = pole.center + np.array([pole.radius + 0.002, 0.0, 0.0])
    m = _observe(side, side + np.array([SPIKE_LEN, 0.0, 0.0]))
    assert m.contaminated and m.cause.startswith("spike on iv_support")


def test_open_port_touching_the_support_contaminates():
    plate = next(o for o in OBSTACLES if isinstance(o, BoxObstacle) and o.name.startswith("iv_support"))
    near = plate.center + plate.rot @ np.array([0.0, plate.half[1] + 0.004, 0.0])
    m = _observe(MOUTH + np.array([0.0, 0.0, -0.05]), MOUTH + np.array([0.0, 0.0, -0.03]), port_center=near,
                 spike_exposed=False)
    assert m.contaminated and m.cause.startswith("port on iv_support")


def test_spike_against_the_bag_face_contaminates():
    face = next(o for o in OBSTACLES if o.name == "bag[0]")
    out = face.rot @ np.array([0.0, 0.0, 1.0])  # the bag's face normal
    p = face.center + out * (face.half[2] + 0.002)
    m = _observe(p, p + np.array([SPIKE_LEN, 0.0, 0.0]))
    assert m.contaminated and m.cause.startswith("spike on bag")


def test_locked_spike_is_no_longer_checked():
    face = next(o for o in OBSTACLES if o.name == "bag[0]")
    assert not _observe(face.center, face.center + np.array([SPIKE_LEN, 0.0, 0.0]), locked=True).contaminated


def test_box_and_cylinder_distances():
    box = BoxObstacle("b", np.zeros(3), np.eye(3), np.array([0.01, 0.02, 0.03]))
    assert abs(box.distance([0.02, 0.0, 0.0]) - 0.01) < 1e-12
    assert box.distance([0.0, 0.0, 0.0]) < 0
    cyl = CylinderObstacle("c", np.zeros(3), np.array([0.0, 0.0, 1.0]), 0.01, 0.05)
    assert abs(cyl.distance([0.03, 0.0, 0.0]) - 0.02) < 1e-12
    assert abs(cyl.distance([0.0, 0.0, 0.07]) - 0.02) < 1e-12
    assert abs(segment_distance(cyl, [0.03, 0.0, -0.1], [0.03, 0.0, 0.1]) - 0.02) < 1e-9


def test_rack_clip_is_a_breakable_joint_not_a_scripted_latch():
    source = open(task.__file__).read()
    assert 'hinge("rack_clip", "sterile_cradle", "giving_set"' in source
    assert "break_force=RACK_CLIP_BREAK" in source
    assert "set_glue_on" not in source and "glue(" not in source
    weight = 0.020 * 9.81
    assert task.RACK_CLIP_BREAK > 10 * weight and task.RACK_CLIP_BREAK > 10 * task.SPIKE_CAP_BREAK
