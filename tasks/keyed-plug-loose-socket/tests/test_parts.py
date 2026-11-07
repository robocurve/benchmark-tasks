"""Offline tests for parts.py: no Isaac Sim, no GPU.

    python -m unittest discover -s tasks/keyed-plug-loose-socket/tests -v

They need numpy and the pure-Python part of yam_arena (installed, or its source found through $YAM_ARENA_SRC or next
to this repository in work/refs/yam-isaac-lab-arena). The USD tests need ``pxr`` (``pip install usd-core``) and are
skipped without it.
"""

from __future__ import annotations

import contextlib
import dataclasses
import enum
import functools
import importlib
import json
import os
import subprocess
import sys
import tempfile
import textwrap
import types
import unittest
from pathlib import Path
from unittest import mock

sys.dont_write_bytecode = True  # keep __pycache__ out of the yam_arena checkout and this folder

TASK_DIR = Path(__file__).resolve().parents[1]


def _locate_yam_arena() -> None:
    try:
        import yam_arena  # noqa: F401

        return
    except ImportError:
        pass
    candidates = [Path(os.environ["YAM_ARENA_SRC"])] if os.environ.get("YAM_ARENA_SRC") else []
    candidates.append(TASK_DIR.parents[2] / "work" / "refs" / "yam-isaac-lab-arena")  # <Robocurve>/work/refs/...
    for root in candidates:
        if (root / "yam_arena" / "task_file.py").is_file():
            sys.path.append(str(root))
            return
    raise unittest.SkipTest("yam_arena is not importable; install it or set YAM_ARENA_SRC to its source directory")


_locate_yam_arena()
sys.path.insert(0, str(TASK_DIR))

import numpy as np  # noqa: E402
import yam_arena  # noqa: E402
from yam_arena import task_file  # noqa: E402
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN  # noqa: E402

import parts  # noqa: E402
from parts import Part, PartState, Slider, gripper_open, on_reset, on_step, part  # noqa: E402

try:
    import pxr  # noqa: F401
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

    HAVE_PXR = True
except ImportError:
    HAVE_PXR = False

GREY, DARK, RED, STEEL = (0.5, 0.5, 0.5), (0.1, 0.1, 0.1), (0.85, 0.15, 0.12), (0.78, 0.79, 0.8)
NO_PXR = "pxr is not installed (pip install usd-core; it has no wheel for every Python)"


def sample_part(name="block", yaw=0.0, with_slider=True, **kw) -> Part:
    sliders = []
    if with_slider:
        sliders = [
            Slider("plunger", rest=(-0.015, -0.04, 0.0), axis="X", travel=0.0015, rate=12000.0, damping=40.0,
                   mass=0.002,
                   cylinders=[((-0.015, -0.04, 0.0), 0.002, 0.006, STEEL, "Z")],
                   boxes=[((-0.015, -0.04, 0.004), (0.002, 0.002, 0.002), STEEL)])
        ]
    return part(name, pos=(0.28, 0.10, 0.78), yaw=yaw, mass=0.30, friction=0.4, sliders=sliders,
                boxes=[((0, 0, 0), (0.085, 0.11, 0.06), GREY), ((0.02, -0.05, 0), (0.01, 0.02, 0.03), DARK)],
                cylinders=[((0, 0.03, 0.03), 0.01, 0.02, RED, "Y")], **kw)


# -- Part ----------------------------------------------------------------------------------------------------------


class PartTests(unittest.TestCase):
    def test_a_part_is_a_free_box_for_yam_arenas_checks(self):
        p = sample_part()
        self.assertIsInstance(p, task_file.Box)
        self.assertTrue(p.free)
        p._check()
        hash(p)  # frozen and hashable

    def test_part_is_frozen(self):
        with self.assertRaises(dataclasses.FrozenInstanceError):
            sample_part().mass = 1.0

    def test_size_is_the_bounding_box_of_boxes_and_cylinders(self):
        p = part("p", (0, 0, 0), mass=0.1,
                 boxes=[((0.1, 0, 0), (0.2, 0.1, 0.1), GREY),  # x 0..0.2, y -.05..0.05, z -.05..0.05
                        ((-0.05, 0.1, 0), (0.1, 0.1, 0.2), GREY)],  # x -.1..0, y .05..0.15, z -.1..0.1
                 cylinders=[((0, -0.1, 0.05), 0.02, 0.3, RED, "X")])  # x -.15..0.15, y -.12..-.08, z .03..0.07
        np.testing.assert_allclose(p.size, (0.35, 0.27, 0.2), atol=1e-12)
        lo, hi = p.local_bounds()
        np.testing.assert_allclose(lo, (-0.15, -0.12, -0.1), atol=1e-12)
        np.testing.assert_allclose(hi, (0.2, 0.15, 0.1), atol=1e-12)

    def test_every_cylinder_axis_and_the_default(self):
        box = ((0, 0, 0), (0.01, 0.01, 0.01), GREY)
        for axis, expected in (("X", (0.3, 0.04, 0.04)), ("Y", (0.04, 0.3, 0.04)), ("Z", (0.04, 0.04, 0.3))):
            p = part("p", (0, 0, 0), [box], 0.1, cylinders=[((0, 0, 0), 0.02, 0.3, RED, axis)])
            np.testing.assert_allclose(p.size, expected, atol=1e-12, err_msg=axis)
        p = part("p", (0, 0, 0), [box], 0.1, cylinders=[((0, 0, 0), 0.02, 0.3, RED)])  # four entries: axis Z
        np.testing.assert_allclose(p.size, (0.04, 0.04, 0.3), atol=1e-12)
        self.assertEqual(p.cylinders[0][4], "Z")

    def test_slider_shapes_count_toward_the_bounding_box(self):
        slider = Slider("tip", rest=(0.08, 0, 0), boxes=[((0.08, 0, 0), (0.02, 0.01, 0.01), RED)])
        p = part("p", (0, 0, 0), [((0, 0, 0), (0.1, 0.1, 0.1), GREY)], 0.1, sliders=[slider])
        np.testing.assert_allclose(p.size, (0.14, 0.1, 0.1), atol=1e-12)

    def test_check_needs_boxes(self):
        p = part("p", (0, 0, 0), [], 0.1, cylinders=[((0, 0, 0), 0.02, 0.3, RED)])
        with self.assertRaisesRegex(ValueError, "at least one box"):
            p._check()

    def test_check_needs_a_positive_mass(self):
        box = [((0, 0, 0), (0.1, 0.1, 0.1), GREY)]
        for mass in (None, 0, -1.0, float("nan")):
            with self.subTest(mass=mass), self.assertRaisesRegex(ValueError, "mass"):
                part("p", (0, 0, 0), box, mass)._check()

    def test_check_rejects_bad_shapes_and_settings(self):
        box = ((0, 0, 0), (0.1, 0.1, 0.1), GREY)
        bad = {
            "zero box size": dict(boxes=[((0, 0, 0), (0.1, 0.0, 0.1), GREY)]),
            "colour out of range": dict(boxes=[((0, 0, 0), (0.1, 0.1, 0.1), (1.5, 0, 0))]),
            "bad cylinder axis": dict(boxes=[box], cylinders=[((0, 0, 0), 0.01, 0.02, RED, "W")]),
            "zero cylinder radius": dict(boxes=[box], cylinders=[((0, 0, 0), 0.0, 0.02, RED)]),
            "friction": dict(boxes=[box], friction=0.0),
            "name": dict(boxes=[box], name="not an identifier"),
            "slider without shapes": dict(boxes=[box], sliders=[Slider("s", (0, 0, 0))]),
            "slider axis": dict(boxes=[box], sliders=[Slider("s", (0, 0, 0), boxes=[box], axis="Q")]),
            "slider travel": dict(boxes=[box], sliders=[Slider("s", (0, 0, 0), boxes=[box], travel=0.0)]),
            "slider rate": dict(boxes=[box], sliders=[Slider("s", (0, 0, 0), boxes=[box], rate=-1.0)]),
            "slider mass": dict(boxes=[box], sliders=[Slider("s", (0, 0, 0), boxes=[box], mass=0.0)]),
            "slider name clashes with the body prim": dict(boxes=[box], sliders=[Slider("body", (0, 0, 0), boxes=[box])]),
            "slider name is not a prim name": dict(boxes=[box], sliders=[Slider("a-b", (0, 0, 0), boxes=[box])]),
            "duplicate sliders": dict(boxes=[box], sliders=[Slider("s", (0, 0, 0), boxes=[box])] * 2),
        }
        for label, kw in bad.items():
            kw = {"name": "p", "pos": (0, 0, 0), "mass": 0.1, **kw}
            with self.subTest(label), self.assertRaises(ValueError):
                part(**kw)._check()

    def test_malformed_shape_specs_fail_at_construction(self):
        with self.assertRaises(ValueError):
            part("p", (0, 0, 0), [((0, 0), (0.1, 0.1, 0.1), GREY)], 0.1)
        with self.assertRaises(ValueError):
            part("p", (0, 0, 0), [((0, 0, 0), (0.1, 0.1, 0.1))], 0.1)

    def test_axes_are_case_insensitive(self):
        s = Slider("s", (0, 0, 0), boxes=[((0, 0, 0), (0.1, 0.1, 0.1), GREY)], axis="x")
        self.assertEqual(s.axis, "X")

    def test_the_task_file_loader_accepts_parts(self):
        source = textwrap.dedent(f"""
            import sys
            sys.path.insert(0, {str(TASK_DIR)!r})
            from parts import part
            SCENE = "station"
            OBJECTS = [part("block", (0.28, 0.1, 0.78), [((0, 0, 0), (0.08, 0.1, 0.06), (0.6, 0.6, 0.6))], mass=0.3),
                       part("loose_cap", (0.2, 0.0, 0.78), [((0, 0, 0), (0.02, 0.02, 0.02), (0.6, 0.6, 0.6))], mass=None)]
            def success(objects): return True
        """)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "fake_task.py"
            path.write_text(source)
            # a Part with no mass is refused by the loader's _check
            with self.assertRaisesRegex(ValueError, "mass"):
                task_file.load(path)
            path.write_text(source.replace(", mass=None", ", mass=0.01"))
            loaded = task_file.load(path)
        self.assertEqual(loaded.names, ("block", "loose_cap"))
        self.assertEqual(loaded.free_objects, ("block", "loose_cap"))
        self.assertIsInstance(loaded.objects[0], Part)


# -- PartState -----------------------------------------------------------------------------------------------------


def yaw_quat(yaw):
    return np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])


def state(pos=(0, 0, 0), quat=(0, 0, 0, 1)) -> PartState:
    return PartState(pos=np.array(pos, float), quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3),
                     ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))


class PartStateTests(unittest.TestCase):
    def test_is_an_object_state(self):
        s = state()
        self.assertIsInstance(s, task_file.ObjectState)
        self.assertEqual(s.sliders, {})
        self.assertTrue(s.resting())

    def test_rotation_of_known_yaws(self):
        np.testing.assert_allclose(state().rotation(), np.eye(3), atol=1e-12)
        r = state(quat=yaw_quat(np.pi / 2)).rotation()
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(r @ [0, 1, 0], [-1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(r @ [0, 0, 1], [0, 0, 1], atol=1e-12)

    def test_rotation_ignores_quaternion_scale_and_is_orthonormal(self):
        rng = np.random.default_rng(0)
        for _ in range(20):
            q = rng.normal(size=4)
            r = state(quat=q * 3.0).rotation()
            np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(r), 1.0)
            np.testing.assert_allclose(r, state(quat=q / np.linalg.norm(q)).rotation(), atol=1e-12)

    def test_zero_quaternion_is_an_error(self):
        with self.assertRaises(ValueError):
            state(quat=(0, 0, 0, 0)).rotation()

    def test_to_world_and_to_local_with_a_yaw(self):
        s = state(pos=(1.0, 2.0, 3.0), quat=yaw_quat(np.pi / 2))
        np.testing.assert_allclose(s.to_world([1, 0, 0]), [1.0, 3.0, 3.0], atol=1e-12)  # local +x is world +y
        np.testing.assert_allclose(s.to_world([0, 1, 0.5]), [0.0, 2.0, 3.5], atol=1e-12)
        np.testing.assert_allclose(s.to_local([1.0, 3.0, 3.0]), [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(s.to_local(s.pos), [0, 0, 0], atol=1e-12)

    def test_round_trips_for_points_and_stacks(self):
        rng = np.random.default_rng(1)
        for yaw in (0.0, 0.7, -2.5, np.pi):
            s = state(pos=rng.normal(size=3), quat=yaw_quat(yaw))
            p = rng.normal(size=3)
            np.testing.assert_allclose(s.to_local(s.to_world(p)), p, atol=1e-12)
            np.testing.assert_allclose(s.to_world(s.to_local(p)), p, atol=1e-12)
            stack = rng.normal(size=(5, 3))
            np.testing.assert_allclose(s.to_local(s.to_world(stack)), stack, atol=1e-12)
            np.testing.assert_allclose(s.to_world(stack)[2], s.to_world(stack[2]), atol=1e-12)
        s = state(quat=rng.normal(size=4))  # a general rotation too
        p = rng.normal(size=3)
        np.testing.assert_allclose(s.to_local(s.to_world(p)), p, atol=1e-12)

    def _spec(self):
        # local box corners: x -.03..0.05, y -.05..0.05, z -.03..0.06 (the bounding box is not centred on the origin)
        return part("p", (0, 0, 0), mass=0.1,
                    boxes=[((0.01, 0, 0), (0.08, 0.1, 0.06), GREY), ((0, 0, 0.03), (0.02, 0.02, 0.06), GREY)],
                    sliders=[Slider("a", (0, 0, 0), boxes=[((0, 0, 0), (0.01, 0.01, 0.01), RED)]),
                             Slider("b", (0, 0, 0), boxes=[((0, 0, 0), (0.01, 0.01, 0.01), RED)])])

    def test_low_and_high_are_the_world_box_around_the_bounding_box_corners(self):
        spec = self._spec()
        origin = np.array([10.0, 20.0, 0.0])
        for yaw in (0.0, np.pi / 2, np.pi / 4, -1.0):
            pose = np.array([11.0, 22.0, 0.8, *yaw_quat(yaw)])
            st = parts._state_from_arrays(spec, pose, np.zeros(6), origin, ["a_slide", "b_slide"], [0.0, 0.0])
            lo, hi = spec.local_bounds()
            corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
            c, s = np.cos(yaw), np.sin(yaw)
            world = np.array([[c * x - s * y, s * x + c * y, z] for x, y, z in corners]) + [1.0, 2.0, 0.8]
            np.testing.assert_allclose(st.low, world.min(0), atol=1e-9, err_msg=str(yaw))
            np.testing.assert_allclose(st.high, world.max(0), atol=1e-9, err_msg=str(yaw))
        # a quarter turn, by hand: local (x, y, z) -> world (-y, x, z)
        pose = np.array([0.0, 0.0, 0.0, *yaw_quat(np.pi / 2)])
        st = parts._state_from_arrays(spec, pose, np.zeros(6), np.zeros(3), ["a_slide", "b_slide"], [0, 0])
        np.testing.assert_allclose(st.low, (-0.05, -0.03, -0.03), atol=1e-9)
        np.testing.assert_allclose(st.high, (0.05, 0.05, 0.06), atol=1e-9)

    def test_state_fields_and_slider_lookup_by_joint_name(self):
        spec = self._spec()
        pose = np.array([1.5, 2.5, 0.8, *yaw_quat(0.3)])
        vel = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        st = parts._state_from_arrays(spec, pose, vel, [1.0, 2.0, 0.0], ["x", "b_slide", "y", "a_slide"],
                                      [9.0, -0.002, 9.0, -0.0005])
        np.testing.assert_allclose(st.pos, (0.5, 0.5, 0.8))
        np.testing.assert_allclose(st.quat_xyzw, yaw_quat(0.3))
        np.testing.assert_allclose(st.lin_vel, vel[:3])
        np.testing.assert_allclose(st.ang_vel, vel[3:])
        self.assertEqual(st.sliders, {"a": -0.0005, "b": -0.002})

    def test_a_missing_slider_joint_is_an_error(self):
        with self.assertRaisesRegex(RuntimeError, "a_slide"):
            parts._state_from_arrays(self._spec(), [0, 0, 0, 0, 0, 0, 1], np.zeros(6), np.zeros(3), ["b_slide"], [0.0])

    def test_wrong_array_sizes_are_an_error(self):
        spec = part("p", (0, 0, 0), [((0, 0, 0), (0.1, 0.1, 0.1), GREY)], 0.1)
        with self.assertRaises(ValueError):
            parts._state_from_arrays(spec, np.zeros(6), np.zeros(6), np.zeros(3))


# -- fakes for the simulator side -------------------------------------------------------------------------------------


class Tensor:
    """Just enough of a torch tensor for parts._to_numpy: detach().cpu().numpy() and indexing."""

    def __init__(self, a, dtype=np.float64):
        self.a = np.asarray(a, dtype=dtype)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.a

    def __getitem__(self, i):
        return Tensor(self.a[i], self.a.dtype)


class Field:
    """An Isaac Lab 3.0 data field: a wrapper whose ``.torch`` is the tensor."""

    def __init__(self, a, dtype=np.float64):
        self.torch = Tensor(a, dtype)


class FakeAsset:
    def __init__(self, joint_names=(), joint_pos=None, pose=None, vel=None, limits=None, dtype=np.float64):
        self.joint_names = list(joint_names)
        self.data = types.SimpleNamespace()
        if joint_pos is not None:
            self.data.joint_pos = Field([joint_pos], dtype)
        if pose is not None:
            self.data.root_link_pose_w = Field([pose], dtype)
        if vel is not None:
            self.data.root_vel_w = Field([vel], dtype)
        if limits is not None:
            self.data.joint_pos_limits = Field([limits], dtype)


class FakeScene:
    def __init__(self, assets, origin=(0.0, 0.0, 0.0)):
        self.assets = dict(assets)
        self.env_origins = Tensor([origin])

    def __getitem__(self, name):
        return self.assets[name]


class FakeEnv:
    def __init__(self, assets, origin=(0.0, 0.0, 0.0)):
        self.unwrapped = types.SimpleNamespace(scene=FakeScene(assets, origin))


ARM_JOINT_NAMES = [f"joint{i}" for i in range(1, 9)]


def arm(q7, q8, names=None, limits="urdf", dtype=np.float64) -> FakeAsset:
    """An arm whose finger joints (joint7, joint8) are at q7 and q8."""
    names = list(names or ARM_JOINT_NAMES)
    q = np.zeros(len(names))
    q[names.index("joint7")], q[names.index("joint8")] = q7, q8
    lim = None
    if isinstance(limits, str):  # "urdf": every joint [-0.04695, 0] (only the fingers are read)
        lim = np.tile([-0.04695, 0.0], (len(names), 1))
    elif limits is not None:
        lim = limits
    return FakeAsset(names, q, limits=lim, dtype=dtype)


# -- gripper_open -----------------------------------------------------------------------------------------------------


class GripperTests(unittest.TestCase):
    def env(self, left=(-0.04695, -0.04695), right=(0.0, 0.0), **kw):
        return FakeEnv({"left_robot": arm(*left, **kw), "right_robot": arm(*right, **kw)})

    def test_open_and_closed(self):
        e = self.env()
        self.assertTrue(gripper_open(e, "left"))
        self.assertFalse(gripper_open(e, "right"))

    def test_both_fingers_must_be_open(self):
        for q7, q8 in ((-0.04695, 0.0), (0.0, -0.04695), (-0.04695, -0.02), (-0.02, -0.04695)):
            with self.subTest(q7=q7, q8=q8):
                self.assertFalse(gripper_open(self.env(left=(q7, q8)), "left"))

    def test_threshold_is_a_fraction_of_the_way_to_open(self):
        for threshold, q, expected in ((0.9, -0.0425, True), (0.9, -0.042, False), (0.5, -0.0235, True),
                                       (0.5, -0.0234, False), (0.0, 0.0, True), (1.0, FINGER_OPEN, True)):
            with self.subTest(threshold=threshold, q=q):
                self.assertEqual(gripper_open(self.env(left=(q, q)), "left", threshold), expected)

    def test_an_exact_boundary_does_not_flip_on_rounding(self):
        # 0.9 of the way open, written as the float32 an Isaac Lab tensor would hold
        q = np.float32(0.9 * FINGER_OPEN)
        self.assertTrue(gripper_open(self.env(left=(q, q), dtype=np.float32), "left", 0.9))

    def test_a_finger_driven_the_wrong_way_is_not_open(self):
        self.assertFalse(gripper_open(self.env(left=(0.04, 0.04)), "left"))
        self.assertFalse(gripper_open(self.env(left=(0.04, -0.04695)), "left"))

    def test_the_arms_are_independent(self):
        e = self.env(left=(0.0, 0.0), right=(-0.04695, -0.04695))
        self.assertFalse(gripper_open(e, "left"))
        self.assertTrue(gripper_open(e, "right"))

    def test_finger_columns_are_found_by_name_not_by_position(self):
        names = ["joint8", "joint3", "joint7", "joint1", "joint2", "joint4", "joint5", "joint6"]
        e = FakeEnv({"left_robot": arm(-0.04695, 0.0, names=names), "right_robot": arm(-0.04695, -0.04695, names=names)})
        self.assertFalse(gripper_open(e, "left"))  # joint8 closed
        self.assertTrue(gripper_open(e, "right"))

    def test_unreadable_joints_fail_loudly(self):
        with self.assertRaisesRegex(RuntimeError, "no simulator env"):
            gripper_open(None, "left")
        with self.assertRaisesRegex(RuntimeError, "left_robot"):
            gripper_open(FakeEnv({}), "left")
        with self.assertRaisesRegex(RuntimeError, "cannot read"):
            gripper_open(FakeEnv({"left_robot": types.SimpleNamespace()}), "left")
        with self.assertRaisesRegex(RuntimeError, "cannot read"):
            gripper_open(types.SimpleNamespace(), "left")  # no .unwrapped
        with self.assertRaisesRegex(RuntimeError, "joint8"):
            gripper_open(FakeEnv({"left_robot": arm(-0.04, -0.04, names=[f"joint{i}" for i in range(1, 8)] + ["gripper"])
                                  if False else FakeAsset(["joint7"], [-0.04695])}), "left")
        with self.assertRaisesRegex(RuntimeError, "position is nan"):
            gripper_open(self.env(left=(float("nan"), -0.04695)), "left")

    def test_bad_arguments(self):
        e = self.env()
        with self.assertRaises(ValueError):
            gripper_open(e, "middle")
        for bad in (-0.1, 1.1, float("nan")):
            with self.subTest(threshold=bad), self.assertRaises(ValueError):
                gripper_open(e, "left", bad)

    def test_limits_that_do_not_contain_the_assumed_open_position_are_refused(self):
        flipped = np.tile([-0.04695, 0.0], (8, 1))
        flipped[ARM_JOINT_NAMES.index("joint8")] = [0.0, 0.04695]  # joint8 would open toward positive
        e = FakeEnv({"left_robot": arm(-0.04695, -0.04695, limits=flipped)})
        with self.assertRaisesRegex(RuntimeError, "joint8.*sign convention"):
            gripper_open(e, "left")

    def test_unreadable_limits_do_not_block_the_answer(self):
        self.assertTrue(gripper_open(self.env(limits=None), "left"))
        garbage = np.zeros((3, 5))  # wrong shape: the check is skipped
        self.assertTrue(gripper_open(self.env(limits=garbage), "left"))


class FingerSignTests(unittest.TestCase):
    """The per-joint open positions in parts.py against the URDF and the robot's own geometry."""

    def setUp(self):
        try:
            from yam_arena import YAM_URDF
            from yam_arena.analysis import world_hulls
            from yam_arena.constants import TCP_OFFSET
            from yam_arena.urdf import forward_kinematics, parse_joints
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"yam_arena's URDF tools are not importable here: {exc!r}")
        if not Path(YAM_URDF).is_file():
            self.skipTest("the YAM URDF is not available")
        self.urdf, self.joints = YAM_URDF, parse_joints(YAM_URDF)
        self.fk, self.hulls, self.tcp_z = forward_kinematics, world_hulls, TCP_OFFSET[2]

    def gap(self, q7, q8):
        poses = self.fk(self.urdf, {"joint7": q7, "joint8": q8}, root="gripper")
        pads = []
        for link in ("tip_left", "tip_right"):
            v = np.concatenate([h.vertices for _, h in self.hulls(poses, [link])])
            pads.append(v[v[:, 2] > self.tcp_z - 0.005])
        return pads[1][:, 0].min() - pads[0][:, 0].max()  # as yam_arena's own tests measure the pad gap

    def test_the_table_has_every_finger_joint(self):
        self.assertEqual(set(parts._FINGER_OPEN_POS), set(FINGER_JOINTS))

    def test_open_is_the_far_urdf_limit_of_each_joint(self):
        for name in FINGER_JOINTS:
            lo, hi = self.joints[name].lower, self.joints[name].upper
            far = lo if abs(lo) > abs(hi) else hi
            self.assertAlmostEqual(far, parts._FINGER_OPEN_POS[name], places=5, msg=name)

    def test_driving_one_finger_to_its_open_position_widens_the_pads(self):
        closed = self.gap(0.0, 0.0)
        self.assertAlmostEqual(closed, 0.0, delta=1e-3)
        for name in FINGER_JOINTS:
            q = {"joint7": 0.0, "joint8": 0.0}
            q[name] = parts._FINGER_OPEN_POS[name]
            self.assertAlmostEqual(self.gap(q["joint7"], q["joint8"]) - closed, abs(FINGER_OPEN), delta=1e-3, msg=name)
            q[name] = -parts._FINGER_OPEN_POS[name]  # the opposite sign squeezes the pad into the other one
            self.assertLess(self.gap(q["joint7"], q["joint8"]), closed - 0.04, msg=name)


# -- hooking yam_arena -----------------------------------------------------------------------------------------------


def fake_scene_module() -> types.ModuleType:
    mod = types.ModuleType("yam_arena.scene")
    mod.calls = []

    def task_object(spec):
        mod.calls.append(("task_object", spec.name))
        return ("original object", spec.name)

    def object_states(environment, names):
        mod.calls.append(("object_states", list(names)))
        return {n: ("original state", n) for n in names}

    mod.task_object, mod.object_states = task_object, object_states
    return mod


class FakeEmbodiment:
    def __init__(self):
        self._env = "the-env"
        self.log = []

    def reset(self, scene, *, seed=None):
        self.log.append(("reset", scene, seed))
        return "observation"

    def _task_step(self, flag=None):
        self.log.append(("task_step", flag))
        return "step result"


def fake_embodiment_module() -> types.ModuleType:
    mod = types.ModuleType("yam_arena.inspect_embodiment")
    mod.YamArenaEmbodiment = type("YamArenaEmbodiment", (FakeEmbodiment,), {})  # a fresh class per test
    return mod


class HookCase(unittest.TestCase):
    """Saves and restores the yam_arena entries in sys.modules and parts' registries around each test."""

    def setUp(self):
        saved_modules = {n: sys.modules.get(n) for n in parts._HOOKS}
        saved = (dict(parts._PARTS), dict(parts._STEP_CALLBACKS), dict(parts._RESET_CALLBACKS), parts._CURRENT["env"])
        for registry in (parts._PARTS, parts._STEP_CALLBACKS, parts._RESET_CALLBACKS):
            registry.clear()
        parts._CURRENT["env"] = None

        def restore():
            for name, module in saved_modules.items():
                if module is None:
                    sys.modules.pop(name, None)
                    with contextlib.suppress(AttributeError):
                        delattr(yam_arena, name.rpartition(".")[2])
                else:
                    sys.modules[name] = module
            for registry, old in zip((parts._PARTS, parts._STEP_CALLBACKS, parts._RESET_CALLBACKS), saved[:3]):
                registry.clear()
                registry.update(old)
            parts._CURRENT["env"] = saved[3]

        self.addCleanup(restore)

    def hooked_scene(self):
        scene = fake_scene_module()
        sys.modules["yam_arena.scene"] = scene
        parts.install()
        return scene

    def hooked_embodiment(self):
        module = fake_embodiment_module()
        sys.modules["yam_arena.inspect_embodiment"] = module
        parts.install()
        return module.YamArenaEmbodiment


class SceneHookTests(HookCase):
    def test_plain_objects_go_to_the_original_task_object(self):
        scene = self.hooked_scene()
        box = task_file.Box("cube", pos=(0, 0, 0.8), mass=0.1)
        self.assertEqual(scene.task_object(box), ("original object", "cube"))
        self.assertEqual(scene.calls, [("task_object", "cube")])
        self.assertEqual(parts._PARTS, {})

    def test_parts_are_registered_and_built_by_parts(self):
        scene = self.hooked_scene()
        p = sample_part()
        with mock.patch.object(parts, "_make_arena_object", return_value="arena object") as make:
            self.assertEqual(scene.task_object(p), "arena object")
        make.assert_called_once_with(p)
        self.assertEqual(scene.calls, [])  # the original never saw it, though a Part is a Box
        self.assertIs(parts._PARTS["block"], p)

    def test_object_states_split_between_the_original_and_parts(self):
        scene = self.hooked_scene()
        p = sample_part()
        with mock.patch.object(parts, "_make_arena_object"):
            scene.task_object(p)
        assets = {"block": FakeAsset(["plunger_slide"], [-0.0004], pose=[1.3, 2.1, 0.8, *yaw_quat(np.pi / 2)],
                                     vel=[0, 0, 0, 0, 0, 0.2])}
        e = FakeEnv(assets, origin=(1.0, 2.0, 0.0))
        out = scene.object_states(e, ("cube", "block", "cap"))
        self.assertEqual(list(out), ["cube", "block", "cap"])  # the caller's order
        self.assertEqual(scene.calls, [("object_states", ["cube", "cap"])])  # the part is not the original's business
        self.assertEqual(out["cube"], ("original state", "cube"))
        st = out["block"]
        self.assertIsInstance(st, PartState)
        np.testing.assert_allclose(st.pos, (0.3, 0.1, 0.8), atol=1e-12)
        np.testing.assert_allclose(st.ang_vel, (0, 0, 0.2))
        self.assertEqual(st.sliders, {"plunger": -0.0004})
        np.testing.assert_allclose(st.to_world([1, 0, 0]), st.pos + [0, 1, 0], atol=1e-12)

    def test_object_states_with_only_parts_never_calls_the_original(self):
        scene = self.hooked_scene()
        p = sample_part(with_slider=False)
        with mock.patch.object(parts, "_make_arena_object"):
            scene.task_object(p)
        e = FakeEnv({"block": FakeAsset(pose=[0.28, 0.1, 0.78, 0, 0, 0, 1], vel=np.zeros(6))})
        out = scene.object_states(e, ["block"])
        self.assertEqual(scene.calls, [])
        self.assertEqual(out["block"].sliders, {})

    def test_a_part_missing_from_the_scene_is_a_clear_error(self):
        scene = self.hooked_scene()
        with mock.patch.object(parts, "_make_arena_object"):
            scene.task_object(sample_part())
        with self.assertRaisesRegex(RuntimeError, "no object 'block'"):
            scene.object_states(FakeEnv({}), ["block"])

    def test_missing_velocity_is_an_error_not_zeros(self):
        scene = self.hooked_scene()
        with mock.patch.object(parts, "_make_arena_object"):
            scene.task_object(sample_part(with_slider=False))
        e = FakeEnv({"block": FakeAsset(pose=[0.28, 0.1, 0.78, 0, 0, 0, 1])})
        with self.assertRaises(AttributeError):
            scene.object_states(e, ["block"])

    def test_hooking_twice_does_not_stack_wrappers(self):
        scene = self.hooked_scene()
        first = (scene.task_object, scene.object_states)
        parts.install()
        parts.install()
        self.assertEqual((scene.task_object, scene.object_states), first)
        self.assertTrue(parts._marked(scene.task_object) and parts._marked(scene.object_states))
        self.assertEqual(sum(isinstance(f, parts._HookFinder) for f in sys.meta_path), 1)

    def test_a_scene_without_the_expected_functions_is_refused(self):
        sys.modules["yam_arena.scene"] = types.ModuleType("yam_arena.scene")
        with self.assertRaisesRegex(RuntimeError, "task_object"):
            parts.install()


class EmbodimentHookTests(HookCase):
    def test_step_callbacks_run_before_scoring_with_the_env(self):
        cls = self.hooked_embodiment()
        emb = cls()
        seen = []

        @on_step
        def first(e):
            seen.append(("first", e, parts.env(), list(emb.log)))

        @on_step
        def second(e):
            seen.append(("second", e, parts.env(), list(emb.log)))

        self.assertIs(parts.env(), None)
        self.assertEqual(emb._task_step("x"), "step result")
        self.assertEqual(seen, [("first", "the-env", "the-env", []), ("second", "the-env", "the-env", [])])
        self.assertEqual(emb.log, [("task_step", "x")])  # the original ran, after both

    def test_reset_callbacks_run_after_the_reset_and_the_observation_is_kept(self):
        cls = self.hooked_embodiment()
        emb = cls()
        seen = []
        on_reset(lambda e: seen.append((e, list(emb.log))))
        self.assertEqual(emb.reset("scene", seed=7), "observation")
        self.assertEqual(seen, [("the-env", [("reset", "scene", 7)])])
        self.assertEqual(parts.env(), "the-env")

    def test_step_callbacks_do_not_run_on_reset_and_vice_versa(self):
        cls = self.hooked_embodiment()
        emb = cls()
        calls = []
        on_step(lambda e: calls.append("step"))
        on_reset(lambda e: calls.append("reset"))
        emb.reset("s")
        emb._task_step()
        emb._task_step()
        self.assertEqual(calls, ["reset", "step", "step"])

    def test_decorators_return_the_function(self):
        def f(e):
            return None

        self.assertIs(on_step(f), f)
        self.assertIs(on_reset(f), f)

    def test_hooking_twice_runs_each_callback_once(self):
        cls = self.hooked_embodiment()
        parts.install()
        parts.install()
        emb = cls()
        calls = []
        on_step(lambda e: calls.append(1))
        emb._task_step()
        self.assertEqual(calls, [1])

    def test_a_task_file_loaded_twice_does_not_stack_callbacks(self):
        cls = self.hooked_embodiment()
        emb = cls()
        calls = []

        def load_task_file():
            @on_step
            def check(e):
                calls.append(len(calls))

        load_task_file()
        load_task_file()  # a second load makes a new function with the same module, name and line
        emb._task_step()
        self.assertEqual(calls, [0])
        on_step(lambda e: calls.append("a"))  # distinct functions on distinct lines all stay
        on_step(lambda e: calls.append("b"))
        emb._task_step()
        self.assertEqual(calls[1:], [1, "a", "b"])

    def test_a_failing_callback_propagates(self):
        cls = self.hooked_embodiment()
        emb = cls()

        @on_step
        def boom(e):
            raise ValueError("task bug")

        with self.assertRaisesRegex(ValueError, "task bug"):
            emb._task_step()
        self.assertEqual(emb.log, [])  # and the step was not scored

    def test_an_embodiment_module_without_the_class_is_refused(self):
        sys.modules["yam_arena.inspect_embodiment"] = types.ModuleType("yam_arena.inspect_embodiment")
        with self.assertRaisesRegex(RuntimeError, "YamArenaEmbodiment"):
            parts.install()


class LateImportTests(HookCase):
    """install() before yam_arena.scene / inspect_embodiment are imported: the finder patches them on import."""

    SCENE = textwrap.dedent("""
        def task_object(spec):
            return ("original object", spec.name)

        def object_states(env, names):
            return {n: "original state" for n in names}
    """)
    EMBODIMENT = textwrap.dedent("""
        class YamArenaEmbodiment:
            def __init__(self):
                self._env = "late-env"
            def reset(self, scene, *, seed=None):
                return "observation"
            def _task_step(self):
                return "result"
    """)

    def test_modules_imported_after_install_are_patched(self):
        for name in parts._HOOKS:
            sys.modules.pop(name, None)
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "scene.py").write_text(self.SCENE)
            (Path(tmp) / "inspect_embodiment.py").write_text(self.EMBODIMENT)
            yam_arena.__path__.insert(0, tmp)  # the fake files shadow the real (Isaac-only) modules
            importlib.invalidate_caches()
            try:
                parts.install()  # nothing imported yet: this only puts the finder in place
                self.assertNotIn("yam_arena.scene", sys.modules)
                scene = importlib.import_module("yam_arena.scene")
                embodiment = importlib.import_module("yam_arena.inspect_embodiment")
            finally:
                yam_arena.__path__.remove(tmp)
                sys.path_importer_cache.pop(tmp, None)
        self.assertTrue(parts._marked(scene.task_object) and parts._marked(scene.object_states))
        self.assertEqual(scene.task_object(task_file.Box("cube", pos=(0, 0, 0), mass=0.1)), ("original object", "cube"))
        with mock.patch.object(parts, "_make_arena_object", return_value="arena object"):
            self.assertEqual(scene.task_object(sample_part()), "arena object")
        self.assertTrue(parts._marked(embodiment.YamArenaEmbodiment._task_step))
        calls = []
        on_step(lambda e: calls.append(e))
        emb = embodiment.YamArenaEmbodiment()
        self.assertEqual(emb._task_step(), "result")
        self.assertEqual(calls, ["late-env"])

    def test_the_finder_ignores_other_modules(self):
        finder = next(f for f in sys.meta_path if isinstance(f, parts._HookFinder))
        self.assertIsNone(finder.find_spec("json", None))
        self.assertIsNone(finder.find_spec("yam_arena.task_file", yam_arena.__path__))
        import json as json_module  # still importable as ever

        self.assertTrue(hasattr(json_module, "dumps"))


class PurityTests(unittest.TestCase):
    def test_importing_parts_loads_no_simulator_modules(self):
        code = textwrap.dedent(f"""
            import json, sys
            sys.dont_write_bytecode = True
            sys.path[:0] = {[str(Path(yam_arena.__file__).resolve().parent.parent), str(TASK_DIR)]!r}
            import parts
            heavy = {{"pxr", "torch", "isaaclab", "isaaclab_arena", "isaaclab_physx", "isaacsim", "omni", "carb", "warp"}}
            print(json.dumps(sorted({{m.split(".")[0] for m in sys.modules}} & heavy)))
        """)
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=120, check=True)
        self.assertEqual(json.loads(out.stdout), [])


# -- the Isaac glue, against fakes --------------------------------------------------------------------------------------


@contextlib.contextmanager
def fake_isaac(stage=None):
    """Stand-ins for the Isaac Lab / Arena modules parts.py imports lazily, shaped after how the reference partkit and
    yam_arena/scene.py use them. They check our wiring, not Isaac's behaviour."""
    log = types.SimpleNamespace(materials=[], bindings=[])
    modules: dict[str, types.ModuleType] = {}

    def module(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        modules[name] = m
        parent, _, child = name.rpartition(".")
        if parent in modules:
            setattr(modules[parent], child, m)
        return m

    class MaterialCfg:
        def __init__(self, **kw):
            self.kw = kw

        def func(self, path, cfg):
            UsdShade.Material.Define(stage, path)
            log.materials.append((path, type(self).__name__, dict(self.kw)))

    class RigidBodyMaterialCfg(MaterialCfg):
        pass

    class PreviewSurfaceCfg(MaterialCfg):
        pass

    def bind_physics_material(prim_path, material_path, stage=None):
        log.bindings.append(("physics", prim_path, material_path))

    def bind_visual_material(prim_path, material_path, stage=None):
        log.bindings.append(("visual", prim_path, material_path))

    def clone(fn):
        @functools.wraps(fn)
        def wrapper(prim_path, cfg, *args, **kwargs):
            return fn(prim_path, cfg, *args, **kwargs)

        return wrapper

    class ObjectType(enum.Enum):
        RIGID = 1
        ARTICULATION = 2
        BASE = 3

    class Pose:
        def __init__(self, position_xyz, rotation_xyzw):
            self.position_xyz, self.rotation_xyzw = position_xyz, rotation_xyzw

    class Object:
        def __init__(self, name, object_type, spawner_cfg, initial_pose):
            self.name, self.object_type, self.spawner_cfg, self.initial_pose = name, object_type, spawner_cfg, initial_pose
            self.prim_path = "{ENV_REGEX_NS}/" + name

        def _add_initial_pose_to_cfg(self, cfg):
            cfg.init_state = self.initial_pose
            return cfg

    class Cfg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    module("isaaclab")
    module("isaaclab.sim", RigidBodyMaterialCfg=RigidBodyMaterialCfg, PreviewSurfaceCfg=PreviewSurfaceCfg)
    module("isaaclab.sim.utils", bind_physics_material=bind_physics_material,
           bind_visual_material=bind_visual_material, clone=clone)
    module("isaaclab.sim.utils.stage", get_current_stage=lambda: stage)
    module("isaaclab.sim.spawners")
    module("isaaclab.sim.spawners.spawner_cfg", SpawnerCfg=type("SpawnerCfg", (), {}))
    module("isaaclab.actuators", ImplicitActuatorCfg=Cfg)
    module("isaaclab.assets", ArticulationCfg=Cfg)
    module("isaaclab.utils", configclass=dataclasses.dataclass)
    module("isaaclab_arena")
    module("isaaclab_arena.assets")
    module("isaaclab_arena.assets.object", Object=Object)
    module("isaaclab_arena.assets.object_type", ObjectType=ObjectType)
    module("isaaclab_arena.utils")
    module("isaaclab_arena.utils.pose", Pose=Pose)
    saved = {n: sys.modules.get(n) for n in modules}
    sys.modules.update(modules)
    try:
        yield log
    finally:
        for name, old in saved.items():
            if old is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old


@unittest.skipUnless(HAVE_PXR, NO_PXR)
class IsaacGlueTests(HookCase):
    def test_a_part_with_a_slider_becomes_an_articulation_that_spawns_its_usd(self):
        stage = Usd.Stage.CreateInMemory()
        spec = sample_part(yaw=np.pi / 2)
        scene = self.hooked_scene()
        with fake_isaac(stage) as log:
            obj = scene.task_object(spec)
            self.assertEqual(scene.calls, [])
            self.assertEqual((obj.name, obj.object_type.name), ("block", "ARTICULATION"))
            np.testing.assert_allclose(obj.initial_pose.position_xyz, (0.28, 0.10, 0.78))
            np.testing.assert_allclose(obj.initial_pose.rotation_xyzw, yaw_quat(np.pi / 2), atol=1e-12)
            cfg = obj.spawner_cfg
            self.assertEqual(cfg.part_name, "block")
            prim = cfg.func("/World/envs/env_0/block", cfg, translation=obj.initial_pose.position_xyz,
                            orientation=obj.initial_pose.rotation_xyzw)
            art = obj._generate_articulation_cfg()
        self.assertEqual(str(prim.GetPath()), "/World/envs/env_0/block")
        self.assertEqual(art.prim_path, "{ENV_REGEX_NS}/block")
        self.assertIs(art.spawn, cfg)
        self.assertEqual(art.init_state, obj.initial_pose)
        self.assertEqual(list(art.actuators), ["plunger"])
        actuator = art.actuators["plunger"]
        self.assertEqual((actuator.joint_names_expr, actuator.stiffness, actuator.damping),
                         (["plunger_slide"], 12000.0, 40.0))
        # materials: one physics material with the part's friction, one look per distinct colour, made under Looks
        by_class = {}
        for path, kind, kw in log.materials:
            by_class.setdefault(kind, []).append((path, kw))
            self.assertTrue(path.startswith("/World/envs/env_0/block/Looks/"))
            self.assertTrue(stage.GetPrimAtPath(path).IsValid())
        (phys_path, phys_kw), = by_class["RigidBodyMaterialCfg"]
        self.assertEqual(phys_kw, dict(static_friction=0.4, dynamic_friction=0.4))
        self.assertEqual(len(by_class["PreviewSurfaceCfg"]), 4)  # grey, dark, red, steel
        shapes = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Gprim)]
        self.assertEqual(len(shapes), 5)  # body: 2 boxes + 1 cylinder; plunger: 1 box + 1 cylinder
        physics_bound = [p for kind, p, m in log.bindings if kind == "physics"]
        self.assertEqual(sorted(physics_bound), sorted(str(s.GetPath()) for s in shapes))
        self.assertTrue(all(m == phys_path for kind, p, m in log.bindings if kind == "physics"))
        visual = [(p, m) for kind, p, m in log.bindings if kind == "visual"]
        self.assertEqual(len(visual), 5)
        # the world pose: yaw 90 degrees puts the part's local +x along world +y
        cache = UsdGeom.XformCache()
        body_box = stage.GetPrimAtPath("/World/envs/env_0/block/body/box_1")  # local centre (0.02, -0.05, 0)
        centre = cache.GetLocalToWorldTransform(body_box).Transform(Gf.Vec3d(0, 0, 0))
        np.testing.assert_allclose(list(centre), (0.28 + 0.05, 0.10 + 0.02, 0.78), atol=1e-9)

    def test_a_part_without_sliders_is_a_rigid_object(self):
        stage = Usd.Stage.CreateInMemory()
        scene = self.hooked_scene()
        with fake_isaac(stage):
            obj = scene.task_object(sample_part(with_slider=False))
            self.assertEqual(obj.object_type.name, "RIGID")
            prim = obj.spawner_cfg.func("/World/block", obj.spawner_cfg)
        self.assertEqual([c.GetName() for c in prim.GetChildren()], ["body", "Looks"])
        self.assertFalse(any(p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in Usd.PrimRange(prim)))

    def test_spawning_an_unregistered_part_is_an_error(self):
        stage = Usd.Stage.CreateInMemory()
        with fake_isaac(stage):
            cfg = types.SimpleNamespace(part_name="ghost")
            with self.assertRaisesRegex(RuntimeError, "ghost"):
                parts._isaac_spawn("/World/ghost", cfg)


# -- USD authoring -----------------------------------------------------------------------------------------------------


class FakePhysxSchema:
    """A stand-in for pxr.PhysxSchema that writes real attributes, so the schema path can be compared with the plain one."""

    def __init__(self):
        self.calls = []
        owner = self

        def api(api_name, prefix):
            class API:
                @classmethod
                def Apply(cls, prim):
                    prim.AddAppliedSchema(api_name)
                    return cls(prim)

                def __init__(self, prim):
                    self.prim = prim

                def __getattr__(self, attr):
                    if not (attr.startswith("Create") and attr.endswith("Attr")):
                        raise AttributeError(attr)
                    camel = attr[len("Create"):-len("Attr")]
                    name = f"{prefix}:{camel[0].lower()}{camel[1:]}"

                    def create(value):
                        from pxr import Sdf

                        owner.calls.append((api_name, attr))
                        kind = (Sdf.ValueTypeNames.Bool if isinstance(value, bool)
                                else Sdf.ValueTypeNames.Int if isinstance(value, int) else Sdf.ValueTypeNames.Float)
                        self.prim.CreateAttribute(name, kind).Set(value)

                    return create

            return API

        self.PhysxRigidBodyAPI = api("PhysxRigidBodyAPI", "physxRigidBody")
        self.PhysxCollisionAPI = api("PhysxCollisionAPI", "physxCollision")
        self.PhysxArticulationAPI = api("PhysxArticulationAPI", "physxArticulation")


def author(spec, path="/World/block", **kw):
    stage = Usd.Stage.CreateInMemory()
    physics, visual = [], []
    prim = parts.author_part(stage, path, spec, physics.append, lambda p, c: visual.append((p, c)), **kw)
    return stage, prim, physics, visual


def physx_attributes(prim) -> dict:
    """Every physx* attribute under ``prim`` as {(prim path, attribute): value}, plus the applied schemas."""
    out = {}
    for p in Usd.PrimRange(prim):
        # read the apiSchemas metadata: GetAppliedSchemas() hides schemas that are not registered, as Physx* is not here
        tokens = p.GetMetadata("apiSchemas")
        for schema in (tokens.GetAddedOrExplicitItems() if tokens else []):
            if schema.startswith("Physx"):
                out[(str(p.GetPath()), "schema", schema)] = True
        for a in p.GetAttributes():
            if a.GetName().startswith("physx"):
                out[(str(p.GetPath()), a.GetName())] = a.Get()
    return out


@unittest.skipUnless(HAVE_PXR, NO_PXR)
class AuthorPartTests(unittest.TestCase):
    def test_prim_hierarchy(self):
        stage, root, _, _ = author(sample_part())
        names = lambda path: [c.GetName() for c in stage.GetPrimAtPath(path).GetChildren()]  # noqa: E731
        self.assertEqual(str(root.GetPath()), "/World/block")
        self.assertEqual(names("/World/block"), ["body", "plunger"])
        self.assertEqual(names("/World/block/body"), ["box_0", "box_1", "cyl_0"])
        self.assertEqual(names("/World/block/plunger"), ["box_0", "cyl_0", "plunger_slide"])
        types_ = {str(p.GetPath()): p.GetTypeName() for p in Usd.PrimRange(root)}
        self.assertEqual(types_["/World/block"], "Xform")
        self.assertEqual(types_["/World/block/body"], "Xform")
        self.assertEqual(types_["/World/block/body/box_0"], "Cube")
        self.assertEqual(types_["/World/block/body/cyl_0"], "Cylinder")
        self.assertEqual(types_["/World/block/plunger/plunger_slide"], "PhysicsPrismaticJoint")

    def test_collision_api_is_on_every_shape_and_nothing_else(self):
        stage, root, physics, visual = author(sample_part())
        shapes = {str(p.GetPath()) for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Gprim)}
        self.assertEqual(len(shapes), 5)  # body: 2 boxes + 1 cylinder; plunger: 1 box + 1 cylinder
        with_api = {str(p.GetPath()) for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.CollisionAPI)}
        self.assertEqual(with_api, shapes)
        self.assertEqual(sorted(physics), sorted(shapes))  # the physics material is bound to each, once
        self.assertEqual(sorted(p for p, _ in visual), sorted(shapes))
        colours = dict(visual)
        self.assertEqual(colours["/World/block/body/box_0"], GREY)
        self.assertEqual(colours["/World/block/body/cyl_0"], RED)
        self.assertEqual(colours["/World/block/plunger/cyl_0"], STEEL)

    def test_one_body_per_part_and_per_slider_with_the_masses(self):
        stage, root, _, _ = author(sample_part())
        bodies = {str(p.GetPath()): UsdPhysics.MassAPI(p).GetMassAttr().Get()
                  for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
        self.assertEqual(set(bodies), {"/World/block/body", "/World/block/plunger"})
        self.assertAlmostEqual(bodies["/World/block/body"], 0.30, places=6)
        self.assertAlmostEqual(bodies["/World/block/plunger"], 0.002, places=6)
        # no shape is a rigid body of its own (that would split the compound)
        self.assertFalse(any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Gprim)))

    def test_articulation_root_is_the_body_only_when_there_are_sliders(self):
        stage, root, _, _ = author(sample_part())
        roots = [str(p.GetPath()) for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        self.assertEqual(roots, ["/World/block/body"])
        stage, root, _, _ = author(sample_part(with_slider=False))
        self.assertFalse(any(p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in Usd.PrimRange(root)))
        self.assertFalse(any(p.IsA(UsdPhysics.Joint) for p in Usd.PrimRange(root)))

    def test_prismatic_joint_limits_and_drive(self):
        stage, _, _, _ = author(sample_part())
        joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath("/World/block/plunger/plunger_slide"))
        self.assertEqual(joint.GetAxisAttr().Get(), "X")
        self.assertAlmostEqual(joint.GetLowerLimitAttr().Get(), -0.0015, places=9)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), 0.0)
        self.assertEqual([str(t) for t in joint.GetBody0Rel().GetTargets()], ["/World/block/body"])
        self.assertEqual([str(t) for t in joint.GetBody1Rel().GetTargets()], ["/World/block/plunger"])
        np.testing.assert_allclose(list(joint.GetLocalPos0Attr().Get()), (-0.015, -0.04, 0.0), atol=1e-7)
        np.testing.assert_allclose(list(joint.GetLocalPos1Attr().Get()), (0, 0, 0))
        drive = UsdPhysics.DriveAPI.Get(joint.GetPrim(), "linear")
        self.assertTrue(drive)
        self.assertEqual(drive.GetTypeAttr().Get(), "force")
        self.assertAlmostEqual(drive.GetStiffnessAttr().Get(), 12000.0, places=3)
        self.assertAlmostEqual(drive.GetDampingAttr().Get(), 40.0, places=4)
        self.assertEqual(drive.GetTargetPositionAttr().Get(), 0.0)

    def test_every_slider_axis(self):
        for axis in "XYZ":
            slider = Slider("s", (0, 0, 0), boxes=[((0, 0, 0), (0.01, 0.01, 0.01), RED)], axis=axis)
            spec = part("p", (0, 0, 0), [((0, 0, 0), (0.1, 0.1, 0.1), GREY)], 0.1, sliders=[slider])
            stage, _, _, _ = author(spec, "/p")
            joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath("/p/s/s_slide"))
            self.assertEqual(joint.GetAxisAttr().Get(), axis)

    def test_geometry_in_the_body_and_in_the_slider_frame(self):
        stage, _, _, _ = author(sample_part())
        cache = UsdGeom.XformCache()

        def world(path, p=(0, 0, 0)):
            return list(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)).Transform(Gf.Vec3d(*p)))

        # no yaw, no translation: world == part frame
        np.testing.assert_allclose(world("/World/block/body/box_1"), (0.02, -0.05, 0), atol=1e-9)
        # the plunger body sits at its rest point; its cylinder (centre == rest) is at the slider origin
        np.testing.assert_allclose(world("/World/block/plunger"), (-0.015, -0.04, 0), atol=1e-7)
        np.testing.assert_allclose(world("/World/block/plunger/cyl_0"), (-0.015, -0.04, 0), atol=1e-7)
        np.testing.assert_allclose(world("/World/block/plunger/box_0"), (-0.015, -0.04, 0.004), atol=1e-7)
        cube = UsdGeom.Cube(stage.GetPrimAtPath("/World/block/body/box_0"))
        self.assertEqual(cube.GetSizeAttr().Get(), 1.0)
        scale = [op for op in cube.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeScale][0]
        np.testing.assert_allclose(list(scale.Get()), (0.085, 0.11, 0.06), atol=1e-7)
        cyl = UsdGeom.Cylinder(stage.GetPrimAtPath("/World/block/body/cyl_0"))
        self.assertEqual(cyl.GetAxisAttr().Get(), "Y")
        self.assertAlmostEqual(cyl.GetRadiusAttr().Get(), 0.01)
        self.assertAlmostEqual(cyl.GetHeightAttr().Get(), 0.02)

    def test_the_root_pose_places_the_whole_part(self):
        yaw = 0.6
        stage, _, _, _ = author(sample_part(), translation=(1.0, 2.0, 3.0), orientation_xyzw=yaw_quat(yaw))
        cache = UsdGeom.XformCache()
        local = np.array([0.02, -0.05, 0.0])  # body/box_1's centre
        c, s = np.cos(yaw), np.sin(yaw)
        expected = (1.0 + c * local[0] - s * local[1], 2.0 + s * local[0] + c * local[1], 3.0)
        m = cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/block/body/box_1"))
        np.testing.assert_allclose(list(m.Transform(Gf.Vec3d(0, 0, 0))), expected, atol=1e-9)
        # the plunger follows the same pose
        rest = np.array([-0.015, -0.04, 0.0])
        m = cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/block/plunger"))
        np.testing.assert_allclose(list(m.Transform(Gf.Vec3d(0, 0, 0))),
                                   (1.0 + c * rest[0] - s * rest[1], 2.0 + s * rest[0] + c * rest[1], 3.0), atol=1e-7)

    def test_physx_settings_on_every_body_collider_and_the_articulation(self):
        stage, root, _, _ = author(sample_part())
        attrs = physx_attributes(root)
        for body in ("/World/block/body", "/World/block/plunger"):
            self.assertIn((body, "schema", "PhysxRigidBodyAPI"), attrs)
            self.assertEqual(attrs[(body, "physxRigidBody:solverPositionIterationCount")], 32)
            self.assertAlmostEqual(attrs[(body, "physxRigidBody:maxDepenetrationVelocity")], 1.0)
            self.assertAlmostEqual(attrs[(body, "physxRigidBody:angularDamping")], parts.ANGULAR_DAMPING)
        shapes = [str(p.GetPath()) for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Gprim)]
        for shape in shapes:
            self.assertIn((shape, "schema", "PhysxCollisionAPI"), attrs)
            self.assertAlmostEqual(attrs[(shape, "physxCollision:contactOffset")], 0.0005, places=9)
            self.assertEqual(attrs[(shape, "physxCollision:restOffset")], 0.0)
        art = "/World/block/body"
        self.assertIn((art, "schema", "PhysxArticulationAPI"), attrs)
        self.assertIs(attrs[(art, "physxArticulation:enabledSelfCollisions")], False)
        self.assertEqual(attrs[(art, "physxArticulation:solverPositionIterationCount")], 32)
        self.assertEqual(attrs[(art, "physxArticulation:solverVelocityIterationCount")], 4)
        self.assertEqual(sum(k[1] == "schema" and k[2] == "PhysxArticulationAPI" for k in attrs), 1)

    def test_a_part_without_sliders_gets_no_articulation_settings(self):
        stage, root, _, _ = author(sample_part(with_slider=False))
        self.assertFalse(any("physxArticulation" in k[1] or "PhysxArticulationAPI" in k for k in physx_attributes(root)))

    def test_the_schema_path_writes_the_same_attributes_as_the_plain_path(self):
        plain_stage, plain_root, _, _ = author(sample_part())  # usd-core has no PhysxSchema: the plain writer
        plain = physx_attributes(plain_root)
        self.assertTrue(any(k[1] == "schema" for k in plain))
        fake = FakePhysxSchema()
        with mock.patch.dict(sys.modules, {"pxr.PhysxSchema": fake}), mock.patch.object(pxr, "PhysxSchema", fake, create=True):
            schema_stage, schema_root, _, _ = author(sample_part())  # now `from pxr import PhysxSchema` succeeds
        self.assertEqual(plain, physx_attributes(schema_root))
        used = set(fake.calls)
        self.assertEqual(used, {
            ("PhysxRigidBodyAPI", "CreateMaxDepenetrationVelocityAttr"),
            ("PhysxRigidBodyAPI", "CreateSolverPositionIterationCountAttr"),
            ("PhysxRigidBodyAPI", "CreateAngularDampingAttr"),
            ("PhysxCollisionAPI", "CreateContactOffsetAttr"),
            ("PhysxCollisionAPI", "CreateRestOffsetAttr"),
            ("PhysxArticulationAPI", "CreateEnabledSelfCollisionsAttr"),
            ("PhysxArticulationAPI", "CreateSolverPositionIterationCountAttr"),
            ("PhysxArticulationAPI", "CreateSolverVelocityIterationCountAttr"),
        })

    def test_the_writer_choice_follows_whether_physxschema_imports(self):
        self.assertIsInstance(parts._physx_writer(), parts._PlainUsdWriter)
        fake = FakePhysxSchema()
        with mock.patch.dict(sys.modules, {"pxr.PhysxSchema": fake}), mock.patch.object(pxr, "PhysxSchema", fake, create=True):
            self.assertIsInstance(parts._physx_writer(), parts._SchemaWriter)

    def test_two_sliders_and_the_stage_survives_a_round_trip(self):
        sliders = [Slider(f"s{i}", (0.01 * i, 0, 0), boxes=[((0.01 * i, 0, 0), (0.004, 0.004, 0.004), RED)], axis="Y")
                   for i in range(2)]
        spec = part("p", (0, 0, 0), [((0, 0, 0), (0.1, 0.1, 0.1), GREY)], 0.1, sliders=sliders)
        stage, root, _, _ = author(spec, "/p")
        joints = sorted(str(p.GetPath()) for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.PrismaticJoint))
        self.assertEqual(joints, ["/p/s0/s0_slide", "/p/s1/s1_slide"])
        reopened = Usd.Stage.CreateInMemory()
        reopened.GetRootLayer().ImportFromString(stage.GetRootLayer().ExportToString())
        self.assertTrue(reopened.GetPrimAtPath("/p/s1/s1_slide").IsValid())


if __name__ == "__main__":
    unittest.main()
