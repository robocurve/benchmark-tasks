"""Commission every guide/orientation and the slot-and-pin retention with gravity
and the actual PhysX colliders."""

import argparse
import json
from math import cos, radians, sin
from pathlib import Path

from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY
from yam_arena.inspect_embodiment import YamArenaEmbodiment

import scoring
import task
from run import install, measure

EXPECTED = {
    "original_guide": {"aligned": "seated", "opposed": "seated"},
    "matching_spare": {"aligned": "seated", "opposed": "blocked"},
    "wrong_spare": {"aligned": "blocked", "opposed": "seated"},
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    install()
    env = YamArenaEmbodiment(task=str(Path(task.__file__)), object_jitter=0,
                             control_hz=task.CONTROL_HZ, cam_width=320, cam_height=176)
    scene = Scene(id="guide-commissioning", instruction=task.INSTRUCTION)
    hold = Action(data=[*READY, 1.0, *READY, 1.0])
    rows = []

    def place(name, xy, above_table, *, yaw=0, tilt=0):
        base = env._base
        body = base.scene[name]
        pose = body.data.default_root_pose.torch.clone()
        pose[:, :] = env._torch.tensor([*xy, task.TOP + above_table,
                                        cos(yaw / 2) * sin(tilt / 2),
                                        sin(yaw / 2) * sin(tilt / 2),
                                        sin(yaw / 2) * cos(tilt / 2),
                                        cos(yaw / 2) * cos(tilt / 2)],
                                       dtype=pose.dtype, device=pose.device)
        pose[:, :3] += base.scene.env_origins
        body.write_root_pose_to_sim_index(root_pose=pose)
        body.write_root_velocity_to_sim_index(
            root_velocity=env._torch.zeros_like(body.data.default_root_vel.torch)
        )

    def kick(name, velocity):
        body = env._base.scene[name]
        vel = env._torch.zeros_like(body.data.default_root_vel.torch)
        vel[:, :3] = env._torch.tensor(velocity, dtype=vel.dtype, device=vel.device)
        body.write_root_velocity_to_sim_index(root_velocity=vel)

    def mount(name, *, pinned=True):
        env.reset(scene, seed=0)
        if name != "original_guide":
            # Out of the way: original on the tray, the guide under test in the slot.
            place("original_guide", (task.REJECT[0] - .0015, task.REJECT[1]), .006)
            place(name, task.FIX, task.FLOOR)
        if pinned:
            place("lock_pin", task.PIN_HOLE, task.TAB_TOP)
        else:
            place("lock_pin", task.PARK, task.PARK_TOP)
        for _ in range(8):
            env.step(hold)

    try:
        for name, checks in EXPECTED.items():
            for direction, expected in checks.items():
                mount(name)
                # Starts clear of the bottom key and is never reset during
                # the measurement. Gravity alone takes it to its stop.
                place("test_part", task.FIX, task.SEAT + .028,
                      yaw=0 if direction == "aligned" else radians(180))
                for _ in range(50):
                    result = env.step(hold)
                part = measure(env._base, "test_part")
                guide = measure(env._base, name)
                observed = scoring.test_reading(part, guide, True)
                info = result.info
                row = {"case": "reading", "guide": name, "orientation": direction, "expected": expected,
                       "observed": observed[1] if observed else None,
                       "height_above_seat_mm": round(info["height_above_seat_mm"], 3),
                       "held_steps": info["reading_hold_steps"], "installed": info["installed"]}
                print(json.dumps(row), flush=True)
                rows.append(row)
                assert info["installed"] == name and observed == (direction, expected), row
                assert info["reading_hold_steps"] >= task.CONTROL_HZ, row

        # Retention: a pinned guide stays under the lips when yanked up or back
        # toward the entry (the slotted pin hole allows a few mm of creep; release
        # needs 16 mm); without the pin the same yank slides it more than 10 mm.
        for label, pinned, velocity in (("pinned, pulled up", True, (0., 0., .6)),
                                        ("pinned, pulled back", True, (-.4, 0., 0.)),
                                        ("unpinned, pulled back", False, (-.4, 0., 0.))):
            mount("matching_spare", pinned=pinned)
            for _ in range(10):
                kick("matching_spare", velocity)
                env.step(hold)
            for _ in range(30):
                env.step(hold)
            guide = measure(env._base, "matching_spare")
            dx, dy, dz = (scoring.offset(guide) * 1000).round(2).tolist()
            home = bool(scoring.slid_home(guide))
            row = {"case": "retention", "label": label, "home": home, "offset_mm": [dx, dy, dz],
                   "pin_home": bool(scoring.pin_home(measure(env._base, "lock_pin")))}
            retained = dx > -8 and abs(dz) <= 1
            row["retained"] = retained
            print(json.dumps(row), flush=True)
            rows.append(row)
            assert retained if pinned else dx < -10, row
    finally:
        if args.output:
            args.output.write_text(json.dumps(rows, indent=2) + "\n")
        env.close()


if __name__ == "__main__":
    main()
