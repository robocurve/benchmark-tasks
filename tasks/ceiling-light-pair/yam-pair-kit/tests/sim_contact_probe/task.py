"""Contact-sensor positive control (not a benchmark task). A's left arm and B's right arm stand on the same world line
y = +0.305, facing each other 0.8 m apart; the scripted demo drives both tool points 6 cm past each other at x = 0, so
the grippers must collide. report() gives the cross-station contact and the measured tool points.

    inspect-robots run --embodiment yam_pair -E task=tests/sim_contact_probe/task.py --policy yam_pair_scripted \\
        --instruction probe --scorer pair_task_score --max-steps 200 -P instruction_a=probe -P instruction_b=probe
"""

from yam_pair.task import FINGER_OPEN, TABLE_TOP, Box, Layout, Waypoint

LAYOUT = Layout("facing", separation=0.8)
OBJECTS = [Box("marker", pos=(0.0, -0.45, TABLE_TOP + 0.01), size=(0.02, 0.02, 0.02), color=(0.9, 0.1, 0.1))]
MEET_Z = 0.95


def score(objects, pair) -> float:
    return min(1.0, pair.contact_peak_n / 1.0)  # 1.0 once a contact of 1 N or more was measured


def report(objects, pair) -> dict:
    a, b = pair.tcp("a", "left"), pair.tcp("b", "right")
    return {"contact_n": pair.contact_n, "contact_peak_n": pair.contact_peak_n,
            "a_left_n": pair.contact_by_arm.get("a_left", 0.0), "b_right_n": pair.contact_by_arm.get("b_right", 0.0),
            "a_right_n": pair.contact_by_arm.get("a_right", 0.0), "tcp_gap_x": float(b[0] - a[0]),
            "a_tcp_y": float(a[1]), "b_tcp_y": float(b[1])}


def demo(layout):
    def meet(station, side, x_world):
        local = layout.to_local(station, (x_world, 0.305, MEET_Z))
        hover = layout.to_local(station, (x_world - 0.2 if station == "a" else x_world + 0.2, 0.305, MEET_Z))
        return {side: [Waypoint(2.0, tuple(hover), FINGER_OPEN, "approach"),
                       Waypoint(3.0, tuple(local), FINGER_OPEN, "push past each other"),
                       Waypoint(2.0, tuple(local), FINGER_OPEN, "hold"),
                       Waypoint(2.0, tuple(hover), FINGER_OPEN, "back off")]}

    return {"a": meet("a", "left", 0.03), "b": meet("b", "right", -0.03)}
