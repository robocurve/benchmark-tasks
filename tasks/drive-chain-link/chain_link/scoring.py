"""Primary current-state 0--1 score, based on the actually measured native assembly."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from .config import CONNECTOR_POSITION, PITCH, TABLE_TOP, CHAIN_POSITION, NODES


def rotation(pose):
    return quat_xyzw_to_matrix(np.array(pose[3:]))


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.violations = set()
        self.physics_samples = 0
        self.hold_start = None

    def observe_physics(self, points):
        self.physics_samples += 1
        for p in points.values():
            if not np.isfinite(p).all():
                self.violations.add("invalid_state")
            elif (
                p[:, 2].min() < TABLE_TOP - 0.006
                or p[:, 0].min() < -0.19
                or p[:, 0].max() > 0.91
                or abs(p[:, 1]).max() > 0.65
            ):
                self.violations.add("part_left_work_area")

    def evaluate(self, s):
        parts = [s["connector"], s["keeper"], *s["bodies"].values()]
        finite = all(
            np.isfinite(p["pose"]).all() and np.isfinite(p["velocity"]).all()
            for p in parts
        )
        finite = (
            finite
            and np.isfinite(s["robot_clearance_m"])
            and all(np.isfinite(h["opening_m"]) for h in s["hands"].values())
        )
        if not finite:
            self.violations.add("invalid_state")
            self.hold_start = None
            return dict(
                score=0.0,
                success=False,
                violations=sorted(self.violations),
                ends_joined=False,
                plate_seated=False,
                retained=False,
                parked=False,
                settled=False,
                hands_clear=False,
                robot_clearance_m=s["robot_clearance_m"],
                end_points_local={},
                keeper_local=[],
                pins_in_keeper=[],
                physics_samples=self.physics_samples,
            )
        c = s["connector"]["pose"]
        k = s["keeper"]["pose"]
        r = rotation(c)
        kr = rotation(k)
        ends = {}
        passed = []
        for arm, body, sign in [("right", "Link00", -1), ("left", "Link10", 1)]:
            item = s["bodies"][body]
            pose = item["pose"]
            point = np.array(pose[:3]) + rotation(pose) @ np.array(
                [sign * PITCH / 2, 0, 0]
            )
            local = r.T @ (point - c[:3])
            target = np.array([0, sign * PITCH / 2, 0.007])
            ends[arm] = local.tolist()
            # 32 mm bore around an 8 mm stem: 12 mm radial clearance.
            passed.append(
                bool(
                    np.linalg.norm(local[:2] - target[:2]) <= 0.012
                    and 0.003 < local[2] < 0.014
                    and np.dot(rotation(pose)[:, 2], r[:, 2]) > 0.98
                )
            )
        joined = all(passed)
        local = r.T @ (np.array(k[:3]) - c[:3])
        axis_ok = np.trace(r.T @ kr) >= 1 + 2 * np.cos(0.10)
        plate_depth = 0.016 < local[2] < 0.025
        pins = [
            kr.T @ (np.array(c[:3]) + r @ np.array([0, y, 0.024]) - k[:3])
            for y in (-PITCH / 2, PITCH / 2)
        ]
        in_apertures = []
        in_necks = []
        for p, y in zip(pins, (-PITCH / 2, PITCH / 2)):
            in_apertures.append(bool(-0.0225 < p[0] < 0.036 and abs(p[1] - y) < 0.005))
            # Include a stem bottomed against the physical closed end at -26 mm:
            # its 4 mm radius puts its centre at -22 mm (0.5 mm contact margin).
            in_necks.append(bool(-0.0225 < p[0] < -0.008 and abs(p[1] - y) < 0.005))
        covered = bool(joined and plate_depth and axis_ok and all(in_apertures))
        retained = bool(covered and all(in_necks))
        parked = bool(
            np.linalg.norm(np.array(c[:3]) - CONNECTOR_POSITION) < 0.015
            and np.trace(r) >= 1 + 2 * np.cos(np.deg2rad(10))
        )
        velocities = [v["velocity"] for v in s["bodies"].values()] + [
            s[n]["velocity"] for n in ("connector", "keeper")
        ]
        settled = all(
            np.linalg.norm(v[:3]) < 0.02 and np.linalg.norm(v[3:]) < 0.3
            for v in velocities
        )
        clear = bool(
            s["robot_clearance_m"] >= 0.025
            and all(h["opening_m"] >= 0.070 for h in s["hands"].values())
        )
        guide_errors={}
        for i in range(1,10):
            pose=np.array(s["bodies"][f"Link{i:02}"]["pose"])
            tooth=CHAIN_POSITION+(NODES[i]+NODES[i+1])/2
            guide_local=rotation(pose).T@(tooth-pose[:3])
            guide_errors[f"Link{i:02}"]=guide_local.tolist()
        # A 6mm-radius tooth must remain inside the real link window:
        # +/-27mm along the link and +/-17mm between the side rails.
        guide_seated=bool(all(abs(v[0]) < .021 and abs(v[1]) < .0115
                             for v in guide_errors.values()) and all(
            abs(s["bodies"][f"Link{i:02}"]["pose"][2]-CHAIN_POSITION[2]) < .006
            and rotation(s["bodies"][f"Link{i:02}"]["pose"])[2,2] > .995
            for i in range(1,10)))
        full = bool(retained and parked and settled and clear and guide_seated and not self.violations)
        if full:
            if self.hold_start is None:
                self.hold_start = s["time_s"]
        else:
            self.hold_start = None
        success = bool(full and s["time_s"] - self.hold_start >= 1.0)
        score = (
            (1.0 if success else 0.7 if covered else 0.4 if joined else 0.0)
            if not self.violations
            else 0.0
        )
        return dict(
            score=score,
            success=success,
            violations=sorted(self.violations),
            ends_joined=joined,
            plate_seated=covered,
            retained=retained,
            parked=parked,
            settled=bool(settled),
            hands_clear=clear,
            guide_seated=guide_seated,
            guide_link_errors_m=guide_errors,
            robot_clearance_m=s["robot_clearance_m"],
            end_points_local=ends,
            keeper_local=local.tolist(),
            pins_in_keeper=[p.tolist() for p in pins],
            physics_samples=self.physics_samples,
        )
