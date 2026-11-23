"""Bedside observations in millimetres. Each foam stripe has its own unloaded height."""

import mujoco as mj
import numpy as np

CALVES = ("calf_prox", "calf_dist")
HAZARDS = ("achilles", "knee_shell")
PEOPLE = (
    "head",
    "torso",
    "far_leg",
    "far_foot",
    "arm_rest",
    "thigh",
    "knee_shell",
    "heel",
    "calf_prox",
    "calf_dist",
    "achilles",
    "forefoot",
)
PADS = tuple("pad" + str(i) for i in range(6))


class Witness:
    def __init__(self, m):
        self.ids = {m.geom(i).name: i for i in range(m.ngeom)}
        self.tare = None

    def sample(self, m, d, heel_load, assist, arms=("arm_",)):
        def gap(a, b):
            return 1000 * mj.mj_geomDistance(m, d, self.ids[a], self.ids[b], 0.25, None)

        strips = np.array([d.joint(f"compression{i}").qpos[0] for i in range(6)])
        if self.tare is None:
            self.tare = strips.copy()
        compressed = (self.tare - strips) * 1000
        calf_gaps = [[gap(c, p) for p in PADS] for c in CALVES]
        active = [
            [i for i, g in enumerate(row) if g <= 2 and compressed[i] >= 3]
            for row in calf_gaps
        ]
        occupied = sorted(set(active[0] + active[1]))
        arm_geoms = [n for n in self.ids if n.startswith(arms) and n not in PEOPLE]
        distal = [
            n
            for n in arm_geoms
            if n.split("_", 1)[1]
            in ("link4", "link5", "gripper", "tip_left", "tip_right")
        ]

        # Use broad phase before convex distance calls; bounds are centred at geom_xpos.
        def min_gap(names, others, cap):
            result = cap
            for a in names:
                i = self.ids[a]
                for b in others:
                    j = self.ids[b]
                    delta = d.geom_xpos[i] - d.geom_xpos[j]
                    if (
                        delta @ delta
                        < (
                            m.geom_rbound[i]
                            + m.geom_rbound[j]
                            + max(result, 0.1) / 1000
                        )
                        ** 2
                    ):
                        result = min(result, gap(a, b))
            return float(result)

        jaw_pair = False
        for arm in arms:
            for region in PEOPLE:
                i = self.ids[region]
                ends = []
                for jaw in ("tip_left", "tip_right"):
                    j = self.ids[arm + jaw]
                    # The closest points give the two sides of a visible clamp around the same body.
                    if (
                        np.linalg.norm(d.geom_xpos[i] - d.geom_xpos[j])
                        > m.geom_rbound[i] + m.geom_rbound[j] + 0.002
                    ):
                        break
                    seg = np.zeros(6)
                    if mj.mj_geomDistance(m, d, i, j, 0.05, seg) > 0.001:
                        break
                    ends.append(seg[:3] - d.geom_xpos[i])
                if len(ends) == 2 and ends[0] @ ends[1] < 0:
                    jaw_pair = True
        heel = d.geom_xpos[self.ids["heel"]]
        knee = d.geom_xpos[self.ids["knee_shell"]]
        mat = d.geom_xmat[self.ids["sole"]].reshape(3, 3)
        corners = [
            d.geom_xpos[self.ids["sole"]] + mat @ np.array([x, y, 0])
            for x, y in (
                (0.082, 0.129),
                (-0.082, 0.129),
                (-0.082, -0.129),
                (0.082, -0.129),
            )
        ]
        return dict(
            heel_channel_n=float(heel_load),
            lift_slack=bool(abs(assist) < 1e-8),
            heel_xyz_mm=(1000 * heel).tolist(),
            knee_xyz_mm=(1000 * knee).tolist(),
            sole_marks_mm=(1000 * np.array(corners)).tolist(),
            sole_up=float(mat[2, 2]),
            heel_clear_mm=min(gap("heel", p) for p in ("bed", "sole", *PADS)),
            protected_clear_mm=min(gap(c, p) for c in HAZARDS for p in (*PADS, "sole")),
            sole_bed_gap_mm=gap("sole", "bed"),
            calf_rows=active,
            stripe_compression_mm=compressed.tolist(),
            bearing_span_mm=(occupied[-1] - occupied[0]) * 43 if occupied else 0,
            person_clear_mm=min_gap(arm_geoms, PEOPLE, 60),
            park_gap_mm=min_gap(distal, (*PEOPLE, "sole", *PADS), 100),
            both_jaws_on_limb=bool(jaw_pair),
            bed_touch=bool(min_gap(distal, ("bed",), 5) <= 0.5),
        )
