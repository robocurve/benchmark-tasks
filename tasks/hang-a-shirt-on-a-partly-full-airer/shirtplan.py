"""The scripted bimanual demo: TCP keyframes for both hands, solved to joint targets every control step. NumPy only.

1. The hands back off from the airer, then come down over the T-shirt's two hem corners, fingers pointing down and
   30 mm apart, closing front to back, and pinch each corner off the table.
2. They lift, turning the fingers to point inward and level (as a person holds a sheet by its corners), and draw the
   hem out to its full width, so the shirt hangs spread from its hem, neck down.
3. High enough that the hanging shirt clears the free front rail, they carry it to above the gap behind that rail
   and lower it, so its neck half goes down into the gap beside the blue face cloth on the middle rail.
4. They draw the hem forward over the front rail, then down in front of the airer until the two halves are about
   even, and open.
5. Both hands draw back and away.
"""

from __future__ import annotations

import math

import numpy as np

import fastik
import layout as L

DT = 0.05  # 20 Hz
BASES = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}
READY = np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])
OPEN, PINCH_OPEN, CLOSED = 1.0, 0.32, 0.0  # gripper commands: 1 = fully open (94 mm), 0.32 = 30 mm

T = L.TOP



def grip(arm: str, down_deg: float) -> np.ndarray:
    """TCP rotation. The fingers always close along world x, so a pinched fold of fabric lies in the y-z plane, the
    plane the shirt hangs in. ``down_deg`` turns the approach about x: 90 points the fingers straight down, 0 points
    them level and inward (the left hand toward -y, the right toward +y), as a person holds a sheet by its corners."""
    side = 1.0 if arm == "left" else -1.0
    a = math.radians(down_deg)
    z = np.array([0.0, -side * math.cos(a), -math.sin(a)])
    x = np.array([-side, 0.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


# hand heights above the rails' tops (m): lowering the neck half into the gap behind the free front rail, then the hem
# hands in front of the airer, below the rail, when they let go (the halves come out about even)
NECK_H = 0.12
HEM_H = -0.08
FRONT_X = 0.265  # the hem hands in front of the airer: the gripper clears rail A
UNDER_X = 0.30  # then back under the rail's front, so the hem side hangs straight
HEM_Y = 0.112  # each hand's distance from the middle once the hem is drawn out
GAP_BEHIND = (L.RAIL_X["A"] + L.RAIL_X["B"]) / 2


def keyframes():
    """[(seconds to get there, label, {arm: (TCP position, approach angle, gripper)})]; y of the left hand."""
    hem_y0, hem_y1 = 0.095, HEM_Y  # pinch points across, then the hem drawn out to its full width
    pick_x = L.TEE_HEM_X + 0.012
    carry_z = T + L.RAIL_TOP + 0.30  # the hanging shirt's neck clears the front face cloth by about 2 cm
    k = []

    def add(t, label, x, y, z, ang, g):
        k.append((t, label, {"left": ((x, y, z), ang, g), "right": ((x, -y, z), ang, g)}))

    add(1.5, "back from the airer", 0.22, 0.18, T + 0.26, 70, PINCH_OPEN)  # the ready pose is over rail A
    add(1.5, "over the hem corners", pick_x, hem_y0, T + 0.08, 90, PINCH_OPEN)
    add(1.5, "down to the table", pick_x, hem_y0, T + 0.016, 90, PINCH_OPEN)
    add(1.0, "pinch", pick_x, hem_y0, T + 0.016, 90, CLOSED)
    add(1.5, "lift the hem", pick_x, hem_y0, T + 0.10, 80, CLOSED)
    add(2.0, "raise, turning the hands inward", 0.24, hem_y1, T + 0.30, 30, CLOSED)
    add(2.0, "shirt hangs spread from its hem", 0.28, hem_y1, carry_z, 0, CLOSED)
    add(1.5, "let it stop swinging", 0.28, hem_y1, carry_z, 0, CLOSED)
    add(2.5, "carry it over the front rail", GAP_BEHIND, hem_y1, carry_z, 0, CLOSED)
    add(1.5, "still, over the gap behind it", GAP_BEHIND, hem_y1, carry_z, 0, CLOSED)
    add(3.0, "neck half down into the gap", GAP_BEHIND, hem_y1, T + L.RAIL_TOP + NECK_H, 0, CLOSED)
    add(2.5, "draw the hem forward over the rail", FRONT_X, hem_y1, T + L.RAIL_TOP + 0.08, 0, CLOSED)
    add(2.5, "hem down in front of the airer", FRONT_X, hem_y1, T + L.RAIL_TOP + HEM_H, 0, CLOSED)
    add(2.0, "hem side hanging straight", UNDER_X, hem_y1, T + L.RAIL_TOP + HEM_H, 0, CLOSED)
    add(1.0, "still", UNDER_X, hem_y1, T + L.RAIL_TOP + HEM_H, 0, CLOSED)
    add(1.0, "let go", UNDER_X, hem_y1, T + L.RAIL_TOP + HEM_H, 0, OPEN)
    add(1.5, "hands back", UNDER_X - 0.07, hem_y1 + 0.03, T + L.RAIL_TOP + HEM_H, 0, OPEN)
    add(2.0, "back away", 0.20, 0.17, T + 0.16, 0, OPEN)
    return k


def _smooth(s: float) -> float:
    return s * s * (3 - 2 * s)


class Plan:
    def __init__(self):
        self.steps = []  # per control step: {arm: (q, gripper, label)}
        self.worst = 0.0
        q = {arm: READY.copy() for arm in BASES}
        g = {arm: OPEN for arm in BASES}
        pos, pitch = {}, {}  # set by the first keyframe (reached in joint space from the ready pose)
        frames = keyframes()
        # out of the ready pose: straight in joint space to the first keyframe's grip
        seconds, label, target = frames[0]
        n = int(round(seconds / DT))
        goal = {}
        for arm in BASES:
            best = None
            for seed in (READY, np.array([0.0, 1.2, 1.0, 0.0, 0.0, 0.0]), np.array([0.0, 1.5, 1.4, -0.3, 0.0, 0.0])):
                qs, pe, re = fastik.solve(target[arm][0], grip(arm, target[arm][1]), seed, BASES[arm], iters=300)
                if best is None or pe + 0.02 * re < best[1]:
                    best = (qs, pe + 0.02 * re)
            goal[arm] = best[0]
            self.worst = max(self.worst, best[1])
        for i in range(1, n + 1):
            s = _smooth(i / n)
            self.steps.append({arm: (READY + s * (goal[arm] - READY), OPEN + s * (target[arm][2] - OPEN), label)
                               for arm in BASES})
        for arm in BASES:
            q[arm] = goal[arm].copy()
            pos[arm] = np.asarray(target[arm][0], float)
            pitch[arm] = target[arm][1]
            g[arm] = target[arm][2]
        for seconds, label, target in frames[1:]:
            n = max(1, int(round(seconds / DT)))
            start = {arm: (pos[arm].copy(), pitch[arm], g[arm]) for arm in BASES}
            for i in range(1, n + 1):
                s = _smooth(i / n)
                row = {}
                for arm in BASES:
                    p0, a0, g0 = start[arm]
                    p1, a1, g1 = target[arm]
                    p = p0 + s * (np.asarray(p1) - p0)
                    a = a0 + s * (a1 - a0)
                    gg = g0 + s * (g1 - g0)
                    qn, pe, re = fastik.solve(p, grip(arm, a), q[arm], BASES[arm], iters=60)
                    self.worst = max(self.worst, pe + 0.02 * re)
                    q[arm] = qn
                    row[arm] = (qn.copy(), gg, label)
                self.steps.append(row)
            for arm in BASES:
                pos[arm] = np.asarray(target[arm][0], float)
                pitch[arm] = target[arm][1]
                g[arm] = target[arm][2]


_PLAN = None


def plan() -> Plan:
    global _PLAN
    if _PLAN is None:
        _PLAN = Plan()
    return _PLAN


if __name__ == "__main__":
    p = plan()
    print(len(p.steps), "steps,", len(p.steps) * DT, "s; worst IK error", round(p.worst * 1000, 2), "mm-equivalent")
    qs = np.array([[row[a][0] for a in BASES] for row in p.steps])
    print("largest joint step (rad):", np.abs(np.diff(qs, axis=0)).max().round(3))
