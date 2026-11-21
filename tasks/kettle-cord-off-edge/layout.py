"""Every size, mass, friction and position of the kettle-cord scene, with its source. The task file and the scorer both
read from here. Frame: x forward from the arm bases, y left, z up, metres; positions are from the midpoint between the
two arm bases. The table stands for a kitchen counter; its LEFT edge (y = 0.65) stands for the counter's front edge,
where a child stands.

Sources (opened 2026-11-20):
- Kettle: Hamilton Beach 40880, 1.7 L stainless electric kettle on a cordless 360-degree power base
  (hamiltonbeach.com/stainless-steel-1-7-liter-kettle-40880g: "8.78 H x 6.24 W x 9.24 D" inches, 1.7 liters, 1500 W;
  about 2.2 lb with its base, retailer spec tables). The manual's parts list names a "Power Switch", "Power Base" and
  "Cord Wrap"; safeguard 9: "Do not let cord hang over edge of table or counter, or touch hot" surfaces
  (manualslib.com/manual/567785/Hamilton-Beach-40880.html).
- Guidance: American Burn Association, Scald Injury Prevention Educator's Guide: "All appliance cords need to be kept
  coiled and away from counter edges. Curious children may reach up and grab handles or cords."
- Sippy cup: Munchkin Miracle 360 trainer cup, 7 oz. Learning tower: Little Partners Original Learning Tower, Soft White
  (littlepartners.com/products/original-learning-tower: "Assembled Dimensions: 38.07"H x 23.18"W x 33.14"D";
  "layered Birch and Poplar"; platform "Level 3: 15.79""), pushed against the counter's front edge.
- Plug: UL 498 (Attachment Plugs and Receptacles), section 119: a receptacle must hold a plug against 3 lbf (13 N) with
  at most 2 mm of movement and must let go of it at 15 lbf (67 N). The simulated outlet lets go at 20 N.
Estimates (no page gives them): the split of the 1.0 kg between jug and base, the switch's lever length, travel and
click force, the cord's length (76 cm), thickness and stiffness, the cup's size, the plug's size and mass, frictions.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # the station table's top (yam_arena)
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)
EDGE_Y = 0.65  # the counter's front edge (the table's left edge)

# -- kettle (jug frame: origin on its axis at the bottom of the jug, z up; handle toward -y, away from the edge) -------
BASE_XY = (0.50, 0.52)  # the power base's centre: its rim is 5 cm from the front edge
BASE_R, BASE_H, BASE_RIM = 0.080, 0.020, 0.006  # base disc 16 cm x 2 cm, a 6 mm rim that seats the jug
BASE_MASS = 0.20  # of the 1.0 kg kettle + base (estimate split)
JUG_R, JUG_H = 0.072, 0.190  # 14.4 cm body (6.24 in wide with its trim), body 19 cm tall: 22.3 cm with lid and base
JUG_EMPTY = 0.80
FILL = 1.00  # 1.0 L of water; a real trial uses 1.0 kg of dry ballast in a sealed bag inside, lid shut (no water)
JUG_MASS = JUG_EMPTY + FILL
JUG_COM = (0.0, 0.0, 0.055)
HANDLE_Y = (-0.112, -0.094)  # the handle's grip bar (y from the axis), 18 mm thick, 30 mm wide
HANDLE_Z = (0.045, 0.175)
LEVER_PIVOT = (0.0, -0.100, 0.030)  # the power switch: a lever under the handle's foot, its tip out behind the handle
LEVER_LEN, LEVER_W, LEVER_T = 0.032, 0.016, 0.006  # tip at y -0.132: 2 cm out past the handle's back face
LEVER_UP, LEVER_DOWN = 8.0, -8.0  # degrees: OFF (tip up) and ON (tip down), the hinge's two stops: 9 mm of tip travel
LEVER_SPRING = 0.015  # N m per degree toward the nearer stop (over-centre): about 4 N at the tip to click it ON
LEVER_MASS = 0.005
ON_BELOW = -4.0  # the lever counts as ON (down, lamp lit) below this angle

# -- the cord ------------------------------------------------------------------------------------------------------------
CORD_R = 0.0035  # 7 mm round cord (estimate)
CORD_LEN = 0.76  # 76 cm (estimate: the short cord the safeguards ask for)
LINK = 0.02  # 38 rigid 2 cm links
N_LINKS = round(CORD_LEN / LINK)
LINK_MASS = 0.0012  # 60 g per metre (estimate)
BEND_K = 0.004  # N m per radian at each joint (estimate); no twist stiffness
BEND_DAMP = 0.0005
CORD_FRICTION = (0.8, 0.7)
EXIT = (BASE_XY[0] - 0.072, BASE_XY[1] + 0.030)  # the cord leaves the base on its side toward the robot
A_X, B_X = 0.40, 0.32  # where the cord crosses the front edge going out (from the base) and coming back (to the plug)

# -- socket strip, cup, tower -------------------------------------------------------------------------------------------
STRIP_XY = (0.30, 0.27)  # 6-outlet strip lying along x, taped down; the kettle's plug sits in its nearest outlet
STRIP_SIZE = (0.27, 0.055, 0.036)
PLUG_XY = (STRIP_XY[0] - 0.08, STRIP_XY[1])
PLUG_Z = STRIP_SIZE[2] + 0.012  # the plug's centre (and the cord's end) 12 mm over the strip's top
PLUG_SIZE, PLUG_MASS = (0.026, 0.030, 0.024), 0.03  # the plug's body, seated face-down in the outlet (estimate)
PLUG_PULL = 20.0  # N: the outlet lets go of the plug (UL 498: holds at 13 N, lets go at 67 N or less)
CUP_XY, CUP_R, CUP_H, CUP_MASS = (0.52, 0.20), 0.038, 0.115, 0.08  # sippy cup, empty
TOWER_X = (0.47, 0.47 + 0.589)  # the tower's 58.9 cm (23.18 in) front along the front edge, outside the counter (y > 0.65)
TOWER_DEPTH, TOWER_RAIL_Z = 0.842, 0.967  # 84.2 cm (33.14 in) deep; 96.7 cm (38.07 in) tall: 22 cm over the counter
TOWER_STEP_Z = 0.401  # its platform at level 3 (15.79 in)

# -- the score ------------------------------------------------------------------------------------------------------------
EDGE_CLEAR = 0.05  # every part of the cord at least 5 cm in from the counter's edges
REST_Z = 0.06  # and lying on the counter: no part higher than 6 cm (the plug's top is 5 cm up)
SHIFT = 0.02  # the kettle moved at most 2 cm
SEAT = 0.01  # the jug's axis within 1 cm of the base's centre
TILT_DEG = 5.0
PART = 0.4  # cord back on the counter, kettle on its base, plug seated, not switched on
PLUG_GAP = 0.002  # the plug counts as seated: its face at most 2 mm off the strip's face (UL 498's 2 mm)


def cord_path(depth: float) -> list[np.ndarray]:
    """The cord's start shape: from the base's exit to the front edge, over it, down ``depth`` below the counter in a U,
    back up and over the edge, and across the counter to the plug."""
    r = CORD_R + 0.002
    z0 = TOP + CORD_R
    pts = [np.array([EXIT[0], EXIT[1], TOP + 0.008])]
    def over(x, sgn):  # the turn over the edge's corner (sgn +1 going out, -1 coming back)
        c = np.array([x, EDGE_Y, TOP])
        arc = [c + np.array([0.0, r * math.sin(a), r * math.cos(a)]) for a in np.linspace(0.0, math.pi / 2, 5)]
        return arc if sgn > 0 else arc[::-1]
    pts += [np.array([A_X, EDGE_Y - 0.02, z0])] + over(A_X, 1)
    yo = EDGE_Y + r
    rad = (A_X - B_X) / 2
    zb = TOP - depth + rad
    pts.append(np.array([A_X, yo, zb]))
    for a in np.linspace(0.0, math.pi, 9)[1:-1]:
        pts.append(np.array([B_X + rad + rad * math.cos(a), yo, zb - rad * math.sin(a)]))
    pts.append(np.array([B_X, yo, zb]))
    pts += over(B_X, -1) + [np.array([B_X, EDGE_Y - 0.02, z0])]
    pts += [np.array([PLUG_XY[0] - 0.02, PLUG_XY[1] + 0.07, z0]), np.array([PLUG_XY[0], PLUG_XY[1] + 0.02, TOP + PLUG_Z])]
    return pts


def _length(pts) -> float:
    return float(sum(np.linalg.norm(b - a) for a, b in zip(pts[:-1], pts[1:])))


def _resample(pts, n: int) -> list[np.ndarray]:
    seg = [np.linalg.norm(b - a) for a, b in zip(pts[:-1], pts[1:])]
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    out = []
    for s in np.linspace(0.0, cum[-1], n + 1):
        k = min(int(np.searchsorted(cum, s, side="right")) - 1, len(seg) - 1)
        t = (s - cum[k]) / seg[k] if seg[k] > 0 else 0.0
        out.append(pts[k] + t * (pts[k + 1] - pts[k]))
    return out


def cord_nodes() -> tuple[list[np.ndarray], float]:
    """N_LINKS + 1 joint points along the start shape, its loop depth chosen so that the path is CORD_LEN long."""
    lo, hi = 0.03, 0.40
    for _ in range(50):
        mid = (lo + hi) / 2
        if _length(cord_path(mid)) < CORD_LEN:
            lo = mid
        else:
            hi = mid
    return _resample(cord_path(lo), N_LINKS), lo


if __name__ == "__main__":
    nodes, d = cord_nodes()
    ch = [np.linalg.norm(b - a) for a, b in zip(nodes[:-1], nodes[1:])]
    print(f"loop depth {d * 100:.1f} cm, {len(ch)} links, chord {min(ch) * 1000:.1f}-{max(ch) * 1000:.1f} mm, "
          f"lowest {min(p[2] for p in nodes) - TOP:.3f} m, outside the edge: {sum(p[1] > EDGE_Y for p in nodes)} nodes")
