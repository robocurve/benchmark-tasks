"""The coin lodged in the inhaler's mouthpiece, held by static friction until something would really move it; declared
into mechkit for the task file (task.py) and the cap's known-load check (cap_task.py). A mechkit helper module: it may
declare (MECHKIT_HELPER below).

In the simulator a coin wedged in the bore creeps along it on any tilt, from the jitter of its contacts, where a real
wedged coin stays: held in the hand, the mouthpiece level within 0.5 deg, it went 23 mm in to the barrel within 4 s,
or 9 mm out to the lip (runs w9i24, w9i26, w9i27). So the coin is welded to the inhaler as built - the weld stands in
for static friction - and the weld lets go, once and for good in that episode, when static friction would give:

- when the coin's load leaves the friction cone of its wedge. The load per kilogram is gravity less the coin's
  acceleration, the acceleration read from the coin's place at the last three control steps (its velocity over each
  control period, and the change between the two), not from one physics step. The cone's half angle is 32.9 deg about
  the wedge's down (the inhaler's -y: down as the inhaler lies): where the wedged coin slid in this simulator's own
  contact, the mouthpiece tilted slowly, before any creep (run w9i1; to be checked on a real inhaler). So, held still,
  it lets go with the mouthpiece tilted 32.9 deg either way (or the inhaler rolled that far about it, assumed: not
  measured); held level, on a jolt of tan 32.9 deg = 0.647 g along the bore (the same friction);
- when something pushes it with more than its hold, tan 32.9 deg x its weight = 14.4 mN: the cap (the mean contact
  force over the control period, from a contact sensor), a hand, a loose dime or the bench (the contact points the
  simulator reports on the coin).

After that the coin is free in the true bore, as before: friction 0.3 on the staves.
"""

from __future__ import annotations

import math

import capfit as F
import mechkit as mk
import numpy as np
import parts as P
from mechkit import sense_contact, weld

MECHKIT_HELPER = True  # mechkit counts this module's declarations as the task file's own

SLIDE = math.radians(32.9)  # the wedged coin's static friction angle: where it slid in run w9i1 (the mouthpiece tilted slowly)
GRAVITY = 9.81  # m/s^2
HOLD = math.tan(SLIDE) * P.LODGED_MASS * GRAVITY  # N: the push it takes to move the wedged coin (14.4 mN)
OWN = "inhaler"  # what the coin is wedged in: its contacts with it are no push

STATE: dict = {}


def declare(cap: str | None = "cap") -> None:
    """The weld, as built (the coin and the inhaler declared before), and the coin's contact sensor for the cap."""
    weld("lodged", "coin", OWN)
    if cap is not None:
        sense_contact("coin", cap)
    STATE["cap"] = cap


def reset() -> None:
    """From the task's @on_reset function (mechkit's reset puts the weld back)."""
    cap = STATE.get("cap", "cap")
    STATE.clear()
    STATE.update(cap=cap, places=[], released=None, most_angle=0.0, most_jolt=0.0, load_in_i=None)


reset()


def lodged() -> bool:
    return not mk.is_cut("lodged")


def acceleration() -> np.ndarray | None:
    """m/s^2 of the coin in the world from its last three places (None before there are three)."""
    places = STATE["places"]
    if len(places) < 3:
        return None
    (t0, p0), (t1, p1), (t2, p2) = places[-3:]
    if t1 - t0 <= 0.0 or t2 - t1 <= 0.0:
        return None
    return ((p2 - p1) / (t2 - t1) - (p1 - p0) / (t1 - t0)) / (0.5 * (t2 - t0))


def cone_angle(load: np.ndarray, down: np.ndarray) -> float:
    """rad between the load and the wedge's down (pi when there is no load: the coin floats)."""
    size = float(np.linalg.norm(load))
    if size < 1e-9:
        return math.pi
    return math.acos(max(-1.0, min(1.0, float(np.dot(load, down)) / size)))


def pushes() -> dict[str, float]:
    """N on the coin from what touches it, by what: the cap from the sensor (mean over the control period), the rest
    from the contact points reported this control step (the inhaler's own contacts left out)."""
    out: dict[str, float] = {}
    if STATE.get("cap") is not None:
        out[STATE["cap"]] = mk.contact_force("coin", STATE["cap"])
    for c in mk.contacts("coin"):
        if c.other in (OWN, STATE.get("cap")) or c.separation > 0.0:
            continue
        out[c.other] = out.get(c.other, 0.0) + float(c.force)
    return out


def watch(events: list) -> None:
    """Once per control step: lets the coin go when its load leaves the cone or a push passes its hold."""
    if not lodged():
        return
    now = mk.now()
    STATE["places"] = [*STATE["places"][-2:], (now, np.array(mk.pose("coin")[0], dtype=float))]
    down = -F.i_pose()[1][:, 1]  # the wedge's down: the inhaler's -y (capfit.py: its frame I)
    a = acceleration()
    cause = None
    if a is not None:
        load = np.array([0.0, 0.0, -GRAVITY]) - a
        STATE["load_in_i"] = F.i_pose()[1].T @ load  # per kg, in the inhaler's frame I: its x along the mouthpiece, out of the mouth
        angle = cone_angle(load, down)
        STATE["most_angle"] = max(STATE["most_angle"], angle)
        STATE["most_jolt"] = max(STATE["most_jolt"], float(np.linalg.norm(a)) / GRAVITY)
        if angle > SLIDE:
            cause = f"its load {math.degrees(angle):.1f} deg from the wedge's down (jolt {float(np.linalg.norm(a)) / GRAVITY:.2f} g)"
    if cause is None:
        for what, force in pushes().items():
            if force > HOLD:
                cause = f"pushed by {what} with {1000 * force:.1f} mN"
                break
    if cause is not None:
        mk.cut("lodged")
        STATE["released"] = (now, cause)
        events.append((now, f"coin let go: {cause}"))
