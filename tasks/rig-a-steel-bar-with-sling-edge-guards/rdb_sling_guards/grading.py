"""The scorer, and the operator's test it calls for. Pure Python (no NumPy): the tests run it on recorded states.

Each control step the adapter measures the scene and calls ``RiggingGrade.update``:

- which guards are seated on which edge (``guard_seat``: inside corner within 3 mm of the edge, within 10 degrees of
  the seated orientation, and placed along the edge so it covers the web with at least 5 mm to spare each side);
- which horns hold a fitting with their latch closed: the fitting's apex bar in the bowl (``hook.in_bowl``) and its
  centre within 14 mm of the hook's plane, so that the triangle is round the horn, not held beside the hook;
- whether both hands have been clear of the hook, the fittings, the web and the guards for a second (no arm contact,
  and both tool points 80 mm from the hook's centre and from both apex bars);
- whether the sling still passes under the bar.

Once both horns hold a fitting with their latches closed and the hands are clear, the operator takes the load: the
hoist rises until the hook carries 50 N, holds 50 N (force control) for 2 s, and during the last second the web's
contact with the bar's bare steel within 15 mm of each edge is read from contact sensors per body pair. If the hook's
load falls below 40 N during the hold (a snag let go), the hoist lifts again and the hold starts over, with its own
last second; a test whose hold is not complete 30 s after the lift began ends there and fails. A hand that comes back
into the rigging during the test fails it.

Score (each step; the last one counts):
- 1.0: the test ran with all four edges guarded and no web on bare steel near any edge, both fittings still hooked;
- 0, locked: the sling was pulled out from under the bar at any time, or the test ran with an edge unguarded or the
  web on bare steel near an edge (the load was taken on an unprotected edge), a fitting came out of its horn under
  load, the hook did not hold the load within the test's time limit, a hand came back during the test, or a force
  reading was not finite (the simulation failed);
- 0: both fittings hooked while an edge is unguarded (the hitch is ready to lift without its padding; not locked:
  seating the missing guards before the hands have been clear for 1 s lifts it);
- 0.7: both fittings hooked, all four edges guarded, the test not run to its end (the hands never cleared, or the
  run limit came first);
- otherwise 0.1 for each seated guard plus 0.2 for each horn holding a fitting with its latch closed (one at most here:
  both would be a complete hitch): 0 to 0.6.
"""

import math

from . import hook as HK
from . import layout as L

CONTROL_HZ = 20
EDGE_NAMES = ("TL", "BL", "TR", "BR")
SUCCESS_S = 1.0                     # after the test, the episode ends this long later


def quat_matrix(q):
    """Rotation matrix (rows) from a quaternion (x, y, z, w)."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.
    x, y, z, w = x / n, y / n, z / n, w / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def angle_between(a, b):
    """Angle (degrees) of the rotation taking matrix a to matrix b."""
    r = matmul(transpose(a), b)
    c = (r[0][0] + r[1][1] + r[2][2] - 1) / 2
    return math.degrees(math.acos(max(-1., min(1., c))))


def seat_rotation(edge):
    """The guard's seated orientation on ``edge``: its local y to the side face's outward normal, local z to the
    horizontal face's, local x = y cross z (as tools/build_assets.py builds it)."""
    _, n1z, n2y = L.EDGES[edge]
    n1, n2 = (0., 0., n1z), (0., n2y, 0.)
    x = (n2[1] * n1[2] - n2[2] * n1[1], n2[2] * n1[0] - n2[0] * n1[2], n2[0] * n1[1] - n2[1] * n1[0])
    return tuple(tuple(col[i] for col in (x, n2, n1)) for i in range(3))


FLIP = ((-1., 0., 0.), (0., 0., 1.), (0., 1., 0.))       # half turn about the guard's bisector: the same guard shape


def guard_seat(position, quat):
    """The edge a guard is seated on (or None) and, per edge, how far it is from that seat (mm, degrees)."""
    R = quat_matrix(quat)
    best = None
    for edge in EDGE_NAMES:
        (ye, ze), _, _ = L.EDGES[edge]
        d = math.hypot(position[1] - ye, position[2] - ze)
        along = abs(position[0] - L.XS)
        seat = seat_rotation(edge)
        ang = min(angle_between(seat, R), angle_between(matmul(seat, FLIP), R))
        ok = d <= L.SEAT_TOL and ang <= L.SEAT_ANGLE_DEG and along <= L.GUARD_LEN / 2 - L.SLING_W / 2 - L.COVER_MARGIN
        row = (ok, -d, edge, round(1000 * d, 2), round(ang, 2), round(1000 * along, 2))
        if best is None or row[:2] > best[:2]:
            best = row
    return (best[2] if best[0] else None), {"edge": best[2], "corner_mm": best[3], "angle_deg": best[4], "along_mm": best[5]}


class RiggingGrade:
    """Feed one measurement per control step; read the grade and the operator's command."""

    def __init__(self):
        self.tick = 0
        self.locked = None
        self.clear_ticks = 0
        self.test = "waiting"           # waiting -> lifting -> holding -> done
        self.test_ticks = 0
        self.test_start = None
        self.bare_peak = dict.fromkeys(EDGE_NAMES, 0.)
        self.guard_low = dict.fromkeys(EDGE_NAMES, None)
        self.result = None              # the test's verdict
        self.done_ticks = 0
        self.finished = False
        self.last = None
        self.command = "rest"

    def update(self, m):
        """``m``: {"guards": {name: ((x, y, z), (qx, qy, qz, qw))} (tabletop frame), "apex": {fitting: (y, z)} in the hook
        frame, "apex_x": {fitting: x} (off the hook's plane), "latch_deg": {"L": a, "R": a}, "hands_clear": bool, "under_bar": bool, "pull_n": float,
        "bare_n": {edge: N}, "guard_n": {edge: N}}. Returns the grade; ``self.command`` says what the hoist does."""
        self.tick += 1
        seated = {}
        details = {}
        for name, (p, q) in m["guards"].items():
            edge, det = guard_seat(p, q)
            details[name] = det
            if edge is not None:
                seated[edge] = name
        horns = {}
        for side in ("L", "R"):
            closed = abs(m["latch_deg"][side]) <= L.LATCH_CLOSED_DEG
            inside = [f for f, yz in m["apex"].items()
                      if HK.in_bowl(yz, side) and abs(m["apex_x"][f]) <= L.APEX_OFF_PLANE]
            horns[side] = bool(inside) and closed
        n_guards, n_hooked = len(seated), sum(horns.values())
        all_guarded, both = n_guards == 4, n_hooked == 2
        if self.locked is None and not all(math.isfinite(float(v)) for v in (m["pull_n"], *m["bare_n"].values())):
            self.locked = "a nonfinite force reading (the simulation failed)"
        if not m["under_bar"] and self.locked is None:
            self.locked = "the sling was pulled out from under the bar"
        # -- the operator's test -----------------------------------------------------------------------------------
        self.clear_ticks = self.clear_ticks + 1 if (m["hands_clear"] and both) else 0
        if self.test in ("lifting", "holding") and not m["hands_clear"] and self.locked is None:
            self.locked = "a hand came back into the rigging during the test lift"   # OSHA 1910.184(c)(9), (c)(10)
        if self.test == "waiting" and both and self.clear_ticks >= L.CLEAR_S * CONTROL_HZ:
            self.test, self.test_start = "lifting", self.tick
            self.guarded_at_test = all_guarded
        elif self.test == "lifting" and m["pull_n"] >= L.PULL_N:
            self.test, self.test_ticks = "holding", 0
        elif self.test == "holding" and m["pull_n"] < L.HOLD_MIN_N:
            self.test, self.test_ticks = "lifting", 0          # the load fell away: lift again, the hold starts over
            self.bare_peak = dict.fromkeys(EDGE_NAMES, 0.)       # ... and so do its last-second readings
            self.guard_low = dict.fromkeys(EDGE_NAMES, None)
        elif self.test == "holding":
            self.test_ticks += 1
            if self.test_ticks > (L.HOLD_S - 1.) * CONTROL_HZ:          # the last second of the hold
                for e in EDGE_NAMES:
                    self.bare_peak[e] = max(self.bare_peak[e], float(m["bare_n"].get(e, 0.)))
                    g = float(m["guard_n"].get(e, 0.))
                    self.guard_low[e] = g if self.guard_low[e] is None else min(self.guard_low[e], g)
            if self.test_ticks >= L.HOLD_S * CONTROL_HZ:
                self.end_test(all_guarded, both, held=True)
        if self.test in ("lifting", "holding") and self.tick - self.test_start >= L.TEST_LIMIT_S * CONTROL_HZ:
            self.end_test(all_guarded, both, held=False)
        self.command = {"waiting": "rest", "lifting": "lift", "holding": "hold", "done": "hold"}[self.test]
        # -- the score ---------------------------------------------------------------------------------------------
        if self.locked is not None:
            score, why = 0., self.locked
        elif self.test == "done" and self.result == "passed":
            score, why = 1., "hooked, guarded, let go, and the test load bore only on the guards"
        elif both and not all_guarded:
            score, why = 0., "both fittings hooked with an edge unguarded"
        elif both and all_guarded:
            score, why = .7, "hooked and guarded, but the hands never cleared for the lift" if self.test == "waiting" \
                else "hooked and guarded; the test is running"
        else:
            score = round(.1 * n_guards + .2 * n_hooked, 2)
            why = f"{n_guards} guard(s) seated, {n_hooked} fitting(s) hooked"
        if self.test == "done":
            self.done_ticks += 1
            self.finished = self.done_ticks >= SUCCESS_S * CONTROL_HZ
        self.last = {"score": score, "why": why, "seated": {e: seated.get(e) for e in EDGE_NAMES}, "horns": horns,
                     "guards": details, "test": self.test, "result": self.result, "pull_n": round(float(m["pull_n"]), 2),
                     "bare_peak_n": {e: round(v, 3) for e, v in self.bare_peak.items()},
                     "guard_low_n": {e: (None if v is None else round(v, 3)) for e, v in self.guard_low.items()},
                     "hands_clear": bool(m["hands_clear"]), "under_bar": bool(m["under_bar"]),
                     "locked": self.locked, "success_ticks": self.done_ticks if score == 1. else 0}
        return self.last

    def end_test(self, all_guarded, both, held):
        """The test's verdict: after the full hold (``held``), or when the time limit ran out before it."""
        bare = [e for e in EDGE_NAMES if self.bare_peak[e] > L.BARE_LIMIT_N]
        if not self.guarded_at_test or not all_guarded:
            self.result = "the load was taken with an edge unguarded"
        elif bare:
            self.result = "the web bore on bare steel at " + ", ".join(bare)
        elif not both:
            self.result = "a fitting came out of its horn under load"
        elif not held:
            self.result = (f"the hook did not hold {L.PULL_N:.0f} N for {L.HOLD_S:.0f} s within "
                           f"{L.TEST_LIMIT_S:.0f} s of the lift's start")
        else:
            self.result = "passed"
        if self.result != "passed" and self.locked is None:
            self.locked = self.result
        self.test = "done"


from dataclasses import dataclass  # noqa: E402


@dataclass(frozen=True)
class RiggingScorer:
    name: str = "sling_guards_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["rigging_grade"]
        return Score(value=grade["score"],
                     explanation=f"{grade['why']}; guards seated {grade['seated']}; horns {grade['horns']}; "
                                 f"test {grade['test']} ({grade['result']}); bare-steel peaks {grade['bare_peak_n']} N")


def make_scorer():
    return RiggingScorer()
