"""Strictly ordered scorer using measured geometry and brief manipulation contacts."""

from collections import deque
import math
from model.human_safety import reading
from native.fixture_v21 import grip, handle, CART_ROOT

SIDES = ("left", "right")


def contact(pairs, side, fixture, child=None, tip=None):
    for a, b in pairs:
        for hand, body in ((a, b), (b, a)):
            if (
                "/" + side.title() + "Arm/" in hand
                and (not tip or "/tip_" + tip in hand)
                and "/cart_fixture/" + fixture + "/" in body
                and (not child or body.rsplit("/", 1)[-1] in child)
            ):
                return True
    return False


def forbidden_contact(pairs):
    """Only fingers may contact the assigned prop grip or the rotary control."""
    for a, b in pairs:
        for arm, other in ((a, b), (b, a)):
            side = next((s for s in SIDES if "/" + s.title() + "Arm/" in arm), None)
            if side is None:
                continue
            tip = any("/tip_" + j + "/" in arm for j in SIDES)
            if "/" + ("Right" if side == "left" else "Left") + "Arm/" in other:
                return "inter-arm contact"
            if "/cart_fixture/" in other:
                part = other.split("/cart_fixture/", 1)[1].split("/")
                if tip and (part[0] == side + "_grip" or part[0] == "valve_knob"):
                    continue
                return side + " arm contacted " + "/".join(part)
            if any(
                "/" + name + "/" in other
                for name in (
                    "table",
                    "floor",
                    "stand_mast",
                    "camera_housing",
                    "stand_foot",
                    "stand_arm",
                )
            ):
                if "/link1/" not in arm and "/base/" in arm and "/table/" in other:
                    continue
                return side + " arm contacted the station"
    return None


class Qualification:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.tick = 0
        self.jaw = {s: {} for s in SIDES}
        self.pinched = set()
        self.seated = {}
        self.valve_touch = None
        self.valve_pinched_at_open = False
        self.valve_jaw = {s: {} for s in SIDES}
        self.forbidden = None
        self.last_angles = {}

        self.opened = None
        self.failed = None
        self.proof_command = False
        self.score = 0
        self.window = deque(maxlen=480)

    def observe(self, r):
        self.tick += 1
        if r["tick"] != self.tick:
            raise ValueError("Discontinuous physics substeps")
        tick = self.tick
        a = r["prop_angle_deg"]
        v = r["relief_angle_deg"]
        deck = r["deck_world_z_m"]
        gaps = r["bearing_gaps_m"]
        if (
            set(a) != set(SIDES)
            or set(gaps) != set(SIDES)
            or not all(
                map(
                    math.isfinite,
                    [
                        *a.values(),
                        *gaps.values(),
                        v,
                        deck,
                        r["ram_relative_z_m"],
                        r["crosshead_roll_deg"],
                    ],
                )
            )
        ):
            raise ValueError("Missing or nonfinite fixture measurement")
        self.last_angles = dict(a)
        self.forbidden = self.forbidden or forbidden_contact(r["collider_pairs"])
        if self.enabled:
            if self.forbidden:
                self.failed = self.failed or self.forbidden
            if tick == 1 and (any(abs(x - 90) > 4 for x in a.values()) or abs(v) > 3):
                self.failed = "invalid reset"
            if set(r["jaw_aperture_m"]) != set(SIDES) or not all(
                math.isfinite(x) for x in r["jaw_aperture_m"].values()
            ):
                raise ValueError("Invalid gripper aperture")
            if len(r["crate_relative_m"]) != 3 or not all(
                map(math.isfinite, r["crate_relative_m"])
            ):
                raise ValueError("Invalid load position")
            h = r["hand_points_w"]
            if set(h) != {
                s + "_" + n for s in SIDES for n in ("gripper", "tip_left", "tip_right")
            } or any(len(x) != 3 or not all(map(math.isfinite, x)) for x in h.values()):
                raise ValueError("Missing hand feedback")
            if (
                r["unsafe_observed"]
                or r["external_load_contact"]
                or reading(deck).deck_underside_to_hand_top_m < 0.04
                or math.dist(r["crate_relative_m"], (0.06, 0, 0.125)) > 0.015
                or min(
                    math.dist(x, (CART_ROOT[0] - 0.11, 0.235, 0.610))
                    for x in h.values()
                )
                < 0.098
            ):
                self.failed = (
                    self.failed or "unsafe contact, hand clearance or load displacement"
                )
            for s in SIDES:
                for tip in SIDES:
                    if contact(
                        r["collider_pairs"],
                        s,
                        s + "_grip",
                        ("reachable_grasp_bail",),
                        tip,
                    ):
                        self.jaw[s][tip] = tick
                both = all(tick - self.jaw[s].get(j, -1000) <= 12 for j in SIDES)
                if (
                    both
                    and tick in self.jaw[s].values()
                    and r["jaw_aperture_m"][s] < 0.030
                ):
                    self.failed = self.failed or "pad aperture too narrow for the neck"
                if both and a[s] > 65 and 0.030 <= r["jaw_aperture_m"][s] <= 0.038:
                    self.pinched.add(s)
                if (
                    s not in self.seated
                    and not both
                    and any(tick - self.jaw[s].get(j, -1000) > 30 for j in SIDES)
                ):
                    self.pinched.discard(s)
                if (
                    s in self.pinched
                    and both
                    and 0.030 <= r["jaw_aperture_m"][s] <= 0.038
                    and -2 < a[s] < -0.01
                ):
                    self.seated.setdefault(s, tick)
            locked = len(self.seated) == 2 and all(-2 < x < 0.1 for x in a.values())
            if v > 5 and not locked:
                self.failed = self.failed or "relief moved before both props seated"
            for s in SIDES:
                for tip in SIDES:
                    if contact(
                        r["collider_pairs"], s, "valve_knob", ("valve_grip",), tip
                    ):
                        self.valve_jaw[s][tip] = tick
                if 0.023 <= r["jaw_aperture_m"][s] <= 0.034 and all(
                    tick - self.valve_jaw[s].get(t, -1000) <= 12 for t in SIDES
                ):
                    self.valve_touch = tick
            if v >= 45 and self.opened is None:
                self.opened = tick
                self.valve_pinched_at_open = tick - (self.valve_touch or -1000) <= 24
                if (
                    not locked
                    or max(self.seated.values(), default=tick) >= tick
                    or not self.valve_pinched_at_open
                ):
                    self.failed = self.failed or "unearned valve opening"
            targets = [grip(s, a[s]) for s in SIDES] + [handle(v)]
            clear = all(math.dist(p, j) > 0.135 for p in h.values() for j in targets)
            if (
                not self.failed
                and locked
                and self.opened
                and clear
                and r["ram_relative_z_m"] < -0.035
            ):
                self.proof_command = True
            if self.proof_command and (not locked or not clear):
                self.failed = self.failed or "handoff lost"
            bearing = (
                self.proof_command
                and r["ram_relative_z_m"] <= -0.165
                and 0.693 <= deck <= 0.733
                and abs(r["crosshead_roll_deg"]) < 2
                and all(-0.004 <= g <= 0.010 for g in gaps.values())
            )
            if bearing and not self.failed:
                self.window.append(deck)
            else:
                self.window.clear()
            self.score = int(
                len(self.window) == 480 and max(self.window) - min(self.window) <= 0.003
            )
        return dict(
            score=self.score,
            first_seat_ticks=dict(self.seated),
            first_open_tick=self.opened,
            stable_substeps=len(self.window),
            independent_release=self.proof_command,
            unsafe=self.failed,
        )
