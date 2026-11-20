"""The pads' trigger, run on every physics step (240 Hz), and the MIDI file it writes. Pure Python.

A hit is a held stick's collider (never a gripper, an arm, or a stick that has left its gripper) meeting a pad's
trigger zone with
  - a contact impulse of at least ``HIT_IMPULSE`` in that physics step (so a contact pair that a large contact offset
    creates before the surfaces touch never counts), and
  - an approach speed toward the zone of at least ``HIT_SPEED`` (0.5 m/s) on the physics step before the contact
    (the stick tip's velocity along the zone's normal into the pad),
  - the stick re-armed on that zone: since its last contact with it, it has been clear of it (no contact, and for a
    head its tip at least ``REARM_GAP`` above the playing surface) for ``REARM_STEPS`` steps, and
  - not within ``MASK_S`` (40 ms) of the last hit on that pad (its module's retrigger mask).
Everything else that touches a zone is recorded as a contact with the reason it was not a hit (slow, masked,
not re-armed, low impulse). Velocity is the proposal's estimated curve (``layout.midi_velocity``).
"""

import math
import struct

from . import layout as L


class Trigger:
    def __init__(self, zones, sticks, dt, *, hit_speed=L.HIT_SPEED, impulse=L.HIT_IMPULSE, mask_s=L.MASK_S,
                 rearm_gap=L.REARM_GAP, rearm_steps=L.REARM_STEPS):
        """``zones``: {zone: (pad name for the mask, note name)}; ``sticks``: names."""
        self.zones, self.sticks, self.dt = dict(zones), tuple(sticks), dt
        self.hit_speed, self.impulse, self.mask_s = hit_speed, impulse, mask_s
        self.rearm_gap, self.rearm_steps = rearm_gap, rearm_steps
        self.hits, self.contacts = [], []
        self.clear = {(s, z): rearm_steps for s in self.sticks for z in self.zones}   # steps clear since last contact
        self.touching = {(s, z): False for s in self.sticks for z in self.zones}
        self.last_hit = {}
        self.prev_speed = {(s, z): 0. for s in self.sticks for z in self.zones}
        self.contact_run = {(s, z): 0 for s in self.sticks for z in self.zones}
        self.longest_rest = {}
        self.pending = {}               # contacts still inside their scan window: key -> (row, impulse so far, steps)

    def step(self, k, impulses, approach, heights, held=None):
        """Physics step ``k`` (time k * dt, after the step). ``impulses[(stick, zone)]``: contact impulse (N s) in
        this step; ``approach[(stick, zone)]``: the tip's speed into the zone (m/s, positive toward it) at the end
        of this step; ``heights[(stick, zone)]``: the tip's clearance above the zone's playing surface (m, None for
        a rim). Returns the hits of this step."""
        new = []
        t = k * self.dt
        for key in self.clear:
            stick, zone = key
            j = impulses.get(key, 0.)
            if j > 0.:
                pad, note = self.zones[zone]
                v = self.prev_speed[key]
                started = not self.touching[key]
                self.contact_run[key] += 1
                self.longest_rest[key] = max(self.longest_rest.get(key, 0), self.contact_run[key])
                if not started and key in self.pending:       # still in the scan window: add this step's impulse
                    row, total, n = self.pending.pop(key)
                    total += j
                    if total >= self.impulse:
                        row = dict(row, impulse=round(total, 6))
                        self.contacts.remove(self.contacts[self.contacts.index(row_ref(self.contacts, row))])
                        self.last_hit[row["pad"]] = row["t"]
                        self.hits.append(row)
                        self.contacts.append(row)
                        new.append(row)
                    elif n + 1 < SCAN_STEPS:
                        self.pending[key] = (row, total, n + 1)
                if started:
                    reason = None
                    if held is not None and not held.get(stick, True):
                        reason = "not_held"            # a stick that has left its gripper never plays
                    elif j < self.impulse and v >= self.hit_speed and self.clear[key] >= self.rearm_steps and not (
                            pad in self.last_hit and t - self.last_hit[pad] < self.mask_s - 1e-9):
                        reason = "low_impulse"         # may still become a hit within the scan window
                    elif j < self.impulse:
                        reason = "low_impulse"
                    elif v < self.hit_speed:
                        reason = "slow"
                    elif self.clear[key] < self.rearm_steps:
                        reason = "not_rearmed"
                    elif pad in self.last_hit and t - self.last_hit[pad] < self.mask_s - 1e-9:
                        reason = "masked"
                    row = {"step": k, "t": round(t, 6), "stick": stick, "zone": zone, "pad": pad, "note": note,
                           "speed": round(v, 4), "impulse": round(j, 6), "velocity": L.midi_velocity(v)}
                    if reason is None:
                        self.last_hit[pad] = t
                        self.hits.append(row)
                        new.append(row)
                    else:
                        row["rejected"] = reason
                        if reason == "low_impulse" and v >= self.hit_speed and SCAN_STEPS > 1:
                            clean = {k2: v2 for k2, v2 in row.items() if k2 != "rejected"}
                            self.pending[key] = (clean, j, 1)
                    self.contacts.append(row)
                self.touching[key] = True
                self.clear[key] = 0
            else:
                self.touching[key] = False
                self.contact_run[key] = 0
                self.pending.pop(key, None)
                h = heights.get(key)
                if h is None or h >= self.rearm_gap:
                    self.clear[key] = min(self.clear[key] + 1, 10 ** 6)
            self.prev_speed[key] = approach.get(key, 0.)
        return new


SCAN_STEPS = 2       # a contact's impulse is summed over its first 2 physics steps (8 ms, as a module's scan time)


def row_ref(rows, row):
    """The contact row (rejected for low impulse) that ``row`` continues."""
    for r in reversed(rows):
        if r.get("rejected") == "low_impulse" and r["step"] == row["step"] and r["stick"] == row["stick"] \
                and r["zone"] == row["zone"]:
            return r
    raise KeyError(row)
