"""Scorer for the t8-tube task (pure Python, no simulator): take a dead 2-ft T8 lamp down from the strip fixture under
the shelf and lay it in the lamp-recycling carton on the bench.

Inputs each control step (20 Hz), all read from the simulator by the task kit (``isaac/tubekit.py``):
* ``holder``: the lampholder model's state: 'locked' (pins in the rotors, turned less than 87.4 deg), 'slot' (turned
  open: pins lined up with the slots, which open downward; nothing holds the lamp up), 'out' (both pin pairs below
  the slots) or 'broken' (a lampholder was loaded past its strength for 0.1 s);
* ``in_carton``: both end caps inside the carton and at its floor height;
* ``on_bench``: both end caps at tabletop height with the whole tube outside the carton.
* ``speed``: maximum rigid-body motion proxy including end-cap speeds (m/s);
* ``contact``: dict of robot-contact flags for 'lamp', 'carton', 'fixture' (PhysX contact forces between the robot's
  links and those bodies: collider identity, not positions);
* ``dropped``: the kit's drop monitor fired: the lamp was out of the rotors, no robot link touched it, and it moved
  faster than 0.8 m/s (a free fall of more than ~3 cm): counted as a broken glass lamp;
* ``broken``: a lampholder broke.

Full credit requires four continuous seconds of stable, released placement on the carton bottom.
A fully removed tube lying clear of the carton on the bench earns 0.5 under the same four-second
hold. Changing surfaces, renewed contact or motion resets the timer. Both damage vetoes latch.
"""

from __future__ import annotations

CLEAR_S = 4.0
REST_SPEED = 0.02


class Scorer:
    def __init__(self):
        self.t = 0.0
        self.veto = None
        self.veto_t = None
        self.clear_s = 0.0
        self.surface = None
        self.out_t = None
        self.carton_t = None
        self.last = {}

    def _veto(self, why):
        if self.veto is None:
            self.veto, self.veto_t = why, round(self.t, 3)

    def step(self, dt, holder, in_carton, speed, contact, dropped=False, broken=False, on_bench=False):
        self.t += dt
        if dropped:
            self._veto("lamp_dropped")
        if broken or holder == "broken":
            self._veto("lampholder_broken")
        out = holder == "out"
        if out and self.out_t is None:
            self.out_t = round(self.t, 3)
        lamp_touch = bool(contact.get("lamp", False))
        surface = "carton" if in_carton else "bench" if on_bench else None
        stable = out and surface is not None and not lamp_touch and 0 <= speed < REST_SPEED
        if not stable or self.surface != surface:
            self.clear_s = 0.0
        if stable:
            self.clear_s += dt
        self.surface = surface if stable else None
        if out and in_carton and self.carton_t is None:
            self.carton_t = round(self.t, 3)
        level = 0.0
        if self.veto is None and stable and self.clear_s >= CLEAR_S - 1e-9:
            level = 1.0 if in_carton else 0.5
        self.last = dict(
            level=level,
            veto=self.veto,
            veto_t=self.veto_t,
            holder=holder,
            out=out,
            out_t=self.out_t,
            in_carton=bool(in_carton),
            carton_t=self.carton_t,
            on_bench=bool(on_bench),
            surface=self.surface,
            speed=round(float(speed), 4),
            contact={k: bool(v) for k, v in contact.items()},
            clear_s=round(self.clear_s, 3),
        )
        return level
