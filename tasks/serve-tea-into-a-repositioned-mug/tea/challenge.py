"""Manipulation controls for incidental contact and recovery after a missed pour."""

import numpy as np

from .model import Kitchen, smooth
from .reference import IK, Server
from .scene import DOWN, TABLE

MODES = {"incidental_counter_touch", "subtrip_contact", "spill_then_recover"}


class Challenge:
    """Modify commands only. All contact, spill and support evidence comes from the plant."""

    def __init__(self, env, mode):
        self.env = env
        self.mode = mode
        self.start_time = None
        self.touch_time = None
        self.touch_target = None
        self.contact_observed = False
        if mode == "spill_then_recover":
            self.blind_env = Kitchen("steady")
            self.blind_server = Server(self.blind_env)
        else:
            self.ik = IK(env)

    def command(self, nominal):
        e = self.env
        if self.mode == "spill_then_recover":
            if self.blind_env is None:
                return nominal
            if e.skin > 2.0:
                self.blind_env = None
                return nominal
            blind = self.blind_server.command()
            self.blind_env.action(blind)
            return blind

        now = e.d.time
        if self.start_time is None:
            if not e.report()["released_on_counter"]:
                return nominal
            self.start_time = now
            self.start = e.d.site("left_tcp").xpos.copy()
            mug = e.d.body("receiving_hand").xpos.copy()
            if self.mode == "incidental_counter_touch":
                self.approach = np.array([0.25, 0.01, TABLE + 0.070])
                self.end = self.approach + [0, 0, -0.085]
            else:
                self.approach = mug + [0.00, -0.095, 0.056]
                self.end = self.approach + [0, 0.072, 0]

        elapsed = now - self.start_time
        if self.mode == "subtrip_contact":
            high_start = self.start + [0, 0, 0.095]
            high_mug = self.approach + [0, 0, 0.13]
            if elapsed < 0.65:
                pos = self.start + (high_start - self.start) * smooth(elapsed / 0.65)
            elif elapsed < 1.85:
                pos = high_start + (high_mug - high_start) * smooth((elapsed - 0.65) / 1.2)
            elif elapsed < 2.7:
                pos = high_mug + (self.approach - high_mug) * smooth((elapsed - 1.85) / 0.85)
            elif self.touch_time is None:
                pos = self.approach + (self.end - self.approach) * np.clip((elapsed - 2.7) / 5, 0, 1)
            else:
                back = now - self.touch_time
                pos = self.touch_target + (self.approach - self.touch_target) * smooth(back / 0.65)
                pos = pos + (high_mug - self.approach) * smooth((back - 0.65) / 0.85)
        elif elapsed < 1.4:
            pos = self.start + (self.approach - self.start) * smooth(elapsed / 1.4)
        elif self.touch_time is None:
            advance = np.clip((elapsed - 1.4) / 5.0, 0.0, 1.0)
            pos = self.approach + advance * (self.end - self.approach)
        else:
            pos = self.touch_target + (self.approach - self.touch_target) * smooth((now - self.touch_time) / 0.75)

        for contact in e.d.contact:
            a, b = contact.geom
            if contact.efc_address < 0:
                continue
            if e.groups[a] != 1:
                a, b = b, a
            matches = (e.gname[b] == "counter" if self.mode == "incidental_counter_touch"
                       else e.groups[b] == 2)
            if e.groups[a] == 1 and e.gname[a].startswith("left_") and matches and not e.gname[a].endswith("base_shell"):
                self.contact_observed = True
                if self.touch_time is None:
                    self.touch_time = now
                    self.touch_target = pos.copy()
                break
        out = nominal.copy()
        out[:6] = self.ik.solve(pos, DOWN)
        out[6] = 0.35
        return out
