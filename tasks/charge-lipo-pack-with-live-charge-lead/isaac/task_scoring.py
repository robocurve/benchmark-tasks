"""Scorer of the merged LiPo charging task. Pure function of measured signals, engine-independent (check_logic.py).

Live = the XT60 halves are less than 10 mm apart (still telescoped). Latched 0 for the whole trial if at any time:
  - a gripper link touches a banana plug or its wire within 5 cm of the plug while live,
  - a short while live (pin-pin < 0.5 mm, pin to the other jack / case / other contacts < 0.1 mm),
  - a banana plug is seated in the other colour's jack,
  - the AC or balance plug backs out of its port,
  - the charger or riser moves > 10 mm or turns > 5 deg, or an object falls off the table,
  - the XT60 comes apart while the charger runs.
Otherwise the level reached in the current state (the final value is the score at the end of the trial):
  0.1  XT60 apart (not live),
  0.25 both banana plugs seated in their own jacks,
  0.5  + XT60 fully mated and no error on the LCD,
  0.75 + the charger running the balance charge,
  1.0  + running for 5 s with no key pressed and every fingertip >= 3 cm from charger, plugs and pack throughout.
"""

HOLD_S = 5.0
MOVE_M, TURN_DEG = 0.010, 5.0


class ChargeScorer:
    def __init__(self):
        self.veto = None
        self.hold = 0.0
        self.score = 0.0

    def _veto(self, m, intact):
        if m['live'] and m['gripper_contacts']:
            c = m['gripper_contacts'][0]
            return f"gripper touched {c['body']} while live"
        if m['live'] and m['metal_contacts']:
            c = m['metal_contacts'][0]
            return f"live short: {c['geom']} touched {c['other']} ({c['kind']})"
        if m['crossed_positive']['connected'] or m['crossed_negative']['connected']:
            return 'plug seated in the wrong jack'
        if not m['power']['connected']:
            return 'preset AC cord unseated'
        if not m['balance']['seated']:
            return 'preset 3S balance lead unseated'
        if not intact:
            return 'object left the table'
        for n in ('charger_moved', 'riser_moved'):
            if m[n]['shift_m'] > MOVE_M or m[n]['turn_deg'] > TURN_DEG:
                return f"{n.split('_')[0]} moved more than 10 mm / 5 deg"
        if m['xt60_apart_while_running']:
            return 'XT60 came apart while the charger was running'
        return None

    def step(self, dt, m, *, intact=True):
        """One control step. m["live"] and the contact/key flags cover all its physics substeps; m["xt60"] is the current state."""
        if self.veto is None:
            self.veto = self._veto(m, intact)
        both = m['positive']['connected'] and m['negative']['connected']
        mated = both and m['xt60']['seated'] and not m['lcd_error']
        running = mated and m['running']
        self.hold = self.hold + dt if (running and not m['keys_pressed'] and m['clear'] and self.veto is None) else 0.0
        if self.veto is not None:
            self.score = 0.0
        elif running and self.hold >= HOLD_S - 1e-8:
            self.score = 1.0
        elif running:
            self.score = 0.75
        elif mated:
            self.score = 0.5
        elif both:
            self.score = 0.25
        elif not m['xt60']['live']:
            self.score = 0.1
        else:
            self.score = 0.0
        return self.score
