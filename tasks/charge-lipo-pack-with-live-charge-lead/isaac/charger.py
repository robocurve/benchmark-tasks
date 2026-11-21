"""Charger front panel: debounced key switches and the state model of the preset LiPo BALANCE 2.2 A 3S program.
Engine-independent. Ported from the balance-charge task's charger model with the program fixed (the program is
preset, so only the start/stop workflow is modelled):

* program screen 'LiPo BALANCE CHG / 2.2A 11.1V(3S)';
* ENTER held 3 s starts the battery check: with the main (XT60) and balance leads connected the screen shows the
  detected (R) and set (S) cell counts and waits for ENTER (confirm, starts the charge) or STOP (back);
  without them it shows ERROR / CHECK MAIN PORT or BALANCE CONNECT until STOP;
* while charging, losing the main or balance connection shows ERROR / CONNECTION BREAK; STOP ends the charge;
* DEC / INC and a short ENTER on the program screen change nothing (program locked).
A key counts only when its plunger is pushed past the switch point (travel measured in the simulator) for 10 ms.
"""
from dataclasses import dataclass, field

KEYS = ('STOP', 'DEC', 'INC', 'ENTER')
CLOSE_TRAVEL, OPEN_TRAVEL, DEBOUNCE_S = 0.0008, 0.0006, 0.010  # m of 1.2 mm travel, hysteresis; s
SHORT_PRESS_S, LONG_PRESS_S = 0.04, 3.0
PROGRAM = dict(chemistry='LiPo', mode='BALANCE CHG', cells=3, current_tenths=22)


@dataclass
class KeySwitches:
    contact: dict = field(default_factory=lambda: {k: False for k in KEYS})
    stable_for: dict = field(default_factory=lambda: {k: 0.0 for k in KEYS})
    closed: dict = field(default_factory=lambda: {k: False for k in KEYS})

    def sample(self, dt, travel):
        for k in KEYS:
            x = float(travel.get(k, 0.0))
            raw = x >= CLOSE_TRAVEL if not self.contact[k] else x > OPEN_TRAVEL
            if raw != self.contact[k]:
                self.contact[k] = raw
                self.stable_for[k] = 0.0
            else:
                self.stable_for[k] += dt
            if self.stable_for[k] >= DEBOUNCE_S - 1e-12:
                self.closed[k] = self.contact[k]
        return dict(self.closed)


@dataclass
class Charger:
    page: str = 'program'
    main: bool = False
    balance: bool = False
    error: str = ''
    seconds: float = 0.0
    elapsed: float = 0.0
    battery_v: float = 11.4
    held: dict = field(default_factory=lambda: {k: 0.0 for k in KEYS})
    down: dict = field(default_factory=lambda: {k: False for k in KEYS})
    long_fired: bool = False
    events: list = field(default_factory=list)

    @property
    def running(self):
        return self.page == 'charging'

    def event(self, key, long=False):
        self.events.append(dict(t=round(self.elapsed, 4), key=key, long=long, page=self.page))
        if key == 'STOP':
            self.page, self.error = 'program', ''
        elif key == 'ENTER' and long and self.page == 'program':
            if not self.main:
                self.page, self.error = 'error', 'CHECK MAIN PORT'
            elif not self.balance:
                self.page, self.error = 'error', 'BALANCE CONNECT'
            else:
                self.page = 'confirm'
        elif key == 'ENTER' and not long and self.page == 'confirm':
            self.page, self.seconds = 'charging', 0.0

    def step(self, dt, closed, main, balance):
        self.elapsed += dt
        self.main, self.balance = bool(main), bool(balance)
        for k in KEYS:
            if closed.get(k, False):
                self.held[k] += dt
                if k == 'ENTER' and self.held[k] >= LONG_PRESS_S - 1e-8 and not self.long_fired:
                    self.event(k, True)
                    self.long_fired = True
            elif self.down[k]:
                if self.held[k] >= SHORT_PRESS_S and not (k == 'ENTER' and self.long_fired):
                    self.event(k)
                self.held[k] = 0.0
                if k == 'ENTER':
                    self.long_fired = False
            self.down[k] = bool(closed.get(k, False))
        if self.page == 'charging':
            if not (self.main and self.balance):
                self.page, self.error = 'error', 'CONNECTION BREAK'
            else:
                self.seconds += dt

    def display(self):
        if self.page == 'confirm':
            return ('R:3SER S:3SER', 'CONFIRM(ENTER)')
        if self.page == 'error':
            return ('ERROR', self.error)
        if self.page == 'charging':
            s = int(self.seconds)
            return (f'LP3s 2.2A {self.battery_v:.2f}V', f'BAL {s // 60:03}:{s % 60:02} {int(2.2 * self.seconds / 3.6):05}')
        return ('LiPo BALANCE CHG', '2.2A 11.1V(3S)')
