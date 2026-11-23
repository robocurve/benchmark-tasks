"""Substep adapter for the outcome scorer; operates on measured rigid-body poses."""
from native.fixture import OPEN_DEG, SEAT, STOW
from native.support_sequence import Observation, SupportSequence


class Qualification:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.sequence = None
        self.score = 0.
        self.seated = {}
        self.opened = None
        self.first_clear = None
        self.first_bearing = None
        self.worker_clear = False
        self.failed = ""
        self.tick = 0

    def observe(self, r):
        self.tick += 1
        if r['tick'] != self.tick: raise ValueError('Discontinuous physics trace')
        if self.sequence is None:
            self.sequence = SupportSequence(r['deck_world_z_m'], motion_tol=.006, dwell_seconds=2., drift_tol=.003)
            if (abs(r['relief_angle_deg']) > 3 or max(abs(v-STOW) for v in r['bar_joint_deg'].values()) > 5
                    or r['worker_gap_m'] >= 0):
                raise ValueError(f"Invalid reset: bars={r['bar_joint_deg']}, lever={r['relief_angle_deg']}, worker gap={r['worker_gap_m']}, deck={r['deck_world_z_m']}")
        seats = {s:abs(v-SEAT) <= 3.0 for s,v in r['bar_joint_deg'].items()}
        for s, placed in seats.items():
            if placed: self.seated.setdefault(s,self.tick)
        self.worker_clear = r['worker_gap_m'] >= self.sequence.worker_margin
        if self.worker_clear and self.first_clear is None: self.first_clear = self.tick
        opening = r['relief_angle_deg'] >= OPEN_DEG
        if opening and self.opened is None: self.opened = self.tick
        bearing = {s: -.005 <= d <= .0025 for s,d in r['bearing_gaps_m'].items()}
        independent = r['ram_gap_m'] >= .008
        ob = Observation(r['worker_gap_m'], seats['left'], seats['right'], opening, r['deck_world_z_m'],
                         bearing['left'], bearing['right'], independent,
                         r['external_vertical_n'] > 20., r['worker_force_n'] > 10.)
        if r['load_shift_m'] > .015:
            self.sequence.fault = 'load_displaced'
        self.score = self.sequence.observe(ob,1/240) if self.enabled else 0.
        self.failed = self.sequence.fault
        if self.sequence.stable_s and self.first_bearing is None: self.first_bearing = self.tick
        return dict(score=self.score, first_seat_ticks=dict(self.seated), first_open_tick=self.opened,
                    first_clear_tick=self.first_clear, first_bearing_tick=self.first_bearing,
                    stable_seconds=self.sequence.stable_s, worker_clear=self.worker_clear, unsafe=self.failed)
