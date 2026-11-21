"""Observable optical proof with retained alignment and terminal dwell."""
from dataclasses import dataclass
import math
import spec

@dataclass
class Evidence:
    beam_on: bool
    card_blocks: bool
    clamp_closed: bool
    hands_clear: bool
    card_parked: bool
    tool_clear: bool
    sensor_yaw_deg: float
    chassis_tilt_deg: float
    inside_workspace: bool
    intact: bool
    shortcut: bool
    card_released: bool
    tightening_torque_nm: float = .10
    peak_tightening_torque_nm: float = .10
    tool_deflection_deg: float = 0.
    nut_motion_deg: float = 0.
    nut_angle_deg: float = 0.
    tool_engaged: bool = False
    torque_path_clear: bool = True

class Scorer:
    def __init__(self):
        self.score=0.;self.dwell=0;self.invalid=False
        self.aligned=0;self.blocked=0;self.proof=False;self.setting=None
        self.torque_confirmed=False;self.torque_steps=0;self.slip=0.;self.torqued_angle=None
        self.witness_nut_min=None;self.witness_nut_max=None
    def update(self,e):
        if not all(math.isfinite(v) for v in [e.sensor_yaw_deg,e.chassis_tilt_deg,e.tightening_torque_nm,e.peak_tightening_torque_nm,e.tool_deflection_deg,e.nut_motion_deg,e.nut_angle_deg]):self.invalid=True
        self.invalid |= e.chassis_tilt_deg>15 or not e.inside_workspace or not e.intact or e.shortcut or e.peak_tightening_torque_nm>spec.TORQUE_MAX
        if self.invalid:self.score=0.;self.dwell=0;return 0.
        if self.torqued_angle is not None and e.nut_angle_deg>self.torqued_angle+1.:
            self.torque_confirmed=False;self.torqued_angle=None
        indicated=-math.radians(e.tool_deflection_deg)*spec.DRIVER_SPRING_NM_RAD
        witness=(e.tool_engaged and e.torque_path_clear and e.beam_on and e.clamp_closed
                 and spec.INDICATED_TORQUE_MIN<=indicated<=spec.INDICATED_TORQUE_MAX)
        if witness:
            if self.witness_nut_min is None:
                self.witness_nut_min=self.witness_nut_max=e.nut_angle_deg
            self.witness_nut_min=min(self.witness_nut_min,e.nut_angle_deg)
            self.witness_nut_max=max(self.witness_nut_max,e.nut_angle_deg)
            # A stationary output and calibrated pointer in the torque band
            # must be visible together for half a second.
            if self.witness_nut_max-self.witness_nut_min>1.:
                self.torque_steps=0;self.slip=0.
                self.witness_nut_min=self.witness_nut_max=e.nut_angle_deg
            self.torque_steps+=1
            if self.torque_steps>=spec.DWELL_STEPS:
                self.torque_confirmed=True;self.torqued_angle=e.nut_angle_deg
        else:
            self.torque_steps=0;self.slip=0.
            self.witness_nut_min=None;self.witness_nut_max=None
        retained=e.clamp_closed and self.torque_confirmed and e.hands_clear and e.tool_clear
        if not retained:
            self.aligned=0;self.blocked=0;self.proof=False;self.setting=None
        elif self.setting is not None and abs(e.sensor_yaw_deg-self.setting)>1.:
            self.aligned=0;self.blocked=0;self.proof=False;self.setting=None
        if retained and e.beam_on:
            self.aligned+=1
            if self.aligned>=spec.DWELL_STEPS and self.setting is None:self.setting=e.sensor_yaw_deg
        else:self.aligned=0
        if retained and self.setting is not None and e.card_blocks and not e.beam_on:
            self.blocked+=1
            if self.blocked>=spec.DWELL_STEPS:self.proof=True
        else:self.blocked=0
        complete=retained and self.proof and e.beam_on and e.card_parked and e.card_released
        self.dwell=self.dwell+1 if complete else 0
        self.score=1. if self.dwell>=spec.DWELL_STEPS else .5 if retained and e.beam_on else .25 if e.beam_on else 0.
        return self.score


def regrade(path):
    """Recompute every recorded score from evidence; fail on disagreement."""
    import json
    from pathlib import Path
    scorer = Scorer()
    count = 0
    peak = 0.
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        score = scorer.update(Evidence(**row['evidence']))
        if score != row['score'] or scorer.invalid != row['invalid'] or scorer.dwell != row['dwell']:
            raise ValueError(f"Recorded score mismatch at step {row['step']}")
        count += 1
        peak = max(peak, score)
    if not count:
        raise ValueError('Empty evidence trace')
    return {'steps': count, 'terminal_score': scorer.score, 'peak_score': peak,
            'dwell': scorer.dwell, 'invalid': scorer.invalid, 'recorded_scores_match': True}


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser(description='Regrade a completed episode evidence trace')
    parser.add_argument('trace')
    print(json.dumps(regrade(parser.parse_args().trace), indent=2))
