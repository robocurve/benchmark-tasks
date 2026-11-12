"""Half credit per washable part with an irreversible motor-loading veto."""
PARTS = ('blender_jug','blender_lid')

class TaskScorer:
    def __init__(self): self.reset()
    def reset(self):
        self.dwell = dict.fromkeys(PARTS, 0)
        self.motor_entered_ever = self.invalid_state_ever = False
        self.last_step = self.last_result = None
    def update(self, *, step, eligible, motor_in_dishwasher, valid=True):
        if step == self.last_step: return self.last_result
        valid = bool(valid and set(eligible)==set(PARTS)
                     and (self.last_step is None or step>self.last_step))
        consecutive = self.last_step is None or step==self.last_step+1
        self.last_step = step
        self.invalid_state_ever |= not valid
        self.motor_entered_ever |= bool(motor_in_dishwasher)
        failed = self.motor_entered_ever or self.invalid_state_ever
        for name in PARTS:
            self.dwell[name] = (self.dwell[name]+1 if consecutive else 1) if eligible.get(name,False) and not failed else 0
        delivered = [name for name in PARTS if self.dwell[name]>=20]
        score = 0. if failed else len(delivered)/2
        self.last_result = dict(score=score,success=score==1.,delivered_count=len(delivered),
            delivered=delivered,motor_entered_ever=self.motor_entered_ever,
            invalid_state_ever=self.invalid_state_ever,delivery_dwell_steps=dict(self.dwell))
        return self.last_result
    @staticmethod
    def configuration():
        return dict(control_hz=20,part_count=2,credit_per_part=.5,
            delivery_dwell_steps=20,motor_veto='Permanent zero on rack/cavity entry; reset clears',
            maximum_linear_speed_m_s=.020,maximum_angular_speed_rad_s=.20,
            minimum_tcp_to_body_clearance_m=.045,basket_boundary_tolerance_m=.002,
            support_gap_tolerance_m=.004,undelivered_parts='Zero for that part only')
