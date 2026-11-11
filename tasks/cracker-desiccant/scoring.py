"""Per-cracker credit with an irreversible non-food packet veto."""
CRACKERS = tuple(f"cracker_{i}" for i in range(1, 7))

class TaskScorer:
    def __init__(self): self.reset()
    def reset(self):
        self.dwell = dict.fromkeys(CRACKERS, 0)
        self.packet_entered_ever = self.invalid_state_ever = False
        self.last_step = self.last_result = None
    def update(self, *, step, eligible, packet_in_basket, valid=True):
        if step == self.last_step: return self.last_result
        valid = bool(valid and set(eligible)==set(CRACKERS)
                     and (self.last_step is None or step>self.last_step))
        consecutive = self.last_step is None or step==self.last_step+1
        self.last_step = step
        self.invalid_state_ever |= not valid
        self.packet_entered_ever |= bool(packet_in_basket)
        failed = self.packet_entered_ever or self.invalid_state_ever
        for name in CRACKERS:
            self.dwell[name] = (self.dwell[name]+1 if consecutive else 1) if eligible.get(name,False) and not failed else 0
        delivered = [name for name in CRACKERS if self.dwell[name]>=20]
        score = 0. if failed else len(delivered)/6
        self.last_result = dict(score=score,success=score==1.,delivered_count=len(delivered),
            delivered=delivered,packet_entered_ever=self.packet_entered_ever,
            invalid_state_ever=self.invalid_state_ever,delivery_dwell_steps=dict(self.dwell))
        return self.last_result
    @staticmethod
    def configuration():
        return dict(control_hz=20,cracker_count=6,credit_per_cracker=1/6,
            delivery_dwell_steps=20,packet_veto='Permanent zero on basket entry; reset clears',
            maximum_linear_speed_m_s=.020,maximum_angular_speed_rad_s=.20,
            minimum_tcp_to_body_clearance_m=.045,basket_boundary_tolerance_m=.002,
            support_gap_tolerance_m=.004,undelivered_crackers='Zero for that cracker only')
