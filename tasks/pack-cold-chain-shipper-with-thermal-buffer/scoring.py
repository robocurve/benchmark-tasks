"""Scorer implementation for pack-cold-chain-shipper-with-thermal-buffer.

Manages:
- Instantaneous state evaluation via logic.py
- Continuous 3.0 s stability hold accumulator
- Latched contact freezing hazard enforcement (caps score at 0.25)
- The closing step: lid seated flat on the rim with both latches engaged (needed, with the correct stack, for 1.00)
The score is the state at the current step, never a best score: undoing any step lowers it.
"""


from dataclasses import dataclass, field
from typing import Any

import geometry as g
import logic


@dataclass
class ScoreResult:
    score: float
    success: bool
    details: dict[str, Any] = field(default_factory=dict)


class Scorer:
    def __init__(self, hold_s: float = g.HOLD_DWELL_S, hz: float = g.CONTROL_HZ):
        self.hold_s = hold_s
        self.dt = 1.0 / hz
        self.reset()

    def reset(self) -> None:
        self.dwell_s = 0.0
        self.latched_violation = False
        self.last_state = logic.State()

    def update(
        self,
        brick_pos,
        brick_rot=None,
        brick_quat=None,
        brick_vel=None,
        buffer_pos=None,
        buffer_rot=None,
        buffer_quat=None,
        buffer_vel=None,
        carton_pos=None,
        carton_rot=None,
        carton_quat=None,
        carton_vel=None,
        arm_tcps=None,
        lid_pos=None,
        lid_rot=None,
        lid_quat=None,
        lid_vel=None,
        latch_press=None,
        shipper_pos=None,
    ) -> ScoreResult:
        st, self.latched_violation = logic.evaluate_step(
            brick_pos=brick_pos,
            brick_rot=brick_rot,
            brick_quat=brick_quat,
            brick_vel=brick_vel,
            buffer_pos=buffer_pos,
            buffer_rot=buffer_rot,
            buffer_quat=buffer_quat,
            buffer_vel=buffer_vel,
            carton_pos=carton_pos,
            carton_rot=carton_rot,
            carton_quat=carton_quat,
            carton_vel=carton_vel,
            arm_tcps=arm_tcps,
            latched_violation=self.latched_violation,
            lid_pos=lid_pos,
            lid_rot=lid_rot,
            lid_quat=lid_quat,
            lid_vel=lid_vel,
            latch_press=latch_press,
            shipper_pos=shipper_pos,
        )
        self.last_state = st

        # If freezing hazard violation latched, cap score at 0.25 and no success
        if self.latched_violation:
            self.dwell_s = 0.0
            score = 0.25 if st.brick_placed else 0.0
            return ScoreResult(
                score=score,
                success=False,
                details={
                    "brick_placed": st.brick_placed,
                    "buffer_placed": False,
                    "carton_placed": False,
                    "safe_order": False,
                    "lid_seated": st.lid_seated,
                    "lid_closed": False,
                    "latched_violation": True,
                    "reason": "direct_freeze_contact_violation",
                    "dwell_s": 0.0,
                },
            )

        # Full success prerequisites:
        # 1. Gel brick seated flat
        # 2. Thermal buffer barrier seated flat
        # 3. Medication carton seated flat
        # 4. Safe vertical order (brick < buffer < carton)
        # 5. All tilts < 4 deg
        # 6. Arms withdrawn clear to standby
        # 7. No direct contact violation
        # 8. Lid seated flat on the rim with both latches engaged over its end edges
        ready_for_hold = bool(
            st.brick_placed
            and st.buffer_placed
            and st.carton_placed
            and st.safe_order
            and st.lid_closed
            and st.tilt_ok
            and st.arms_clear
            and not self.latched_violation
        )

        if ready_for_hold:
            self.dwell_s += self.dt
        else:
            self.dwell_s = 0.0

        if ready_for_hold and (self.dwell_s + 1e-4) >= self.hold_s:
            final_score = 1.00
            final_success = True
        elif st.carton_placed and st.buffer_placed and st.brick_placed:
            final_score = 0.75
            final_success = False
        elif st.buffer_placed and st.brick_placed:
            final_score = 0.50
            final_success = False
        elif st.brick_placed:
            final_score = 0.25
            final_success = False
        else:
            final_score = 0.00
            final_success = False

        return ScoreResult(
            score=final_score,
            success=final_success,
            details={
                "brick_placed": st.brick_placed,
                "buffer_placed": st.buffer_placed,
                "carton_placed": st.carton_placed,
                "safe_order": st.safe_order,
                "lid_seated": st.lid_seated,
                "latch_pos_engaged": st.latch_pos_engaged,
                "latch_neg_engaged": st.latch_neg_engaged,
                "lid_closed": st.lid_closed,
                "tilt_ok": st.tilt_ok,
                "arms_clear": st.arms_clear,
                "latched_violation": self.latched_violation,
                "dwell_s": round(self.dwell_s, 3),
            },
        )
