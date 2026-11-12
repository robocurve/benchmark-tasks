"""Unvalidated low-clearance variant; retains privileged simulator-pose feedback."""

from dataclasses import replace

from pallet_lifter.policy import PalletPolicy, make_plan

# Level, contact-supported geometry: tine top = TCP - 11 mm and the
# pallet feet are 20 mm below its plate underside. Thus bottom ~= TCP - 31 mm.
# Trial 001's 56 mm target reached only 14.08 mm actual bottom at lift end,
# then hit a 15 mm rail during carry. Add 15 mm to that TCP target, retaining
# the rest of the controller. Nominal level bottom: 40 mm (rails: 15 mm).
# This is an unvalidated target, not a physical clearance or height guarantee.
CARRY_TCP_HEIGHT = .071
# Trial 002 seated near y=-.131 m, then withdrew with the tines still high
# enough to hit pallet feet. Centre the placement and lower below the feet.
# Level tine top ~= TCP - 11 mm: 23 mm gives a nominal 12 mm tine top,
# 3 mm below feet seated on 15 mm rails. Reach/table contact need GPU checks.
DESTINATION_Y = -.120
CLEAR_TCP_HEIGHT = .023
CLEAR_HOLD_S = 2.


def make_low_clearance_plan(top):
    plan = make_plan(top)
    left = []
    for w in plan['left']:
        x, y, z = w.pos
        if w.label in ('lift pallet', 'carry pallet'):
            z = top + CARRY_TCP_HEIGHT
        if w.label in ('carry pallet', 'lower pallet', 'settle pallet',
                        'lower fork clear', 'withdraw fork', 'lift clear'):
            y = DESTINATION_Y
        if w.label in ('lower fork clear', 'withdraw fork'):
            z = top + CLEAR_TCP_HEIGHT
        left.append(replace(w, pos=(x, y, z)))
        if w.label == 'lower fork clear':
            # Same label preserves the existing payload-bias handling while
            # holding the low target before any horizontal withdrawal.
            left.append(replace(left[-1], duration=CLEAR_HOLD_S))
    plan['left'] = left
    plan['right'] = [replace(w, duration=sum(p.duration for p in left))
                     for w in plan['right']]
    return plan


class LowClearancePolicy(PalletPolicy):
    def plan_for(self, instruction, target=None):
        if self._task is None or set(self._task.names) != {
            'lifter', 'pallet', 'rests', 'handle_support', 'tine_support'
        }:
            raise ValueError('Low-clearance policy requires the pallet-lifter task')
        return make_low_clearance_plan(self._task.table_top)


def make_policy(**kwargs):
    return LowClearancePolicy(**kwargs)
