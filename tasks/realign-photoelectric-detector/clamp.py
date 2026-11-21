"""Reduced M5 joint model; instrumented torque, never an angle success threshold."""
import math
import spec


class FrictionContact:
    """Passive elastic-perfectly-plastic friction, in SI coordinates.

    The rest coordinate follows only actual slip beyond force/stiffness. It
    never receives an alignment target, a policy command, or a scoring state.
    Native drive force remains capped by the same Coulomb resistance.
    """
    def __init__(self, stiffness, position, angular=False):
        self.stiffness = float(stiffness)
        self.rest = float(position)
        self.angular = angular

    def update(self, position, resistance):
        delta = float(position)-self.rest
        if self.angular:
            delta = (delta+math.pi)%(2*math.pi)-math.pi
        elastic = max(-resistance/self.stiffness,
                      min(resistance/self.stiffness, delta))
        self.rest = float(position)-elastic
        return self.rest

def load(clockwise_deg):
    # Datum is established by a calibrated low-range torque instrument at reset.
    # K, nut factor and friction are estimates; the final torque band is the
    # manufacturer's E39-L261 requirement. A native slipping tool demonstrates it.
    initial = (spec.RESET_TORQUE - .002)/(spec.NUT_FACTOR*spec.BOLT_DIAMETER)
    advance = math.radians(clockwise_deg)*spec.THREAD_PITCH/(2*math.pi)
    force = max(0., min(spec.MAX_CLAMP_FORCE, initial + spec.CLAMP_STIFFNESS*advance))
    torque = .002 + spec.NUT_FACTOR*spec.BOLT_DIAMETER*force
    return force, torque, spec.FRICTION*force*spec.CLAMP_FRICTION_RADIUS

def angle_delta(current, previous):
    return (current-previous+180.)%360.-180.
