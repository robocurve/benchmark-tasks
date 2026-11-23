"""Pure bounded joint correction and conservative mesh reference allowance."""

import numpy as np


def bounded_joint_pi_step(bias, reference, actual, dt, allowed=True):
    """Return a fresh six-joint bias, integrating only when allowed.

    Angles are radians and dt is seconds. The reference is the unbiased joint
    reference, not the command with the previous bias already added.
    """
    vectors = []
    for name, value in (("bias", bias), ("reference", reference), ("actual", actual)):
        vector = np.asarray(value, dtype=float)
        if vector.shape != (6,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} must be a finite vector of shape (6,)")
        vectors.append(vector)
    if np.ndim(dt) != 0 or not np.isfinite(dt) or not 0.0 <= dt <= 0.1:
        raise ValueError("dt must be finite and between 0 and 0.1 seconds")
    bias, reference, actual = vectors
    if np.any(np.abs(bias) > 0.02):
        raise ValueError("bias must be within +/-0.02 radians")
    if not allowed or dt == 0:
        return bias.copy() if not allowed else np.clip(bias, -0.02, 0.02)
    with np.errstate(over="ignore"):
        increment = np.clip(2.0 * dt * (reference - actual), -0.00025, 0.00025)
    return np.clip(bias + increment, -0.02, 0.02)


def mesh_reference_step_allowance(budget_m, error_m):
    """Return half the remaining mesh slack; a nonpositive budget allows none.

    The caller supplies fresh geometric bounds and checks actual motion. This
    triangle-inequality allowance does not predict future physical tracking.
    """
    if np.ndim(budget_m) != 0 or not np.isfinite(budget_m):
        raise ValueError("budget_m must be a finite scalar")
    if np.ndim(error_m) != 0 or not np.isfinite(error_m) or error_m < 0:
        raise ValueError("error_m must be a finite nonnegative scalar")
    return 0.5 * max(0.0, float(budget_m) - float(error_m))


def bounded_joint_pid_step(bias, reference, actual, velocity, dt):
    """Conditional trajectory I and transient native-velocity damping.

    Return integral bias, D correction, capped total correction, and the
    conditional integration mask. D is never stored in the integral bias.
    """
    proposed = bounded_joint_pi_step(bias, reference, actual, dt)
    bias = np.asarray(bias,dtype=float)
    error = np.asarray(reference,dtype=float)-np.asarray(actual,dtype=float)
    velocity = np.asarray(velocity,dtype=float)
    if velocity.shape != (6,) or not np.isfinite(velocity).all():
        raise ValueError('native joint velocity must be finite shape (6,)')
    step = proposed-bias
    toward = velocity*error > 0
    mask = ~(toward & (np.abs(velocity)*.1 >= np.abs(error)))
    # Error-directed integration also brakes motion away from the reference.
    step = np.where(mask,step,0.)
    updated = np.clip(bias+step,-.02,.02)
    damping = np.clip(-.05*velocity,-.002,.002)
    return updated,damping,np.clip(updated+damping,-.02,.02),mask
