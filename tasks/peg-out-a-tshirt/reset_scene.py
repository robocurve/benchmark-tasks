"""One deterministic native reset shared by every policy.

Only env.reset writes initial nodal/rigid state. The next five seconds advance
normal native physics with ordinary ready-position joint targets. No sensor or
assessor reading changes those targets, the duration or any object state.
"""
import time
import numpy as np

RESET_SETTLE_STEPS=100
RESET_CONTROL_HZ=20


def reset_and_settle(emb,observer=None):
    import torch
    base=emb._base
    start=time.monotonic()
    before=int(base.sim._physics_step_count)
    if observer is not None:observer("before_raw_reset",0.)
    emb._env.reset()
    if observer is not None:observer("after_raw_reset",0.)
    after_raw=int(base.sim._physics_step_count)
    ready=emb._ready.detach().clone()
    values=ready.detach().cpu().numpy()
    if values.shape!=(1,14) or not np.isfinite(values).all():
        raise ValueError('Invalid ordinary ready action during reset')
    if np.any(values[0]<emb._low) or np.any(values[0]>emb._high):
        raise ValueError('Ready action exceeds normal adapter limits')
    for step in range(1,RESET_SETTLE_STEPS+1):
        emb._env.step(ready)
        if observer is not None and (step in (1,2) or step%10==0):
            observer("settle",step/RESET_CONTROL_HZ)
    emb._t=0.
    return dict(method='raw environment reset followed by fixed native settling',
                status='completed',physics_step_count_before=before,
                physics_step_count_after_raw_reset=after_raw,
                physics_step_count_after=int(base.sim._physics_step_count),
                settle_control_steps=RESET_SETTLE_STEPS,
                settle_simulated_seconds=RESET_SETTLE_STEPS/RESET_CONTROL_HZ,
                settle_action14=values[0].tolist(),wall_seconds=time.monotonic()-start,
                controller='constant ordinary ready joint targets; no object feedback',
                writes='initial state only inside reset events; no pose or node writes during settling',
                stability='must be assessed from evidence; fixed duration is not a stability certificate')
