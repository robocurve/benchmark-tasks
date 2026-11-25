"""Represent a float32 gripper trajectory at the controller's float64 precision."""
import numpy as np


def gripper_waypoints(actions, max_step=.05):
    """Preserve each native actuator value while respecting the 20 Hz slew limit."""
    native=np.asarray(actions,dtype=np.float32)
    values=native.astype(np.float64)
    before=np.nextafter(native,np.float32(-np.inf)).astype(np.float64)
    after=np.nextafter(native,np.float32(np.inf)).astype(np.float64)
    lower=np.nextafter((before+values)/2,values)
    upper=np.nextafter((after+values)/2,values)
    lower=np.maximum(lower,0.)
    upper=np.minimum(upper,1.)
    limit=max_step-1.e-13
    for step in range(1,len(values)):
        lower[step]=np.maximum(lower[step],lower[step-1]-limit)
        upper[step]=np.minimum(upper[step],upper[step-1]+limit)
    if np.any(lower>upper):raise ValueError('gripper samples exceed native slew envelope')
    commands=values.copy()
    commands[-1]=np.clip(commands[-1],lower[-1],upper[-1])
    for step in range(len(commands)-2,-1,-1):
        lo=np.maximum(lower[step],commands[step+1]-limit)
        hi=np.minimum(upper[step],commands[step+1]+limit)
        commands[step]=np.clip(commands[step],lo,hi)
    if not np.array_equal(commands.astype(np.float32),native):
        raise ValueError('gripper waypoint changed a native target')
    return commands
