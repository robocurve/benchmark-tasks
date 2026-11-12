"""Known-layout plan with optional normal joint feedback; no object-state oracle."""
import numpy as np
LOG=[]

class JointTracker:
    """Bounded quasi-static integral correction through ordinary joint targets.

    No actuator gains, simulator states, object poses, contacts or images are changed.
    Integration pauses on larger target moves to avoid chasing trajectory lag.
    """
    def __init__(self):self.bias=np.zeros(14);self.previous=None
    def command(self,nominal,measured):
        from yam_arena import ik
        nominal=np.asarray(nominal,dtype=float);measured=np.asarray(measured,dtype=float)
        if nominal.shape!=(14,) or measured.shape!=(14,) or not np.isfinite(np.r_[nominal,measured]).all():
            raise ValueError('Valid original 14D joint state required')
        if self.previous is not None:
            for start in (0,7):
                s=slice(start,start+6)
                if np.max(np.abs(nominal[s]-self.previous[s]))<.005:
                    self.bias[s]=np.clip(self.bias[s]+.10*(self.previous[s]-measured[s]),-.04,.04)
        output=nominal+self.bias
        for start in (0,7):output[start:start+6]=np.clip(output[start:start+6],ik.LOWER,ik.UPPER)
        self.previous=nominal.copy()
        return output

def install(plan,tracking=False):
    from yam_arena.scripted import YamScriptedPolicy
    from inspect_robots.types import Action,ActionChunk
    from inspect_robots_yam.packing import pack,STATE_KEY
    from yam_arena.constants import READY
    import numpy as np
    old=YamScriptedPolicy.reset
    def reset(self,scene):old(self,scene);LOG.clear();self._probe_tracker=JointTracker()
    def act(self,observation):
        i=len(LOG)
        row=plan[min(i,len(plan)-1)] if plan else dict(action=np.r_[READY,1.,READY,1.].tolist(),label='ready')
        q=np.asarray(row['action'])
        if tracking:q=self._probe_tracker.command(q,observation.state[STATE_KEY])
        LOG.append(dict(step=i,label=row['label'],known_layout=True,joint_tracking=tracking,
                        joint_bias=self._probe_tracker.bias.tolist() if tracking else None))
        return ActionChunk(actions=[Action(data=pack(q[:7],q[7:]))],control_hz=20)
    YamScriptedPolicy.reset=reset;YamScriptedPolicy.act=act
