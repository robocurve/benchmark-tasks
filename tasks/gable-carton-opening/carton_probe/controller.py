"""Known-layout controller using only ordinary joint commands and joint feedback."""
import numpy as np


def install(plan):
    from yam_arena.scripted import YamScriptedPolicy
    from yam_arena import ik
    from inspect_robots.types import Action,ActionChunk
    from inspect_robots_yam.packing import pack,STATE_KEY
    original=YamScriptedPolicy.reset
    def reset(self,scene):
        original(self,scene);self._i=0;self._bias=np.zeros(14);self._previous=None
    def act(self,observation):
        row=np.array(plan[min(self._i,len(plan)-1)]['action'],float)
        measured=np.asarray(observation.state[STATE_KEY],float)
        if self._previous is not None:
            for start in (0,7):
                s=slice(start,start+6)
                if np.max(abs(row[s]-self._previous[s]))<.005:
                    self._bias[s]=np.clip(self._bias[s]+.10*(self._previous[s]-measured[s]),-.04,.04)
        command=row+self._bias
        for start in (0,7):command[start:start+6]=np.clip(command[start:start+6],ik.LOWER,ik.UPPER)
        self._previous=row;self._i+=1
        return ActionChunk(actions=[Action(data=pack(command[:7],command[7:]))],control_hz=20)
    YamScriptedPolicy.reset=reset;YamScriptedPolicy.act=act
