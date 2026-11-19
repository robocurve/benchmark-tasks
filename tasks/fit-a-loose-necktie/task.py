"""Finish and fit an already-threaded loose necktie on a rigid collar form.

The textile uses the disclosed reduced sliding model in material.py, not native
FEM. Run with run.py so every policy receives the same model and reset.
"""
from yam_arena.task_file import Box,Cylinder
SCENE='station'
INSTRUCTION='Finish the pre-wrapped necktie: pass the wide blade down through the front loop, snug the knot, slide it up using the narrow blade, and leave both blades neat with hands clear. The collar is a rigid practice form, not a person.'
SCORE_OVER='end'
OBJECTS=[
    Cylinder('neck',pos=(.40,0.,1.25),radius=.055,height=.10,color=(.92,.92,.90)),
    Box('stand_base',pos=(.59,0.,.7535),size=(.18,.16,.012),color=(.35,.35,.37)),
    Box('stand_post',pos=(.59,0.,.975),size=(.025,.025,.455),color=(.35,.35,.37)),
    Box('collar_support',pos=(.495,0.,1.190),size=(.210,.025,.025),color=(.35,.35,.37)),
    Box('collar_left',pos=(.353,-.055,1.225),size=(.008,.050,.055),yaw=-.45,color=(.95,.95,.92)),
    Box('collar_right',pos=(.353,.055,1.225),size=(.008,.050,.055),yaw=.45,color=(.95,.95,.92)),
    Box('centre_mark',pos=(.344,0.,1.205),size=(.001,.003,.007),color=(.08,.08,.08)),
]
def success(objects):return False

