"""Offline geometric audit of the ACTUAL interpolated robot target trajectory.

Assumes nonslipping grasps for this audit only. It never runs physics or moves
simulated objects. PhysX contact and grasp validation remain required.
"""
import json
import numpy as np
import geometry as g
import collision as c
import task
from yam_arena.demo import ArmPlan
from yam_arena.constants import LEFT_BASE_POS,RIGHT_BASE_POS

def main():
    plans=[ArmPlan(task.demo()['right'],RIGHT_BASE_POS),ArmPlan(task.demo()['left'],LEFT_BASE_POS)]
    down=task._original(0.)
    tray=c.placed(g.tray(),[*g.TRAY_XY,task.TOP],np.eye(3))
    max_pivot_error=0.; floor=10.; failures=[]
    times=np.arange(6.,23.50001,.025)
    for t in times:
        positions=[]; rotations=[]
        for i,p in enumerate(plans):
            tcp,hand,finger,label=p.target(float(t)); r=hand@down.T
            root=tcp-hand[:,2]*(.004-.013)-r@g.grasp(i)
            positions.append(root); rotations.append(r)
        if t<=11.00001:
            max_pivot_error=max(max_pivot_error,float(np.linalg.norm(np.array(positions[0])[:2]-np.array(positions[1])[:2])))
        placed=[c.placed(g.half(i),positions[i],rotations[i]) for i in range(2)]
        hits=c.intersections(*placed,tolerance=.00001)
        for i in range(2):
            hits+=c.intersections(placed[i],tray,tolerance=.00001)
            floor=min(floor,min(s[1]-task.TOP for s in placed[i]))
        if hits: failures.append({'time_s':round(float(t),3),'contacts':hits[:5]})
    result={'kind':'geometric nominal-grasp path, no physics or arm-link collision check',
            'samples':len(times),'interval_s':.025,'maximum_assembled_pivot_mismatch_mm':1000*max_pivot_error,
            'minimum_table_clearance_mm':1000*floor,'collision_samples':len(failures),'examples':failures[:6]}
    result['passed']=not failures and floor>=-.00001 and max_pivot_error<.0001
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)

if __name__=='__main__':main()
