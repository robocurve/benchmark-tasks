"""Scheduled straight-down presses, with the modifier settled before the other cap descends."""
import os
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED
import geometry as g

READY={'left':(.10,.14,.12),'right':(.10,-.14,.12)}
PRESS=.039
HOVER=.090

def w(t,p,f=FINGER_CLOSED,label='move'):
    return Waypoint(t*float(os.getenv('DEMO_TIME_SCALE','1')), (p[0],p[1],g.TABLE_TOP+p[2]),f,label,yaw=0.)

def total(plan): return sum(p.duration for p in plan)
def wait(plan,t):
    last=plan[-1]
    plan.append(Waypoint(t,last.pos,last.finger,'wait',yaw=0.))
def sync(left,right):
    if total(left)<total(right): wait(left,total(right)-total(left))
    elif total(right)<total(left): wait(right,total(left)-total(right))
def at(key,z): return (*g.KEYS[key],z)
def tap(plan,key):
    plan += [w(.8,at(key,HOVER),label='above '+key),w(1.2,at(key,PRESS),label='press '+key),w(.6,at(key,PRESS),label='dwell'),w(.9,at(key,HOVER),label='release '+key),w(.5,at(key,HOVER),label='allow return')]

def make_settle_plan():
    return {a:[w(10,p,label='ready')] for a,p in READY.items()}

def make_demo_plan(ablation=None):
    left=[w(2,at('shift',HOVER),label='above modifier'),w(1.2,at('shift',PRESS),label='press modifier'),w(.8,at('shift',PRESS),label='settle modifier')]
    right=[w(2,at('a',HOVER),label='above character')]
    sync(left,right)
    tap(right,'a'); sync(left,right)
    left += [w(1.,at('shift',HOVER),label='release modifier'),w(.8,at('shift',HOVER),label='return')]
    sync(left,right)
    for key in ('b','7','enter'): tap(right,key)
    sync(left,right)
    left += [w(1.5,at('ctrl',HOVER),label='above second modifier'),w(1.2,at('ctrl',PRESS),label='press second modifier'),w(.8,at('ctrl',PRESS),label='settle modifier')]
    right += [w(1.5,at('l',HOVER),label='above final character')]
    sync(left,right)
    tap(right,'l'); sync(left,right)
    left += [w(1,at('ctrl',HOVER),label='release modifier'),w(.8,at('ctrl',HOVER),label='return')]
    sync(left,right)
    for arm,plan in [('left',left),('right',right)]:
        plan += [w(2,READY[arm],label='withdraw'),w(.8,READY[arm],FINGER_OPEN,'open hands'),w(2,READY[arm],FINGER_OPEN,'stillness')]
    if ablation=='left': left=[w(total(left)/float(os.getenv('DEMO_TIME_SCALE','1')),READY['left'],FINGER_OPEN,'idle')]
    if ablation=='right': right=[w(total(right)/float(os.getenv('DEMO_TIME_SCALE','1')),READY['right'],FINGER_OPEN,'idle')]
    return {'left':left,'right':right}

def plans():
    return make_settle_plan() if os.getenv('SETTLE_ONLY') else make_demo_plan(os.getenv('ABLATION_ARM','left') if os.getenv('ABLATION') else None)
