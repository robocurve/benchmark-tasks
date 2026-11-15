"""Extend an ordinary peg alignment while preserving the total action budget."""
import copy

def extend_alignment(plan,servos,current,step,extra=20,maximum=2400):
    if step!=current.end+1 or extra<=0 or len(plan['frames'])+extra>maximum:
        raise RuntimeError('No remaining ordinary-action budget for peg alignment')
    hold=copy.deepcopy(plan['frames'][step-2]);hold['stage']='continue measured peg alignment before releasing handles'
    plan['frames'][step-1:step-1]=[copy.deepcopy(hold) for _ in range(extra)]
    plan['action_count']=len(plan['frames'])
    for _,servo in servos:
        if servo is current:servo.end+=extra
        elif servo.start>=step:servo.start+=extra;servo.end+=extra
    for config in plan.get('clip_servos',[]):
        start,end=config['window']
        if end==step-1:config['window'][1]+=extra
        elif start>=step:config['window']=[start+extra,end+extra]
