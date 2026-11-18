"""Offline physical recipe for the disclosed, pose-assisted feasibility demo.

Reservoir compression and expected outflow produce fixed piston targets before
simulation. No actual material state, score or resulting decoration is read.
Only the separate demo servo reads tool poses, then commands ordinary motors.
"""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np
from plan import compile_plan,calibrated_hand_pose,rx,DOWN
from material import IcingParameters


def build(shells=3,*,pitch=.014,tail=.014,head_seconds=.4,tail_seconds=.7):
    from yam_arena import ik,station
    from yam_arena.constants import FINGER_OPEN
    old=compile_plan('pickup')
    frames=old['frames'];state={s:np.asarray(frames[-1]['action14'][sl],float) for s,sl in [('left',slice(0,7)),('right',slice(7,14))]}
    R=rx(np.pi/4)@np.diag((-1.,-1.,1.));axis=R[:,1]
    push_R=rx(-np.pi/4)@DOWN@np.diag((-1.,-1.,1.))
    maximum=[0.,0.];recipe_last=None
    def transition(seconds,label,goals,*,tip=None,piston=None,linear=False):
        nonlocal recipe_last
        starts={s:(*ik.tcp_pose(v[:6],station.BASES[s]),v[6]) for s,v in state.items()}
        first_recipe=recipe_last
        for j in range(1,round(seconds*20)+1):
            t=j/round(seconds*20);w=t if linear else ik.smoothstep(t);action=[]
            for s,v in state.items():
                p0,r0,f0=starts[s];p1,r1,f1=goals.get(s,(p0,r0,f0))
                point=p0+(np.asarray(p1)-p0)*w;rot=ik.slerp_matrix(r0,np.asarray(r1),w)
                q,pe,re=ik.solve(point,ik.matrix_to_quat_xyzw(rot),v[:6],station.BASES[s],iters=180,tol=1e-6)
                if pe>.001 or re>.008:raise RuntimeError(f'Unreachable{label}/{s}: {pe},{re}')
                maximum[0]=max(maximum[0],pe);maximum[1]=max(maximum[1],re)
                state[s]=np.r_[q,f0+(f1-f0)*w];action.extend(state[s].tolist())
            row={'phase':label,'action14':action}
            if tip is not None:
                final=np.asarray(tip)-axis*.08
                initial=final if first_recipe is None else np.asarray(first_recipe['barrel_position_m'])
                recipe={'barrel_position_m':(initial+(final-initial)*w).tolist(),'barrel_rotation':R.tolist()}
                if piston is not None:
                    q0=0. if first_recipe is None else first_recipe.get('piston_m',0.)
                    recipe['piston_m']=q0+(piston-q0)*w
                row['scripted_tool_target']=recipe;recipe_last=recipe
            else:recipe_last=None
            frames.append(row)
    tip=np.array((.235,-.040,.8035));centre=tip-axis*.08
    hand,hand_R=calibrated_hand_pose(centre,R);contact=centre-axis*.0755
    # Move to the first contact point with fixed joint references, then engage
    # tool feedback only after the nominal calibrated grasp has settled.
    transition(3.,'position above first shell',{'right':(hand,hand_R,0.),'left':(contact-axis*.04,push_R,1.)})
    transition(1.,'close pressing fingers',{'left':(contact-axis*.04,push_R,0.)})
    transition(1.,'approach passive plunger',{'left':(contact,push_R,0.)},tip=tip)
    material=IcingParameters();A=material.piston_area_m2;expected_volume=0.
    def q_for_force(force,seconds):
        nonlocal expected_volume
        q=(expected_volume+force/A*material.initial_volume_m3/material.bulk_modulus_pa)/A
        expected_volume+=material.conductance_m3_per_pa_s*max(force/A-material.yield_pressure_pa,0.)*seconds
        return q
    for shell in range(shells):
        tip=np.array((.235,-.040+shell*pitch,.8035));centre=tip-axis*.08
        hand,hand_R=calibrated_hand_pose(centre,R);contact=centre-axis*.0755
        if shell:
            transition(.6,'move to next shell',{'right':(hand,hand_R,0.),'left':(contact-axis*.004,push_R,0.)},tip=tip)
            transition(.3,'recontact passive plunger',{'left':(contact,push_R,0.)},tip=tip,piston=expected_volume/A)
        # A finite rise precedes the prescribed head and tapered tail.
        for j in range(6):
            force=7.*(j+1)/6;q=q_for_force(force,.05)
            transition(.05,'build shell head',{'left':(contact+axis*q,push_R,0.)},tip=tip,piston=q,linear=True)
        for j in range(round(head_seconds*20)):
            q=q_for_force(7.,.05)
            transition(.05,'hold shell head',{'left':(contact+axis*q,push_R,0.)},tip=tip,piston=q,linear=True)
        for j in range(round(tail_seconds*20)):
            fraction=(j+1)/round(tail_seconds*20);force=7.+(3.3-7.)*fraction;q=q_for_force(force,.05)
            moved=tip+np.array((0.,tail*fraction,0.));centre=moved-axis*.08;hp,hr=calibrated_hand_pose(centre,R)
            cp=centre-axis*.0755
            transition(.05,'draw tapered shell tail',{'right':(hp,hr,0.),'left':(cp+axis*q,push_R,0.)},tip=moved,piston=q,linear=True)
        tip=moved;centre=tip-axis*.08;contact=centre-axis*.0755
        transition(.3,'release pressure',{'left':(contact-axis*.008,push_R,0.)},tip=tip)
    transition(1.,'withdraw pressing hand',{'left':(contact-axis*.04,push_R,0.)},tip=tip)
    raised=tip+np.array((0.,0.,.04));hp,hr=calibrated_hand_pose(raised-axis*.08,R)
    transition(1.,'lift nozzle clear',{'right':(hp,hr,0.)},tip=raised)
    park=np.array((.18,-.16,.95));hp,hr=calibrated_hand_pose(park,R)
    transition(2.,'move dispenser to rest',{'right':(hp,hr,0.)})
    horizontal=np.diag((-1.,-1.,1.));rest=np.array((.18,-.16,.7815))
    hp,hr=calibrated_hand_pose(rest+np.array((0.,0.,.12)),horizontal)
    transition(2.,'turn dispenser above rest',{'right':(hp,hr,0.)})
    rp,rr=calibrated_hand_pose(rest,horizontal)
    transition(2.,'lower dispenser to rest',{'right':(rp,rr,0.)})
    transition(1.,'release dispenser',{'right':(rp,rr,1.)})
    transition(2.,'withdraw empty hand',{'right':(hp,hr,1.)})
    transition(2.,'observe released decoration',{})
    return {'schema':'state-assisted-recipe-v1','control_hz':20,'no_object_feedback':False,
        'assistance':'Native barrel and plunger poses guide ordinary bounded joint targets; Dream is unassisted.',
        'physics_state_writes':False,'shell_count':shells,'frames':frames,'expected_emitted_ml':expected_volume*1e6,
        'recipe':{'pitch_m':pitch,'tail_m':tail,'head_seconds':head_seconds,'tail_seconds':tail_seconds,'head_force_n':7.,'tail_final_force_n':3.3},
        'maximum_ik_position_error_m':maximum[0],'maximum_ik_rotation_error_rad':maximum[1]}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--shells',type=int,choices=range(1,7),default=3)
    a=p.parse_args();plan=build(a.shells);a.out.write_text(json.dumps(plan,indent=2)+'\n');print(json.dumps({k:v for k,v in plan.items() if k!='frames'}))
