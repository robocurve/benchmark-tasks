"""Continuous printer pickup and measured mid-strip regrasp, no object resets."""
import json
import os
import sys
import time
from pathlib import Path
import numpy as np
from yam_arena import station, ik
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED, READY
from bagtag_probe import Plan, Run, W, frame, hold_ready
from exit_guide_pickup_probe import pickup_waypoints, panel_frame


def main(out):
    run = Run(out, 'direct_thread_transport')
    from isaaclab.sim.utils.stage import get_current_stage
    material = {}
    for prim in get_current_stage().Traverse():
        if 'checked_suitcase' in str(prim.GetPath()) and prim.GetName() == 'physics':
            material = {a.GetName(): str(a.Get()) for a in prim.GetAttributes() if 'friction' in a.GetName().lower()}
    (out/'case_material.json').write_text(json.dumps(material,indent=2)+'\n')
    reports = []
    def current(side):
        return np.asarray(run.state()['robots'][side]['joints'])
    def hold(side):
        q = current(side)
        # Maintain motor closure, not the measured obstruction position: using
        # the latter removes squeeze force and lets the paper slide away.
        a = np.r_[q[:6], 1.0 if np.mean(q[6:]) < -.02 else 0.0]
        return lambda t: (a, 'hold current')
    def stage(side, waypoints, name):
        q = current(side)
        check = Plan(waypoints, station.BASES[side], q_start=q[:6], finger_start=float(np.mean(q[6:])))
        for t in np.arange(.05, check.duration+.001, .05):
            check.action7(t)
        if check.worst_ik_mm > 5 or check.worst_ik_deg > 3:
            raise RuntimeError(f'Preflight IK rejected {name}: {check.worst_ik_mm:.2f} mm / {check.worst_ik_deg:.2f} deg')
        plan = Plan(waypoints, station.BASES[side], q_start=q[:6], finger_start=float(np.mean(q[6:])))
        count = round(plan.duration * 20)
        start = run.step_i
        funcs = {'left': hold('left'), 'right': hold('right')}
        funcs[side] = plan.action7
        run.go(count, funcs['left'], funcs['right'], snaps=(start+count,), video_every=4)
        reports.append({'phase': name, 'step': run.step_i, 'ik_mm': plan.worst_ik_mm, 'ik_deg': plan.worst_ik_deg})
        run.save({'stage': 'continuous_partial_transport_not_scored', 'phases': reports})
        if plan.worst_ik_mm > 5 or plan.worst_ik_deg > 3:
            raise RuntimeError('IK gate failed in ' + name)
    def move_joint_target(side, pos, R, finger, name):
        from handle_route_cpu import best_ik, obstacle_points
        from exit_guide_pickup_cpu import inside_any
        import yam_hulls
        from scipy.spatial import cKDTree
        q0 = current(side)[:6]
        q1, pe, re = best_ik(side, np.asarray(pos), R)
        if pe > .001 or np.degrees(re) > 1:
            raise RuntimeError('No accurate pregrasp IK')
        obs = obstacle_points(.008)
        other = 'right' if side == 'left' else 'left'
        other_h = [h for hs in yam_hulls.hulls(current(other),other).values() for h in hs]
        other_v = np.concatenate([h.vertices for h in other_h])
        other_tree = cKDTree(other_v)
        for f in np.linspace(0,1,25):
            q = q0+(q1-q0)*ik.smoothstep(f)
            hs = [h for pieces in yam_hulls.hulls(np.r_[q,finger,finger],side).values() for h in pieces]
            if any(inside_any(points,hs).any() for points in obs.values()):
                raise RuntimeError('Pregrasp joint route obstacle collision')
            if other_tree.query(np.concatenate([h.vertices for h in hs]))[0].min() < .005:
                raise RuntimeError('Pregrasp joint route arm clearance')
        def command(t):
            return np.r_[q0+(q1-q0)*ik.smoothstep(min(t/3,1)),finger/FINGER_OPEN],name
        funcs={'left':hold('left'),'right':hold('right')};funcs[side]=command
        start=run.step_i
        run.go(80,funcs['left'],funcs['right'],snaps=(start+80,),video_every=4)
        reports.append({'phase':name,'step':run.step_i,'ik_mm':float(pe*1000),'ik_deg':float(np.degrees(re))})
        run.save({'stage':'continuous_partial_transport_not_scored','phases':reports})
    try:
        staged = os.environ.get('BAGTAG_STAGE_STATE')
        if staged:
            from restore_contact_diagnostic import restore
            restore(run, staged, int(os.environ.get('BAGTAG_STAGE_STEP','1838')))
            run.snap('staged_initial_not_demo')
            run.save({'stage':'staged_contact_diagnostic_not_demo','phases':reports})
        else:
            run.snap('reset')
            stage('right', pickup_waypoints(.8), 'proven pickup replay')
            if os.environ.get('BAGTAG_NEGATIVE') == 'drop_after_extraction':
                q=current('right')[:6].copy()
                run.go(120,hold_ready(),lambda t:(np.r_[q,1.],'open grip and let strip fall'),snaps=(run.step_i+120,),video_every=4)
                reports.append({'phase':'physical negative: drop without attachment','step':run.step_i})
                assert run.state()['closure']['score']==0., 'Dropped tag must score zero'
                run.save({'stage':'physical_negative_drop_passed','phases':reports})
                return
            Rx = frame((1,0,0),(0,0,1))
            stage('right', [W(2.0,(.18,-.18,1.34),Rx,FINGER_CLOSED,'present tag beside case'),
                            W(1.0,(.18,-.18,1.34),Rx,FINGER_CLOSED,'settle tag')], 'present')
            s = run.state()
            c, P = panel_frame(s['panel_xyz'][5], s['panel_quat_xyzw'][5])
            approach = np.array([0.,-1.,0.])
            # The opposite pad normal avoids the left wrist roll limit. With this
            # grasp, rolling to Rx presents the panel00 end toward -Y for threading.
            R = frame(approach, -P[:,2])
            gp = c - .013*approach
            move_joint_target('left',gp-.080*approach,R,FINGER_OPEN,'orient beside panel05')
            stage('left', [W(1.5,gp,R,FINGER_OPEN,'straddle panel05 faces'),
                           W(1.,gp,R,FINGER_CLOSED,'pinch panel05'),
                           W(.8,gp,R,FINGER_CLOSED,'hold both pinches')], 'left mid-strip grasp')
            before = run.state()
            rc, rr = ik.tcp_pose(current('right')[:6],station.BASES['right'])
            stage('right',[W(.7,rc,rr,FINGER_OPEN,'release end A'),
                           W(1.5,rc+[0,-.06,0],rr,FINGER_OPEN,'clear released end')], 'right release')
            s = run.state()
            lp, lr = ik.tcp_pose(current('left')[:6],station.BASES['left'])
            offset = lr.T @ (np.asarray(s['panel_xyz'][5])-lp)
            reports.append({'phase':'left grip verification','panel05_in_tcp_m':offset.tolist(),
                            'left_fingers_m':current('left')[6:].tolist(),
                            'panel05_motion_after_right_release_m':float(np.linalg.norm(np.asarray(s['panel_xyz'][5])-before['panel_xyz'][5]))})
            if abs(offset[0]) > .006 or np.linalg.norm(offset) > .045:
                run.save({'stage':'midstrip_regrasp_failed_no_feed','phases':reports})
                return
            stage('right',[W(2.,(.20,-.30,1.30),Rx,FINGER_OPEN,'clear route for left hand')], 'right clear')
            stage('left',[W(2.5,(.28,-.08,1.25),Rx,FINGER_CLOSED,'roll strip above case'),
                           W(2.5,(.337,.24,1.25),Rx,FINGER_CLOSED,'roll free end toward handle'),
                           W(1.5,(.337,.24,1.25),Rx,FINGER_CLOSED,'settle before feed')], 'pre-feed roll')
            run.save({'stage':'midstrip_transport_finished_not_threaded','phases':reports})
        # Replay the validated robot commands in one continuous fresh rollout.
        # These are motion targets, never saved object states or physics resets.
        replay = os.environ.get('BAGTAG_REPLAY_COMMANDS')
        if replay:
            replay_limit = int(os.environ.get('BAGTAG_REPLAY_LIMIT', '999'))
            for path in sorted(Path(replay).glob('*.json')):
                cmd = json.loads(path.read_text())
                if int(cmd['id']) > replay_limit:
                    continue
                # Read actual paper position for the receiving grasp. This changes
                # robot targets only; it never moves the paper by assignment.
                if cmd.get('measure_panel') is not None:
                    centre=np.asarray(run.state()['panel_xyz'][int(cmd['measure_panel'])])
                    delta=centre-np.array([.013,0,0])-np.asarray(cmd['waypoints'][-1]['pos'])
                    for w in cmd['waypoints']:w['pos']=(np.asarray(w['pos'])+delta).tolist()
                if cmd.get('rotate_held_endpoint'):
                    p0,R0=ik.tcp_pose(current('right')[:6],station.BASES['right'])
                    turn_R=frame([.5,.8660254,0],[0,0,1])
                    cmd['waypoints']=[dict(duration=t,pos=p0.tolist(),approach=R[:,2].tolist(),closing=R[:,0].tolist(),finger=f,label=label)
                        for t,R,f,label in [(.4,R0,-.004,'loosen before changing approach'),(2.,turn_R,-.004,'turn open jaws around supported endpoint'),(.5,turn_R,0.,'regrasp without twisting strip'),(.4,turn_R,0.,'settle')]]
                if cmd.get('press_endpoint_pair'):
                    from scipy.spatial.transform import Rotation
                    state=run.state();xyz=np.array(state['panel_xyz'])
                    normal=Rotation.from_quat(state['panel_quat_xyzw'][0]).as_matrix()[:,2]
                    p0,_=ik.tcp_pose(current('right')[:6],station.BASES['right'])
                    target=p0+xyz[9]+.0012*normal-xyz[0];pre=target-np.array([0,.045,0])
                    press_R=frame([.5,.8660254,0],[0,0,1])
                    stage('right',[W(2.,pre,press_R,0.,'approach second layer'),W(.3,pre,press_R,-.004,'open for two layers')],'open receiving jaws')
                    state=run.state();xyz=np.array(state['panel_xyz'])
                    normal=Rotation.from_quat(state['panel_quat_xyzw'][0]).as_matrix()[:,2]
                    p0,_=ik.tcp_pose(current('right')[:6],station.BASES['right'])
                    target=p0+xyz[9]+.0012*normal-xyz[0]
                    stage('right',[W(1.5,target,press_R,-.004,'align two layers using measured poses')],'align inside receiving jaws')
                    g=run.state()['closure']
                    if g['overlap_m2']<.0006 or g['normal_dot']<.985:
                        raise RuntimeError('Layers not aligned inside jaws; do not close')
                    cmd['waypoints']=[dict(duration=t,pos=target.tolist(),approach=[.5,.8660254,0],closing=[0,0,1],finger=0.,label=label) for t,label in
                        [(.8,'deliberately pinch aligned patches'),(3.,'sustain pinch at sealed overlap before release')]]
                if cmd.get('release_ready'):
                    side=cmd['side'];p0,R0=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    pr,Rr=ik.tcp_pose(READY,station.BASES[side]);sg=1 if side=='left' else -1
                    opening=-.025 if side=='right' else -.006
                    release_p=p0+(.025*R0[:,0] if side=='right' else np.zeros(3))
                    back=release_p+(np.array([-.06,-.12,.045]) if side=='right' else np.array([-.08,0.,.025]))
                    cmd['waypoints']=[dict(duration=t,pos=p.tolist(),approach=R[:,2].tolist(),closing=R[:,0].tolist(),finger=f,label='release then withdraw and return ready')
                        for t,p,R,f in [(1.,release_p,R0,opening),(2. if side=='right' else 1.5,back,R0,opening),(2.,back,frame([1,0,0],[0,0,-sg]),-.047),(1.5,np.array([.24,sg*.26,1.2]),frame([1,0,0],[0,0,-sg]),-.047),(2.,pr,Rr,-.047),(3.,pr,Rr,-.047)]]
                (out/'commands').mkdir(exist_ok=True)
                (out/'commands'/path.name).write_text(json.dumps(cmd,indent=2))
                waypoints = [W(w['duration'], w['pos'], frame(w['approach'], w['closing']),
                               w['finger'], w['label']) for w in cmd['waypoints']]
                stage(cmd['side'], waypoints, cmd.get('name', path.stem))
                if cmd.get('ensure_adhesive') and not run.state()['closure'].get('bonded'):
                    raise RuntimeError('Physical pressing failed; keep evidence and do not release')
            # Record a substantial live terminal interval so sampled review
            # frames show released attachment, rather than the last manipulation.
            run.go(160,hold('left'),hold('right'),snaps=(run.step_i+160,),video_every=4)
            last=run.state()['closure']
            valid=last.get('score')==1. and last.get('hands_clear') and last.get('hands_open')
            run.save({'stage': ('staged_press_diagnostic_complete' if staged else
                               'continuous_full_demo_complete') if valid else 'demo_incomplete',
                      'terminal_success':bool(valid),'phases':reports})
            if not valid:raise RuntimeError('Final released stable loop not verified')
        # Keep this same physical rollout for bounded, state-informed feed
        # commands rather than repeating printer extraction for each probe.
        commands = os.environ.get('BAGTAG_COMMAND_FILE')
        if commands:
            seen = None
            deadline = time.monotonic()+900
            (out/'command_status.json').write_text(json.dumps({'state':'awaiting_command','step':run.step_i}))
            while time.monotonic() < deadline:
                try:
                    cmd = json.loads(Path(commands).read_text())
                except (FileNotFoundError,json.JSONDecodeError):
                    time.sleep(.2); continue
                if cmd.get('id') == seen:
                    time.sleep(.2); continue
                seen=cmd.get('id')
                if cmd.get('stop'): break
                try:
                    waypoints=[W(w['duration'],w['pos'],frame(w['approach'],w['closing']),w['finger'],w['label']) for w in cmd['waypoints']]
                    stage(cmd['side'],waypoints,cmd.get('name',str(seen)))
                    state={'state':'completed','id':seen,'step':run.step_i}
                except RuntimeError as e:
                    state={'state':'rejected','id':seen,'step':run.step_i,'reason':str(e)}
                (out/'command_status.json').write_text(json.dumps(state,indent=2)+'\n')
                deadline=time.monotonic()+900
    finally:
        run.close()


if __name__ == '__main__':
    main(Path(sys.argv[1]))
