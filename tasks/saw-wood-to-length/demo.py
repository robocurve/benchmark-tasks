"""Native robot mechanism diagnostic. All manipulation uses public 14D actions."""
import argparse,json,math
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True)
    p.add_argument('--mode',choices=['idle','grasp','clamp_one','clamp','full'],default='idle')
    a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    from isaaclab.app import AppLauncher  # load app bindings before task modules
    import task as T
    import runtime
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import ik,station
    from yam_arena.constants import READY
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import imageio.v2 as imageio
    emb=yam_arena_embodiment(task=str(Path(T.__file__).resolve()),object_jitter=0,control_hz=20,cam_width=640,cam_height=352,device='cpu')
    obs=emb.reset(Scene(id='saw-wood-to-length',instruction=T.INSTRUCTION))
    from yam_arena.scene import object_states
    commands={side:np.r_[np.array(READY),1.] for side in ['left','right']}
    frames={k:[] for k in obs.images};steps=0;measurements=[];plan=[]
    video_stride=5 if a.mode=='full' else 2
    def state(label):
        st=object_states(emb._env,['wood','saw','jaw','screw'])
        r={'label':label,'step':steps,'score':T.score(st),'objects':{k:{'pos':v.pos.tolist(),'quat':v.quat_xyzw.tolist()} for k,v in st.items()}}
        if runtime.ENGINE is not None:
            c=runtime.ENGINE.cutting
            r['cut']={'depth_m':float(c.stock.progress.max()),'events':len(c.events),
                      'separated':c.cut_time is not None,'cut_before_clamp':c.cut_before_clamp,
                      'contact_N':runtime.ENGINE.last_fn,
                      'geometry_score':c.last_score,'clamp_gap_m':c.last_gap}
        measurements.append(r);print('MEASUREMENT '+json.dumps(r),flush=True)
        for cam,img in obs.images.items():imageio.imwrite(out/(label+'-'+cam+'.png'),np.asarray(img))
        return st
    def step():
        nonlocal obs,steps
        r=emb.step(Action(data=pack(commands['left'],commands['right'])));obs=r.observation;steps+=1
        if steps%video_stride==0:
            for cam,img in obs.images.items():frames[cam].append(np.asarray(img).copy())
    def hold(n,label=None):
        for _ in range(n):step()
        if label:state(label)
    def solve(side,point,rot,seed=None):
        q0=commands[side][:6] if seed is None else seed
        quat=ik.matrix_to_quat_xyzw(rot)
        q,pe,re=ik.solve(point,quat,q_init=q0,base_pos=station.BASES[side],iters=450,damping=.02)
        if pe>.002 or re>.03:
            for seed in np.random.default_rng(14).uniform(ik.LOWER,ik.UPPER,size=(20,6)):
                qq,pp,rr=ik.solve(point,quat,q_init=seed,base_pos=station.BASES[side],iters=350,damping=.02)
                if pp+.1*rr<pe+.1*re:q,pe,re=qq,pp,rr
                if pe<.001 and re<.01:break
        if pe>.003 or re>.06:raise RuntimeError(f'IK {side}: {point}, {pe}, {re}')
        return q
    def solve_fast(side,point,rot):
        # A saw stroke changes its target only a few millimetres per action.
        # Start from the previous command and search alternate branches only
        # if that warm start cannot meet the same acceptance bounds.
        quat=ik.matrix_to_quat_xyzw(rot)
        q,pe,re=ik.solve(point,quat,q_init=commands[side][:6],
                         base_pos=station.BASES[side],iters=120,damping=.02)
        return q if pe<=.003 and re<=.06 else solve(side,point,rot)
    def move(side,point,rot,n,label):
        q=solve(side,point,rot);start=commands[side][:6].copy()
        plan.append({'label':label,'side':side,'point':np.asarray(point).tolist(),'q':q.tolist()})
        for i in range(n):commands[side][:6]=start+ik.smoothstep((i+1)/n)*(q-start);step()
        state(label)
    def grip(side,g,n=25):
        start=commands[side][6]
        for i in range(n):commands[side][6]=start+(g-start)*ik.smoothstep((i+1)/n);step()
    status='completed';error=None
    try:
        state('reset');hold(30,'settled')
        if a.mode in ['clamp_one','clamp','full']:
            center=np.array(T.KNOB.pos)
            rot=np.diag([1.,-1.,-1.])
            move('right',center+[0,0,.08],rot,70,'knob_pregrasp')
            move('right',center,rot,60,'knob_grasp_pose');grip('right',0.)
            hold(15,'knob_closed')
            for cycle in range(1 if a.mode=='clamp_one' else 6):
                for angle in np.linspace(0,.55,16)[1:]:
                    ry=np.array([[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]])
                    commands['right'][:6]=solve('right',center,ry@rot);step()
                last_rot=ry@rot
                hold(5,f'knob_turn_{cycle+1}')
                # Slide off along the screw axis before opening the fingers;
                # opening against the rotated square knob back-drives it.
                move('right',center+[0,-.04,0],last_rot,20,f'knob_slide_off_{cycle+1}')
                grip('right',1.,12)
                hold(3,f'knob_released_{cycle+1}')
                move('right',center+[0,-.04,.03],last_rot,15,f'knob_withdrawn_{cycle+1}')
                if cycle<5 and a.mode!='clamp_one':
                    move('right',center+[0,0,.03],rot,25,f'knob_reset_{cycle+1}')
                    move('right',center,rot,20,f'knob_regrasp_{cycle+1}')
                    grip('right',0.,15)
            move('right',center+[0,0,.08],rot,45,'right_clear')
            hold(20,'vise_tightened')
        if a.mode in ['grasp','full']:
            st=state('before_saw_grasp');point=st['saw'].to_world(T.GRIP_LOCAL)
            # YAM's fingers close along gripper-local X; this pose pinches
            # across the handle's 26 mm width.
            rot=np.diag([1.,-1.,-1.])
            move('left',point+[0,0,.10],rot,70,'saw_pregrasp')
            move('left',point,rot,60,'saw_grasp_pose');grip('left',0.)
            hold(25,'saw_closed');move('left',point+[0,0,.08],rot,70,'saw_lift')
            hold(30,'saw_held')
            if a.mode=='grasp':grip('left',1.);hold(40,'saw_released')
        if a.mode=='full':
            # The robot alone carries the saw. Read the visible cut depth to
            # feed the blade; the physics task, not this demo, removes wood.
            # Aim the saw centre on the waste side of the 150 mm pencil mark.
            # The 1 mm kerf and measured lateral compliance make its final
            # retained face shorter than the commanded tool centre.
            cut_center_x=.15175
            wood=object_states(emb._env,['wood'])['wood']
            target=np.array([float(wood.pos[0])+cut_center_x,.145,float(wood.pos[2])+.020+.035+.025])
            move('left',target,rot,100,'saw_above_mark')
            def tooth_at_wood(y_local=0.):
                w=object_states(emb._env,['wood','saw'])
                saw=w['saw'];board=w['wood'];r=saw.rotation()
                ends=[board.to_local(saw.pos+r@np.array([0,y,-.035])) for y in (-.055,-.255)]
                u=(y_local-ends[0][1])/(ends[1][1]-ends[0][1])
                return np.asarray(ends[0])+u*(np.asarray(ends[1])-np.asarray(ends[0]))
            mark_error=cut_center_x-float(tooth_at_wood()[0])
            if abs(mark_error)>.010:raise RuntimeError(f'saw alignment {mark_error:.4f} m')
            target[0]+=mark_error
            move('left',target,rot,35,'saw_aligned')
            # Calibrate the grasp offset from the actual held saw pose.
            root=object_states(emb._env,['saw'])['saw'].pos
            z_bias=float(root[2]-target[2])
            target[2]=float(wood.pos[2])+.020+.035-z_bias-.00035
            move('left',target,rot,35,'first_tooth_contact')
            target[0]+=cut_center_x-float(tooth_at_wood()[0])
            move('left',target,rot,25,'contact_line_aligned')
            anchor_x=float(target[0]);initial_anchor_x=anchor_x
            roll_command=0.;yaw_command=0.
            for pass_idx in range(75):
                cutter=runtime.ENGINE.cutting
                ix=round(.15075/cutter.stock.dx)
                half=max(1,round(.00075/cutter.stock.dx))
                cleared=cutter.stock.cleared[ix-half:ix+half+1,:]
                depth=float(cleared.max(axis=0).min())*cutter.stock.dz
                line_error=cut_center_x-float(tooth_at_wood()[0])
                if abs(line_error)>.010:
                    raise RuntimeError(f'saw left the length mark by {line_error:.4f} m')
                anchor_x=float(np.clip(anchor_x+.50*line_error,
                                       initial_anchor_x-.010,initial_anchor_x+.010))
                height_error=.020-depth-.00035-float(tooth_at_wood()[2])
                if abs(height_error)>.005:
                    raise RuntimeError(f'saw tooth height error {height_error:.4f} m')
                target[2]+=float(np.clip(height_error,-.001,.001))
                from_y=target[1];to_y=.170 if pass_idx%2==0 else .120
                for i in range(20):
                    live_error=cut_center_x-float(tooth_at_wood()[0])
                    if abs(live_error)>.010:
                        raise RuntimeError(f'saw left the length mark by {live_error:.4f} m')
                    desired_x=anchor_x+float(np.clip(live_error,-.004,.004))
                    target[0]+=float(np.clip(desired_x-target[0],-.0008,.0008))
                    target[1]=from_y+ik.smoothstep((i+1)/20)*(to_y-from_y)
                    saw_rotation=object_states(emb._env,['saw'])['saw'].rotation()
                    saw_roll=math.atan2(float(saw_rotation[2,1]),float(saw_rotation[2,2]))
                    saw_yaw=math.atan2(float(saw_rotation[1,0]),float(saw_rotation[0,0]))
                    desired_roll=float(np.clip(-4.*saw_roll,-.45,.45))
                    roll_command+=float(np.clip(desired_roll-roll_command,-.020,.020))
                    desired_yaw=float(np.clip(-2.*saw_yaw,-.08,.08))
                    yaw_command+=float(np.clip(desired_yaw-yaw_command,-.005,.005))
                    cr,sr=math.cos(roll_command),math.sin(roll_command)
                    cy,sy=math.cos(yaw_command),math.sin(yaw_command)
                    roll_rotation=np.array([[1.,0.,0.],[0.,cr,-sr],[0.,sr,cr]])
                    yaw_rotation=np.array([[cy,-sy,0.],[sy,cy,0.],[0.,0.,1.]])
                    commands['left'][:6]=solve_fast('left',target,yaw_rotation@roll_rotation@rot);step()
                print('CUT_PASS',pass_idx+1,'depth',depth,'line_error',live_error,
                      'force',runtime.ENGINE.last_fn,'saw_roll',saw_roll,
                      'saw_yaw',saw_yaw,'roll_command',roll_command,
                      'yaw_command',yaw_command,
                      'edge_error',max(abs(cut_center_x-float(tooth_at_wood(y)[0])) for y in (-.01,.01)),flush=True)
                if pass_idx%5==4 or cutter.cut_time is not None:
                    state(f'saw_pass_{pass_idx+1:02d}')
                if cutter.cut_time is not None:break
            target[2]=float(wood.pos[2])+.020+.035-z_bias+.035
            move('left',target,rot,50,'saw_clear')
            hold(20,'cut_result')
            if T.score({})<1.:
                raise RuntimeError('full robot demonstration did not meet the end-state score')
        if a.mode=='idle':hold(70,'idle_end')
    except Exception as e:
        import traceback;traceback.print_exc();status='failed';error=repr(e)
    finally:
        cut=runtime.ENGINE.cutting if runtime.ENGINE else None
        result={'mode':a.mode,'status':status,'error':error,'steps':steps,'score':T.score({}),
                'measurements':measurements,'plan':plan,
                'cut_geometry':cut.stock.score() if cut else None,
                'max_cut_depth_m':float(cut.stock.progress.max()) if cut else None,
                'cut_before_clamp':cut.cut_before_clamp if cut else None}
        (out/'result.json').write_text(json.dumps(result,indent=2))
        (out/'physics.json').write_text(json.dumps(runtime.ENGINE.rows if runtime.ENGINE else [],indent=2))
        for cam,seq in frames.items():
            if seq:imageio.mimsave(out/(cam+'.mp4'),seq,fps=20/video_stride,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
        print('RESULT '+json.dumps(result),flush=True);emb.close()
        if status!='completed':raise SystemExit(1)

if __name__=='__main__':main()
