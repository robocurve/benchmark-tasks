"""Contact-gated adhesive approximation and outcome scoring; no pose writes.

The designated front patch on panel09 adheres to panel00's designated back patch after
aligned, physically pressed surface contact for 0.4 s. Chemistry and peel mechanics are simplified
as a breakable fixed joint, created at the *existing* relative transform.
"""
import numpy as np
from scipy.spatial.transform import Rotation

STATE = {}
JOINT_PATH = '/World/baggage_tag_adhesive'
_EVENT_SUB = None
MIN_PRESS_N = 1.0
PRESS_SECONDS = .4
RELEASE_SECONDS = .25
SUCCESS_SECONDS = 2.


def reset(environment=None):
    global _EVENT_SUB
    _EVENT_SUB = None
    import press_contacts
    press_contacts.reset()
    STATE.clear()
    STATE.update(contact_s=0., stable_s=0., bonded=False, broken=False,
                 extracted=False, threaded=False, score=0., diagnostics={})
    if environment is not None:
        from isaaclab.sim.utils.stage import get_current_stage
        stage=get_current_stage()
        if stage.GetPrimAtPath(JOINT_PATH): stage.RemovePrim(JOINT_PATH)
        import carb
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)


def patch_press_force(xyz, rotations, contacts, dt):
    """Mean compressive force on designated faces during the sampled physics substep.

    Points must lie within both printed patches, with a surface-normal contact.
    Normal impulses divided by elapsed simulated time are forces, not counts.
    """
    if not np.isfinite(dt) or dt <= 0:
        return 0.
    total = 0.
    for point, normal, impulse in contacts:
        if not all(np.all(np.isfinite(v)) for v in (point, normal, impulse)):
            continue
        if abs(normal @ rotations[0,:,2]) < .98:
            continue
        local = [rotations[i].T @ (point-xyz[i]) for i in (0,9)]
        if all(abs(p[0]) <= .01951 and abs(p[1]) <= .02301 and abs(p[2]) <= .002 for p in local):
            total += abs(float(impulse @ normal))
    return total / dt


def press_dwell(previous, geometry_contact, speed, force, dt):
    valid = (np.all(np.isfinite([previous, speed, force, dt])) and
             previous >= 0. and dt > 0. and 0. <= speed < .04)
    return previous+dt if valid and geometry_contact and force>=MIN_PRESS_N else 0.


def geometry(xyz, rotations, bar_x=.350, bar_z=1.0965):
    xyz=np.asarray(xyz); R=np.asarray(rotations)
    d=R[0].T@(xyz[9]-xyz[0])
    align=float(np.dot(R[0,:,2],R[9,:,2]))
    area=max(0.,.039-abs(d[0]))*max(0.,.046-abs(d[1]))
    contact=(align>.985 and abs(np.dot(R[0,:,0],R[9,:,0]))>.985 and
             -.0018<=d[2]<=-.00055 and area>=.00060)
    # Every hinge must still join its two actual anchors.
    anchors=[np.linalg.norm(xyz[i-1]-.0215*R[i-1,:,0]-(xyz[i]+.0215*R[i,:,0])) for i in range(1,10)]
    # Follow panel edges through each physical hinge, rather than cutting
    # diagonally between centres across a sharp fold.
    curve=np.concatenate([np.array([p+.0215*r[:,0],p,p-.0215*r[:,0]]) for p,r in zip(xyz,R)])
    yz=curve[:,1:3]-[0.,bar_z]
    nxt=np.roll(yz,-1,axis=0)
    winding=float(np.sum(np.arctan2(yz[:,0]*nxt[:,1]-yz[:,1]*nxt[:,0],np.sum(yz*nxt,axis=1)))/(2*np.pi))
    under=False
    for a,b in zip(curve[:-1],curve[1:]):
        if a[1]*b[1]<0:
            t=-a[1]/(b[1]-a[1]);p=a+t*(b-a)
            if abs(p[0]-bar_x)<.065 and 1.0668<p[2]<bar_z-.003:under=True
    linked=abs(winding)>.8 and under and np.all(np.abs(xyz[:,0]-bar_x)<.075)
    # Both faces are printed. Require a complete sampled barcode face to have
    # a clear outward line of sight, not merely an outward panel-centre normal.
    barcode_outward = barcode_exposed(xyz, R, bar_z)
    return dict(barcode_outward=bool(barcode_outward),face_gap_m=float(d[2]),overlap_m2=float(area),normal_dot=align,
                end_distance_m=float(np.linalg.norm(xyz[9]-xyz[0])),
                max_hinge_gap_m=float(max(anchors)),contact=bool(contact),
                under=bool(under),linked=bool(linked),winding=winding)


def panel_corners(xyz, rotations):
    local = np.array([[x,y,0.] for x in (-.0215,.0215) for y in (-.025,.025)])
    return np.asarray(xyz)[:,None,:] + np.einsum('nij,kj->nki',rotations,local)


def barcode_exposed(xyz, R, bar_z):
    """Nine distributed surface samples, obstructed by any other paper panel.

    This is a finite geometric exposure test, not OCR or a visibility proof.
    Only upward exterior views are accepted, keeping rays clear of the shell.
    """
    for i in (1,5):
        radial = xyz[i] - np.array([xyz[i,0],0.,bar_z])
        for sign in (-1.,1.):
            n = sign*R[i,:,2]
            if n @ radial <= .01 or n[2] < .1 or xyz[i,2] < 1.070:
                continue
            visible = True
            for x in (-.014,0.,.014):
                for y in (-.0165,0.,.0165):
                    origin = xyz[i]+x*R[i,:,0]+y*R[i,:,1]+.0008*n
                    for j in range(10):
                        if j == i: continue
                        o=R[j].T@(origin-xyz[j]);v=R[j].T@n
                        # Ray / oriented thin box, including grazing cases.
                        lo,hi=0.,.30
                        for k,h in enumerate((.0215,.025,.0007)):
                            if abs(v[k])<1e-10:
                                if abs(o[k])>h: hi=-1.; break
                            else:
                                a,b=sorted(((-h-o[k])/v[k],(h-o[k])/v[k]))
                                lo=max(lo,a);hi=min(hi,b)
                        if hi>=lo and hi>=0.: visible=False; break
                    if not visible: break
                if not visible: break
            if visible: return True
    return False


def motion_speed(history, dt):
    """Largest consecutive corner displacement: detects rotation and oscillation."""
    if len(history)<2: return float('inf')
    return float(np.max(np.linalg.norm(np.diff(np.asarray(history),axis=0),axis=-1))/dt)


def final_valid(extracted, bonded, broken, linked, barcode, opened, clear, speed):
    return bool(extracted and bonded and not broken and linked and barcode
                and opened and clear)


def on_joint_break(path):
    # Printer retention is intentionally breakable; only the seal is fatal here.
    if str(path) == JOINT_PATH:
        STATE['broken'] = True
        STATE['bond_break_event'] = True


def monitor_bond_breaks():
    global _EVENT_SUB
    from omni.physx import get_physx_interface
    from omni.physx.bindings._physx import SimulationEvent
    from pxr import PhysicsSchemaTools
    def on_event(event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            pair=event.payload['jointPath']
            on_joint_break(PhysicsSchemaTools.decodeSdfPath(pair[0],pair[1]))
    _EVENT_SUB = get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_event)


def create_bond(environment, xyz, rotations):
    if STATE.get('contact_s', 0.) + 1e-9 < PRESS_SECONDS:
        raise RuntimeError('Bond requires verified physical pressing dwell')
    from pxr import Gf,Sdf,UsdPhysics
    from isaaclab.sim.utils.stage import get_current_stage
    stage=get_current_stage();paths={}
    for prim in stage.Traverse():
        if prim.GetName() in ('tag_panel_00','tag_panel_09'):
            body=str(prim.GetPath())+'/body'
            if stage.GetPrimAtPath(body):paths[prim.GetName()]=body
    if len(paths)!=2:raise RuntimeError('Cannot locate both tag bodies for contact bond')
    monitor_bond_breaks()
    joint=UsdPhysics.FixedJoint.Define(stage,JOINT_PATH)
    anchor=(xyz[0]+xyz[9])/2
    for slot,i in enumerate((0,9)):
        getattr(joint,f'CreateBody{slot}Rel')().SetTargets([Sdf.Path(paths[f'tag_panel_{i:02d}'])])
        local=rotations[i].T@(anchor-xyz[i]);q=Rotation.from_matrix(rotations[i].T).as_quat()
        getattr(joint,f'CreateLocalPos{slot}Attr')(Gf.Vec3f(*local))
        getattr(joint,f'CreateLocalRot{slot}Attr')(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    joint.CreateBreakForceAttr(8.0);joint.CreateBreakTorqueAttr(.15)
    joint.CreateCollisionEnabledAttr(False)


def step(environment):
    from yam_arena.scene import object_states
    from yam_arena import ik,station
    st=object_states(environment,[f'tag_panel_{i:02d}' for i in range(10)]+['checked_suitcase'])
    panels=[st[f'tag_panel_{i:02d}'] for i in range(10)]
    xyz=np.array([p.pos for p in panels]);R=np.array([Rotation.from_quat(p.quat_xyzw).as_matrix() for p in panels])
    g=geometry(xyz,R,bar_z=1.0965+st['checked_suitcase'].sliders['carry_handle'])
    raw_speed=max(float(np.linalg.norm(p.lin_vel)) for p in panels)
    dt=float(getattr(environment.unwrapped, 'step_dt', .05))
    # Use consecutive corner motions over the last 0.25 s. Raw root velocity
    # can retain solver impulses even when constrained panel poses are static.
    history=STATE.setdefault('position_history', [])
    history.append(panel_corners(xyz,R))
    if len(history)>6: history.pop(0)
    speed=motion_speed(history,dt)
    # Headless GPU simulation does not dispatch contact subscription events.
    # Poll the latest physics substep explicitly, using its dt (not control dt).
    import press_contacts
    # The sealed pair has collisions disabled. PhysX may retain its last
    # manifold, so do not report cached pre-bond impulses as live pressure.
    contacts = [] if STATE['bonded'] else press_contacts.read()
    press_n = patch_press_force(xyz, R, contacts, float(environment.unwrapped.cfg.sim.dt))
    # Extraction uses the actual outlet plane; no policy flags or phase IDs.
    STATE['extracted']=bool(np.max(panel_corners(xyz,R)[:,:,0])<.465)
    if g['under'] and np.min(xyz[:,1])<-.025 and np.max(xyz[:,1])>.025:
        STATE['threaded']=True
    if not STATE['bonded']:
        STATE['contact_s']=press_dwell(STATE['contact_s'],g['contact'],speed,press_n,dt)
        if STATE['contact_s']+1e-9>=PRESS_SECONDS:
            create_bond(environment,xyz,R);STATE['bonded']=True
            STATE['bond_press_n']=press_n
    if g['max_hinge_gap_m']>.008 or (STATE['bonded'] and
            (g['overlap_m2']<.00060 or g['normal_dot']<.985 or
             not -.003<=g['face_gap_m']<=.001)):
        STATE['broken']=True
    base=environment.unwrapped;clear=True;opened=True
    for side in ('left','right'):
        robot=base.scene[side+'_robot'];q=robot.data.joint_pos.torch[0].detach().cpu().numpy()
        opened &= bool(np.mean(q[-2:])<-.025)
        tcp,_=ik.tcp_pose(q[:6],station.BASES[side])
        clear &= bool(np.min(np.linalg.norm(xyz-tcp,axis=1))>.12)
        poses=robot.data.body_pose_w.torch[0].detach().cpu().numpy()
        for name in ('tip_left','tip_right','gripper'):
            p=poses[robot.body_names.index(name),:3]
            clear &= bool(np.min(np.linalg.norm(xyz-p,axis=1))>.075)
    final=final_valid(STATE['extracted'],STATE['bonded'],STATE['broken'],
                      g['linked'],g['barcode_outward'],opened,clear,speed)
    corners=panel_corners(xyz,R)
    if not final:
        STATE['hold_anchor']=None
        STATE['stable_s']=0.
    elif STATE.get('hold_anchor') is None:
        STATE['hold_anchor']=corners.copy()
        STATE['stable_s']=0.
    elif np.max(np.linalg.norm(corners-STATE['hold_anchor'],axis=-1)) > .005:
        STATE['hold_anchor']=corners.copy()
        STATE['stable_s']=0.
    else:
        STATE['stable_s']+=dt
    released_stable=final and STATE['stable_s']>=RELEASE_SECONDS
    value=outcome_score(STATE['extracted'], STATE['bonded'], STATE['broken'],
                        g['linked'], released_stable, STATE['stable_s'], g['barcode_outward'])
    STATE['score']=value
    STATE['diagnostics']={**g,'speed_m_s':speed,'raw_solver_speed_m_s':raw_speed,'hands_clear':bool(clear),'hands_open':bool(opened),
                         'bonded':STATE['bonded'],'broken':STATE['broken'],
                         'bond_break_event':STATE.get('bond_break_event',False),'contact_s':STATE['contact_s'],
                         'patch_press_n':press_n,'patch_contact_points':len(contacts),
                         'pair_force_n':sum(float(np.linalg.norm(v[2])) for v in contacts)/float(environment.unwrapped.cfg.sim.dt),
                         'bond_press_n':STATE.get('bond_press_n',0.),
                         'stable_s':STATE['stable_s'],'score':value}


def outcome_score(extracted, bonded, broken, linked, released_stable, stable_s, barcode_outward=True):
    """Current physical outcome only; extraction/threading alone earn nothing."""
    if broken or not (extracted and bonded and linked and barcode_outward):
        return 0.
    if not released_stable or not np.isfinite(stable_s) or stable_s < RELEASE_SECONDS:
        return .75
    return 1. if stable_s >= SUCCESS_SECONDS else .85


def score(_objects):return float(STATE.get('score',0.))
def diagnostics():return dict(STATE.get('diagnostics',{}))
reset()
