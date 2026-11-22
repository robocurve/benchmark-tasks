"""Contact evidence for the scripted demonstration, independent of the task score."""
import math
from collections import Counter
import numpy as np

HANDLES={'left':'guard_finger_handle','right':'bolt_grip'}
PADS=('tip_left','tip_right')
MIN_FORCE=.04
MAX_NORMAL_DOT=-.4

def signature(pads):
    if any(name not in pads for name in PADS):
        return None
    a,b=(pads[name] for name in PADS)
    return [min(a[0],b[0]),float(np.dot(a[1],b[1]))]

def opposed(pads):
    if any(name not in pads for name in PADS):
        return False
    a,b=(pads[name] for name in PADS)
    return min(a[0],b[0])>=MIN_FORCE and float(np.dot(a[1],b[1]))<MAX_NORMAL_DOT

class NativeContacts:
    def __init__(self,env):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        from yam_arena.robot import asset_link_paths
        import carb.settings
        self.settings=carb.settings.get_settings()
        self.key='/physics/disableContactProcessing'
        self.old=self.settings.get(self.key)
        self.dt=float(env.unwrapped.sim.get_physics_dt())
        self.steps=round(float(env.unwrapped.step_dt)/self.dt)
        if self.steps<1 or not math.isclose(self.dt*self.steps,.05,abs_tol=1e-5):
            raise ValueError('Unexpected physics to action timing')
        paths=asset_link_paths()
        self.roots={}
        self.fingers={}
        for side in HANDLES:
            robot=str(env.unwrapped.scene[side+'_robot'].cfg.prim_path)
            robot=robot.replace('env_.*','env_0').replace('env_[^/]+','env_0').replace('{ENV_REGEX_NS}','/World/envs/env_0')
            self.roots[side]=robot
            for pad in PADS:self.fingers[robot+paths[pad]]=(side,pad)
        self.decode=PhysicsSchemaTools.intToSdfPath
        self.raw=[]
        self.error=None
        self.physics_device = str(env.unwrapped.device)
        if self.physics_device != 'cpu':
            raise RuntimeError('Guard qualification requires CPU PhysX and RTX cameras; got '+self.physics_device)
        self.settings.set_bool(self.key,False)
        self.engine = env.unwrapped.sim
        self.interface = get_physx_simulation_interface()
        self.step_before = self.engine.step
        self.event_batches = []
        self.headers_seen = 0
        self.points_seen = 0
        self.polls = 0
        self.deliveries = 0
        self.sub = self.interface.subscribe_contact_report_events(self.buffer_event)
        self.engine.step = self.step_with_reports

    def buffer_event(self, headers, points):
        # Decode immediately. PhysX owns the buffer for one physics step.
        self.on_contact(headers,points)
        if self.error is None:
            self.event_batches.append((len(headers),len(points),self.raw.pop()))

    def step_with_reports(self, *args, **kwargs):
        self.event_batches.clear()
        result = self.step_before(*args, **kwargs)
        report = self.interface.get_contact_report()
        headers, points = report[:2]
        if len(self.event_batches)>1:
            raise RuntimeError('Multiple contact event deliveries in a single solver advance')
        # Count each completed physics step exactly once. A subscribed report
        # can already have consumed the buffer before the synchronous read.
        if len(headers) or not self.event_batches:
            self.on_contact(headers,points)
            nh,npnt=len(headers),len(points)
        else:
            nh,npnt,decoded=self.event_batches[0]
            self.raw.append(decoded)
        self.headers_seen+=nh
        self.points_seen+=npnt
        self.deliveries+=len(self.event_batches)
        self.polls+=1
        return result
    def on_contact(self,headers,points):
        try:
            sample={s:{} for s in HANDLES}
            pairs=set()
            flags=set()
            for h in headers:
                if not h.num_contact_data:continue
                actors=[str(self.decode(i)) for i in (h.actor0,h.actor1)]
                shapes=[str(self.decode(i)) for i in (h.collider0,h.collider1)]
                sides=[next((s for s,r in self.roots.items() if actor.startswith(r+'/')),None) for actor in actors]
                if set(sides)=={'left','right'}:flags.add('arm_to_arm')
                if any('/machine_guard/'+body+'/' in path for path in shapes for body in ('door','bolt','switch')):
                    flags.add('shape_pair|'+'|'.join(sorted(shapes)))
                names={path.rsplit('/',1)[-1] for path in shapes}
                if all('/machine_guard/' in path for path in shapes) and names=={'e_right','keeper_groove'}:flags.add('keeper_bearing')
                if all('/machine_guard/' in path for path in shapes) and names=={'sensing_plunger','door_tongue'}:flags.add('tongue_contact')
                if all('/machine_guard/' in path for path in shapes):
                    if 'lock_shaft' in names:
                        for wall in ('e_left','e_right','e_top','e_bottom'):
                            if wall in names:flags.add('shaft_contact_'+wall)
                    if names=={'striker_stop','polycarbonate_guard'}:flags.add('door_on_striker')
                for i,side in enumerate(sides):
                    if side and '/machine_guard/' in shapes[1-i]:
                        flags.add('robot_fixture')
                        hand=self.fingers.get(actors[i])
                        other=shapes[1-i].rsplit('/',1)[-1]
                        if hand is None or other!=HANDLES[side]:
                            flags.add('unexpected_robot_fixture')
                for i,actor in enumerate(actors):
                    if actor not in self.fingers:continue
                    side,pad=self.fingers[actor]
                    other=shapes[1-i].split('/')[-1]
                    pairs.add((side,pad,other))
                    if '/machine_guard/' not in shapes[1-i] or other!=HANDLES[side]:continue
                    for j in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                        pt=points[j]
                        n=np.array(tuple(pt.normal),float)
                        norm=float(np.linalg.norm(n))
                        if not math.isfinite(norm) or norm<.9:continue
                        n=n/norm
                        raw=pt.impulse
                        impulse=np.asarray(raw if isinstance(raw,(float,int,np.number)) else tuple(raw),float)
                        f=abs(float(impulse) if impulse.ndim==0 else float(impulse@n))/self.dt
                        if not math.isfinite(f):raise ValueError('Nonfinite contact impulse')
                        if f>sample[side].get(pad,(0,None))[0]:
                            sample[side][pad]=(f,(n if i==0 else -n).tolist())
            self.raw.append((sample,sorted(pairs),flags))
        except Exception as exc:
            self.error=exc
    def read(self):
        if self.error is not None:raise RuntimeError('PhysX contact report decoding failed') from self.error
        callbacks=self.raw
        self.raw=[]
        if len(callbacks)>self.steps:
            raise ValueError('More contact callback batches than physics steps in action')
        counts=Counter()
        pairs=set()
        last={s:{} for s in HANDLES}
        flags=set()
        signatures={s:[] for s in HANDLES}
        for last,contacts,events in callbacks:
            flags.update(events)
            pairs.update(contacts)
            for s in HANDLES:
                counts[s]+=int(opposed(last[s]))
                signatures[s].append(signature(last[s]))
        info={}
        if hasattr(self,'polls'):
            if self.polls!=self.steps or len(callbacks)!=self.steps:
                raise RuntimeError(f'Contact observation incomplete: {self.polls} of {self.steps} solver steps')
            info=dict(polled_steps=self.polls,report_headers=self.headers_seen,contact_points=self.points_seen,
                      contact_backend='cpu_physx',event_deliveries=self.deliveries)
            self.polls=self.headers_seen=self.points_seen=self.deliveries=0
        shape_pairs = sorted(event for event in flags if event.startswith('shape_pair|'))
        flags.difference_update(shape_pairs)
        return dict(moving_fixture_pairs=[event.split('|')[1:] for event in shape_pairs],
                    physics_steps=self.steps,callbacks=info.get('event_deliveries',len(callbacks)),**info,events=sorted(flags),signatures=signatures,pairs=sorted(pairs),pads=last,
                    bilateral_substeps={s:counts[s] for s in HANDLES})
    def close(self):
        self.engine.step=self.step_before
        self.sub=None
        if self.old is None:self.settings.destroy_item(self.key)
        else:self.settings.set_bool(self.key,bool(self.old))

def verified(sample,side):
    n=sample['physics_steps']
    entries=sample['signatures'][side]
    if not isinstance(n,int) or n<1 or len(entries)>n:
        raise ValueError('Invalid physics sample count')
    hits=0
    for item in entries:
        if item is None:continue
        f,dot=item
        if not (math.isfinite(f) and f>=0 and math.isfinite(dot) and abs(dot)<=1.00001):
            raise ValueError('Invalid pad contact signature')
        hits+=int(f>=MIN_FORCE and dot<MAX_NORMAL_DOT)
    if hits!=sample['bilateral_substeps'][side]:
        raise ValueError('Opposed-contact count disagrees with force/normal samples')
    return hits*3>=2*n

def supporting_force(signatures, steps):
    """Weaker-pad normal force sustained by opposed pads over two thirds of a tick."""
    count = (2 * steps + 2) // 3
    forces = sorted((pair[0] for pair in signatures if pair is not None
                     and pair[1] < MAX_NORMAL_DOT), reverse=True)
    return forces[count - 1] if count and len(forces) >= count else 0.

def check_reference(rows):
    """Demand centered, opposed contacts before motion and during every working action."""
    report={}
    for r in rows:
        for side in HANDLES:verified(r['contact'],side)
        if r['phase'] in ('approach_bolt','grasp_bolt','hold_bolt_alignment','advance_bolt') and not verified(r['contact'],'left'):
            raise ValueError('Door lost opposed support during right-hand work')
    for side,acquire,move,joint in [('left','grasp_door','close_door','hinge'),('right','grasp_bolt','advance_bolt','bolt_slide')]:
        before=[i for i,r in enumerate(rows) if r['phase']==acquire]
        moving=[i for i,r in enumerate(rows) if r['phase']==move]
        if not before or not moving or max(before)>=min(moving):
            raise ValueError('Missing acquisition before '+move)
        lead=rows[before[-1]+1:moving[0]]
        if (len(before)<3 or any(r['phase']!='hold_bolt_alignment' or side!='right' or
                                 not verified(r['contact'],side) for r in lead)
                or not all(verified(rows[i]['contact'],side) for i in before[-3:])):
            raise ValueError('No sustained opposed fingertip contact before '+move)
        flags=[verified(rows[i]['contact'],side) for i in moving]
        coverage=sum(flags)/len(flags)
        if not all(flags):
            raise ValueError(f'{side} opposed contact on only {coverage:.1%} of {move}')
        rest=rows[0]['measured_joint_positions'][joint]
        drift=max(abs(r['measured_joint_positions'][joint]-rest) for r in rows[:moving[0]])
        if drift>(math.radians(3) if side=='left' else .003):
            raise ValueError(f'{joint} moved materially before grasp confirmation')
        def location(i):
            vector=np.asarray(rows[i]['measured_arms'][side]['handle_in_jaws_mm'],dtype=float)
            if vector.shape!=(3,) or not np.isfinite(vector).all():
                raise ValueError('Invalid handle-to-gripper transform')
            return vector
        first=next(i for i in before if verified(rows[i]['contact'],side))
        origin=location(first)
        registered=location(before[-1])
        acquisition=max(float(np.linalg.norm(location(i)-origin)) for i in range(first,before[-1]+1))
        stroke=max(float(np.linalg.norm(location(i)-registered)) for i in moving)
        if acquisition>3.5 or stroke>8.0:
            raise ValueError(f'{side} handle moved inside jaws: acquisition {acquisition:.2f} mm, stroke {stroke:.2f} mm')
        report[side]=dict(phase=move,acquisition_excursion_mm=acquisition,stroke_excursion_mm=stroke,verified_ticks=sum(flags),motion_ticks=len(flags),coverage=coverage,
                          displacement_before_motion=drift)
    return report
