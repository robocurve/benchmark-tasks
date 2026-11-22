"""Sample fabric clamp load, feed-table support and transient wrist extension at each PhysX advance."""
import numpy as np
from handle_approach import Acquisition,camera_clearance
from sleeve_contact import PADS, SLEEVE_COLLIDERS, sleeve_pinch
from fixture import K_LOAD,WORLD,SIDES,SHAPES
SHOE_SHAPES={name for name,owner,*_ in SHAPES if owner=="shoe"}
def knob_pinch(samples,angle):
    """Two different fingertips must load opposite physical sides in one physics advance."""
    cs,sn=np.cos(angle),np.sin(angle)
    axis=np.array([cs,0.,-sn])
    centre=np.asarray(WORLD)+(.060-.036*cs+.081*sn,.120,.088+.036*sn+.081*cs)
    sides={"tip_left":[],"tip_right":[]}
    for pad,point,normal in samples:
        offset=(point-centre)@axis
        lateral=normal@axis
        if abs(offset)>.002 and abs(lateral)>.20:
            sides[pad].append((offset,lateral))
    return any(a[0]*b[0]<0 and a[1]*b[1]<0 for a in sides["tip_left"] for b in sides["tip_right"])

class Reader:
    def __init__(self,env):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        import carb.settings
        base=env.unwrapped
        self.dt=float(base.sim.get_physics_dt());self.n=round(float(base.step_dt)/self.dt)
        if str(base.device)!="cpu" or abs(self.dt-1/240)>1e-6 or self.n!=12:
            raise RuntimeError("Use CPU PhysX at 240 Hz with 20 Hz control")
        self.rig=base.scene["ironer"]
        self.ext=self.rig.joint_names.index("extension");self.lift=self.rig.joint_names.index("lift")
        view=getattr(self.rig,"root_view",None)
        if not callable(getattr(view,"get_dof_positions",None)):view=self.rig.root_physx_view
        self.positions=view.get_dof_positions
        from yam_arena import station,ik,YAM_URDF
        from yam_arena.constants import ALL_JOINTS,FINGER_OPEN
        self.approach=Acquisition()
        self.handle_pairs=set()
        left=base.scene["left_robot"]
        arm_view=getattr(left,"root_view",None)
        if not callable(getattr(arm_view,"get_dof_positions",None)):
            arm_view=left.root_physx_view
        indices=[left.joint_names.index(n) for n in ALL_JOINTS]
        mount=np.array(station.wrist_camera("left").pos)
        offset=np.array(station.BASES["left"])-WORLD
        def read_camera(angle):
            from scenekit import _np
            measured=_np(arm_view.get_dof_positions())[0][indices]
            mat=ik.forward_kinematics(YAM_URDF,dict(zip(ALL_JOINTS,measured)))["gripper"]
            point=offset+mat[:3,3]+mat[:3,:3]@mount
            return camera_clearance(point,angle),min(measured[-2:]/FINGER_OPEN)
        self.read_camera=read_camera
        def path(key):
            return str(base.scene[key].cfg.prim_path).replace("{ENV_REGEX_NS}","/World/envs/env_0").replace("env_.*","env_0").replace("env_[^/]+","env_0").rstrip("/")+"/"
        self.paths={key:path(key) for key in ("ironer","left_robot","right_robot")}
        self.decode=PhysicsSchemaTools.intToSdfPath
        self.sim=base.sim;self.api=get_physx_simulation_interface()
        self.settings=carb.settings.get_settings();self.key="/physics/disableContactProcessing"
        self.old=self.settings.get(self.key);self.settings.set_bool(self.key,False)
        self.saved=self.sim.step;self.rows=[];self.packet=None;self.error=None
        self.sub=self.api.subscribe_contact_report_events(self.callback);self.sim.step=self.advance
    def owner(self,id_):
        path=str(self.decode(id_))
        for name,prefix in self.paths.items():
            if path.startswith(prefix):return path.rsplit("/",1)[-1] if name=="ironer" else name
        return "other"
    def surfaces(self,actors,shapes):
        """The fixed mounting joint is excluded from right-moving-link checks."""
        found=set()
        for index,actor in enumerate(actors):
            if not actor.startswith(self.paths["right_robot"]):continue
            link=actor.rsplit("/",1)[-1]
            if link=="base":continue
            other=1-index
            collider=shapes[other].split("/")
            opposite=actors[other]
            if "feed_table" in collider or "exit_shelf" in collider or any(k.lower()=="table" for k in collider):
                found.add(link+" / "+shapes[other])
            if opposite.endswith("/base") and (opposite.startswith(self.paths["left_robot"]) or
                (opposite.startswith(self.paths["right_robot"]) and link!="link1")):
                found.add(link+" / "+opposite)
        return found

    def contacts(self,headers,points):
        nip=rest=hand=0.;touch=lever=arm=glove=False;sleeve_samples=[];knob=[];surfaces=set()
        handle_pairs=set()
        for h in headers:
            if not h.num_contact_data:continue
            actors=[str(self.decode(x)) for x in (h.actor0,h.actor1)]
            shapes=[str(self.decode(x)) for x in (h.collider0,h.collider1)]
            surface_pair=self.surfaces(actors,shapes)
            pair={self.owner(h.actor0),self.owner(h.actor1)}
            left_shape=None
            pad=None
            if pair=={"left_robot","shoe"}:
                li=int(actors[1].startswith(self.paths["left_robot"]))
                link=actors[li].rsplit("/",1)[-1]
                left_shape=next((part for part in reversed(shapes[1-li].split("/")) if part in SHOE_SHAPES),"unknown")
                if "release_grip" in shapes[1-li].split("/") and actors[li].rsplit("/",1)[-1] in ("tip_left","tip_right"):
                    pad=actors[li].rsplit("/",1)[-1]

            touch|=bool(pair & {"left_robot","right_robot"}) and bool(pair-{"left_robot","right_robot","other"})
            lever|=pair=={"left_robot","shoe"};arm|=pair=={"right_robot","arm"}
            sleeve_pad=None
            if pair=={"right_robot","arm"}:
                ri=int(actors[1].startswith(self.paths["right_robot"]))
                link=actors[ri].rsplit("/",1)[-1]
                if link in PADS and SLEEVE_COLLIDERS.intersection(shapes[1-ri].split("/")):
                    sleeve_pad=link
            fabric="shoe" in pair and any(n.startswith("fabric") for n in pair)
            table=pair=={"frame","hand"}
            glove|="hand" in pair and bool(pair & {"left_robot","right_robot"})
            hit="hand" in pair and bool(pair & {"shoe","left_robot","right_robot"})
            if not (fabric or table or hit or left_shape or pad or sleeve_pad or surface_pair):continue
            for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                p=points[i];imp=p.impulse
                normal=abs(float(imp)) if np.isscalar(imp) else abs(float(np.dot(tuple(imp),tuple(p.normal))))
                if left_shape and (normal/self.dt>.01 or p.separation<=0.):
                    handle_pairs.add((link,left_shape))
                if surface_pair and (normal/self.dt>.01 or p.separation<=0.):
                    surfaces.update(surface_pair)
                if pad and normal/self.dt>.01:
                    knob.append((pad,np.array(tuple(p.position)),np.array(tuple(p.normal))*(1 if li==0 else -1)))
                if sleeve_pad and normal/self.dt>.01:
                    direction=np.array(tuple(p.normal))*(1 if ri==0 else -1)
                    sleeve_samples.append((sleeve_pad,tuple(p.position),direction,normal/self.dt))
                if hit:hand+=normal/self.dt
                if fabric:nip+=normal/self.dt
                if (table and -.418<=float(p.position[0])-WORLD[0]<=-.020
                    and -.255<=float(p.position[1])-WORLD[1]<=.169 and abs(float(p.position[2])-WORLD[2]-.070)<.004):
                    rest+=normal*abs(float(p.normal[2]))/self.dt
        self.handle_pairs=handle_pairs
        return nip,rest,touch,lever,arm,hand,sleeve_pinch(sleeve_samples),glove,knob,surfaces
    def callback(self,h,p):
        try:self.packet=self.contacts(h,p)
        except Exception as exc:self.error=exc
    def advance(self,*args,**kwargs):
        self.packet=None
        result=self.saved(*args,**kwargs)
        if self.error:raise RuntimeError("Contact event decoding failed") from self.error
        h,p=self.api.get_contact_report()[:2]
        contact=self.contacts(h,p) if len(h) or self.packet is None else self.packet
        from scenekit import _np
        joints=_np(self.positions())[0]
        q=float(joints[self.ext])
        angle=float(joints[self.lift])
        pinch=knob_pinch(contact[-2],angle)
        margin,opening=self.read_camera(angle)
        self.approach.observe(angle,margin,opening,self.handle_pairs,pinch)
        contact=(*contact[:-2],pinch,contact[-1])
        if not np.isfinite(q):raise ValueError("Nonfinite wrist extension")
        self.rows.append((max(0.,q)*K_LOAD,*contact))
        return result
    def read(self):
        rows,self.rows=self.rows,[]
        if len(rows)!=self.n:raise RuntimeError(f"Missing simulation advances: {len(rows)}/{self.n}")
        peak,nip,rest,touch,lever,arm,hand,opposed,glove,pinch,surfaces=zip(*rows)
        self.opposed_knob=any(pinch);self.surface_contacts=sorted(set().union(*surfaces))
        self.glove_touch=any(glove)
        self.sleeve_steps=sum(int(paired) for paired,_ in opposed)
        self.sleeve_pad_peak={pad:max(loads[pad] for _,loads in opposed) for pad in PADS}
        self.opposed=self.sleeve_steps>0
        return max(peak),float(np.mean(nip)),float(np.mean(rest)),any(touch),any(lever),any(arm),max(hand)
    def close(self):
        self.sim.step=self.saved;self.sub=None;self.settings.set_bool(self.key,bool(self.old))
