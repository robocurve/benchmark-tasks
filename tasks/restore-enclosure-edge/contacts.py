"""Classify rigid-body contact pairs in both engine adapters."""
def classify(a,b):
    names={a,b}
    robot=any(s.startswith(("left_","right_")) for s in names)
    return ("probe" in names and "bulb" in names,
            robot and bool(names & {"door","sensor","bulb","button","sled","probe","flag","frame"}),
            robot and bool(names & {"door","sensor","bulb","probe"}),
            "operator" in names and (robot or bool(names & {"door","sensor","bulb","probe","sled"})))

class Reader:
    def __init__(self,env):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        import carb.settings
        base=env.unwrapped
        self.sim=base.sim
        if abs(self.sim.get_physics_dt()-1/240)>1e-6 or abs(base.step_dt-.05)>1e-6:
            raise RuntimeError("Expected 240 Hz physics and 20 Hz control")
        self.api=get_physx_simulation_interface()
        self.decode=PhysicsSchemaTools.intToSdfPath
        self.prefix={}
        for key in ("guard","left_robot","right_robot"):
            self.prefix[key]=str(base.scene[key].cfg.prim_path).replace("{ENV_REGEX_NS}","/World/envs/env_0").replace("env_.*","env_0").replace("env_[^/]+","env_0").rstrip("/")+"/"
        self.rig=base.scene["guard"]
        view=getattr(self.rig,"root_view",None)
        if not callable(getattr(view,"get_dof_positions",None)):view=self.rig.root_physx_view
        self.pos=view.get_dof_positions
        self.idx=self.rig.joint_names.index("pointer")
        self.rows=[]
        self.settings=carb.settings.get_settings()
        self.setting="/physics/disableContactProcessing"
        self.old=self.settings.get(self.setting)
        self.settings.set_bool(self.setting,False)
        self.packet=None
        self.sub=self.api.subscribe_contact_report_events(self.event)
        self.saved=self.sim.step
        self.sim.step=self.advance
    def owner(self,val):
        p=str(self.decode(val))
        for name,start in self.prefix.items():
            if p.startswith(start):
                link=p.rsplit("/",1)[-1]
                return link if name=="guard" else name.split("_")[0]+"_"+link
        return "world"
    def pairs(self,h):
        out=[False]*4
        for hit in h:
            if hit.num_contact_data:
                parts=[]
                for actor,collider in ((hit.actor0,hit.collider0),(hit.actor1,hit.collider1)):
                    owner=self.owner(actor)
                    if owner=="frame" and any(n.startswith("operator_") for n in str(self.decode(collider)).split("/")):
                        owner="operator"
                    parts.append(owner)
                flags=classify(*parts)
                out=[x or y for x,y in zip(out,flags)]
        return out
    def event(self,h,p):
        flags=self.pairs(h)
        self.packet=flags if self.packet is None else [a or b for a,b in zip(flags,self.packet)]
    def advance(self,*args,**kwargs):
        self.packet=None
        result=self.saved(*args,**kwargs)
        headers=self.api.get_contact_report()[0]
        flags=self.pairs(headers)
        if self.packet is not None:
            flags=[a or b for a,b in zip(flags,self.packet)]
        from scenekit import _np
        self.rows.append((*flags,max(0.,-float(_np(self.pos())[0,self.idx]))))
        return result
    def read(self):
        rows,self.rows=self.rows,[]
        if len(rows)!=12:raise RuntimeError(f"Incomplete contact window: {len(rows)}")
        return [max(c) for c in zip(*rows)]
    def close(self):
        self.sim.step=self.saved
        self.sub=None
        self.settings.set_bool(self.setting,bool(self.old))
