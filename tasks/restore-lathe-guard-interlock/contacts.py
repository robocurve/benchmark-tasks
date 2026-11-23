"""Read actual CPU PhysX contact impulses and joint speed at each physics step."""
import numpy as np
from fixture import SHAPES
from sensing import classify,opposed_pads
from logic import COVERED

GEOMS = {shape[0] for shape in SHAPES} | {"yellow_insert"}


class Reader:
    def __init__(self, env):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        import carb.settings
        base = env.unwrapped
        self.sim = base.sim
        self.dt = self.sim.get_physics_dt()
        if str(base.device) != "cpu" or abs(self.dt-1/240)>1e-6 or abs(base.step_dt-.05)>1e-6:
            raise ValueError("Use CPU PhysX at 240 Hz and 20 Hz control")
        self.paths = {}
        for name in ("guard", "insert", "left_robot", "right_robot"):
            path = str(base.scene[name].cfg.prim_path)
            self.paths[name] = path.replace("{ENV_REGEX_NS}","/World/envs/env_0").replace("env_.*","env_0").replace("env_[^/]+","env_0").rstrip("/")
        rig = base.scene["guard"]
        self.view = getattr(rig,"root_view",None)
        if not callable(getattr(self.view,"get_dof_positions",None)):
            self.view = rig.root_physx_view
        self.shield = rig.joint_names.index("shield_angle")
        self.spindle = rig.joint_names.index("shaft")
        self.api = get_physx_simulation_interface()
        self.decode = PhysicsSchemaTools.intToSdfPath
        self.settings = carb.settings.get_settings()
        self.setting = "/physics/disableContactProcessing"
        self.old = self.settings.get(self.setting)
        self.settings.set_bool(self.setting,False)
        self.states = []
        self.error = None
        self.packet = None
        self.sub = self.api.subscribe_contact_report_events(self.callback)
        self.saved = self.sim.step
        self.sim.step = self.advance

    def owner(self, actor):
        for name, root in self.paths.items():
            if actor == root or actor.startswith(root+"/"):
                if name == "guard":
                    return actor.rsplit("/",1)[-1]
                if name == "insert":
                    return name
                return name.split("_")[0]+"_"+actor.rsplit("/",1)[-1]
        return "world"

    def contacts(self, headers, points, operator_pairs=None):
        person = shield = 0.
        tray = touched = dropped = False
        pads = {}
        for h in headers:
            if not h.num_contact_data:
                continue
            owners = [self.owner(str(self.decode(a))) for a in (h.actor0,h.actor1)]
            shapes = [next((n for n in reversed(str(self.decode(c)).split("/")) if n in GEOMS),"") for c in (h.collider0,h.collider1)]
            skin, panel, touch, seat, pad = classify(*owners,*shapes)
            lost = "insert" in owners and ("rotor" in owners or "lathe_bed" in shapes)
            if not any((skin,panel,touch,seat,pad,lost)):
                continue
            load = 0.
            for j in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                p = points[j]
                impulse = p.impulse
                normal = float(impulse) if np.isscalar(impulse) else float(np.dot(tuple(impulse),tuple(p.normal)))
                load += abs(normal)/self.dt
                if pad and abs(normal)/self.dt>.005:
                    direction=np.array(tuple(p.normal))*(1 if owners[0]=="insert" else -1)
                    pads.setdefault(pad,[]).append((np.array(tuple(p.position)),direction))
            if skin:
                person += load
                if operator_pairs is not None and load > 0.:
                    skin_index = 0 if shapes[0].startswith("operator_") else 1
                    other = 1 - skin_index
                    pair = (shapes[skin_index], owners[other], shapes[other])
                    operator_pairs[pair] = operator_pairs.get(pair, 0.) + load
            if panel: shield += load
            if load > .005:
                dropped |= lost
                tray |= seat
                touched |= touch
        bilateral = opposed_pads(pads)
        return person,shield,tray,touched,bilateral,dropped

    def callback(self, headers, points):
        try:
            self.packet_pairs = {}
            self.packet = self.contacts(headers,points,self.packet_pairs)
        except Exception as exc:
            self.error = exc

    def advance(self,*args,**kwargs):
        from scenekit import _np
        self.packet = None
        result = self.saved(*args,**kwargs)
        if self.error: raise RuntimeError("Contact reporting failed") from self.error
        report = self.api.get_contact_report()
        direct_pairs = {}
        direct = self.contacts(report[0],report[1],direct_pairs)
        pairs = self.packet_pairs if self.packet is not None and self.packet[0] > direct[0] else direct_pairs
        values = direct if self.packet is None else tuple(map(max,zip(direct,self.packet)))
        if not self.states:
            self.peak_operator_n = 0.
            self.operator_peak = None
        if values[0] > self.peak_operator_n:
            self.peak_operator_n = values[0]
            self.operator_peak = dict(substep=len(self.states)+1, contacts=[
                dict(surface=key[0], body=key[1], shape=key[2], normal_n=load)
                for key,load in sorted(pairs.items())])
        positions = _np(self.view.get_dof_positions())[0]
        angle = float(positions[self.shield])
        speed = abs(float(_np(self.view.get_dof_velocities())[0,self.spindle])) if angle>COVERED else 0.
        self.states.append((*values,speed,float(positions[self.spindle])))
        return result

    def read(self):
        rows,self.states = self.states,[]
        if len(rows)!=12: raise ValueError(f"Expected 12 physics steps, received {len(rows)}")
        self.shaft_angles = [row[-1] for row in rows]
        return tuple(map(max,zip(*(row[:-1] for row in rows))))

    def close(self):
        self.sim.step = self.saved
        self.sub = None
        self.settings.set_bool(self.setting,bool(self.old))
