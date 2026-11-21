"""Stock-dimension removable-straw bottle on the calibrated dual-YAM table."""
import os
from pathlib import Path
import runpy
from types import SimpleNamespace
from xml.etree import ElementTree as ET

import mujoco as mj
import numpy as np

# Use the published reference station. Local validation can use an extracted
# copy of the identical pinned wheel without altering the shared environment.
if station_source := os.environ.get("ROBOCURVE_DREAM_YAM"):
    yam_env = SimpleNamespace(**runpy.run_path(str(Path(station_source) / "yam_env.py")))
else:
    from dream_examples.yam_mujoco import yam_env

TABLE = .760
TABLE_X = (-.65, .65)
TABLE_Y = (-.67, .43)
CAMERAS = {name: name for name in ("top_cam", "left_cam", "right_cam", "judging_front")}
ORIGIN = np.array((.012, -.113, TABLE + .0006))
RELEASE_TRAVEL = .0045
GUIDE_TOP = .155
STRAW_COMPRESSED = .146
STRAW_RISE = .016
# Estimate the span from the catalog's exploded view and keep the inlet
# 9 mm above the base of the vessel when the lid compresses the mouthpiece.
STRAW_SPAN = .148
STRAW_MOUTH_Z = .011
STRAW_INLET_Z = STRAW_MOUTH_Z - STRAW_SPAN
EXTRACTION_START_MAX = STRAW_COMPRESSED + STRAW_RISE + STRAW_INLET_Z + .015
EXTRACTION_MIN_LIFT = .120
MINT = (.62, .75, .60, 1.)
DARK = (.36, .48, .34, 1.)
CLEAR = (.62, .77, .82, .25)
LID_CLEAR = (.67, .76, .77, .48)
STRAW_COLOR = (.92, .91, .80, 1.)


def nums(values):
    return yam_env.numbers(values)


def body(parent, name, position):
    return ET.SubElement(parent, "body", name=name, pos=nums(position))


def geom(parent, name, kind, size, pos, color, **props):
    return ET.SubElement(parent, "geom", name=name, type=kind, size=nums(size),
                         pos=nums(pos), rgba=nums(color), **props)


def box(parent, name, dimensions, pos, color, **props):
    return geom(parent, name, "box", np.asarray(dimensions) / 2, pos, color, **props)


def capsule(parent, name, a, b, radius, color, **props):
    return ET.SubElement(parent, "geom", name=name, type="capsule", size=str(radius),
                         fromto=nums((*a, *b)), rgba=nums(color), **props)


def site(parent, name, p):
    ET.SubElement(parent, "site", name=name, pos=nums(p), size=".001", rgba="0 0 0 0")


def annulus(parent, prefix, outer, inner, height, z, color, slices=24, xy=(0.,0.), **props):
    """Collision-visible polygonal tube with open bore."""
    for i in range(slices):
        angle = 2 * np.pi * i / slices
        r = (outer + inner) / 2
        tangential_half = outer * np.sin(np.pi / slices) * 1.05
        geom(parent, f"{prefix}_{i:02d}", "box", ((outer-inner)/2, tangential_half, height/2),
             (xy[0]+r*np.cos(angle), xy[1]+r*np.sin(angle), z), color,
             quat=nums((np.cos(angle/2), 0, 0, np.sin(angle/2))), **props)


def tube_skin(parent, assets, name, outer, inner, height, center, color):
    count=56
    profile=((inner,center[2]-height/2),(outer,center[2]-height/2),
             (outer,center[2]+height/2),(inner,center[2]+height/2))
    angles=np.arange(count)*2*np.pi/count
    vertices=[(center[0]+r*np.cos(t),center[1]+r*np.sin(t),z)
              for r,z in profile for t in angles]
    faces=[]
    for band in range(4):
        other=(band+1)%4
        for i in range(count):
            a,b=band*count+i, band*count+(i+1)%count
            c,d=other*count+i,other*count+(i+1)%count
            faces.extend(((a,b,d),(a,d,c)))
    ET.SubElement(assets,"mesh",name=name,vertex=nums(np.ravel(vertices)),
                  face=" ".join(map(str,np.ravel(faces))),smoothnormal="true")
    ET.SubElement(parent,"geom",name=name,type="mesh",mesh=name,rgba=nums(color),
                  mass="0",contype="0",conaffinity="0")


def build():
    root = ET.fromstring(yam_env.build_xml())
    root.set("model", "Set a push-button bottle to wide-spout mode")
    root.find("option").attrib.update(timestep=".001", iterations="60", cone="elliptic")
    w = root.find("worldbody")
    assets=root.find("asset")
    for name in ("apple", "banana", "orange", "water_bottle", "bowl", "plate"):
        w.remove(w.find(f"body[@name='{name}']"))
    for asset in list(assets):
        if asset.get("name", "").startswith(("apple_", "banana_", "bowl_", "brown_glaze", "blue_glaze")):
            assets.remove(asset)
    for lamp in w.findall("light"):
        lamp.set("castshadow", "false")
    w.remove(w.find("camera[@name='top']"))
    top = yam_env.calibrated_camera(w, "top_cam", yam_env.CALIBRATION["cameras"]["top"], world=True)
    for side in ("left", "right"):
        w.find(f".//camera[@name='{side}']").set("name", f"{side}_cam")
    # Move the physical camera support to the unlowered calibration, too.
    top_pos = np.fromstring(top.get("pos"), sep=" ")
    stand = w.find("body[@name='center_camera_stand']")
    foot, mast, beam = stand.findall("geom")
    mast_y = top_pos[1] + .07
    foot.set("pos", nums((top_pos[0], mast_y, TABLE+.008)))
    mast.set("fromto", nums((top_pos[0], mast_y, TABLE+.016, top_pos[0], mast_y, top_pos[2])))
    beam.set("fromto", nums((top_pos[0], mast_y, top_pos[2], *top_pos + [0., .02, .02])))
    housing = stand.find("body")
    housing.attrib.update(pos=top.get("pos"), quat=top.get("quat"))
    # Fixed 34 mm reference lens. Retarget the side view to this taller
    # manipulation zone in the Dream reference frame.
    center = ORIGIN + np.array((-.043, .006, .176))
    eye_offset = 1.14 * np.asarray((-.527, -.374, .373))
    yam_env.camera(w, "judging_front", center + eye_offset, center)
    fovy = np.degrees(2. * np.arctan(20.955 * 480 / (2. * 34. * 640)))
    w.find("camera[@name='judging_front']").set("fovy", str(fovy))

    bottle = body(w, "bottle", ORIGIN)
    ET.SubElement(bottle, "freejoint", name="bottle_free")
    hidden=(0,0,0,0)
    annulus(bottle, "bottle_wall", .0388, .0356, .129, .0675, hidden,
            mass=".00565", friction=".72 .008 .001")
    tube_skin(bottle,assets,"bottle_wall_skin",.0388,.0356,.129,(0,0,.0675),CLEAR)
    geom(bottle, "bottle_heel", "cylinder", (.0370, .0020), (0, 0, .0021), (.56,.70,.70,.65),
         mass=".010", friction="1.25 .018 .002")
    geom(bottle, "cap_skirt_mass", "cylinder", (.0410, .0110), (0, 0, .139), hidden,
         mass=".042", contype="0", conaffinity="0")
    geom(bottle, "cap_skirt_skin", "cylinder", (.0410, .0080), (0, 0, .136), MINT,
         mass="0", contype="0", conaffinity="0")
    annulus(bottle, "cap_seat", .041, .029, .006, .147, hidden,
            mass=".0006", friction=".58 .006 .001")
    tube_skin(bottle,assets,"cap_seat_skin",.041,.029,.006,(0,0,.147),MINT)
    annulus(bottle, "wide_spout", .022, .013, .007, .1535, hidden,
            mass=".0002")
    tube_skin(bottle,assets,"wide_spout_skin",.022,.013,.007,(0,0,.1535),DARK)
    # Lower guiding bore only constrains the installed stem, never a released one.
    annulus(bottle, "straw_guide", .010, .0057, .021, .139, MINT,
            slices=12, mass=".00015", friction=".16 .004 .0005")
    for sign in (-1, 1):
        geom(bottle, f"cap_pin_{sign}", "cylinder", (.0045, .006),
             (sign*.014, -.035, .151), DARK, mass=".001",
             quat=".7071067812 0 .7071067812 0")
    site(bottle, "brace_center", (0, .013, .085))

    lid = body(bottle, "transparent_lid", (0, -.035, .152))
    ET.SubElement(lid, "joint", name="lid_hinge", type="hinge", axis="1 0 0",
                  range="0 1.91", stiffness=".043", springref="1.84",
                  damping=".0035", armature=".00008")
    geom(lid, "cover_disc", "cylinder", (.0390,.00225), (0,.034,.0155), LID_CLEAR, mass=".015")
    annulus(lid, "cover_rim", .039, .0365, .013, .008, hidden, xy=(0.,.034), mass=".00045")
    tube_skin(lid,assets,"cover_rim_skin",.039,.0365,.013,(0,.034,.008),LID_CLEAR)
    latch_contact = dict(solref=".004 1", solimp=".95 .998 .0002")
    box(lid, "latch_toe", (.011,.0063,.003), (0,.074,-.0021), DARK, mass=".0009", **latch_contact)
    box(lid, "latch_toe_molding", (.011,.006,.005), (0,.072,.001), DARK,
        mass="0", contype="0", conaffinity="0")
    geom(lid, "straw_compression_pad", "cylinder", (.0086,.0014), (0,.035,.0069), LID_CLEAR,
         mass=".0011")

    button = body(bottle, "button", (0,.0382,.136))
    ET.SubElement(button, "joint", name="button_slide", type="slide", axis="0 -1 0",
                  range="0 .0066", stiffness="550", damping=".21", armature=".00004",
                  solreflimit=".004 1", solimplimit=".95 .998 .0002")
    box(button, "button_face", (.0180,.0070,.0140), (0,.0038,0), MINT,
        mass=".004", friction=".73 .010 .001", contype="4", conaffinity="1")
    for sign in (-1,1):
        box(button, f"latch_stalk_{sign}", (.003,.004,.018),
            (sign*.0088,.0005,.0088), DARK, mass=".0005")
    box(button, "latch_dog", (.020,.005,.002), (0,.0018,.017), DARK, mass=".001", **latch_contact)
    for sign in (-1,1):
        box(button, f"button_runner_{sign}", (.003,.003,.0014),
            (sign*.012,0,.001), MINT, mass=".0005", contype="4", conaffinity="1", **latch_contact)
        box(button, f"button_runner_molding_{sign}", (.005,.003,.0014),
            (sign*.0097,.0008,.001), MINT, mass="0", contype="0", conaffinity="0")
    site(button, "button_contact_point", (0,.0072,0))

    ring = body(bottle, "security_ring", (0,.043,.1260))
    ET.SubElement(ring, "joint", name="ring_hinge", type="hinge", axis="-1 0 0",
                  range="0 2.65", stiffness=".008", springref="1.10",
                  frictionloss=".010", damping=".003", armature=".00003")
    shell_contact = dict(contype="2", conaffinity="1")
    capsule(ring, "ring_left", (-.010,0,.000), (-.010,.001,.0185), .0021, DARK,
            mass=".002", **shell_contact)
    capsule(ring, "ring_right", (.010,0,.000), (.010,.001,.0185), .0021, DARK,
            mass=".002", **shell_contact)
    capsule(ring, "ring_bridge", (-.010,.001,.0185), (.010,.001,.0185), .0021, DARK,
            mass=".003", **shell_contact)
    box(ring, "ring_thumb", (.020,.007,.004), (0,.0047,.0184), DARK,
        mass=".002",
        friction="1.00 .010 .001", **shell_contact)
    # A locking tongue at each side keys into the pusher channels when shut.
    for sign in (-1,1):
        box(ring, f"ring_interlock_{sign}", (.0035,.0018,.0014),
            (sign*.012,-.0089,.0117), DARK, mass=".0004", **latch_contact)
        box(ring, f"ring_tongue_molding_{sign}", (.0032,.009,.002),
            (sign*.0115,-.0046,.0117), DARK, mass="0", contype="0", conaffinity="0")
    site(ring, "ring_bridge_site", (0,.0051,.0184))

    # An installed spring cassette pushes the complete drinking straw upward.
    # Its spring collar is a physical contact surface for withdrawing the cassette.
    straw = body(w, "straw", ORIGIN + (0,0,STRAW_COMPRESSED))
    ET.SubElement(straw, "freejoint", name="straw_free")
    geom(straw, "straw_grip_surface", "cylinder", (.00415,STRAW_MOUTH_Z), (0,0,0), STRAW_COLOR,
         mass=".0027", friction="1.10 .012 .001")
    # Cylinder ends meet the mouthpiece at -11 mm. The inlet and mouth sites
    # mark the geometric endpoints of the complete removable assembly.
    tail_length = -STRAW_MOUTH_Z - STRAW_INLET_Z
    geom(straw, "straw_tail", "cylinder", (.0032,tail_length/2),
         (0,0,STRAW_INLET_Z+tail_length/2), STRAW_COLOR, mass=str(.0042*tail_length/.073))
    geom(straw, "straw_cassette_collar", "cylinder", (.0050,.0030), (0,0,-.0230), MINT,
         mass=".0016", friction=".35 .005 .0005")
    for name in ("straw_grip_surface","straw_tail","straw_cassette_collar"):
        ET.SubElement(root.find("contact"),"pair",geom1="table",geom2=name,
                      condim="6",friction=".36 .36 .005 .00012 .00012")
    site(straw, "straw_grip_site", (0,0,.0022))
    site(straw, "straw_lower_end", (0,0,STRAW_INLET_Z))
    site(straw, "straw_upper_end", (0,0,STRAW_MOUTH_Z))

    # Small compliant tongues provide radial retention. They never attach to
    # the straw. The shape and contact resisting an axial lift are physical.
    for j, angle in enumerate(np.arange(3)*2*np.pi/3):
        radial = np.array((np.cos(angle), np.sin(angle), 0.))
        tongue = body(bottle, f"retainer_{j}", (*(.00550*radial[:2]), .1448))
        ET.SubElement(tongue, "joint", name=f"retainer_slide_{j}", type="slide", axis=nums(radial),
                      range="0 .0016", stiffness="1600", damping=".095", armature=".00001")
        geom(tongue, f"tongue_nose_{j}", "sphere", (.00095,), (0,0,0), DARK, mass=".0005",
             friction=".17 .002 .0005")
    pieces=[g for part in (bottle,straw) for g in part.iter("geom")]
    scale=.244/sum(float(g.get("mass")) for g in pieces)
    for g in pieces:
        g.set("mass",str(float(g.get("mass"))*scale))
    return ET.tostring(root, encoding="unicode")


class Bottle:
    """Observed state, native contact events, and compression spring in the seat."""

    def __init__(self, model, data, unlocked_reset=False):
        self.m, self.d = model, data
        self.lid = model.joint("lid_hinge").qposadr[0]
        self.button = model.joint("button_slide").qposadr[0]
        self.ring = model.joint("ring_hinge").qposadr[0]
        self.straw_dof = model.joint("straw_free").dofadr[0]
        self.bottle_id = model.body("bottle").id
        self.straw_id = model.body("straw").id
        self.geom_names = [model.geom(i).name for i in range(model.ngeom)]
        self.pad_side = {i: 0 if n.startswith("left_") else 1 for i,n in enumerate(self.geom_names)
                         if "_tip_" in n and "_contact_" in n}
        self.bodies = list(model.geom_bodyid)
        self.robot_side = {
            i: int(model.body(j).name.startswith("right_")) for i,j in enumerate(self.bodies)
            if model.body(j).name.startswith(("left_", "right_"))
        }
        descendants = {self.bottle_id,self.straw_id}
        for i, parent in enumerate(model.body_parentid):
            if i and parent in descendants:
                descendants.add(i)
        self.apparatus = {i for i,j in enumerate(self.bodies) if j in descendants}
        self.grippers = [model.site(f"{side}_tcp").id for side in ("left","right")]
        self.pull_anchor = np.full(2,np.nan)
        self.held_lift_m = 0.
        self.tip_clearance_m = None
        self.peak_cross_arm_n = 0.
        self.peak_button_travel_m = 0.
        self.f = np.zeros(6)
        self.peak_button_n = 0.
        self.peak_straw_n = 0.
        self.peak_tilt_deg = 0.
        self.ring_operated = False
        self.unlocked_reset = unlocked_reset
        self.lid_released = False
        self.extracted = False
        self.straw_separated = False
        self.final_hold_s = 0.
        self.faults = set()
        self.max_height = -.1
        self.pad_straw_n = np.zeros(2)
        self.pad_straw_sides = np.zeros((2,2))
        self.native_guide_n = 0.
        self.last_button_n = 0.
        self.last_ring_n = 0.
        self.events = []
        self.table = model.geom("table").id

    def bottle_local(self, p):
        return self.d.xmat[self.bottle_id].reshape(3,3).T @ (p-self.d.xpos[self.bottle_id])

    @staticmethod
    def segment_distance(point, end_a, end_b):
        axis=end_b-end_a
        length_squared=axis@axis
        fraction=np.clip((point-end_a)@axis/length_squared,0.,1.) if length_squared>1e-12 else 0.
        return np.linalg.norm(point-end_a-fraction*axis)

    def spring(self):
        """Reciprocal force delivered only inside the compression seat."""
        d = self.d
        d.qfrc_applied[:] = 0.
        a = self.bottle_local(d.xpos[self.straw_id])
        rb = d.xmat[self.bottle_id].reshape(3,3)
        straw_axis = d.xmat[self.straw_id].reshape(3,3)[:,2]
        if np.linalg.norm(a[:2]) > .003 or np.dot(rb[:,2],straw_axis)<.95:
            return
        compression = np.clip((STRAW_COMPRESSED + STRAW_RISE) - a[2], 0, .026)
        force = rb[:,2] * (67.*compression)
        if not compression:
            return
        d.qfrc_applied[self.straw_dof:self.straw_dof+3] += force
        mj.mj_applyFT(self.m,d,-force,np.zeros(3),d.xpos[self.straw_id],self.bottle_id,d.qfrc_applied)

    def observe(self):
        d = self.d
        self.pad_straw_n.fill(0)
        self.pad_straw_sides.fill(0)
        button = ring = guide = cross_arm = 0.
        touching_apparatus = bottle_on_table = straw_on_table = False
        for i,c in enumerate(d.contact[:d.ncon]):
            one, two = int(c.geom1), int(c.geom2)
            if self.table in (one, two):
                other = two if one == self.table else one
                if self.bodies[other] == self.straw_id:
                    straw_on_table = True
                elif self.bodies[other] == self.bottle_id:
                    bottle_on_table = True
            if one in self.robot_side and two in self.robot_side:
                if self.robot_side[one]!=self.robot_side[two]:
                    mj.mj_contactForce(self.m,d,i,self.f)
                    cross_arm+=max(0,self.f[0])
            if ((one in self.apparatus and two in self.robot_side)
                    or (one in self.robot_side and two in self.apparatus)):
                touching_apparatus=True
            if one not in self.pad_side and two not in self.pad_side:
                pair_names = [self.geom_names[one], self.geom_names[two]]
                if any(x.startswith("tongue_nose") for x in pair_names) and "straw_cassette_collar" in pair_names:
                    mj.mj_contactForce(self.m,d,i,self.f)
                    guide += max(0,self.f[0])
                continue
            if two in self.pad_side:
                one,two=two,one
            n = self.geom_names[two]
            if n.startswith(("straw_", "ring_", "button_")):
                mj.mj_contactForce(self.m,d,i,self.f)
                force=max(0,self.f[0])
                if n.startswith("ring_"):
                    ring += force
                elif n == "button_face":
                    button += force
                elif self.bodies[two] == self.straw_id:
                    side=self.pad_side[one]
                    self.pad_straw_n[side] += force
                    jaw = int("_tip_right_" in self.geom_names[one])
                    self.pad_straw_sides[side,jaw] += force
        self.native_guide_n=guide
        self.last_button_n=button
        self.last_ring_n=ring
        self.peak_cross_arm_n=max(self.peak_cross_arm_n,cross_arm)
        if cross_arm > 1.:
            self.faults.add("cross-arm contact exceeded 1 N")
        self.peak_button_n=max(self.peak_button_n,button)
        self.peak_straw_n=max(self.peak_straw_n,max(self.pad_straw_n))

        angle=float(d.qpos[self.ring])
        lid=float(d.qpos[self.lid])
        press=float(d.qpos[self.button])
        self.peak_button_travel_m=max(self.peak_button_travel_m,press)
        tilt=float(np.degrees(np.arccos(np.clip(d.xmat[self.bottle_id,8],-1,1))))
        self.peak_tilt_deg=max(self.peak_tilt_deg,tilt)
        tip=d.site("straw_lower_end").xpos
        local_tip=self.bottle_local(tip)
        local_head=self.bottle_local(d.xpos[self.straw_id])
        head=d.site("straw_upper_end").xpos
        bottle=d.xpos[self.bottle_id]
        self.max_height=max(self.max_height,float(local_head[2]))
        if tilt>18 or not TABLE-.009<bottle[2]<TABLE+.021:
            self.faults.add("bottle tipped or left the table")
        xmin, xmax = TABLE_X
        ymin, ymax = TABLE_Y
        if not (xmin+.050 <= bottle[0] <= xmax-.050 and ymin+.050 <= bottle[1] <= ymax-.050):
            self.faults.add("bottle over the table edge")
        if (any(not (xmin+.005<=x<=xmax-.005 and ymin+.005<=y<=ymax-.005) for x,y,z in (tip,head))
                or min(tip[2],head[2])<TABLE-.012):
            self.faults.add("straw left the table")
        if ring>.10 and angle>.88 and not self.ring_operated:
            self.ring_operated=True
            self.events.append(("ring released by contact",float(d.time)))
        unlocked = self.ring_operated or self.unlocked_reset
        if button>.20 and press>RELEASE_TRAVEL and unlocked and not self.lid_released:
            self.lid_released=True
            self.events.append(("button pressed beyond release travel",float(d.time)))
        bilateral=np.min(self.pad_straw_sides,axis=1)>.07
        if self.lid_released and lid>1.20 and not self.extracted:
            near_axis = np.linalg.norm(local_tip[:2])<.020
            parallel = np.dot(d.xmat[self.bottle_id].reshape(3,3)[:,2],
                              d.xmat[self.straw_id].reshape(3,3)[:,2])>np.cos(np.deg2rad(20))
            for side, held in enumerate(bilateral):
                if not (held and near_axis and parallel):
                    self.pull_anchor[side]=np.nan
                elif np.isnan(self.pull_anchor[side]) and local_tip[2]<=EXTRACTION_START_MAX:
                    self.pull_anchor[side]=local_head[2]
                elif np.isfinite(self.pull_anchor[side]):
                    lift=local_head[2]-self.pull_anchor[side]
                    self.held_lift_m=max(self.held_lift_m,lift)
                    if lift>=EXTRACTION_MIN_LIFT and local_tip[2]>GUIDE_TOP+.009:
                        self.extracted=True
                        self.tip_clearance_m=float(local_tip[2]-GUIDE_TOP)
                        self.events.append(("held straw lower tip cleared wide spout",float(d.time)))
        if lid>1.20 and not self.lid_released:
            self.faults.add("lid opened without a confirmed button press")
        if self.lid_released and lid < 0.5 and self.extracted:
            self.faults.add("lid closed after extraction")
        bare_spout=self.segment_distance(bottle[:2],head[:2],tip[:2])>.055
        self.straw_separated = bool(bare_spout or min(local_tip[2], self.bottle_local(head)[2])>GUIDE_TOP+.009)
        away=all(np.linalg.norm(d.site_xpos[j]-(bottle+[0,0,.075]))>.115
                 and self.segment_distance(d.site_xpos[j],head,tip)>.055 for j in self.grippers)
        finish=(self.extracted and lid>np.pi/2 and tilt<6 and bare_spout and away
                and bottle_on_table and straw_on_table and not touching_apparatus
                and TABLE-.007<=min(head[2],tip[2])<=max(head[2],tip[2])<=TABLE+.014
                and np.linalg.norm(d.qvel[self.straw_dof:self.straw_dof+3])<.028
                and np.max(self.pad_straw_n)<.06)
        self.final_hold_s=self.final_hold_s+self.m.opt.timestep if finish else 0.

    def report(self):
        lid=float(np.degrees(self.d.qpos[self.lid]))
        score=(1.0 if self.final_hold_s>=1.2 else .75 if self.extracted and self.straw_separated and lid>70 else
               .30 if self.lid_released and lid>70 else 0.0)
        return dict(score=0. if self.faults else score, ring_deg=float(np.degrees(self.d.qpos[self.ring])),
                    lid_deg=lid, button_mm=float(1000*self.d.qpos[self.button]),
                    successful_release=self.lid_released, extracted=self.extracted, straw_separated=self.straw_separated,
                    hold_s=self.final_hold_s, peak_tilt_deg=self.peak_tilt_deg,
                    peak_button_n=self.peak_button_n, peak_straw_pad_n=self.peak_straw_n,
                    peak_button_travel_mm=1e3*self.peak_button_travel_m,
                    peak_cross_arm_n=self.peak_cross_arm_n, held_axial_lift_mm=1e3*self.held_lift_m,
                    tip_clearance_mm=None if self.tip_clearance_m is None else 1e3*self.tip_clearance_m,
                    ring_pad_n=self.last_ring_n, button_pad_n=self.last_button_n,
                    straw_head_mm=float(1000*self.bottle_local(self.d.xpos[self.straw_id])[2]),
                    straw_lower_mm=float(1000*self.bottle_local(self.d.site("straw_lower_end").xpos)[2]),
                    peak_head_mm=float(1000*self.max_height), guide_contact_n=self.native_guide_n,
                    pad_straw_sides_n=self.pad_straw_sides.tolist(), faults=sorted(self.faults))
