"""Install a table-lamp harp and seat its shade before fitting the finial."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_contacts import ArmContacts
from compound import Link, Part, env, on_reset, on_step
import geometry as geo
from pose import as_waypoint_rotation, install_expert, ready_pose
from scoring import Assembly, LegGauge, base_supported
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = "Fit the harp into the lamp, lower both retaining sleeves, and seat the shade on the harp."
TOP = TABLE_TOP[SCENE]
BASE_RADIUS = .071
BASE_BOTTOM_OFFSET = .105
LAMP = np.array([.302,0.,TOP+BASE_BOTTOM_OFFSET])
REST = np.array([.157,0.,TOP+.009])
SHADE = np.array([.365,.183,TOP])
FINIAL = np.array([.315,-.159,TOP])
GRIP_SHIFT = np.array([0.,0.,.006])
SIGNS = np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])
SIDES = ("left","right")


def xyz(pos):
    return tuple(float(v) for v in pos)


def mobile(name, pos, size, mass, color, **shapes):
    return Part(name=name, pos=xyz(pos), size=size, mass=mass, color=color, **shapes)


lamp = mobile("table_lamp",LAMP,(.142,.142,.186),1.35,geo.BRASS,friction=.72,
              boxes=geo.saddle_boxes(),wedges=geo.saddle_entrances()+geo.ring(.012,.017,.004,.031,20),
              cylinders=(((0.,0.,-.096),BASE_RADIUS,.018,"Z",geo.DARK),
                         ((0.,0.,-.066),.030,.044,"Z",geo.DARK),
                         ((0.,0.,-.030),.011,.036,"Z",geo.BRASS),
                         ((0.,0.,.004),.017,.012,"Z",geo.DARK)))
members = tuple(
    Link(f"{side}_leg","body",(0.,s*geo.LEG_Y,geo.LEG_PIVOT),"X",
         (-.105,.035) if s == 1 else (-.035,.105),
         .016,geo.BRASS,boxes=geo.leg_boxes(s),stiffness=6.5,damping=.20,friction=.66)
    for side,s in zip(SIDES,(1,-1))
)
sleeves = tuple(
    mobile(f"{side}_sleeve",REST+[0.,s*geo.LEG_Y,geo.COLLAR_Z],
           (.019,.019,geo.COLLAR_LENGTH),.009,geo.LIGHT_BRASS,friction=.82,
           wedges=geo.sleeve_body())
    for side,s in zip(SIDES,(1,-1))
)
harp = mobile("harp",REST,(.022,.103,.222),.036,geo.BRASS,friction=.56,links=tuple(members),
              wedges=geo.harp_crown()+geo.offset(geo.threads(False)+geo.post_tip(),z=geo.HARP_HEIGHT),
              cylinders=(((0.,0.,geo.HARP_HEIGHT-.00175),.009,.0035,"Z",geo.BRASS),
                         ((0.,0.,geo.HARP_HEIGHT+.0036),geo.POST_ROOT_RADIUS,.0072,"Z",geo.LIGHT_BRASS)),
              boxes=(((0.,geo.LEG_Y,geo.LEG_PIVOT+.002),(.0035,.0035,.005),geo.BRASS),
                     ((0.,-geo.LEG_Y,geo.LEG_PIVOT+.002),(.0035,.0035,.005),geo.BRASS),
                     ((0.,0.,geo.HARP_HEIGHT-.002),(.010,.018,.0035),geo.BRASS)))
rack_boxes = [((0.,0.,-.0045),(.069,.135,.009),geo.DARK)]
for side in (-1,1):
    for x in (-.00425,.00425):
        rack_boxes.append(((x,side*geo.LEG_Y,.003),(.004,.004,.006),geo.DARK))
rack = mobile("rest",REST,(.069,.135,.015),.54,geo.DARK,friction=.7,boxes=tuple(rack_boxes))
shade = mobile("shade",SHADE,(.216,.216,geo.SHADE_HEIGHT),.067,geo.LINEN,friction=.55,
               wedges=geo.shade_wedges())
finial = mobile("finial",FINIAL,(.024,.024,geo.FINIAL_HEIGHT),.018,geo.LIGHT_BRASS,friction=.72,
                wedges=geo.finial_body(),
                cylinders=(((0.,0.,.0082),.0058,.002,"Z",geo.LIGHT_BRASS),
                           ((0.,0.,.011),.009,.007,"Z",geo.LIGHT_BRASS),
                           ((0.,0.,.0175),.012,.007,"Z",geo.LIGHT_BRASS),
                           ((0.,0.,.0225),.008,.003,"Z",geo.LIGHT_BRASS)))
OBJECTS = [lamp,rack,harp,*sleeves,shade,finial]


STATE = {}


@on_reset
def reset(_):
    STATE.clear()
    STATE.update(step=0,assembly=Assembly(),contacts=None)
    path = os.environ.get("HARP_ARM_CONTACTS")
    if path:
        from isaaclab_physx.physics.physx_manager import PhysxManager

        STATE["contacts"] = ArmContacts(path)
        if not getattr(PhysxManager.step,"_a1_harp",False):
            native_step = PhysxManager.step.__func__

            def step(cls):
                native_step(cls)
                STATE["contacts"].poll(cls.get_physics_dt())

            step._a1_harp = True
            PhysxManager.step = classmethod(step)


@on_step
def tick(_):
    STATE["step"] += 1


def relative(origin, point):
    return origin.rotation().T @ (point-origin.pos)


def hand_poses():
    scene = env().unwrapped.scene
    for side in SIDES:
        arm = scene[f"{side}_robot"]
        ids,_ = arm.find_joints([*ARM_JOINTS,*FINGER_JOINTS],preserve_order=True)
        q = arm.data.joint_pos.torch[0,ids].detach().cpu().numpy()
        p,r = ik.tcp_pose(q[:6],station.BASES[side])
        yield p,r,max(abs(q[6:]))


def commanded_hands():
    manager = env().unwrapped.action_manager
    return [ik.tcp_pose(manager.get_term(f"{side}_arm").processed_actions[0].detach().cpu().numpy(),
                        station.BASES[side]) for side in SIDES]


def commanded_tcp():
    return [position.tolist() for position, _ in commanded_hands()]


def jaw_boxes(hands):
    for p,r,gap in hands:
        hand = p + (SIGNS*[gap+.013,.039,.033]+[0.,0.,-.016]) @ r.T
        yield hand.min(0),hand.max(0)


def score(objects):
    assembly,step = (STATE[k] for k in ("assembly","step"))
    if step == assembly.last_tick:
        return assembly.score
    fixture,loop,cover = (objects[k] for k in ("table_lamp","harp","shade"))
    pair = []
    coords = {}
    for side,s in zip(SIDES,(1,-1)):
        p,r = loop.links[f"{side}_leg"]
        toe = relative(fixture,p+r @ [0.,0.,-geo.LEG_PIVOT])
        sleeve = objects[f"{side}_sleeve"]
        sleeve_p,sleeve_r = sleeve.pos,sleeve.rotation()
        sleeve_axis = sleeve_r[:,2]
        leg_axis = r[:,2]
        sleeve_offset = sleeve_p-p
        slide = float(sleeve_offset @ leg_axis)+geo.LEG_PIVOT-geo.COLLAR_Z
        on_leg = bool(sleeve_axis @ leg_axis > .985
                      and np.linalg.norm(sleeve_offset-(sleeve_offset @ leg_axis)*leg_axis) < .003)
        ends = [relative(fixture,sleeve_p+sleeve_axis*z)
                for z in (-geo.COLLAR_LENGTH/2,geo.COLLAR_LENGTH/2)]
        lo,hi = sorted(v[2] for v in ends)
        overlap = s*toe[1]+geo.TOE_OUT-geo.LIP_INNER
        sleeve_cover = geo.sleeve_blocking_band(lo,hi)
        gauge = LegGauge(float(toe[2]),float(overlap),float(sleeve_cover))
        in_width = abs(toe[0]) < .0024 and .039 < s*toe[1] < .044
        pair.append((in_width and gauge.hooked,gauge.retained and on_leg))
        coords[side] = dict(toe=toe.tolist(),sleeve_ends=[lo,hi],overlap=overlap,
                           blocking_band=sleeve_cover,
                           sleeve_travel=slide,sleeve_pos=sleeve_p.tolist(),
                           sleeve_axis=sleeve_axis.tolist(),on_leg=on_leg)
    cover_p = cover.pos+cover.rotation() @ [0.,0.,geo.SHADE_HEIGHT-geo.SHADE_WASHER]
    post_p = loop.pos+loop.rotation() @ [0.,0.,geo.HARP_HEIGHT]
    cover_error = relative(loop,cover_p)-[0.,0.,geo.HARP_HEIGHT]
    loop_r = loop.rotation()
    shade_seated = bool(np.linalg.norm(cover_error[:2]) < .0021 and abs(cover_error[2]) < .0014
                        and (loop_r.T @ cover.rotation())[2,2] > .998)
    notches,sleeves = (all(v) for v in zip(*pair))
    independent = False
    path = os.environ.get("HARP_DIAGNOSTICS")
    hands = list(hand_poses()) if path or (notches and sleeves) else []
    if notches and sleeves:
        saddle = fixture.pos
        lo,hi = saddle+[-.021,-.064,-.007],saddle+[.021,.064,.034]
        independent = all(np.maximum(lo-hand_hi,hand_lo-hi).max() > .004
                          for hand_lo,hand_hi in jaw_boxes(hands))
    release = False
    if independent and shade_seated:
        lower = np.minimum(cover.pos-[.108,.108,0.],loop.pos-[.014,.054,0.])
        upper = post_p+[.108,.108,geo.POST_LENGTH]
        clear = all(np.maximum(lower-hi,lo-upper).max() > .008 for lo,hi in jaw_boxes(hands))
        release = clear and all(np.linalg.norm(part.lin_vel) < .004
                                and np.linalg.norm(part.ang_vel) < .12
                                for part in (fixture,loop,cover))
    valid = base_supported(fixture.pos,fixture.rotation()[2,2],station.TABLE_CENTER,
                           station.TABLE_SIZE,TOP,BASE_RADIUS,BASE_BOTTOM_OFFSET)
    value = assembly.update(step,notches,sleeves and independent,shade_seated,release,valid)
    if path:
        commanded = commanded_hands()
        with open(path,"a") as stream:
            stream.write(json.dumps(dict(step=step,score=value,valid=valid,legs=coords,notches=notches,
                sleeves=sleeves,independent=independent,shade_seated=shade_seated,
                shade_error=cover_error.tolist(),spindle=post_p.tolist(),
                harp_axis=loop_r[:,2].tolist(),shade_axis=cover.rotation()[:,2].tolist(),
                harp=loop.pos.tolist(),shade=cover.pos.tolist(),
                lamp=fixture.pos.tolist(),release=release,tcp=[p.tolist() for p,_,_ in hands],
                finger_gap_m=[float(gap) for _,_,gap in hands],
                commanded_tcp=[p.tolist() for p,_ in commanded],
                **(STATE["contacts"].snapshot() if STATE["contacts"] else {})))+"\n")
    return value


def success(objects):
    return score(objects) == 1.


def hand_rotation(side, tilt=-34., yaw=0.):
    s = 1 if side == "left" else -1
    sideways = axis_angle_matrix(np.array([1.,0.,0.]),math.radians(s*tilt))
    twist = axis_angle_matrix(np.array([0.,0.,1.]),yaw)
    return as_waypoint_rotation(twist @ sideways @ grip_rotation(-s*math.pi/2))


def axis_rotation(source,target):
    tilt_axis = np.cross(source,target)
    tilt_angle = math.atan2(np.linalg.norm(tilt_axis),float(source @ target))
    return axis_angle_matrix(tilt_axis,tilt_angle) if tilt_angle > 1e-7 else np.eye(3)


def demo(*, shade_start=None, spindle=None, spindle_axis=None, shade_grasp=None):
    left,right = [],[]
    opened,closed = FINGER_OPEN,FINGER_CLOSED
    pick = np.array([REST+[0.,s*geo.LEG_Y,geo.COLLAR_Z]+GRIP_SHIFT for s in (1,-1)])
    work = pick+LAMP-REST
    perch = pick+[0.,0.,geo.CARRY_SLEEVE_RISE]
    expanded = work+[0.,0.,geo.CARRY_SLEEVE_RISE]
    squeeze = np.array([[0.,-.0107,0.],[0.,.0107,0.]])
    deep_squeeze = np.array([[0.,-.018,0.],[0.,.018,0.]])
    docking = np.array([[.0004,0.,0.],[.0023,0.,0.]])
    seat = expanded+docking+[0.,0.,-.002]
    outside = np.array([[0.,.105,.055],[0.,-.105,.055]])
    lr,rr = [hand_rotation(side,-52.) for side in SIDES]
    wait_l,wl = ready_pose("left")
    wait_r,wr = ready_pose("right")
    control = os.environ.get("HARP_CONTROL","")
    phase = os.environ.get("HARP_PHASE","")

    def both(t, points, fingers, label, rotations=(lr,rr)):
        for plan,point,finger,rot in zip((left,right),points,fingers,rotations):
            plan.append(Waypoint(t,xyz(point),finger,label,rot))

    both(1.35,[[.178,.267,.977],[.178,-.267,.977]],(opened,opened),
         "approach the harp clear of the loose shade and finial",(wl,wr))
    both(1.5,pick+[[0.,.062,.045],[0.,-.062,.045]],(opened,opened),"approach the harp from both sides")
    both(2.,pick,(opened,opened),"place open jaws on the two sliding sleeves")
    both(.8,pick,(closed,closed),"grasp the sleeves")
    both(1.95,perch,(closed,closed),"slide both sleeves partway up the wire")
    both(.8,perch+squeeze,(closed,closed),"press both sleeve bores against the harp wires")
    both(1.3,perch+squeeze+[0.,0.,.040],(closed,closed),"lift with the sleeves pressed against the wires")
    both(2.8,expanded+squeeze+[0.,0.,.046],(closed,closed),"carry the compressed harp above the lamp")
    miss = np.array([[0.,0.,0.],[.011,0.,0.]]) if control == "one_foot" else np.zeros((2,3))
    both(.85,expanded+deep_squeeze+docking+[0.,0.,.032]+miss,(closed,closed),
         "finish compressing the toes above the saddle")
    both(1.0,expanded+deep_squeeze+docking+[0.,0.,.013]+miss,(closed,closed),
         "enter the open side of the saddle channels")
    both(1.15,seat+deep_squeeze+miss+[[0.,0.,.004],[0.,0.,0.]],(closed,closed),
         "land the right toe first")
    both(.7,seat+deep_squeeze+miss+[[0.,0.,0.],[0.,-.005,0.]],(closed,closed),
         "lower the left toe while the right settles under its shoulder")
    centered = seat+miss+[-.0017,0.,0.]
    both(.70,centered+deep_squeeze,(closed,closed),"center the compressed toes between the cheeks")
    both(1.0,centered,(closed,closed),"expand the feet")
    if control == "sleeves_up":
        placement = centered
        both(2.9,placement,(closed,closed),"hold the sleeves raised")
    else:
        placement = work+miss+[[0.,.0018,0.],[-.001,-.0022,0.]]
        aligned = placement+[0.,0.,.022]
        both(1.1,aligned,(closed,closed),"align both sleeve mouths over the saddle")
        both(.6,aligned,(closed,closed),"hold the aligned sleeves above the wings")
        both(1.2,placement,(closed,closed),"lower both sleeves over the saddle")
    if phase == "retention" and not control:
        both(.90,placement,(opened,opened),"let both installed sleeves settle")
        placement = LAMP+[[.0001,.0417,.0255],[-.0001,-.0418,.0255]]
        both(.65,placement,(opened,opened),"line up with the settled sleeve cups")
        both(.60,placement,(closed,closed),"regrasp both settled sleeves")
    if phase == "retention" or control:
        inward = placement+[[0.,-.004,0.],[0.,.004,0.]]
        both(.65,inward,(closed,closed),"apply the same small inward handling displacement")
        both(.65,inward+[0.,0.,.004],(closed,closed),"lift the sleeve grips by four millimeters")
        both(.65,placement,(closed,closed),"relax the handling check")
    both(.8,placement,(opened,opened),"release the harp")
    park = work+outside
    park[0,0] -= .063
    first_clear = work+[[-.053,.031,.041],[0.,-.078,.045]]
    both(.95,first_clear,(opened,opened),"raise the open fingertips away from the saddle")
    both(1.05,park,(opened,opened),"clear the lamp and loose shade")
    if phase in ("harp","retention") or control:
        both(3.,park,(opened,opened),"observe the released harp")
        return {"left":left,"right":right}

    if spindle is None:
        spindle = LAMP+[-.0105,-.0001,geo.HARP_HEIGHT-.0002]
    if shade_start is None:
        shade_start = SHADE

    if spindle_axis is None:
        spindle_axis = np.array([-.05598,.00005,.99843])
    vertical = np.array([0.,0.,1.])
    seating = axis_rotation(vertical,spindle_axis)
    if shade_grasp is None:
        shade_grip = np.array([0.,-.032,geo.SHADE_HEIGHT+.006])
        shade_grip_rotation = hand_rotation("left",-53.).rotation
    else:
        shade_grip,shade_grip_rotation = shade_grasp

    def carry_shade(t,position,rotation,finger,label):
        point = position+rotation @ shade_grip
        r = as_waypoint_rotation(rotation @ shade_grip_rotation)
        both(t,[point,wait_r],(finger,opened),label,(r,wr))

    upright = np.eye(3)
    tilt_small = axis_angle_matrix(np.array([1.,0.,0.]),math.radians(-8.))
    tilt_carry = axis_angle_matrix(np.array([1.,0.,0.]),math.radians(-22.))
    carry_shade(1.8,shade_start+[0.,0.,.039],upright,opened,"approach the shade spider from above")
    carry_shade(1.1,shade_start,upright,opened,"straddle an exposed spider spoke")
    carry_shade(.75,shade_start,upright,closed,"grasp the existing shade spoke")
    carry_shade(3.,[.180,.158,.904],upright,closed,"lift the shade beside the lamp")
    carry_shade(.85,[.160,.100,.986],tilt_small,closed,"keep the rim outside the harp")
    carry_shade(.80,[.160,.029,1.062],tilt_carry,closed,"raise the forward rim clear of the harp")
    carry_shade(1.75,[*spindle[:2],1.064],tilt_carry,closed,"center the tilted shade over the post")
    shade_seat = spindle-spindle_axis*(geo.SHADE_HEIGHT-geo.SHADE_WASHER)
    carry_shade(1.,shade_seat+spindle_axis*.050,seating,closed,"level the shade above the post")
    carry_shade(1.05,shade_seat+spindle_axis*.016,seating,closed,"approach the top of the post")
    carry_shade(.20,shade_seat+spindle_axis*.016,seating,closed,"steady the shade over the post")
    carry_shade(1.2,shade_seat,seating,closed,"seat the shade washer")
    carry_shade(.65,shade_seat,seating,opened,"release the shade")
    shade_retreat = shade_seat+seating @ shade_grip+[0.,.025,.045]
    both(1.8,[shade_retreat,wait_r],(opened,opened),"lift clear of the whole shade",
         (hand_rotation("left",-66.),wr))
    both(2.5,[wait_l,wait_r],(opened,opened),"observe the supported shade",(wl,wr))
    return {"left":left,"right":right}


def calibrate_expert(stage):
    from yam_arena.scene import object_states

    states = object_states(env(),("harp","shade"))
    harp_state, shade_state = states["harp"], states["shade"]
    spindle_axis = harp_state.rotation()[:,2]
    spindle = harp_state.pos+spindle_axis*geo.HARP_HEIGHT
    if stage == "seat":
        hand_pos,hand_rot,_ = next(hand_poses())
        body_inv = shade_state.rotation().T
        grasp = (body_inv @ (hand_pos-shade_state.pos),body_inv @ hand_rot)
        return demo(spindle=spindle,spindle_axis=spindle_axis,shade_grasp=grasp)
    return demo(spindle=spindle,spindle_axis=spindle_axis,shade_start=shade_state.pos)


if os.environ.get("HARP_EXPERT") == "1":
    install_expert(
        Path(__file__).resolve(),
        {"approach the shade spider from above":"shade","seat the shade washer":"seat"},
        calibrate_expert,
    )
