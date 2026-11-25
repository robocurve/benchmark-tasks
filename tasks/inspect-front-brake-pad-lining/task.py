"""Check the installed outboard Camry front pad through a wheel spoke opening."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from contacts import Contacts
import geometry as geo
from partkit import Part, env, on_reset, on_step
from pose import as_waypoint_rotation, ready_pose
from scoring import Measurement
import report_board as report
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.75
INSTRUCTION = "Measure the outer brake pad through the wheel, return both gauges, and mark the measured thickness range on the inspection card."
TOP = TABLE_TOP[SCENE]
HUB = np.array([.585,.045,TOP+.332])
MOUNT_YAW = axis_angle_matrix(np.array([0.,0.,1.]),math.radians(-15.))
HUB_ROTATION = MOUNT_YAW @ np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
GAUGE_ROTATION = MOUNT_YAW @ np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
PAD_RADIAL = .106
LANE = -.069
EAR_EDGE = PAD_RADIAL+.016
REST_ROTATION = axis_angle_matrix(np.array([0.,0.,1.]),-math.pi/2)
REST_ORIENTATION = tuple(ik.matrix_to_quat_xyzw(REST_ROTATION).tolist())
RESTS = np.array([[.355,-.173,.900],[.355,-.287,.900]])
LINING = {"narrow":.0066,"wide":.0114}.get(os.environ.get("GAUGE_CONTROL_GAP"),.009)
NAMES = ("gauge_8mm","gauge_10mm")
CORNERS = np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])
STATE = {}


def hub_point(point):
    return HUB+HUB_ROTATION @ point


def xyz(point):
    return tuple(float(v) for v in point)


def rack(name, at, thickness):
    top = at[2]+geo.HANDLE_BASE
    dimensions = (
        ("floor", (0.,-.094,(TOP-top)/2+geo.HANDLE_BASE), (.032,.038,top-TOP)),
        ("rail_0", (-.005,-.022,-thickness/2+.0015), (.003,.017,.003)),
        ("rail_1", (.005,-.022,-thickness/2+.0015), (.003,.017,.003)),
        ("probe_rest", (0.,-.022,(TOP-at[2]-thickness/2)/2), (.017,.016,at[2]-thickness/2-TOP)),
    )
    return [Box(f"{name}_{label}",pos=xyz(at+REST_ROTATION @ offset),
                size=xyz((size[1],size[0],size[2])),color=geo.RACK)
            for label,offset,size in dimensions]


def mounted(name, *, at=(0.,0.,0.), boxes=(), wedges=(), cylinders=(), color=geo.STEEL,
            mass=1., orientation=HUB_ROTATION):
    return Part(name, pos=xyz(hub_point(at)), size=(.02,.02,.02), mass=mass,
                boxes=boxes, wedges=wedges, cylinders=cylinders, color=color,
                mounted=True, orientation=xyz(ik.matrix_to_quat_xyzw(orientation)))


rotor = mounted("rotor", mass=7.2,color=geo.ROTOR,
                wedges=geo.ring(.071,geo.RADIUS,.021,geo.ROTOR_HEIGHT)
                +geo.ring(.031,.076,.0,.051))
pad = mounted("pad",at=(PAD_RADIAL,0.,geo.ROTOR_HEIGHT),boxes=geo.installed_pad(LINING),mass=.46)
inner_pad = mounted("inner_pad",at=(PAD_RADIAL,0.,.021),
                    orientation=HUB_ROTATION @ np.diag([1.,-1.,-1.]),
                    boxes=geo.installed_pad(LINING),mass=.46)
caliper = mounted("caliper",boxes=geo.caliper_shell(PAD_RADIAL),mass=3.)
wheel = mounted("wheel",color=geo.SILVER,mass=9.,
                wedges=geo.ring(.187,.216,-.098,.103,40)+tuple(geo.spokes())
                +geo.ring(.032,.086,.049,.092,32),
                cylinders=tuple(((.057*math.cos(i*math.tau/5),.057*math.sin(i*math.tau/5),.093),
                                 .010,.006,"Z",geo.STEEL) for i in range(5)))
tyre = mounted("tire",color=geo.TIRE,mass=10.,wedges=
               geo.ring(.211,.281,-.108,.109,40)+geo.ring(.278,.321,-.103,.103,40)
               +geo.ring(.316,.332,-.083,.082,40))
fixture = mounted("knuckle_and_stand",color=geo.KNUCKLE,mass=35.,boxes=(
    geo.box((0.,0.,-.101),(.116,.138,.150),geo.KNUCKLE),
    geo.box((.008,-.157,-.138),(.105,.347,.092),geo.KNUCKLE),
    geo.box((.0,-.317,-.141),(.336,.028,.215),geo.KNUCKLE),
))
PAINT = ((.05,.66,.86),(.95,.29,.10))
gauges = [Part(name,pos=xyz(at),size=(.004,.117,.027),mass=.009,
               boxes=tuple((center,size,paint) for center,size,_ in geo.gauge(thickness)),
               wedges=geo.gauge_profile(thickness),marks=tuple(geo.gauge_marks(thickness)),
               color=paint,friction=.62,orientation=REST_ORIENTATION)
          for name,at,thickness,paint in zip(NAMES,RESTS,geo.THICKNESSES,PAINT)]
board = Part("inspection_card", pos=xyz((*report.BOARD_ORIGIN, TOP+report.BOARD_SIZE[2]/2)),
             size=report.BOARD_SIZE, mounted=True, mass=1.,
             boxes=(geo.box((0.,0.,0.),report.BOARD_SIZE,report.PAPER),),
             marks=tuple(report.board_marks()))
marker = Part("report_marker", pos=(*report.SLOTS["START"], TOP+report.BOARD_SIZE[2]+report.TOKEN_HEIGHT/2),
              size=(2*report.TOKEN_RADIUS,2*report.TOKEN_RADIUS,report.TOKEN_HEIGHT),
              mass=.026, friction=.65,
              cylinders=(((0.,0.,0.),report.TOKEN_RADIUS,report.TOKEN_HEIGHT,"Z",(.94,.73,.10)),))
OBJECTS = [rotor,pad,inner_pad,caliper,wheel,tyre,fixture,board,marker,*gauges,
           *(p for name,at,thickness in zip(NAMES,RESTS,geo.THICKNESSES) for p in rack(name,at,thickness))]


@on_reset
def reset(_):
    from isaaclab_physx.physics.physx_manager import PhysxManager

    STATE.update(tick=0,measurement=Measurement(),contact=Contacts(NAMES),access=[geo.SpokeAccess() for _ in NAMES],parking=0,recording=0)
    if not getattr(PhysxManager.step,"_d4_lining",False):
        native = PhysxManager.step.__func__

        def step(cls):
            native(cls)
            STATE["contact"].poll(cls.get_physics_dt())

        step._d4_lining = True
        PhysxManager.step = classmethod(step)


@on_step
def tick(_):
    STATE["tick"] += 1


def hand_clear(points):
    scene = env().unwrapped.scene
    for side in ("left","right"):
        arm = scene[f"{side}_robot"]
        ids,_ = arm.find_joints([*ARM_JOINTS,*FINGER_JOINTS],preserve_order=True)
        q = arm.data.joint_pos.torch[0,ids].cpu().numpy()
        point,rotation = ik.tcp_pose(q[:6],station.BASES[side])
        box = point+(CORNERS*[max(abs(q[6:]))+.013,.039,.033]+[0.,0.,-.016]) @ rotation.T
        if any(np.maximum(p.min(0)-box.max(0),box.min(0)-p.max(0)).max() < .010 for p in points):
            return False
    return True


def forces(tool_index, tool, rotor, contacts):
    rotor_frame = rotor.rotation()
    tool_frame = tool.rotation()
    half = geo.THICKNESSES[tool_index]/2
    face, backing, stop = 0.,0.,0.
    for reference, samples in enumerate(contacts.pairs[tool_index]):
        for force,point,normal in samples:
            if abs((point-tool.pos) @ tool_frame[:,1]) > half+.0006:
                continue
            radial,tangent,axial = (point-rotor.pos) @ rotor_frame
            if not abs(tangent-LANE) <= .007 or not PAD_RADIAL-.004 <= radial <= EAR_EDGE+.012:
                continue
            if reference == 0 and abs(axial-geo.ROTOR_HEIGHT) <= .0012 and abs(normal @ rotor_frame[:,2]) > .75:
                face += abs(float(force))
            if reference == 1 and abs(axial-geo.ROTOR_HEIGHT-LINING) <= .0012:
                if abs(normal @ rotor_frame[:,2]) > .75:
                    backing += abs(float(force))
                if abs(radial-EAR_EDGE) <= .0015 and abs(normal @ rotor_frame[:,0]) > .70:
                    stop += abs(float(force))
    return face,backing,stop


def score(objects):
    count,measurement,contact = (STATE[k] for k in ("tick","measurement","contact"))
    if count == measurement.last_tick:
        return measurement.score
    rotor_state = objects["rotor"]
    tools = [objects[name] for name in NAMES]
    frame = rotor_state.rotation()
    tips = [(tool.pos-rotor_state.pos) @ frame for tool in tools]
    aligned = [tool.rotation()[:,1] @ (-frame[:,2]) >= math.cos(math.radians(8.)) for tool in tools]
    access = [check.update(tip) for check,tip in zip(STATE["access"],tips)]
    interface = [forces(i,tool,rotor_state,contact) for i,tool in enumerate(tools)]
    radial_depth = [EAR_EDGE-tip[0]+t/2 for tip,t in zip(tips,geo.THICKNESSES)]
    in_lane = [abs(tip[1]-LANE) <= .006 for tip in tips]
    go = access[0] and aligned[0] and in_lane[0] and .0008 <= radial_depth[0] <= .004
    stopped = (access[1] and aligned[1] and in_lane[1] and abs(radial_depth[1]) <= .002
               and interface[1][0] > .035 and interface[1][2] > .035)
    intact = all(tool.pos[2] >= TOP-.022 for tool in tools)
    parked = all(np.linalg.norm((tool.pos-at)[:2]) <= .009
                 and abs(tool.pos[2]-at[2]) <= .003 and tool.rotation()[2,2] > .994
                 and np.linalg.norm(tool.lin_vel) < .003 and np.linalg.norm(tool.ang_vel) < .12
                 for tool,at in zip(tools,RESTS))
    if parked:
        areas = [tool.pos+(CORNERS*[.005,.060,.015]+[0.,-.053,.008]) @ REST_ROTATION.T for tool in tools]
        parked = hand_clear(areas+[hub_point((.124,LANE,.082))+CORNERS*[.022,.018,.020]])
    STATE["parking"] = STATE["parking"]+1 if parked else 0
    token = objects["report_marker"]
    recorded = report.record_is_set(token.pos, token.lin_vel, token.ang_vel, TOP)
    if recorded:
        recorded = hand_clear([token.pos+CORNERS*[.019,.019,.016]])
    STATE["recording"] = STATE["recording"]+1 if recorded else 0
    result = measurement.update(count,go and interface[0][0]>.035,go and interface[0][1]>.035,
                                stopped,intact,STATE["parking"] >= 20,STATE["recording"] >= 20)
    if path := os.environ.get("GAUGE_DIAGNOSTICS"):
        robot = env().unwrapped.scene["right_robot"]
        ids,_ = robot.find_joints([*ARM_JOINTS,*FINGER_JOINTS],preserve_order=True)
        joints = robot.data.joint_pos.torch[0,ids].cpu().numpy()
        tcp, wrist = ik.tcp_pose(joints[:6],station.BASES["right"])
        with open(path,"a") as file:
            file.write(json.dumps(dict(step=count,tcp=tcp.tolist(),wrist=wrist.tolist(),jaw=joints[6:].tolist(),mount_loads=contact.mount_loads,score=result,tip_mm=[(p*1000).tolist() for p in tips],
                depth_mm=[float(v*1000) for v in radial_depth],access=[bool(v) for v in access],
                aligned=[bool(v) for v in aligned],contacts_n=interface,witness=measurement.witness,
                samples=measurement.counts,parked=bool(parked),recorded=bool(recorded),marker=token.pos.tolist(),positions=[tool.pos.tolist() for tool in tools],
                tool_rotation=[tool.rotation().tolist() for tool in tools],
                raw_contacts=[[sum(abs(float(f)) for f,_,_ in pair) for pair in pairs]
                              for pairs in contact.pairs], arm_contacts=int(contact.arm_collisions)))+"\n")
    return result


def success(objects):
    return score(objects) == 1.


def demo():
    plan = []
    ready,rotation = ready_pose("right")
    handle = np.array(geo.GRASP)
    straight_grip = (axis_angle_matrix(np.array([1.,0.,0.]),math.radians(82))
                     @ grip_rotation(math.pi/2))
    pickup_wrist = as_waypoint_rotation(REST_ROTATION @ straight_grip)
    axial_wrist = as_waypoint_rotation(GAUGE_ROTATION @ straight_grip)
    clear = np.array([-.037,0.,.036])
    opened,closed = FINGER_OPEN,FINGER_CLOSED

    def put(seconds, point, grip, label, angle=pickup_wrist):
        plan.append(Waypoint(seconds,xyz(point),grip,label,angle))

    def tcp(radial, axial):
        return hub_point((radial,LANE,axial))+GAUGE_ROTATION @ handle

    for index,(rest,t) in enumerate(zip(RESTS,geo.THICKNESSES)):
        half = t/2
        pick = rest+REST_ROTATION @ handle
        park = pick
        lifted = pick + np.array([0., 0., .048])
        outside = tcp(EAR_EDGE+.007,geo.WHEEL_FACE+.017)
        height = geo.ROTOR_HEIGHT+half
        entry = tcp(EAR_EDGE+.007,height+.00055)
        depth = .0022 if index == 0 else .0028
        approach = tcp(EAR_EDGE+half-depth,height+.00055)
        floor = approach-HUB_ROTATION[:,2]*.0026
        backing = approach+HUB_ROTATION[:,2]*.0034
        put(1.4,pick+clear,-.016,"approach the gauge")
        put(1.65,pick,-.016,"lower around the stock handle")
        put(1.25,pick,closed,"grasp the gauge")
        put(1.2,lifted,closed,"lift the gauge clear of the rest")
        put(2.,outside,closed,"align the measuring head outside the spoke opening",axial_wrist)
        put(2.2,entry,closed,"feed through the spoke opening toward the rotor",axial_wrist)
        put(1.65,approach,closed,"move radially in toward the pad edge",axial_wrist)
        put(.8,floor,closed,"seat the head against the rotor",axial_wrist)
        put(.8,floor,closed,"check the rotor reference",axial_wrist)
        if index == 0:
            put(.9,backing,closed,"touch the pad backing with the fitted head",axial_wrist)
            put(.65,backing,closed,"hold the backing contact",axial_wrist)
        put(1.25,entry,closed,"withdraw from the pad edge",axial_wrist)
        put(1.65,outside,closed,"withdraw along the spoke opening",axial_wrist)
        put(2.,lifted,closed,"return above the gauge rest")
        put(1.2,park,closed,"set the gauge on its rest")
        put(.75,park,-.016,"release the gauge")
        put(1.15,park+clear,-.016,"retreat from the parked gauge")
    put(1.45,ready,opened,"clear both gauges",rotation)
    left_ready, left_rotation = ready_pose("left")
    from yam_arena.constants import PAD_TIP_PAST_TCP

    grip_height = TOP+report.BOARD_SIZE[2]+report.TOKEN_HEIGHT/2+PAD_TIP_PAST_TCP
    pickup = (*report.SLOTS["START"], grip_height)
    target = (*report.SLOTS[report.RESULT], grip_height+.001)
    hover = (*pickup[:2], grip_height+.070)
    above_target = (*target[:2], hover[2])
    write_record = [
        Waypoint(sum(w.duration for w in plan),left_ready,opened,"wait for both gauge readings",left_rotation),
        Waypoint(1.4,hover,opened,"approach the yellow inspection marker"),
        Waypoint(1.0,pickup,opened,"lower to the marker"),
        Waypoint(.8,pickup,closed,"grasp the marker"),
        Waypoint(.9,hover,closed,"lift the marker"),
        Waypoint(1.2,above_target,closed,"carry to the measured range"),
        Waypoint(1.0,target,closed,"place the result in the 8 to 10 mm box"),
        Waypoint(.9,target,opened,"release the result marker"),
        Waypoint(1.0,above_target,opened,"clear the recorded result"),
        Waypoint(1.2,left_ready,opened,"retreat",left_rotation),
    ]
    return {"right":plan,"left":write_record}
