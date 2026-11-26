"""Seat a collapsible utility cart floor, bridge both sidewall hinges, and press-check the locks."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
import geometry as geo
from contacts import Contacts
from model import OBJECTS
import native_joints
from partkit import env,on_reset,on_step
from pose import as_waypoint_rotation,ready_pose
from scoring import CartCheck,CartState
from yam_arena import ik,station
from yam_arena.constants import ARM_JOINTS
from yam_arena.demo import FINGER_CLOSED,FINGER_OPEN,Waypoint,grip_rotation
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .40
INSTRUCTION = "Lower the utility cart floor, slide both sidewall locks across their hinges, and press the sidewalls inward to check that they stay open."

STATE = {}
SIDES = (("left",1),("right",-1))


# The task framework imports scene.py after startup. The compound assets register
# the matching post-spawn joint authoring before its environment is initialized.
import partkit


def _cart_scene(scene):
    partkit._patch_scene(scene)
    native_joints.install()
partkit._Finder.targets["yam_arena.scene"] = _cart_scene
partkit.install()


@on_reset
def reset(_):
    from isaaclab_physx.physics.physx_manager import PhysxManager

    STATE.update(check=CartCheck(),contacts=Contacts(),tick=0)
    if not getattr(PhysxManager.step,"_cart_contact",False):
        original = PhysxManager.step.__func__
        def advance(cls):
            original(cls)
            STATE["contacts"].poll(cls.get_physics_dt())
        advance._cart_contact = True
        PhysxManager.step = classmethod(advance)


@on_step
def tick(_):
    STATE["tick"] += 1


def hand_positions():
    scene = env().unwrapped.scene
    for side,_ in SIDES:
        robot = scene[f"{side}_robot"]
        ids,_ = robot.find_joints(ARM_JOINTS,preserve_order=True)
        joints = robot.data.joint_pos.torch[0,ids].cpu().numpy()
        yield ik.tcp_pose(joints,station.BASES[side])[0]


def score(objects):
    check = STATE["check"]
    count = STATE["tick"]
    if count == check.last_tick:
        return check.value
    front,back,floor = (objects[f"cart_{name}"] for name in ("front","back","floor"))
    contact = STATE["contacts"]
    folds,travel = [],[]
    for side,_ in SIDES:
        r = objects[f"{side}_back_leaf"]
        f = objects[f"{side}_front_leaf"]
        bar = objects[f"{side}_rigidity_bar"]
        relative = r.rotation().T @ f.rotation()
        folds.append(abs(math.degrees(math.atan2(relative[1,0],relative[0,0]))))
        travel.append(float((bar.pos-r.pos) @ r.rotation()[:,0])-geo.BAR_START)
    horizontal = abs(floor.rotation()[2,0]) < math.sin(math.radians(2.))
    supported = all(abs(part.pos[2]-geo.BASE_Z) < .005 and part.rotation()[2,2] > .9994
                    for part in (front,back))
    hands = list(hand_positions())
    clear = all(hand[0] < geo.REAR_X-.035 or abs(hand[1]) > geo.HALF_W+.072
                or hand[2] > geo.RAIL_Z+.080 for hand in hands)
    # Confirm that the genuine receiving rail carries load on both sides under the inward push.
    load_transmitted = tuple(force > 3. and receiver > .4
                             for force,receiver in zip(contact.loads,contact.bar_forces))
    data = CartState(
        floor_seated=bool(horizontal and contact.floor_force > 1.0),
        both_rails_engaged=min(travel) >= .080 and max(folds) <= 2.,
        both_walls_straight=max(folds) <= 2.,
        supported=bool(supported),
        inward_load=load_transmitted,
        hands_clear=clear,
    )
    result = check.sample(count,data)
    log = os.environ.get("CART_DIAGNOSTICS")
    if log:
        with open(log,"a") as file:
            file.write(json.dumps({
                "step":count,"score":result,"floor_deg":math.degrees(math.asin(floor.rotation()[2,0])),
                "floor_support_n":contact.floor_force,"floor_end_n":contact.floor_end_force.tolist(),"fold_deg":folds,"bar_mm":[v*1000 for v in travel],
                "loads_n":contact.loads.tolist(),"palm_loads_n":contact.palm_loads.tolist(),"supported":bool(supported),"clear":data.hands_clear,
                "back":back.pos.tolist(),"front":front.pos.tolist(),"hands":[p.tolist() for p in hands],
                "floor":floor.pos.tolist(),"floor_axis":floor.rotation()[:,0].tolist(),
                "arm_arm_contacts":int(contact.arm_collisions),
                "floor_wall_n":contact.floor_pinches.tolist(),"bar_receiver_n":contact.bar_forces.tolist(),
                "floor_bar_n":contact.floor_bar_force.tolist(),
            })+"\n")
    contact.clear()
    return result


def success(objects):
    return score(objects) == 1.


def demo():
    plan = {}
    mode = os.environ.get("CART_EXPERT", "full")
    for side,sign in SIDES:
        path = []
        ready,rot = ready_pose(side)
        opened,closed = FINGER_OPEN,FINGER_CLOSED
        rim = geo.RAIL_Z
        wall = geo.HALF_W
        x_mid = geo.REAR_X+geo.LENGTH
        yaw = 0.

        def put(seconds,x,y,z,label,finger=closed,angle=None):
            path.append(Waypoint(seconds,(x,sign*y,z),finger,label,yaw if angle is None else angle))

        # Push both panels outward near their central hinges using the sides of the closed fingertips.
        put(2.0,x_mid-.031,wall-.074,rim+.024,"approach within the open cart")
        put(1.35,x_mid-.031,wall-.052,rim-.012,"lower inside the folding wall")
        put(2.25,x_mid-.031,wall-.014,rim-.012,"square the folding wall")
        put(1.5,x_mid-.031,wall-.014,rim-.012,"let the cart floor settle")
        put(.95,x_mid-.031,wall-.053,rim-.012,"back away from the wall")
        put(1.0,x_mid-.034,wall-.055,rim+.026,"clear the wall rail")
        if mode != "unlocked":
            start = geo.REAR_X+geo.BAR_START
            put(1.2,start,wall,rim+.023,"align with the stock slide",opened)
            put(1.1,start,wall,rim+.002,"lower around the stock slide",opened)
            put(.85,start,wall,rim+.002,"grasp the stock rigidity bar")
            put(2.1,start+geo.BAR_TRAVEL,wall,rim+.002,"slide the lock across the wall hinge")
            put(.85,start+geo.BAR_TRAVEL,wall,rim+.002,"let go of the rigidity bar",opened)
            put(1.05,start+geo.BAR_TRAVEL,wall,rim+.026,"raise clear of the lock",opened)
        # Opening the sides lets the unassisted stock base panel lower on its hinge.
        put(1.45,x_mid-.020,wall+.082,rim+.034,"clear the cart while the floor lowers",opened)
        put(11.2,x_mid-.020,wall+.082,rim+.034,"let the stock floor settle onto the bottom wall",opened)

        if mode == "unlock_before_proof":
            put(1.2,start+geo.BAR_TRAVEL,wall,rim+.026,"align above the installed slide",opened)
            put(1.1,start+geo.BAR_TRAVEL,wall,rim+.002,"lower around the installed slide",opened)
            put(.85,start+geo.BAR_TRAVEL,wall,rim+.002,"grasp the installed slide")
            put(2.1,start,wall,rim+.002,"return the lock to its open position")
            put(.85,start,wall,rim+.002,"let go of the opened slide",opened)
            retreat = as_waypoint_rotation(axis_angle_matrix(np.array([0.,1.,0.]),math.radians(-18.))
                                            @ grip_rotation(0.))
            put(1.2,start,wall,rim+.050,"lift fully clear of the retracted locks",opened,retreat)
            put(1.2,start,wall+.093,rim+.048,"move outside at a constant axial position",opened,retreat)

        # The fingers lead diagonally inward from outside the molded wall. The wrist stays clear of the rim.
        direction = np.array([1.,-.95*sign,-.55])
        direction /= np.linalg.norm(direction)
        grip_axis = np.cross([0.,0.,1.],direction)
        grip_axis /= np.linalg.norm(grip_axis)
        press = as_waypoint_rotation(np.column_stack([-grip_axis,-np.cross(direction,grip_axis),direction]))
        peek_x = x_mid-.012
        put(1.2,peek_x-.031,wall+.085,rim-.052,"approach the outside wall",closed,press)
        put(1.05,peek_x,wall+.058,rim-.070,"align the tips with the molded wall rib",closed,press)
        put(1.90,peek_x,wall+.005,rim-.070,"press the sidewalls inward",closed,press)
        put(1.45,peek_x,wall+.005,rim-.070,"hold inward test pressure",closed,press)
        put(1.2,peek_x,wall+.075,rim-.070,"remove test pressure",closed,press)
        put(1.05,peek_x-.035,wall+.095,rim-.026,"retreat away from the sidewall",closed,press)
        path.append(Waypoint(2.,tuple(ready),opened,"return to ready",rot))
        plan[side] = path
    return plan
