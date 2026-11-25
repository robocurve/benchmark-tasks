"""Restore an installed dishwasher's rack roller and demonstrate a supported crossing."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import appliance_geometry as g
import mechkit as m
from scorer import Reading, Scorer
import wheel_geometry as w
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP, READY, TCP_OFFSET
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix


SCENE = 'station'
INSTRUCTION = 'Press the loose wheel back onto the dishwasher rack, then roll the rack into the tub on both tracks and let go.'
GRIPPER['effort'] = 40.
OBJECTS = list(g.OBJECTS)
scorer = Scorer()
AXLE = np.array((g.AXLE_X, g.RAIL_Y, g.AXLE_Z))
OTHER_WHEELS = np.array(((g.AXLE_X,-g.RAIL_Y,0.),
                         (g.REAR_AXLE_X,-g.RAIL_Y,0.),
                         (g.REAR_AXLE_X,g.RAIL_Y,0.)))
FACE_LOWER = np.array((-.183,0.,.021))
RACK_CORNERS = m.box((.39,.46,.12),(0.,0.,.039)).v
Y_AXIS = np.array((0.,1.,0.))


@m.on_reset
def reset(env):
    scorer.reset()
    base = env.unwrapped
    for name in ('basket', 'replacement_roller'):
        part = base.scene[name]
        root = part.data.default_root_pose.torch.clone()
        root[:, :3] += base.scene.env_origins
        part.write_root_pose_to_sim_index(root_pose=root)
        part.write_root_velocity_to_sim_index(root_velocity=part.data.default_root_vel.torch.clone().zero_())
        start = part.data.default_joint_pos.torch.clone()
        part.write_joint_state_to_sim_index(position=start, velocity=part.data.default_joint_vel.torch.clone().zero_())
        part.set_joint_position_target(start)
    tool = base.scene['press_block']
    root = tool.data.default_root_pose.torch.clone()
    root[:, :3] += base.scene.env_origins
    tool.write_root_pose_to_sim_index(root_pose=root)
    tool.write_root_velocity_to_sim_index(root_velocity=tool.data.default_root_vel.torch.clone().zero_())
    base.sim.forward()
    cameras = [base.scene[name] for name in ('top_cam','left_cam','right_cam')]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


def track_support(positions):
    local = positions - g.FRONT
    on_rail = np.abs(np.abs(local[:, 1]) - g.RAIL_Y) <= .012
    at_track_height = np.abs(local[:, 2] - g.RAIL_Z - w.RADIUS) <= .003
    within_track = (local[:, 0] >= -.328) & (local[:, 0] <= .443)
    return bool(np.all(on_rail & at_track_height & within_track))


@m.on_step
def observe(env):
    rack = m.state(env, 'basket')
    roller = m.state(env, 'replacement_roller')
    position = np.asarray(roller.pos)
    wheel_axis = roller.rotation()[:, 1]
    relative_position = rack.to_local(position[None, :])[0] - AXLE
    axis_matches = float(wheel_axis @ rack.rotation()[:, 1]) >= math.cos(math.radians(9.))
    seated = bool(np.all(np.abs(relative_position) <= (.0025,.0025,.0025)) and axis_matches)
    tips = m.fingertips(env)
    wheel_clear = float(np.linalg.norm(tips - position, axis=1).min()) >= .036
    rim_tilt = rack.tilt_deg()
    other_centres = rack.to_world(OTHER_WHEELS)
    first_spoke = roller.rotation()[:, 0]
    spoke_angle = math.atan2(-float(first_spoke[2]), float(first_spoke[0]))
    front = rack.to_world(FACE_LOWER[None, :])[0]
    away_from_rack = np.maximum(np.abs(rack.to_local(tips)-(0.,0.,.039))-(.195,.230,.060),0.)
    clear = float(np.linalg.norm(away_from_rack, axis=1).min()) >= .020
    bounds = np.vstack((rack.to_world(RACK_CORNERS),position-w.RADIUS,position+w.RADIUS))
    valid = (bool(np.isfinite(bounds).all()) and rim_tilt <= 15.
             and bool(np.all((bounds[:,0] >= -.16) & (bounds[:,0] <= .88)))
             and float(np.abs(bounds[:,1]).max()) <= .605
             and float(bounds[:,2].min()) >= g.FRONT[2] - .004)
    reading = Reading(seated, wheel_clear, track_support(position[None, :]),
                      track_support(other_centres) and rim_tilt <= 5.,
                      float(position[0] - g.FRONT[0]), spoke_angle,
                      bool(front[0] >= g.FRONT[0] + .005 and np.linalg.norm(rack.lin_vel) <= .012),
                      bool(clear and wheel_clear), bool(valid))
    value = scorer.update(reading)
    trace = os.getenv('DISHRACK_TRACE')
    if trace:
        with open(trace, 'a', encoding='utf-8') as file:
            sample = dict(vars(reading), score=value, axle_offset_m=relative_position.tolist(),
                          rack_tilt_deg=rim_tilt, wheel_center=position.tolist(),
                          wheel_axis=wheel_axis.tolist(), axle_center=rack.to_world(AXLE[None, :])[0].tolist(),
                          left_tips_relative_m=(tips[:12]-position).tolist(),
                          tip_min_distance_m=float(np.linalg.norm(tips-position,axis=1).min()), tool_pos=m.state(env,'press_block').pos.tolist())
            file.write(json.dumps(sample,allow_nan=False)+'\n')


def score(_objects):
    return scorer.score


class WristPose(float):
    def __new__(cls, rotation):
        pose = float.__new__(cls, 0.)
        pose.rotation = rotation
        return pose


def demo():
    from yam_arena import demo as motion, ik, station

    original = motion.grip_rotation
    if not getattr(original, 'dishrack_pose', False):
        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw, WristPose) else original(yaw)
        grip_rotation.dishrack_pose = True
        motion.grip_rotation = grip_rotation
    down = WristPose(g.PICKUP_ROTATION)
    roll = WristPose(axis_angle_matrix(Y_AXIS,math.radians(-65.)) @ down.rotation)
    wheel_at_axle = g.FRONT + g.RACK_START + AXLE
    fitted = wheel_at_axle + (-.001,-.0130-g.TOOL_FACE,.000) + g.TOOL_HEAD + (0.,0.,.004)
    mouth = fitted + (0.,.051,0.)
    pickup = g.TOOL_START + g.TOOL_HEAD + (0.,0.,.004)
    hover = pickup + (0.,0.,.067)
    crossbar = g.FRONT + g.RACK_START + g.FRONT_GRIP
    driving = crossbar - .010 * roll.rotation[:,2]
    finished = driving + (.274,0.,0.)
    camera = station.wrist_camera('left')
    eye = g.FRONT + (-.125,.275,.222)
    look = g.FRONT + (0.,g.RAIL_Y+.006,g.RAIL_Z+.017) - eye
    look /= np.linalg.norm(look)
    side = np.cross(look,(0.,0.,1.))
    side /= np.linalg.norm(side)
    view = np.column_stack((side,np.cross(look,side),look))
    mount = quat_xyzw_to_matrix(np.asarray(camera.rot_xyzw)) @ np.diag((1.,-1.,-1.))
    witness_rotation = axis_angle_matrix(look,math.radians(55.)) @ view @ mount.T
    witness = eye + witness_rotation @ (np.asarray(TCP_OFFSET)-camera.pos)
    witness_pose = WristPose(witness_rotation)
    right_start, right_basis = ik.tcp_pose(READY, station.BASES['right'])
    p = motion.Waypoint
    left = [
        p(.85,witness,FINGER_OPEN,'inspect the loose wheel from beside the door',witness_pose),
        p(.25,witness,FINGER_OPEN,'keep the axle and roller in view',witness_pose),
        p(1.15,hover,FINGER_OPEN,'approach the square press block',down),
        p(.70,pickup,-.027,'straddle the block grip',down),
        p(.60,pickup,FINGER_CLOSED,'grip the press block',down),
        p(.70,hover,FINGER_CLOSED,'lift the press block',down),
        p(.75,mouth+(0.,0.,.055),FINGER_CLOSED,'carry the press block above the door',down),
        p(.65,mouth,FINGER_CLOSED,'align the flat face with the roller',down),
        p(1.00,fitted,FINGER_CLOSED,'push the roller straight onto the axle',down),
        p(.55,fitted,FINGER_CLOSED,'seat the split hub',down),
        p(.55,mouth,FINGER_CLOSED,'withdraw along the axle',down),
        p(.6,hover,FINGER_CLOSED,'return the press block above the counter',down),
        p(.55,pickup,FINGER_CLOSED,'put the press block back on the counter',down),
        p(.5,pickup,FINGER_OPEN,'release the press block',down),
        p(.45,hover,FINGER_OPEN,'clear the counter',down),
        p(1.10,witness,-.025,'view the original rail',witness_pose),
    ]
    right = [
        p(8.9,right_start,FINGER_OPEN,'wait clear while the wheel seats',WristPose(right_basis)),
        p(1.35,driving-.055*roll.rotation[:,2],-.017,'approach the front of the basket',roll),
        p(.65,driving,-.017,'straddle the front grip',roll),
        p(.45,driving,FINGER_CLOSED,'hold the front grip',roll),
        p(2.2,finished,FINGER_CLOSED,'roll the basket over the hinge and into the tub',roll),
        p(.55,finished,-.017,'let the basket rest',roll),
        p(.75,finished-.085*roll.rotation[:,2],-.017,'back away from the original rack',roll),
    ]
    return {'left': left, 'right': right}
