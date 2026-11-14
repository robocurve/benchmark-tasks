"""Demonstrate four contact poses with ordinary joint targets.

The demo uses only the robot's own joint angles: the tool pose comes from forward kinematics of the measured
left-arm joints (the tool is bolted at the stock tool point), and the reference position is the one in the setup.
It reads no simulator state and no contact sensor. It presses by a fixed small depth past first touch.
"""

import math

import numpy as np
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena import ik
from yam_arena.demo import grip_rotation
from yam_arena.scripted import YamScriptedPolicy

from .motion import Arm, Move
from .scoring import REFERENCE, TIP_LOCAL

PRESS_M = .003  # after alignment, the tracking correction may push the tip at most 3 mm deeper
DESCEND_MPS = .0015  # touch-phase descent rate of the commanded tip
FELT_S = 2.5  # end a hold after this long with the tip (from joint angles) within 0.04 mm of the contact's known height


def rotation(azimuth):
    az, tilt = math.radians(azimuth), math.radians(25.)
    axis = np.array([-math.sin(az),math.cos(az),0.])
    cross = np.array([[0.,-axis[2],axis[1]],[axis[2],0.,-axis[0]],[-axis[1],axis[0],0.]])
    return (np.eye(3)+math.sin(tilt)*cross+(1-math.cos(tilt))*cross@cross) @ grip_rotation(0.)


class CalibrationPolicy(YamScriptedPolicy):
    def __init__(self, mode='demo', **kwargs):
        super().__init__(**kwargs)
        if mode not in ('demo','hover','onepose'):
            raise ValueError('Use demo, hover, or onepose mode.')
        self.mode = mode

    def reset(self, scene):
        self._built = False

    def target(self, rot, height):
        return lambda sim: (REFERENCE+np.array([0.,0.,height])-rot@TIP_LOCAL,rot)

    def touch(self, sim, rot):
        pos, measured_rot = self.sensed
        tip = pos+measured_rot@TIP_LOCAL
        if rot is not self.touch_rotation:
            self.touch_rotation, self.contact_phase = rot, 'align'
            self.bias[2] = 0.
            self.touch_height, self.aligned_s = .003, 0.
            self.press_floor = None
            self.felt_s, self.touch_done = 0., False
        lateral = np.linalg.norm(tip[:2]-REFERENCE[:2])
        settled = lateral<.0004 and abs(tip[2]-REFERENCE[2]-.003)<.0008
        self.aligned_s = self.aligned_s+self._dt if settled else 0.
        if self.contact_phase=='align' and self.aligned_s>=.5:
            self.contact_phase = 'touch'
            self.press_floor = self.bias[2]-PRESS_M
        if self.contact_phase=='touch' and lateral<.0006:
            self.touch_height = max(-.0003,self.touch_height-DESCEND_MPS*self._dt)
        on_contact = (self.contact_phase=='touch' and lateral<.0006 and tip[2]-REFERENCE[2]<.00004
                      and self.touch_height<0.)
        self.felt_s = self.felt_s+self._dt if on_contact else 0.
        if self.felt_s>=FELT_S and self.mode!='hover':
            self.touch_done = True
        height = .006 if self.mode=='hover' else self.touch_height
        desired = REFERENCE+np.array([0.,0.,height])
        error = desired-tip
        change = np.clip(.8*error,-.001,.001)*self._dt
        self.bias = np.clip(self.bias+change,-.01,.01)
        if self.press_floor is not None:
            self.bias[2] = max(self.bias[2],self.press_floor)
        return desired+self.bias-rot@TIP_LOCAL,rot

    def _build(self):
        self.arm = Arm(self._bases['left'])
        self.bias = np.zeros(3)
        self.touch_rotation = None
        azimuths = (90.,) if self.mode=='onepose' else (90.,150.,210.,270.)
        first = rotation(azimuths[0])
        self.arm.push(Move(3.,self.target(first,.08),FINGER_OPEN,'approach'))
        for az in azimuths:
            rot = rotation(az)
            self.arm.push(Move(2.,self.target(rot,.02),FINGER_OPEN,'turn above reference'),
                          Move(1.5,self.target(rot,.006 if self.mode=='hover' else .003),FINGER_OPEN,'lower'),
                          Move(26.,lambda sim, r=rot:self.touch(sim,r),FINGER_OPEN,'contact hold',track=True),
                          Move(1.5,self.target(rot,.02),FINGER_OPEN,'lift'))
        self.arm.push(Move(2.,self.target(rotation(azimuths[-1]),.08),FINGER_OPEN,'withdraw'))
        self._built = True

    def act(self, observation):
        if not self._built:
            self._build()
        q_left = np.asarray(observation.state['joint_pos'],float)[:6]
        self.sensed = ik.tcp_pose(q_left,np.asarray(self._bases['left'],float))
        if getattr(self,'touch_done',False) and self.arm.label=='contact hold':
            self.arm.interrupt()
            self.touch_done = False
        left = self.arm.act(None,float(observation.state_time)+self._dt)
        meta = {'request_stop':True} if self.arm.idle else {}
        return ActionChunk(actions=[Action(data=pack(left,np.r_[READY,1.]),meta=meta)],control_hz=1./self._dt)
