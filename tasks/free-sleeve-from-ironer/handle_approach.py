"""Observe left-hand acquisition at each physics step. Coordinates use the fixture frame."""
from math import cos, sin, isfinite
import numpy as np
from fixture import SHAPES
from measure import shoe_gap

# The SDK specifies an optical pose but no wrist-camera enclosure dimensions.
# Treat a 50 mm radius about the calibrated optical centre as occupied space.
CAMERA_RADIUS = .050
GRIP_CENTER, GRIP_HALF = next((np.array(p),np.array(size)/2)
                              for n,_,p,size,*_ in SHAPES if n=="release_grip")


def camera_clearance(point, angle):
    cs,sn=cos(angle),sin(angle)
    rotation=np.array(((cs,0,sn),(0,1,0),(-sn,0,cs)))
    local=rotation.T @ (point-np.array((.060,0,.088)))-GRIP_CENTER
    return float(np.linalg.norm(np.maximum(abs(local)-GRIP_HALF,0.))-CAMERA_RADIUS)


class Acquisition:
    def __init__(self):
        self.baseline=None
        self.attempt_steps=0
        self.streak=0
        self.acquired_step=0
        self.first_touch=""
        self.violation=""
        self.pre_motion=0.
        self.min_camera=float("inf")

    def observe(self, angle, camera_margin, opening, pairs, opposed):
        self.attempt_steps+=1
        gap=shoe_gap(angle)
        if not all(map(isfinite,(angle,camera_margin,opening,gap))):
            raise ValueError("Invalid left-hand acquisition telemetry")
        self.min_camera=min(self.min_camera,camera_margin)
        if camera_margin<=0 and not self.violation:
            self.violation="left camera envelope intersected release grip"
        if not self.first_touch and pairs:
            self.first_touch=", ".join(sorted(link+"/"+shape for link,shape in pairs))
        if self.baseline is None:
            self.baseline=gap
        if self.acquired_step:
            return
        self.pre_motion=max(self.pre_motion,abs(gap-self.baseline))
        bad=[link+"/"+shape for link,shape in pairs
             if link not in ("tip_left","tip_right") or shape!="release_grip"]
        if not self.violation:
            if bad:
                self.violation="unintended pre-grasp contact: "+", ".join(sorted(bad))
            elif pairs and opening>=.93:
                self.violation="contact during open-jaw approach"
            elif self.pre_motion>.00065:
                self.violation="shoe displaced before opposed grasp"
        self.streak=self.streak+1 if opposed else 0
        if self.streak>=3:
            self.acquired_step=self.attempt_steps

    def record(self):
        return dict(left_grasp_step=self.acquired_step,left_first_contact=self.first_touch,
                    left_approach_fault=self.violation,left_pregrasp_motion_m=self.pre_motion,
                    left_min_camera_clear_m=self.min_camera)
