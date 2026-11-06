"""Scripted contact demo. Only robot joints are commanded; no object attachment."""
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY, FINGER_OPEN as O, FINGER_CLOSED as C
from yam_arena.demo import Waypoint, ArmPlan, grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import quat_xyzw_to_matrix
from inspect_robots_yam.packing import split


def make_plan(top):
    def w(t, x, y, z, f, label):
        return Waypoint(t, (x, y, top+z), f, label)
    left = [w(2,.14,.27,.15,O,"approach handle"),
            w(1.5,.14,.27,.046,O,"lower to handle"),
            w(1,.14,.27,.046,C,"grasp handle"),
            w(1.5,.14,.27,.127,C,"lift spatula"),
            w(2,.105,.005,.127,C,"carry in front of opening"),
            w(2,.105,.005,.012,C,"lower outside guard"),
            w(3,.30,.005,.012,C,"insert beside part"),
            w(2,.30,.05,.012,C,"move blade behind part"),
            w(1,.30,.05,.012,C,"settle blade"),
            w(4,.14,.05,.012,C,"rake part forwards"),
            w(1,.14,.05,.012,C,"hold extracted"),
            w(1.5,.14,.05,.127,C,"lift tool clear"),
            w(2,.14,.27,.127,C,"return spatula"),
            w(1.5,.14,.27,.046,C,"lower to supports"),
            w(1,.14,.27,.046,O,"release spatula"),
            w(1.5,.14,.27,.15,O,"retreat from tool")]
    right_pos = np.array((.18, -.23, top+.15))
    right = [Waypoint(sum(x.duration for x in left), tuple(right_pos), O, "wait for extraction"),
             w(2,.253,.075,.12,O,"approach recovered part"),
             w(1.5,.253,.075,.018,O,"lower to recovered part"),
             w(1,.253,.075,.018,C,"grasp recovered part"),
             w(1.5,.253,.075,.12,C,"lift recovered part"),
             w(2,.235,-.175,.12,C,"carry to tray"),
             w(1.5,.235,-.175,.023,C,"lower into tray"),
             w(1,.235,-.175,.023,O,"release part"),
             w(2,.235,-.175,.15,O,"withdraw from tray")]
    return {"left": left, "right": right}


class FeedbackPlan(ArmPlan):
    def target(self, t):
        pos, rot, finger, label = super().target(t)
        from tool_retrieval.embodiment import DEMO_STATES
        if (getattr(self, 'side', None) == 'left' and 'spatula' in DEMO_STATES
                and label not in ('approach handle','lower to handle','grasp handle',
                                  'release spatula','retreat from tool','done')):
            tool = DEMO_STATES['spatula']
            actual_tcp, actual_rot = self.actual_pose
            tool_rot = quat_xyzw_to_matrix(tool.quat_xyzw)
            target_correction = tool_rot.T @ actual_rot @ grip_rotation().T
            self.correction = ik.slerp_matrix(getattr(self,'correction',np.eye(3)), target_correction, .12)
            rot = self.correction @ grip_rotation()
            desired_center = pos + (.10, 0., -.0033)
            pos = desired_center + actual_tcp - tool.pos
            self.last_tool_command = (pos.copy(), rot.copy())
        elif getattr(self,'side',None) == 'left' and hasattr(self,'last_tool_command'):
            if label == 'release spatula':
                pos, rot = self.last_tool_command
            elif label == 'retreat from tool':
                previous, rot = self.last_tool_command
                pos = pos + previous - (.14,.27,self.top+.046)
        if "part" in DEMO_STATES:
            if label in ("approach recovered part", "lower to recovered part", "grasp recovered part"):
                self.pick = DEMO_STATES["part"].pos.copy()
                self.offset = self.pick[:2] - (.253, .075)
            if label in ("approach recovered part", "lower to recovered part", "grasp recovered part", "lift recovered part"):
                pos = pos + np.r_[getattr(self, "offset", np.zeros(2)), 0.]
        return pos, rot, finger, label


class RetrievalPolicy(YamScriptedPolicy):
    def reset(self, scene):
        plan = self.plan_for(scene.instruction, scene.target)
        self._plans = {arm: FeedbackPlan(list(waypoints), self._bases[arm]) for arm, waypoints in plan.items()}
        for side,p in self._plans.items():
            p.side = side
            p.top = self._task.table_top
            p.actual_pose = ik.tcp_pose(np.asarray(READY),self._bases[side])
        self._end = max(p.duration for p in self._plans.values())

    def act(self, observation):
        for side,q in zip(('left','right'),split(observation.state['joint_pos'])):
            self._plans[side].actual_pose = ik.tcp_pose(q[:6],self._bases[side])
        return super().act(observation)


def make_policy(**kwargs):
    return RetrievalPolicy(**kwargs)
