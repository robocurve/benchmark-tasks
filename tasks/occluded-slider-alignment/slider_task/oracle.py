"""Privileged open-loop physical feasibility candidate, not a visual policy result."""

from .layout import HOME, TARGET, TOP


def manipulation_waypoints():
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint
    import math
    x, y, z = HOME[0], -.132, TOP + .065
    # Nominal 26 mm jaw gap around the 28 mm tab: ~1 mm pad compression per
    # side, rather than a fully closed target that could reach the 50 N cap.
    grip_tab = -.013
    # Contact-only translation: close on exposed tab, slide in X, release, retreat.
    # No object state writes, lifting, joint motor or joint target.
    return {'right': [
        Waypoint(2.5, (x, y, TOP + .16), FINGER_OPEN, 'approach outside housing', math.pi / 2),
        Waypoint(3.0, (x, y, z), FINGER_OPEN, 'descend to exposed tab', math.pi / 2),
        Waypoint(1.0, (x, y, z), grip_tab, 'grasp tab', math.pi / 2),
        Waypoint(4.0, (x + TARGET, y, z), grip_tab, 'slide via gripper contact', math.pi / 2),
        Waypoint(1.0, (x + TARGET, y, z), FINGER_OPEN, 'release at target', math.pi / 2),
        Waypoint(1.0, (x + TARGET, y, TOP + .16), FINGER_OPEN, 'retreat', math.pi / 2),
    ]}


def contact_probe_waypoints():
    """Deliberately invalid simulator-only trajectory to test the collision scorer."""
    import math
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint
    return {'right': [
        Waypoint(2.5, (.25, 0, TOP+.24), FINGER_OPEN, 'probe above roof', math.pi/2),
        Waypoint(3.0, (.25, 0, TOP+.10), FINGER_OPEN, 'deliberate roof collision', math.pi/2),
    ]}


def inspection_solution(camera_position=(.26, .23, TOP + .080), aim=(.25, .028, TOP + .061)):
    """Solve the left arm TCP that places the original wrist camera at a side view."""
    import numpy as np
    from yam_arena import ik, station
    from yam_arena.constants import TCP_OFFSET, READY
    from yam_arena.urdf import quat_xyzw_to_matrix
    cam = station.rig().wrist_cams['left']
    backward = np.asarray(camera_position) - aim
    backward /= np.linalg.norm(backward)
    right = np.cross([0., 0., 1.], backward)
    right /= np.linalg.norm(right)
    up = np.cross(backward, right)
    camera_rotation = np.column_stack((right, up, backward))  # OpenGL camera
    grip_rotation = camera_rotation @ quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw)).T
    grip_position = np.asarray(camera_position) - grip_rotation @ np.asarray(cam.pos)
    tcp = grip_position + grip_rotation @ np.asarray(TCP_OFFSET)
    q, pe, re = ik.solve(tcp, ik.matrix_to_quat_xyzw(grip_rotation), q_init=READY,
                         base_pos=station.BASES['left'], iters=500)
    return dict(q=q.tolist(), position_error_m=pe, rotation_error_rad=re,
                camera_position=list(camera_position), aim=list(aim), tcp=tcp.tolist())


def install_inspection_plan(solution):
    """Add a joint-space camera motion to the stock scripted policy, process-locally."""
    import numpy as np
    from yam_arena import ik
    from yam_arena.constants import READY
    from yam_arena.scripted import YamScriptedPolicy
    if solution['position_error_m'] > .002 or solution['rotation_error_rad'] > .02:
        raise RuntimeError('Inspection pose IK is not accurate enough to execute')
    original_reset = YamScriptedPolicy.reset

    class ObservationPlan:
        duration = 3.0

        def action7(self, t):
            s = ik.smoothstep(t / self.duration)
            q = np.asarray(READY) * (1 - s) + np.asarray(solution['q']) * s
            return np.r_[q, 1.0], 'side inspection (privileged scripted pose)'

    def reset(self, scene):
        original_reset(self, scene)
        self._plans['left'] = ObservationPlan()
        self._end = max(self._end, ObservationPlan.duration)

    YamScriptedPolicy.reset = reset
