"""Arm joint and cartesian target controller.
Bounded additional IK seeds handle the horizontal side-grasp branch.
"""
from __future__ import annotations
import numpy as np
CONTROL_HZ = 20

class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0

    def _solve(self, pos, rot):
        from yam_arena import ik

        from scipy.optimize import least_squares
        def residual(q):
            p,R=ik.tcp_pose(q,self.base)
            return np.r_[p-np.asarray(pos), .3*ik._rot_error(rot,R)]
        fit=least_squares(residual,np.clip(self.q,ik.LOWER+1e-7,ik.UPPER-1e-7),
                          bounds=(ik.LOWER,ik.UPPER),max_nfev=80,
                          ftol=1e-8,xtol=1e-8,gtol=1e-8)
        q=fit.x
        if np.linalg.norm(residual(q))>.0001:
            for seed in ([.28,2.43,1.0,1.43,-1.28,-1.57],[-.28,2.43,1.0,1.43,1.28,1.57]):
                alt=least_squares(residual,np.clip(seed,ik.LOWER+1e-7,ik.UPPER-1e-7),bounds=(ik.LOWER,ik.UPPER),max_nfev=100,ftol=1e-8,xtol=1e-8,gtol=1e-8)
                if np.linalg.norm(residual(alt.x))<np.linalg.norm(residual(q)):q=alt.x
                if np.linalg.norm(residual(q))<.0001:break
        pe=float(np.linalg.norm(ik.tcp_pose(q,self.base)[0]-pos))
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        before = self.worst_mm
        self.worst_mm = 0.0
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield
        if self.worst_mm > 1.0:
            print(f"IK {self.name}: {self.worst_mm:.1f} mm on the move to {np.round(pos, 3).tolist()}", flush=True)
        self.worst_mm = max(before, self.worst_mm)

    def joint_move(self,pos,rot,duration,grip=None):
        """Ordinary joint interpolation for free-space transitions.
        Cartesian contact phases retain move(); no object state is accessed.
        """
        from yam_arena import ik
        q0=self.q.copy();g0=self.grip;g1=g0 if grip is None else grip
        self._solve(np.asarray(pos),rot);q1=self.q.copy()
        pe=np.linalg.norm(ik.tcp_pose(q1,self.base)[0]-pos)
        re=np.linalg.norm(ik._rot_error(rot,ik.tcp_pose(q1,self.base)[1]))
        if pe>.0001 or re>.001:raise RuntimeError(f'Unreachable {self.name} free-space endpoint: {pe}m {re}rad')
        for k in range(1,max(1,round(duration*CONTROL_HZ))+1):
            s=ik.smoothstep(k/max(1,round(duration*CONTROL_HZ)))
            self.q=q0+s*(q1-q0);self.grip=g0+s*(g1-g0)
            self.pos,self.rot=ik.tcp_pose(self.q,self.base);self.cmd=np.r_[self.q,self.grip]
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def width(w: float) -> float:
    """Gripper command for a finger opening of ``w`` m (1 = 94 mm open)."""
    return float(np.clip(w / 0.094, 0.0, 1.0))

