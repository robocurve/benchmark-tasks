"""Target the interchangeable parallel jaws using the pinned robot kinematics."""
import numpy as np
from yam_arena import ik


def orientation(forward, closing):
    z = np.array(forward, dtype=float)
    z /= np.linalg.norm(z)
    x = np.array(closing, dtype=float)
    x -= z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def solve_pose(point, rotation, measured, base):
    # Reversing both in-plane axes interchanges identical pads without changing the
    # approach line or the jaw opening direction. Try both; one wrist roll may be out of range.
    quats = [ik.matrix_to_quat_xyzw(rotation), ik.matrix_to_quat_xyzw(rotation @ np.diag([-1.,-1.,1.]))]
    def solve(seed,quat):
        q, translation, angle = ik.solve(point, quat, seed, base, iters=170)
        return (q, translation, angle)
    solutions = [solve(measured, q) for q in quats]
    if not any(s[1]<=.003 and s[2]<=.05 for s in solutions):
        # Wrist and elbow folds can leave damped IK at a local minimum. Compare
        # fixed alternate seeds and interpolate from the actual measured joint pose.
        for shoulder, wrist in [(-.7,-1.45),(.7,1.45),(0.,0.)]:
            seed = np.clip([shoulder,1.45,1.05,-.55,0.,wrist],ik.LOWER,ik.UPPER)
            solutions.extend(solve(seed,q) for q in quats)
            if any(s[1]<=.001 and s[2]<=.02 for s in solutions):
                break
    return min(solutions, key=lambda item: item[1]+.07*item[2]+.00025*np.linalg.norm(item[0]-measured))
