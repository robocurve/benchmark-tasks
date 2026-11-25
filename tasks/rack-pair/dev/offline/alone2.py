"""One station alone (iStarUSA layout X=0.40, d=0.38): pinch the front and rear faces' middles, 3 kg per arm."""
import math, numpy as np
import plan2 as P
from reach import Layout, grip_rot
from sweep import solve_world, rx, rzm
from plan import gravity_torque
from yam_arena import station
P.E_START = -0.07
L = Layout(0.40, 0.38)
z = P.Z_RACKED + 0.002 - 0.081 + P.H / 2   # on the stand, mid-height
ff = P.front_face(L, P.E_START)
lines = []
for side, y, yaw_list in (('right', ff + 0.012, (90, 60, 75)), ('left', ff + P.D - 0.012, (-90, -60, -75))):
    best = None
    for yaw in yaw_list:
        for pitch in (0, 25, -25):
            for roll in (math.pi, 0.0):
                R = grip_rot('a', yaw) @ rx(math.radians(pitch)) @ rzm(roll)
                q, pe, re = solve_world(L, 'a', side, np.array([-0.0, y, z]), R, None)
                if pe * 1000 < 2 and math.degrees(re) < 2:
                    tau = np.abs(gravity_torque(q, np.array(station.BASES[side]), P.MASS / 2 * 9.81))
                    if best is None or tau.max() / 27 < best[0]:
                        best = (float(np.max(tau / P.PEAK)), tau, yaw, pitch)
    base = L.base_world('a', side)
    lines.append(f'a-{side} pinching the {"front" if side == "right" else "rear"} face middle at y={y:+.3f}: reach {np.hypot(*(np.array([0, y]) - base[:2])):.3f} m; '
                 + ("no IK solution" if best is None else f'{P.MASS / 2:.1f} kg: tau={np.round(best[1],1).tolist()} N m, max tau/peak={best[0]:.2f}, tau/continuous max={np.max(best[1]/P.CONT):.2f}'))
txt = '\n'.join(lines); print(txt); open('alone2.txt', 'w').write(txt + '\n')
