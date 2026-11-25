"""Wrist-camera perception for the battery-pills demo: robot-observable data only.

Inputs: a 320 x 176 RGB frame from a wrist camera, that arm's measured joint angles, the rig's published calibration
(arm bases, the D405 mount in the gripper link and its intrinsics: yam_arena.station) and the card's part drawings
(sizes and heights). No simulator pose is read.

Method: inside the dish (the blue mask's region) the non-blue blobs are the six discs; the darkest compact blob is the
steel-grey cell, the others the white tablets (relative brightness, so the absolute render calibration does not
matter). Each blob's centroid is back-projected through the calibrated camera onto the plane at its disc's
mid-height. The dish centre is the centre of the 7-slot lattice (a least-squares fit of the six discs to "on the
centre or 18 mm from it").
"""
import math

import numpy as np

import geometry as G

CAM_W, CAM_H = 320, 176


class Camera:
    """OpenGL-convention pinhole (looks along -z, +y up) with intrinsics at the observation size."""

    def __init__(self, pos, rot, fx, fy, cx, cy, width=CAM_W, height=CAM_H):
        self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)
        self.fx, self.fy, self.cx, self.cy, self.width, self.height = fx, fy, cx, cy, width, height

    def rays(self, us, vs):
        d = np.stack([(np.asarray(us, float) - self.cx) / self.fx, -(np.asarray(vs, float) - self.cy) / self.fy,
                      -np.ones(np.shape(us))], -1)
        d = d @ self.rot.T
        return d / np.linalg.norm(d, axis=-1, keepdims=True)

    def project(self, pts):
        p = (np.asarray(pts, float) - self.pos) @ self.rot
        z = -p[..., 2]
        return np.stack([self.cx + self.fx * p[..., 0] / z, self.cy - self.fy * p[..., 1] / z], -1), z


def lens_at_observation(lens):
    sx, sy = CAM_W / lens.width, CAM_H / lens.height
    return lens.fx * sx, lens.fy * sy, (lens.cx + .5) * sx - .5, (lens.cy + .5) * sy - .5


class ArmModel:
    """One arm's kinematics and wrist camera from the rig calibration (yam_arena.station, URDF)."""

    def __init__(self, side):
        from yam_arena import YAM_URDF, station
        from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
        self.side = side
        self.urdf, self.joints, self.tcp_offset = YAM_URDF, ARM_JOINTS, np.asarray(TCP_OFFSET)
        self.base = np.asarray(station.BASES[side], float)
        cam = station.wrist_camera(side)
        self.cam_pos_g = np.asarray(cam.pos, float)
        self.cam_rot_g = G.quat_xyzw_to_matrix(cam.rot_xyzw)
        if cam.convention != 'opengl':
            raise ValueError('expected the station wrist camera in the OpenGL convention')
        self.intrinsics = lens_at_observation(cam.lens)

    def gripper(self, q):
        from yam_arena.urdf import forward_kinematics
        t = forward_kinematics(self.urdf, {n: float(v) for n, v in zip(self.joints, q)})['gripper'].copy()
        t[:3, 3] += self.base
        return t

    def tcp(self, q):
        t = self.gripper(q)
        return t[:3, :3] @ self.tcp_offset + t[:3, 3], t[:3, :3]

    def camera(self, q):
        t = self.gripper(q)
        return Camera(t[:3, :3] @ self.cam_pos_g + t[:3, 3], t[:3, :3] @ self.cam_rot_g, *self.intrinsics)


# -- masks ----------------------------------------------------------------------------------------------------
def hsv(img):
    x = np.asarray(img, float)[..., :3] / 255.0
    mx, mn = x.max(-1), x.min(-1)
    sat = np.where(mx > 1e-6, (mx - mn) / np.maximum(mx, 1e-6), 0)
    return x, mx, sat


def box(mask, k=3):
    m = np.asarray(mask, float)
    h = k // 2
    p = np.pad(m, h, mode='edge')
    return sum(p[i:i + m.shape[0], j:j + m.shape[1]] for i in range(k) for j in range(k)) / (k * k)


def blue_mask(img):
    x, v, s = hsv(img)
    return (x[..., 2] > x[..., 0] + 0.08) & (x[..., 2] > x[..., 1] + 0.03) & (s > 0.30) & (v > 0.12)


def components(mask):
    """Connected components (4-neighbour): label image and sizes, no SciPy."""
    h, w = mask.shape
    labels = np.zeros((h, w), int)
    sizes = [0]
    cur = 0
    for y0 in range(h):
        for x0 in range(w):
            if mask[y0, x0] and not labels[y0, x0]:
                cur += 1
                stack = [(y0, x0)]
                labels[y0, x0] = cur
                n = 0
                while stack:
                    y, x = stack.pop()
                    n += 1
                    for yy, xx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                        if 0 <= yy < h and 0 <= xx < w and mask[yy, xx] and not labels[yy, xx]:
                            labels[yy, xx] = cur
                            stack.append((yy, xx))
                sizes.append(n)
    return labels, np.asarray(sizes)


def fill_holes(mask):
    """Pixels enclosed by ``mask`` (not reachable from the border through ~mask)."""
    outside = ~mask
    lab, _ = components(outside)
    border = set(np.unique(np.r_[lab[0], lab[-1], lab[:, 0], lab[:, -1]]).tolist()) - {0}
    enclosed = outside & ~np.isin(lab, list(border))
    return mask | enclosed


def back_project(cam, u, v, z_world):
    ray = cam.rays(np.array([u]), np.array([v]))[0]
    t = (z_world - cam.pos[2]) / ray[2]
    return cam.pos + t * ray


def px_per_mm(cam, z_world):
    return cam.fx * 1e-3 / max(1e-6, cam.pos[2] - z_world)


def find_discs(img, cam, dish_bottom_z):
    """The six discs in the dish: dict(ok, cell (x, y) m, tablets [(x, y)], blobs [...], reason)."""
    blue = blue_mask(img)
    blue = box(blue, 3) > 0.5
    lab, sizes = components(blue)
    if len(sizes) < 2:
        return dict(ok=False, reason='no blue')
    dish_lab = int(np.argmax(sizes[1:]) + 1)
    region = fill_holes(lab == dish_lab)
    inner = region & ~(lab == dish_lab)
    # grow the enclosed region by 1 px (anti-aliased disc edges are part blue)
    inner = (box(inner, 3) > 0.3) & ~(box(lab == dish_lab, 3) > 0.95)
    dl, ds = components(inner)
    ppm = px_per_mm(cam, dish_bottom_z + G.DISH_FLOOR_TOP * G.MM)
    min_px = 0.35 * math.pi * (G.CELL_R * ppm) ** 2
    max_px = 2.2 * math.pi * (G.TAB_R * ppm) ** 2
    x, v, s = hsv(img)
    blobs = []
    for k in range(1, len(ds)):
        if not (min_px <= ds[k] <= max_px):
            continue
        vv, uu = np.nonzero(dl == k)
        if s[vv, uu].mean() > 0.60:        # dark, saturated blue shading of the dish wall (job 4296409), not a disc
            continue
        blobs.append(dict(n=int(ds[k]), u=float(uu.mean()), v=float(vv.mean()), value=float(v[vv, uu].mean()),
                          sat=float(s[vv, uu].mean()), spread=float(np.sqrt(uu.var() + vv.var()))))
    if len(blobs) > 6:                      # keep the six largest
        blobs = sorted(blobs, key=lambda b: -b['n'])[:6]
    if len(blobs) < 1:
        return dict(ok=False, reason='no discs', px_per_mm=ppm, n_regions=int(len(ds) - 1))
    order = sorted(range(len(blobs)), key=lambda i: blobs[i]['value'])
    ci = order[0]
    floor = dish_bottom_z + G.DISH_FLOOR_TOP * G.MM
    out_t = []
    for i, b in enumerate(blobs):
        h = G.CELL_H if i == ci else G.TAB_H
        p = back_project(cam, b['u'], b['v'], floor + h / 2 * G.MM)
        b['xy'] = p[:2]
        if i != ci:
            out_t.append(p[:2])
    cell = blobs[ci]['xy']
    contrast = (blobs[order[1]]['value'] - blobs[ci]['value']) if len(blobs) > 1 else 0.0
    # job 4296409: cell-tablet contrast 0.16-0.20 at the start, 0.025-0.05 after a push (the pinch reading rejects a tablet)
    return dict(ok=len(blobs) == 6 and contrast > 0.02, cell=cell, tablets=out_t, blobs=blobs, px_per_mm=ppm,
                contrast=contrast, reason=None if len(blobs) == 6 else f'{len(blobs)} discs')


def lattice_centre(points, guess):
    """Dish centre: the point c minimising sum min(|p - c|, ||p - c| - 18 mm|)^2 over the discs (pattern search)."""
    pts = np.asarray(points, float)
    R = G.SLOT_R * G.MM

    def cost(c):
        d = np.linalg.norm(pts - c, axis=1)
        return float(np.sum(np.minimum(d, np.abs(d - R)) ** 2))

    best_c, best = np.asarray(guess, float), cost(np.asarray(guess, float))
    for p in list(pts) + [np.mean(pts, 0)]:          # start from each disc and from the mean
        c = np.asarray(p, float)
        val = cost(c)
        step = 2e-3
        while step > 2e-5:
            moved = False
            for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                cand = c + step * np.asarray(d)
                cv = cost(cand)
                if cv < val - 1e-15:
                    c, val, moved = cand, cv, True
            if not moved:
                step /= 2
        if val < best - 1e-15:
            best_c, best = c, val
    return best_c, math.sqrt(best / max(1, len(pts))) / G.MM


# -- closing-line choice ---------------------------------------------------------------------------------------
# Finger tip region at tablet height (URDF collision hulls of tip_left / tip_right, gripper frame): the last ~3 mm of
# each finger is ~4.2 mm thick along the closing axis and +-8.6 mm across it (it widens to 6.4 x +-13.2 mm 16 mm up).
TIP_THICK = 4.3e-3
TIP_HALF_W = 8.6e-3


def pad_clearance(cell_xy, tablets, angle, half_open):
    """Smallest distance (m) from a tablet's edge to either finger tip's footprint for a closing line at ``angle``."""
    u = np.array([math.cos(angle), math.sin(angle)])
    w = np.array([-u[1], u[0]])
    best = 1.0
    for t in tablets:
        d = np.asarray(t) - np.asarray(cell_xy)
        a, b = abs(d @ u), abs(d @ w)
        dx = max(0.0, half_open - a, a - (half_open + TIP_THICK))
        dy = max(0.0, b - TIP_HALF_W)
        best = min(best, math.hypot(dx, dy) - G.TAB_R * G.MM)
    return best


def best_closing_angle(cell_xy, tablets, half_open, dish_xy=None):
    """Closing-line angle (rad, 0..pi) with the most finger clearance to the tablets; ties broken toward the line
    that also keeps the fingers clear of the dish wall."""
    best = None
    for k in range(0, 180, 3):
        a = math.radians(k)
        c = pad_clearance(cell_xy, tablets, a, half_open)
        if best is None or c > best[1] + 1e-6:
            best = (a, c)
    return best
