"""Geometry of the scene: the board, the plate, the bag's two films and its zip, and the tortillas. NumPy only.

World frame: x forward from the arms, y to the left, z up (m); the station's table top is at z = TOP. The bag lies
flat on the board with its mouth (the zip) toward the arms (-x), just in from the board's near edge. Its own frame: u
across the mouth (along world y), v from the mouth into the bag (world +x).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # the station's table top
BASES = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}
READY = (0.0, 1.0, 1.0, -0.6, 0.0, 0.0)
DT = 0.05  # 20 Hz

# -- the cutting board: end-grain maple, 12 x 14 in, 1 1/2 in thick ---------------------------------------------
BOARD_SIZE = (0.305, 0.356, 0.038)  # x, y, z
BOARD_NEAR_X = 0.36  # the board's near edge (toward the arms)
BOARD_AT = np.array([BOARD_NEAR_X + BOARD_SIZE[0] / 2, 0.0, TOP + BOARD_SIZE[2] / 2])
BOARD_TOP = TOP + BOARD_SIZE[2]

# -- the plate: a white 10.5 in (267 mm) ceramic dinner plate --------------------------------------------------------
PLATE_R = 0.1335  # rim radius
PLATE_WELL_R = 0.095  # the flat well
PLATE_H = 0.022  # rim height above the table
PLATE_WELL_Z = 0.006  # top of the well above the table
PLATE_AT = np.array([0.33, 0.320, TOP])
PLATE_MASS = 0.60

# -- the bag: an 8 in tortilla package, clear LDPE film with a press-to-close zip, lying flat ------------------------
FILM_T = 0.001  # modelled film thickness for collision (the real film is about 0.06 mm)
FILM_MASS = 0.012  # each film (both films with the zip: about 24 g)
IN_W = 0.240  # inside width across the mouth (u)
SEAL_W = 0.008  # the side seals
IN_L = 0.285  # inside length from the zip to the bottom seal (v)
BOTTOM_SEAL = 0.010
LIP_TOP = 0.022  # the front (top) film runs this far past the zip
LIP_BOTTOM = 0.034  # the back (bottom) film's lip
LIP_CURL_DEG = 55.0  # the top lip's set: it curls up from the zip, as the lips of an opened bag do
ZIP_IN = 0.020  # the zip lies this far in from the board's edge
ZIP_X = BOARD_NEAR_X + ZIP_IN
BAG_Y = 0.0  # the bag's centre line
EDGE = 0.0064  # film grid spacing (m)

# -- the tortillas: 8 in (203 mm) flour tortillas, about 2 mm thick, 50 g each --------------------------------------
N_TORTILLAS = 6
T_R = 0.1015
T_TH = 0.002
T_MASS = 0.050
T_EDGE = 0.0075
T_GAP_TO_ZIP = 0.015  # the stack's near edge lies this far inside the zip
STACK_X = ZIP_X + T_GAP_TO_ZIP + T_R  # the stack's centre
STACK_Y = BAG_Y


def tortilla_z(k: int) -> float:
    """Mid-plane height of tortilla k (0 at the bottom) at the start."""
    return BOARD_TOP + FILM_T + T_TH / 2 + k * T_TH


STACK_TOP = tortilla_z(N_TORTILLAS - 1) + T_TH / 2  # top face of the stack


def _ramp(d, width):
    """1 inside (d <= 0) falling smoothly to 0 at d = width."""
    t = np.clip(d / width, 0.0, 1.0)
    return 1.0 - (3 * t * t - 2 * t * t * t)


def _grid_axes():
    nu = int(round((IN_W + 2 * SEAL_W) / EDGE)) + 1
    us = np.linspace(-(IN_W / 2 + SEAL_W), IN_W / 2 + SEAL_W, nu)
    v_end = IN_L + BOTTOM_SEAL
    nv_body = int(round(v_end / EDGE))
    vs_body = np.linspace(0.0, v_end, nv_body + 1)
    return us, vs_body


def _faces(nu, nv):
    f = []
    for j in range(nv - 1):
        for i in range(nu - 1):
            a, b, c, d = j * nu + i, j * nu + i + 1, (j + 1) * nu + i, (j + 1) * nu + i + 1
            f += [(a, b, d), (a, d, c)] if (i + j) % 2 == 0 else [(a, b, c), (b, d, c)]
    return np.array(f, int)


def _lip_rows(length: float):
    n = max(2, int(round(length / EDGE)))
    return np.linspace(-length, 0.0, n + 1)[:-1]  # v < 0, the zip row (v = 0) excluded


def film(which: str):
    """(points V x 3 world, faces, uv, info) of the top or bottom film. Rows run along v (from the lip edge, v < 0,
    through the zip at v = 0, into the bag); vertex (row j, column i) is j * nu + i. ``info`` has nu, nv, the v of each
    row and the index of the zip row."""
    us, vs_body = _grid_axes()
    lip = _lip_rows(LIP_TOP if which == "top" else LIP_BOTTOM)
    vs = np.r_[lip, vs_body]
    nu, nv = len(us), len(vs)
    U, V = np.meshgrid(us, vs)
    y = BAG_Y + U
    x = ZIP_X + V
    if which == "bottom":
        z = np.full_like(x, BOARD_TOP + FILM_T / 2)
        # the bottom lip lies past the board's edge: it hangs down over it
        over = x < BOARD_NEAR_X
        s = BOARD_NEAR_X - x[over]
        ang = np.radians(70.0)
        x[over] = BOARD_NEAR_X - s * math.cos(ang) - FILM_T / 2
        z[over] = BOARD_TOP + FILM_T / 2 - s * math.sin(ang)
    else:
        # draped over the stack: full height over the tortillas, down to the bottom film at the seals and the zip
        h_stack = STACK_TOP + FILM_T / 2
        h_low = BOARD_TOP + FILM_T * 1.5
        r = np.hypot(x - STACK_X, y - STACK_Y)
        d_disc = r - T_R
        # the distance to the nearest seal (or the zip), inside the bag
        d_side = np.minimum(IN_W / 2 - np.abs(U), V)
        d_side = np.minimum(d_side, IN_L - V)
        up = _ramp(d_disc, 0.022) * np.clip(d_side / 0.012, 0.0, 1.0)
        z = h_low + (h_stack - h_low) * up
        z = np.where(V < 0, h_low, z)
        # the top lip curls up from the zip
        lipm = V < 0
        s = -V[lipm]
        rad = LIP_TOP / math.radians(LIP_CURL_DEG)  # a circular arc of this radius through LIP_CURL_DEG
        th = s / rad
        x[lipm] = ZIP_X - rad * np.sin(th)
        z[lipm] = h_low + rad * (1 - np.cos(th))
    pts = np.c_[x.ravel(), y.ravel(), z.ravel()]
    uv = np.c_[(U.ravel() + IN_W / 2 + SEAL_W) / (IN_W + 2 * SEAL_W),
               (V.ravel() + LIP_BOTTOM) / (IN_L + BOTTOM_SEAL + LIP_BOTTOM)]
    faces = _faces(nu, nv)
    info = {"nu": nu, "nv": nv, "vs": vs, "us": us, "zip_row": len(lip)}
    return pts, faces, uv, info


def face_parts(which: str):
    """Face index lists of the film's parts, for their looks: {"label", "zip", "seal"} (by face centre)."""
    p, f, uv, info = film(which)
    nu = info["nu"]
    us, vs = info["us"], info["vs"]
    j, i = np.divmod(np.arange(len(p)), nu)
    U, V = us[i], vs[j]
    cu, cv = U[f].mean(axis=1), V[f].mean(axis=1)
    parts = {"zip": np.where((cv > -0.0055) & (cv < 0.0015))[0],
             "seal": np.where((np.abs(cu) > IN_W / 2) | (cv > IN_L))[0]}
    if which == "top":
        parts["label"] = np.where((np.abs(cu) < 0.100) & (cv > 0.150) & (cv < 0.225))[0]
    return parts


def seam_pairs():
    """(top vertex, bottom vertex) pairs welded together: the two side seals and the bottom seal."""
    _, _, _, ti = film("top")
    _, _, _, bi = film("bottom")
    nu = ti["nu"]
    pairs = []
    for jt, v in enumerate(ti["vs"]):
        if v < 0:
            continue
        jb = jt - ti["zip_row"] + bi["zip_row"]
        for i, u in enumerate(ti["us"]):
            if abs(u) > IN_W / 2 - 1e-6 or v > IN_L - 1e-6:
                pairs.append((jt * nu + i, jb * nu + i))
    return np.array(pairs, int)


def zip_pairs():
    """(top vertex, bottom vertex, u) along the zip, inside the side seals: each is one cell of the zip."""
    _, _, _, ti = film("top")
    _, _, _, bi = film("bottom")
    nu = ti["nu"]
    out = []
    for i, u in enumerate(ti["us"]):
        if abs(u) <= IN_W / 2 - 1e-6:
            out.append((ti["zip_row"] * nu + i, bi["zip_row"] * nu + i, u))
    return out


def tortilla(k: int):
    from clothkit import disc

    p, f, uv = disc(T_R, T_EDGE, centre=(STACK_X, STACK_Y, tortilla_z(k)))
    # turn each tortilla a little, so their meshes (and texture spots) do not line up
    a = 0.7 * k
    c, s = math.cos(a), math.sin(a)
    d = p[:, :2] - [STACK_X, STACK_Y]
    p[:, :2] = [STACK_X, STACK_Y] + d @ np.array([[c, s], [-s, c]])
    return p, f, uv


if __name__ == "__main__":
    for w in ("top", "bottom"):
        p, f, uv, info = film(w)
        print(w, len(p), "vertices", len(f), "triangles", "z", p[:, 2].min() - TOP, p[:, 2].max() - TOP,
              "x", p[:, 0].min(), p[:, 0].max())
    print("seams", len(seam_pairs()), "zip cells", len(zip_pairs()))
    p, f, _ = tortilla(0)
    print("tortilla", len(p), "vertices", len(f), "triangles; stack centre y", STACK_Y, "top", STACK_TOP - TOP)
    print("board near edge x", BOARD_NEAR_X, "zip x", ZIP_X, "stack", STACK_X, STACK_Y, "plate", PLATE_AT)
