"""Geometria wizualna tłoka i korbowodu wzorowana na Hondzie D16 (1,6 l) — czysty numpy, bez Isaac Sim.

Źródła wymiarów (stan 2026-11-15):
- średnica 75,0 mm, skok 90 mm, korbowód 137 mm między osiami: https://www.drifted.com/d16-engine/ ,
  https://motor-car.net/rover-engines/item/20075-honda-d16-engine
- wysokość kompresyjna ~30 mm, masa tłoka 242-251 g (oferty CP Pistons):
  https://a4h-tech.com/en/cp-pistons-pistons-11-1-compression-d16z6-engines
- korbowód ~470 g (oferta sportowego H-beam, nie dane fabryczne):
  https://www.hybrid-racing.com/products/speedfactory-h-beam-long-connecting-rods-d16-vitara-pistons
Reszta (rowki, grubości ścianek, łeb i stopa) to typowe proporcje — przybliżenie, opisane w `fidelity`.

Układ tłoka: oś z, denko na +z, środek bryły w 0 (jak w piston_scoring). Układ korbowodu: oś sworznia x,
długość wzdłuż y (stopa/mała główka na +y), środek między osiami otworów w 0. Jednostki: metry.
"""

from __future__ import annotations

import math

import numpy as np

MM = 1e-3

# --- tłok -----------------------------------------------------------------------------------------------------
BORE = 75.0 * MM
PISTON_HEIGHT = 50.0 * MM
COMPRESSION_HEIGHT = 30.0 * MM  # od osi sworznia do denka
PIN_Z = PISTON_HEIGHT / 2 - COMPRESSION_HEIGHT  # oś sworznia względem środka bryły (-5 mm)
PIN_D = 19.0 * MM
R = BORE / 2
GROOVE_R = R - 3.5 * MM
DISH_R, DISH_FLOOR_R, DISH_DEPTH = 26.0 * MM, 20.0 * MM, 3.0 * MM
CROWN_T = 9.0 * MM
WALL = 3.5 * MM
# rowki od denka w dół: (górna krawędź, dolna krawędź), mm od denka
GROOVES_MM = ((5.0, 6.2), (9.5, 11.0), (14.0, 16.5))
RING_BELT = 17.5 * MM  # pas pierścieni poza punktacją
BELT_Z = PISTON_HEIGHT / 2 - RING_BELT - 2.0 * MM  # dolna krawędź grubego pierścienia pod rowkami
BELT_INNER = GROOVE_R - 3.0 * MM  # promień wewnętrzny pod rowkami: >= 3 mm materiału za rowkiem

# --- korbowód -------------------------------------------------------------------------------------------------
CTC = 137.0 * MM
ROD_T = 20.0 * MM  # grubość (wzdłuż osi sworznia)
SMALL_IN, SMALL_OUT = PIN_D / 2, 13.0 * MM
BIG_IN, BIG_OUT = 24.0 * MM, 31.0 * MM
BEAM_W_SMALL, BEAM_W_BIG = 18.0 * MM, 28.0 * MM  # szerokość belki przy stopie i przy łbie


def rod_swing_limit_deg(half_width: float = 10.0 * MM) -> float:
    """Kąt, przy którym bok belki dotyka dolnej krawędzi płaszcza (od osi tłoka)."""
    r_in, below = R - WALL, PIN_Z + PISTON_HEIGHT / 2  # promień wewnętrzny, sworzeń nad krawędzią
    rr = math.hypot(r_in, below)
    return math.degrees(math.acos(half_width / rr) - math.atan2(below, r_in))
FLANGE_T, WEB_T = 4.0 * MM, 7.0 * MM  # dwuteownik: półki i środnik


def _top_z() -> float:
    return PISTON_HEIGHT / 2


def piston_profile() -> list[tuple[float, float]]:
    """Zamknięty profil (r, z) do obrotu wokół osi z: denko z wgłębieniem, rowki, pusty płaszcz."""
    top, bot = _top_z(), -PISTON_HEIGHT / 2
    pts = [(0.0, top - DISH_DEPTH), (DISH_FLOOR_R, top - DISH_DEPTH), (DISH_R, top), (R, top)]
    for a, b in GROOVES_MM:
        za, zb = top - a * MM, top - b * MM
        pts += [(R, za), (GROOVE_R, za), (GROOVE_R, zb), (R, zb)]
    inner = R - WALL
    # od środka: cienki płaszcz do pasa pierścieni, wyżej gruby pierścień, żeby za rowkami zostało >= 3 mm
    pts += [(R, bot), (inner, bot), (inner, BELT_Z), (BELT_INNER, BELT_Z), (BELT_INNER, top - CROWN_T), (0.0, top - CROWN_T)]
    return pts


def piston_colliders(segments: int = 16) -> list[tuple[str, tuple, tuple]]:
    """Puste zderzenia tłoka z prostych brył: denko (walec), pierścień pasa i płaszcz (prostopadłościany).
    Zwraca (rodzaj, środek, rozmiar): ("cyl", c, (r, h)) albo ("box", c, (sx, sy, sz, yaw))."""
    top, bot = _top_z(), -PISTON_HEIGHT / 2
    out = [("cyl", (0.0, 0.0, top - CROWN_T / 2), (R, CROWN_T))]
    for r_in, z0, z1 in ((BELT_INNER, BELT_Z, top - CROWN_T), (R - WALL, bot, BELT_Z)):
        rm, t = (R + r_in) / 2, R - r_in
        w = 2 * R * math.tan(math.pi / segments) + 0.0005  # zakładka, żeby nie było szpar
        for i in range(segments):
            a = 2 * math.pi * i / segments
            out.append(("box", (rm * math.cos(a), rm * math.sin(a), (z0 + z1) / 2), (t, w, z1 - z0, a)))
    for side in (-1, 1):  # piasty sworznia
        out.append(("box", (side * (R - WALL - 0.008), 0.0, PIN_Z), (0.016, 0.022, 0.026, 0.0)))
    return out


def lathe(profile: list[tuple[float, float]], segments: int = 64) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Bryła obrotowa: (punkty Nx3, liczby wierzchołków ścian, indeksy) w formacie UsdGeom.Mesh (czworokąty)."""
    prof = np.asarray(profile, dtype=float)
    n = len(prof)
    th = np.linspace(0.0, 2 * math.pi, segments, endpoint=False)
    pts = np.array([[r * math.cos(t), r * math.sin(t), z] for t in th for r, z in prof])
    counts, idx = [], []
    for i in range(segments):
        j = (i + 1) % segments
        for k in range(n):
            m = (k + 1) % n  # profil zamknięty
            a, b, c, d = i * n + k, i * n + m, j * n + m, j * n + k
            counts.append(4)
            idx += [a, b, c, d]
    return pts, np.asarray(counts, dtype=np.int32), np.asarray(idx, dtype=np.int32)


def annulus(r_in: float, r_out: float, thickness: float, segments: int = 48):
    """Pierścień (oko korbowodu) o osi z."""
    h = thickness / 2
    return lathe([(r_in, -h), (r_out, -h), (r_out, h), (r_in, h)], segments)


def rod_beam_boxes() -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """Belka dwuteowa z prostopadłościanów: (środek, rozmiar) w układzie korbowodu."""
    y0, y1 = -CTC / 2 + BIG_OUT - 3 * MM, CTC / 2 - SMALL_OUT + 2 * MM
    boxes = []
    steps = 6  # zbieżność belki przybliżona schodkami
    for s in range(steps):
        ya = y0 + (y1 - y0) * s / steps
        yb = y0 + (y1 - y0) * (s + 1) / steps
        w = BEAM_W_BIG + (BEAM_W_SMALL - BEAM_W_BIG) * (s + 0.5) / steps
        cy, ly = (ya + yb) / 2, yb - ya
        for sz in (-1, 1):  # półki wzdłuż krawędzi belki (oś z korbowodu = szerokość belki)
            boxes.append(((0.0, cy, sz * (w / 2 - FLANGE_T / 2)), (ROD_T, ly, FLANGE_T)))
        boxes.append(((0.0, cy, 0.0), (WEB_T, ly, w - 2 * FLANGE_T)))  # środnik
    return boxes


def mesh_volume(pts: np.ndarray, counts: np.ndarray, idx: np.ndarray) -> float:
    """Objętość zamkniętej siatki (wzór dywergencji na trójkątach)."""
    vol, o = 0.0, 0
    for c in counts:
        f = idx[o : o + c]
        for t in range(1, c - 1):
            a, b, d = pts[f[0]], pts[f[t]], pts[f[t + 1]]
            vol += np.dot(a, np.cross(b, d)) / 6
        o += c
    return abs(vol)


def piston_mass_estimate(density: float = 2700.0) -> float:
    """Masa z objętości siatki (bez piast i sworznia) — kontrola zgodności z 245 g."""
    return mesh_volume(*lathe(piston_profile(), 96)) * density
