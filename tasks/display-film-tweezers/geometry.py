"""Physical dimensions shared by the scene, cohesive model, and scorer (SI units)."""
import itertools
import numpy as np

from fixture_geometry import (TABLE_Z, CARRIER_SIZE, CARRIER_CENTER as CARRIER_POS,
                              FILM_SIZE as STRIP_SIZE, TAB_BOUNDS as TAB_SIZE,
                              TAB_ROOT as TAB_POS, FILM_Z, CREASE_Y, TAB_SEAM)
PITCH = float(STRIP_SIZE[1])  # 5 mm film strips
N = 26  # film strips after the tab: 130 mm
BONDED_FROM = 3  # film_00..film_02: the 15 mm non-adhesive starter zone next to the tab.
# The tab must rise about 11.5 mm to clear the slot, which puts its seam about 15.9 mm from the
# first adhesive edge. A 10 mm starter made the first 5 mm strip release during that vertical
# exit and lifted the carrier by up to 4.2 mm; at 15 mm the first release happens in the peel.
N_BONDED = N - BONDED_FROM  # adhesive strips: 115 mm
TAB_QUAT = np.array([0.,0.,0.,1.])
FILM_NAMES = ('pull_tab',) + tuple(f'film_{i:02d}' for i in range(N))
SIZES = dict(carrier=CARRIER_SIZE, pull_tab=TAB_SIZE,
             **{name: STRIP_SIZE for name in FILM_NAMES[1:]})


def rotation(q):
    """Quaternion xyzw to rotation; reject invalid observations instead of awarding credit."""
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def corners(state, size):
    local = np.array(list(itertools.product((-0.5, 0.5), repeat=3))) * size
    return local @ rotation(state.quat_xyzw).T + np.asarray(state.pos)


def strip_pos(i):
    return np.array([0.28, CREASE_Y - (i + 0.5) * PITCH, FILM_Z])
