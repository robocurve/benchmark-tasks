"""The drawn pillow surface follows the pads and never changes or hides the physics."""

import numpy as np

from rdb_overfull_drawer import fixed_scene as fixture
from rdb_overfull_drawer import layout as L
from rdb_overfull_drawer.skin import BELOW_PAD, NX, NY, PillowSurface

SURFACE = PillowSurface()
BASE = np.asarray(fixture.START["pillow_base"])


def start_tops():
    return np.array([[fixture.START[L.pad_name(r, c)][2] for c in range(L.COLS)] for r in range(L.ROWS)])


def upper(tops):
    return SURFACE.vertices(BASE, np.eye(3), tops)[:NX * NY].reshape(NX, NY, 3)


def height_at(sheet, x, y):
    i = np.argmin(np.abs(sheet[:, 0, 0] - x))
    j = np.argmin(np.abs(sheet[0, :, 1] - y))
    return sheet[i, j, 2]


def test_one_closed_surface_the_size_of_the_pillow_in_the_drawer():
    vertices = SURFACE.vertices(BASE, np.eye(3), start_tops())
    assert vertices.shape == (2 * NX * NY, 3) and np.isfinite(vertices).all()
    assert SURFACE.faces.min() == 0 and SURFACE.faces.max() == len(vertices) - 1
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    assert np.allclose((high - low)[:2], L.BASE_SIZE[:2], atol=1e-6)
    assert low[2] >= L.FLOOR_Z and high[2] <= start_tops().max()
    # the two sheets meet along the seam all the way round
    lower = vertices[NX * NY:].reshape(NX, NY, 3)
    sheet = upper(start_tops())
    for edge in (np.s_[0, :], np.s_[-1, :], np.s_[:, 0], np.s_[:, -1]):
        assert np.allclose(sheet[edge], lower[edge])


def test_surface_sits_just_under_each_pad_where_fingers_press_and_never_above_a_pad():
    tops = start_tops()
    sheet = upper(tops)
    for row in range(1, L.ROWS - 1):
        for y in (.0795, -.0195):                       # where the pressing hand's two fingertips come down
            x = L.drawer_x(L.OPEN_START) + L.row_centre(row)
            assert 0. <= tops[row].max() - height_at(sheet, x, y) <= BELOW_PAD + .003
    assert sheet[..., 2].max() <= tops.max() - BELOW_PAD + 1e-9


def test_pressed_row_shows_as_a_hollow_and_others_do_not_move():
    tops = start_tops()
    before = upper(tops)
    tops[2] -= .025
    after = upper(tops)
    x2, x1 = (L.drawer_x(L.OPEN_START) + L.row_centre(row) for row in (2, 1))
    assert abs((before - after)[..., 2].max() - .025) < .002
    assert abs(height_at(before, x2, .03) - height_at(after, x2, .03) - .025) < .002
    assert abs(height_at(before, x1, .03) - height_at(after, x1, .03)) < 1e-9
    assert height_at(after, x2, .03) < L.RAIL_Z                 # drawn below the opening when it is below it


def test_part_above_the_opening_is_drawn_above_it():
    sheet = upper(start_tops())
    x3 = L.drawer_x(L.OPEN_START) + L.row_centre(3)
    assert height_at(sheet, x3, 0.) > L.RAIL_Z + .015


def test_scene_file_has_no_drawn_only_prim_and_skin_adds_no_collider():
    text = fixture.USD.read_text()
    assert "PillowFabric" not in text                    # the surface is added at run time, for display
    source = (fixture.USD.parent / "skin.py").read_text()
    assert "Collision" not in source and "RigidBody" not in source
