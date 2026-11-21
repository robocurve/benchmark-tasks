"""Builds drape_start.npz: the tea towel folded in four on the shoebox (top layer's free edge 2 cm proud of the lower
one, over the box's near side) and the gel pack lying along the upturned loaf pan, both settled. Run once offline."""
import pathlib
import numpy as np
import drape as dr

HERE = pathlib.Path(__file__).resolve().parent
FOLD_U, FOLD_X = 0.235, 0.480        # fold line 2 cm past the box's far side; top edge 3.5 cm over the near side


def towel_start(t):
    u, v = t.rest_local[:, 0], t.rest_local[:, 1]
    top = dr.SHOEBOX[1][2]
    x = np.where(u <= FOLD_U, FOLD_X - (FOLD_U - u), FOLD_X - (u - FOLD_U))
    z = np.where(u <= FOLD_U, top + dr.TOWEL_THICK, top + 3 * dr.TOWEL_THICK)
    return np.c_[x, -0.525 + v, z]


def pack_start(p):
    u, v = p.rest_local[:, 0], p.rest_local[:, 1]
    return np.c_[0.18 - dr.PACK_W / 2 + u, 0.33 - dr.PACK_L / 2 + v,
                 np.full(len(u), dr.TIN[1][2] + dr.PACK_HALF + 0.002)]


def build(steps=60):
    d = dr.Drape()
    d.place(towel_start(d.towel), pack_start(d.pack))
    far = np.zeros((2, 3)) + [0, 0, 5]; I2 = np.tile(np.eye(3), (2, 1, 1)); op = np.full(2, 0.09)
    for _ in range(steps):
        d.step(far, I2, op)
    assert not d.bad and not d.touched
    return d


if __name__ == "__main__":
    d = build()
    t, p = d.towel, d.pack
    print("towel proud edge x/z", t.x[t.tid[-1, :], 0].mean().round(3), (t.x[t.tid[-1, :], 2].mean() - dr.TOP).round(3),
          "pack end z", (p.x[p.tid[:, 0], 2].mean() - dr.TOP).round(3), (p.x[p.tid[:, -1], 2].mean() - dr.TOP).round(3),
          "speed", np.abs(t.v).max().round(4), np.abs(p.v).max().round(4))
    np.savez_compressed(HERE / "drape_start.npz", towel=t.x, pack=p.x)
