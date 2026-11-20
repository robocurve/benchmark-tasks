"""Judge sheet containment, physical flatness, bottoming and release."""

import itertools

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as c

HALF = np.array([c.PANEL_LENGTH / 2, c.PAPER_WIDTH / 2, c.PAPER_THICKNESS / 2])
PANEL_SAMPLES = np.array(list(itertools.product(
    (-HALF[0], 0., HALF[0]), (-HALF[1], 0., HALF[1]), (0.,))))
EDGES = np.array([[HALF[0], -HALF[1], 0.], [HALF[0], HALF[1], 0.]])


def link_frame(pose):
    pose = np.asarray(pose)
    return pose[:3], quat_xyzw_to_matrix(pose[3:])


def leading_corners(page):
    position, rotation = link_frame(page["links"]["Leading"])
    return EDGES @ rotation.T + position


class Scorer:
    def reset(self):
        self.since = None

    def evaluate(self, state):
        page = state["objects"]["page"]
        link_frames = [link_frame(page["links"][name]) for name in c.PAPER_PANELS]
        samples = np.array([PANEL_SAMPLES @ rot.T + pos for pos, rot in link_frames])
        x, y, z = ((samples - c.MOUTH) @ c.SLEEVE_ROTATION).transpose(2, 0, 1)
        upper_pos, upper_rotation = link_frame(state["objects"]["sleeve"]["links"]["UpperFilm"])
        under = (samples - upper_pos) @ upper_rotation[:, 2] <= -.00013
        within = ((x >= 0.) & (x <= c.SLEEVE_DEPTH + .001)
                  & (np.abs(y) <= c.INNER_HALF_WIDTH + .0005))
        inside = within & under & (z >= -.0007)
        tips = np.array([tip for hand in state["hands"].values() for tip in hand["points"][:2]])
        clear = all(
            np.min(np.linalg.norm(np.maximum(np.abs((tips - pos) @ rot) - HALF, 0.), axis=1)) >= .035
            for pos, rot in link_frames
        )
        bends = np.array(list(page["joints"].values()))
        flat = float(np.ptp(z)) <= .006 and np.all(np.abs(bends) <= np.deg2rad(6.))
        front = (leading_corners(page) - c.MOUTH) @ c.SLEEVE_ROTATION
        bottomed = np.all(front[:, 0] >= c.SLEEVE_DEPTH - .009)
        closed = abs(state["objects"]["sleeve"]["joints"]["FilmFlex"]) <= .018
        still = (np.linalg.norm(page["velocity"][:3]) <= .016
                 and np.linalg.norm(page["velocity"][3:]) <= .3)
        good = bool(np.all(inside) and flat and closed and still and clear and bottomed)
        t = state["time_s"]
        self.since = (t if self.since is None else self.since) if good else None
        success = self.since is not None and t - self.since >= .45
        # The leading third in the pocket is meaningful released progress.
        partial = bool(clear and still and np.all(inside[-1]))
        return {
            "score": 1. if success else .5 if partial else 0.,
            "success": success,
            "inside_samples": int(np.sum(inside)),
            "near_edge_m": float(np.min(x)),
            "front_corners_x_m": front[:, 0].tolist(),
            "front_corners_y_m": front[:, 1].tolist(),
            "front_corners_height_m": front[:, 2].tolist(),
            "lip_angle_rad": float(state["objects"]["sleeve"]["joints"]["FilmFlex"]),
            "max_page_bend_rad": float(np.max(np.abs(bends))),
            "sheet_flat": bool(flat),
            "cover_closed": closed,
            "hands_clear": bool(clear),
            "bottomed": bool(bottomed),
        }
