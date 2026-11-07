"""On the station table, a conveyor belt runs from left to right across the far side of the table, past both arms,
and off the right edge into a tote. Eight wooden parts ride on it in a line, one every 5 s. Beyond the belt a card
leans back on the table, facing the arms, with a photo of a finished model on it. Nearer the arms are two white marks
and, to their left, a grey staging mat for parking parts.

The goal is the model in the photo, built on the white marks: a tall green cylinder standing on each mark, the
yellow beam across their tops and the red cube on the middle of the beam. Only those four parts are needed. The
other four each differ from one of them in one feature (a square green post, an orange beam, a blue cube and a
shorter green cylinder) and must ride on off the end of the belt. The cube and the beam arrive before the
cylinders, so they have to be taken off the belt and parked until the cylinders stand. Holding a part back on the
running belt jams the line.

Run it (the belt needs CPU physics):  inspect-robots run --embodiment yam_arena -E task=task.py -E device=cpu \\
    -E object_jitter=0 --instruction "..." --scorer task_score ...
CONVEYOR_STUDIO=1 instead spawns the finished model on the marks and the extra parts in the tote, with the card
blank, for taking the photo (make_photo.py).
"""
import json
import os
from pathlib import Path
import sys

# yam_arena loads task files by path, without adding their directory to sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from yam_arena.task_file import Box  # noqa: E402

from scenekit import BELT, CARD, install  # noqa: E402
from layout import (BELT_CY, BELT_END, BELT_LEN, BELT_T, BELT_UP, BELT_W, BELT_X, CARD_CENTER, CARD_SIZE,  # noqa: E402
                    DARK, FLOOR_Z, GREY, LEG_W, MARK, MARK_T, MARKS, ORDER, PARTS, STAGE, STAGE_SIZE,
                    STAGE_T, STEEL, TABLE_EDGE_Y, TOP, TOTE, TOTE_BLUE, TOTE_FLOOR, TOTE_IN, TOTE_T, TOTE_WALL_H,
                    WHITE, spawn_pos, studio_pos)
from scoring import Trial  # noqa: E402

SCENE = 'station'
SUCCESS_HOLD_S = 1.0
SCORE_OVER = 'end'
STUDIO = os.environ.get('CONVEYOR_STUDIO') == '1'
if STUDIO:  # the photo is taken from in front of the model (studio.py); trials keep the station's overhead camera
    from studio import use_studio_camera
    use_studio_camera()


def _fixtures():
    out = [Box(BELT, pos=(BELT_X, BELT_CY, TOP + BELT_T / 2), size=(BELT_W, BELT_LEN, BELT_T), color=DARK)]
    # The belt's frame: two steel side strips below the running surface, and legs where it runs off the table.
    for i, s in enumerate((-1, 1)):
        out.append(Box(f'belt_frame_{i}', pos=(BELT_X + s * (BELT_W / 2 + 0.005), BELT_CY, TOP + 0.009),
                       size=(0.010, BELT_LEN, 0.018), color=STEEL))
    for i, y in enumerate((TABLE_EDGE_Y + 0.30, BELT_UP - 0.05)):
        out.append(Box(f'belt_leg_{i}', pos=(BELT_X, y, (FLOOR_Z + TOP) / 2), size=(LEG_W, LEG_W, TOP - FLOOR_Z),
                       color=STEEL))
    for i, (x, y) in enumerate(MARKS):
        out.append(Box(f'mark_{i}', pos=(x, y, TOP + MARK_T / 2), size=(MARK, MARK, MARK_T), color=WHITE))
    out.append(Box('staging_mat', pos=(*STAGE, TOP + STAGE_T / 2), size=(*STAGE_SIZE, STAGE_T), color=GREY))
    # The photo card: scenekit leans it back and maps photo.png onto its front (left blank in the studio).
    out.append(Box(CARD, pos=CARD_CENTER, size=CARD_SIZE, color=WHITE))
    # The downstream tote under the end of the belt, standing on the floor beside the table.
    (tx, ty), (ix, iy) = TOTE, TOTE_IN
    stand_h = TOTE_FLOOR - TOTE_T - FLOOR_Z
    out.append(Box('tote_stand', pos=(tx, ty, FLOOR_Z + stand_h / 2), size=(ix, iy, stand_h), color=STEEL))
    out.append(Box('tote_floor', pos=(tx, ty, TOTE_FLOOR - TOTE_T / 2), size=(ix + 2 * TOTE_T, iy + 2 * TOTE_T, TOTE_T),
                   color=TOTE_BLUE))
    wz = TOTE_FLOOR + TOTE_WALL_H / 2
    for i, s in enumerate((-1, 1)):
        out.append(Box(f'tote_wall_x{i}', pos=(tx + s * (ix + TOTE_T) / 2, ty, wz),
                       size=(TOTE_T, iy + 2 * TOTE_T, TOTE_WALL_H), color=TOTE_BLUE))
        out.append(Box(f'tote_wall_y{i}', pos=(tx, ty + s * (iy + TOTE_T) / 2, wz),
                       size=(ix, TOTE_T, TOTE_WALL_H), color=TOTE_BLUE))
    return out


def _part(name):
    """Every part is a box to the physics; scenekit draws the three in CYLINDERS round (see its _round_look)."""
    size, mass, color = PARTS[name]
    pos = studio_pos(name) if STUDIO else spawn_pos(name)
    return Box(name, pos=pos, size=size, mass=mass, color=color)


PART_OBJECTS = [_part(name) for name in ORDER]
OBJECTS = [*_fixtures(), *PART_OBJECTS]
install(Path(__file__).resolve())

trial = Trial()


def success(objects):
    """Runs first in every control step, so it also keeps the jam record."""
    trial.observe(objects)
    if TRACE:
        _trace(objects)
    return trial.success(objects)


TRACE = os.environ.get('CONVEYOR_TRACE')  # diagnostics: a JSON-lines file of part poses every 0.5 s
TRACE_EVERY = int(os.environ.get('CONVEYOR_TRACE_EVERY', '10'))  # control steps between rows


def _trace(objects):
    if (trial.steps - 1) % TRACE_EVERY:
        return
    row = {'t': round((trial.steps - 1) * 0.05, 2), **trial.report(objects),
           'parts': {n: {'pos': [round(float(v), 4) for v in objects[n].pos],
                         'quat': [round(float(v), 4) for v in objects[n].quat_xyzw],
                         'vel': [round(float(v), 4) for v in objects[n].lin_vel]} for n in ORDER}}
    with open(TRACE, 'a') as f:
        f.write(json.dumps(row) + '\n')


def score(objects):
    return trial.score(objects)


def distance(objects):
    """Metres from each of the four model parts to its place in the finished model, summed (diagnostic only)."""
    return trial.distance(objects)


from demo_plan import demo as _demo  # noqa: E402


def demo():
    """The scripted plan for yam_scripted; in the studio the arms stay at their ready pose."""
    return {} if STUDIO else _demo()


assert BELT_END < -TABLE_EDGE_Y < BELT_UP
