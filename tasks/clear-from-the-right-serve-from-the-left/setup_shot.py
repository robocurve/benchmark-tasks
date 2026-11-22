"""Writes setup.png: the top camera's view of the starting layout at 1280 x 704."""
import sys

import imageio
import numpy as np
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment

task, out = sys.argv[1], sys.argv[2]
emb = yam_arena_embodiment(task=task, control_hz=20.0, cam_width=1280, cam_height=704, object_jitter=0.0)
obs = emb.reset(Scene(id="setup", instruction=emb.task.module.INSTRUCTION))
imageio.imwrite(out, np.asarray(obs.images["top_cam"]))
emb.close()
