"""Keep the practice ball returning from the tabletop rebound board for three complete rallies.
Use the held paddle. Each rally must hit the board, bounce once on the supplied table, then meet the paddle again.
"""
from yam_arena.task_file import Box,Sphere
from rally_task.config import TABLE
SCENE='station'
OBJECTS=[Box('board',(.75,.10,TABLE+.15),size=(.017,.60,.30),color=(.12,.25,.70)),Box('board_left_foot',(.80,-.14,TABLE+.015),size=(.16,.04,.03),color=(.25,.25,.25)),Box('board_right_foot',(.80,.34,TABLE+.015),size=(.16,.04,.03),color=(.25,.25,.25)),Box('paddle',(.32,.12,.91),size=(.026,.16,.26),mass=.17),Sphere('ball',(.68,.25,1.0),radius=.02,mass=.0027,color=(1.,.65,.02))]
