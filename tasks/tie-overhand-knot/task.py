"""Tie one overhand in a free cord. Use run.py to record and audit a trial."""
from config import CONFIG, initial_nodes
from rope import rope
from yam_arena.task_file import Box

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.
CORD = rope("cord", nodes=initial_nodes(), radius=CONFIG["radius"],
            mass=CONFIG["mass"], friction=CONFIG["friction"],
            bend_stiffness=CONFIG["bend_stiffness"],
            bend_damping=CONFIG["bend_damping"], color=(.1, .38, .85),
            linear_damping=CONFIG["linear_damping"], angular_damping=CONFIG["angular_damping"],
            contact_offset=CONFIG["contact_offset"],
            solver_position_iterations=CONFIG["solver_position_iterations"],
            solver_velocity_iterations=CONFIG["solver_velocity_iterations"],
            anchor_start=CONFIG.get('anchor_start', False) and not CONFIG.get('diagnostic_only', False),
            start_color=(.9, .7, .05), end_color=(.85, .12, .1))
TOP = CONFIG['table_top']
CLAMP = [Box('clamp_top', pos=(-.015, 0., TOP+.014), size=(.03,.04,.008), color=(.2,.2,.22)),
         Box('clamp_left', pos=(-.015, .017, TOP+.005), size=(.03,.006,.010), color=(.2,.2,.22)),
         Box('clamp_right', pos=(-.015, -.017, TOP+.005), size=(.03,.006,.010), color=(.2,.2,.22))]
OBJECTS = [CORD] + (CLAMP if CONFIG.get('anchor_start') and not CONFIG.get('diagnostic_only') else [])
if CONFIG.get('development_jig') == 'three-peg':
    from fixtures import three_peg_jig
    from dataclasses import replace
    # Same declared clamp, moved to the jig layout's standing end.
    first = tuple((a+b)/2 for a,b in zip(CORD.nodes[0], CORD.nodes[1]))
    shift = (first[0]+.015, first[1], 0.)
    shifted = [replace(jaw, pos=tuple(p+d for p,d in zip(jaw.pos, shift))) for jaw in CLAMP]
    OBJECTS = [CORD, *shifted, *three_peg_jig()]
_tracker = None


def score(objects):
    return 0. if _tracker is None else _tracker.value


def success(objects):
    return _tracker is not None and _tracker.value == 1.
