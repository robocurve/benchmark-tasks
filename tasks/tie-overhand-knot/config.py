"""Physical dimensions and fixed layout. All values are metres, kg and seconds.

This is a development configuration; validation records, not this file, establish
whether the robot can complete the task or whether the rope model is adequate.
"""
import numpy as np

TABLE_TOP = .7475
RADIUS = .005
LENGTH = 1.30
SEGMENT = .0125
# 74.7 g/m: the nominal mass of Marlow 10 mm Doublebraid polyester.
# This grounds mass only; friction and bending below are uncalibrated.
MASS = .0747 * LENGTH
RUN_LIMIT = 3000
POLICY_PHYSICS_SCHEDULE = {
    'initial_physics_hz': 240,
    'target_physics_hz': 960,
    'control_hz': 20,
    'switch_after_completed_actions': 1560,
    'switch_capture_time_s': 78.0,
}
INSTRUCTION = (
    "Tie one overhand knot in the blue cord. Pull the ends apart to at least "
    "117 cm for one second with the whole cord at least 1 cm above the table, then let go "
    "and leave the knot intact for five seconds."
)


def initial_nodes():
    """Uncrossed flat U: 24 straight links, 56 semicircle links, 24 straight."""
    radius = SEGMENT / (2 * np.sin(np.pi / 112))
    a = np.column_stack((np.linspace(.06, .36, 25), np.full(25, radius)))
    theta = np.linspace(np.pi / 2, -np.pi / 2, 57)
    b = np.column_stack((.36 + radius * np.cos(theta), radius * np.sin(theta)))
    c = np.column_stack((np.linspace(.36, .06, 25), np.full(25, -radius)))
    xy = np.vstack((a, b[1:], c[1:]))
    # Keep the same uncrossed U within both arms' table-level workspace.
    xy[:, 1] -= .18
    return np.column_stack((xy, np.full(len(xy), TABLE_TOP + RADIUS + .0005)))


CONFIG = dict(radius=RADIUS, length=LENGTH, segment=SEGMENT, mass=MASS,
              table_top=TABLE_TOP, run_limit=RUN_LIMIT, instruction=INSTRUCTION,
              friction=.8, bend_stiffness=.000390625, bend_damping=.0000390625,
              linear_damping=.5, angular_damping=2.,
              contact_offset=.0005, solver_position_iterations=64,
              solver_velocity_iterations=16, anchor_start=False,
              score_version='overhand-bimanual-v4',
              policy_physics_schedule=POLICY_PHYSICS_SCHEDULE.copy(),
              proof_load_n=.981, proof_seconds=5.)
