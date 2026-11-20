"""Full-scale foam roll and two-loop elastic carrier."""

from yam_arena.station import TABLE_TOP

KEY = "yoga_sling"
INSTRUCTION = "Secure the rolled yoga mat in both loops of the carrying sling. Lift it by the sling, then set it down."
DESCRIPTION = "A loosely rolled foam mat and a separate elastic-loop shoulder sling sit on the bench."
PHYSICS_HZ = 480
MAX_STEPS = 1380
MAT_X = .391
MAT_Z = TABLE_TOP + .069
MAT_LENGTH = .610
MAT_RADIUS = .051
BAND_WIDTH = .028
BAND_RADIUS = .063
BAND_UPPER_HEIGHT = .074
BAND_LOWER_HEIGHT = .061
BAND_CLOSE_TRAVEL = .012
BAND_X = .180
BAND_Z = TABLE_TOP + BAND_RADIUS + .003
WARMUP_CLEARANCE = .100
BAND_Y = .347
SEATED_Y = .240
