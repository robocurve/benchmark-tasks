"""Working envelope on its side. Dimensions in metres, positive X towards the bit."""
# Overall height in the illustrated grip direction is 180 mm. The offsets, cross-sections,
# assembled length and mass distribution are estimates, not manufacturer tolerances.
import numpy as np

SHELL_BOXES = (
    ('backstrap',(-.009,.020,0),(.034,.094,.035),(.16,.18,.19)),
    ('grip_base',(-.013,.075,0),(.044,.065,.038),(.15,.29,.33)),
    ('battery',(-.014,.102,0),(.049,.028,.043),(.16,.18,.19)),
    ('motor',(.005,-.043,0),(.125,.042,.050),(.15,.37,.41)),
    ('chuck',(.082,-.043,0),(.048,.035,.038),(.19,.22,.23)),
    ('magazine',(.151,-.043,0),(.105,.041,.034),(.19,.34,.38)),
    ('feed_guide',(.165,-.013,0),(.031,.033,.023),(.20,.21,.22)),
    ('strip_envelope',(.166,.015,0),(.032,.040,.014),(.40,.39,.34)),
    ('nose',(.213,-.043,0),(.020,.017,.017),(.28,.30,.31)),
)
TRIGGER = np.array([.018,0.,0.])
TRIGGER_SIZE = (.017,.019,.031)
TIP = np.array([.223,-.043,0.])
