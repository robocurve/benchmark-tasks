from pathlib import Path
import numpy as np
TOP=.7475
HOOK=np.array([.39,.040,TOP+.49])
POT=np.array([.30,-.145,TOP+.001])
RIM_RADIUS=.060
BASE_RADIUS=.044
HEIGHT=.096
CONTROL_HZ=20
MAX_STEPS=1200
TASK_ID='seat-a-flowerpot-in-a-cord-hanger'
SCORER='planter_fit'
USD=Path(__file__).with_name('station_prop.usda')
INSTRUCTION='Seat the pot inside the empty four-cord hanger. Let go of it and leave it centered and upright as the hanger swings.'

# Four cord exits on the upper half of the terminal knot. The last strand
# centres are above the knot's crown when the web bears on the pot.
KNOT_Z = -.405
CORD_RADIUS = .0024
# Four equal round cords meet on a square before the gathering wrap. A
# circumferential wrap adds one cord diameter to the outer bundle radius.
EYE_RADIUS = np.sqrt(2.) * CORD_RADIUS
KNOT_RADIUS = EYE_RADIUS + 3. * CORD_RADIUS
WRAP_TURNS = 4
KNOT_HALF_HEIGHT = WRAP_TURNS * CORD_RADIUS
EYE_HEIGHT = KNOT_HALF_HEIGHT - .5 * CORD_RADIUS
REST_STATIONS = ((.009,0.),(.015,-.010),(.024,-.245),(.015,-.350),(EYE_RADIUS,KNOT_Z+EYE_HEIGHT))
