"""flushkit: the saline in the IV flush, for scrub-the-hub (works with yamkit).

The simulator has no fluids, so the saline is a volume model driven by the plunger, plus a flow resistance applied
to the plunger's slide:

- Volumes (mL). The primed syringe starts with 10 mL in its barrel; the extension line, the training arm's vein and
  its outflow tube start full (primed), and the graduated cylinder starts empty. Plunger travel pushes saline out of
  the barrel at 10 mL per 6 cm. While the luer tip is seated on the connector (the split septum open), what leaves the
  barrel enters the primed line, so the same volume leaves the outflow tube into the cylinder. While it is not seated,
  it runs out at the tip and is spilled. Drawing the plunger back while seated pulls the line back (the cylinder's
  count drops by as much; the sim counts the net volume pushed through); drawing it back unseated draws in air. The
  task's scorer does this bookkeeping each step (`account`), so offline tests and the sim share it.
- Resistance. The plunger's slide gets a mechkit-style law (`Flow`): an implicit damper equal to the hydraulic
  resistance of the path the saline takes, seen at the plunger (F = A x dP, dP = R x Q, Q = A x v, so c = A^2 R), and
  the rubber stopper's glide force against the plunger's motion. Seated, R is the connector, the extension line and
  the catheter in series, each a Hagen-Poiseuille tube (R = 8 mu L / (pi r^4)); unseated, only the luer tip's bore.
  Both are applied by PhysX's own drive as a damper (implicit, so stable on the 8 g plunger at the 20 Hz control
  rate): the glide as a regularized Coulomb friction, a damper of GLIDE / speed, the speed being the plunger's travel
  per control step averaged over about 0.5 s (at least GLIDE_V), so the plunger meets about GLIDE at a steady push
  and holds still when nothing pushes it. An explicit glide force held for a 50 ms control step throws the light
  plunger from stop to stop, and the joint's speed read at the end of a step jitters well above its mean travel.
- What the cameras see: the barrel's saline is read as on a real syringe, by the dark stopper against the mL marks.
  The cylinder's saline is a blue column on a slide inside it that follows the collected volume (`level`). Spilled
  saline is counted, not drawn. (Isaac Lab visualization markers, as pourkit draws liquids, did not show in this
  task's camera frames, so the level is real geometry.)
"""

import math

import yamkit as kit

# -- the hydraulics ---------------------------------------------------------------------------------------------------
MU = 1.0e-3  # Pa s: 0.9 % saline at room temperature (water 1.00 mPa s at 20 C; 0.9 % NaCl about 2 % more)
BORE = 0.0145  # m: a 10 mL syringe's barrel inside diameter
AREA = math.pi * BORE ** 2 / 4  # m^2: the stopper's face, 1.65 cm^2
ML_PER_M = 10.0 / 0.060  # mL per m of plunger travel: 10 mL over the 6 cm stroke (1.65 cm^2 x 6 cm = 9.9 mL)


def tube_resistance(length, inner_d):
    """Hagen-Poiseuille resistance of a round tube, Pa s / m^3."""
    r = inner_d / 2
    return 8.0 * MU * length / (math.pi * r ** 4)


# The path from the luer tip to the vein (lengths and inside diameters, m); see the README for the sources.
PATH = {
    "connector": (0.026, 0.0011),  # split-septum valve's fluid path, about 2.6 cm of about 1.1 mm bore (estimate)
    "extension line": (0.18, 0.0010),  # small-bore extension set
    "catheter": (0.025, 0.000648),  # 22 G x 25 mm peripheral catheter, 0.648 mm inside
}
TIP = (0.010, 0.0018)  # the luer tip's own bore: what an unseated push runs out through
R_LINE = sum(tube_resistance(*p) for p in PATH.values())
R_TIP = tube_resistance(*TIP)
GLIDE = 2.5  # N: the stopper's glide (sustaining) force in a 10 mL syringe; see the README
GLIDE_V = 0.002  # m/s (0.33 mL/s): below this plunger speed the glide is a damper of GLIDE / GLIDE_V (1,250 N s/m)
SPEED_AVG = 0.1  # the glide's speed is averaged with this weight per 20 Hz control step (about 0.5 s)
CONTROL_HZ = 20.0  # the law runs once per control step


def pressure_force(q_ml_s, seated=True):
    """The force on the plunger from the flow alone at q mL/s, N."""
    return AREA * (R_LINE if seated else R_TIP) * q_ml_s * 1e-6


def plunger_force(q_ml_s, seated=True):
    """Total force to push the plunger at a steady q mL/s, N: the flow's pressure on the stopper plus its glide."""
    return pressure_force(q_ml_s, seated) + (GLIDE if q_ml_s > 0 else 0.0)


# -- what the scorer and the law share --------------------------------------------------------------------------------
STATE = {"episode": None, "seated": False, "barrel": 10.0, "cylinder": 0.0, "spilled": 0.0}


def fresh():
    """A trial's starting volumes."""
    return dict(barrel=10.0, cylinder=0.0, spilled=0.0, gone=0.0)


def account(vol, delta_m, seated):
    """Move saline for `delta_m` of plunger travel this step (positive pushes in). `vol` holds the trial's barrel,
    cylinder, spilled and gone (mL, gone = what has left the barrel, net)."""
    dv = delta_m * ML_PER_M
    if dv > 0:
        dv = min(dv, vol["barrel"])
        vol["barrel"] -= dv
        if seated:
            vol["cylinder"] += dv
        else:
            vol["spilled"] += dv
    elif dv < 0 and seated:  # drawing back through the open valve pulls the line (and so the net count) back
        back = min(-dv, vol["cylinder"])
        vol["cylinder"] -= back
        vol["barrel"] += back
    vol["gone"] = 10.0 - vol["barrel"]
    return vol


class Flow:
    """The plunger's slide law: the flow's resistance and the stopper's glide, both as PhysX's implicit damper."""

    def drive(self, key, q, qd, J, mem):
        speed = (q - mem.get("q", q)) * CONTROL_HZ  # the last step's mean speed, from the plunger's travel
        mem["q"] = q
        v = mem.get("v", 0.0) + SPEED_AVG * (abs(speed) - mem.get("v", 0.0))
        mem["v"] = v
        c = AREA ** 2 * (R_LINE if STATE["seated"] else R_TIP) + GLIDE / max(v, GLIDE_V)
        return q, 0.0, 0.0, c


def level(area_m2):
    """The saline's height in the graduated cylinder (m) for an inside cross-section `area_m2`, from this trial's
    volume (0 before the scorer has seen the trial). The task's cylinder draws it with a blue column on a slide that
    follows this height (a yamkit `follow` link, like a gauge needle)."""
    if STATE["episode"] != kit.episode():
        return 0.0
    return STATE["cylinder"] * 1e-6 / area_m2
