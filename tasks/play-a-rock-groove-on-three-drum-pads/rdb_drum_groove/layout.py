"""Every dimension and rule of the drum-groove task, in metres, seconds and MIDI units, each marked by where it comes
from. Pure Python (``math`` only), so that the hit rule's and the scorer's tests need nothing else.

Tabletop frame: x forward from the line through the two arm bases, y to the robot's left, z up from the table top.

Sources (TECHNICAL.md has the links):
  [N] Alesis Nitro Max kit (retail listings): 10 in dual-zone mesh snare, 8 in single-zone mesh toms, 10 in hi-hat
      pad with a foot controller.
  [V] Vic Firth American Classic 5A, wood tip: 16 in (406 mm) long, 0.565 in (14.4 mm) across (retail listings).
  [G] General MIDI percussion notes: 42 closed hi-hat, 46 open hi-hat, 44 pedal hi-hat, 38 snare, 40 snare rim
      (electric snare), 43 floor tom; the module's pad notes are set to these and recorded with the trial.
  [S] the task as specified.
  [E] ESTIMATE: not a published figure. Measure it on the real parts (TECHNICAL.md, "Estimates").
"""

import math

INCH = .0254

# -- the pads (Alesis Nitro Max), real size ------------------------------------------------------------------------
SNARE_HEAD_R = 10 * INCH / 2      # [N] 10 in mesh head
TOM_HEAD_R = 8 * INCH / 2         # [N] 8 in mesh head
HIHAT_R = 10 * INCH / 2           # [N] 10 in hi-hat pad
RIM_W = .012                      # [E] the hoop round a mesh head: 12 mm wide ...
RIM_H = .006                      # [E] ... its top 6 mm above the head
SHELL_DEPTH = .065                # [E] the pad's shell under the head
HIHAT_T_EDGE = .012               # [E] the rubber hi-hat pad: 12 mm thick at its edge ...
HIHAT_BOW_RISE = .006             # [E] ... its bow rising 6 mm to the bell ...
HIHAT_BELL_R = .045               # [E] ... a bell 90 mm across ...
HIHAT_BELL_TOP = .015             # [E] ... standing 15 mm above the edge
HIHAT_AIM_IN = .060               # [S] the left stick plays the hi-hat's bow 60 mm in from its centre, toward the robot

# -- the layout (B-round2's "P": each pad in front of the arm that plays it), centre (x, y), height z, tilt ----------
#    tilt: degrees toward the robot (the far edge higher). Height: the snare's and tom's heads, the hi-hat's edge.
HIHAT = ((.500, .180), .120, 0.)  # [S] left of the snare and higher, level, in front of the left arm
SNARE = ((.500, -.140), .100, 5.) # [S] in front of the right arm
TOM = ((.500, -.420), .090, 5.)   # [S] the floor-tom place: right of the snare, lower
PAD_SPECS = {"hihat": HIHAT, "snare": SNARE, "tom": TOM}

# -- the sticks (Vic Firth 5A), the finger pads on their upper and lower sides --------------------------------------
STICK_LEN = 16 * INCH             # [V] 406 mm
STICK_R = .565 * INCH / 2         # [V] 7.2 mm
GRIP_FROM_BUTT = .135             # [S] tape mark a third of the way up
TIP_OUT = STICK_LEN - GRIP_FROM_BUTT          # 271 mm from the grip point to the tip's end
TIP_R = .0050                     # [E] the wood tip, about 10 mm across
NECK_R = .0050                    # [E] the taper, simulated as a thinner capsule
TAPER_FROM_GRIP = .150            # [E] where the taper starts, from the grip point
STICK_MASS = .050                 # [V]/[E] about 50 g (a pair is listed at 99 g); weigh it
BALANCE_FROM_BUTT = .175          # [E] the balance point from the butt
STICK_I_TRANSVERSE = 6.0e-4       # [E] about the centre of mass (a uniform 406 mm rod would be 6.9e-4)
STICK_I_AXIAL = 1.3e-6            # [E] m r^2 / 2
STICK_MU = .8                     # [E] hickory on the finger pads (static and dynamic)
PAD_RESTITUTION = .7              # [E] tip on mesh or rubber; to be set from a drop-rebound test
CONTACT_OFFSET = .015             # sticks' contact offset: a 3.6 m/s tip moves 15 mm in a physics step (CCD is off)

# -- the hit rule (hits.py, every physics step) -------------------------------------------------------------------
HIT_SPEED = .5                    # [S] m/s toward the pad on the step before contact
HIT_IMPULSE = 3.e-3               # [S] N s over the contact's first two physics steps
REARM_GAP = .002                  # [S] a new hit needs the stick to have left that pad by 2 mm ...
REARM_STEPS = 2                   # [S] ... for 2 physics steps
MASK_S = .040                     # [S] the same pad's note-ons less than 40 ms apart are one hit
ARM_CONTACT_N = 2.                # [S] any other part of the robot touching a pad this hard is a gripper or arm strike
REST_S = .5                       # [S] a stick in contact with one pad this long is resting on it
HELD_FORCE = 1.                   # a stick is held while both finger pads press on it at least this hard, ...
HELD_OFFSET = .10                 # ... its tape mark within this of the grip point ...
HELD_TURN_DEG = 45.               # ... and it has turned less than this in the grip
VELOCITY_FLOOR, VELOCITY_SPAN, VELOCITY_TOP_SPEED = 20., 107., 3.0   # [E] velocity = 20 + 107 ln(v/0.5) / ln 6


def midi_velocity(speed):
    """The estimated log curve from approach speed (m/s) to MIDI velocity 1..127."""
    if speed <= 0:
        return 1
    v = VELOCITY_FLOOR + VELOCITY_SPAN * math.log(speed / HIT_SPEED) / math.log(VELOCITY_TOP_SPEED / HIT_SPEED)
    return int(min(max(round(v), 1), 127))


def speed_for_velocity(velocity):
    """The inverse of ``midi_velocity`` (m/s)."""
    return HIT_SPEED * math.exp((velocity - VELOCITY_FLOOR) / VELOCITY_SPAN * math.log(VELOCITY_TOP_SPEED / HIT_SPEED))


# MIDI notes [G]: what the scorer reads them as (pedal 44 ignored; anything else is "other")
NOTE_OF_PAD = {"hihat": 42, "snare": 38, "snare_rim": 40, "tom": 43}
PAD_OF_NOTE = {42: "hihat", 46: "hihat", 23: "hihat", 38: "snare", 40: "snare_rim", 43: "tom", 44: None}

# -- the groove (41 notes, 8 pairs) -----------------------------------------------------------------------------
BPM = 75.                         # [S] written tempo: 0.8 s a beat, 16 control steps at 20 Hz
CONTROL_HZ = 20
PHYSICS_HZ = 240
STEPS_PER_BEAT = round(CONTROL_HZ * 60. / BPM)     # 16


def written_notes():
    """(beat from the first note, pad, dynamic) for the 41 written notes, in time order (hi-hat before snare on a
    shared count)."""
    notes = []
    for bar in range(3):                          # bars 1-3: 1 & 2 & 3 & 4 &
        for k in range(8):
            notes.append((4 * bar + k / 2, "hihat", "loud" if k % 2 == 0 else "soft"))
        notes += [(4 * bar + 1., "snare", "loud"), (4 * bar + 3., "snare", "loud")]
    for k in range(6):                            # bar 4: 1 & 2 & 3 &
        notes.append((12 + k / 2, "hihat", "loud" if k % 2 == 0 else "soft"))
    notes += [(13., "snare", "loud"), (15., "tom", "medium"), (15.5, "tom", "medium"),
              (16., "hihat", "loud"), (16., "snare", "loud")]
    notes.sort(key=lambda n: (n[0], ("hihat", "snare", "tom").index(n[1])))
    return notes


NOTES = written_notes()
PAIRS = (1., 3., 5., 7., 9., 11., 13., 16.)      # beats where the hi-hat and snare sound together
LAST_PAIR = 16.

# -- the score (judge_midi.py) ------------------------------------------------------------------------------------
BPM_RANGE = (60., 90.)            # the fitted tempo is searched in this range (every level needs it)
FULL_BPM = (71., 79.)             # 1.0 and 0.9 need the fitted tempo here
WINDOW_S = .050                   # a note is played by a hit on its pad within 50 ms of its fitted time
ON_TIME_S = .025                  # on time within 25 ms; off the grid from 25 to 50 ms
PAIR_S = .040                     # a pair is together when two different sticks land within 40 ms
RMS_FULL_S, RMS_075_S = .025, .035
ACCENT_MARGIN = 25.               # MIDI velocity: numbers over ands, snare over hi-hat ands
END_CLEAR = .050                  # clean end: both sticks held, tips at least 50 mm from every pad ...
END_HOLD_S = 1.                   # ... for 1 s, starting within 3 s of the last pair
END_WITHIN_S = 3.
AFTER_LAST_S = .5                 # hits more than this after the last note are extra (never matched)
COUNT_IN = 4                      # up to 4 hi-hat hits in the bar before the first note are a count-in
LEVELS = (  # (score, notes played H, pairs together G, RMS limit (s) or None, extra or wrong hits E at most)
    (.75, 37, 7, RMS_075_S, 3),
    (.5, 31, 5, None, 3),
    (.25, 16, 0, None, 3),
)
FULL = {"H": 41, "G": 8, "E": 1}
CAP_ARM_CONTACT = .5              # a gripper or arm striking a pad caps the score at 0.5
CAP_REST = .75                    # a stick resting on a pad for 0.5 s or more caps it at 0.75
POSSIBLE_SCORES = (0., .25, .5, .75, .9, 1.)
