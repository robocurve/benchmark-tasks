"""Every dimension and rule of the ukulele, its geared tuning buttons, its G string's pluckable stretch and the
clip-on tuner, in metres, each marked by where it comes from. Pure Python (``math`` only), so that the rules' and
the scorer's tests need nothing else.

Instrument frame (the ukulele's own): origin on the plane of the body's back, under the middle of the nut;
x across the strings toward the G string; y along the strings from the nut toward the bridge; z out of the back
through the soundboard toward the strings. The ukulele stands on its side in playing position (``fixed_scene``):
soundboard toward the robot, neck along the table's y, headstock to the robot's right, so the instrument's x points
up (the G string is the top string) and its z points toward the robot.

Sources (TECHNICAL.md has the links):
  [K] Kala's published specification of the KA-15S ("Specs" and "Measurements"): scale 13.625 in, overall length
      21.062 in, body length 9.5 in, 12 frets, upper bout 5.187 in, lower bout 7.0 in, waist 4.467 in, body depth
      2.367 in, fingerboard 1.375 in wide at the nut and 1.672 in at the neck/body joint, Aquila Super Nylgut
      strings.
  [P] Kala's product photos of the KA-15S: from the front, the G and A strings (the outer ones) run straight to the
      posts nearer the nut and the C and E strings angle outward, just inside those posts, to the posts farther
      from it, the far posts only a little farther out; from the back, open-gear tuners (a worm and gear exposed on
      a plate) with pearloid buttons.
  [S] the setup as written in task.json: re-entrant G4 C4 E4 A4, G string 100 cents flat, Snark ST-2 on the
      headstock tip.
  [E] ESTIMATE: not a published figure. Measure it on a real Kala KA-15S (TECHNICAL.md, "Estimates").
"""

import math

# -- the ukulele: Kala KA-15S soprano ---------------------------------------------------------------------------
SCALE = .346                     # [K] nut to saddle: 13.625 in


def fret(n):
    """Distance of fret ``n`` from the nut (equal temperament)."""
    return SCALE * (1. - 2. ** (-n / 12.))


LENGTH = .535                    # [K] overall, headstock tip to tail: 21.062 in
BODY_LENGTH = .2413              # [K] 9.5 in
LOWER_BOUT = .178                # [K] 7.0 in
UPPER_BOUT = .132                # [K] 5.187 in
WAIST = .113                     # [K] 4.467 in
BODY_Y0 = fret(12)               # [E] the neck meets the body at the 12th fret (173 mm; [K] 12 frets)
TAIL_Y = BODY_Y0 + BODY_LENGTH   # 414 mm from the nut
HEAD_LEN = LENGTH - TAIL_Y       # nut to the headstock's tip (121 mm), from the two published lengths
TIP_Y = -HEAD_LEN
# Half the body's width along its length (y, half-width): its widths at the upper bout, waist and lower bout are
# Kala's ([K]); where along the body they fall, and the curves between them, are estimates ([E]).
BODY_OUTLINE = ((BODY_Y0, .024), (.177, .0385), (.1845, .0525), (.1965, .0605), (.211, .065), (.2265, .066),
                (.243, .0625), (.2595, .057), (.277, .0565), (.2965, .0635), (.318, .078), (.3405, .087), (.360, .089),
                (.3795, .0855), (.394, .077), (.4045, .064), (.4115, .045), (TAIL_Y, .020))
INSTRUMENT_MASS = .45            # [E] fixed in place, so it acts on nothing


def body_half_width(y):
    """Half the body's width at ``y`` (straight between the outline's points); 0 off the body."""
    for (y0, w0), (y1, w1) in zip(BODY_OUTLINE, BODY_OUTLINE[1:]):
        if y0 <= y <= y1:
            return w0 + (w1 - w0) * (y - y0) / (y1 - y0)
    return 0.


# -- body, neck, fretboard ----------------------------------------------------------------------------------------
BODY_DEPTH = .060                # [K] back to soundboard: 2.367 in (constant here; a real body tapers a little)
BACK_Z = 0.                      # the back's plane
TOP_Z = BACK_Z + BODY_DEPTH      # the soundboard; the neck's top and the headstock's face are level with it
FB_T = .005                      # [E] fretboard thickness
FB_TOP = TOP_Z + FB_T
FB_END = .185                    # [E] the fretboard ends a little past the 12th fret
FRETS = 12                       # [K] 12 frets
NUT_W = .035                     # [K] fingerboard width at the nut: 1.375 in
NECK_W12 = .0425                 # [K] fingerboard width at the neck/body joint (the 12th fret): 1.672 in
NECK_DEPTH = (.019, .022)        # [E] fretboard top to the back of the neck, at the nut and at the 12th fret
SOUNDHOLE_Y, SOUNDHOLE_R = .225, .0255      # [E] 51 mm soundhole, its middle 225 mm from the nut
BRIDGE_Y = (.336, .353)          # [E] the bridge along the strings; the saddle is at the scale length
BRIDGE_W, BRIDGE_H = .060, .008  # [E]


def neck_width(y):
    return NUT_W + (NECK_W12 - NUT_W) * y / fret(12)


def neck_back_z(y):
    """Where the back of the neck is at ``y`` (straight from the nut to the 12th fret)."""
    return FB_TOP - (NECK_DEPTH[0] + (NECK_DEPTH[1] - NECK_DEPTH[0]) * y / fret(12))


# -- cradle: two padded foam V-blocks and one strap (the ukulele on its side) -------------------------------------
BODY_REST_Y = .360               # the lower bout's widest point sits in a foam V-block ...
NECK_REST_Y = fret(3)            # ... and the neck's lower edge in another, under the 3rd fret
REST_LEN = .060                  # [E] each block's length along the neck
REST_CHEEK = (.012, .015)        # [E] cheek thickness, and how far the cheeks rise past the edge they cradle
STRAP_Y = (.365, .390)           # the 25 mm strap round the lower bout and its block, behind the bridge
STRAP_W, STRAP_T = .025, .0015

# -- strings ------------------------------------------------------------------------------------------------------
STRINGS = ("g", "c", "e", "a")   # top to bottom when the ukulele is on its side: the G string is the top string
TUNED = "g"                      # [S] the string to tune
OTHERS = ("c", "e", "a")
NOTES = {"g": "G4", "c": "C4", "e": "E4", "a": "A4"}          # [S] re-entrant tuning
ACROSS = {"g": 1., "c": 1. / 3., "e": -1. / 3., "a": -1.}       # place across the string spacing
SPREAD = (.013, .020)            # [E] outer strings' distance from the centre line at the nut and at the saddle
STRING_HEIGHT = (.0015, .0055)   # [E] string middle above the fretboard top's plane at the nut and at the saddle
STRING_R = .0006                 # collider and drawn radius (a Super Nylgut G string is about 0.8 mm across)
START_CENTS = {"g": -100., "c": 0., "e": 0., "a": 0.}           # [S] G at F#4; C, E and A in tune


def string_point(name, y):
    """The middle of string ``name`` at ``y`` along the strings (nut 0, saddle SCALE)."""
    t = y / SCALE
    spread = SPREAD[0] + (SPREAD[1] - SPREAD[0]) * t
    return (ACROSS[name] * spread, y, FB_TOP + STRING_HEIGHT[0] + (STRING_HEIGHT[1] - STRING_HEIGHT[0]) * t)


# -- headstock and the four open-gear tuners ------------------------------------------------------------------------
HEAD_W = .052                    # [E] headstock width at the tuners
HEAD_T = .014                    # [E] headstock thickness; its face is level with the neck's top (TOP_Z)
HEAD_OUTLINE = ((0., .0175), (-.008, .021), (-.016, .026), (TIP_Y + .014, .026), (TIP_Y + .007, .022),
                (TIP_Y + .002, .014), (TIP_Y, .006))   # [E] (y, half-width), convex
POST_Y = {"g": -.035, "c": -.072, "e": -.072, "a": -.035}       # [P] G and A wind onto the posts nearer the nut
POST_X = {"g": .016, "c": .0185, "e": -.0185, "a": -.016}       # [E] the far posts 2.5 mm farther out ([P] a little)
POST_R, POST_H = .003, .010      # [E] chrome posts standing on the headstock's face
POST_WRAP_Z = .006               # strings wind onto the posts this far above the headstock's face, on the posts'
                                 # inner sides: the G and A strings run straight from the nut to theirs
SIDE = {"g": 1, "c": 1, "e": -1, "a": -1}        # which edge each button sticks out of: +1 the G-string edge (up)
HOUSING = (.014, .012)           # [E] each open gear (its plate, worm and gear) drawn as one box behind the
                                 # headstock: along y, thickness
HOUSING_IN, HOUSING_OUT = .010, .030             # [E] its extent across, from the centre line (4 mm past the edge)
AXIS_Z = TOP_Z - HEAD_T - HOUSING[1] / 2         # the buttons' shafts: 20 mm behind the headstock's face
SHAFT_R = .002                   # [E]
BUTTON_LEN, BUTTON_WIDE, BUTTON_THICK = .012, .018, .006     # [E] flat button: along its shaft, across, thick
BUTTON_IN = .036                 # [E] the button's inner end from the centre line (6 mm of shaft)
BUTTON_OUT = BUTTON_IN + BUTTON_LEN
BUTTON_MASS = .006               # [E]
BUTTON_ARMATURE = .00002         # kg m2: steadies the light button's joint
BUTTON_DRAG = .03                # [E] N m to keep a button turning
BUTTON_DAMPING = .1745           # N m s per degree, capped at BUTTON_DRAG: a friction-like drag (0.03 N m)
CENTS_PER_DEG = .22              # [E] about 80 cents per turn of the button (a 14:1 worm on a 6 mm post)
FREE_PLAY_DEG = 4.               # [E] the gear's free play: reversing, the button turns this far before the post
TIGHTEN = -1                     # [E] the pitch rises when a button turns clockwise seen from its end (a positive turn
                                 # about the button's outward axis is anticlockwise seen from its end)


def button_axis(name):
    """Outward unit vector of a button's shaft (the joint turns about the instrument's +x; see ``tighten_sign``)."""
    return (float(SIDE[name]), 0., 0.)


def button_centre(name):
    """The middle of a button, instrument frame."""
    return (SIDE[name] * (BUTTON_IN + BUTTON_LEN / 2), POST_Y[name], AXIS_Z)


def button_tip(name):
    """The middle of a button's outer end."""
    return (SIDE[name] * BUTTON_OUT, POST_Y[name], AXIS_Z)


def tighten_sign(name):
    """+1 if the button's joint angle (about the instrument's +x) rising tightens the string, else -1."""
    return TIGHTEN * SIDE[name]


# -- the G string's pluckable stretch over the soundhole -----------------------------------------------------------
PLUCK_Y = SOUNDHOLE_Y
PLUCK_LEN = .036                 # 36 mm: its ends stay over the soundhole (the finger pads are 15 mm wide)
PLUCK_TRAVEL = .008              # it stops this far either way across the strings (the C string is 11.7 mm away)
STRING_TENSION = 36.             # [E] N, G string at pitch
PLUCK_STIFFNESS = STRING_TENSION * (1. / PLUCK_Y + 1. / (SCALE - PLUCK_Y))       # N/m sideways there (about 460)
PLUCK_OMEGA = 30.                # rad/s: chosen so that a released string is back within 0.5 mm in about 0.1 s
PLUCK_ARMATURE = PLUCK_STIFFNESS / PLUCK_OMEGA ** 2                               # kg
PLUCK_DAMPING = 2. * .7 * math.sqrt(PLUCK_STIFFNESS * PLUCK_ARMATURE)            # damping ratio 0.7, N s/m
PLUCK_MASS = .002


def pluck_point():
    """The middle of the G string's pluckable stretch at rest."""
    return string_point(TUNED, PLUCK_Y)


# -- the pluck rule ---------------------------------------------------------------------------------------------
PLUCK_OUT = .002                 # pushed at least this far across the strings ...
PLUCK_BACK = .0005               # ... and back within this ...
PLUCK_RETURN_S = .3              # ... this soon after it was last that far out, with nothing touching it meanwhile
PLUCK_GAP_S = .3                 # plucks closer together than this count once

# -- the clip-on tuner: Snark ST-2, clipped on the headstock tip, its head in front of the headstock's face --------
TUNER_ACTIVE_S = 3.              # it reads for this long after a pluck
NEEDLE_MAX_DEG = 45.             # the needle swings this far ...
NEEDLE_RANGE_CENTS = 50.         # ... for 50 cents either side of the note shown
NEEDLE_REST_DEG = -45.           # and rests at the left end when the tuner is not reading
IN_TUNE_CENTS = 3.               # green within this
TUNER_NOTES = ("F#", "G", "G#")  # the simulated display's note lamps: semitones -1, 0, +1 from G4
TUNER_ELEV_DEG = 50.             # [S] screen turned toward the robot and up: its normal this far above level
TUNER_HEAD = (.046, .038, .014)  # [E] the head: across the screen, up the screen, thick
TUNER_SCREEN = (.030, .024)      # [E]
TUNER_CENTRE = (.008, TIP_Y + .009, TOP_Z + .034)    # [E] the head's middle: 34 mm in front of the headstock's face
CLIP_Y = (TIP_Y, TIP_Y + .014)   # [E] the clip's jaws grip the headstock's last 14 mm ...
CLIP_SPINE = .004                # ... from a spine just beyond the tip
CLIP_W, CLIP_T = .016, .004      # [E] jaw width and thickness
NEEDLE_LEN = .011
NEEDLE_PIVOT = (0., -.010)       # on the screen: (across, up) from its middle
LAMP_SIZE = (.008, .004)
LAMPS = {"lamp_fs": (-.010, .009), "lamp_g": (0., .009), "lamp_gs": (.010, .009)}     # (across, up)
IN_TUNE_LAMP = ("lamp_in_tune", (0., .0045), (.012, .002))
NOTE_LAMP = {-1: "lamp_fs", 0: "lamp_g", 1: "lamp_gs"}
NEEDLE_MASS = .0005
NEEDLE_ARMATURE = .00001
NEEDLE_STIFFNESS = 1.            # N m/rad, position drive; the target is set every step from the display
NEEDLE_DAMPING = .0063           # N m s/rad
NEEDLE_SIGN = -1.                # joint angle = this x displayed angle (a positive joint turn swings it to the left)


def screen_frame():
    """Columns, instrument frame: across the screen (the robot's view: its right), up the screen, out of the screen.
    On the table the screen faces the robot and up, its normal TUNER_ELEV_DEG above level (``fixed_scene``: the
    instrument's x is up, its z toward the robot)."""
    e = math.radians(TUNER_ELEV_DEG)
    across = (0., -1., 0.)
    up = (math.cos(e), 0., -math.sin(e))
    out = (math.sin(e), 0., math.cos(e))
    return across, up, out


def on_screen(across, up, lift=0.):
    """A point on the screen given (across, up) from its middle and ``lift`` out of it, instrument frame."""
    a, b, n = screen_frame()
    face = TUNER_HEAD[2] / 2 + lift
    return tuple(TUNER_CENTRE[i] + a[i] * across + b[i] * up + n[i] * face for i in range(3))


# -- what "at least 50 mm from the ukulele and tuner" is measured to ---------------------------------------------
CLEAR_M = .050


def clear_boxes():
    """Boxes ((x0, x1), (y0, y1), (z0, z1)) covering the ukulele, its buttons and the tuner, instrument frame."""
    boxes = []
    string_top = string_point("g", SCALE)[2] + STRING_R
    for (y0, w0), (y1, w1) in zip(BODY_OUTLINE, BODY_OUTLINE[1:]):
        w = max(w0, w1)
        boxes.append(((-w, w), (y0, y1), (BACK_Z, string_top)))
    w = NECK_W12 / 2
    boxes.append(((-w, w), (0., BODY_Y0), (neck_back_z(BODY_Y0), string_top)))
    boxes.append(((-HEAD_W / 2, HEAD_W / 2), (TIP_Y, 0.), (AXIS_Z - HOUSING[1] / 2, TOP_Z + POST_H)))
    for name in STRINGS:
        s = SIDE[name]
        x = sorted((s * HEAD_W / 2, s * BUTTON_OUT))
        boxes.append(((x[0], x[1]), (POST_Y[name] - BUTTON_WIDE / 2, POST_Y[name] + BUTTON_WIDE / 2),
                      (AXIS_Z - BUTTON_WIDE / 2, AXIS_Z + BUTTON_WIDE / 2)))
    corners = [on_screen(sa * TUNER_HEAD[0] / 2, sb * TUNER_HEAD[1] / 2, -TUNER_HEAD[2] * back)     # front, back face
               for sa in (-1, 1) for sb in (-1, 1) for back in (0, 1)]
    boxes.append(tuple((min(c[i] for c in corners), max(c[i] for c in corners)) for i in range(3)))
    boxes.append(((-CLIP_W / 2, CLIP_W / 2), (TIP_Y - CLIP_SPINE, CLIP_Y[1]), (TOP_Z - HEAD_T - CLIP_T, TUNER_CENTRE[2])))
    return boxes
