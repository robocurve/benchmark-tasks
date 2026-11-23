# Loosen a nylon nut with an adjustable wrench, rolling the jaw closed onto the flats first

An 8-inch adjustable wrench sits on an M5 nylon 6/6 hex nut (8 mm across flats), run down finger-tight and then a
quarter turn on a stud of a steel equipment mounting plate, its jaw left 1 mm wider than the nut. The robot is
asked to loosen the nut and take the wrench off. Nylon nuts are a normal fastener where a joint must insulate or not
corrode (electrical panels and enclosures, electronics mounts, wet equipment). Their service torque sits close to
their failure torque, and it is a load the arm can deliver.
A trained worker first rolls the worm (thumbwheel) until the jaw is closed onto the flats, then pulls so that the nut presses
into the fixed jaw, then lifts the wrench off and lays it down. Pulling with the loose jaw lets it bear on the nut's corners;
pulling the other way loads the movable jaw. Both score 0 as wrench-use faults, the two rules hand-tool safety
guidance gives for adjustable wrenches (Canadian Centre for Occupational Health and Safety, "Hand Tools - Wrenches"):
"Do not pull on an adjustable wrench that is loosely adjusted" and "the direction of the turn should be against
(towards) the permanent jaw". They are read from the jaw's play and the nut's turn; they do not measure damage to
the nut.

## Spec

| Object | Type, size, material | Position (cm, table frame) |
|---|---|---|
| Mounting plate | steel plate 26 x 30 x 0.8 cm, painted blue-gray, four corner bolts; fixed | center (27, 4) |
| Stud, washer, nut | M5 stainless stud; zinc M5 washer (DIN 125) 10 mm OD x 1 mm; M5 hex nut, natural nylon 6/6, DIN 934, 8 mm AF, 4 mm thick, a fresh one each trial, run down finger-tight and then a marked quarter turn | axis at (30, 0) |
| Adjustable wrench | 8-inch (205 mm), chrome steel, 0.25 kg, red grip; jaw open 9.0 mm; worm 16 mm across, 2 mm jaw per turn; lies on the washer, the nut in the lower 4 mm of its 10 mm jaws | on the nut, handle toward 60 deg |

Scoring (see `score()` in task.py):

| Outcome | Score |
|---|---|
| Doing nothing | 0 |
| Nut turned 2 deg or more in total, over all pulls, with jaw play over 0.28 mm (loose-jaw pull), or forced 2 deg or more clockwise (movable jaw loaded) | 0 (irreversible) |
| Nut loosened under 5 deg, or the wrench not let go at the end (a release check: a gripper with its tool point within 5 cm of the wrench and fingers under 60 % open still counts as holding it) | 0 |
| Loosened 5 to 60 deg | 0.25 |
| Loosened 60 deg or more, wrench not lifted off and laid down 3 cm or more from the nut | 0.5 |
| Loosened 60 deg or more, wrench laid down, not held | 1.0 |

Why each rule and level:

- Loose-jaw pull: a procedural fault, pulling on a loosely adjusted wrench (CCOHS: "Do not pull on an adjustable
  wrench that is loosely adjusted"; "Select the correct jaw size to avoid slippage"). The fault is the nut turning 2
  degrees or more, summed over every pull, while the play is over 0.28 mm (or the wrench turning 30 degrees round a
  nut its jaws cannot grip, over 1.24 mm of play). "Loosely adjusted" is set by the standard: ISO 6787:2001
  (adjustable wrenches), Table 1, gives the maximum clearance j of a new wrench's adjustable jaw for each length. The
  limit for a closed jaw is that clearance for this wrench's size: 0.28 mm (ISO 6787:2001, Table 1, 200 mm row; this
  wrench is 8-inch, 205 mm). A jaw rolled onto the flats has no more play than its own clearance, so a wider reading
  is a jaw left open. The rule scores wrench use, not damage to the nut. 2 degrees is read on the nut itself, so the
  jaws' play and the wrench's flex do not count toward it, and it is twice the 1-degree angle check of the real
  trial's video reading (see the real-world build).
  Play is tracked over the whole run, so a jaw closed for the first pull and opened again before a later one is
  caught. A real trial reads the same play and turn from video, frame by frame (see the real-world build).
- Movable jaw loaded: the other procedural fault, a turn against the movable jaw (CCOHS: "the direction of the turn
  should be against (towards) the permanent jaw"). A tightening pull lands on the movable jaw's tip, which levers it
  open in its slot. The fault is the nut forced 2 degrees or more clockwise. The turn is read on the nut, which does not move
  until it turns on its thread (in the sim, after 3.4 degrees of modeled wrench flex), so 2 degrees of it is a real
  tightening pull, not flex. A real trial
  reads that turn from the paint line on the nut.
- 5 degrees: a turn of the nut itself, well past the 2-degree reading margin, so the nut has broken loose and turned
  on its thread (the jaws' play and the wrench's flex move the wrench, not the nut).
- 0.25 for 5 to 60 degrees: the hard part is done (a closed jaw and the right pull broke the nut loose), but it is less
  than one swing.
- 60 degrees: one flat of the hex, a full swing before a mechanic resets the wrench on the next flats.
- 0.5 with the wrench left on: the nut is loose, but the wrench is still on it, in the way of spinning the nut off
  by hand. Set down means lifted off and lying on the plate with its jaws 3 cm or more from the nut's axis, wholly
  clear of the nut.
- Not let go at the end scores 0 because the job isn't finished while the hand is still on the wrench. This is a
  release check: a gripper with its tool point within 5 cm of the wrench and its fingers under 60 % open still
  holds it, so an open hand resting near it has let go.

Where the arms end up does not matter.

## How the sim models it

- `base` is one yamkit Mechanism (fixed). The nut is a link turned only by the model. The wrench is a chain: `lift`
  (vertical slide) -> `wr_turn` (about the nut's axis) -> `wr_x`, `wrench` (slides in the wrench's frame), with the
  movable `jaw` (kinematic, set from the worm) and the `wheel` (worm: free to roll, rubbed round by a fingertip) on
  the wrench body.
- The model (`Model`, `model_step`) runs once per control step from the nut's law. On the nut, with the jaw at least
  as wide as the nut, the wrench couples to the nut through a breakaway thread: within the jaws' play (`play_angle`,
  from the hex's geometry) it turns freely, then flexes against a spring (12.7 N*m/rad) up to 3.4 degrees (0.76 N*m),
  past which the nut turns with it. The worm stops when the jaw meets the flats.
- Off the nut (lifted 16 mm or more, where the jaws clear the stud's end) the wrench moves freely in its plane; laid
  down, its slides and turn are held. It lifts straight off the flats with the jaw closed, as a real one does: the
  jaw is never clamped on the nut, so it does not need opening first.
- In task.py the model's flag for a loose-jaw pull is named `rounded`; it marks the procedural fault described
  above.
- `yamkit.py` is the task's helper module for building the scene, with Link.parent chains, cylinder colliders along
  x/y and law-set joint limits.

## Real-world build

Off-the-shelf parts: a 260 x 300 x 8 mm steel plate bolted to the bench, an M5 x 20 stainless stud threaded into it, a
DIN 125 M5 zinc washer, M5 hex nuts of natural nylon 6/6 (DIN 934; one per trial), and an 8-inch
(205 mm) adjustable wrench (Crescent, Bahco or similar) with a dipped grip.

Tightening (the start state). Each trial's nut is run down the stud by hand until it seats on the washer
(finger-tight), then turned a further quarter turn (90 degrees) with the wrench against a pencil mark on the washer.
Setting the start by a rotation from contact, rather than by a torque reading, is the idea of the turn-of-nut method
used for structural bolts (RCSC Specification for Structural Joints Using High-Strength Bolts, Section 8.2.1: a set
nut rotation applied from the snug-tight state; match-marking is the usual aid). RCSC's own rotations are for steel
bolts; the quarter turn is this task's rotation for the nylon nut, so the starting state is set by the procedure, with
no waiting period and no torque reading. A nut that strips or spins during the quarter turn is discarded and a fresh
one fitted. Every scoring rule reads angles (the play
while the nut turns, the nut's clockwise turn, how far it loosened), and the offline check
`scoring_holds_across_breakaway_range` runs the same pulls with the thread breaking loose at 0.10, 0.38, 0.76 and
0.93 N*m, from a nut that has almost let go up to 0.93 N*m, the ultimate torque listed for M5 nylon 6/6 (Plastic Nuts & Bolts,
nylon 6/6 mechanical data, dry as molded, 23 C), so the most the quarter turn can leave in the nut. The scores are the same at every one:
a full job with a closed jaw 1.0, a loose-jaw pull 0, a turn against the movable jaw 0, 0.25 mm of play 1.0 and 0.32 mm 0. The
sim's thread breaks loose at 0.76 N*m, the maker's recommended torque (0.73-0.79 N*m), inside that range. Even at the
nut's 0.93 N*m strength the pull is 7.2 N at the grip, 13 cm from the nut; the YAM's shoulder and elbow servos (Damiao
DM-J4340, 9 N*m rated, 27 N*m peak) and wrist servos (DM-J4310, 3 N*m rated, 7 N*m peak) give, worked through the
arm's Jacobian with the torque that holds the arm up included, 11 N or more in any horizontal direction at rated
(continuous) torque in typical tabletop poses, so every start the procedure can give is within the arm's continuous
rating.

What the faults score: the two procedural rules above (a closed jaw, the turn toward the fixed jaw), read from video.
Before and after photos of the nut are kept with the results as a record.

Before and after each trial, photograph the nut's top square-on from 15 cm with a macro lens or a phone in macro mode,
in the same raking light and at the same magnification.

Measuring during the run (the nut's turn and the jaw's play, which decide both faults): fix a camera 25 cm
above the nut, looking straight down. Paint a thin line across the nut's top and a reference line on the plate. Draw
a fine line on the movable jaw's top face and stick a printed 0.1 mm vernier on the head beside it (main scale on the
head, vernier on the jaw), zeroed with the jaw closed on the nut's flats. Calibrate the reading before the trials: set
the play with 0.1, 0.2, 0.25, 0.3 and 1.0 mm feeler gauges between jaw and flat, film each from the trial camera, and
check the video reading is within 0.05 mm of the gauge. The vernier reads in 0.1 mm steps, so a reading of 0.3 mm or
more is over the 0.28 mm limit. Check the angle reading the same way: film a painted nut turned to 0, 2, 5 and 60
degrees against a printed protractor and check the angle tool reads each within 1 degree. Record both checks with the
results. After the run, step through the
video with an angle tool (Kinovea or similar): in every frame read the nut's angle and the jaw's play. Add up the
nut's turn in the frames where the play reads over 0.28 mm, and its clockwise turn, across every pull and every worm
adjustment the robot makes. 2 degrees or more of either is a fault, as in the sim. If the scale is hidden by a finger
while the nut turns, use the larger of the readings just before and just after. Measure where the wrench was laid
down with a tape measure.

Open the jaw to 9.0 mm with a caliper and lay the wrench on the washer with the nut in its jaws and its handle toward
the robot's front-left (60 degrees), the flats square in the jaws.

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/adjustable-wrench-jaw-snug/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Loosen the nut on the base plate with the adjustable wrench, then take the wrench off it and set it down." \
      --scorer task_score --max-steps 2400
    python tasks/adjustable-wrench-jaw-snug/test_task.py      # offline scorer checks, prints "all pass"

Dream-1: the same with `--policy dream`. One trial takes about 2 minutes (2,400 steps at 20 Hz);
object_jitter=0, so there is no seed dependence.

Recorded runs on this task.py: the scripted demo scores 1.0 in 1,082 steps (54 seconds); Dream-1 scores 0 in 2,400 steps. In the sim the nut turns on its stud through a modeled thread (breakaway at 0.76 N*m); the demo breaks
it loose by pulling the wrench round in three strokes and then lifts the wrench off, and the nut is never lifted.
