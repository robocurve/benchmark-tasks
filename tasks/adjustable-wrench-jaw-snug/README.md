# Loosen a nylon nut with an adjustable wrench, rolling the jaw snug on the flats first

An 8-inch adjustable wrench sits on an M5 nylon 6/6 hex nut (8 mm across flats), tightened to its maker's
recommended 0.76 N*m on a stud of a steel equipment mounting plate, its jaw left 1 mm wider than the nut. The robot is
asked to loosen the nut and take the wrench off. Nylon nuts are a normal fastener where a joint must insulate or not
corrode (electrical panels and enclosures, electronics mounts, wet equipment). Their service torque sits close to
their failure torque, so a steel jaw bearing on the corners can shave or mark them at this load, and it is a load the
arm can deliver.
A trained worker first rolls the worm (thumbwheel) until the jaw is snug on the flats, then pulls so that the nut presses
into the fixed jaw, then lifts the wrench off and lays it down. Pulling with the loose jaw lets it bear on the nut's corners,
which it can shave or mark; pulling the other way loads the movable jaw. Both score 0, through operational proxies
(the jaw's play and the nut's turn), not a validated measurement of rounding.

## Spec

| Object | Type, size, material | Position (cm, table frame) |
|---|---|---|
| Mounting plate | steel plate 26 x 30 x 0.8 cm, painted blue-gray, four corner bolts; fixed | center (27, 4) |
| Stud, washer, nut | M5 stainless stud; zinc M5 washer (DIN 125) 10 mm OD x 1 mm; M5 hex nut, natural nylon 6/6, DIN 934, 8 mm AF, 4 mm thick, a fresh one each trial, tightened to 0.76 N*m | axis at (30, 0) |
| Adjustable wrench | 8-inch (205 mm), chrome steel, 0.25 kg, red grip; jaw open 9.0 mm; worm 16 mm across, 2 mm jaw per turn; lies on the washer, the nut in the lower 4 mm of its 10 mm jaws | on the nut, handle toward 60 deg |

Scoring (see `score()` in task.py):

| Outcome | Score |
|---|---|
| Doing nothing | 0 |
| Nut turned 2 deg or more in total, over all pulls, with jaw play over 0.4 mm (loose-jaw pull), or forced 2 deg or more clockwise (movable jaw loaded) | 0 (irreversible) |
| Nut loosened under 5 deg, or the wrench not let go at the end (a release check: a gripper with its tool point within 5 cm of the wrench and fingers under 60 % open still counts as holding it) | 0 |
| Loosened 5 to 60 deg | 0.25 |
| Loosened 60 deg or more, wrench not lifted off and laid down 3 cm or more from the nut | 0.5 |
| Loosened 60 deg or more, wrench laid down, not held | 1.0 |

Why each rule and level:

- Loose-jaw pull. A jaw with play lets the wrench turn on the nut until it bears on the corners, where a steel jaw
  can shave or mark a nylon nut near its service torque. The sim's nut can't deform, so the scorer uses an
  operational proxy for jaw slip and corner damage, not a validated measurement of rounding: the nut turning 2
  degrees or more, summed over every pull, while the play is over 0.4 mm (or the wrench turning 30 degrees round a
  nut its jaws cannot grip, over 1.24 mm of play). 0.4 mm comes from the
  hex's geometry: with 0.4 mm of play the 8 mm nut turns 5.4 degrees in the jaws before they bear, and they bear on its
  corners. A new 150 mm adjustable wrench's jaw clearance is at most 0.25 mm (ISO 6787:2001, Table 1), so a snug jaw
  reads well under 0.4 mm.
  Play is tracked over the whole run, so a jaw snugged for the first pull and opened again before a later one is
  caught. A real trial reads the same play and turn from video, frame by frame; before and after photos of the
  nylon nut's corners are a secondary check (see the real-world build).
- Movable jaw loaded. A tightening pull lands on the movable jaw's tip, which levers it open in its slot, and the
  wrench springs off. At 0.76 N*m a real jaw won't spring open, so this is an operational proxy too: the nut forced 2 degrees or
  more clockwise. The nut only moves once the wrench has flexed 3.4 degrees against the thread, so 2 degrees of nut
  turn is a real tightening pull, not flex. A real trial reads that turn from the paint line on the nut.
- 5 degrees: more than the jaws' play and the thread's flex, so the nut has really turned on its thread.
- 0.25 for 5 to 60 degrees: the hard part is done (a snug jaw and the right pull broke the nut loose), but it is less
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
  down, its slides and turn are held. It lifts straight off the flats with the jaw snug, as a real one does: the
  jaw is never clamped on the nut, so it does not need opening first.
- `yamkit.py` is the task's helper module for building the scene, with Link.parent chains, cylinder colliders along
  x/y and law-set joint limits.

## Real-world build

Off-the-shelf parts: a 260 x 300 x 8 mm steel plate bolted to the bench, an M5 x 20 stainless stud threaded into it, a
DIN 125 M5 zinc washer, M5 hex nuts of natural nylon 6/6 (DIN 934; one per trial, a few cents each), and an 8-inch
(205 mm) adjustable wrench (Crescent, Bahco or similar) with a dipped grip.

Torque. Nylon 6/6 M5 fasteners are rated at 0.73-0.79 N*m recommended torque and 0.93 N*m ultimate torque (Plastic
Nuts & Bolts, nylon 6/6 mechanical data, dry as molded, 23 C). Tighten each nut to 0.76 N*m with an adjustable torque
screwdriver (0.3 to 1.2 N*m range) driving an 8 mm socket, wait 5 minutes and start the trial. Nylon relaxes under
load, so the breakaway is measured rather than assumed: every tenth nut, instead of a trial, is turned loose with a
peak-hold torque screwdriver after the same 5 minutes, and the reading is logged. The sim uses 0.76 N*m; the logged
breakaways are reported with the results. At 0.76 N*m the pull is 5.8 N at the grip, 13 cm from the nut.

Damage at this load. The scored fault is the play-and-turn reading below: the 0.4 mm play threshold and the turn
readings are operational proxies for jaw slip and corner damage, not a validated measurement of rounding. Whether a
loose jaw visibly marks a nylon nut's corners at 0.76 N*m has not been bench-tested yet; no published data quantify it. Before scored
trials, a validation set is run by hand: ten nuts loosened with the jaw 1 mm open and ten with it snug, all at the
same torque, each photographed before and after. The share of corners marked in each set is reported with the results.
Until then the photos are a second check beside the video readings, not the score.

Before and after each trial, photograph the nut's top square-on from 15 cm with a macro lens or a phone in macro mode,
in the same raking light and at the same magnification. Any visible flattening, shaving or burr on a corner in the
after photo that is not in the before photo is a loose-jaw pull.

Measuring during the run (the nut's turn and the jaw's play, which decide both faults): fix a camera 25 cm
above the nut, looking straight down. Paint a thin line across the nut's top and a reference line on the plate. Draw
a fine line on the movable jaw's top face and stick a 0.5 mm scale on the head beside it, zeroed with the jaw closed
on the nut's flats; check it with 0.4 mm and 1 mm feeler gauges between jaw and flat. After the run, step through the
video with an angle tool (Kinovea or similar): in every frame read the nut's angle and the jaw's play. Add up the
nut's turn in the frames where the play reads over 0.4 mm, and its clockwise turn, across every pull and every worm
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

Recorded runs on this task.py: the scripted demo scores 1.0 in 1,082 steps (54 seconds), and Dream-1 scores 0 in
2,400 steps. In the sim the nut turns on its stud through a modeled thread (breakaway at 0.76 N*m); the demo breaks
it loose by pulling the wrench round in three strokes and then lifts the wrench off, and the nut is never lifted.
