# Put the lid back on a jack-o'-lantern

Instruction: "Put the lid back on the jack-o'-lantern."

A carved pumpkin stands on the bench with its lid resting on the opening at the wrong angle. The lid and the opening
were cut with the knife leaning inward, so both cut edges are 45-degree cones. The cut has one key, at the back of the
pumpkin, opposite the face: a 10 mm tooth left standing on the opening's cut edge, and a matching notch cut in the
lid's edge. With the notch anywhere but over the tooth, the lid rests on the tooth, about 5 mm proud and tilted.
Pressing it down does nothing. It has to be lifted slightly and turned about its axis until the notch comes round to
the tooth. Then it drops flush.

Carving guides tell people to cut the lid this way. The inward lean stops the lid falling in, and the notch makes it
go back only one way ([HowStuffWorks: cutting the lid](https://people.howstuffworks.com/culture-traditions/holidays-halloween/pumpkin-carve2.htm),
[HGTV](https://www.hgtv.com/lifestyle/holidays/how-to-carve-a-pumpkin)). The lid comes off every time the candle is
lit or checked.

## Why this is not just putting a lid on

A placement task, such as standing a candle in a holder or setting a lid on a tin, is decided by where the object is
put down. Here the lid is already on the opening at the start, so a policy that only puts it down, presses it or lets
go scores 0. The deciding move is turning the lid about the opening's axis while it bears lightly on the cut. The
target of that turn cannot be seen once the lid is over the opening:

- **The tooth is hidden.** It stands under the lid's edge. The lid hides it from the top camera and from both wrist
  cameras for the whole trial, unless the lid is lifted right off. The lid's own notch can be seen on its top face;
  the tooth it has to meet cannot.
- **The face is the only visible cue.** The tooth is on the far side from the carved face, where carving guides put
  it. A policy can reason from the face to roughly where the tooth is, and then has to find it by feel: turn under
  light contact until the lid drops. Pressing down on a lid that sits on the tooth leaves it proud.
- **A memorised angle fails.** Where the face points changes between trials (three layout lots), and so does the
  lid's start angle. A memorised final lid angle, or a memorised turn, scores in at most one lot.
- **What scores is the end state.** The lid only scores if its notch ends on the tooth with the lid flush. Turning it
  in the air and lowering it, when the angle has been judged right from the face, also scores. Either way the robot
  has to work out the hidden key's position and turn the lid to it.

Before any simulator run the contact was checked with a rough MuJoCo model of the same shapes (rigid bodies, default
contact settings, no robot). These are probe results, not measurements:

- Dropped 10 mm onto the opening with the key lined up, the lid seated level with the rim and stayed still.
- From any wrong angle between 40 and 320 degrees, it rested 4.9 to 5.1 mm proud, tilted 4 to 6 degrees.
- Pressed straight down with 1, 3 or 10 N, it stayed 3.9 to 4.4 mm proud.
- Held level by the stem with a 0.5 or 2 N downward press and turned toward the key, it dropped flush once the notch
  reached the tooth, from all five start angles tried.
- A turn that stopped 12 degrees short left it 4.7 mm proud.

The GPU runs below confirm the start state: the perched lid sat 4.4 to 4.8 mm proud, tilted 3 to 5 degrees, and still.

## Run

Install yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) and run, from this folder:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
      --policy yam_scripted --no-prompt \
      --instruction "Put the lid back on the jack-o'-lantern." \
      --scorer task_score --max-steps 1200 --epochs 3

Trial k uses lot k mod 3 (lot A first); `--epochs 3` runs lots A, B and C and reports their mean as `task_score`.
`TASK_LOT=A`, `B` or `C` pins every trial to one lot. Use `--policy dream` for Dream-1. The task needs
`object_jitter=0`. No seeds are used.

- **Runtime per trial:** the scripted demo takes about 28 s of simulated time.
- **Offline checks (no simulator):** `python near_misses.py` scores every near miss below, the whole-trial rules,
  lots B and C, and the hand-gap sampling.
- **The demo** (`demo()` in task.py) is open loop. It reads only the published layout of the trial: which way the
  pumpkin faces (so where the tooth is, opposite the face) and the angle of the lid's notch, which is visible on the
  lid's top. It never reads the simulator's states. The left hand pinches the stem with the fingers closing along
  the tooth's direction: the perched lid leans that way, and the pads stand the stem upright as they close. It lifts
  the lid 12 mm off the tooth, turns it the short way round until the notch is over the tooth, lowers it to 4 mm
  above seated and lets go, so the lid drops into the cut. It then squares the lid up: it pinches the stem four
  times where it stands, a quarter turn round between pinches, and lets go each time. Then the hand leaves.
  `tilt.py` is imported only to carry the rest pose's rotation in a waypoint; every grip is straight down. `lots.py`
  turns the pumpkin and the lid between lots; `parts.py` builds the multi-shape bodies.

## Scene

World frame: x away from the robot, y to its left, z up; bench top z 0.7475 m. All parts are rigid.

- **Pumpkin** (free on the bench): a carvable foam craft pumpkin, ballasted to 0.80 kg, 146 mm across at the
  widest and 105 mm from the bench to the cut at the top, with a flat 100 mm bottom. Orange skin with ten shallow
  ribs. A face is carved on one side: two eyes, a nose and a mouth. Its axis stands at (0.300, 0.110) m.
- **The opening:** cut with the knife leaning 45 degrees inward, so the cut is a cone 66 mm across at the rim and
  36 mm across 15 mm down (the wall's thickness there). The rim is 105 mm above the bench. On the side opposite the
  face, a **tooth** 10 mm wide is left on the cut: there the cut surface stands 8 mm further in, over the whole 15 mm
  depth.
- **Lid** (ballasted to 0.030 kg), cut from the same wall: a cone 65 mm across at its top face and 41 mm across at its
  bottom, 12 mm thick, with the same 45-degree edge. A **notch** 14 mm wide and 10 mm deep is cut in its edge to
  take the tooth, 2 mm clear of it each side. The stem stands on its middle, 15 mm across and 26 mm tall. Seated with the notch on the tooth, the
  lid's top is level with the rim.
- **Start:** the lid is set on the opening level, 10 mm up, with its notch turned the lot's wrong angle from the
  tooth, and settles onto the tooth before the trial.
- **Lot A:** the face points at the robot (−x), so the tooth is on the far side (+x). The lid's notch is turned
  110 degrees counterclockwise (seen from above) from the tooth, pointing 110 degrees from +x.
- **Lot B:** the pumpkin is turned 90 degrees counterclockwise: the face points to the robot's right (−y) and the
  tooth to its left (+y). The notch is turned 75 degrees clockwise from the tooth, pointing 15 degrees from +x.
- **Lot C:** the pumpkin is turned 55 degrees clockwise: the face points toward the robot and to its left, and the
  tooth away and to the right, 55 degrees clockwise from +x. The notch is turned 155 degrees counterclockwise from the
  tooth, pointing 100 degrees from +x.

## Scoring

Measured in the pumpkin's own frame (its axis up, through the opening's centre), per trial:

- **Keyed:** the lid's notch is within 8 degrees of the tooth, about the opening's axis.
- **Flush:** the lid's top face, at its centre, is within 2 mm of level with the rim.
- **Level and centred:** the lid is tilted under 3 degrees from the pumpkin's axis, and its centre is within 3 mm of
  the opening's axis.
- **Let go and still:** every gripper link and fingertip is more than 5 mm from the lid and its stem, and the lid and
  pumpkin move no more than 1 mm for 1 s.

1.0 when all of these hold together; 0 otherwise. That includes a lid still held, a lid resting on the tooth, a lid
hanging on the notch's edge, a lid seated with its notch anywhere else, a lid upside down, and a lid on the bench.

Whole trial: the trial scores 0 in these cases, even if the lid ends seated:

- the pumpkin is ever pushed more than 10 mm, or turned or tipped more than 5 degrees, from where it started;
- the lid ever falls in through the opening (its top more than 20 mm below the rim, inside the cut);
- the hands, the lid or the pumpkin cannot be read at any step.

Lifting the lid off and setting it down on the bench, to look at the opening, is allowed.

Trials are run on lots A, B and C and averaged.

How the code checks it:

- **Hand gap:** the hand gap uses the YAM collision hulls of each arm's `gripper`, `tip_left` and `tip_right` links,
  with every face sampled at 3 mm. It is measured to every shape of the lid and stem (round shapes as their boxes).
- **Hold:** the 1 s hold restarts whenever the lid is not keyed and flush, a hand is within 5 mm, or any of seven
  tracked points on the lid and pumpkin moves more than 1 mm.
- **Fail closed:** any unreadable step ends credit for the trial.

Near misses in `near_misses.py`, each with its expected score:

| Case | Score |
|---|---|
| Keyed and seated, hands clear, 1 s still | 1.0 |
| As it starts: on the tooth 110 deg out, 5 mm proud, tilted 5 deg | 0 |
| Pressed down level but still on the tooth (4 mm proud) | 0 |
| Hanging on the notch's edge: 15 deg out, 1.2 mm proud, tilted 7 deg | 0 |
| Key 7.9 deg out / 8.1 deg out | 1.0 / 0 |
| Notch toward the face (180 deg out) at seated height | 0 |
| Top 1.9 mm proud / 2.1 mm proud / 2.1 mm low | 1.0 / 0 / 0 |
| Tilted 2.9 / 3.1 deg | 1.0 / 0 |
| 3.1 mm off the opening's axis | 0 |
| Upside down in the opening | 0 |
| Set down on the bench | 0 |
| Fallen in, on the pumpkin's floor | 0 |
| A fingertip 4.9 mm from the stem / nearest hand point 5.1 mm | 0 / 1.0 |
| 0.9 s still / 1.0 s still | 0 / 1.0 |
| Pumpkin shoved 10.5 mm at one step / 9.5 mm | 0 / 1.0 |
| Pumpkin turned 5.5 deg at one step | 0 |
| Lid fell in once, then fished out and seated | 0 |
| Lid set on the bench once, then seated | 1.0 |
| Lots B and C: lid keyed to the turned tooth / lid at lot A's angle | 1.0 / 0 |

## Results

Same task.py (SHA-256 `5e9711d84571958e4e7fa4ce20c2cb107fe9468f5e67f74971c6ca3b5118dc94`), same instruction,
`object_jitter=0`, `--epochs 3`:

- **Scripted demo:** 1.0 in each of lots A, B and C (`task_score` 1.0).
- **Dream-1:** 0.0 in each lot. In every trial it ran to the 1,200-step limit; it drew both arms back near their
  bases and never touched the pumpkin or the lid.

These are simulation results, not measurements on a physical rig.

## Fidelity and sources

- **Pumpkin and lid sizes.** True size for a small carving pumpkin. A carvable foam craft pumpkin stands in for a
  real one, so the rig does not spoil. A bare foam pumpkin weighs tens of grams and would be dragged by the lid, so it
  is ballasted to 0.80 kg with a sand bag on its floor, and the lid to 30 g with a steel washer glued under the stem.
  The masses, the 15 mm wall at the opening and the 12 mm lid are estimates; weigh and measure them on the foam
  pumpkin used. For a repeatable physical trial, cut the
  lid and the opening to a printed template: a 66 mm circle at the rim, the knife held at 45 degrees inward, and the
  tooth and notch marked on the template opposite the face.
- **Shape.** The pumpkin's body is drawn as stacked cylinders with ribs. It collides as a 24-sided wall, a top skin
  and a flat floor. The conical cut on the lid and the opening is 24 flat facets. Faces are drawn, not cut through.
- **Material.** Friction is 0.6 for the cut foam on the lid and 0.7 for the pumpkin's painted skin; both are
  estimates. The foam is rigid here; a real foam cut edge can dent slightly under a pinch.
- **Force.** Contact force is not scored. Hard pinching can dent the foam, which the simulator does not model.
- **Lots.** The three lots stop a single memorised angle or replay. They do not stop a policy that has memorised all
  three layouts.

Code MIT; documentation CC BY 4.0. All geometry is generated in task.py; no third-party assets are used.
