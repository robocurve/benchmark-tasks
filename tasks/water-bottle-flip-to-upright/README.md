# Flip a one-third-full water bottle end over end so it lands standing upright on the desk pad

A capped 500 ml plastic water bottle, filled to one third (170 ml), stands on a felt desk pad. The robot has to throw it
so that it turns once end over end in the air and lands standing on its base: the "water bottle flip".

Instruction: *"Flip the water bottle so that it lands standing upright on the desk pad."*

[`mechkit.py`](mechkit.py) builds the bottle (with its water) and the pad, [`simready.py`](simready.py) loads the
distractor props, and [`demokit.py`](demokit.py) plays the closed-loop scripted demonstration in `task.py`.

## The knack

The 185 g bottle is held near the cap and flicked: it has to leave the hand spinning at roughly 12 to 15 rad/s and
moving upward at about 3 to 4 m/s, at the moment its body is near horizontal. In the air, the spin flings the water
from the base towards the cap. That raises the bottle's moment of inertia about threefold and slows the spin to about
a third (Dekker et al. 2018, below), so that after one turn the bottle comes down base first, almost without spin, and
the water sliding back to the base soaks up the landing. The window is narrow:

- in our simulation, 0.25 rad/s more or less on the wrist roll (3.0 or 3.5 instead of 3.25), or starting the throw
  2 cm further back, and the bottle still lands base first but with a little spin or drift, and topples (0.5);
- flick much too hard and it over-rotates onto its side or cap (0.25); let go a step (50 ms) early and it leaves
  moving forward and lands off target (see the failure checks below).

Of 17 throw settings we tried on the GPU physics near the final one, 2 stood the bottle up (the rest scored 0.5,
apart from one 0.25 and one 0); a clip of a few seconds shows whether it worked.

## Importance and sources

- **Relatability.** The water bottle flip became a worldwide craze in 2016 after a high-school talent-show video:
  "within weeks, this 30 seconds clip becomes viral on the internet and kids around the globe are seen attempting the
  'water bottle flipping challenge'", and "millions of flippers" argue over the best filling. Dekker et al. open their
  paper with that history and analyse the
  physics: P. J. Dekker, L. A. G. Eek, M. M. Flapper, H. J. C. Horstink, A. R. Meulenkamp, J. van der Meulen,
  E. S. Kooij, J. H. Snoeijer and A. Marin, "Water bottle flipping physics", *American Journal of Physics* 86, 733
  (2018); [arXiv:1712.08271](https://arxiv.org/abs/1712.08271). They found that "the angular velocity of the bottle
  decreases dramatically, enabling a nearly vertical descent", give the best filling as 20 % to 40 % of the bottle's
  height (a third is the folk rule), and model the water with a "tennis bottle": two balls in a tube. Their model
  predicts the spin falling to 0.36 of its starting value at their optimum filling.
- **Why it is a robot test.** It is open-loop, dynamic manipulation: one release decides everything, with a timing
  window of about one control step, an object whose mass moves inside it, and a landing that is either stable or
  not. None of it can be corrected after the bottle leaves the gripper. It tests timing, speed and spin control
  together, not seeing or choosing.

## Distinct from

| Nearest | What it does | Why this is a different ability |
|---|---|---|
| #3270, rolling five dice from a cup into a tray | Tips loose dice out of a cup so that they tumble into a tray | The dice land in random poses and the cup is never thrown. Here a single object is thrown free of the hand with a set amount of spin and must land in one pose. |
| #1435, flipping a pancake in a pan (not accepted) | Tosses food from a held pan and catches it in the pan | A pan toss keeps the object near the pan and catches it. Here the object leaves the hand, turns a full turn about its own centre, and lands on a narrow base with no catch. Its moving internal mass is what lets it land. |
| #2746, sending a pull-back car through a gate (accepted) | Winds a toy and lets it go cleanly | Release timing only, on the table. Nothing flies or turns in the air. |

We found no bottle flip, or other thrown object that has to land in a set pose, on the task board
(pull request titles searched for flip, toss, throw and bottle). LIBERO, RoboCasa, SimplerEnv, BEHAVIOR-1K and
RoboArena have no throwing tasks that we found. This was a manual check of their task lists, not an exhaustive search.

## The scene

Positions in cm from the midpoint between the arm bases (x forward, y left), on the table top (z 74.75 cm):

| # | Object | Model | Size, material, mass | Where |
|---|---|---|---|---|
| 1 | Desk pad | fixed box (custom) | 40 x 40 x 0.3 cm grey felt, taped flat | centred at x 36, y -16 |
| 2 | Water bottle | free articulation (custom) | capped 500 ml (16.9 fl oz) PET bottle, Poland Spring size: 6.2 cm across, 20.3 cm tall, cap 3.1 cm across and 1.9 cm tall, neck ring 3.3 cm; 15 g empty with cap; 170 ml of water to a line 6.2 cm above the base; 185 g in all | standing upright on the pad, base centred at x 30, y -6 |
| 3 | Notebook (distractor) | SimReady `misc_notebook_a01` | 22.9 x 17.0 x 1.0 cm paper, 120 g | centred at x 20, y 27 |
| 4 | Mug (distractor) | SimReady `dishware_mug_a01` | 11.4 x 9.0 x 8.0 cm ceramic, 300 g | x 42, y 22 |
| 5 | Apple (distractor) | SimReady `food_produce_apple_b01` | about 7 cm across, 180 g | x 56, y 12 |

Both arms start at the ready pose with their grippers open. The distractors are 6 cm or more from the pad and the
bottle, and away from the throw. The scorer ignores them.

### How the bottle and its water are modelled

- **Shell.** The bottle and cap are one 15 g rigid body, made as a lathe shape: a 6.2 cm body up to 12.5 cm, a
  shoulder tapering to a 2.7 cm neck at 17.2 cm, a 3.3 cm neck ring at 18.1 cm and a 3.1 cm cap from 18.4 cm to
  20.3 cm. It collides as a cylinder (body), a convex hull (shoulder) and a cylinder (neck and cap). Its base is flat;
  a real bottle stands on five shallow feet in the same footprint. The label is drawn on, and the 170 ml line is drawn
  as a black ring.
- **Water.** The 170 g of water is two 85 g slugs, each on its own slider along the bottle's axis. Both have gravity
  and 0.4 N s/m of viscous damping (an estimate for the sloshing loss). At rest both sit at the base, filling the
  bottom 5.9 cm. The lower slug can travel 15 cm (with the bottle upside down at rest, the water collects at the cap).
  The upper slug can travel 7.8 cm. We set that travel from the paper's eq. 14: with M = 500/15 and a fill of 0.29,
  the paper's water spread along the whole bottle gives I0/Imax = 0.30, and two slugs reach the same ratio at 7.8 cm.
  Spinning in flight throws the upper slug towards the cap, which slows the spin. On landing, it slides back and its
  damping absorbs the impact. Nothing sloshes sideways.
- **Contact.** Friction (static/dynamic): bottle 0.6/0.5, felt pad 0.8/0.7 (estimates); restitution 0 everywhere
  (mechkit's default). A real PET bottle bounces a little on landing; the water absorbs most of it.

## Scoring

The scorer runs every control step (20 Hz). It reads the bottle's pose, its two water slugs (for the centre of mass)
and 24 points on the gripper fingers (mechkit's `fingertips`).

- **Touch.** A finger touches the bottle when any finger point is within 12 mm of its surface (the lathe profile).
- **Flight.** A flight starts on the first step on which the bottle is airborne (no finger touching, its lowest point
  more than 12 mm above the table or pad) straight after a step on which a finger was touching it. So it was released
  from the hand; a bounce off the table does not start a flight. It ends on the first step on which a finger touches
  the bottle, its lowest point is within 12 mm of the table or pad, or its centre of mass leaves a free-fall path by
  more than 15 mm over two steps (an impact). A bottle that falls off the table earns nothing from that flight.
- **Turn.** The angle the bottle's axis sweeps, summed step by step from the last step in the hand to the end of the
  flight. This is end-over-end rotation; spin about the bottle's own axis does not count.
- **Flip.** A flight that turns 240 degrees or more and ends without a finger touching the bottle. A half flip (let go
  upside down) turns 180 degrees, and a drop turns almost nothing.

| Score | Condition |
|---|---|
| 0 | No flip (doing nothing, setting the bottle down, dropping it, a half flip, a flight a finger touched, or one that comes down on the robot or off the table) |
| 0.25 | A flip that comes down on the table or pad: its lowest point within 3 cm of the surface on the step that ends the flight |
| 0.5 | Such a flip whose first touchdown is base first on the pad: axis within 45 degrees of upright and base centre over the pad, on the step that ends the flight |
| 1.0 | After a flip, with no finger touching the bottle since then, it stands on its base on the pad (axis within 10 degrees of vertical, base within 5 mm of the pad's top, base centre on the pad), still (under 2 cm/s and 0.3 rad/s), with every finger point 5 cm or more from it, for 2 s (40 steps) |

The best level reached counts. A finger touching the bottle cancels a landed flip's claim to 1.0; it must be flipped
again. Any number of attempts fits in the run limit (1,200 steps, a minute). The trial's record is cleared only in
mechkit's `on_reset` hook. Possible scores: 0, 0.25, 0.5, 1.0.

## A real trial: preparation, measurement, reset

**Parts (about $20).** A 16.9 fl oz (500 ml) Poland Spring bottle (or any 500 ml single-serve PET water bottle 6.0 to
6.6 cm across and 20 to 21.5 cm tall; no maker's drawing is published, so measure it), a 40 x 40 cm felt desk pad (or
a 3 mm felt sheet), masking tape, a kitchen scale, a marker, and a phone that films at 240 fps.

**Preparation.** Empty the bottle, dry it, and weigh it with its cap (record it; ours is modelled at 15 g). Fill it
with water to 185 g total (170 ml; adjust for your empty mass). Draw a ring at the water line, 6.2 cm above the base,
so you can check it each trial. Tape the pad flat with its centre 36 cm in front of the arm bases and 16 cm to the
right of their midpoint. Tape a 6.2 cm circle on it with its centre at x 30, y -6, and the distractors' footprints.

**Rig safety.** The bottle is capped, so no water reaches the arms. It weighs 185 g and comes down from about 1.3 m at
most, onto felt. Keep people out of a 1.5 m radius during the throw. If the cap leaks or the bottle is dented, replace
it.

**Measurement.** Film from the side, level with the pad, at 240 fps, 2 m away. In the frames, find the release (the last
frame in contact with a finger) and the first touchdown. Count the bottle's turn between them from the cap's direction
frame by frame, as the simulator does. A full turn is about 360 degrees and a half flip 180, so the 240-degree line is
easy to call. The touchdown frame shows whether the base touched first and the axis was within 45 degrees of upright.
For full credit, the video must show the bottle standing still for 2 s with the gripper at least 5 cm away. Then an
overhead photo with a protractor checks the axis (10 degrees) and the base's position on the pad; a feeler of 5 mm
checks it is flat on the pad.

**Reset (under a minute).** Check the water line (a capped bottle loses none), dry the bottle, and stand it on its
taped circle. Pick up any distractor that was knocked and put it on its footprint.

## Model run

Dream-1 was attempted once on the final version, with the instruction above and a 1,200-step limit. Its API refused
the request before the first action, so nothing was scored and there is no model video: "DreamAPI refused the
inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Flipping a bottle
involves throwing/tossing objects uncontrolled on the tabletop." The bottle is capped, weighs 185 g and lands on a
felt pad; the rig safety notes below keep people out of its path.

## Run and score a trial

Requirements: [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena with Isaac
Sim, Linux and an NVIDIA RTX GPU, and network access the first time to download the three SimReady distractor props
into `~/.cache/simready`.

Model run (Dream-1; needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/water-bottle-flip-to-upright/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Flip the water bottle so that it lands standing upright on the desk pad." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/bottle-flip-dream
```

The scripted demo is closed-loop and runs through `demokit.py`, from this folder, in the simulator's Python
environment (`source ~/IsaacLab-Arena/yam.env`):

```bash
cd tasks/water-bottle-flip-to-upright
python demokit.py task.py ~/yam-eval/bottle-flip/demo --mode full                         # writes full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/bottle-flip/checks --no-cameras --mode none,place,drop,half,hard,early   # the failure checks below
```

Runtime: the demo ends about 11 s into the trial (222 steps, with the 2 s hold); wall time was 3 to 5 minutes on a
shared RTX-class GPU, about a minute on an idle one. No seeds: the layout is fixed (`object_jitter=0`) and the demo
reads the bottle's position before it grasps. Each step's physics is the simulator's own. The final demo was run on
the GPU; the CPU physics path (`--fast`) gives a slightly different throw, as it does for any tuned throw.

### The scripted demo

Right arm only; the left arm keeps its ready pose.

1. The gripper comes in level, pointing to the robot's left (+y), and closes across the cap (front to back) with
   40 N.
2. It lifts the bottle 6 cm, then carries it to x 28, y -13, 42 cm above the table, still hanging cap up, and squeezes
   to 70 N.
3. Wind-up (1.2 s): the elbow drops 0.4 rad and the wrist pitches back 1.2 rad, so the base swings back and up.
4. The flick (in joint space, at 20 Hz): the wrist pitch target jumps to its upper limit, while the elbow rises at
   5.5 rad/s and the wrist roll turns at 3.25 rad/s. The base swings down, forward and up. The gripper opens on the
   fifth step, when the bottle is about horizontal and moving straight up at about 3.5 m/s.
5. The arm goes back to its ready pose in joint space, clear of the bottle's path. The bottle rises to about 1.2 m
   above the table, turns once, slows as its water moves to the cap end, lands base first on the pad and stands.

Results, from the final code on the GPU physics: the demo scored 1.0 in two fresh recorded runs (222 steps each; the
bottle turned 264 degrees from the hand and touched down 3 degrees from upright at x 43, y -22). The failure checks
(`--mode`, same throw unless stated):

| Mode | What it does | Score | Why |
|---|---|---|---|
| `none` | nothing | 0 | no flight |
| `place` | lifts the bottle and sets it down upright on the pad | 0 | no flight; it stands, but was never flipped |
| `drop` | lets go of it upright 15 cm above the pad | 0 | turned 18 degrees, then fell over |
| `half` | turns it cap down 25 cm up and lets go (a half flip) | 0 | turned 69 degrees from the hand (the rest was in the hand) |
| `hard` | the wrist roll twice as fast (a careless flick) | 0.25 | flipped (281 degrees) and over-rotated; came down on the table behind the arms, off the pad |
| `early` | lets go one step (50 ms) early | 0 | left moving forward and fell off the table, 1.8 m out |

## Fidelity and what is simplified

- The water is two sliding masses on the bottle's axis, damped. It reproduces the spin-down that makes the flip work
  (calibrated to the paper's ratio) but not sloshing across the bottle, splashing or the water's own swirl.
- The bottle is rigid with a flat base: no crumpling of the thin PET and no feet. Restitution is 0.
- Friction values are estimates. The pad is a rigid box.
- The gripper force (40 N to carry, 70 N to throw) is the simulator's setting for a firm pinch on the ribbed cap.
- The robot's arm speeds are within the simulated YAM's joint limits (wrist joints up to 20 rad/s, elbow 5.5 rad/s),
  and every joint target stays 0.02 rad inside its range limit, so the joint position control slows each joint before
  its stop.
  The real arm's controller and gripper speed (about 0.15 m/s per finger) should be checked on the real robot.

## Assets and licences

- `mechkit.py` is copied from `tasks/heat-shrink-lug/mechkit.py` in this repository (another team's open-source
  task), unchanged apart from a credit line in its docstring, and `demokit.py` is the closed-loop demo player from the
  same folder, unchanged. `simready.py` is adapted from the SimReady loader in `tasks/cutters-off-pegboard/`.
- The bottle and pad are built in `task.py` from the dimensions above: the bottle needs moving water inside it, so it
  is a custom articulation rather than a SimReady prop. Its size is that of SimReady's 500 ml bottle
  `food_beverage_bottledWater_d01` (6.2 x 20.3 cm), which is not loaded. No third-party geometry is used.
- Distractors, from NVIDIA's SimReady library (`nvidia/simready-assets` on Hugging Face, revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`), © NVIDIA, CC-BY 4.0. They download at first run and nothing is
  committed here. At load time each is given the mass stated above and is a single loose rigid body (the apple with a
  convex collider).
  - [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01) (notebook);
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) (mug);
  - [`food_produce_apple_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_apple_b01) (apple).
