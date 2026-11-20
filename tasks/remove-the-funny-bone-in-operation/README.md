# Play a turn of Operation: tweezer the Funny Bone out of Cavity Sam without a buzz and lay it on its card

Operation is a family board game nearly everyone has played: a patient, Cavity Sam, lies on the board with small
plastic "ailments" in openings edged with metal, and each player in turn lifts one out with a pair of tweezers wired
to the board. Touch the metal edge and the circuit closes: the buzzer sounds and Sam's red nose lights up. Here one
turn is played: the Funny Bone lies in its opening at Sam's elbow, the game's tweezers stand tips down in a tweezer
stand beside the board, and the Funny Bone's ailment card lies face up on the table. The instruction asks for the
Funny Bone to be taken out with the tweezers without getting buzzed, laid on its card, and the tweezers put down; it
does not say how.

The task is the task file [`remove_the_funny_bone_in_operation.py`](remove_the_funny_bone_in_operation.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## The rules and the hand skill

From the game guide of the Classic Operation game (Hasbro B2176, [instructions.hasbro.com][guide]):

- "Use the tweezers to remove the ailment without touching the metal sides."
- "If you touch the metal sides, you'll get buzzed, and Cavity Sam's nose will light up! If this happens, leave the
  ailment in its space, and pass the ailment card you drew to the player on your left."
- "On your turn, if you remove the ailment without touching the metal sides, you get to keep the card and the
  ailment! Place the card in front of you money-side up." "Place the tweezers where all players can reach them."

The tweezers and the metal edge of every opening are the two ends of a battery circuit, so the game itself judges the
hand: any touch, however light, is a buzz.

What is hard is the hand skill. The gripper does not pick the bone up itself (its fingers are far too thick for the
opening); it holds the light, sprung tweezers by their upper arms and works them through its fingers: squeezing shuts
the tips, easing off lets the tweezers' own spring open them. Opened too wide, the tips straddle the steel edge;
opened too little, they land on the bone. The opening is only 3.5 mm wider than the bone on every side and each tip
is 1 mm thick, so with the tips open to about 9 mm each tip clears the steel by about 0.9 mm and the bone by about
1.6 mm: the tips must come straight down, centred to about a millimetre, 65 mm below the fingers that hold them,
pinch the 2 g bone across its 6 mm shaft and lift it straight up and out without swinging. Then the bone is laid on
its card and the tweezers put back. One arm does it; the other is not needed (the board is taped down).

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on
Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs everything, including the
Dream-1 policy. The task needs no models from Arena's object library and uses no third-party assets: the board, the
Funny Bone, the tweezers, the stand and the card are built in these files from boxes and cylinders.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a hinge. The
board (a block with the bone-shaped opening, its steel edge band and plastic pocket), the bone, the stand and the
card are built with `bodykit.py`, and the tweezers as a small loose articulation: a joined end with two arms on
sprung hinges. `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives
the task file each body's pose and the hinges' angles, and turns on PhysX contact reporting for the two tweezer arms,
so the task file reads the contact force between each arm and the steel edge every control step: that is the
game's circuit.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left; heights above the table top.

- Classic Operation game board (Hasbro B2176; 25.1 x 40.0 x 4.1 cm and 0.45 kg, its retail dimensions), taped to the
  table: x 11.95 to 37.05, y -18.0 to 22.0. Cavity Sam is printed on it lying along y, head at +y, his red nose at
  (24.5, 17.0); the other ailments' openings are painted on (only the Funny Bone's is modelled).
- The Funny Bone's opening at Sam's near elbow, centred at (16.5, -0.5), its long axis along y: the bone's outline
  grown by 3.5 mm all round (39 mm long, 13 mm wide along the shaft, 20 mm wide at the knuckle ends), edged by a steel
  band 3 mm wide and 1 mm thick flush with the board's top (the circuit), over a plastic pocket of the same outline,
  6.5 mm deep.
- Funny Bone, loose, white plastic, 2 g: a 6 x 18 mm shaft between two knuckle ends of two 7 mm round lobes each;
  33 mm long, 13 mm across the ends, 5 mm thick; lying flat in its pocket, centred in the opening.
- The game's tweezers, loose, steel, 14 g, 11.0 cm long: a joined end (7 x 4 x 10 mm, with the red sleeve of the
  wire) and two arms 1 mm thick, 7 mm wide tapering to 3 mm at the tips, on hinges at the joined end, each with a
  spring of 2 N m/rad (1.2 N at each tip to shut them); the tips 12 mm apart inside when free. They stand tips down,
  arms apart along x, in the slot of a tweezer stand, their top 11.6 cm above the table.
- Tweezer stand, loose, pine, 5.0 (x) x 4.0 (y) x 3.5 cm, 45 g, centred at (15.5, -23.5), with a 22 (x) x 9 (y) mm
  slot 30 mm deep.
- Funny Bone ailment card, loose, 8.8 (x) x 6.3 (y) cm, 1 mm, face up, centred at (29.0, -23.5).
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the contact force between each tweezer arm and the steel edge band (PhysX pair-wise
contact forces), the poses of the tweezers' parts, the bone and the card, and the points of each finger's collision
hulls.

The scorer follows the game's own rule ("If you touch the metal sides, you'll get buzzed ... leave the ailment in its
space"): a buzz during the removal loses the turn.

- A buzz: either tweezer arm touches the steel band with 0.05 N or more. Sam's nose lights up (a bright red disc laid
  over it) while a touch lasts.
- The bone is clear of the opening once its middle has left the opening's outline or its lowest point has risen 2 mm
  above the board's top. It counts as out at the end unless it is back in its pocket (middle inside the outline,
  lowest point below the top).
- 0 for the rest of the run: a buzz while the bone is not clear of the opening (still in it, or being lifted through
  it). A touch once the bone is clear of the opening does not count. Nothing else ends the run early.
- 0: the bone is not out at the end (never taken out, or dropped back in).
- 1.0 (success, held 1 s): the bone lies on its card (its middle over the card, its lowest point within 3 mm of the
  card's top), the tweezers are put down (every finger point 15 mm or more from them), and the tweezers and the bone
  are still (under 1 mm in 1 s).
- 0.3: the bone is out without a buzz but the turn is not finished (it lies on the board or the table, it is still in
  the tweezers, or the tweezers are still held).

Doing nothing scores 0. A real trial is scored from the video with sound: the game's own buzzer and nose light show
whether it buzzed before the bone was clear of the opening; the card and the tweezers are checked in the final frame.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=remove_the_funny_bone_in_operation.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Take the Funny Bone out of Cavity Sam with the game's tweezers without getting buzzed, lay it on its ailment card, and put the tweezers down." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/operation-dream
```

The scripted demo, which reaches full score (open loop from the known layout; the left arm stays at the ready pose):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=remove_the_funny_bone_in_operation.py -E object_jitter=0 -E control_hz=20 \
  --instruction "scripted demo" --scorer task_score --max-steps 1100 --store-frames --log-dir ~/yam-eval/operation-demo
```

The right gripper, fingers closing along x across the tweezers' arms, comes down round the tweezers standing in their
stand and squeezes them shut 45 mm below their top, lifts them out and carries them over the opening. With the shut
tips 25 mm above the board it eases its fingers (commanded 7.6 mm apart) so the tweezers' spring opens the tips to
about 9 mm, lowers them straight down in 4 s on either side of the bone's shaft until their ends are 1.2 mm above the
pocket's floor, squeezes (the tips pinch the 6 mm shaft), lifts the bone 40 mm straight up, carries it over its card,
lowers it to 6 mm above the card and eases the fingers so the bone drops onto the card. It squeezes the tweezers shut
again, carries them back, lowers them into the stand's slot, lets go and rises clear.

To turn every frame into a video: `cd <log-dir> && inspect-robots video <log>.json`.

## Debugging and the wrong ways

- `OPER_DEBUG=1` prints a `DBG` line every 5 control steps: the contact force on the steel edge, the buzz count and whether the turn is lost, the
  tips' middle relative to the opening, the tips' height, their gap and the hinge angles, the bone's position and
  state, and both TCPs and finger gaps.
- `OPER_MODE=sloppy` runs the same demo with the tips aimed `OPER_SLOP` mm (default 2.5) off centre across the
  shaft: one tip lands on the steel band (a buzz with the bone still in the opening, up to 0.75 N: the turn is lost),
  the tips then shut without the bone between them, and the bone stays in its pocket: score 0. Aimed 1.5 mm or
  1.0 mm off (`OPER_SLOP=1.5`, `OPER_SLOP=1.0`) the same happens (one buzz each, up to 0.50 and 0.45 N): score 0. The scripted demo's tips came down 0.6 mm off centre.
- `OPER_MODE=fingers` leaves the tweezers in their stand and tries to pinch the bone out with the gripper's own
  fingers: they do not fit the opening, press on the board round it, and the bone stays in its pocket: score 0 (the
  robot's fingers are not wired, so they do not buzz; the rules also say to use the tweezers).
- `OPER_AIM=dx,dy` (mm) shifts where the tips are aimed; `OPER_EASE`, `OPER_SHUT` set the finger commands.

## Runtime per trial

The scripted demo reaches full score after about 1,000 control steps (50 s simulated at 20 Hz). On one NVIDIA RTX PRO
6000 Blackwell GPU a debug run of it (320 x 176 cameras, no video) took about 4.5 minutes of wall-clock time, Isaac Sim
start-up included; the Dream-1 run of 2,400 steps (320 x 176) took about 13 minutes, sharing the GPU with another run.

## Seeds

The scene has no randomness (`object_jitter=0`): every trial starts from the same layout, so a seed changes nothing.

## Building it for real

A Classic Operation game (Hasbro B2176) with fresh batteries, taped to the table so its near edge is 11.95 cm in front
of the arm bases and centred 2.0 cm to the robot's left, Cavity Sam's head to the left. A small wooden block with a
22 x 9 mm slot 30 mm deep (any tweezer stand that holds the tweezers upright) 23.5 cm to the right, 15.5 cm forward;
the Funny Bone card beside it. Put the Funny Bone in its opening, stand the tweezers tips down in the slot with their
arms apart front to back, and record each trial with sound. The buzzer and the nose light count the buzzes; nothing
else is needed.

[guide]: https://instructions.hasbro.com/en-us/instruction/classic-operation-game
