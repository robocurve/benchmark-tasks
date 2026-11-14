# Put one eye drop in a care-training manikin's right eye with the lower lid drawn down and the tip off the face

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). An adult-size care-training
head lies back on a gel pad, face up. The robot picks a 15 mL eye-drop bottle off the medication tray, draws the right
lower lid down with one hand on the cheek, holds the bottle tip-down about 2 cm above the pocket without touching the
face, squeezes out one drop and puts the bottle back on the tray. The setup, scoring and fidelity notes are in
`task.json`.

## Run a trial

Scripted demo (reaches full score):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/instill-one-eye-drop/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put one drop of the eye medicine in the manikin's right eye, then put the bottle back on the tray." \
  --scorer task_score --max-steps 650 --store-frames
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400` (2,400 steps = 2 minutes at 20 Hz).

Render the videos of a run with `inspect-robots video <log.json>`. Offline checks (no simulator):
`python3 test_task.py` (runs pytest; 47 checks).

## Scoring

`--scorer task_score` gives 1.0 when the task succeeds (the full condition held for 1 s) and otherwise `score()` at
the end of the trial. The rules, with their reasons:

- **How a drop forms** (dropkit, below). Liquid moves only while the bottle points at least 30 degrees below
  horizontal and its walls are squeezed with more than 4 N; a lighter hold releases nothing. Above that it flows into
  a drop hanging at the tip at 50 uL/s per N above 4 N. At 40 uL the drop detaches and falls straight down. So a 5 N
  squeeze (2.5 mm) makes one drop after 0.8 s and a second after 1.6 s; 8 N makes five drops in a second. Easing off
  below 4 N before the drop detaches draws it back into the bottle.
- **Where it lands.** The fall line is traced to the first surface it meets: a hand, the head's skin (its exact
  shape, `manikin.py`), the tray or the table. On the head it is in the **pocket** when it lands in the strip the
  drawn-down lower lid uncovers below the eye opening (2.6 cm wide), with the lid drawn down at least 3 mm (less than
  that leaves no pocket to hold it); in the **eye** when it lands in the 3.0 x 1.0 cm eye opening (or that strip with
  the lid drawn under 3 mm); anything else is a miss, including the left eye.
- **Latched 0.** At any moment of the trial:
  - the nozzle touches the skin (eye, lids, lashes or face): its rounded end or the last 7 mm of its cone within
    0.5 mm of the visible surface, the contact solver's resting gap. A touched tip carries germs from the eye back into
    the bottle and the next dose. Real trial: washable marker ink on the nozzle leaves a mark on whatever it touched.
  - a hand touches either eye opening (guardkit's contact check of the hand's own shape, within 2 mm, not a distance
    zone): a finger in the eye. Real trial: the side and top video.
  - a strike: the right eye pushed 6 mm out of place for 0.1 s (two steps), or 9 mm at any single step. The head sits
    on a sprung neck: 1 N straight into the face at the eye moves it 1.5 mm, so 6 mm is about 4 N and 9 mm about 6 N,
    a push that moves a person's head. A fingertip drawing the lid down presses well under 1 N. The two-step rule keeps
    a single contact-solver kick from counting. Real trial: a light pointer taped to the forehead in front of a 1 mm
    grid card, read on the side video.
  - the bottle is dropped on the table: held by no hand, it falls faster than 0.5 m/s (a free fall of 1.3 cm) within
    0.25 s of coming to lie off the tray (its center under 1.8 cm): a dropped bottle is discarded. A bottle laid on the
    table by hand is not a drop; it only keeps the trial from full credit until it is back on the tray.
- **Tiers.** 0 if no drop landed in the right eye; doing nothing scores 0. 0.25 if a drop landed in the right eye
  (some medicine reached it). 0.5 if the first drop landed in the right eye, at most two drops were released, and each
  was released with the tip no more than 3 cm above where it landed: guides say to hold the tip about 2 cm above the
  eye; from higher the drop is hard to aim and its impact makes the patient blink. 0.75 if exactly one drop was
  released and it landed in the pocket: the pocket holds the drop against the eye, while one on the eye itself starts
  a blink that squeezes it out, and a second drop overflows the eye (it holds less than one drop). 1.0 if, on top of
  that, the bottle lies on the tray, no hand touches it (guardkit's contact check), it is still (under 0.5 mm per step)
  and the lid is back within 1 mm of closed, all for 1 s: the dose is over, the lid is released and the bottle is put
  away where it does not roll or get knocked. The arm's pose is not scored. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.
  Each threshold is defined once in `task.py` (`TIP_TOUCH`, `MAX_TIP_H`, `POCKET_MIN`, `STRIKE_MM`, `JAB_MM`,
  `STRIKE_STEPS`, `ON_TABLE_Z`, `FALL_V`, `FALL_STEPS`, `STILL_MM`, `LID_LET_GO`, `HOLD_STEPS`) and in the bottle
  spec (`open_force`, `flow`, `drop_ul`, `tip_down_deg`).

In a real trial: the bottle holds water with blue food coloring; a strip of white absorbent paper lies in the lower-lid
pocket and white tissue on the cheek, so each drop leaves a blue mark where it landed; the bottle is weighed before and
after on a 0.01 g scale (one drop is about 0.04 g) to count the drops; a side camera at 60 fps with a 1 cm grid card
behind the head gives the tip's height at each drop and how far the lid was drawn; the nozzle's ink shows any touch;
the forehead pointer shows a strike.

## How the drops work (dropkit)

There is no simulated fluid. `dropkit.py` keeps a ledger each control step from the bottle's own state:

- The bottle is a yamkit `Mechanism`: a rigid core and four wall pads in two opposite pairs (along the bottle's y and
  z axes, 90 degrees apart, so a grasp from any usual side closes on a pair), each sprung back at 2 N per mm
  (`mechkit.Spring`, a PhysX drive). The squeeze force is 2 N times the larger pair's mean travel in mm. The real bottle's walls are measured the same way: a push-pull gauge (or a luggage
  scale and a ruler) on the two walls at mid-body.
- Opening force 4 N, flow 50 uL/s per N above it, drop 40 uL: calibrate on the real bottle tip-down over a 0.01 g
  scale. Squeeze with the gauge at a steady force and time the first drop (0.8 s at 5 N in the model); the force at
  which a drop never forms in 10 s is the opening force. Measured forces to express one drop from common glaucoma
  eye-drop bottles are about 3 to 8 N (Connor and Severn, Eye, 2011), so 4 N is in the range of real bottles.
  Commercial eye drops are 30 to 50 uL; 40 uL is the middle.
- Tip-down rule: with the tip less than 30 degrees below horizontal the liquid is not at the nozzle, so a squeeze
  moves air only.
- Each drop is traced straight down (`land()` in task.py) and logged with its zone, the tip's height above where it
  landed and how far the lid was drawn. The drops are drawn as blue beads (Isaac Lab markers); the drawing never
  affects the score.

What it leaves out: the drop's swing and the splash, the bottle's own suck-back speed (a hanging drop is drawn back at
once), and the real wall's give anywhere around the round body (only the four pads give, and a grasp across the pads' corners, 45 degrees between pairs, meets the
rigid core).

## Real-world build

- Manikin head: an adult-size vinyl care-training head with lifelike eyes for eye-drop practice, for example the
  head of a nursing-care patient simulator (Gaumard's Simple Simon nursing simulator lists eyes for ophthalmic
  exercises), 15 cm wide. Mount it on a sprung neck: a spring hinge, or a short stiff compression spring
  standing on a base board, under the back of the jaw, so the head lies back 60 degrees with the right eye 18.9 cm
  above the table and 1 N on a luggage scale pushed straight into the face at the right eye moves it 1.5 mm (set it on
  the assembled head; the reading then includes its weight). A blue gel positioning pad (20 x 18 x 1 cm) under the
  back of the head.
- Lower lid: if the head's lid is not soft, cut a lower lid from 2 mm platinum-cure silicone sheet, 2.8 cm wide, and
  glue it along its lower edge 2.5 cm down the cheek, so a fingertip on the cheek patch draws its margin down; check
  that 0.8 N on a luggage scale draws it 8 mm.
- Bottle: a 15 mL LDPE drop-dispensing bottle with a dropper tip (lab-supply bottles of this size are about 2.5 cm
  across and 7.5 cm tall with the cap on), 10 mL of water with blue food coloring, 17 g. It weighs less than most
  objects the arms pick up, and that is the real object: it is held by squeezing, not by friction under its weight.
- Medication tray: white plastic, 22 x 14 cm. Bottle rest: a white foam block 8 x 5 x 1 cm with a 2.6 cm groove so
  the round bottle cannot roll. Cap on the tray.
- Absorbent paper strip, white tissue, washable marker, a 0.01 g pocket scale, a 1 cm grid card, a side camera.
- Safety: water, a soft plastic bottle and a vinyl head on a sprung neck; nothing sharp, hot or heavy.

## Runtime and seeds

- Model run limit: 2,400 steps (2 minutes at 20 Hz). The scripted demo plan lasts 24.8 s (496 steps): the right
  hand picks the bottle off its rest (fingers closing across it), lifts it 14 cm, turns it tip-down 8 cm above the eye
  (tip leaning 35 degrees from vertical toward the crown), lowers the tip to 2 cm above the pocket, waits for the
  lid, presses the walls to their stop for 0.1 s between a 0.1 s press and release (about 56 uL: one drop), lifts
  clear and lays the bottle back in its rest. The left hand closes into a fist and comes to the cheek patch from the
  chin and temple side, stopping where its fingertips first touch the patch. It then presses 7 mm into the skin
  along the patch's normal, so that friction grips the patch (at 5 mm the fist slid and drew the lid only 3.0 mm),
  draws it 7 mm toward the chin, holds it until after the squeeze and lifts off along the normal, letting the lid
  spring back. The success condition then needs 1 s held. In the simulator the demo scores 1.0 at step 491.
- No randomness: the layout is fixed (`-E object_jitter=0`).
- Offline, every demo pose solves with the arm's own IK (under 0.2 mm), and the arms' collision meshes keep the two
  hands 2.6 cm apart, the fist 1.0 cm from the bottle and the bottle at least 4 mm from the skin. The fist's contact
  is all on the cheek patch: its fingertips' convex hulls reach none of the head's other colliders and neither eye
  opening.

## How the file works

- **The manikin** is a yamkit `Mechanism` tilted back 60 degrees, with a fixed mount and three links: `tilt` (side to
  side) and `head` (nodding) on hinges through the neck point, each with a mechkit `Spring` (9.64 and 16.3 N·m/rad,
  which is 2.0 and 1.5 mm per N at the right eye), and `lid`, the right lower lid and cheek patch on a slide down the
  face (0 to 8 mm, 0.1 N per mm). Gravity is off for all three (`floating`): the real neck is set on the assembled head.
  The head's mesh is traced from `manikin.py`'s signed-distance shape (skull, jaw, cheeks, brow, nose, eye sockets and
  eyeballs at adult sizes); its colliders are seven boxes inside the skin (0.4 to 11 mm under it).
- **The scorer** keeps one record per trial (yamkit's episode counter): the touch, eye, strike and fall latches, the
  bottle's last position and the count of steps with the end state met. dropkit keeps the drop ledger.
- **Contact checks:** the nozzle against the skin's exact surface (the lid's position included); the hands against the
  eyes and the bottle with guardkit's `touching()`; the strike from the head's pose.

## Files

- `task.py`: the scene, `score()`, `success()` and the scripted `demo()`.
- `dropkit.py`: the task's helper module for the dropper bottle (squeeze force, flow and drops).
- `manikin.py`: the task's helper module for the head's shape.
- `yamkit.py`, `mechkit.py`, `guardkit.py`: the task's helper modules that spawn the modeled props and mechanisms, set their springs, and check gripper contact.
- `test_task.py`: offline checks of the scorer, the drop model and the demo plan.
