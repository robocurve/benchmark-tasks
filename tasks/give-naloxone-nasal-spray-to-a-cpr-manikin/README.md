# Give a CPR manikin naloxone nasal spray: head held tilted back, nozzle in a nostril up to the flange, one full press

An adult CPR training manikin lies on its back on the table, the top of its head toward the robot. To the robot's
right, a naloxone nasal spray training device (the size and shape of the 4 mg NARCAN Nasal Spray) lies in an open
holder. The instruction is only: "Give the CPR training manikin one dose from the naloxone nasal spray training
device in the holder."

## Why it matters

About 80,000 people in the US died of opioid overdoses in 2023 (CDC, provisional drug overdose death counts).
Naloxone reverses an opioid overdose, and in March 2023 the FDA approved NARCAN Nasal Spray for sale over the counter
so that bystanders (family, friends, staff of shelters, libraries and schools) can give it before an ambulance
arrives. Public-health programmes train lay people on CPR manikins with the reusable training device. A robot that
does it right gives the dose the way the package says; done wrong, the spray goes onto the face or only into the
front of the nostril, or the head flops back to level.

The procedure is the package's own (NARCAN Nasal Spray Quick Start Guide, printed in every box and in the
[prescribing information](https://dailymed.nlm.nih.gov/dailymed/drugInfo.cfm?setid=a0ebc83d-2892-40de-bc0a-775ce90b30db)):
"Hold the NARCAN Nasal Spray with your thumb on the bottom of the plunger and your first and middle fingers on either
side of the nozzle." "Tilt the person's head back and provide support under the neck with your hand. Gently insert
the tip of the nozzle into one nostril, until your fingers on either side of the nozzle are against the bottom of the
person's nose." "Press the plunger firmly to give the dose of NARCAN Nasal Spray."

Why two hands: the device fires by a squeeze between the plunger and the finger flange (thumb against two fingers),
so the hand that holds it is the hand that fires it; a parallel gripper does the same squeeze, one finger on the
plunger's disc and one on the flange beside the nozzle. The manikin's head does not stay tilted back: its neck springs
it back to level when let go, so the other hand has to hold it tilted while the nozzle goes in and the dose is given.
With the head level the nostrils point along the face toward the top of the head, not up toward the hand.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1,
on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs everything, including the
Dream-1 policy. The task needs no models from Arena's object library and uses no third-party assets: the head, the
chest, the device and the holder are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or hinge one on another. The
head, the device with its sprung plunger and the holder are built with `bodykit.py`, which adds such bodies and
joints: importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives the task
file each body's pose. This copy lets a body report its contacts (the head's contacts with the device and with each
arm are read every step) and lets a shape collide without being drawn (the head's colliders under its skin). The
head's skin is the adult head mesh of `manikin_head.py` (a signed-distance model built from ANSUR II and NIOSH
head-and-face survey means), cached in `manikin_head_mesh.npz`.

## The scene

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.

- Manikin head: adult size (head 195 mm long, 151 mm wide, face 118 mm from nasal root to chin), lying face up with
  the top of its head toward the robot; the nasal root at (0.210, 0.000), 0.222 above the table, with the head level.
  The head is rigid (a CPR manikin's head is firm vinyl). Its drawn skin is one mesh; it collides as convex hulls of
  that skin (forehead, tip of the nose, lips, chin, cheeks), a sphere for the back of the head, and boxes that form
  the base of the nose around two nostrils.
- Nostrils: openings 12 x 8 mm in the base of the nose, their middles 8 mm either side of the midline, 10 mm in front
  of the nasal root's plane and 47 mm below it; each runs 20 mm up into the nose (the channel's walls start 3 mm in)
  along an axis leaning 25 deg back from the face's long axis. With the head level that axis points 25 deg below
  horizontal toward the top of the head; tilted back by t, (25 + t) deg below horizontal.
- Neck: a hinge about the world y axis, 110 mm behind and 50 mm below the nasal root (head frame), with a spring of
  1.6 N m/rad preloaded 0.17 N m against the level stop: holding the head 23 deg back takes 0.81 N m (about 9 N on the
  forehead). It stops at 30 deg and springs back to level when let go.
- Chest and shoulders: a fixed block 26 x 40 x 14 cm from x 0.35 to 0.61; the neck between it and the head is drawn
  only.
- Spray device (training device, NARCAN size): nozzle 15 mm long, 6 mm across (the last 6 mm 4.5 mm across); finger
  flange 42 x 10 x 3 mm across the nozzle's base; body 16 mm across and 20 mm long; plunger with a 28 mm thumb disc,
  standing 9 mm out of the body, 5 mm stroke, held out by a spring preloaded to 15 N (16.5 N at the end of the
  stroke); 15 g. Its origin (the flange's nozzle-side face on the axis) is at (0.200, -0.120), 0.020 above the table,
  its axis along x with the nozzle toward the robot and the flange along y.
- Holder: a 70 x 50 x 4 mm plate (fixed) with a block under the device's body and a lip on either side of it.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the head's tilt, the device's pose, the plunger's travel and the contact forces on
the head from the device and from each arm (PhysX contact reports).

- A dose is given the first time the plunger reaches the end of its stroke (within 0.5 mm; on the training device,
  the click). The device holds one dose, so only that moment counts and the score is fixed from then on.
- 0 when, at that moment, the nozzle's tip is not at least 3 mm inside a nostril opening (within the opening's 12 x 8
  mm, with 0.5 mm to spare) or the nozzle's axis is more than 20 deg off that nostril's axis: the spray goes onto the
  face or the nostril's wall.
- 1.0 (success) when, in addition, (a) the device or the hand holding it pressed on the nose (0.5 N or more at some
  step in the second before the click: inserted until the flange and fingers are against the bottom of the nose),
  (b) the head was tilted back 15 deg or more from level, and (c) the other hand was on the head (1 N or more).
- 0.5 when the dose went into a nostril but (a), (b) or (c) was missing.
- No dose by the end of the run scores 0. Doing nothing scores 0.

A real trial is scored the same way: the training device clicks at the end of its stroke; a side video shows where
the nozzle is (inside a nostril, up to the flange, and its line against the nose), whether the flange or fingers
touch the bottom of the nose, the head's tilt against a protractor card standing beside the head, and whether the
other hand is on the head at the click.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=give_naloxone_nasal_spray_to_a_cpr_manikin.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Give the CPR training manikin one dose from the naloxone nasal spray training device in the holder." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/nalox-dream
```

Dream-1 scored 0.0: its right gripper reached for the spray and knocked it out of its holder onto the table, then
hovered over it for the rest of the 2,400 steps; the left arm stayed back and no dose was given. A first wording of
the instruction ("The manikin is a person who has overdosed on opioids. Give it one dose of the naloxone nasal spray
in the holder.") was refused before acting: "DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed):
unsafe instruction in this scene, refused: Administering nasal spray to a human-like head presents a injury risk."

Scripted demo (scores 1.0):

```bash
inspect-robots run --embodiment yam_arena --policy scripted \
  -E task=give_naloxone_nasal_spray_to_a_cpr_manikin.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Give the CPR training manikin one dose from the naloxone nasal spray training device in the holder." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/nalox-demo
inspect-robots video ~/yam-eval/nalox-demo/*.json    # videos of the run
```

The scripted demo is open loop from the known layout. The left arm puts its closed fingertips on the right side of
the manikin's forehead from above and presses it down and toward the top of the head until the head is tilted back
about 23 deg, and holds it there. The right arm comes down over the device with its fingers closing along the
device's axis, one finger on the flange beside the nozzle and one on the plunger's disc (both 21.5 mm to the side of
the axis, clear of the nozzle), squeezes lightly (about 8 N, under the 15 N that moves the plunger) and lifts it,
carries it high over the face while turning it nozzle-down, lines it up with the left nostril's axis 60 mm back,
slides the nozzle in until the flange is against the bottom of the nose, squeezes harder (about 35 N): the plunger
runs to the end of its stroke and clicks. It eases off and draws the nozzle straight out; then the left hand lets the
head back down.

Environment variables: `NALOX_DEBUG=1` prints a `DBG` line every 5 steps (tilt, plunger travel, where the nozzle's tip
is, the contact forces on the head from the device, the plunger, the left arm and the right arm, both TCPs) and a
`DOSE` line at the click. `NALOX_ONE=1` is the wrong way: the left arm stays put and nobody holds the head; the right
arm does exactly the same. The head stays level, so its nostril is 4.5 cm from where the tilted nostril was and points
along the face: the nozzle lands on the bridge of the nose and the click scores 0. Lining the nozzle up with
the level head's nostril instead can earn at most 0.5 (head not tilted, not held).

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell (Server Edition) shared with other simulation jobs: the scripted demo while
rendering 1280 x 720 video took about 2 minutes (456 steps, 22.8 s of simulated time, ending on success); the
Dream-1 run at 320 x 176 took about 5 minutes for its 2,400 steps; debug runs of the demo without video took 3 to 5
minutes for 520 to 600 steps. Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## What is modelled and what is not

- Modelled: the manikin's adult head at true size with two nostrils of real size and depth, a sprung neck that returns
  the head to level, the device at the product's size with a sprung plunger that fires only at the end of its stroke
  (its 15 N preload is an estimate of a firm thumb press), the squeeze between plunger and flange that fires it, the
  contacts of the device, the hand and the other hand with the head.
- Not modelled: the spray itself (the dose is the full stroke, scored by where the nozzle is), soft skin (the head is
  rigid; real manikin heads are firm vinyl), the manikin's body below the chest, breathing, and the recovery position
  the package asks for after the dose. The device's sizes (nozzle length, flange, plunger disc) are estimates from
  product photographs; check them against the training device used.

## Building it for real

- An adult CPR manikin with a tilting head and open nostrils (for example Prestan Professional Adult or Laerdal Little
  Anne, about 150 to 300 US dollars). If its head stays where it is put, an elastic cord from the forehead to the
  chest returns it to level, as in the simulation.
- A NARCAN Nasal Spray training device (reusable, clicks at the end of its stroke; about 15 to 20 US dollars), and a
  simple holder: a 70 x 50 mm plate with a 10 mm block under the device's body and a lip either side, or any open
  tray that keeps it lying with the nozzle toward the robot and the flange across.
- A protractor card standing beside the manikin's head for reading the tilt from the side video.
