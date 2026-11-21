# Switch on a banker's desk lamp by its pull chain: catch the swinging bell pull, one straight pull down past the click

A banker's desk lamp (brass base and stem, green glass shade, 38 cm tall, 1.6 kg) stands switched off, loose on the
table. Its bulb socket is a pull-chain socket: a brass ball chain hangs from under the shade with a small brass bell pull
on its end. The instruction asks for the lamp to be switched on and left standing where it is.

Many table and desk lamps, the green-shaded banker's lamp among them, are switched this way: one pull on the chain
turns the light on, the next turns it off. Replacement sockets for lamps are sold the same way; a lamp-parts retailer
([1000Bulbs][bulbs]) lists an on-off pull-chain medium-base socket (660 W, 250 V, 1.25 in. across) "With a 6.3-inch
chain on the side for easy on/off performance". Pulled straight down until the switch clicks and let back, the chain
turns the lamp on; a pull that stops short does nothing; a second full pull turns it off again.

## The hand skill

Most people have done it; it is a small, quick gesture, and the robot has to do it the same way:

- The pull is small and hangs free: a brass bell 11 mm across and 16 mm long on 128 mm of 2.4 mm ball chain, hanging
  under the shade 16-18 cm above the desk, over the lamp's base, 6 cm behind the shade's front lip. It swings away when
  bumped. The hand has to come in under the shade's lip and close on the bell (or the chain) without knocking it away.
- The switch needs 3 N to start and 6 N at its click, 10 mm down (6.9 N at its stop, 13 mm down). A pull that stops
  short of the click does nothing; a second full pull turns the lamp off again: exactly one full pull is wanted.
- The pull has to go straight down along the chain. More than 30 degrees off, the chain binds on the socket's bushing
  and the switch does not turn. The lamp is loose: pulled out toward the robot it lifts onto its front edge (in the
  wrong-way run below it tilted 13 degrees), and yanked harder it falls over.
- Then the chain is let back up and released, and the lamp is left standing where it was.

One hand is enough, as for a person; the other arm is free to steady the lamp but does not have to. The task tests
catching and pulling a small hanging pull by the right amount and in the right direction, once.

The task is the task file [`switch_on_a_lamp_by_its_pull_chain.py`](switch_on_a_lamp_by_its_pull_chain.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the lamp, the switch and the chain are built in these files from boxes, cylinders and spheres.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint. The lamp
(one loose body of many shapes), the switch's plunger (a body on a sprung slide joint inside the socket) and the chain
(five bodies on ball joints) are built with `bodykit.py`, which adds such bodies and joints: importing it hooks
`yam_arena.scene` when the simulator starts, without changing yam_arena, and gives the task file each body's pose. The
task file switches the lamp's glow by setting the emissive colour of the shade's and bulb's materials (USD
`UsdPreviewSurface` inputs) when the switch clicks.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Banker's desk lamp, loose, 1.6 kg in all, 38 cm tall. Brass base 13 x 22 cm (long side along y), 2.6 cm tall in two
  steps, felt underneath, centred at x 31, y 0. Brass stem 18 mm across at x 36 rising to 37 cm, with a yoke forward
  over the shade.
- Shade: green glass, half a cylinder with the open side down, radius 6.5 cm, 25 cm long along y, axis at x 29 and
  30 cm up (front lip at x 22.5, back lip at x 35.5, both 30 cm up), brass end caps.
- Socket: a brass shell 35 mm across and 57 mm long, its axis along y at x 28.5, 32.5 cm up, from y 2.2 to 7.8; a 60 mm
  (A19) bulb on it toward -y.
- Chain: leaves the socket through a bushing at x 28.5, y 6.0, 30.75 cm up, and hangs straight down: 128 mm of brass
  ball chain (2.4 mm beads at 3.2 mm pitch), then the brass bell pull (11 mm across, 16 mm long), its middle 17.05 cm up
  and its bottom 16.25 cm up.
- The lamp is off. Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled:

- The switch: the chain's top end is a plunger inside the socket on a slide (straight up and down) with 0 to 13 mm of
  travel and a spring (300 N/m, preloaded to 3.0 N against its top stop: 6.0 N at 10 mm, 6.9 N at 13 mm; estimates).
  The click is a rule on its travel, read every control step: reaching 10 mm toggles the lamp once; it acts again only
  after the plunger has come back above 3 mm.
- Binding: if the top length of chain is more than 30 degrees off the lamp's vertical when the plunger reaches 10 mm,
  the chain is bound on the bushing's rim: no click, and that pull is spent (the plunger must come back above 3 mm
  first). The top length is stopped at 50 degrees by the bushing's rim (a joint limit).
- The chain: five rigid lengths of 25.6 mm, each 8 beads, hung one from the next by ball joints (free swinging, stopped
  at 80 degrees); 2 g per length, 8 g for the last one with the bell. A real chain bends at every bead and weighs less
  (about 1 g in all, the bell 3-6 g): the lengths and masses are what the simulator handles stably when a gripper
  squeezes the bell.
- The light: when the lamp is on, the shade's glass and the bulb glow (their materials' emissive colour is switched on).
- Not modelled: the wiring and the switch's inner cam; the click's sound; the bulb's heat.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the poses of the lamp, the plunger and the chain lengths and the points of each
finger's collision hulls, and applies the click and binding rules.

- 0 if the lamp ever tips over: its base tilted more than 25 degrees at any moment (latched for the episode).
- 1.0 (success, held 1 s): the lamp is on after exactly one click, the chain is let go (every finger point 1 cm or more
  from the chain and bell, and the plunger back within 2 mm of its top), the lamp stands upright (under 5 degrees),
  within 2 cm of where it started, and it is still (moving less than 1 mm in 1 s).
- 0.5: the lamp is on otherwise (three or more clicks, the chain still held at the end, or the lamp moved more than
  2 cm).
- 0.3: the lamp is off after two or more clicks (it was switched on and off again).
- 0: it never clicked.

A real trial is scored from the trial video with sound: the lamp's light at the end, the clicks heard (each also
toggles the light), whether the lamp tipped, whether the gripper let go, and the base against tape marks at its
corners (2 cm).

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=switch_on_a_lamp_by_its_pull_chain.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Switch the desk lamp on and leave it standing where it is." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pchain-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The left gripper, level and pointing
forward, fingers open across y, comes in under the shade's front lip at the bell's height so that the bell hangs
between its open fingers, and closes on the bell (a 6 mm commanded gap on the 11 mm bell: the fingers stop on it at
about 11 mm and squeeze). It pulls straight down, commanded 50 mm over 2.5 s: the level arm, soft at its wrist, follows
about 19 mm against the switch's 3-7 N, which takes the plunger to its 13 mm stop; the click comes at step 255. It holds
0.6 s, rises back to where it caught the bell (the chain goes back up), opens and backs out. The right arm stays at its
ready pose. It reaches success at step 336 (16.8 s).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=switch_on_a_lamp_by_its_pull_chain.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Switch the desk lamp on and leave it standing where it is." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/pchain-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pchain-dream/*.json; do inspect-robots video "$log"; done
```

`PCHAIN_DEBUG=1` prints, every 5 steps (`PCHAIN_DEBUG_EVERY=1` for every step), the plunger's travel and its peak, the
top chain length's angle, whether the lamp is on, the clicks and bound pulls, the lamp's position, tilt and how far it
has moved, the bell's position, how close the nearest finger is, whether the lamp is still and the chain let go, the
score, the last events, and where each gripper is and how far open.

Two wrong ways (`PCHAIN_WRONG`):

- `PCHAIN_WRONG=side`: the same catch, then the chain is yanked 8 cm out toward the robot and 3 cm down. The top chain
  length is 31 degrees off the lamp's vertical when the plunger reaches 10 mm: the chain binds, no click; the lamp
  lifts onto its front edge (13 degrees) and drops back. The lamp stays off: score 0.
- `PCHAIN_WRONG=double`: the same straight pull, done twice. Two clicks: on, then off again. Score 0.3.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: a Dream-1 trial of 2,400 steps
at 320 x 176 took about 6 minutes from start to finish; the scripted demo while rendering 1280 x 720 video reached
success at step 336; debug runs of the demo at 320 x 176 without video took 4 to 6 minutes from start to finish
(several runs at once on the GPU). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A banker's desk lamp with a pull-chain socket and a bell or bead pull on the chain (or any desk lamp of about this
size whose chain hangs below the shade 15-18 cm above the desk), with a working bulb; a LED bulb keeps the shade cool.
Its base is placed with its middle 31 cm in front of the midpoint between the arm bases, the chain at the front left
under the shade, and tape marks at the base's four corners. Before each trial switch it off (one pull) and let the chain
hang still. Record the trial with sound.

[bulbs]: https://www.1000bulbs.com/product/86042/SOCK-801052.html
