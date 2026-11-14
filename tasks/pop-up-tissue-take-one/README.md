# Take one tissue from a pop-up box and leave the next one standing ready

Instruction given to the policy: `Take one tissue out of the box and put it on the tray, leaving the box in place with the next tissue ready to pull.`

Files: `task.json`, `pop_up_tissue_take_one.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`
and an offline self-test), `softkit.py` and `mechkit.py` (helper modules, see below), `README.md`.

## Response to review

- **TK-4QRAYN: "Box should resemble a more typical layout with an oval cutout, slit along the longer axis."** The box
  is now a real flat pop-up box at full size, built the usual way. It is a 22.5 x 12.1 x 5.1 cm paperboard box
  (Kleenex Professional flat box 21400). An oval opening (8.9 x 5.7 cm) sits in the middle of its top, with its long
  axis along the box's. Clear film covers the opening, with one 8.3 cm slit along that long axis. The sources are in
  "The box at full size" below. The old box had a straight slot across its width near one end, and that slot is gone.
  The tissues, the film squeeze and the sprung film edge are the same as before (only the second sheet is shorter,
  7 x 6.1 cm, to fit the narrower box), and so is the scoring. Two things changed with the centred slit, both
  measured in sim and stated in `task.json`:
  - A single pinch on the sheet now creeps out of the fingers under the slit's drag. It fell back into the box in 3
    debug runs, with the sheet pinched 15 mm off its middle, 4 mm off, and with a tighter pinch. So the demo pulls
    hand over hand (two 3 cm strokes, re-pinching lower each time, then 10 cm), as people do with a stubborn tissue.
    That demo scores 1.0.
  - Pulled at the box's middle, the drag no longer lifts the 40 g box. A one-handed run (nobody holding the box)
    only rocked it by 1.6 deg and moved it 8 mm, and it still scored 1.0. The steadying hand in the demo, on the
    lid's near-left corner, is therefore a precaution, not a requirement. The context no longer claims that one
    hand fails.
- **TK-NTJF9F, importance: "extracting a tissue onto a tray is not plainly a regular core housekeeping task."** The
  category is Societal relatability, and the context describes the everyday tissue-box scenario (done in the
  previous revision, kept).
- **TK-NTJF9F, scoring: "A momentary 6 mm box lift or 6-degree tilt permanently zeros an otherwise successful
  trial."** Only consequential events latch: a tip-over beyond 45 deg, the box leaving the table (carried more than
  5 cm up, or fallen off), or the second tissue spilling out too. Otherwise the box's final placement is judged (done
  in the previous revision, kept). All three latches have now been shown firing in sim (see "Controls run in sim").

## The real rig: spacer and sheets

- Under the two sheets, a light foam spacer block fills the box to 2 cm under the lid top. This is the rigid 31 mm
  block in the code, at full box length and width. With the spacer and the two sheets, the box weighs 40 g, the
  weight used in the sim.
- The two sheets are pale pink 2-ply facial tissues cut down from full sheets (8.3 x 7.8 in) to 7 x 7.8 cm (the
  top one) and 7 x 6.1 cm (the next one), then interfolded by hand as in the box. They are cut down because the sim
  runs the cloth at 3 mm resolution: a full sheet would need about ten times the mesh nodes. The cut-down sheets keep
  everything the task tests: they still fill most of the 8.3 cm slit, they are interfolded, and they are squeezed
  by the film.
- Pink is a common facial-tissue tint and shows up on camera against the blue box and the grey tray. The spacer is
  white.

## The box at full size

- Box: Kleenex Professional facial tissue, flat box, item 21400, 8.875 x 4.75 x 2.0 in (22.5 x 12.1 x 5.1 cm). See
  Kimberly-Clark Professional's product page
  (kcprofessional.com/en-us/products/restroom-and-hygiene/facial-tissue/flat-box/kleenex-professional-facial-tissue---flat-box/21400).
  Sheets are 8.3 x 7.8 in.
- Oval opening and film slit: a commercial Kleenex carton's opening as described in Kimberly-Clark's US 5,415,320
  (patents.google.com/patent/US5415320). The oval is 3.5 x 2.25 in (8.9 x 5.7 cm), covered by plastic film with a
  single slit of about 3.25 in (8.3 cm) along the oval's long axis. The patent measures it on an upright carton; we
  found no published figure for the flat box's oval and use these dimensions on the flat box.
- In the sim, the oval's edge is a staircase of paperboard strips in bands about 3 mm wide, and the film over the
  opening is 1 mm thick. The slit has two edges. One is fixed: a 10 mm lip under the film. The other is a light film
  strip sprung against the sheets.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/pop-up-tissue-take-one/pop_up_tissue_take_one.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). The scoring cases include a brief 6 mm lift and a brief
6 deg tilt of the box that then settles on its spot (still full credit), and a tip-over, a box carried more than 5 cm
off the table and a box knocked off the table (0, latched for the trial). Expected output:

```
scoring: 21 cases pass
left: 33.1 s plan, worst IK 0.085 mm / 0.012 deg
right: 33.1 s plan, worst IK 0.083 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/pop-up-tissue-take-one/pop_up_tissue_take_one.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take one tissue out of the box and put it on the tray, leaving the box in place with the next tissue ready to pull." \
  --scorer task_score --max-steps 700 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pop-up-tissue-take-one-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

Scoring in short (full text in `task.json`): a trial is zeroed for good only by a consequential event (the box tilted
more than 45 deg, its lowest corner more than 5 cm above the table or its bottom more than 2 cm below the table top,
or the second tissue out of the box too). A brief lift or tilt short of that costs nothing; at the end the box must be
back within 1.5 cm of its spot, turned less than 10 deg, tilted less than 5 deg and resting on the table.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/pop-up-tissue-take-one/pop_up_tissue_take_one.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take one tissue out of the box and put it on the tray, leaving the box in place with the next tissue ready to pull." \
  --scorer task_score --max-steps 700 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pop-up-tissue-take-one-demo
```

The demo is `demo()` in the task file, open loop with 33 s of plan. The left hand steadies the box at the lid's
near-left corner, then moves to a viewing pose beside the box (see "Videos and screenshot"). The right hand pulls the
tissue out hand over hand and lays it on the tray. The episode ends on
success about 1 s after the tissue is let go (about 610 steps), inside the run limit of 700 steps.

## Closest tasks

Unlike #531 (hold a towel roll, tear a sheet, wipe), nothing is torn here. Unlike #1310 (brace a block while pulling
a slip-sheet from under it), the pulled sheet must leave its follower standing ready, and the box does not need
bracing.

## Controls run in sim

All runs below use the final task file, changed only as noted. The full `score()` was computed every step.

| Run | What it does | Result |
| --- | --- | --- |
| Demo | as shipped | 1.0. The box never moved (lift 0, tilt 0). Tissue 1 came fully out, ending 80 mm above the lid, and landed on the tray. Tissue 2 stood 19.6 mm up with 68 % still in the box. |
| One-handed | `NAIVE = True`: same pull, nobody holds the box | 1.0. The box rocked (max tilt 1.6 deg, up about 3.5 mm) and drifted 8 mm, within its 15 mm spot tolerance. No latch fired. |
| Take two | tissue 1 dropped beside the box, then the standing next tissue pulled out too (hand over hand) | 0. The second-tissue latch fired at step 830. |
| Box on the floor | box spawned lying on its side, past the table's far edge | 0. The tip-over (90 deg) and off-table (bottom 0.7 m below the table top) latches fired at the first scored step. |
| Single pinch | three earlier demo builds that pulled in one stroke (pinch 15 mm off the sheet's middle, 4 mm off, or tighter) | 0 each. The sheet crept out of the jaws and fell back with 27-38 % still in the box. No latch fired. |

A box spawned in the air (80 mm up, tilted 60 deg) settled onto the table before the first scored step, so it could
not test the latches. That is why the floor spawn is used.

## Videos and screenshot

```bash
for log in ~/yam-eval/pop-up-tissue-take-one-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/pop-up-tissue-take-one-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

The recorded demo video and the setup screenshot were rendered with `-E cam_width=640 -E cam_height=360`, which is
the cameras' native render size. The scripted demo does not use the images. Dream-1 runs at 320 x 176, as above.
The top camera looks along the slit, so it sees a tissue standing in the slit edge-on, as a thin line. To show it,
the demo's left hand ends in a viewing pose once it lets go of the box. The pose is 10 cm off the box's long side,
with the TCP 10 cm above the table, pointing across the box and 30 deg down. From there, its wrist camera sees the
standing tab face-on. In the offline replay, the hand stays about 10 cm from the box in the pose, and at least
4.4 cm from the box and 4.8 cm from the tab's region on the way there. The score is unchanged: a debug run with the
pose scored 1.0, with the next tissue standing 19.5 mm. The scorer reads the tab's height from the cloth mesh nodes,
not from any image.

The recorded demo video is the top camera and the left wrist camera side by side, from the same run (ffmpeg
`hstack`). The wrist stream is rotated 180 deg for display, because the camera is mounted upside down in that pose.
Its last frame shows the pink tissue on the tray in the top view. The wrist view shows the next tissue standing pink
in the slit above the box's lid.

## Run limit, runtime and seeds

- Run limit: 700 steps (35 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is open loop (PhysX here is not bit-deterministic, so the cloth can differ slightly from run to run).
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Take the second tissue back down, re-interfold the two sheets as at the start (the top one round the next one's edge, its end standing 2.5 cm up through the slit) and put the box back on its spot; empty the tray.

## Helper modules shipped with the task

- `softkit.py` (v0.2): puts real deformable bodies in a task file. A `softkit.Soft` entry in `OBJECTS` is spawned as
  an Isaac Lab-Arena `DeformableObject` (here `kind="cloth"`: a PhysX surface deformable); v0.2 adds `profile`, a
  creased sheet given by its cross-section, spawned about its node centroid. `score()` receives the mesh nodes
  (`softkit.nodes(state)`; `softkit.motion(state, name)` measures stillness from node displacement). It hooks into
  yam-arena in memory at import time (no files are changed or installed) and works offline on hand-built node sets
  for the self-test (`softkit.rest_nodes`, `softkit.fake`).
- `mechkit.py` (v0.2): the tissue box and the sprung film edge of its slit. The box is one free rigid body made of
  boxes: walls, the spacer, the paperboard lid with its oval opening, and the film over the opening. The sprung edge
  is a prismatic joint with a spring, authored as a plain PhysX joint. The module also provides the per-episode
  memory and the grippers' TCP positions.
  No step rules are used in this task.
- Both are imported by the task file from its own folder; the commands above run them as they are.

The stdout of a run prints one line per soft body:

```
[softkit] tissue_1: 675 nodes
[softkit] tissue_2: 550 nodes
```

## Assets and licences

None. The box (with its oval opening and film), the film's sprung edge and the tray are built from yam-arena boxes and the tissues' meshes are generated in code at
spawn time. No third-party model, texture or mesh file is used.
