# Release a car's handbrake: raise it off its pawl, hold the button in, lower the lever all the way to its stop

A car's hand-operated parking brake (handbrake) lever at true size, as fitted between the front seats of a small
hatchback, is bolted through its own bracket to a board in a low centre-console trim. It is applied: pulled up to its
4th click. The instruction is only "Release the handbrake fully."

Every driver of a car with a centre-console handbrake releases it at the start of every trip. The owner's manual of
the Toyota Yaris ("Parking brake", [Toyota Yaris owner's manual][manual]) gives the release in three steps: "Slightly
raise the lever", "Press the button", "Lower it completely".

## The hand skill

- Under the load of the brake cable the ratchet's pawl is pressed into its tooth. Pressed now, the button jams: it goes
  in only 1 mm. The lever must first be raised a little, off the pawl.
- The button, 15 mm across at the end of the lever, must be pushed in along the lever's axis (5 mm of its 8 mm travel
  lifts the pawl clear) and kept in the whole way down. Let go part-way, the pawl drops into the next tooth below and
  the lever stops there.
- The lever is then lowered all the way to its bottom stop, with its end swinging down through a 16 degree arc, and
  let go.

A person does this with one hand, the thumb on the button and the fingers round the grip. A parallel gripper that
holds the lever cannot also push the button along the lever's axis at its end, so here one arm raises and lowers the
lever while the other keeps the button pressed, following the lever's end round its arc. Pushing the lever down
without the button does nothing: the pawl holds it on its tooth.

The task is the task file
[`release_a_cars_handbrake_with_its_button_held_in.py`](release_a_cars_handbrake_with_its_button_held_in.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the lever and the console are built in these files from boxes and cylinders.

`yam_arena` task files place single boxes, cylinders and spheres and cannot put one on a joint. The handbrake (a fixed
base with the pawl's stop and the lever on revolute joints about the pivot, and the button on slide joints along the
lever) is built with `bodykit.py`, which adds such bodies and joints: importing it hooks `yam_arena.scene` when the
simulator starts, without changing yam_arena, and gives the task file each joint's position. This copy of
`bodykit.py` adds a start position for a joint (`init`, written at every reset): the lever is built lying level and
starts pulled up to its 4th click. Every control step the task file sets the drives' targets (the ratchet, the cable
spring, the button's spring and the jam) through Isaac Lab's articulation API, as described below.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Console, fixed: a board and a low trim, 18 x 43 cm and 1.2 cm tall, centred at x 22, y -1, with a 6 cm slot along
  the lever; the lever's steel bracket, its two plates either side of the pivot.
- Lever, 0.5 kg, in the plane x 22: pivot at y -15, 4.0 cm above the table, its axis along x. Pivot to the button's
  end 29.7 cm: a pressed steel bar 12 mm thick and 26 mm deep to 17 cm; a black grip 35 mm across from 17 to 27.2 cm;
  a 25 mm end cap to 28.9 cm; the release button, 15 mm across, standing 8 mm out of the cap. Fully down, the lever
  lies level on its bottom stop.
- Ratchet: a tooth every 4 deg from the bottom stop to 32 deg. Start: on the 4th tooth (16 deg; the button's end
  12.2 cm above the table). Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled:

- The pawl holds the lever on the highest tooth below it, unless the button is pressed in 5 mm or more. Raised, the
  lever clicks over the teeth (the pawl drops into each one it passes). Lowered with the button let out, the lever
  stops on the next tooth below.
- The button has 8 mm of travel and a spring (5 N at 5 mm, 8 N fully in).
- The jam: while the lever rests on its pawl (within 0.6 deg of the tooth) the button goes in at most 1 mm.
- The cable: a spring torque pulling the lever down, 0.24 N m at the bottom stop and 1.3 N m per radian above it
  (0.60 N m at the 4th click), on top of the lever's own weight (about 0.7 N m).

In the simulation the pawl's stop is a jointed drum round the pivot under the lever (3 kg, so the solver holds the
lever on it firmly; its weight sits on the pivot), held on the tooth's angle by a stiff drive (1500 N m/rad): the arm
pushing the lever down meets a hard stop. The jammed pawl is a stiff slide (30 kN/m) behind the button.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the lever's angle above its bottom stop, the button's travel, and the points of each
finger's collision hulls. "Released" means every point of both grippers' fingers is 3 cm or more from the lever.

- 1.0 (success, held 1 s): the lever within 2 deg of its bottom stop (resting on it), released.
- 0.75: the lever on its bottom stop but still held.
- 0.5: the lever lowered at least one tooth (14 deg or lower) but caught on a tooth above the bottom.
- 0: the lever at its 4th click or higher.

Doing nothing scores 0. A real trial is scored from the end of the video and a photo of the lever against a protractor
mark on the bracket (the bottom stop, then one mark per tooth): resting on its stop, lower than at the start, or not.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=release_a_cars_handbrake_with_its_button_held_in.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Release the handbrake fully." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/handbrake-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The right gripper, fingers closing
across the steel bar 15.5 cm from the pivot, raises the lever about 4 deg, off its pawl (it clicks over the 5th tooth).
The left gripper, fingers closed and turned so that their outer face is square to the lever's axis, pushes the button
in along the axis and keeps it in while the right lowers the lever over 5 s to its bottom stop; the left follows the
button's end round its arc. At the bottom the left lets the button out and backs off, the right lets go, and both go
back to their ready poses. It reaches success at step 425 (21 s).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=release_a_cars_handbrake_with_its_button_held_in.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Release the handbrake fully." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/handbrake-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/handbrake-dream/*.json; do inspect-robots video "$log"; done
```

`HB_DEBUG=1` prints, every 5 steps (`HB_DEBUG_EVERY=1` for every step), the lever's angle, its height above the pawl's
stop and the tooth the pawl is on, the button's travel, whether the pawl is engaged and loaded (the jam), how close the
nearest finger is to the lever, and where each gripper is and how far it is open.

The wrong ways, both scoring 0:

- `HB_NOBUTTON=1`: the left arm stays at its ready pose; the right raises the lever and pushes it down without the
  button. The lever clicks onto the 5th tooth on the way up and the pawl holds it there (20 deg) however hard it is
  pushed. Score 0.
- `HB_NOLIFT=1`: the left pushes the button without the lever first being raised. The button jams (it goes in less
  than 2 mm), and the lever stays on its 4th tooth when the right pushes it down. Score 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: debug runs of the scripted demo
without video (about 430 steps, 22 s of simulated time, ending on success) took about 5 minutes from start to finish;
the demo rendered at 1280 x 720 took about 4 minutes, and a Dream-1 trial of 2,400 steps at 320 x 176 about 12 minutes (its connection renewed itself a few times). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A hand parking-brake lever sub-assembly with a push-button release on the end of its grip, bought as a spare part
(for example the lever of a 2007-2011 Toyota Yaris; Toyota's part numbers for these start 46201, matched to the car by
the dealer), bolted through its own bracket to an 18 mm plywood board clamped to the table, with a wedge under the
bracket if needed so that the released lever lies level, its pivot 4 cm above the table at x 22, y -15 and its grip
toward the robot's left. A tension spring hooked from the cable's end to an eye on the board stands in for the brake
cable: chosen so that the lever pulls down with about 0.6 N m at its 4th click (about 3 N at the middle of the grip,
plus the lever's weight), much less than a car's cable, so that the arms can raise it. A low trim panel with a slot
round the lever (or dark card) stands in for the console. Mark a protractor on the bracket beside the pivot: the
bottom stop and each tooth. Before each trial pull the lever up until it has clicked 4 times, let it settle on the
4th tooth and check that the button has sprung back out.

[manual]: https://www.manualsdir.com/manuals/746448/toyota-2010-yaris.html?page=149
