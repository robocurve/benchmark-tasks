# Cut a cable tie off a cable bundle without nicking the cables

A bundle of seven insulated cables lies across two beech blocks, held together by a black nylon cable tie between the
blocks. The tie's head sits on the near side of the bundle. A pair of flush cutters with red handles lies in a tray to
the right. Two hands cut the tie off:
- One hand presses the bundle down on its left block.
- The other takes the cutters by their handles and pushes the jaws onto the strap, right beside the tie's head, then
  squeezes. The tie springs open and drops onto the table.
- The cutters go back in the tray, the hand lets go of the bundle, and both arms go back to where they started.

The task is one task file,
[`cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py`](cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests:
- using a hand tool that has to be squeezed;
- placing its jaws to within a few millimetres;
- knowing where a cut is safe for the material underneath;
- steadying a light, round workpiece with the other hand;
- putting the tool back.

Why two hands: the bundle is light and round. When the cutters' jaws touch it, it rolls or slides away, and the tie's
head rolls away from the jaws. In the one-hand check, the right hand did its part as in the demo and the left never
moved; it scored 0.25. The jaws pushed the bundle 1.2 cm along the blocks, the strap stayed 9 mm short of the tips, and
the squeeze cut nothing.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The blocks and the tray are kinematic, fixed where
they are put. The other parts are loose rigid bodies made of shapes from `bodykit.py`.

- **The bundle.** One rigid body of seven cables, 120 g.
- **The tie.** Glued to the bundle by a fixed joint (a `Glue` from `bodykit.py`).
- **The cutters.** Two steel halves joined at their pivot by a hinge (a `Hinge` from `bodykit.py`). A spring holds the
  jaws 8 mm open at the tips. A hand holding the handles lightly keeps them open; squeezing them shut turns the halves
  up to 16 degrees together, closing the jaws.
- **The cut.** After every control step, the task file checks for the moment the halves close past 10 degrees. If the
  strap is then truly between the blades, the tie is cut. That means all 4 mm of its width lies between the 8 mm-apart
  edges (its middle within 3 mm of the mouth's middle line), the tips are at or past its inner face (within 10.9 mm of
  the bundle's axis), the cutters point into the bundle (within 50 degrees) and their pivot runs along the strap
  (within 50 degrees). When that holds:
  - the glue switches off;
  - the closed tie is moved out of sight;
  - an open tie, its strap cut beside the head, takes its place and drops to the table.
- **A nick.** The task file judges where the cut was made. A cut more than 12 mm along the strap from the middle of the
  head counts as a nick: there, the strap lies tight on the insulation. So does closing the jaws with a corner of a tip
  pressed 1.5 mm or more into any cable.
- **Off the bundle.** The cut tie must end off the bundle (its middle 1.5 cm below the bundle's axis or 3 cm from it)
  for any credit for the cut.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. This gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: full score, with the cutters and the bundle
still. Full score includes the cutters back in the tray, both hands let go and both arms back where they started.
Otherwise it gives `score()` at the end of the episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Cut the cable tie off the cable bundle with the flush cutters from the tray, without nicking any cable's insulation. Then put the cutters back on the tray's rests, leave the bundle across its blocks with nothing holding it, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/tie-dream
```

The scripted demo reaches full score:
1. The left hand comes straight down on the bundle above its left block, fingers closed, and presses it down.
2. The right comes straight down round the cutters' handles and closes on them just enough to hold them.
3. It lifts the cutters out of the tray and carries them, jaws first, level with the strap just below the tie's head.
4. It pushes the jaws in until they touch the bundle, the strap between the tips.
5. It squeezes the handles: the tie springs off and drops to the table.
6. It carries the cutters back, lays them on the tray's rests and lets go.
7. The left lets go of the bundle, and both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cut_a_cable_tie_off_a_bundle_without_nicking_the_cables.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Cut the cable tie off the cable bundle with the flush cutters from the tray, without nicking any cable's insulation. Then put the cutters back on the tray's rests, leave the bundle across its blocks with nothing holding it, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/tie-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tie-dream/*.json; do inspect-robots video "$log"; done
```

Set `TIE_DEBUG=1` to print what the score is built from twice a second:
- how far the cutters' halves are closed;
- where the jaws' tips are against the strap: the gap to it, the offset across it, and the distance along it from the
  head;
- whether the tie is cut, nicked, the cutters taken or back, and the bundle still on its blocks;
- whether both hands are clear;
- where each TCP is from the cutters' handles and from the bundle above its left block.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 900 steps (262 s on the clock, with Dream answering in 0.68 s on average), and about 2 minutes for the scripted demo (377 steps, 18.9 s of simulated time, 120 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-KA9BNZ. Its review said:

> Construct validity (not met): Closing the cutters within 4 mm of the strap can score a cut without severing it, and actual damage to cables is not checked.
>
> Scoring (not met): The proximity-based cut and location-based nick rules do not reliably establish whether the tie was severed or insulation was damaged.
>
> What to change: Revise the real-world scoring to require the tie to be physically severed, rather than treating jaw closure within 4 mm as a cut, and inspect the cable insulation, awarding zero if any cable is nicked. Relabel the task as workplace maintenance or another appropriate domain: the setup does not specify an immediate hazard to a person.

What changed in response:

- A cut now needs the strap truly between the blades as they close: all 4 mm of its width between the 8 mm-apart
  edges (its middle within 3 mm of the mouth's middle line), the tips at or past the strap's inner face (within
  10.9 mm of the bundle's axis), the cutters pointing into the bundle and their pivot along the strap's run. Closing
  the jaws merely near the strap no longer counts. Every credit for the cut also needs the severed tie to have come
  off the bundle.
- Nicks: besides closing on the strap away from its head, closing the jaws with a corner of a tip pressed 1.5 mm or more into
  any cable scores 0. In a real trial every cable's insulation is inspected after the trial, and any nick or cut
  scores 0.
- The instruction states only the goal and its constraints; the category is Impact on jobs (workplace maintenance).
