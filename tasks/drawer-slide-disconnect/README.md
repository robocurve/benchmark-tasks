# Take a drawer off its slides by pressing two opposite-sense release levers while pulling, then set it down

The robot takes a drawer completely off its two ball-bearing slides and sets it down on the table. Each slide has its
own spring release lever, and the two work in opposite senses: the left lever is a tab pushed up, the right lever is a
tab pushed down. Each lever lifts a latch dog out of the way of a fixed block in its slide channel, so the drawer
stops at the block unless both levers are held. The robot holds both levers and pulls at the same time, carries the
drawer off the rails, lowers it onto the table and lets go. At the end the drawer is off both slides, flat on the
table, at rest, and both grippers are clear. Taking a drawer out to clean, paint or fix a cabinet is a routine
household job.

Physics, not the instruction, defeats the natural plan. The drawer stands pulled out 129 mm against its latch.
Pulling harder does nothing and one hand on one lever does nothing, because the other dog still meets its block. Once
the inner members are past the blocks the drawer is held only by its rails and the grippers: it is a free body, with
no joint between it and the cabinet. When its rear leaves the rails it hangs from the grippers and must be lowered.

Instruction given to the policy (18 words; goal only, no lever or technique):

`Take the drawer fully out of its slides and set it down on the table in front of the cabinet, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Revision

Reviewer points on the first filing and what changed:

1. "Scoring (not met): an extension of 280 mm and gripper clearance can score 1.0 without the drawer being detached or
   resting on the table." The extension rung is gone. Score is now the final state: 1.0 needs the drawer off both
   slides (rear 10 mm or more in front of the bay face), all four bottom corners within 4 mm of the table top, at rest
   for 1 s, and both grippers 30 mm clear and still for 1 s. A cheat that pulls and lets go (`pull_release`, the old
   demo's shortcut) leaves the drawer hanging on its rails and scores 0.
2. "Feasibility (not met): the drawer remains constrained to a fixed-height prismatic joint." The drawer is now a free
   body. The slide joint, the joint limit that lifted at the stop and the scripted latch are removed. Its inner members
   rest on rails in the bay's slide channels; the latch is two real dogs and blocks that collide; past the blocks only
   gravity, the rails and the grippers act on the drawer. The demo pulls it off the rails, lowers 98 mm and lets go.
3. "Update the full-score conditions to require actual detachment and stable table support." See point 1 and the
   scoring table.
4. "Clarify how the 1.5 mm lever gauge is measured, since the specified tab-to-flange pad span is 77 mm." The earlier
   "gap 1.5 mm or less" was wrong: the tab-to-flange pad span is 34 mm at rest (both levers) and 29.5 mm or less when
   pressed (28 mm at full 6 mm travel). The 1.5 mm is the width of a band printed on each latch bar that is covered at
   rest and visible only at 4.5 to 6 mm of travel. See "Lever check on the bench". The score no longer reads the levers.
5. Also changed so the demo can set the drawer on the table: both lever tabs now sit above the drawer bottom (the left
   tab no longer hangs 12 mm below it), so the drawer lies flat.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task.py sha 9eb243b81bc4 (`sha256sum task.py | cut -c1-12`)

## Offline check, no simulator

```bash
pytest -q tests        # 14 scorer tests, pure Python (yam_arena must be importable for TABLE_TOP)
```

## Run and score one Dream-1 trial

Dream-1 run on inspect-robots 1.0.0 at 0059d02a1a91, task 9eb243b81bc4, 1200 steps (7 minutes), log adhoc_8a1bb513.json, score 0.0,
max_steps.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant left_only      # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only the drawer's link poses (what a camera could
measure) and the robot's own grippers; it never reads the levers' travel or the record.

1. Both arms together. Each arm starts 80 mm in front of its lever, tilted 15 deg down, then goes forward until its
   pads straddle the lever tab and the fixed flange (pad span 34 mm, opened to 94 mm) and closes to 25.5 mm. Closing
   pushes each tab 6 mm toward its flange (left up, right down), which lifts or lowers its dog clear of its block.
2. Holding the squeeze, both arms draw the drawer out 183 mm over 8 s (129 to 312 mm). The dogs pass their blocks at
   about 152 mm; the rear leaves the rails near 290 mm and the drawer hangs from the pinches.
3. Both lower 98 mm over 4 s, setting the drawer on the table, wait 1 s, open to 45 mm, wait, and rise 90 mm.

The demo scores 1.0 (`success`, step 519 of 1,200). Both levers reach 6.0 mm, the drawer starts to move at about 8 s,
ends 305 mm out with its bottom on the table (all four corners within 1 mm) and both grippers end 52 mm clear.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file; every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `left_only` | Only the left arm presses its lever and pulls. | Left lever 6 mm, right at 0.16 mm; the right dog meets its block and the drawer stays at 129 mm on the rails. | 0.0 |
| `right_only` | Only the right arm presses its lever and pulls. | The mirror; the drawer stays at 129 mm on the rails. | 0.0 |
| `pull_release` | The old demo's shortcut: both levers pressed, pull 95 mm (to 224 mm), open, back off 70 mm. No carry, no set-down. | Both grippers end 58 mm clear but the drawer's rear is still on the rails; it tips nose down and hangs there. | 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Final state only. The record is a linkkit History fed every control step; nothing earlier in the run (a wrong first
try, a lever pressed and let go, a drawer pushed back and pulled again) is held against the score.

| Score | Condition at the end |
|---|---|
| 0 | the drawer is not off both slides: any part of it is less than 10 mm in front of the bay face |
| 0.25 | off both slides, but not flat on the table and at rest for 1 s (held in the air, tilted, still moving) |
| 0.5 | off both slides, all four bottom corners within 4 mm of the table top, at rest (under 5 mm/s) for 1 s, a gripper not yet clear |
| 1.0 | also both grippers 30 mm or more from every part of the drawer and under 10 mm/s for 1 s (release gate) |

Real-trial measurement: a side video at 30 fps and a tape on the bench top. Off the slides: the rear of the drawer
reads at least 10 mm in front of the bay face on the tape. Flat on the table: a 4 mm feeler blade does not pass under
any of the four bottom corners. At rest: no tape movement over 30 frames. Clear: a 30 mm spacer card passed around each
gripper and the drawer. The drawer cannot reach this state without both levers, because a dog that is not lifted
stops against its block, so the score does not read the levers.

### Lever check on the bench (rig set-up, not scoring)

Each lever's bar carries a printed 6 mm scale and, from 4.5 to 6.0 mm of travel, a 1.5 mm band that is covered at rest
and appears beside the bar slot only when the lever is pressed. The tab-to-flange pad span reads 34 mm at rest and
29.5 mm or less when the band shows (28 mm at full travel); it never closes to 1.5 mm. Latch check: clamp either
lever at rest and pull 30 N, the drawer must not pass the block; with both levers at 4.5 mm or more it passes at under
5 N.

## Run limit, runtime and seeds

1,200 steps (1 minute at 20 Hz). The demo finishes its script near step 440 and succeeds at step 519. The scene is
deterministic; there is no random seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 519.
- Dream-1 run: score 0.0, max_steps, 1200 steps, 7 minutes, log adhoc_8a1bb513.json.

## Materials

Bench dimensions (mm); positions x from the drawer front, y from the centre line, z from the drawer bottom.

- Drawer bay: plywood on the table on an 85 mm plinth, 300 x 316 x 200 mm, 8 kg, front face at x 380 from the arm
  midpoint. Per side a steel slide channel from 20 mm in front of the bay face to its back: an 8 mm web (y 137.2 to
  145.4, 48 mm tall) and a bottom rail 11 mm wide (y 121.5 to 132.8) and 7 mm thick whose top is the drawer's support
  plane, 100 mm above the table.
- Latch blocks, 10 mm long, fixed on the web, 4 mm wide (y 133.0 to 137.2), their faces 126 to 136 mm behind the bay
  face: the left block is 5.5 mm tall standing on the rail; the right block is 10.5 mm tall hanging from the web top.
- Drawer: 12 mm plywood, 1.2 kg, 280 x 240 x 65 mm walls with an inset front, 6 mm bottom, which rests flat on the
  rails by its two steel inner members (230 x 12.7 x 35 mm, 0.5 mm above the rail tops). It stands pulled out 129 mm.
  No part hangs below the bottom plane.
- Left lever (red, ABS, 20 g): a 35 x 21.5 x 3 mm tab at z 31 to 34 beside the left wall (y 123.5 to 145), pushed up
  6 mm against a 1.2 N/mm spring (5.4 N at 4.5 mm, 7.2 N at 6 mm), on a 3.8 x 8 mm latch bar along the inner member
  (y 133 to 136.8, z 28 to 36) ending in a 12 mm dog that hangs to z 16.5. Fixed above it on the wall, a 55 x 21.5 x 5 mm
  steel flange at z 60 to 65 (pad span from tab underside to flange top 34 mm).
- Right lever (blue, ABS, 20 g): a tab of the same size at z 65 to 68 pushed down 6 mm against the same spring, on a
  3.8 mm stem through a 3.8 mm slot in the flange and a latch bar at z 28.5 to 37.5 ending in a dog that stands up to
  z 48.5. Fixed below it on the wall, a flange at z 34 to 39 with that slot (pad span from flange underside to tab top
  34 mm).
- Latch geometry: at rest each dog overlaps its block by 3.5 mm; at 4.5 mm of travel it clears by 1 mm (left dog
  bottom z 16.5 against block top z 20; right dog top z 48.5 against block bottom z 45). The dog faces meet the
  blocks at 130 mm of extension.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Fidelity

The drawer, channels, springs, dogs and levers are full-size parts and the drawer is a free rigid body, so detaching it
and setting it down on the table are done by the arms and gravity alone, with no joint or brake standing in for the
slide. The simulator keeps the intermediate slide members rigid and fixed to the bay; the channel is modelled as a
rail 20 mm proud of the bay face that carries the drawer until its rear leaves it at about 290 mm. A real drawer would
not close back through the lever tabs, so the rig is reset by lowering it onto the rails from above (below).

## Reset between trials

Nothing is consumed. Lift the drawer onto the rails from above (inner members resting on the rail tops, dogs in front
of their blocks), slide it to the 129 mm stop, let both levers spring back to rest, and check the 5.4 N lever force at
4.5 mm with a spring scale.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
