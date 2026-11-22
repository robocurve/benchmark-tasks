# Space the rice pans on the cooler shelf

A low cooler shelf with a front lip stands on the station table, and one hotel pan of hot cooked rice already stands in
the middle of it. A second pan of hot rice stands in front of the shelf, on a hot-holding block as high as the shelf.
The instruction is only: "Put the pan of hot rice on the cooler shelf to cool down, with the other one." FDA Food Code
3-501.15(B) asks that food containers being cooled are arranged in the cooling equipment to give the most heat transfer
through the container walls (FDA's guide for food employees on cooling cooked food; adopted in Minnesota Rules
4626.0390), and a food-safety guide for cafes (Mays) puts it as arranging the containers so air can move around them,
naming hot pans stacked tight together as the classic mistake. The Food Code gives no distance; this task asks for at
least 5 cm of clear space between the pans and no stacking. The instruction does not state the rule, so the task
checks whether the robot spaces the pans unprompted:

- **Make room first.** With the first pan in the middle of the 74 cm shelf there is no room for the second at 5 cm.
  The first pan has to be moved to one end of the shelf before the second is brought over.
- **Strictly bimanual.** Each 1.5 kg pan has no handles and is gripped by its rim; held by one end its weight acts
  about 16 cm from the grip (about 2.4 N m). In the recorded `one_arm` variant the pan tipped in the single pinch,
  slipped out on the way over and came to rest mostly off the shelf. The shelf's front lip stops a pan being slid on.
- **Spacing, not only placing.** Both pans on the shelf but touching scores 0.25, 1-5 cm apart 0.5; only 5 cm or more
  scores 1.0. A pan stacked, tipped past 30 degrees or dropped scores 0.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (visual-only parts for the rice and the shelf's rack lines), and per-step readings: finger-pad points and part poses, with the tilt, proximity, fall and stillness checks the scorer uses |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants (`democore.py` is shared with our other kitchen tasks) |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `demo.mp4` (on the task board) | Video of the full demo: the scene camera above, the two wrist cameras below |
| `scorer_check.py` | Feeds set end states (pan poses) to `task.py`'s scorer directly, without physics, for the cases the scripted demo could not reach |
| `setup_shot.py` | Writes `setup.png` |
| `setup.png` (on the task board) | The scene camera's view of the starting layout |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |

The demo video, the Dream-1 run video and the setup screenshot are uploaded to the task board with the repository's `tools/upload_media.py`; `task.json` names them by media ID.

`kitchenkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the pan of hot rice on the cooler shelf to cool down, with the other one." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/rice-pans-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The episode ends
once `success()` has held for 1 s (yam_arena's default `success_hold_s`, `SUCCESS_HOLD_S = 1.0`), or at 2,400 steps.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the recorded full demo ended at step 1076 (about 54 s of simulated time at 20 Hz). A model trial
runs up to 2,400 steps, plus the policy's own response time. The demo's wall-clock time is not representative: it
was recorded while the two other variants ran on the same GPU. Its rim-grip search takes 20-30 minutes; `demo.py`
caches it in `grips_cache.json` (optional) and reuses it while the layout is unchanged. Measured on Nov 16: the
Dream-1 run (`dreamlabs/dream-1-beta`, 2,400 steps, `rows_per_request=12`) took 6.6 min of wall-clock time on one
NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_squeeze --variant squeeze    # any variant below; --no-record skips the video
python demo.py task.py --plan-only                      # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out           # print the pans' state every 10 steps
```

Both arms come down from above onto the two ends of the pan in the middle of the shelf and pinch its rim: one finger
inside on the end wall, the other outside on the flange's edge, the pinch closing along the pan's length. They lift it
4 cm, carry it with its underside 4.5 cm above the shelf's top (over the 1.5 cm front lip) to the shelf's right end and
let go. Then they pinch the rim of the pan on the hot-holding block, lift it, carry it level onto the shelf's left end
and let go. While a pan is carried over the shelf and set down, both arms add the same correction, steering the pan
(not each hand) onto its planned path, so the hands move together. The demo aims the pans at y -21 and +21 cm, 9.6 cm
apart flange edge to flange edge; they ended 7.6 cm apart.

| Variant | What it does | Score |
|---|---|---|
| `full` | both arms move the first pan to the shelf's right end, then carry the second pan onto its left end | **1.0** (both pans resting on the shelf, released, tilt 0.0 degrees; clear gap 7.6 cm, first pan's centre at y -20.4 cm, second at y 19.6 cm; shelf shares 1.0 and 1.0; largest tilt during the run 17.38 degrees; not stacked, still, no drop; 1076 steps) |
| `squeeze` | the first pan is left in the middle; the second is set down beside it, hanging off the shelf's end | 0.0 (second pan not resting on the shelf: shelf share 0.489, tilt 4.78 degrees, centre at y 32.5 cm; first pan still in the middle, y -0.5 cm; largest tilt during the run 14.5 degrees; no drop; 586 steps) |
| `one_arm` | the right arm alone lifts the hot pan by the rim at its right end | 0.0 (the pan tipped in the single pinch, largest tilt 14.18 degrees, slipped out on the way over and came to rest mostly off the shelf: shelf share 0.178, tilt 5.26 degrees, centre at y 6.0 cm; no drop flag; 586 steps) |

The numbers are from `demo_results.json`.

**Not recorded:** pans set down closer than 5 cm (0.5), touching (0.25) and stacked (0). `scorer_check.py` feeds those
end states to `task.py`'s scorer directly instead (below).

## Scorer check

This is not a simulation. `scorer_check.py` sets the two pans' poses (level, on the shelf's top, at the shelf's
centre line) and feeds them to `task.py`'s scorer for 25 steps with no gripper near either pan, except where stated.
Recorded output of `python scorer_check.py task.py`:

| Case | Set end state | Score | Gap | Placed | Stacked | Still |
|---|---|---|---|---|---|---|
| full demo targets (9.6 cm apart) | pans at y -21 and +21 cm | 1.00 | 0.096 m | both | no | yes |
| 2.4 cm apart | pans at y -17.4 and +17.4 cm | 0.50 | 0.024 m | both | no | yes |
| 0.5 cm apart (touching) | pans at y -16.5 and +16.4 cm | 0.25 | 0.005 m | both | no | yes |
| stacked in the middle | second pan one pan height above the first, both at y 0 | 0.00 | none | first only | yes | yes |
| second pan still held | as the demo targets, finger-pad points at the flange on the second pan's right end | 0.00 | none | first only | no | yes |
| pans still moving | as the demo targets, the pans sliding apart 3 mm per step | 0.00 | none | neither | no | no |

All six cases scored as expected.

## Scoring

Every step, from poses; the score at the end counts (`SCORE_OVER = "end"`).

- **Resting on the shelf:** at least 90% of a grid of 45 points (5 across by 9 along) over the pan's underside lies
  inside the shelf's 22 x 74 cm outline, the underside is within 6 mm of the shelf's top, and the pan is tilted 5
  degrees or less.
- **Released:** no finger pad within 8 mm of the pan's box out to its flange (17.5 x 32.4 x 6.5 cm).
- **Gap:** with both pans resting and released, the clear gap along the shelf, flange edge to flange edge: the
  distance between the pans' centres along y less 32.4 cm.
- **1.0 (success):** the gap is at least 5 cm.
- **0.5:** the gap is 1-5 cm.
- **0.25:** the gap is under 1 cm (touching).
- **0:** either pan is not resting on the shelf or not released; a pan is stacked on the other (more than 30% of its
  underside grid over the other pan's 14.5 x 29.4 cm outline and its underside more than 3 cm above the other's); or the
  pans are not still (each moving less than 2 mm over 20 steps, 1 s at 20 Hz). Also 0 from the moment either pan tips
  past 30 degrees (the rice spills) or is dropped (it falls 3 cm within 10 steps, 0.5 s, with no finger pad within 2 cm
  of it).

The episode ends once `success()` has held for 1 s. `report()` records each pan's shelf share, tilt, position along
the shelf and whether it is resting, released or stacked, plus the gap, the largest tilt so far and the spill, drop and
stillness flags. Rice and heat transfer are not simulated: the rice is a visual block whose mass is part of the pan's,
and the rule is scored from the pans' poses.

## Real-world trials

Use two third-size steel steam-table pans 2.5 in (6.5 cm) deep, with no handles, each loaded to 1.5 kg with cooked rice
about 3 cm deep at room temperature (heat is
not part of the score, so hot food is not needed), a low rack with a lip along its front edge and a block as high as
the rack for the second pan. Put tape marks on the rack's front edge to read the gap between the pans' flange edges at
the score's steps, 1 cm and 5 cm, and mark the rack's ends. Score the end state from an overhead camera: both pans fully
on the rack, level, released, not stacked, and the gap between them.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** FDA Food Code 3-501.15(B): when placed in cooling or cold holding equipment, food containers being cooled are arranged to give the most heat transfer through the container walls (FDA, Cooling Cooked Time/Temperature Control for Safety Foods and the FDA Food Code, for food employees; adopted in Minnesota Rules 4626.0390, item B). A food-safety guide for cafes (Mays, on the FDA Food Code as applied in Florida) puts it as arranging the containers so air can move around them, and names stacking hot pans tight together, lids on, as the classic mistake. The Food Code gives no distance: this task takes at least 5 cm of clear space between the pans and no stacking. The instruction does not state the rule, and with the first pan in the middle of the shelf there is no room for the second at that distance, so the first pan has to be moved to one end first. Two arms: each 1.5 kg pan has no handles and is gripped by its rim; held by one end its weight acts about 16 cm from the grip (about 2.4 N m), and in the recorded one-arm variant the pan tipped 14 degrees in the single pinch, slipped out on the way over and came to rest mostly off the shelf. The shelf's front lip stops a pan being slid on. Done by: cooks and food-preparation staff in commercial kitchens, who cool batches of cooked food in hotel pans.

**Simulation fidelity.** Rigid bodies: boxes (each pan's floor, walls and rim flange; the shelf with its lip and back stop; the hot-holding block), with contact forces and friction; each pan is one 1.5 kg rigid body. Heat, steam, cooling and the rice are not simulated: the rice is a visual block inside each pan. The scorer stands in for heat transfer with the pans' poses: the Food Code asks for the arrangement that gives the most heat transfer through the container walls, so the score asks for both pans resting level on the shelf, at least 5 cm of clear space between them and neither stacked on the other; a tilt past 30 degrees stands in for the rice sliding out. The pans have no lids, so the covering part of 3-501.15(B) is not tested.
