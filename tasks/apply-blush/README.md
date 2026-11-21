# Apply blush to both cheeks of a practice head, inside the drawn oval

A vinyl cosmetology practice head stands on its stand on the table, fixed in place, facing the robot. On each cheek
an oval (4.4 x 2.8 cm) is drawn in solid ink, and a limit (6.8 x 4.8 cm) is drawn round it in dashed ink. Beside each
arm, a pressed-blush pan stands upright in a small stand, facing the arm, and a clean blush brush lies on the table.
The robot must pick up each brush, load it tip-first from its upright pan, and sweep it over its cheek's oval until
the oval is covered, without pigment going past the dashed limit.

Applying blush is a routine step in everyday makeup application; a 2016 YouGov poll found 38 % of US women wear
makeup at least a few times a week (26 % never) (YouGov, "Why American women wear makeup", yougov.com/en-us/articles/48130).
The difficulty is physical: the brush has to be picked up by its slim handle, loaded by swirling its bristles on the
cake, and brought to a curved cheek with the bristles square on it, then swept in strokes that stay in contact and
inside the oval; a dry brush or one held off the skin leaves nothing, and a brush pressed in one spot covers only its
own width.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the pigment model (`touch`), `score()`, `success()`, `self_test()` and the scripted `demo()` |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `tiltkit.py` | The tilted grip used to hold the brush's handle (shared helper) |
| `assets/bust.usdc` | The practice head and stand, one fixed rigid body |
| `assets/brush.usdc` | One blush brush, a free rigid body (its bristle cluster collided as a convex hull) |
| `assets/blush_pan.usdc` | One pressed-blush pan, a fixed rigid body |
| `assets/blush_samples.npz` | Sample points used by the pigment model and the demo: the head's skin, each cheek's oval/limit grid, the bristle head's surface, and the cake's surface |

No third-party USD assets; `work/blush/make_bust.py`, `make_tools.py` and `make_samples.py` build the four files
above from measured dimensions.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Practice head | centred (34.5, 0.0), facing the robot, fixed on its stand (taped to the table) | Vinyl cosmetology practice head: skull 19 cm front to back, 14 cm across, chin 12.5 cm and crown 31 cm above the table (as the accepted task "put-reading-glasses-on-a-care-manikin"); cheekbone and jaw widths after Zhuang and Bradtmiller 2005 |
| Right/left cheek oval | cheek centre 4.6 cm off the midline, 4.2 cm above the chin (16.7 cm above the table) | Drawn in solid ink, 4.4 x 2.8 cm (across the cheek x up) |
| Right/left cheek limit | round the oval, same centre | Drawn in dashed ink, 6.8 x 4.8 cm |
| Right/left blush pan | centred (30.0, -24.0) / (30.0, 24.0), held upright in its stand, facing the arm | Round tin pan, 36 mm across and 3.94 mm tall (TKB Trading 36 mm pan), filled level with pressed powder blush |
| Right/left pan stand | immediately behind its pan, fixed to the table | Plain rectangular plastic riser/palette stand, 30 x 15 x 23 mm, holding the pan upright and clear of the table |
| Right/left blush brush | lying on the table beside its pan, bristles towards the head, at (20.0, -17.0) / (20.0, 17.0) | Round blush brush, 157 mm long, 121 mm handle, 12 mm ferrule, 12 g (Kent TWMU10-style), clean |

Sources: makeup-use statistic, YouGov (yougov.com/en-us/articles/48130-why-american-women-wear-makeup-poll); head
dimensions, the accepted task "put-reading-glasses-on-a-care-manikin" and Zhuang and Bradtmiller 2005, "Prediction
models for the ANSUR II facial anthropometric data", Table VI (bizygomatic, bigonial breadth); pan, TKB Trading 36 mm
round pressed-powder pan listing; brush, Kent TWMU10.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Apply blush to both cheeks: sweep each brush over the inked oval on its cheek until it is covered." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/apply-blush-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Seed 0; the
scene has no random placement (`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules on hand-built pigment layouts and replays the
demo's plan through both arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Apply blush to both cheeks: sweep each brush over the inked oval on its cheek until it is covered." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/apply-blush-demo
```

Each arm, in parallel: picks up its brush by the handle, carries it to its upright pan and swirls the bristle tip
against the cake to load it (the same square-on, tip-first touch it later uses on the cheek), carries it to its
cheek, presses the bristles on the skin and sweeps two passes across the oval, then lifts the brush clear of the
face. Both ovals reach full coverage and the run scores 1.0 at 23.2 s (464 steps), once the brushes are lifted clear.

A wrong strategy, scrubbing both brushes over a wide area of the face well past the dashed limit, scores 0 (216 skin
points show pigment outside the limit). A dry brush, swept over the oval without ever dipping it in the pan, also
scores 0 (no dose, no pigment ever shows).

## Scoring

The score at the end of the episode counts. Tools for a real trial: a phone camera.

- **Covered:** in a phone photo taken square-on to each cheek, the fraction of the drawn oval's area showing pigment.
- **Bled:** pigment shows outside the dashed limit (either cheek), judged from the same photos, over a combined area
  of more than roughly 1 cm2 (about the size of a shirt button).
- **Clear:** at the end, both brushes held at least 2 cm off the skin, for 1 second (photo from the side).

| Outcome | Score |
|---|---|
| More than 1 cm2 of pigment outside the dashed limits | 0 |
| Less than 40 % of an oval covered | 0 for that cheek |
| 40-80 % of an oval covered | 0.5 for that cheek |
| 80 % or more of an oval covered | 1.0 for that cheek |
| Cheek scores average, a brush still within 2 cm of the skin at the end | capped at 0.75 |
| Cheek scores average 1.0, both brushes clear for 1 s | 1.0 |

The run's score is the mean of the two cheeks' scores (subject to the bleed and clear rules above).

## Reset

Wipe both cheeks clean, redraw the oval and limit in the same places, refresh each pan's cake surface level with the
rim, wipe both brushes clean and lay them back beside their pans.

## Real trials

The brushes are light (12 g) and soft-bristled; pressing them on the cake or the face takes well under a newton.
Nothing is sharp, hot or powered. The head's stand is taped down so a press against the cheek cannot tip it.
