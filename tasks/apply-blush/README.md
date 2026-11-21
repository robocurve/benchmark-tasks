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
| `contactkit.py` | Reads the PhysX contact force between each brush and the head or its pan (used by the pigment rule) |
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
| Right/left blush pan | centred (30.0, -24.0) / (30.0, 24.0), held upright in its stand, facing the arm | Round tin pan, 36 mm across and 3.94 mm tall (TKB Trading 36 mm pan), filled level with pressed (not loose) matte powder blush in one rose-pink shade from a single production lot; it swatches at about sRGB (215, 60, 105) on white paper, and a pan is replaced when its swatch differs by more than 10 in any channel |
| Right/left pan stand | immediately behind its pan, fixed to the table | Plain rectangular plastic riser/palette stand, 30 x 15 x 23 mm, holding the pan upright and clear of the table |
| Right/left blush brush | lying on the table beside its pan, bristles towards the head, at (20.0, -17.0) / (20.0, 17.0) | Round blush brush, 157 mm long, 121 mm handle, 12 mm ferrule, 12 g (Kent TWMU10-style), clean |
| Drop sheet | under the head stand, both pans and both brushes | Wipe-clean plastic sheet, 70 x 40 cm, 2 mm thick, taped to the table; it keeps loose powder off the robot, its cameras and the table |

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
against the cake, pressing it (0.4-1.3 N measured), to load it, carries it to its cheek, presses the bristles 4 mm
into the skin (about 0.3-0.7 N) and sweeps three passes across the oval (rows 4 mm apart), then lifts the brush clear
of the face. Both ovals are covered (91 % and 96 %), no pigment goes past the limit, and the run scores 1.0 by 28.2 s
(564 steps), once the brushes are lifted clear.

Wrong strategies, each scored 0:
- Scrubbing both brushes over a wide area of the face well past the dashed limit: 164 skin points show pigment outside
  the limit.
- Hovering about 6 mm over the cake instead of pressing it, then sweeping the cheek: the brush never loads (dose 0), so
  no pigment shows.
- Loading properly, then sweeping 4 mm above the skin instead of pressing it: 0 to 5 % of the ovals show pigment, score 0.
- A dry brush swept over the oval without dipping it in the pan: no dose, no pigment (checked in the offline tests).

## Scoring

The score at the end of the episode counts. Tools for a real trial: a phone camera.

- **Covered:** in a phone photo taken square-on to each cheek with a printed 1 cm grid on a clear sheet laid over it, the fraction of the drawn oval's grid squares showing pigment.
- **Bled:** from the same photos, the grid squares outside the dashed limit (either cheek) that show pigment: more than 1
  square in all (about 1 cm2, the size of a shirt button).
- **Clear:** judged from the trial video. A 2 cm block stands beside each cheek in view, and both brushes' bristles are at
  least that far from the skin for the last 1 second of the run.

| Outcome | Score |
|---|---|
| More than 1 cm2 of pigment outside the dashed limits | 0 |
| Less than 40 % of an oval covered | 0 for that cheek |
| 40-80 % of an oval covered | 0.5 for that cheek |
| 80 % or more of an oval covered | 1.0 for that cheek |
| Cheek scores average, a brush still within 2 cm of the skin at the end | capped at 0.75 |
| Cheek scores average 1.0, both brushes clear for 1 s | 1.0 |

The run's score is the mean of the two cheeks' scores (subject to the bleed and clear rules above).

## Pigment rule (simulation)

Pigment is modelled, and needs contact and pressure. A brush starts clean. It loads from the cake only while its
bristles rest on the pan and press it with at least 0.3 N; it deposits on skin within 2 mm of the bristles only while
they press the head with at least 0.2 N. Both forces are the PhysX contact force between the brush and the body,
smoothed over 0.1 s. A brush that hovers, or touches without pressing, leaves nothing. Both thresholds are light
presses, under half a newton.

## Reset

Wipe both cheeks clean, redraw the oval and limit in the same places, refresh each pan's cake surface level with the
rim and check its swatch against the shade in the setup, wipe both brushes clean and lay them back beside their pans.
Wipe or replace the drop sheet, and wipe the head stand and the robot's fingertips with a damp cloth.

## Real trials

The brushes are light (12 g) and soft-bristled; pressing them on the cake or the face takes well under a newton.
Nothing is sharp, hot or powered. The head's stand is taped down so a press against the cheek cannot tip it.
