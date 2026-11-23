# Wipe the plate rim, holding the plate

A plated main course stands on the wet stainless-steel pass in the middle of the table: the food (a whole English muffin,
the stand-in used in the real trials) in the middle of a square porcelain plate, and three round smudges of tomato
ketchup: near side and right side on the rim band inside the lip, and far-right at the band's inner corner, partly on
the plate's flat middle. A folded, damp towel pad lies on a small stand to the right.
The instruction is only: "Clean up the plate's rim with the towel pad before the plate goes out to the guest."

Restaurant cooks (O*NET 35-2014.00, about 1.4 million US workers) list among their core tasks "Portion, arrange, and
garnish food, and serve food to waiters or patrons"; BLS OEWS (May 2022) counts 988,060 of them in full-service
restaurants (NAICS 722511), where plates go out to seated guests. Wiping the rim is the last step of plating (Culinary
Arts Switzerland: with a damp cloth, before serving; Chefitt: before the plate leaves the kitchen; Personal Touch Dining,
a caterer: every rim, before sending out). Wet porcelain on a wet steel pass slides when a towel is dragged across it,
so one arm has to hold the plate still by its rim while the other wipes, and the food must stay untouched. The
instruction states neither. Beyond holding, the wipe needs control of force and position: the 3 cm pad must be pressed
down (at least 1.2 N) and slid over the whole of each 3 cm smudge without twisting the held plate or touching the food.

- **Strictly bimanual: hold and wipe.** With the same wiping and nobody holding the plate (`no_hold`), the plate slid
  10.34 cm and turned 13.44 degrees under the towel, and the trial scored 0.0. Once the towel has touched the plate,
  the plate must stay within 1 cm and 5 degrees of where it was.
- **Each smudge wiped over its whole area, with pressure.** Each smudge is 113 sample points (a 2.5 mm grid over the
  disc). A point is wiped once the towel pad's pressed underside (at least 1.2 N on the plate, moving at least 1 cm/s
  relative to it) has slid at least 1 cm over it; a smudge is clean when 95% of its points are wiped. Short strokes
  over one side of each smudge (`partial`) score 0.0, a towel that only skims the rim (`light`) 0.0, and leaving one
  smudge (`two`) 0.67.
- **The food is kept out of.** The muffin collides and rests loose on the plate. A contact force of 0.02 N or more
  between it and the towel or the forearm, wrist, gripper body or either finger of either arm scores 0 for the rest of
  the trial, as does a finger pad or the towel coming within 1 cm of any part of it (`food`: 0.0, with the rim clean
  and the plate within its limits).

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes, and per-step readings: finger-pad points, part poses, contact forces (mean and peak over the physics substeps) between parts and between a part and robot links |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants |
| `scorer_check.py` | Eleven edge cases fed straight to the scorer (no physics): dabbing, partial strokes, too little pressure, touches by each kind of link, the 1 cm keep-out |
| `demo_results.json` | Each variant's score and end state, from the runs below |
| `setup_shot.py`; `setup.png` (on the task board) | The 1280 x 704 starting layout, and the script that renders it |
| `demo.mp4` (on the task board) | The full demo (1.0): scene camera above, wrist cameras below; it ends with the towel carried off the plate and the whole rim clean, and its last frame is held for 3.5 s |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |
| `NOTES.md` | Context and simulation fidelity in full, the changes in detail, `report()`'s fields, every variant's end state |
| `template.svg` | The sauce template at 1:1 for printing: the 20.4 cm card, its three 3 cm holes, the side toward the robot marked |

**Assets:** no third-party meshes, scans or scenes; every part is a primitive box built in `kitchenkit.py`, in
yam_arena's station scene with its YAM arms. The robot and the station come with the program's `yam_arena` embodiment and are not part of this submission: the
YAM arm model (URDF and meshes) is from I2RT Robotics (github.com/i2rt-robotics/i2rt) under the MIT licence, and the
station scene is the one yam_arena provides for this program.

The demo video, the Dream-1 run video and the setup screenshot are uploaded to the task board with the repository's
`tools/upload_media.py`; `task.json` names them by media ID. `kitchenkit.py` hooks `yam_arena.scene` and the
embodiment's step and reset without modifying yam_arena, so the task runs with the standard `yam_arena` embodiment and
`task_score` scorer. Each step it reads the contact force between the towel pad and the plate, and between the food and
eleven bodies: the towel and, on both arms, the forearm (`link4`), wrist (`link5`), gripper body (`gripper`) and both
fingers (`tip_left`, `tip_right`). A pair that cannot be read is listed under `contact_errors` in `report()`; in every
run below all pairs were read.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clean up the plate's rim with the towel pad before the plate goes out to the guest." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/plate-rim-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The trial ends
when the task reports success: all three smudges clean with the plate within its limits, and then the towel off the
plate (not touching it, its underside at least 2 cm above the rim band) for 1 s.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Control rate:** the scorer measures the towel's speed over a 0.05 s step, so run at `control_hz=20`.

**Runtime:** the scripted demo ends on success at step 709 (35.45 s of simulated time at 20 Hz), about 2 s after the
towel leaves the plate. A model trial runs up to 2,400 steps, plus the policy's own response time.

**Dream-1 run (this version):** score 0.0 over the full 2,400 steps, about 9.8 minutes wall-clock (198 model requests).

## Scripted demo

```bash
python demo.py task.py demo_out                          # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_partial --variant partial     # any variant below; --no-record skips the video
python demo.py task.py --plan-only                       # inverse-kinematics check of every variant's plan
python scorer_check.py                                   # the scorer's edge cases, no physics
```

The left arm pinches the plate's lip on its left side and holds the plate. The right arm picks up the towel pad and,
at each smudge in turn (near, right, far-right), presses it and rubs it along the rim: eight strokes of 0.4 s, 2.5 cm
to each side (commanded), pairs alternating 1 mm out and in across the rim. It raises or lowers the towel to keep the
towel-plate contact near 1.2 N (median while rubbing: near 1.45 N, right 1.30 N, far-right 1.32 N), its targets follow
the plate's measured shift and turn, and the towel's measured position keeps the pad centred on the smudge (the 3 cm
pad is as wide as the 3 cm smudge). After the far-right smudge it lifts the towel off the plate and carries it back
toward its stand. The recorded run ends on success at step 709, with the towel off the plate for its last 1.95 s and
the whole rim in view.

| Variant | What it does | Steps | Score (points wiped near / right / far-right) |
|---|---|---|---|
| `full` | the left arm holds the plate by its lip; the right arm rubs each smudge in eight strokes, then lifts the towel off | 709 | **1.0** (99.1 / 99.1 / 99.1%; after the towel's first touch the plate slid 0.38 cm and turned 0.79 degrees; the food was never touched, closest approach 1.18 cm; the towel was off the plate for the last 1.95 s) |
| `no_hold` | the same wiping; nobody holds the plate | 896 | 0.0 (the plate slid 10.34 cm and turned 13.44 degrees, past the 3 cm limit at step 262, which set the 0; later a finger pad also came within 1 cm of the food, step 620) |
| `light` | as `full`, but the towel skims 2 mm above the rim (no pressure) | 942 | 0.0 (0 / 0 / 0%) |
| `two` | as `full`, but the far-right smudge is left | 758 | 0.67 (99.1 / 99.1 / 0%) |
| `partial` | as `full`, but the towel is centred 1.5 cm along the rim from each smudge's centre and rubbed in short strokes | 942 | 0.0 (54.0 / 36.3 / 46.9%, no smudge clean; the first version's rule would have counted all three clean: 31 / 31 / 35 rubbing steps against the 10 it needed) |
| `food` | as `full`, but after the right smudge the towel is lifted 2 mm and pushed 5 mm into the muffin's side (commanded) | 1020 | 0.0 (the food rule fired at step 506, finger pads within 1 cm, then the towel pushed the muffin with 0.41 N; the rim came clean, 99.1% each, and the plate stayed within its limits, slid 0.5 cm and turned 0.80 degrees, so the food rule alone set the 0) |

The numbers are from `demo_results.json` (each variant's full end state is in `NOTES.md`). Variants that do not
succeed run their whole plan. A run ends once success has held for 1 s (yam_arena's `success_hold_s`), and success
itself needs the towel off the plate for 1 s, so the full demo ends about 2 s after the towel leaves the plate.
`scorer_check.py` passes all eleven cases (listed in `NOTES.md`).

## Scoring

Every step, from poses and contact forces; the score at the end counts.

- **Wiping:** each smudge is 113 sample points, a 2.5 mm grid over its whole 3 cm disc. A point lies under the towel
  pad's pressed underside when it is inside the pad's 3 x 3 cm footprint and within 2 mm of it. In a step in which the
  towel presses on the plate with at least 1.2 N and moves at least 1 cm/s relative to it, every point under the pad at
  both the start and the end of the step is credited with the pad's travel in that step. A point is wiped once its
  credit reaches 1 cm; a smudge is clean when at least 95% of its points are wiped (at most 5 of 113 left). The sauce
  tile over each wiped point disappears from view (visual only), and so do a clean smudge's last tiles, up to 5
  unwiped points, where the real rule allows no mark of 2 mm or more. A dab, too little pressure, or short strokes
  over one side leave most of a smudge unwiped.
- **1.0:** all three smudges clean, with the plate within 1 cm and 5 degrees of where it was when the towel first
  touched it (gripping the lip may nudge it before then), and none of the zero conditions below.
- **End:** the trial ends (success) once that holds and the towel has then been off the plate for 1 s: no contact
  (under 0.05 N) and its underside at least 2 cm above the rim band, so a run ends with the rim in view. Otherwise the
  trial runs to the step limit; the score counts clean smudges either way.
- **Partial:** one third per clean smudge (0.33, 0.67), capped at 0.5 if the plate slid more than 1 cm or turned more
  than 5 degrees once the towel touched it.
- **0:** from the moment the plate is more than 3 cm from its start or lifted more than 2 cm (it is wiped where it
  stands on the pass, ready for pick-up), or the food is touched or crowded: a contact force of 0.02 N or more (peak
  over the step's physics substeps) between the food and the towel or the forearm, wrist, gripper body or either finger
  of either arm; or a finger pad of either arm, or any point of the towel pad's surface (sampled every 2.5 mm), within
  1 cm of any of the food's slats.

## Real-world trials

All items are commercially available, and nothing wet or saucy reaches the robot.

- **Plate:** a square white porcelain plate, 24 ± 1 cm, with a raised lip 1.6 ± 0.4 cm wide and 1.2 ± 0.3 cm tall
  round a flat rim band 3.7 ± 0.5 cm wide. It is heavier than the simulated 0.45 kg (roughly 0.65-1.2 kg, an
  estimate from catalogue weights): weigh it and log the mass.
- **Pass:** a flat stainless-steel tray at least 30 x 42 cm, raised on flat boards so its top is 4 ± 0.5 cm above the
  table, taped down, with tape marks for the plate centred at x 24, y 0.
- **Towel stand:** a block with a flat top at least 4 cm across and 3 ± 0.5 cm tall (for example an upturned
  stainless-steel sauce cup of that height) at x 15, y -24.
- **Sauce:** Heinz Tomato Ketchup at room temperature (20-25 C; the bottle kept at room temperature and shaken before
  filling), 0.2 mL per smudge from a 1 mL oral dosing syringe (no needle; filled from the back).
- **Template:** `template.svg`, printed at 100% (check its 10 cm scale bar), printed side up, stuck to thin card and cut
  to fit inside the lip (20.4 x 20.4 cm), with three 3 cm holes cut out, centred 8.35 cm from its centre: toward the
  robot (near), toward the right arm (right), and at the far-right corner, 45 degrees between them. The near and right
  holes lie on the rim band; the far-right one is at the band's inner far-right corner, about 45% of it on the band and
  the rest on the plate's flat middle.
- **Food stand-in:** one whole, untoasted Thomas' Original English Muffin (about 9.5 cm across, 3 cm tall, 57 g; use
  one 9-10 cm across), at room temperature from a sealed pack within its date; a new one each session and after any
  touch.
- **Towel pad:** a plain white 100% cotton handkerchief (about 40 x 40 cm) folded in thirds and then in half twice,
  each way, into a pad about 3 x 3 x 2.5 cm. Its top 1.5 cm is wrapped in one turn of vinyl electrical tape, the only
  part the grippers touch; only the untaped bottom is wetted.
- **Gripper protection:** a strip of vinyl electrical tape over each finger pad of both grippers, renewed each session
  or whenever ketchup reaches it.
- **Tools:** a measuring teaspoon (5 mL), a kitchen scale (1 g steps), a saucer of water, a ruler, masking tape, paper
  towels, a phone on a stand about 60 cm above the plate (overhead photos and video) and one at table height beside the
  pass (side view).

The only liquids are 5 mL of water under the plate, the damp bottom of the pad and three 0.2 mL dots of ketchup; wipe
them up after each trial.

Reset before each trial:

1. Wash and dry the plate and dry the pass. Put a small tape mark on each of the plate's four rim corners, so that
   at least two stay clear of both hands in the overhead view, and lay a ruler on the pass in that view.
2. Wet the pass: pour 5 mL of tap water (one level measuring teaspoon) where the plate stands, set the plate on it and
   slide it twice round a circle about 5 cm across, ending on its tape marks.
3. Slide check, before any ketchup: with nobody holding the plate, set a 130 g weight on the damp pad and draw it
   once along the near rim band by a thread at about 2 cm/s; the plate must slide 1 cm or turn 5 degrees. If it holds, add 5 mL of water and try
   again. If it still holds after this second 5 mL, use a lighter plate within the tolerances above (preferably one
   with a glazed, flat foot) and start again from step 1. Put the plate back on its marks.
4. Sauce: lay the template inside the lip, square with it; put 0.2 mL of ketchup in each hole, spread it with the
   syringe tip to fill the hole, and lift the template straight up.
5. Food: place the muffin with its edge 5.45 cm from the lip's inner face on all four sides (2.1 cm from the inner
   edge of each smudge), measured with the ruler.
6. Towel: rinse the pad's bottom and squeeze it out, then dip the untaped bottom 1 cm into the saucer and blot it (or
   add drops with the syringe) until the pad weighs 30 g (dry, about 15-20 g); put it on its stand.
7. Take the before photo and start the trial 2 minutes after the sauce went on.

The trial ends at the 2,400-step limit, or once all three smudges look clean with the plate within its limits and the
robot has then held the towel off the plate (not touching it, at least 2 cm above the rim) for 1 s, judged in the side
view. The after photo is taken then; at the step limit, the operator first lifts any towel still on the plate straight
up. Scoring uses the simulation's bands:

- **Clean smudge:** no red or pink mark of 2 mm or more (dot, streak or film) within 1 cm of its stencil circle or
  anywhere the towel smeared it, in the zoomed after photo with the ruler in view; any tint is judged against the clean
  plate surface (or a white card laid on the pass) in the same photo. Only sauce removed by the towel pad counts.
- **Plate:** the slide and turn of the line between two corner marks that stay visible, since the towel's first
  touch (1 cm, 5 degrees), and the plate's largest distance from the start (3 cm), read against the ruler in the
  overhead video. A lift of more than 2 cm, judged in the side view against a ruler, scores 0.
- **Food:** 0 if any part of either arm or gripper, or the towel, touches the muffin at any time before the end, or the
  muffin moves relative to the plate (it rides along when the plate slides), judged from the overhead, side and wrist
  videos. The simulation's 1 cm keep-out is a margin for the simulated muffin only.

No force sensor is needed: the ketchup shows whether each smudge was wiped all over, hard enough to lift it.

## Changes in this revision

Changes from the first version (details in `NOTES.md`):

1. **Construct validity.** Each 3 cm smudge is scored over its whole area: 113 sample points, each wiped only once the
   pressed pad has slid 1 cm over it, and a smudge clean at 95%, so rubbing part of it earns nothing (`partial`). The
   food is a collidable muffin stand-in; a contact on the towel or five links of either arm, or a finger pad or the
   towel within 1 cm of any of it, scores 0 (`food`).
2. **Scoring.** Credit needs the full area, pressed with at least 1.2 N (it was 0.8 N, below the 1.30-1.45 N the
   scripted wipe holds); the sauce over wiped points vanishes from view, so the cameras show what is left. A
   successful trial ends only once the towel has been off the plate for 1 s, so it ends with the clean rim in view.
3. **Clarity and reproducibility.** The real trials now name the sauce, its amount, temperature and template (a
   printable 1:1 `template.svg`); the wetting of the pass and the pad and a slide check; the food stand-in and its
   handling and placement; the plate's tolerances, mass and marks; how a trial ends; a numeric clean rule; and the
   gripper protection. The sauce-removal photo
   check is kept.
