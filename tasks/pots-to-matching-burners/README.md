# Put each pot on the burner that fits it

A black glass cooktop has two burner rings printed on it: a large one (21.2 cm across) on the right and a small one
(14 cm) on the left. A large steel pot sits on the small burner and hides it; it weighs 1.5 kg and has a handle on each
side, pointing left and right. A small copper pot (0.35 kg, no handles) waits on the counter to the right of the
cooktop. The instruction: "Someone has put the big pot on the small burner. Put each pot on the burner that matches its size. Lift the pots: never slide a pot across the glass cooktop."

Cookware guidance gives both rules the task scores. The
[Le Creuset care and use guide](https://www.lecreuset.com/care-and-use.html) says never to slide its cookware on a
glass-topped cooktop but always to lift it on and off, and to use a burner similar in size to the base of the pan. The
[Kaizen Cookware buyer guide for glass-top stoves](https://kaizencookware.com/blogs/news/best-cookware-for-glass-top-stove)
says to lift pans instead of sliding them (most scratches come from dragging pans or debris trapped under the base)
and to match the pan size to the burner so heat is not wasted. The task tests four things a pick-and-place policy can get wrong:

- **Matching by size.** The two pots must go to different burners chosen by comparing sizes. The large burner is
  free at the start, so putting the nearby small pot on it is the easy mistake (0 for that pot).
- **Order.** The small burner is under the large pot, so the large pot has to move first.
- **Bimanual by necessity.** The large pot weighs 1.5 kg and has a handle on each side; the scripted demo lifts it
  with both arms, one per handle. In the recorded `one_hand` variant the right arm alone moved it by one handle: the
  pot slid across the glass (`slid_on_glass` 0.2528 m, about 25 cm) and did not tip (tilt 0.0 degrees at the end,
  and it never passed 30 degrees), and the run scored 0.0. The small pot (0.35 kg, no handles) is moved with one arm,
  after the large pot.
- **How, not only where.** A pot slid on the glass (its centre more than 3 cm from where it touched down, before it is
  lifted clear again) scores 0, even if it ends on the right burner. So does a pot tipped past 30 degrees or dropped.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (visual-only parts for the burner rings), and per-step readings: finger-pad points, part poses, and the drop and stillness watches |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants (`democore.py` is shared with our other kitchen tasks); `demo.py` also writes `setup.png` |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold, position, mass or the sliding rule no longer matches the text) |

`kitchenkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Someone has put the big pot on the small burner. Put each pot on the burner that matches its size. Lift the pots: never slide a pot across the glass cooktop." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/pots-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the scripted demo ends at step 637 (31.85 s of simulated time at 20 Hz). Wall-clock time per trial was
not recorded with these runs. A model trial runs up to 2,400 steps, plus the policy's own response time. Measured on Nov 15: the Dream-1 run (`dreamlabs/dream-1-beta`, 2,400 steps, `rows_per_request=12`) took 7.0 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                            # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_one_hand --variant one_hand     # any variant below; --no-record skips the video
python demo.py task.py --plan-only                         # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out              # also print both pots' state every 10 steps
```

Both arms pinch the large pot's two side handles (each across its 2 cm width), lift the pot 4 cm off the small burner,
carry it to the large burner and set it down, pressing about 8 mm past contact so that the pot seats flat before they
let go. Then the right arm pinches the small pot's near rim on the counter (1.5 cm below the rim, the fingers on
either side of the wall), lifts it until its base is 11 cm above the glass, clear of the large pot's 9 cm rim, carries
it across to the small burner and sets it down. The plan is 35.6 s (712 steps) long, but the recorded trial ended on
success at step 637 (the trial stops once `success()` has held for 1 s). The failure variants ran to the end of their
plans.

| Variant | What it does | Score |
|---|---|---|
| `full` | both arms lift the large pot by its two handles and carry it to the large burner; then the right arm lifts the small pot over it to the small burner | **1.0** (success at step 637; the pots' centres 9.5 mm and 1.8 mm from their burners' centres; on-glass travel 2.1 cm and 3.1 mm) |
| `drag` | both arms take the large pot by its handles and slide it across the glass to the large burner; then the small pot as in `full` | 0.0 (the large pot slid 22.5 cm; it still ends on the large burner, 9.8 mm off, and the small pot on the small burner) |
| `one_hand` | the right arm alone takes the large pot by its right-hand handle and follows the `full` lift and carry; then the small pot as in `full` | 0.0 (the large pot slid 25.3 cm across the glass and did not tip: tilt 0.0 degrees at the end, never past 30; it ends 21.3 mm from the large burner's centre, outside the 15 mm) |
| `one_only` | moves only the large pot, with both arms as in `full`; the small pot stays on the counter | 0.5 (the large pot 9.5 mm off the large burner's centre; the small pot still on the counter, 46.0 cm from the small burner) |
| `wrong` | the right arm puts the small pot on the large burner, which is free; the large pot stays on the small burner | 0.0 (each pot on the other's burner: the small pot 1.8 mm off the large burner's centre, the large pot 0.0 mm off the small burner's) |
| `drop` | both arms let go of the large pot 6 cm above the large burner; then the small pot as in `full` | 0.0 (dropped; it lands upright, 5.7 mm off the large burner's centre, and the small pot ends on the small burner) |

End state of each run, from `report()` at its last step (distances are from each pot's centre to the burner's centre;
"slid on glass" is the largest distance a pot's centre got from a touch-down point while it stayed on the glass; the
small pot's tilt is 0.0 degrees in every run):

| Variant | Score | Success | Steps | Large pot to large burner (m) | Large pot to small burner (m) | Large pot tilt (deg) | Large pot slid on glass (m) | Small pot to small burner (m) | Small pot to large burner (m) | Small pot slid on glass (m) | Matched | Released | Still | Slid | Tipped | Dropped |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `full` | 1.0 | yes | 637 | 0.0095 | 0.2363 | 0.0 | 0.0210 | 0.0018 | 0.2368 | 0.0031 | 2 | yes | yes | – | – | – |
| `drag` | 0.0 | no | 638 | 0.0098 | 0.2252 | 0.0 | 0.2252 | 0.0019 | 0.2368 | 0.0018 | 2 | yes | yes | `large_pot` | – | – |
| `one_hand` | 0.0 | no | 712 | 0.0213 | 0.2229 | 0.0 | 0.2528 | 0.0020 | 0.2356 | 0.0024 | 1 | yes | yes | `large_pot` | – | – |
| `one_only` | 0.5 | no | 380 | 0.0095 | 0.2363 | 0.0 | 0.0210 | 0.4604 | 0.2259 | 0.0000 | 1 | yes | yes | – | – | – |
| `wrong` | 0.0 | no | 412 | 0.2350 | 0.0000 | 0.0 | 0.0000 | 0.2342 | 0.0018 | 0.0010 | 0 | yes | yes | – | – | – |
| `drop` | 0.0 | no | 664 | 0.0057 | 0.2407 | 0.0 | 0.0210 | 0.0021 | 0.2359 | 0.0011 | 2 | yes | yes | – | – | `large_pot` |

The numbers are from `demo_results.json`.

## Scoring

Every step, from poses; the score at the end counts.

- **On a burner:** the pot's centre within 15 mm of the burner's centre, within 10 degrees of upright, base within
  5 mm of the glass.
- **0.5 for each pot on the burner that matches its size** (large on large, small on small), counted once both pots
  are released (no finger pad within 8 mm of either pot, the large pot's handles included) and both still (each
  moving less than 2 mm over 1 s). **1.0** for both.
- **On the glass:** a pot is on the glass while its centre is over the cooktop and the lowest point of its base's rim
  is within 3 mm of the glass. The lowest point allows for tilt: it is the base's height minus the pot's outer radius
  times the sine of its tilt, so a pot tilted up onto its rim is still on the glass.
- **Slid:** each time a pot comes onto the glass, the position of its centre is noted. If its centre then gets more
  than 3 cm (horizontally, in a straight line) from that position before the pot leaves the glass again, the pot was
  slid. Lifting a pot more than 3 mm clear of the glass ends the stretch, so a lifted carry is not sliding, and small
  shifts in separate stretches do not add up.
- **0** if a pot was ever slid, tipped past 30 degrees, or dropped (it falls 5 cm within 0.5 s with no finger pad
  within 2 cm of it).

In `full`, `one_only` and `drop`, which share the two-handed pick-up, the large pot's `slid_on_glass` is
0.021 m, 9 mm under the 3 cm limit; the small pot's is at most 0.0031 m in any run.

`report()` returns, for each pot, its distance to each burner's centre (m), its tilt (degrees), its base's height
above the glass (m), whether it is on its matching or on the wrong burner, and `slid_on_glass` (m); plus the number of
pots on their matching burners, whether both are released and still, and the lists of pots slid, tipped or dropped.

## Real-world trials

Use two empty pots of about these sizes, the large one with two side handles and about 1.5 kg, and a glass-ceramic
cooktop that is switched off, or a black board with the two rings drawn on it (the cooktop is never heated). Score
from an overhead camera with the same thresholds, from a marker on each pot's rim. The sliding rule also needs to
know when a pot is on the glass: from a side camera, check whether the lowest point of the pot's rim is within 3 mm
of the glass, and from the overhead camera, how far the pot's centre moves from where it touched down while it stays
there.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Simulation fidelity.** Rigid bodies: boxes. The burners are rings printed on the glass (visual only) and there is no heat. Scratching is not simulated, so sliding is judged from the pots' motion: how far a pot's centre moves from where it touched down while the lowest point of its rim stays on the glass. The pots are empty. The large pot (1.5 kg) has two rigid side handles and the small pot (0.35 kg) none; the demo lifts the large pot with both arms, one per handle, and pinches the small pot by its near rim with the right arm. In the recorded one-handed run (the right arm alone, by one handle) the large pot slid about 25 cm across the glass and did not tip (tilt 0.0 degrees at the end), and scored 0. Each pot's outside diameter is within 6 mm of its matching ring's, so the match is visible to the cameras; the large pot hides the small ring at the start, and the instruction says where it is.
