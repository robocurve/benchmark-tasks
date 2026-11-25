# Streak an agar plate in four quadrants, with a fresh inoculating loop for each

An uncovered agar plate rests on the bench in front of the right arm. A marker cross on its underside divides it into
four numbered quadrants, and a 10 µL drop of a bacterial culture sits in quadrant 1. Four sterile disposable loops lie
side by side on the bench beside the plate. The robot must streak the plate for isolated colonies the way the CDC's
four-quadrant method does it with plastic loops: **a new loop for each quadrant, discarded after it**. The first loop
goes through the drop and streaks quadrant 1; each next loop starts with a few strokes that **cross the streaks just
drawn in the quadrant before**, then streaks its own quadrant (quadrant 4 never touching quadrant 1), always with the
ring gliding on the agar, never digging in. Each used loop goes into the discard tray.

Streaking for isolation is the standard way a microbiology, clinical or research lab gets a pure culture from a
sample (CDC, "Quadrant streaking for isolation" job aid and its 4-quadrant streak training video; Sanders 2012,
"Aseptic laboratory techniques: plating methods", J. Vis. Exp. 63, e3064). The dilution happens at the crossings: a
fresh loop carries no bacteria, so the only cells in quadrant 2 are the few it picks up where it crosses quadrant 1's
streaks, and so on, until single colonies grow in quadrants 3 and 4. The CDC training says it plainly: "If you are
using a plastic loop, get a new loop before streaking each quadrant and discard into a sharps biohazard container
after streaking each quadrant." The difficulty is physical: four times, a slim handle has to be picked up off the
bench and held like a pen, and the 4.5 mm ring drawn over a soft gel in long strokes with almost no force (held off
the surface it draws nothing; pressed past about a quarter of a newton it tears the agar), and each new loop has to
find and cross the thin streaks the previous one left.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the streak record, `score()`, `success()`, `self_test()` and the scripted `demo()` |
| `contactkit.py` | Reads the PhysX contact force between each loop and the agar |
| `touch.py` | Reads whether any robot link touches the agar or a loop (for "let go") |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `tiltkit.py` | Any gripper orientation for the scripted demo (the loop is held tilted, like a pen) |
| `assets/plate.usdc` | The Petri dish base (92 mm outside, 88 mm inside, 13.5 mm tall), a fixed body, 23 KB |
| `assets/agar.usdc` | The agar (88 mm across, 3.3 mm deep), with the quadrant cross, quadrant numbers (dots) and the drop's dot drawn on it, a fixed body, 76 KB |
| `assets/loop.usdc` | A 10 µL loop (200 mm): its handle and, on a sprung hinge at the neck, the neck and ring (two links of a free articulation); used four times, 21 KB |
| `assets/height_cal.npz` | The scripted demo's calibration, per loop: how far the ring rides above or below its planned height over the plate, and how far sideways it lands from where it is aimed (measured by running the plan clear of the agar), a few KB |

| `tools/make_cal.py` | Builds `assets/height_cal.npz` from a calibration run (see "Re-measuring the demo's calibration") |
| `tools/log_streak.py`, `tools/latch_cam.py`, `tools/frames_to_mp4.py` | The demo video's extra views (four close-up cameras and the streak map; not policy cameras) and the video assembly |

No third-party assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top. Same layout every trial.

| Object | Where | What |
|---|---|---|
| Agar plate | centred at (34.0, -6.0), on the table, held by a 2 cm loop of double-sided tape under the dish (it doesn't move in the simulation); its lid off for the trial, set aside off the work area with its inner face down (not in the scene) | Fisherbrand FB0875712 polystyrene Petri dish base, 100 x 15 mm style: 92 mm outside, 88 mm inside (Fisher's product note), 13.5 mm tall; holding 20 mL of LB agar (1.5 % agar), 3.3 mm deep, poured level the day before, set, stored lid-down at 4 °C and warmed to room temperature for 1 h before the trial, with no surface water |
| Quadrant marks | on the plate's underside | A cross in black marker through the centre, one line along x and one along y, dividing the agar into quadrants 1 (far right: x > 0, y < 0 from the plate's centre), 2 (far left), 3 (near left) and 4 (near right), numbered by 1 to 4 dots near the rim; a filled blue dot 5 mm across marks where the drop goes |
| Sample drop | quadrant 1, on the blue dot, centred 1.6 cm forward of and 1.4 cm right of the plate's centre | 10 µL of an *Escherichia coli* K-12 culture (for example MG1655; biosafety level 1) grown 16-18 h at 37 °C in LB broth, shaking, pipetted onto the agar with a 2-20 µL pipette just before the trial (a wet patch about 5 mm across) |
| Loops | four, lying flat on the sterile sheet along x, ring towards the robot, side by side 3.5 cm apart: ring centres at (20.2, -16.5), (20.2, -20.0), (20.2, -23.5) and (20.2, -27.0), butts 19.8 cm further forward | Sterile disposable inoculating loops, 10 µL, polypropylene, 200 mm long, 3.6 mm handle, ring 4.5 mm across (MTC Bio M4025 or Heathrow Scientific 03-448-345), laid out from their pack onto the sheet just before the trial |
| Sterile sheet | under the loops, centred at (30.0, -21.75), long side along x, its corners taped down | Autoclaved white paper sheet, 24 x 15 cm, 0.3 mm thick: the loops' rings touch nothing but the sheet before use. Laying out the loops: each ring's centre 2.2 cm in from the sheet's near (robot-side) edge; the first loop 2.25 cm in from the sheet's left long edge (+y), the others every 3.5 cm towards the right |
| Discard tray | centred at (16.0, 5.0), between the arms, its long side along x, taped down | Autoclavable polypropylene biohazard tray, 25 x 18 cm, 4 cm deep, lined with an autoclave bag (an open container standing in for the CDC's sharps biohazard container, so the loops can be laid in) |

Both arms start in their ready pose with their grippers open. Nothing else is on the table.

## Scoring

**What is read, and how.** After the trial the plate's lid goes on and the plate is incubated upside down at 37 °C for
18-24 h. Colonies then grow exactly where a ring rested on the agar: a ring held off the surface leaves nothing, a
ring resting on it leaves a line of growth, and a cut in the agar shows as a groove. The incubated plate is
photographed from above on a light box beside a millimetre ruler. A camera filming the plate from above during the
trial (a phone on a stand about 30 cm over it, 1080p, 30 frames a second) shows which loop drew which strokes and
where each ring touched down.

- A **streak** is a line of growth along a stroke: a continuous line, or (where the bacteria have thinned out, as
  they should by quadrants 3 and 4) a row of colonies no more than 5 mm apart along the stroke, measured from its
  first colony to its last (sim: agar the ring rested on with at least 0.01 N, once the loop carries bacteria).
  Lengths are measured along the lines on the photo with the ruler (a thread laid along curved lines).
- **Each quadrant starts by crossing the one before.** A fresh loop carries no bacteria until it has passed through
  the drop (loop 1) or crossed the previous loop's streak (loops 2-4), so anything it draws before then grows nothing
  and doesn't count; in the simulation a loop's streak length is counted only from that moment.
- **Loops are counted in the order they first touch the agar**: loop *k* streaks quadrant *k* (video).
- A **crossing** is a stroke of loop *k* that passes through growth left by loop *k − 1*: on the photo, a line of
  growth that runs out of loop *k − 1*'s growth (which is often a solid lawn in quadrants 1 and 2) into quadrant *k*;
  the plate camera shows which strokes they were (sim: the ring of loop *k* comes onto agar that loop *k − 1*
  streaked; two crossings are separate when the ring travels at least 2 mm off that streak between them). Crossing
  the marker line where the previous quadrant has no growth does not count, and gives no growth: a fresh loop that
  never crosses the previous streak leaves its quadrant bare.
- Only lines of growth along streaks are measured: an isolated colony elsewhere (an airborne contaminant while the
  lid was off) is ignored.
- A **gouge** is agar torn or cut into anywhere: a groove in the photo (sim: a ring pressing on the agar with more
  than 0.25 N, the agar's gel strength over the ring's contact patch; see below).

Score:

- **0** if the agar is gouged anywhere, or a gripper touches the agar (video).
- Otherwise 0.25 for each quadrant streaked in turn, stopping at the first that fails:
  - **Quadrant 1** (0.25): loop 1 passes through the drop (over the blue dot) and draws at least 40 mm of streak in
    quadrant 1 from there.
  - **Quadrant 2** (0.5): a fresh loop 2 crosses loop 1's streaks at least twice and, from its first crossing on,
    draws at least 30 mm of streak in quadrant 2.
  - **Quadrant 3** (0.75): a fresh loop 3 likewise: at least 30 mm in quadrant 3, crossing loop 2's streaks at least
    twice.
  - **Quadrant 4** (1.0): a fresh loop 4 likewise: at least 30 mm in quadrant 4, crossing loop 3's streaks at least
    twice, never touching quadrant 1; and every loop that touched the agar lying in the discard tray (all of it
    inside the tray's walls), let go, for 1 s.
- **Each loop stays in its place**: loop *k* may touch only quadrant *k* and the quadrant before it (where it crosses
  the previous streaks; contact within 2 mm of the marker line counts as on the line). A loop streaking into a later
  quadrant ends the credit before that quadrant (it carried undiluted bacteria there); a loop streaking into an
  earlier quadrant other than the one before ends the credit before its own. A loop that touches the agar again
  after a newer loop has touched it ends the credit after the quadrants already begun. So one loop used for every
  quadrant scores at most 0.25.
- At most **0.25** if any loop after the first touches the drop (re-inoculating).
- At most **0.75** until every loop that touched the agar lies in the discard tray, let go.
- Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Only the end of the trial counts, apart from the conditions marked "at
  any time" by their nature (a gouge, a gripper on the agar, the order of the loops and their strokes).

**Checking the gouge readout** (once per session, with a spare plate from the same batch): rest a spare loop's ring
on the agar, held level at its usual angle, with a 30 g weight on the handle over the ring (0.29 N, just over the
limit below) and drag it 2 cm; a groove must show on the light box. Drag it again with only a 5 g weight (0.05 N, a
gentle streak): no groove may show. If either check fails, the batch's agar is too soft or too firm: pour a fresh
batch.

The gouge limit: suppliers give 550-950 g/cm² as the gel strength of 1.5 % bacteriological agar (Thermo Fisher
J10906, Avantor, Neogen); at the lowest, 550 g/cm² over the ring's contact patch (its 0.8 mm strand resting along
about half its 4.5 mm round, 0.045 cm²) is 25 g, so the simulation counts a contact force over 0.25 N as a gouge.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Streak the plate for isolated colonies, using a new loop for each quadrant: with the first loop, drag through the drop and streak quadrant 1; then with a fresh loop each time, streak quadrants 2, 3 and 4 in turn, each starting with a few strokes that cross the streaks of the quadrant before. Don't dig into the agar. Put each used loop in the discard tray." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/streak-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Seed 0; the
scene has no random placement (`-E object_jitter=0`). On an RTX 3060 the scripted demo (about 130 s of simulated
time) takes about 10 minutes including start-up; a full 3,600-step trial about 13 minutes.

Offline checks (no GPU): `python task.py` runs the scoring rules on hand-made streak sequences (26 cases, among them
one loop used throughout, a loop re-used, strokes that cross the marker line but not the previous streak, a loop
that streaks its quadrant before crossing, a skipped quadrant, a re-dip, and quadrant 4 running into quadrant 1),
checks that the tilted loop's shaft clears the dish's rim wherever the ring works, and replays the demo's plan
through the arm's kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=2 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 --instruction "..." \
  --scorer task_score --max-steps 2700 --store-frames --log-dir ~/yam-eval/streak-demo
```

For each quadrant in turn the right arm picks the next fresh loop up by its handle, tilts it down to 50 degrees like
a pen and streaks at 20 mm/s with the ring resting on the agar: loop 1 through the drop and zigzagging over quadrant
1; loops 2, 3 and 4 each starting with three strokes 8 mm back across the line, through the previous quadrant's
streaks, then zigzagging over their own quadrant (quadrants 2 and 3 finish with a stroke down their far side, where
the next loop will cross). After each quadrant the loop is lifted, carried over the discard tray, turned level and
let go into it. `STREAK_LOG=1` prints which loop is on the agar, the force, the streak lengths and crossings per
quadrant every 0.5 s.

The demo video: top row, the policy cameras (left wrist, top, right wrist); bottom row, demo-only views that are
not policy cameras: the streak record seen from above (agar streaked by loop 1 in blue, loop 2 red, loop 3 green,
loop 4 purple; where two colours meet, a loop crossed the previous loop's streak; x points up the panel, y to the
left) and four close-up cameras round the plate. The demo's plan comes with a per-loop calibration
(`assets/height_cal.npz`: each loop's height error over the plate and its sideways offset, measured by running the
plan clear of the agar), because each loop sits a little differently in the hand.

### Tested with

yam-arena 0.1.0, inspect-robots 1.0.0, Isaac Lab 17.0.2 on Isaac Sim 6.1.0, an NVIDIA RTX 3060 (driver 580).

### Re-measuring the demo's calibration

The scripted demo aims each loop using `assets/height_cal.npz`, measured on the machine above. On a different GPU or
version the arm can sag differently; if the demo then draws too lightly or too hard (`STREAK_LOG=1` shows the
force), re-measure it from this folder: run the plan 22 mm up, clear of the agar, recording each ring's track, then
rebuild the file (about 12 minutes):

```bash
STREAK_LIFT=0.022 STREAK_NOCAL=1 STREAK_TRACE=/tmp/streak_trace.npy inspect-robots run --embodiment yam_arena \
  --policy yam_scripted -P hold_s=2 -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "..." \
  --scorer task_score --max-steps 2700 --log-dir /tmp/streak-cal
python tools/make_cal.py /tmp/streak_trace.npy 0.022 0.002
```

### The demo video's extra views

```bash
LATCH_FRAMES=/tmp/streak-views inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=2 \
  -E task=tools/log_streak.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "..." --scorer task_score --max-steps 2700 --store-frames --log-dir /tmp/streak-media
LABELS=1 python tools/frames_to_mp4.py /tmp/streak-media/frames/* demo.mp4 20 /tmp/streak-views
```

## Reset

Each trial uses a fresh agar plate (marked, with a new drop of the culture), four fresh sterile loops laid out on a
fresh autoclaved sheet, and an emptied tray; the plate is then lidded and incubated for scoring, and autoclaved with
the used loops in the tray's bag. Label each plate with its trial number on the underside rim before the trial;
plates from many trials incubate together and are photographed the next day. The culture is biosafety level 1: wear gloves and a lab coat, and wipe the bench
with 70 % ethanol before laying out the sheet and afterwards.
