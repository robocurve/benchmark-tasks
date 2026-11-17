# Copy a door casing's profile with a contour gauge

A contour gauge is pressed onto a door-casing offcut until its frame seats, lifted straight off with its fingers
holding the copied profile, and stood on the plank across the pencilled cutting line, ready to trace the cut. The gauge
can be steadied with one or both hands; the scorer does not ask which (the scripted demo uses both, one near each end).

**Instruction:** "Copy the casing's profile with the contour gauge, then stand the gauge on the plank across its
pencilled cutting line, set to mark that cutting line, with the copied profile kept."

## Running it

From this folder, on a machine with the official simulator (`yam-isaac-lab-arena`):

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/contour_gauge_casing_profile.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/contour-gauge-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/contour_gauge_casing_profile.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/contour-gauge-demo
```

Runtime per trial: about 2 min for the scripted demo and about 7 min for Dream-1 on one A10 (measured, one trial each, host start-up included).

Seeds: seed 0, object_jitter=0 (fixed layout; the scripted demo uses constants only).

## Why it matters

Fitting skirting board, flooring or a board end to an uneven door casing or wall (scribing) is a routine step for carpenters (SOC 47-2031, 670,090 US workers; BLS OEWS https://www.bls.gov/oes/current/oes472031.htm). The O*NET core tasks (https://www.onetonline.org/link/summary/47-2031.00) include "Measure and mark cutting lines on materials, using a ruler, pencil, chalk, and marking gauge." To measure and mark the cutting line for a board end, the carpenter copies the casing's profile with a contour gauge, then sets the gauge on the board at the line to mark it. One press captures the whole moulding at once and the gauge carries that outline to the board, but its fingers are held only by friction: it must be pressed level and lifted off without a finger moving, or the cut will not fit. The gauge is a USD 20 hardware-store tool (real_trial_cost_usd = 22); the casing and plank are job leftovers.

The O*NET page also lists "Install structures or fixtures, such as windows, frames, floorings, trim, or hardware, using carpenters' hand or power tools."

Real trial equipment: one 10 in plastic contour gauge, USD 21.95 (https://www.penntoolco.com/general-10-contour-gage-can-duplicate-any-shape-833-1/) or USD 19.95 (https://lakefrontsupply.com/general-833-contour-and-profile-gauge-10.html), listed prices before tax and shipping; a casing offcut and a plank offcut are leftovers, not purchases. real_trial_cost_usd = 22, far below the USD 300 cap.

How this differs from nearby tasks (README only): a compass scribe (#1871) follows one wall point by point with a locked compass and draws a pencil line; a bevel gauge (#216) transfers one angle. The contour gauge captures a whole stepped profile in one press, in many independent friction-held fingers, and has to keep that shape through the lift and the carry to the board.

## Sources and model

| part | real (source) | mechanism in the model | status |
|---|---|---|---|
| contour gauge | 833-class plastic-finger gauge: 254 mm (10 in) wide, 76 mm fingers held by friction, 32 mm usable depth (maker-authored brochure mirror https://www.autotoolworld.com/assets/General_Tools/General-Tools-Instruments-833-Contour-Gauge-10-Inch-Brochure.pdf; the 6 in gauge on the same page has other numbers, the 10 in figures are used; the historical maker page https://generaltools.com/angle-izer-contour-gauge-profile-gauge-shape-duplicator-10-inch now redirects to a 6 in listing and is not relied on) | frame 254 x 22 x 10 mm + 31 finger slide joints (axis z) at 8 mm pitch, 76 mm long with rounded tips, 32 mm below the frame when fully out; 100 g in all | sizes sourced; pitch, finger width, mass estimated |
| finger hold | fingers stay where they were pushed, held by friction in the frame | per finger: a drive (200 N/m) held at the latch point, which slips past 0.5 N (the on_step hook moves the latch, Coulomb-like); the same hook runs in Isaac and MuJoCo | 0.5 N = engineering estimate, disclosed |
| casing | colonial door casing 11/16 x 2-1/4 in (17.5 x 57 mm), pine (common stock size; no dimensional drawing of the pattern was found) | loose part, 0.2 m offcut, 0.085 kg, friction 0.4; bead-and-cove face from the inner edge: raised bead 12 mm high x 10 mm, field 6 mm high x 18 mm, cove rising to 12 mm over 12 mm, 5.5 mm step up to the 17.5 mm outer band (17 mm wide) | outside size: stock size; profile: authored outline, every feature >= 5.5 mm high and >= 10 mm wide (sized to the 8 mm finger pitch) |
| plank | 1x6 board, 19 x 140 mm actual, 0.3 m | fixed part, pencilled cutting line across it 30 mm from its end | scene choice |

Layout (fixed): the gauge stands 150 mm nearer the robot than the casing's centre line across the bench; the plank's
far end is 50 mm nearer the robot than the gauge.

Disclosed estimates: finger hold 0.5 N, finger pitch 8 mm (the real gauge has ~1.5 mm fingers: one model finger stands
for about five), finger mass 1.6 g, the profile steps, casing friction.

**Grain, disclosed:** the model has 31 fingers at 8 mm pitch; a real gauge has ~1.5 mm fingers, so the model copies
the profile more coarsely (one model finger stands for about five real ones). Wherever the casing lies fully under
the gauge, 7 or 8 fingers read the 57 mm casing, so the count gate (at least 5) only fails a press with the casing
under the gauge's end. The profile is authored so that every feature (bead, field, cove, outer band) is at least
10 mm wide and every step at least 5.5 mm high: each is read by at least one finger. Each finger is held by 0.5 N
(estimate); the drive's elastic give at that force is 2.5 mm.

**What the profile gates distinguish:** RMS 1.5 mm / worst 3.0 mm / bench spread 2.5 mm are ruler-checkable: lay the
gauge back over the casing and the copy should sit on every feature within about 3 mm. They fail an unpressed gauge,
a gauge pressed flat on the bench or plank, a press tilted past 3 deg, a flat block of the casing's width, and a plain
bevel rising 6 to 17.5 mm (tests). The MuJoCo demo copy reads RMS 0.5 mm, worst 1.3 mm.

**Pressing at one end only:** in MuJoCo, the same plan with one hand parked drives that end down first: the frame lies
8 deg off level during the press (copy line 8.4 deg, RMS 1.6 mm) and the copy fails the 3 deg gate. One hand at the
middle of the spine presses level and scores 1.0; two hands near the ends score 1.0 (RMS 0.5 mm).

**End pose:** the gauge ends standing on its outer fingers (those that rested on the bench during the copy) across the
cutting line with the copied profile kept, clear above the plank, ready to be traced; laying it flat needs a re-grip the
scripted demo does not do. This is a scripted-demo constraint, not a claim that a trace already exists on the board.
Scene positions are fixed (not randomised per
seed) because the scripted demo uses constants only; mj_check jitters the loose parts by 5 mm / 3 deg.

## Scoring (identical to the code's `score()`)

Judged at the end of the trial. 0 unless the copied profile matches the casing laid back against it at some offset and
face (candidates are screened against every gate, then the lowest RMS is taken): at least 5 fingers at least 1 mm onto
the casing within RMS 1.5 mm and worst 3.0 mm of its heights, the bench fingers within 30 mm of its edges within a
peak-to-peak spread of 2.5 mm about one straight line, and that line (how level the frame was pressed) within 3 deg of
level. Every feature of the casing's profile is at least 5.5 mm high and 10 mm wide, so these gates tell a copy from
an unpressed gauge, a flat or tilted press, a flat block and a plain bevel of the same rise. Then 0.4; 1.0 if the
gauge also stands upright (within 3 deg) on the plank across the cutting line: centre within 6 mm of the line and 10 mm of
the plank's centre line, along the line within 5 deg, lowest finger tip within 3 mm of the plank top, and each hand's
tool point more than 12 cm from the gauge centre in plan or more than 15 cm above the plank top. Capped at 0.5 if the
casing's final position is more than 8 mm or its final heading more than 3 deg from where it lay. Possible: 0, 0.4,
0.5, 1.0.

## Scripted demo (open loop, constants only)

1. The hands take the gauge's spine 22 mm in from each end (jaws across the 22 mm frame) and lift it level.
2. They carry it over the casing and press it straight down until the frame seats on the casing's outer band: fingers
   over the casing are pushed back to its profile, those over the bench to the bench.
3. They lift it straight off; the fingers keep the profile.
4. They stand it on the plank across the cutting line, let go and park (8 s).

The demo uses fixed scene constants only; it reads no simulator state.

## Shortcuts that score below 1 (tests and mj_check)

Do nothing (0); a press driven down at one end only (frame 8 deg off level, 0); a press tilted more than 3 deg (0); a
press off the casing (0); the gauge pressed flat on the plank or bench after the copy (0); a flat block or a plain bevel in place of the profile (0); gauge dropped, tilted or off
the cutting line (0.4); casing shoved more than 8 mm (capped 0.5).

## Offline check (no GPU)

`python -m pytest -q test_contour_gauge_casing_profile.py` with the benchmark runtime (`yam_arena`) importable: the
latch, the start (0), the goal (1.0), each partial tier and shortcut, and a regression for the fit selection (a copy
with one valid offset must not be rejected because a different offset has a lower RMS).

## Licence and provenance

labkit.py, arena_assets_fix.py and pitch_demo.py are the benchmark kit's helper modules, copied unchanged next to the
task file; the task code and tests are original to this task. No third-party mesh or texture is used. The
repository licence applies.
