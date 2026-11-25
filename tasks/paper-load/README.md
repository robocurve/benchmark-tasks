# Refill a home printer that has run out of paper: stand five sheets in its rear tray, guide snug, press OK

A Canon PIXMA TS3720 all-in-one printer stands on an IKEA LACK side table in front of the robot, whose arms stand on
pedestals in a floor bay. The printer is on, its display showing support code 1000 (out of paper): it ran out
mid-job, so its rear tray is empty with the paper support still raised, and the paper guide is at its widest. Five
fresh sheets of Letter copy paper lie on the printer's shut document cover, their near edge hanging 40 mm over the
cover's front edge, their right edge 15 mm left of the rear tray's right side. The instruction is:
**"The printer is out of paper. Load the fresh sheets so it can print again."**

To refill it the robot has to:
1. take the five sheets by the edge that hangs over the cover and lift them off;
2. carry them hanging over the printer and lower them upright into the rear tray behind it, down on its floor and
   against its right side, so that they lean back on the support;
3. slide the paper guide against their left edge;
4. press OK, which clears code 1000.

Paper that stands in the tray but not down on its floor, not against its right side, or with the guide left wide
looks loaded, but then feeds skewed or jams. The trial is scored by the method a person would use: a 0.5 mm feeler
leaf at the tray's right side and at the guide, the printer's display, and a test page it prints afterwards.

## Why it matters

Most people who use a home printer have refilled it. In YouGov's US poll of August 2026 (2,144 adults), 87 % had
used a home printer and 54 % use one now. Canon's TS3700-series online manual (ij.manual.canon) makes loading the
rear tray the owner's job:
- "Loading Plain Paper / Photo Paper": "Open paper guide." "Load paper vertically against
  the far right of the rear tray WITH THE PRINT SIDE FACING YOU." "Adjust the paper guide with the paper left side."
  "Always load paper in portrait orientation. Loading paper in landscape orientation can cause paper jams."
- Support code 1000: "There is no paper in the rear tray. Paper is not loaded in the rear tray properly." "Load
  paper in the rear tray." "Align the paper guides of the rear tray with both edges of the paper." "After carrying
  out the above measures, press the printer's OK button to cancel the error."
- "Front View": "Paper Support: Open to load paper on the rear tray." "Paper Guide: Align with the left side of the
  paper stack."

## What makes it hard

**A limp stack, carried by one edge.** Five sheets weigh 22.5 g and bend under their own weight. Lying flat on the
cover they can't be picked up by their faces, so they are taken by the edge that hangs over the cover's front.
Held there, they hang 279 mm down, so the hand has to carry them high over the printer and bring them down from above
into the rear tray, a narrow slot behind the document cover, in front of the raised support.

**Square in a slot it can't see into.** The stack starts 15 mm to the left of where it must end up. It has to go
down onto the tray's floor with its right edge against the tray's right side: a stack lowered a few millimetres to the
left, or held off the floor, looks loaded but feeds skewed. Then the guide, a slider in the slot, has to be closed onto
the stack's left edge without pushing the stack off the right side or knocking the raised support back down.

## The printer, checked against Canon's TS3700-series manual

| Item | What the manual says | In the task |
|---|---|---|
| Paper support | "Paper Support: Open to load paper on the rear tray." The TS3720 has no separate feed-slot cover: the support is the flap over the rear tray | A plate on a hinge along the housing's back; raised at the start (the printer ran out mid-job), it leans back past upright on its stop |
| Rear tray | "Load paper vertically against the far right of the rear tray"; "Always load paper in portrait orientation" | A slot behind the document cover's hinge, with a floor and a right side |
| Paper guide | "Adjust the paper guide with the paper left side. Do not slide the paper guide too hard against the paper." | One guide, a slider in the slot closing from the left, with the raised grip of Canon's figure on top, grey |
| OK button | Code 1000: "press the printer's OK button to cancel the error" | A sprung key on the operation panel, beside Paper Select |
| Code 1000 | "There is no paper in the rear tray. Paper is not loaded in the rear tray properly." | Shown at the start; cleared by OK pressed with the stack in the tray |
| Size, mass | 435 x 327 x 145 mm (trays retracted), 4.0 kg | as stated |

## Files

| File | What it is |
|---|---|
| `task.py` | the yam_arena task file: objects, the monitor and scoring (the stack against the tray, the guide, OK, release) |
| `geometry.py` | every size and the layout, the scoring tolerances |
| `build_assets.py` | writes `assets/*.usda` and the panel's textures from `geometry.py` (needs `pxr` and PIL, not the simulator) |
| `tests/test_scoring.py` | scoring checks without the simulator (yam_arena's IK and forward kinematics place the grippers' collision hulls) |
| `scenekit.py` | USD bodies and articulations for task files, the floor bay, and the physics-rate hook |
| `demokit.py`, `demo.py` | the scripted demo and the scorer checks in simulation |

All assets are built from scratch in `build_assets.py` (no third-party meshes, scans or textures). The robot is
yam_arena's.

**Font.** The lettering on the display in the panel texture (`assets/textures/pl_panel_1.png`) is drawn in DejaVu Sans
Bold (`DejaVuSans-Bold.ttf`, DejaVu fonts 2.37, Ubuntu's `fonts-dejavu-core` package). Only these rendered glyphs are
in the repository, not the font file. Its license (https://dejavu-fonts.github.io/License.html) is the Bitstream Vera
Fonts license, free to use and redistribute: "Copyright (c) 2003 by Bitstream, Inc. All Rights Reserved. Bitstream
Vera is a trademark of Bitstream, Inc. DejaVu changes are in public domain." The glyphs DejaVu imported from the Arev
fonts carry the Arev Fonts license, but none of them are drawn here.

## Run a trial

```
source <ARENA_DIR>/yam.env
python build_assets.py                                    # once
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    --scorer task_score --max-steps 2000 --policy ...
python tests/test_scoring.py                              # the scoring checks (no simulator)
```

No seeds are used: the layout is the same in every trial. On one GPU shared with other jobs (240 Hz physics, three
cameras at 640 x 352) the demo's 1,006 steps took 3.5 minutes of wall time, so a full 2,000-step trial takes about 7
minutes at that load, plus about 2 minutes to start. `success()` turns true 1 s after the 1.0 state, which ends the
trial; otherwise it runs to the step limit.

## Dream-1 run

Dream-1 (dreamlabs/dream-1-beta), run once with the task's exact instruction, `object_jitter=0`, 20 Hz and the
2,000-step limit, scored 0. Its hands moved over the printer's front and its document cover, the right one over the
sheets, but it never lifted them, so the rear tray stayed empty and code 1000 stayed on. `model_run.mp4` shows the
whole run (top, left arm and right arm cameras).

## Scripted demo

```
python demo.py task.py demo_out [--no-record] [--hold S] [--variant guide_wide|no_ok|nothing]
```

The demo reads object poses from the simulator (privileged: the stack's strips, the guide's travel, OK's travel). In
order (steps of the recorded run at 20 Hz):

1. **Take the paper** (right arm, to step 118). The gripper, its approach pitched 20 degrees down, opens 22 mm, goes
   over and under the stack's near edge where it hangs 40 mm over the cover's front, and closes on it 12 mm in from
   the edge: the pads stop 2.2 mm apart.
2. **Over the rear tray** (to 238). Up 60 mm, then back over the printer with the tool point 450 mm above the table,
   the hand pitching to 60 degrees down, so the stack hangs clear of the printer, in front of the raised support.
3. **Load** (to 722). The stack's lower edge is lined up over the slot, level (the hand turned about x), its right
   edge 3 mm from the tray's right side, 12 mm to the right of where it started; lowered to 8 mm above the housing and
   lined up again (2.0 mm from the side), then lowered into the slot at up to 16 mm/s until its lowest corner is
   0.69 mm above the floor. The held edge is carried back on an arc about the lower edge, from 3.5 degrees forward to
   12 degrees back, then down until the stack is on the floor again and along the slot against the right side. The
   hand opens and backs away; the stack falls back onto the support and settles on the floor (by step 760), its
   lowest 93 mm rising 77.6 degrees, 0.25 mm from the right side, its top bent back over the support.
4. **Guide** (left arm, to 893; the right arm goes home). The closed fingertips, the hand leaning 25 degrees toward
   the printer, go down beside the guide's raised grip and push it 44 mm toward the paper until it meets the stack's
   left edge, then on, slowly, until the stack is 0.13 mm from the right side. The hand eases off 2 mm and rises.
5. **OK** (step 963). The left fingertip pushes OK in 1.36 mm, once, and the left arm goes home.

**Result: 1.0** at step 987, the run ending 1 s later at step 1,006. Run twice from the same code, each in a new
process: every logged number (the pinch, the carry, the line-up, the lean, the load, the guide, the press, the end state
and the step counts) came out the same. `demo.mp4` is the first of the two.

| Check | In the demo | Needed |
|---|---|---|
| stack in the tray | lower edge in the slot, its lowest 93 mm rising 77.6 degrees | lower edge in the slot, rising at least 50 degrees |
| down on the floor | 0.0 mm | within 3 mm |
| right edge to the tray's right side | 0.13 mm | 0.5 mm or less (the feeler leaf can't pass) |
| guide to the stack's left edge | against it (0.19 mm past its leftmost corner) | 0.5 mm or less |
| OK | pressed once, with the stack in the tray | pressed with the stack in the tray |
| paper support | on its stop, 110 degrees, throughout | past upright (95 degrees) |
| released | both hands clear, their collision hulls more than 2 mm from the printer and the paper in the final 1 s | yes |
| the stack's largest hinge bend | 62.1 degrees (over the support's top) | under 150 (folded) |

**Scorer checks in simulation** (`demo.py --variant NAME`, run one after another in one process from fresh resets;
the demo's code, each from the start state):

| Run | What the script does | Score (what the scorer read) |
|---|---|---|
| demo | as above | **1.0** (every logged number as in the recorded run) |
| check: guide left wide | the demo without the guide step | 0.5 (loaded: on the floor, 0.32 mm from the right side; the guide 43.8 mm from the stack) |
| check: OK not pressed | the demo without the press | 0.75 (loaded, the guide against the stack, the support up; OK never pressed) |
| check: nothing | the arms stay still, as after a refusal | 0 (the stack still on the cover, 15 mm from the right side) |
| demo again | the demo, in the same process after the checks above (a reset there is not bit-identical) | 1.0 (972 steps; the stack 0.05 mm from the right side) |

**Scorer checks without the simulator** (`python tests/test_scoring.py`, 9 of 9 pass): task.py's monitor fed
hand-built trials step by step (the strips' poses, the guide's travel, the support's angle, OK's travel, the
grippers' gap from their collision hulls placed by yam_arena's IK and forward kinematics):

| Trial | Score |
|---|---|
| loaded against the right side, guide snug, OK pressed, support raised, hands at the ready pose | 1.0 |
| guide at its widest; 1.0 mm off the stack's edge; 0.4 mm off | 0.5; 0.5; 1.0 |
| the stack 1 mm off the right side | 0.25 |
| the stack 5 mm above the tray's floor | 0.25 |
| OK not pressed; OK pressed before the paper was loaded | 0.75; 0.75 |
| the paper support shut at the end | 0.75 |
| nothing done (the stack on the cover); a hinge bent to 160 degrees | 0; 0 |
| a fingertip against the guide's grip at the end | 0.75 |
| the hands at the ready pose are clear of the printer and the paper | (more than 50 mm) |

## Scoring a real trial

Everything is read from the rear tray, the printer's display and a test page, with a 0.5 mm feeler leaf (from a
feeler gauge set) and a phone camera.

- **Loaded.** Photograph the tray from above and from the side. The stack stands portrait in the rear tray, its lowest
  93 mm at least 50 degrees from level, its lower edge down on the tray's floor (no visible gap; within 3 mm), and the
  0.5 mm leaf can't be slid down between the stack's right edge and the tray's right side.
- **Guide snug.** The 0.5 mm leaf can't be slid down between the guide and the stack's left edge.
- **Accepted.** OK was pressed (in the video) with the stack in the tray, and code 1000 is gone from the display.
  After the trial, print a nozzle check (Setup, then Black): it feeds from the stack and prints without a jam (no
  code 1300). If OK was pressed but code 1000 is still shown, or the nozzle check jams, the trial is not accepted.
- **Support.** The paper support is still past upright (95 degrees).
- **Released.** In the final second of the video, neither gripper touches the printer or the paper.
- **The paper.** Check the sheets for a crease or a tear.

The score:
- 0: the stack is not in the tray (doing nothing included), or a sheet is folded or torn.
- 1.0: loaded, guide snug, accepted, support past upright, released.
- 0.75: loaded and guide snug, but not accepted, the support no longer past upright, or a gripper touching. Not
  accepted covers OK never pressed with the paper in, OK pressed but code 1000 still shown, and a nozzle check that
  jams.
- 0.5: loaded, but the guide not snug.
- 0.25: in the tray, but not down on its floor or not against its right side.

## Reset

Take the sheets out of the rear tray and square them (replace any that crease); lay them on the shut cover at the tape
marks; leave the paper support raised; open the guide to its widest. With the tray empty, start a print (Setup, nozzle
check) so the display shows code 1000 again. About 2 minutes. Each trial's test page uses one sheet and a little ink
(about $0.05).

## Real build

| Item | Product | Size, mass | Price |
|---|---|---|---|
| Printer | Canon PIXMA TS3720 Wireless All-in-One (6671C022), white | 435 x 327 x 145 mm, 4.0 kg (Canon) | $64.99 (Staples) |
| Ink | the PG-275 and CL-276 cartridges it comes with | | about $0.05 a test page |
| Side table | IKEA LACK side table, white | 55 x 55 x 45 cm | $16.99 (IKEA) |
| Copy paper | 20 lb (75 g/m2) Letter, any ream | 215.9 x 279.4 mm, 4.5 g a sheet | $9 a ream |
| Feeler gauge | any blade set with a 0.5 mm (0.020 in) leaf | | $7 |
| Marks | masking tape: the printer's outline on the table, the stack's marks on the cover | | owned |

About $98 in all. Setting up takes about 20 minutes: unpack, install the cartridges, select LTR, place the table and
the printer on their marks. Nothing in the trial is sharp or hot; the grippers never go inside the printer's feed path.

## Simulation model and estimates

| Item | Model |
|---|---|
| Printer | one articulation fixed to the world: the body (boxes: the document cover's frame, the rear housing with the slot, its floor and right side), the shut document cover, the paper support on a hinge with a stop, the guide on a slide, and the OK, Black and Color keys (sprung, 2 mm proud, 1.5 mm of travel) |
| Paper | the five sheets as one limp stack: 9 rigid strips across its short side, each the stack's full 215.9 mm wide and 31.0 mm long, joined by 8 sprung hinges (a floating articulation). Each strip's collider is a 2 mm box. Mass and bending stiffness are both 8 times the five sheets' (180 g in all), so it hangs and bends as the stack does |
| Easier than paper | the sheets never separate or fan; the stack is rigid across its 215.9 mm width (it can't bow against the guide); held at its edge it hangs flat where paper curls; no air slows it; it can't crease or tear (a hinge past 150 degrees counts as folded over) |
| Code 1000 | cleared by task code: OK pressed while the stack stands in the tray. The test page isn't simulated (no feed or jam), so in sim accepted means that press |
| Physics | PhysX, 240 Hz, 20 Hz control |

Estimates: the rear tray's slot (22 mm wide, its floor 112 mm above the table, its right side at y -160); the paper
support (110 mm long, its stop 110 degrees); the guide (52 mm of travel, 3 N s/m of damping, its raised grip 13 mm
above the housing); the OK key's position and force (2 N to
press it 0.8 mm); the frictions (paper 1.0, the printer's plastic 0.25); the stack's hinges' stiffness (5.56e-2 N
m/rad).
