# Pry a nailed 1x4 off two joists with a flat bar

Instruction given to the policy: `Pry the board off the joists with the pry bar and lay it flat on the table.`

A 600 mm length of 1x4 pine is nailed across two 2x4 joists standing on edge (a piece of floor or deck framing, its
joists ending in a rim joist clamped to the table), one 3d box nail into each: a temporary cleat of the kind framers
nail across joists to hold their spacing until the subfloor goes on. A Vaughan B215 Original SuperBar (VN45001), 15 in / 381 mm, lies on the table in front. The robot works the bar's blade into the joint where the board's eased
bottom edge meets a joist, rests the bar's bend on that joist and pushes the handle down: the bar rocks on the joist
and lifts the board's edge, pulling that nail. It does the same at the other joist, then lifts the loose board off
and lays it flat on the table.

## Why it matters

Stripping nailed lumber off framing with a flat bar (braces, cleats, form boards, blocking, salvage) is routine work
for construction laborers (about 1.1M US workers) and carpenters (about 0.7M). O*NET task statements:
construction laborers (47-2061.00) "Erect or dismantle scaffolding, shoring, braces, traffic barricades, ramps, or
other temporary structures" and "Raze buildings or salvage useful materials"; carpenters (47-2031.00) "Remove damaged
or defective parts or sections of structures and repair or replace, using hand tools".

## Why it is hard

1. **Starting the blade without a hammer.** The board sits tight on the joist; the only way in is the small V its
   eased bottom edge leaves (3 mm). The 1.6 mm edge of the blade has to go in there, the blade flat on the joist.
2. **Using the joist as the fulcrum, with real force.** Each nail holds about 150 N. The bar's bend (the heel) rests
   on the joist and the handle is pushed down: about 3.6:1 leverage where the demo holds the bar, so roughly 40 N at
   the hand, through a parallel-jaw grip on a flat bar that can turn in the fingers.
3. **Re-setting the blade.** With only the edge in the V, the blade slips out as the board rises. The first stroke
   lifts the board a few millimetres; the blade then has to go 2 to 3 cm under the board before the nail can be
   levered out.
4. **Two nails, one bar, then a loose board.** The other end needs the same work (the demo hands the bar to the
   other arm). Once both nails are out the board rests loose on the bar's blade: drawing the bar out drags it off the
   joists, so the demo's free hand lifts the board off first while the other keeps the bar still.

## Files

| File | What |
|---|---|
| `task.py` | The task file: the objects, the nails' rule (`_nails`), the scorer hooks, the demo entry point |
| `layout.py` | Every dimension, mass and threshold |
| `scoring.py` | The scorer and the nails' withdrawal model, pure Python |
| `demo.py` | The scripted demo (closed loop on the bar's and the board's poses) |
| `tests/test_scoring.py`, `tests/test_demo.py` | Offline tests (32), no simulator |
| `setup.png` | The setup image (top camera, arms at their ready pose) |
| `mechkit.py`, `rigstate.py`, `demokit.py` | Shared helpers, copied unchanged (see below) |

## The scene

| Object | What |
|---|---|
| Frame (fixed) | A piece of floor or deck framing: two kiln-dried SPF 2x4 joists (38.1 x 88.9 mm), 612 mm long, on edge along x from 25 to 86 cm out, 16 in (406.4 mm) apart on centre either side of the middle, and a 2x4 rim joist on edge across their back ends (444.5 mm, flush with their outer faces, face-nailed with two 16d nails at each end) |
| C-clamps | Two 6 in C-clamps hold the rim to the table's back edge, 12 cm either side of the middle (pad on the rim's top edge, fixed jaw under the table top). Nothing is screwed into the robot's table; while the bar levers, a joist is pushed down overall (the heel presses harder than the nail pulls), so the clamps only keep the frame in place |
| Board (nailed) | 600 mm of 1x4 common pine (19.05 x 88.9 mm, S4S, eased edges), 0.46 kg, flat across the joists' tops, centred 40 cm out, ends overhanging 78 mm |
| Nails | Two 3d box nails (1-1/4 in, 0.076 in shank, 7/32 in head, bright), one into each joist through the board's centre line, heads flush; 12.7 mm in the joist. The cleat is tacked lightly, as temporary cleats often are (about 150 N to pull a nail). Heavier nailing would be beyond what the arm can lever; the skill (seating the blade, levering on the joist, re-setting deeper) is the same |
| Pry bar (loose) | **Vaughan B215 Original SuperBar**, current part **VN45001**, 15 in / 381 mm, 24 oz / 0.680 kg, blue spring-tempered steel. Simulated with an approximately 32 x 6 mm shaft, 45 mm flared ends, a 48 mm blade bent 15 deg with a 1.6 mm edge, a curved rocker head, and three nail openings. Detailed shape dimensions are approximations; see below |

Nothing else is on the table. Coordinates for every object are in `task.json` (setup) and `layout.py`.

## Pry bar make, model and geometry

Use a **Vaughan B215 Original SuperBar**, current part **VN45001** (item 43278). The manufacturer's
[product page](https://marshalltown.com/pro-4748-original-superbar?variantItemId=43278) identifies the legacy
B215 and current part number. Its [VGSS001 sell sheet](https://marshalltown-ecommerce.azureedge.net/mtdocuments/VGSS001_SuperBars.pdf)
specifies **15 in / 381 mm** and **24 oz** (0.68039 kg), tempered spring steel, the curved rocker head and three
nail slots. This closely matches the original task's 15-inch blue flat-bar design and retains its flat gripping surface.

`layout.py` uses the published overall length and tool mass. The old generic model was described as 380 mm but
its actual collision geometry spanned 354.24 mm; the straight shaft is now sized so the complete folded tool spans
381 mm. The blade/heel relationship and 200 mm shaft grip reference are retained. The revised model has flared
ends and three real collision openings: a notch at either tip and a closed keyhole near the blade. It uses 23 solid
boxes rather than filling the slots with a solid collision hull.

The manufacturer does **not** publish a dimensioned forging drawing in those references. The 32 x 6 mm stock,
45 mm flares, 48 mm blade, 15-degree bend, 1.6 mm edge and roughly 58 mm claw depth are modelling approximations,
not certified B215 measurements. The stock and working blade dimensions retain the original approximation;
end flares and slot locations follow the manufacturer's product photographs. Curves, bevels and slot outlines
are stepped box approximations. No product photographs, logos or third-party meshes are included as assets.

## Scoring (scoring.py)

At the end of the trial, with a ruler, a photo and the trial video:

- **1.0**: both nails out of the joists and the board lying flat on the table (face within 20 deg of horizontal,
  lowest point within 10 mm of the table, highest within 104 mm: not propped on a joist or the bar), both grippers at
  least 3 cm away, the board still;
- **0.75**: both nails out, but the board not laid flat on the table, or still held;
- **0.5**: both nails loosened (the board's underside at least 6 mm off the joist at each nail);
- **0.25**: one nail loosened;
- **0** otherwise. A board split along its length scores at most 0.75 (cannot happen in the simulation).

In the simulator "out" is the rule's own record that a nail has slid 12.7 mm (it releases the board then); in a real
trial it is a board that lifts away freely. The simulator credits each millimetre a nail slides to the bar when, at
that step, the bar's blade is beside that nail's joist (within 6 cm of its centre line), its edge under the board's
footprint and no more than 25 mm over the joists' tops, with the heel on the robot's side of it
(`scoring.bar_at_nail`); this is reported, not scored.

**Why the bar is needed.** A fresh nail holds about 150 N. A parallel gripper pinching the planed board holds roughly
2 x 50 N x 0.5, about 50 N, before it slips, and the arm cannot lift 150 N, so a gripper cannot pull a nail. Nor can
the board be levered on a joist while both nails hold: popping the near nail by pressing an overhanging end down
would take on the order of 800 N against the far nail. So the first nail needs the bar. Once one end is free, working
the board up by that end pivots it on the other joist and can draw the second nail: a normal technique on site, and
it counts. (In the simulation the board's end-to-end tip is limited to 0.12 rad while nailed, so that second route is
not available there; the demo uses the bar on both.)

## The nails (a declared rule, `task.py` `_nails`)

Simulators cannot model a nail's grip in wood, so a rule stands in for it, and the robot still has to apply the
real lift at the nail to trigger it. The board is glued (a PhysX fixed joint) to a hidden carrier inside the frame
that can only rise straight up and tip end to end (about a line across the board through its middle, on the joists'
tops), so one end can rise while the other is still nailed. The carrier's two
springs are the two nails: each is stiff (75 kN/m, 2 mm of give at full load) until the board pulls on it harder
than its withdrawal force; then the rule lets it slide and holds it where it was left. The withdrawal force is
150 N with the whole 12.7 mm in, from the Wood Handbook's formula for a smooth nail freshly driven into side grain,
p = 54.12 G^2.5 D L (N, mm), with SPF's G = 0.42 and D = 1.93 mm, and it falls in proportion to the length still in.
At 12.7 mm a nail is out; with both out, the glue is released and the board is loose. The rule never moves the
board: only contacts do.

## Run it

From this folder, with the benchmark's environment (`source ~/IsaacLab-Arena/yam.env`):

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
        --scorer task_score ...

The layout is fixed (`object_jitter=0`); there are no seeds to set. The scorer reads the end state (`SCORE_OVER =
"end"`, 1 s hold).

The scripted demo:

    python demokit.py task.py <out_dir>                   # final: records <out_dir>/full/demo.mp4 and setup.png
    python demokit.py task.py <out_dir> --fast            # physics on the CPU, no cameras: about 6 times faster

The B215 demo reached 1.0 in 1,998 steps (99.9 s of robot time), with zero blank frames. The recorded run took
449 s (about 7.5 minutes) of wall time on one NVIDIA RTX PRO 6000 Blackwell Max-Q Workstation Edition at the
default 640 x 360 camera resolution. The demo's other modes are `idle` (the setup image) and `right` (the first nail only).

Offline tests (32; no simulator; `yam-isaac-lab-arena` on `PYTHONPATH` for `yam_arena.task_file`):

    python3 -m unittest discover -s tests -v

The Dream-1 run:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Pry the board off the joists with the pry bar and lay it flat on the table." \
      --scorer task_score --max-steps 4800 --seed 0 --no-prompt --store-frames --log-dir <out_dir>

## Demo (demo.py)

Closed loop: every stage reads the bar's and the board's poses; it never reads the scorer or the nails' rule, and
nothing is teleported or edited.

1. The right arm picks the bar up by its plain shaft 200 mm from the blade tip (fingers across its 32 mm width), turns it
   blade forward and lays the blade flat on the right joist, its edge in the board's eased-edge V.
2. It levers in small commanded steps until the board is seen 4 mm up (the arm is soft, so the command runs well
   past the bar's angle), re-sets the blade about 2 cm under the raised board, and levers again until the board
   is seen 14 mm off the joist: the nail is out. That observation is retained if the loose board settles back
   onto the joist during the pause.
3. It eases the handle back up so the board settles, draws the blade out, and sets the bar down in the middle; the
   left arm picks it up and does the same at the left joist.
4. With the board now loose, the left hand keeps the bar where it is (drawing it out from under a loose board would
   drag the board) while the right hand grips the board inside the bay, lifts it straight up off the joists and the
   blade, carries it toward the robot and lays it flat on the table, clear of the bar's handle, and lets go. Then the
   left arm draws the bar out and puts it down on the table outside the left joist.

The demo holds the bar only by its painted shaft: the grippers' pads never touch the ground blade edge or the claw.
The recorded demo ends when the success condition has held for 1 s (the harness then stops the episode): the board
lies flat on the table, still, with both grippers at least 3 cm from it. At that moment the left hand is still holding
the bar, clear of the board; in a real trial it would then put the bar down.

## Results

| Run | Score | Steps |
|---|---|---|
| Vaughan B215 demo, GPU, recorded | 1.0: both nails out, board flat and released on the table; zero blank frames | 1,998 |
| Original generic-bar demo, GPU, recorded | 1.0 (right nail out at step 694, left at 1693; all 12.7 mm of each credited to the bar) | 2,031 |
| Original generic-bar Dream-1 (`--policy dream`, seed 0, full limit) | 0.0: it picked the bar up with its right arm several times and brought the blade to the board's front edge near the right joist, but never seated it in the joint or levered; neither nail moved, and the bar was dropped on the table and across the joists | 4,800 |

The existing `task.json` media ID refers to the original generic-bar submission. Revised setup/demo media must
be uploaded before resubmitting; the Dream-1 result above is historical and has not been rerun on the B215.

## Third-party code and assets

No meshes, scans or other assets: every object is built from boxes and cylinders here. `mechkit.py`, `rigstate.py`
and `demokit.py` are shared helpers copied unchanged from accepted tasks in this repository
(`tasks/shim-sleeper-to-level/mechkit.py`, `tasks/casement-stay-last-hole/rigstate.py`,
`tasks/alaris-set-load/demokit.py`, the last under the MIT licence as that task states). The O*NET task statements
quoted above are from O*NET OnLine (USDOL/ETA), used under CC BY 4.0. The withdrawal formula is from the USDA Forest
Products Laboratory's Wood Handbook (FPL-GTR-282, chapter 8), a US government work.
