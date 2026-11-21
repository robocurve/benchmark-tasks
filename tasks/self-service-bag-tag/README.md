# Loop a printed baggage tag through a suitcase handle and seal it

At a self-service bag-drop counter, take the waiting printed tag from the outlet,
thread the flexible strip under the suitcase's closed side handle, bring its ends
together and press the matching faces to seal the loop. Release the label and
leave the barcode exposed. This evaluates flexible-strip routing, receiving a
partly hidden end, regrasping and adhesive-face alignment. No handle lift, hand
assignment or exact arm resting pose is required for scoring.

## Real procedure and equipment

[Sihl PRESSURETAC luggage labels](https://www.sihl.com/wp-content/uploads/SIHL_PRESSURETAC_Leaflet_Airline_E_low.pdf)
use cold-seal coating that adheres to the opposite coated surface without a peel
liner. The manufacturer describes looping and pressing the ends together. This
scene represents that type of stock, not a peel-and-stick label with a skipped
liner-removal step. The barcode is a visible pattern; no scanner or passenger
check-in is evaluated.

Use a small cabin suitcase, a bench/scale, preprinted cold-seal tag stock and a
passive presenter with a single outlet. An operational airport kiosk is not
required: printing is complete before reset. A low-cost presenter can be made
from ordinary sheet material using `PRESENTER.md`; its retention holds the tag
until pulled, without supporting the hanging portion. The mechanism being
scored is the strip/handle/seal, using full-size strip width and ordinary handle
clearance. The decorative terminal does not perform any hidden manipulation.

## Setup

The official `yam_arena` station supplies two YAM arms with parallel grippers,
one overhead camera and two wrist cameras. Use the fixed task layout and seed 42,
with object jitter zero. Coordinates, materials and all collision shapes are
specified in `task.py`. Coordinates are metres in the station world frame.

* A hard-shell cabin-case visual, approximately 450 mm long, 178 mm deep and
  334 mm tall while standing on its narrow side, is centred at (0.330, 0) on
  the counter. Its long-side handle faces upward. The shell is supported/fixed
  on the scale; lifting or moving the suitcase is outside this task.
* A closed padded strap-style handle spans approximately 170 mm between mounts.
  The central aperture is about 30 mm above the shell. A passive 40 mm slider
  approximates handle lift, but the demonstrated route uses its resting gap.
* A 350 by 145 mm presenter/terminal sits at (0.675, -0.265), with outlet plane
  x=0.500 and tag centre 240 mm above the table. The internal channel is hollow,
  21 mm high. Approximately 90 mm of the 430 by 50 mm tag protrudes initially.
* One paper-like tag, represented by ten connected 43 by 50 mm panels. Blue
  39 by 46 mm marks identify the two mating end faces. Barcode patterns are
  on the strip. The entire tag exists at reset; pulling never spawns new paper.

Replace the used label, reload its trailing end and reset both arms between
trials. No hot, sharp, powered or contaminating equipment is used.

## Primary scoring

Score the current outcome, not the highest previous milestone. Use the highest
applicable tier, not their sum. All nonzero tiers require an intact, fully
extracted strip whose designated ends are bonded into a closed loop around the
actual handle, with an outward-facing barcode clear of the suitcase shell.
The printed barcode must face away from the loop interior and remain exposed
after release; sealing the tag with its barcode hidden inside is not a successful outcome.
This is an orientation/exposure requirement, not a barcode-scanner accuracy test.

* **0:** any prerequisite absent, including extraction alone, loose threading,
  a label merely on the case, an unlinked loop, or a broken label/connection.
* **0.75:** the prerequisites hold, but the robot has not yet released it stably.
* **0.85:** both grippers are open and clear of the label, and the loop remains within 5 mm for at least 0.25 seconds.
* **1.0:** the released stable state persists for at least 2 seconds.

In a real trial, inspect the loop and bonded overlap from video/photos, with a
ruler or an area grid if needed. At least 600 mm² of the marked patches must be
joined; a loose crossing is not a seal. Check that a printed face is exposed,
both grippers are open and at least 75 mm from the strip, and that it remains
attached without any corner drifting more than 5 mm from its hold-start position for
two seconds. Returning to a particular joint pose is not scored.

The simulation uses the physical strip's geometry to test handle linkage,
adhesive state and integrity. It additionally checks TCP clearance of 120 mm and
finger/palm-origin clearance of 75 mm. All outcome conditions, including barcode
exposure and complete extraction, must hold throughout the two-second interval.
A PhysX seal-break event permanently invalidates the trial; current overlap, face
gap and alignment are also checked. Rotations and intermediate oscillations enter
the stability check through panel-corner displacement from the hold-start pose. The barcode-side test is a geometric
visibility proxy with nine rays per candidate printed face against the other paper
panels. Both printed sides are considered; an upward exterior viewing direction
is required. This is not barcode decoding or an exhaustive occlusion proof. See `tag_closure.py` for all thresholds.

## Fidelity

The marked end faces include separate 39 x 46 x 0.1 mm collision patches so
pressing on the coating can be distinguished from contact at the uncoated rim.
This thickness is a numerical approximation, not measured coating thickness.
The strip uses colliding rigid panels joined by hinges; it is not a continuous
paper/cloth solver. Collision thickness is 1 mm and total mass 20 g, compared
with the reference [PRESSURETAC BT 28/3](https://sihl.com/en/products/pressure-tac-bt-283/) stock's 170 ± 12 micrometres and 177 ± 8 g/m² (about 3.81 g at
this area). Friction and hinge drives are simulation tuning, not product
measurements. They preserve feeding, bending, catching and regrasping, but do
not establish real material precision or hardware success probability.

The adhesive approximation requires aligned correct faces (normal dot >0.985),
at least 600 mm² overlap, surface contact within the 1 mm collision-sheet solver
margin, low observed motion, and actual opposing normal contact force of at least
1 N on the designated patches for 0.4 seconds. Force comes from GPU-native PhysX
contact impulses divided by the physics timestep, sampled at each control step;
alignment or proximity without physical pressure cannot create a seal. The 1 N
threshold is an explicit simulation approximation, not a certified adhesive specification. It then creates
a breakable paper-to-paper joint at the existing relative pose: no snapping,
object pose correction, attachment to a gripper or force-free motion is applied.
The 8 N / 0.15 Nm break limits and 2 N presenter release are representative tuning
values, not manufacturer-certified measurements. Peel propagation and adhesive
chemistry are not modeled. The handle is a spring-loaded rigid proxy for a
strap, and the suitcase/scale/presenter are fixed supports.

## Reproduction

Install the official Inspect Robots `yam_arena` embodiment and Dream plugin,
Isaac dependencies, NumPy, SciPy, trimesh and imageio/imageio-ffmpeg in the same
Python environment. The task runs through the official embodiment. Tested package
versions are listed in `runtime_versions.json`.

From this task folder, in that activated environment:

```bash
BAGTAG_RENDER_STRIDE=4 BAGTAG_VIDEO_WRIST=1 BAGTAG_REPLAY_COMMANDS=demo_commands BAGTAG_REPLAY_LIMIT=31 \
  python direct_thread_transport_probe.py runs/demo
python finalize_demo.py runs/demo
python compose_closeup_video.py runs/demo   # top view + wrist close-ups
python test_score_outcomes.py
python test_score_geometry.py
python test_hold_window.py
python test_press_gate.py
BAGTAG_RENDER_STRIDE=4 BAGTAG_NEGATIVE=drop_after_extraction \
  python direct_thread_transport_probe.py runs/drop-control
```

The demo is one continuous reset, using simulator poses to choose robot targets.
It does not restore object state. The revised continuous demonstration took
2,756 steps (137.8 simulated seconds) at 20 Hz and scored 1.0, with 13.8 seconds
of released stability. The coated-patch force at seal formation was 2.919 N.
Actual wall time depends on rendering. See `VALIDATION.md` for evidence and
limits. The revised Dream-1-beta session scored 0.0 over the full 4,800 steps; it did not extract the tag. Rendering every fourth step affects recording only; Dream uses
its normal camera observation cadence.

Dream (one episode, 4,800 control steps maximum):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  --policy-checkpoint dreamlabs/dream-1-beta -P timeout_s=90 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 -E top_cam=demo \
  --instruction "Attach the printed baggage tag securely to the suitcase's long-side handle, with the barcode facing out. Press the designated ends together, release the tag and withdraw both arms." \
  --scorer task_score --max-steps 4800 --store-frames --seed 42 \
  --no-prompt --grader none --no-rerun --log-dir runs/dream
```

## Third-party assets and licenses

The suitcase visual is [Innerscene Rolling suitcase](https://www.innerscene.com/tools/library/3d-parts/rolling-suitcase-ee9d7578),
released under CC0 1.0. It was resized and converted to USD, and the original
recessed handle was replaced by the task-authored strap visual. Asset details,
source hash and changes are in `assets/SOURCE.md`. This is an adapted generic
case, not an exact replica of a commercial product. The remaining task geometry,
markings and presenter are procedurally authored. Official YAM/Arena robot assets
are installed runtime dependencies, not redistributed here. The cited product
manual is a reference, not an included asset.
