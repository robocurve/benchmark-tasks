# Substitute rail and complete machine envelope

This is a **construction plan, not a built or physically validated fixture**. It keeps three genuine Opulo **v1
8 mm spool-arm feeders**, including their original hooks, release levers and catches. The front rail is real 2020
V-slot. The recommended mating blades are the genuine Opulo front blades, including their PCBs; no approximate
printed peg replaces the catch interface. The other machine parts are closed, rigid wood envelopes. Every one of
`task.py`'s 20 `MACHINE` boxes is retained, including the full bed, both side frames and the parked gantry.

`machine-boxes.csv` gives all six exterior face coordinates. `frame-plan.svg` is the dimensioned plan and
`cut-list.csv` gives the shell panels. All three are generated from the simulation by `make_drawings.py`.
Coordinates in mm: x_m along the front rail, y_m inward from its front face, z_m + 100 above the bench.
Robot coordinates: x = 150 + x_m, y = -60 - y_m. Tolerance: +/-1 mm on exterior faces, +/-0.5 mm at rail and blades.

## Price and availability (USD, observed November 23-24, 2026)

Recommended route: ABRA OpenBuilds OB-210-LP 20 x 20 x 1500 mm V-slot **$47.19**, marked in stock, cut to 600;
Opulo Feeder Connection Kit **$155**, containing the genuine front blades and their fasteners, with a stated
**four-week lead time**. Wood: two 1/4 in 2 x 4 ft MDF panels **$32.46**, one 1 x 3 x 8 ft whitewood board **$7.62**,
glue **$3.98**, wood screws **$8.98**, and one ProWood 109112 nominal 1/8 in 2 x 4 ft hardboard bed panel **$7.68**. These observed-price lines total **$262.91**. Add **$22.35 catalogue-priced** for a 12-pack of rail brackets ($6.95), 100 M5x8 screws ($5.45) and 100 M5 hammer nuts ($9.95); use sheet offcuts for shims and the listed glue/screws for attachments: **about $285.26** for the rail/frame.
Source URLs and observation methods are in `bom.csv`. Hardware component prices are from the linked supplier catalogue; confirm stock and fit to the selected rail before ordering. Mat stock is nominally 2.38 mm; enroll the backed top surface at 3 mm and measure the finished mass.

This excludes feeders, loaded reels, mat, tools, tax and shipping. Use the **$59.00** blue Bertech 1059-18x24BKT mat kit, trimmed to 300 x 420 mm with flat offcut backing to a 3 mm top surface
and **$25.50 observed** for three LCSC C17414 full reels (their exact reel width/diameter and tape material must be
checked before enrollment): about **$369.76 with already-owned suitable feeders and tools**. If feeders must be
bought, the current Opulo 8 mm five-pack costs **$495**, giving **$864.76** before shipping/tax. It ships v2.0.0,
which is **not a verified replacement for the v1 spool-arm specimen**. The maker's mounting guide explicitly calls
the spool-arm feeder v1 and describes the v2 nose as extending farther. No complete, newly purchased v1 kit under
$500 or delivery before the review deadline has been verified. Borrowing existing v1 feeders/blades is the route
for an immediate trial; availability has not been confirmed. No purchase has been made.

The $120 Opulo Frame Kit is an alternative to the $47.19 rail (add $72.81; one-week stated lead time). A printed
blade route is a **cost reduction candidate only**: use the maker's `blade12.FCStd`, `blade13.FCStd` and matching
PCB CAD at machine revision `64cf68d5b7980f095ef11913065e7f45a1e3e447`. Printing and PCB manufacture have no firm
quote or fit test. A printed body without the PCB is not asserted to reproduce the catch/contacts. Do not enroll
that alternative until its assembled interface and release force match a genuine blade. The feeder geometry
reference is `c9869fe2cb97fd6939f2ed5aa6db446c46731e13`; no v2 geometry equivalence is assumed.

## Cut list and assembly

Tools (already owned/borrowed): saw with aluminium blade, drill, screwdriver, square, ruler/calipers, clamps, phone,
angle reference and a spring scale suitable for single-digit newtons. Initial fabrication includes cutting and glue
cure and is not a minutes-long build. Once assembled, reseating three already-threaded feeders resets a trial in
about 2-5 minutes (planning estimate, not timed).

1. Measure the sheet thickness. `cut-list.csv` assumes 6.35 mm. For a box of exterior dimensions X, Y, Z, cut two
   X x Y top/bottom faces, two X x (Z - 2t) front/back faces, and two (Y - 2t) x (Z - 2t) end faces. Glue into a closed
   shell with the specified outside dimensions. The two guides are solid wood strips **12 x 550 x 8 mm**, not shells.
   The bed is a separate **560 x 365 mm nominal 1/8 in hardboard** panel. Select or finish it to **3.0 +/-0.5 mm**
   by caliper, keep the top at **143 +/-0.5 mm**, and verify the underside at **140 +/-1 mm**. The seller page gives
   conflicting actual thicknesses (0.11 and 0.16 in); nominal thickness is not an enrollment measurement. Select a
   suitable sheet or finish the underside before assembly. Do not use 6.35 mm shell sheet for the bed. Total shell/bed
   panel area is 0.824 m2; two MDF sheets provide 1.486 m2 for shells before saw kerf, plus the separate thin bed panel.
   Lay out the large beam panels first, then strips. All exterior faces now use the stated tolerance.
2. Make all 20 named envelopes in `machine-boxes.csv`. Close hollow faces so fingers cannot enter them. Where boxes
   intersect, trim concealed internal panels to join them; retain the **union's exterior faces**. Use board offcuts
   as internal corner/rib reinforcement. Brace against a 20 N test press without more than 1 mm exterior movement
   (conservative fixture design check, not a measured extraction load). If the frame flexes, reinforce internally.
3. Mark the rail front face y_m=0, left end x_m=0 and bench z=0 on a flat, rigid bench. Place the four leg and two
   foot envelopes at their table coordinates. Front legs begin 50 mm above the bench on their 50 mm feet; do not
   move them down to the bench. Back legs reach the bench. Glue/screw the foot/leg joints from inside their union.
4. Cut the V-slot to **600 mm**, deburr, and put its lower face at 100 mm above the bench, x_m=0..600, y_m=0..20.
   Secure with compatible M5 T-nuts and L brackets to the back faces of the front legs. Put brackets on the
   underside and inside the leg/rail union so they do not project into the feeder path. With the rail supported,
   tighten and check both ends at centre height **110 +/-0.5 mm**.
5. Fit the genuine front blades as the maker's [blade installation guide](https://docs.opulo.io/feeders/2-install-harness/feeder-blade/)
   describes: front blades #1-12 and #13-25, using their M5x10 screws and T-nuts. The simulated blade occupies
   x_m=103..498, y_m=-28..0, heights 75..120; slot centres are **110.9, 126.7, 142.5 mm** for the three feeders.
   Align and measure the first three pegs before tightening. Preserve genuine blade bodies, PCBs and mating peg
   thickness. The model simplifies the interface; use the genuine part, not the model's rectangular peg drawing,
   as the manufacturing reference. The rail's real V groove is also retained rather than cutting a 5 mm plain slot.
6. Fit both lower and upper side rail envelopes at their **full 600 mm length**, y_m=-80..520 (including their
   portions projecting ahead of the front rail), and the back rail at y_m=450..470. Fit the guides y_m=-56..494 at
   heights 210..218. Connect to legs and rail internally; supports must not create open access absent from the model.
7. Fit the full bed at x_m=20..580, y_m=85..450, top 143. Support it from the side frames with offcut cleats below
   the bed and outside the feeder sweep. Its front edge is 5 mm behind the seated feeders' inner ends. Fit the bottom
   camera envelope underneath and the control box at x_m=-65..0, y_m=30..244, heights 20..140; do not let it rest on
   the bench or shorten it. Use concealed spacers within the frame, away from the feeder sweep.
8. Fit both carriage envelopes, the beam and toolhead at the parked positions in the table. In particular the beam
   is x_m=-65..640, y_m=420..440, heights 225..265. Retain the toolhead's height up to 340 mm. Fix the parked assembly
   so it cannot move during a trial. Check every exterior face against the table with ruler and square.
9. Put the rail at robot x=150..750, centre y=-70. Mount genuine v1 feeders with original catches in slots 1-3, each
   with a part-used 178 mm reel, threaded tape, no tape tied to an external bin, and no spindle clip added solely
   for the trial. Record actual specimen masses, winding/back-drive behavior and tape exit routing. Do not force a
   specimen whose tape connects to an external holder into this onboard-reel setup.
10. Lay the 300 x 420 mat at the lot centre (A: x=350, y=425; B: x=320, y=425). Check the real extraction with phone
    video as in README. No trial is physically validated until observations and pass/fail checks are recorded.

## Enrollment and reset checks

- Rail centre height 110 +/-0.5, middle slot centre robot x=276.7 +/-0.5 and y=-70 +/-0.5 mm; all envelope faces within
  1 mm of the table. Frame does not rock. Recheck after a collision or moving the rig.
- Bed front gap 5 +/-1 mm, top 143 +/-1. No omitted leg, guide, camera, control-box or parked-gantry envelope.
- Original hooks/catches click home; bodies approximately 0.8 mm apart, reels approximately 2.8 mm apart and at least
  6 mm above the bench. Measure with feelers/calipers rather than paper/card thickness guesses.
- Threaded tape is initially taut without imposed preload. Mark reel/body orientation. Film a slow check in each
  rotation direction; record play, payout and any back-driving. Measure catch release at a stated lever arm; the
  phone figure is 8 N on a new feeder, obtained by phone from a technician who works with this product (Nov 2026).
  The model maps this to the tab with an assumed 11.92 mm maximum radius; the real inner-end press is a separate reading.
- Reset only by reseating the genuine feeders, taking up loose tape as the maker prescribes and moving the mat to
  its lot. Do not replace their catches with magnets, fixtures, springs or altered levers. Confirm both neighbours
  remain hooked and latched; photograph all three starts. Repeat enrollment if a different specimen/version is used.

## Scene and parts sources

Pinned machine assembly source: https://github.com/opulo-inc/lumenpnp/blob/64cf68d5b7980f095ef11913065e7f45a1e3e447/src/assembly.FCStd. Pinned feeder design source: https://github.com/opulo-inc/feeder/tree/c9869fe2cb97fd6939f2ed5aa6db446c46731e13. The hook, clip and lever dimensions are photo-based model estimates.

Exact feeder CAD object/path, confirmed in the pinned repository tree: [`src/FDM/frame-8.FCStd`](https://github.com/opulo-inc/feeder/blob/c9869fe2cb97fd6939f2ed5aa6db446c46731e13/src/FDM/frame-8.FCStd). These are source pointers, not independently remeasured CAD dimensions or physical fit checks.
