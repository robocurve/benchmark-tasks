# Put a water bottle into a side-entry cage under a small bike's top tube

Created 11_16_26 (America/New_York).

An extra-small Salsa Rangefinder hardtail stands in a bike workstand beside the table, its left side to the robot. An
Elite Custom Race X bottle cage is bolted to the upper two of the down tube's three bottle bosses. On this frame size
the cage sits so close under the top tube that a bottle cannot be dropped in or lifted out along the cage: there are
53 mm of room above the bottle's top front edge, and its base would have to rise 92 mm to clear the cage's front band.
The robot picks up an empty Elite Fly 550 ml bottle standing on the table, carries it to the bike, tilts it to the
down tube's 61 degree angle, and pushes it in sideways, square to the cage: the bottle's own side opens the cage's
sprung front arm, which snaps shut round it. It then sets the bottle down on the cage floor, lets go and draws the hand
out of the frame without dragging the bottle back out. Instruction: "Put the bottle in the bike's bottle cage so it
stays put."

Install once, from this folder: `pip install -e . --no-deps`

Run a trial:

    inspect-robots run --embodiment yam_bike_cage -E task=task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt \
      --instruction "Put the bottle in the bike's bottle cage so it stays put." \
      --scorer task_score --max-steps 1400 --seed 0

The score is printed as `task_score`. Use `--policy dream` for Dream-1.

- **Runtime per trial:** the scripted demo's episode (24 s of simulated time; it ends 1 s after the full-score state is
  reached) took 81 s on one L40S and 106 s on the aarch64 workstation, plus about a minute to start the simulator (several
  minutes on a machine's first run).
- **Seeds:** none. The start is fixed and `object_jitter=0`; `--seed 0` was used throughout.
- **Scoring tests, no simulator** (in the Python environment that has `yam_arena`): `python -m pytest -q test_score.py`
  (21 tests: every band, each shortcut, and which criteria are judged at the end and which at any time). `python task.py`
  prints the geometry (headroom, gap to the top tube, the arm's tip force) and the demo's reach. `python dev/clearance.py`
  reports how close the arm's collision hulls and the carried bottle come to every frame and cage part along the demo.
  `dev/frame_from_drawing.py` re-reads the frame's geometry from Salsa's drawing (instructions inside).
- **The scripted demo** is open loop: it uses only the fixed starting layout and reads nothing from the simulator. The
  right hand comes from the right-rear and closes diagonally across the standing bottle's body, 135 mm up; lifts it;
  tilts it to the cage's angle while carrying it to the bike's near side; holds it square to the cage 12 mm high and
  11 cm out; pushes it 11 cm sideways into the cage (the bottle's side opens the front arm to about 15 degrees, which
  springs shut once the bottle is past); lowers it onto the floor; opens to an 80 mm pad gap and draws back along its
  fingers, then moves away. The grip is diagonal (60 degrees from the frame's plane) because a grip square to the frame
  puts one finger between the bottle and the down tube (8 mm) and one straight under the top tube.
- **What the physics does:** the frame and cage parts are fixed colliders; the cage's front arm is a rigid band on a
  revolute joint with a preloaded torsion spring (a joint drive), and the bottle and the hand are free rigid bodies with
  real contact. Nothing is bookkept: the bottle is held in the cage only by the arm, the floor, the lips and gravity.
- **Shortcuts tried in the simulator** (`BIKE_DEMO=<name>`; see Results): `nothing`, `hold` (the demo, but the hand never
  lets go), `lean` (pushed in only part way and let go against the arm's tip), `drop` (let go 12 mm short of the cage's
  near side). Dropping the bottle in from above is not possible on this frame (the top tube is in the way), so it is
  not a variant. `BIKE_DEBUG=1` prints each step's measurements.
- **Third-party assets:** none. Every part is built from boxes, cylinders and convex prisms in the task's own code.

## How it differs from the nearest tasks

The nearest accepted task, #1815 (phone into a spring-clamp car holder), holds a spring open by hand or edge in open
space and seats a phone on visible feet. Here the bottle's approach is obstructed: the top tube blocks the natural
straight-in path, so the bottle must be squared to a 61 degree cage and driven in sideways, the bottle itself opening
the arm, then released and the hand withdrawn from a closed frame triangle with millimetres to spare (the finger pads pass
within a few millimetres of the down tube and the top tube). #1387 (canister over a shelf fence) and #1902 (shelf
through a face frame) are accepted turning-through-a-gap tasks without a sprung retainer. Nothing on the board (searched
for bottle, cage, bike, bicycle, holder, clip, flex) nor in BEHAVIOR-1K, RoboCasa, LIBERO, SimplerEnv, DataCenterBench,
RoboDojo or the Embodied First Exam sets (dexhand, embodied-olympics) puts a bottle into a bike's cage; the bike tasks
there are pumping, washing, hanging or parking a bike.

## Results

Simulator runs with the command above (`--seed 0`), scripted policy. Cloud: an L40S host; local: the aarch64 workstation.

| Run | What the plan does | `task_score` |
|---|---|---|
| demo (cloud, g2) | the full demo | 1.0 |
| demo (cloud, g3) | the full demo | 1.0 |
| demo (cloud, g4) | the full demo | 1.0 |
| demo (local) | the full demo | 1.0 |
| `hold` | the demo, never letting go: seated and enclosed but held | 0.25 |
| `lean` | pushed in to 45 mm short of home and let go: the arm's spring pushes the bottle back out and it falls to the table | 0.0 |
| `drop` | let go 12 mm short of the cage's near side: it falls to the table | 0.0 |
| `nothing` | nothing | 0.0 |
| Dream-1 (local aarch64, 1,400 steps) | the right arm moved around the bottle and the frame but never put the bottle in the cage | 0.0 |

g2 ran on one cloud host, g3, g4 and the shortcuts on a second, all from this revision's demo plan (g2 and g3 before a
collider for the near pedal was added, which the demo's path does not come near). In every demo run the arm opened to
14.5 to 15.0 degrees during the push (about 3.9 N at its tip in the model) and shut fully; the bottle ended with its
base within 1.0 mm of the floor's middle and its axis within 0.1 degree of the cage's. Demo video and the setup image:
rendered from the local run (`demo_top_cam.mp4`, `demo_right_cam.mp4`, `setup.png`).

## Where each dimension comes from

| Item | Value | Source |
|---|---|---|
| Frame tubes in the frame's plane (down tube, top tube, seat tube edges), the three down-tube bottle bosses, the bottom bracket | as traced | Salsa, "Rangefinder Front Triangle Frame Bag Guide", XS, a 1:1 vector drawing ("Drawing size is 68 x 44 in, scale 1:1", +-2 mm): https://cdn.shopify.com/s/files/1/0912/4973/8003/files/470236_SAL_Frame_Bag_Guide_Rangefinder_VF_OL.pdf. Its 127 mm scale bar measures 127.00 mm; we read the bosses 64.0 mm apart (labelled 64) and the lowest 143.0 mm up from the seat-tube corner (labelled 143.1) |
| Ground orientation | page horizontal | the drawing's head tube axis lies 67.9 degrees from horizontal; Salsa gives 68.0 (27.5+ build) and 68.6 (29 in) |
| Head tube length, chainstay, bottom-bracket drop | 95, 439, 62 mm | Salsa Rangefinder 29 XS geometry (salsacycles.com; also https://bikeinsights.com/bikes/5e4381d0cd68f900174e7f48-salsa-cycles-rangefinder) |
| Cage diameter, height, mass, material | 74 mm, 147.3 mm, 28.5 g, glass-fibre polyamide | Elite spec sheet: https://api.elite-it.com/assets/documents/Schede-tecniche/CUSTOM-RACE-X_2026_EN.pdf ("lowered profile to facilitate the side insertion of the water bottle on mountain bikes and gravel bikes with compact main triangles") |
| Bottle diameter, height, mass | 74 mm, 184.5 mm, 51 g | Elite spec sheet: https://api.elite-it.com/assets/documents/FLY-TEX_2026_EN.pdf |
| Bottle cap | 59 mm across from 160 mm up | estimate from the 54 mm mouth opening (https://www.elite-it.com/en/water-bottles/sport/fly-tex) plus its wall |
| Tube widths across the frame | down 45, top 40, seat 34.9, head 50 mm | not published; estimates (the in-plane depths are from the drawing) |
| Cage floor below its lower bolt; bottle-to-tube stand-off | 35 mm; 8 mm | not published (the cage's holes are slotted); estimates. Any floor position from 25 to 45 mm leaves 45 to 63 mm of headroom, still well short of the 92 mm a straight lift needs |
| Cage front band height, arm shape, spring | band 48 to 92 mm up; 1.3 N at the tip closed, 3.9 N at 15 degrees | not published; estimates from the maker's photo, to be measured on the real cage (below) |

## Real-world setup and safety

| Item | Product | Price (one-off unless noted) |
|---|---|---|
| Bike | Salsa Rangefinder Deore 10 29, size XS (any Rangefinder build: the frame is the same) | $999 at https://salsacycles.com/collections/rangefinder; $1,099 at https://bikeandtrike.com/products/c9b92b80f5692d595ff367c70677fdcb8a3276fab442353dfb7e04d9eea7cf62; used Rangefinders sell for about $600 to $700 (for example https://bicyclecollective.org/products/salsa-rangefinder-14-5-a21080203, $700; confirm the size is XS) |
| Workstand | Park Tool PCS-10.3 (clamps the seat post, 22 to 76 mm) | $279.95 list (https://camelcamelcamel.com/product/B097DZ9XRG); $254.95 scratch-and-dent at https://nashbar.com/scratch-dent-park-tool-pcs10.3-deluxe-home-mechanic-repair-stand-blue-pcs-10.3-snd/p1622336 |
| Bottle cage | Elite Custom Race X | $24.99 at https://www.biketiresdirect.com/product/elite-custom-race-x-bottle-cage |
| Bottle | Elite Fly Tex 550 ml | $9.99 at https://planetcyclery.com/products/elite-fly-tex-water-bottle-550ml-red |
| Tools | Park Tool AWS-10 hex keys (4 mm for the cage bolts) | $10.95 at https://contenderbicycles.com/collections/accessories/products/park-tool-aws-10-metric-folding-hex-wrench-set |
| Measuring | steel ruler, phone inclinometer app, phone slow-motion video, luggage scale | already at any lab |

Per trial nothing is consumed. The bike is the one item above a few hundred dollars; a used XS Rangefinder brings that
down, and it can be lent or resold after the trials (any XS Rangefinder build has the same frame).

Set-up: take both wheels off (thru-axles), clamp the seat post in the stand with the bike level (top of the down tube's
upper bosses measured against the drawing) and its bottom bracket 10 cm above the table top, left side to the robot,
frame mid-plane 38 cm in front of the arm bases; turn the cranks level with the near crank pointing back. Bolt the cage
to the upper two of the three down-tube bosses with its floor at the bottom of its slots. Measure and record: the
cage floor's distance below the lower bolt; the force at the front arm's tip to open it 13 mm (about 15 degrees) with
the luggage scale (the model uses 3.9 N; a real cage reading over 5 N is noted in the results); the room above the
seated bottle's top edge along the cage. Stand the empty bottle at its mark, cap closed.

Safety: the bike is clamped in the stand and the stand's feet are weighted or taped to the floor so it cannot tip;
there are no blades, hot parts, motors or stored energy beyond the cage's light spring; the cranks are tied to the
frame with a strap so they cannot swing. Arm forces stay low: the push needs under 5 N, the bottle weighs 51 g.

## Real measurement and simulated stand-in

| Criterion | Real trial | Simulation |
|---|---|---|
| Seated | base on the floor, its middle within 10 mm of the floor's middle (ruler, photo square to the cage); axis within 10 degrees of the down tube (inclinometer on both) | base's middle within 10 mm of the cage floor's middle; axis within 10 degrees of the cage axis |
| Enclosed | front arm closed on the bottle, its tip within 8 mm (5 degrees) of its rest position (photo with ruler) | arm joint under 5 degrees, bottle axis within 15 mm of the cage's at the band's height |
| Retained | both grippers' fingertip midpoints 60 mm or more from the bottle for the last 3 s; bottle still at the end (video) | the same, from the fingertip midpoints and the bottle's pose and speed |
| Over-bent | the arm's tip opened more than 47 mm (30 degrees) at any time (slow-motion video, ruler) | arm joint over 30 degrees at any control step |
| Resting on the cage | base's middle within the stated box against the cage (ruler) | the same box in the cage's frame |

## Known limits

- The cage's flex is lumped into one front arm; the real cage flexes both sides and its front tongue. The spring is an
  estimate; the real-trial measurement above is the reference.
- The bottle is rigid; the real empty bottle squeezes, which makes the real push easier. Its cap is an estimate.
- Mainly one-armed: the bike is fixed in the stand, so the second arm has nothing to steady. The difficulty is the
  obstructed approach, squaring the bottle to a 61 degree cage, pushing against the arm and withdrawing the hand
  through a closed triangle.
