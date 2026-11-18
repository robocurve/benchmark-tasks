# Clip a car air freshener onto a dashboard vent vane: fork over one pivoting vane, pushed home along it, vent aim kept

Clip the air freshener onto one of the centre vent's horizontal vanes, pushed all the way on, without changing where the vent is aimed.

## Scoring

The rules: task.json's `scoring`, word for word (real trial: the end of the video, then a ruler, a protractor and a 100 g weight).

How `score()` reads each condition (clip frame: origin at the body's centre, x along the fork, z across the gap):

- **Engaged:** for any vane, the point of its front edge nearest the fork, taken into the clip's frame, lies between the
  prongs: the vane's centre plane where a 2 mm vane fits between their inner faces (1.6 mm apart plus
  however far the sprung prong has opened), within 0.8 mm; 12 mm or less in front of the fork's root (at least
  8 mm of the 20 mm prongs over the vane), and across the prongs' width inside
  the vane's length.
- **Home:** that edge at most 2 mm from the root.
- **Let go:** both grippers' tool points at least 30 mm outside the clip's bounding box and the vent
  opening's box.
- **Still:** over the last 1 s three points on the clip moved under 1 mm and no vane turned more
  than 1 degree (positions, not solver velocities), held for a further 1 s.
- **Hold check:** then a 1 N force straight down on the clip's body (ramped over 0.5 s, held
  2 s, applied by the scorer to the free body); the clip must move under 2 mm. A clip resting
  loosely on a vane slides off; one gripped by the sprung prong stays.
- **Aim:** the largest angle of any vane from level at the end. **Square:** the angle between
  the fork's axis and the dashboard face's normal; turning about the fork itself is not counted.

`success()` is a score of 1.0.

Why the shortcuts fail: the vanes keep any angle they are pushed to (joint friction, no spring back), so a fork landed
on a vane's face, or pushed on pitched to the vane, turns the vane: past 15 degrees the vent is knocked off
its aim and the score is 0. Our wrong-way run is the scripted demonstration with the fork pitched
30 degrees nose-down to the vane and pushed along its own axis: the fork bound on the vane 5 mm on; the push (about 9 N in, 7 N down) forced the vane's front edge down 9 degrees, and turned the clip in the fingers. Let go, the clip hung tilted on the vane's edge 15 mm from its root, not engaged: score 0, with the vent left aimed 9 degrees down. A clip
pushed into the gap between two vanes, laid on top of the vent, or left in the hand scores 0. Doing nothing scores 0.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clip_car_air_freshener_onto_vent_vane.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Clip the air freshener onto one of the centre vent's horizontal vanes, pushed all the way on, without changing where the vent is aimed." \
  --scorer task_score --max-steps 1200 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. The score is read at the end of the run; each trial allows 1,200
control steps (60 s at 20 Hz). The scripted demonstration takes 23.2 s. `pytest` here checks the scorer
on named layouts.

## The scene

Products, sizes and positions are in task.json's `setup` (mm from the midpoint between the arm bases; x forward, y to
the robot's left; heights above the table). The dashboard, its vent and the mat are fixed; the clip is the one free
body. Both arms start at their ready pose with the grippers open.

- **Dashboard and vent (fixed, the vanes on hinges):** dashboard 600 x 150 x
  190 mm, face at x = 240; opening 110 x 70 mm centred at
  (240, -120, 130), duct 45 mm deep. Vanes 108 x 17 x
  2 mm, 3 g, centre lines at heights 107.5, 122.5, 137.5, 152.5; each on a hinge about y
  8.5 mm behind its front edge, limits +-30 degrees, joint friction 0.03 N m
  static and 0.025 N m moving (the detent), damping 0.0002 N m s/rad, armature
  2e-06 kg m^2 (numerical). Seven vertical vanes 30 mm behind the face, fixed.
- **Clip (free, 7 g):** body a rounded-rectangle prism 41 x 38 x 16 mm;
  lower prong fixed to it; upper prong (0.3 g) on a slide across the gap, 0 to 0.8 mm
  open, spring 1250 N/m with 1.0 N preload (1.5 N
  spread over a 2 mm vane), damping 5.0 N s/m, armature 0.02 kg (numerical). Prongs 20 mm long, 7 mm wide, 1.4 mm thick at the root,
  1.6 mm apart, each tip turned out 1.45 mm over its last 3 mm (mouth
  4.5 mm); each prong is one convex piece, its outer face running straight from the root to the
  tip, so its inner face has no seam for a vane's corner to catch on. Friction: clip 0.5/0.45, ABS
  0.4/0.35, mat 0.8/0.7.

## Scripted demonstration

The right hand does the work; the left arm stays at its ready pose. The hand points at the dashboard pitched
30 degrees down and pinches the clip's two flat sides; the clip sits in the pinch turned the same
30 degrees, so its fork is level. The demonstration reads (it never moves an object) where the
standing clip is before the pick, and the held clip's pose in front of the vane (twice, each read moving the hand so
the clip is where the plan has it: at most 15 mm / 10 degrees, then
8 mm / 5 degrees) and after each push step (height and pitch only,
at most 0.8 mm / 1.5 degrees a step, 3 mm /
4 degrees in all): the 4.5 mm mouth must meet the 2 mm vane within
about a millimetre, and the arm sags a little under load. The pinch is firm (pads commanded 23 mm apart on the
41 mm body: about 45 N, within the arena's 50 N limit for the YAM gripper); held at about 10 N, the clip turned in the
fingers when the fork met the vane.

Steps (s): out in front of the dashboard, clearing its top 0.0-1.5; over the clip 1.5-3.0; down behind the clip
3.0-4.2; pinch its sides 4.2-5.0; settle 5.0-5.3; lift it 5.3-6.3; carry it in front of the vane 6.3-8.8; hold in
front of the vane 8.8-9.4; line up with the vane 9.4-10.0; push along the vane (8 steps of 0.35 s, height and pitch
re-read after each) 10.0-12.8; hold it home 12.8-13.4; let go 13.4-14.2; back away 14.2-15.2; rise clear 15.2-16.2;
the hand to its parking point 16.2-18.2; hold still (the run stops at full credit) 18.2-23.2.

The push starts with the fork's tips 8 mm short of the third vane's edge and goes 5 mm past
home along the vane's plane; the vane's edge stops at the fork's root and the arm's compliance takes the rest. The pads stay
4 mm behind the clip's back, out of the vent. In our recorded
demonstration (no object jitter): full credit at 18.5 s, the vane's edge at the fork's root (0.0 mm), every vane within 0.01 degrees of level, the sprung prong 0.38 mm open on the 2 mm vane. In three runs of the same scene with the clip's start jittered by up to 10 mm, all three scored 1.0; the hold check moved the clip 0.01 mm.

## Simulation and the real objects

True-size rigid parts. Each vane has its own pins and its own detent; the rear link that turns a car's vanes together is left out, in the simulation and in the built vent. The fork's sprung prong is a plate on a sprung slide (1.5 N on a vane), with a numerical armature; its small barbs are left out. The oil is mass only. The clip starts clicked open, standing on its flat edge.

- The vent: no car maker or parts seller we found publishes a centre vent's vane thickness, spacing and depth
  (universal in-dash louvres list only bezel and cut-out sizes), so the vent is built to the simulated sizes and
  detent (Building the vent). Vent-mount makers publish what real louvres must be for their hooks: blades under
  3.5-4.2 mm thick, 10-34 mm deep, at least 8 mm apart; our vanes are 2.0 thick, 17 deep and 13 apart.
- The clip: the Febreze CAR Vent Clip. The maker publishes no clip sizes or forces; its patent for such clips gives
  prongs 10 to 40 mm long (narrowest range 16 to 20), gaps 0.5 to 8 mm at their widest (3 to 5) and 0.5 to 3 mm at
  their narrowest (1 to 2). Our 20 mm prongs, 4.5 mm mouth and 1.6 mm gap
  lie inside the narrowest ranges. Every clip value is an estimate (The clip, below).
- Estimates: every friction.

## Building the vent

Sizes in mm, taken from the task's module. Register coordinates: from the opening's centre on the face, x back into
the dashboard, y along the vanes, z up. Parts (US prices looked up November 2026, about $110 in all):

- Black ABS filament, 1.75 mm, 1 kg (Bambu Lab, $18.99; about 150 g used): https://us.store.bambulab.com/products/abs-filament
- 3 x 16 mm stainless dowel pins, 80 (Walmart, $10.99; 8 used): https://www.walmart.com/ip/16651518561
- Nitrile O-rings 3 mm ID x 1.5 mm (6 mm OD), 8 (Totally Seals, $1.00 each): https://www.totallyseals.com/en-us/products/3mm-x-1-5mm-nitrile-o-rings
- Set-screw shaft collars, 3.05 mm bore, 8 mm OD, 4 mm wide, 10 (Walmart, $10.49; 8 used): https://www.walmart.com/ip/5275673485
- M3 x 12 mm flat-head machine screws, 4 (Everbilt 837751, Home Depot, $1.95): https://www.homedepot.com/p/321071710
- 1/2 in. (12 mm) MDF panel, 2 x 4 ft (ProWood 109097, Home Depot, $24.99 to $28.00): https://www.homedepot.com/p/202093821;
  dark grey spray paint (Rust-Oleum Painter's Touch 2X Satin Charcoal Gray, Home Depot, $6.98):
  https://www.homedepot.com/b/Paint-Spray-Paint/Rust-Oleum-Painters-Touch-2X/Gray/N-5yc1vZapz5Zbm2Z1z1a3o5
- 6 in. digital caliper (Pittsburgh 63711, Harbor Freight, $24.99): https://www.harborfreight.com/6-in-digital-caliper-63711.html
- For the checks: thread, a small bag, 80 US nickels (5.000 g each; or a kitchen scale) and a dime.

Prints (black ABS, solid round the pins). Printable STL files (mm, from the dimensions below, laid out for printing):
fixture/vane.stl (print 4; on its front edge), fixture/front_plate.stl (on its front face), fixture/rear_half.stl (on
its back wall).

1. Vanes, 4: blade 108 (y) x 17 (x) x 2.0 (z), front edge at x = 0; at each end a hub 6 across and 5 long (|y| 54
   to 59) on the pin axis (x = 8.5, mid-thickness) with a 3.0 hole, 5 deep. Print standing on the front edge,
   supports under the hubs. Caliper check: 2.0 +-0.1 thick, 17.0 +-0.2 deep, 108.0 +-0.3 long.
2. Rear half, x 8.5 to 47: a block 140 (y) x 90 (z) with the 110 x 70 duct through it to a back wall at x 45 to 47;
   seven fins 1.5 thick at y = -45, -30, -15, 0, 15, 30, 45, from x = 26 back to the back wall, joining the duct's
   top and bottom (the simulated fins end at x = 34; nothing reaches past 26).
3. Front plate, x 0 to 8.5: 140 x 90 with the 110 x 70 opening, square corners; four countersunk 3.4 holes at
   y = +-62, z = +-38, over 2.5 holes in the rear half.

Both halves carry half-bores on their joint face (x = 8.5) for each pin (z = -22.5, -7.5, 7.5, 22.5, both sides): 8
across for |y| 55 to 60 (the hub's pocket), 3.2 for 60 to 63, 10 for 63 to 70 (O-ring and collar); in the rear half
the 10 bore runs back to x = 47 as a slot, so the collar's set screw can be reached from behind.

Assembly and the detent:

1. Press a pin to the bottom of each hub's hole (a drop of super glue if loose); its outer end is then flush with
   the register's side (|y| = 70). Thread an O-ring, then a collar, onto each pin.
2. Lay the vanes in the rear half's half-bores and screw the front plate on.
3. With a vane level, push each collar inward to squeeze its O-ring against the wall and tighten its set screw;
   both ends alike, so the vane does not twist.
4. Check: hang the bag from two thread loops round the vane's front edge, 10 mm in from each end (8.5 mm in front of
   the pins). 330 g (66 nickels, 0.0275 N m) must leave the vane level for 10 s; 390 g (78 nickels, 0.0325 N m) must
   turn it. The simulated 0.030 N m is 360 g there. Adjust the collars until both hold; recheck before each
   session. The 100 g hold check puts about 0.017 N m on a vane, under the lower bound.
5. Box from the MDF: front 600 x 190, top 600 x 138, sides 138 x 178, glued and screwed, open at back and bottom,
   painted dark grey with the register's front. Cut a 140 x 90 hole in the front centred on the vent's centre
   (y = -120, 130 above the table), push the register in from behind, fronts flush, and hot-glue it. Clamp the box
   to the table, face at x = 240, ends at y = -300 and 300.

A built vane comes to about 6 g with its pins (calculated; 3 g simulated); its weight sits on the pin axis, so it adds no turning
torque. The simulated +-30 degree stops are not built: a vane past 15 degrees already scores 0.

## The clip: estimates and checks

No physical measurement of the purchased clip has been made: its values are estimates, and the lab checks the key
ones before the first trial (below).

| Value | Estimate | Obtained from |
| --- | --- | --- |
| Body, w x h x t | 41 x 38 x 16 mm | a retail photo scaled by the listed pack size (about 39 x 41); thickness estimated |
| Prongs, length x width; thickness root / mid / tip | 20 x 7; 1.4 / 2.1 / 1.4 mm | length inside the patent's range; the rest estimated |
| Gap, mouth, flare | 1.6, 4.5, 3 mm | gap and mouth inside the patent's ranges; flare estimated |
| Vane, thickness x depth | 2.0 x 17 mm | the printed vane |
| Mass | 7 g | calculated from the shell: 5.5 g empty, 7.3 g with the oil |
| Spread to a 2.0 mm gap | 1.5 N (simulated spring) | calculated 0.39 N (both prongs bend) / 0.78 N (one braced), at the tips |
| Push-on peak; pull-off | 1.66 / 3.22 N; 0.68 / 1.34 N | simulated with the task's spring, height held / floating |

What was run, no hardware: a beam calculation of the task's fork (polypropylene, modulus 1.5 GPa assumed), and a
contact simulation (MuJoCo 3.4) of the task's own fork mesh and sprung prong pushed onto and off the 2.0 mm
vane at 2.5 mm/s, height held (one face loaded) or floating (both); halving the time step to 25 µs (held) and 12.5 µs
(floating) changed the results by under 0.01 N; pull-off is the seated load times static friction 0.45 (assumed).

The simulated spring, 1.5 N on a vane (1.0 N preload plus 1.25 N/mm), is 2 to 4 times the calculated 0.39 to 0.78 N.
So the simulated push-on is harder than for a fork as soft as calculated (slide model of the same fork: 1.64 to
3.29 N, against 0.40 to 0.54 N with bending prongs and no preload) and its grip stronger (pull-off 1.35 against
0.45 N). The scored hold check pulls straight down, not along the vane, and none of these covers it: the bought clip
must pass it by hand (below).

Checks before the first trial, about a minute each; a clip outside a band is not the simulated fork: use another.

- Fork: gap at the root 1.35 to 1.95 (a dime, 1.35 thick, slides in to the root freely; a nickel, 1.95, only by
  spreading the prongs); by caliper, mouth at the tips 3.5 to 5.5 and prongs 18 to 22 from the clip's back. A
  four-prong clip (the patent's preferred form): check the pair that takes a horizontal vane.
- Spreading force, kitchen scale: hang the clip by a thread round one prong's tip, a small bag from the other's; add
  nickels until a nickel slides between the prongs 3 mm in from the tips; weigh the bag: at most 150 g (the simulated
  spring gives 147 g at a nickel's 1.95; calculated under 40 g).
- Push-on force, kitchen scale: tape a spare printed vane to the scale on its back edge, tare, push the fork straight
  down onto it until home over about 3 s: highest reading at most 340 g (3.3 N, the top of the simulated range).
- Hold: pushed onto a built vane by hand, the clip passes the 100 g hold check.

Scented oil (review note): opening the clip pierces the foil over the oil, and the maker's FAQ covers oil leaking onto
surfaces, so real trials use a used-up clip: clicked open, dial on high, left until the scent is gone (up to 30 days
on low, says the maker), then squeezed by its flat sides in a paper towel: no oil mark. It is about 2 g lighter
(calculated 5.5 g against 7.3 g); the simulated 7 g lies between.

Sources: maker's FAQ, https://circle.supersavvyme.co.uk/uk/projects/febreze/faq; patent US 12,708,692 B2,
https://patents.google.com/patent/US12708692B2/en; retail photo and pack size,
https://www.target.com/p/-/A-14966291; a universal louvre listing (sizes of bezel and cut-out only), https://watsons-streetworks.com/product/trique-mfg-rectangular-a-c-vent/; coin mass and thickness,
https://en.wikipedia.org/wiki/Nickel_(United_States_coin), https://en.wikipedia.org/wiki/Dime_(United_States_coin).

## Reset

Pull the clip straight off its vane along the vane's plane, stand it on its flat bottom edge on its mark on the mat (within 1 cm), fork toward the dashboard, and set all four vanes level (within 1 degree). The same clip is reused; the vent is never removed. Zero object jitter.

## Revision

Review 2 (not accepted), what to change: "The real-parts check is still open: measure the purchased clip's fork dimensions and the
force needed to spread or insert it over the specified vane, report the measurements, and update the setup and
simulation where needed. Please also provide 3D-printable files for the vent fixture if they are not already
included."

Changes, text and print files only: the README states that no physical measurement of the purchased clip has been
made, gives each clip value as an estimate with its source, states the simulated spring's difference from the
calculation and which way it errs, and adds kitchen-scale checks of spreading and push-on force, banded so the real
clip is no stiffer than the simulated one; STL files of the printed vent parts (fixture/); README Scoring points to
task.json. Simulation, demonstration and runs unchanged.
