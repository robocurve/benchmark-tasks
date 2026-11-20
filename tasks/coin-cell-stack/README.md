# Stack a zinc CR2032 coin cell with tweezers and dose its electrolyte by pipette

Two YAM arms at the station table assemble a zinc symmetric (Zn||Zn) CR2032 coin cell in an open can that sits in an
assembly mould, the way aqueous-zinc battery labs do on an open bench. With plastic battery tweezers for every part, the
robot lays a 12 x 0.1 mm zinc disc on the can's floor and a 19 mm glass-fibre separator on it. It then draws 80 uL of
2 M ZnSO4 from a tube with a micropipette and dispenses it onto the separator: one hand holds the pipette, the other works
the plunger. Then come the second zinc disc, a steel spacer, a wave spring and the cap, open side down. Every part lies
flat and centred. The other hand presses the cap down, and both tools go back in their stands. Centring is
measured against the can, not by blind alignment: each zinc disc must lie within 1 mm of the can's axis, read from its gaps
to the can's wall as it is laid, and the zinc-to-zinc distance is computed from those two centres.

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena),
Isaac Lab-Arena 0.3.1).

## Run a trial

Any policy, here Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 9600 --store-frames --log-dir runs/dream
```

Scripted demo (scores 1.0), the same command through Inspect Robots' CLI with the demo registered as a policy:

```bash
python run_demo.py runs/demo            # 640 x 360; --width 320 --height 176 for Dream-1's size
```

The run prints `task_score` at the end: 1.0 once every check has held, with the fingers 3 cm clear of both tools and
the cell for 1 s; otherwise the tier the trial reached.

**The demo is a scripted reference that reads the poses of visible objects.** Each control step it reads, from the
simulator, the poses of the parts, the tweezers and the pipette (all in view of the cameras), how far the tweezers' tips
stand apart and how far the plunger is down (both visible), and its own arms' joints. A policy gets only the cameras and
the joints. `celldemo.py` holds the demo; `demokit.py` its arm primitives. The left gripper holds the tweezers sideways
across both legs, 80 mm above the tips. Squeezing the legs closes the tips; opening the pads by 1.5 mm lets the legs
spring the tips apart, while the tweezers stay held by the legs' spring. The demo's log gives the pads' widths and tip
gaps used at each pick and release.

## How it is scored

`cellrules.py` is the scorer, a state machine fed once per control step (`task.py` feeds it). It reads what the overhead
videos, the fingertip-cover blot, the depth gauge, the meter and the scale would show:

- **Order, each part once.** Each part counts once, at the first moment it lies in the can. The order must be zinc,
  separator, zinc, spacer, spring, cap, with the dispense between the separator and the second zinc. A part that leaves
  the can after it has been laid in it (lifted back out, more than 6 mm above the rim or out past its wall, to re-lay or
  swap it) scores 0. The log gives each part's highest point over the rim and farthest offset after it arrived.
- **Offsets, from each part's own frame.** A part's offset is read in the latest step where the left fingers are more
  than 5 cm from the can's axis (the arm is away at a dish), until the next part covers it. The offset is the distance
  from the part's centre to the can's axis, which equals half of (widest minus narrowest gap to the wall). The
  zinc-to-zinc distance is measured between the two zinc discs' centres, each taken from its own frame.
- **Flat: the depth gauge.** After the trial, with the cap and the spring lifted off, a depth gauge zeroed on the can's
  rim reads the spacer's top at 4 points 6.5 mm from the axis, 90 degrees apart (`gauge_mm`). The 4 heights must agree
  within 0.3 mm (a raised edge anywhere under the spacer tilts it), and none may stand more than 0.3 mm over the nominal
  1.46 mm stack (zinc 0.1, separator 0.26, zinc 0.1, spacer 1.0: a doubled, folded or raised part lifts it). The log
  gives the 4 heights at every change of tier.
- **Electrolyte (`pipette.py`).** The plunger, held at the first stop, is let back up to rest with the tip at least
  2 mm under the surface the whole way: the tip then holds 80 uL. Pressing to the first stop with the tip 1 to 6 mm above
  the separator and within 3 mm of the can's axis, with the separator in the can, puts 0.105 g on the separator.
  Anywhere else the dose is spilled, and any electrolyte dispensed off the separator scores 0. Letting the plunger up
  with the tip still in the dispensed liquid would draw some back in a real pipette; that is not modelled, so the demo
  lifts the tip 15 mm clear before it lets the plunger up.
- **Meter (over R0).** Read with the cap and the spring lifted off, so it needs neither: a weighted probe on the
  spacer, the other on the can's wall. The separator lies between the zinc discs (above the first, under the second) and
  covers both, and the spacer does not touch the can's wall: an offset of 1.85 mm brings it to the wall, and a spacer
  offset of 1.85 mm or more scores 0. In the simulation the separator-between-zincs check is taken in the last step
  before the cap press (after it the simulated thin discs may sink into each other; see Fidelity).
- **Spring.** Seen in its own frame before the cap covers it: an offset of 2.25 mm or more puts it on the can's wall
  (a short once the cell is crimped) and scores 0.
- **Cap.** The cap is seated when it is open side down, square, and its gasket is inside the can's mouth all round. It
  is pressed when the right fingers bear on its top and push it below where the spring holds it, while it is seated.
- **Tweezers only.** A contact is the run of steps in which a gripper's fingertip is within 2 mm of a part. A part's
  finger travel is how far it moved during each contact, summed over all contacts in the trial; 1.0 mm or more scores 0,
  however slowly it moved. 1 mm is about 18 pixels in phone A's video, so it is read off the video. The cap press is
  exempt, and only narrowly: while the right fingers bear on the cap's top, they may move the cap and the parts lying in
  the can under it. A fingertip in the can's well after the separator is wet and before the cap closes it counts as a wet
  blot.
- **Tools.** The tweezers stand tips down in their slot and the pipette hangs on its cradle. Every fingertip is at least
  3 cm from both tools and from the cell (the can and its cap) for 1 s, so the press hand must lift away.
- **Tiers.** 1.0, 0.5, 0.25 and 0 exactly as in `task.json`'s `scoring`, in `CellJudge.tier`.

### Scoring a real trial

- **Video.** Phone A, 45 cm above the mould on its 3x camera, films the can in 4K at about 0.055 mm per pixel (the
  can's 20.0 mm outside diameter, about 360 px, is the scale). Phone B, 60 cm up on its 1x camera, films the whole
  station for the tools and the hands. A part's offset is half of (widest minus narrowest gap to the can's wall), read
  in the part's last frame with the arm away at a dish (re-centring is allowed). Each zinc disc's centre is marked in its
  own frame, before the next part covers it; the zinc-to-zinc distance is between those two marks. The video also shows
  the order, each part laid once, and how far any fingertip contact moves a part (summed over the trial).
- **Blot.** Each gripper's finger pads wear single-use 50 um film covers. After the trial, press the covered fingertips on
  blot paper: a wet mark scores 0. The covers then go to zinc waste, so electrolyte never reaches the robot's pads.
- **Cap and spring.** Lift them out with plastic tweezers, if present, onto a tared boat; weigh them (0.001 g).
- **Depth gauge.** Can still in the mould, gauge zeroed on the rim: the spacer's top at 4 points, 6.5 mm from the axis.
- **Meter.** A flat-tipped probe in a 50 g weight stands on the spacer's centre; the other probe touches the can's outer
  wall; read on the 2 kOhm range after 5 s. R0 = max(5 Ohm, a fifth of the lowest reading of 3 cells built by hand and
  read the same way). Under R0 is a short.
- **Scale.** Lift the can by its wall with plastic tweezers and weigh it with its contents. Electrolyte = boat + can minus
  the dry mass of exactly the parts present, each weighed alone before the trial, so a missing cap or spring changes
  nothing.

Offline checks of the rules on hand-built trials, with no simulator: `python -m unittest test_cellrules`.

## Fidelity

- Physics runs at 480 Hz instead of the harness's 240 Hz, set when the task loads (`mechkit.set_physics_hz`).
- The zinc discs are rigid 0.1 mm discs. Their colliders are strips whose corners lie on the 12 mm circle, because a
  0.1 mm convex hull is below PhysX's cooking tolerance. The separator is a rigid 0.26 mm disc. A real glass-fibre mat
  wets and clings, so the upper zinc sticks where it lands; here it can glide.
- The paper-on-silicone linings of the dishes give about 0.3 mm under a 1 N press (soft contacts). The tweezers' tips
  press into the lining and close under a disc's edge.
- The tweezers are two rigid legs on a sprung hinge at the head (0.45 N m/rad: about 1 N at the tips when closed). Each
  leg carries a ribbed grip where the gripper bears.
- The wave spring is a rigid 1.2 mm ring with soft contacts, about 20 N per mm.
- The can is fixed in the mould's pocket; in the lab it is a loose fit.
- The electrolyte is an event model, not a fluid (above). Re-aspiration when the plunger is let up is not modelled.
- Under the cap press the simulated 0.1 mm and 0.26 mm discs can sink into each other (0.34 mm for the second zinc in the demo).
  So the meter's separator-between-zincs check uses the last step before the press; the depth gauge reads after it (a
  sunk stack reads lower, which the gauge allows).
- The finger-pad covers (50 um film) are not modelled.
- The cell is not crimped: the task ends at the pressed cap, where NPL's procedure hands the cell to the crimper.

## Runtime and seeds

- Demo: about 6,070 steps (304 s simulated); the success state holds from about 303 s. About 1.5 to 2 control steps per
  second of wall time on one RTX 4090 with physics at 480 Hz, so a recorded demo takes about an hour.
- The run limit is 9,600 steps (8 minutes at 20 Hz): the demo uses about 6,070, which leaves a policy room to re-centre
  and re-seat parts. A full 9,600-step trial takes about 1.5 to 2 hours.
- `object_jitter=0` keeps the layout fixed, so no seed is needed.
- Model run: Dream-1 on the task's own instruction and 9,600-step limit (320 x 176 cameras, 8 rows per request), one
  session from the first action to the end: task score 0.0. The video is `model_run.mp4`.

## Files

- `task.py`: the scene, `score()` and `success()`, the per-step tracker, and the demo's entry point.
- `cellrules.py`, `test_cellrules.py`: the scoring rules and their offline tests.
- `pipette.py`: the electrolyte events.
- `cellparts.py`: every part, tool and fixture, as plain meshes and simple colliders.
- `mechkit.py`: props written as USD text (fixed, free and jointed), soft contacts, the physics rate.
- `celldemo.py`, `demokit.py`, `run_demo.py`: the scripted demo.
- `make_stl.py`, `stl/`: the printed parts, as STL files.
- `steptime.py`: step timing.
- `inset.py`: the demo video's overhead view of the can (only with `TASK_INSET` set; not a policy camera).

## Printed parts (STL)

`stl/` holds the mould (60 x 60 x 20 mm POM, with a 20.3 mm pocket 2 mm deep), the tweezer stand (40 x 40 x 30 mm, a
37 x 9 mm slot 25 mm deep), the tube rack (30 x 30 x 25 mm, an 11.4 mm hole) and the pipette holder. The holder has a
60 x 60 x 8 mm base, a post at one side and a cradle of two prongs 22 mm apart at 203 mm. All dimensions are in
millimetres. `python make_stl.py` writes them from the same numbers the simulation uses. The stand, rack and holder are
unions of touching boxes, which a slicer merges. Print the mould and the stand in POM or PETG and the holder in PLA. Set
the holder's post height from the measured hook-to-tip length of the pipette with its tip on, so that the tip hangs
about 10 mm clear of the base.

## Sources

Every mesh is generated by `cellparts.py` from plain shapes; there are no third-party assets. Sizes come from real
products:

- MTI CR2032 304SS case and cap with a polypropylene gasket: can 20.0 mm OD, about 19.5 mm inside, 2.9 mm tall; cap
  about 19.3 mm across the gasket, 2 mm tall.
- MTI CR20SPA10 spacer, 15.8 x 1.0 mm; MTI CR20WS-SPR wave spring, about 15 mm OD and 1.2 mm free height.
- Zinc foil 0.1 mm, punched to 12 mm. Whatman GF/A glass microfibre, 260 um, punched to 19 mm.
- Bergeon 6460 plastic battery tweezers, 119 x 15 x 11 mm.
- A 10-100 uL air-displacement micropipette with a 200 uL tip: about 25 cm long, first stop at 6 mm (about 3 N) and
  blow-out at 8 mm.
- A 1.5 mL microcentrifuge tube with 1.0 mL of 2 M ZnSO4 (1.31 g/mL), and 90 mm glass petri dishes.

## Licence

MIT
