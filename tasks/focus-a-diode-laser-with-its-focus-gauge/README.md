# Focus a diode laser before cutting robot parts

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1).

An open-frame 5 W diode laser engraver (Creality CR-Laser Falcon 5W, named for identification only), unplugged, stands
on the table, its front facing the robot, with a 3 mm aluminium plate under the work and a 3 mm plywood sheet for
robot-arm parts. Its laser module hangs 12 mm too high on its fixing plate, held by two finger-tight hand screws on its
right (-y) side; its fixed focus bar is stowed level on its magnet, on the same side below the screws. The robot turns the
bar down, holds the module while it loosens both hand screws (pinching each knurled head and turning its wrist), lowers
the module until the hanging bar just touches the sheet, tightens both screws, turns the bar back up onto its magnet
and lets go, without dropping the module or moving the sheet.

The procedure is Creality's (operation guide, p. 13; Creality wiki, CR-Laser Falcon user manual, step 5, and FAQ item
7): "the fixed focus bar should contact with the surface of the processed material when it is naturally vertical
downward. Then lock it with two hand screws on the right and turn the fixed focus bar clockwise until it is attracted by
the magnet."

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c "import json; print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted`. `TASK_DEMO=nothing` plays no motion (scores 0);
`TASK_DEMO=slip` loosens the holding grip before the lowering, so the module slides down past the fingers onto its bar
(scores 0: dropped); `TASK_DEMO=unheld` loosens both screws with no hand on the module (scores 0: dropped).

The demo, open loop. Right hand, pointing +y (tipped 8 deg down), fingers closed: press the stowed bar's leg 20 mm down
off its magnet; it swings down and hangs on its stop. Left hand, straight down: grip the module's top across y and push up
against the clamped slide, so that it carries the module's weight once freed. Right hand, pointing +y, fingers open round
the upper screw's head: pinch it and turn the wrist 90 deg anticlockwise (seen from the head); let go; the same for the
lower screw, which frees the module. Left hand: lower the module 14 mm over 6 s (12 mm to the contact, 2 mm past it).
Right hand: pinch the lower screw and turn it clockwise until it stops at snug (the clamp takes hold), then the upper
one. Left hand: let go and clear. Right hand, its approach pointing down toward +y (45 deg from vertical) and leaning forward, fingers
closed: from behind the hanging bar's leg, sweep it forward and up 90 deg onto its magnet; clear.

## How `score()` reaches the score

- `focus_usd.py` writes the scene's USD files into `build/` when the task loads. The engraver's frame, gantry, carriage
  and fixing plate and the aluminium plate are fixed bodies; the sheet is a free body. The module is a fixed-base
  articulation: a prismatic slide (module on its fixing plate), the bar's hinge and the two screws' hinges. The focus
  bar's visual is the real 2.5 mm strip; its collider is 7 mm thick (`BAR_COL_T`), so a fingertip cannot pass through it.
- Rules, every control step (`_apply_rules`):
  - screws: a PhysX contact view between each screw's head and the finger links reads each pad's force (averaged over
    the control step's physics steps). While both pads of one gripper press a head at 14 N or more, that screw's drive
    target moves by the gripper's own rotation about the screw's axis in that step (from the gripper link's
    orientation), never past snug (0 deg) and at most 295 deg out. Otherwise the target sticks until a push exceeds
    0.15 N m (a finger-tight screw's breakaway, estimate), then trails it;
  - clamp: while either screw is within 30 deg of snug, a stiff drive (300 kN/m) holds the slide where it was when the
    clamp took hold and carries the module's weight, as the real screws' friction does; otherwise the slide is free
    (5 N s/m damping);
  - magnet: within 5 deg of the stowed angle, a drive holds the bar there with up to 0.02 N m.
- Drop rules (either latches 0): the slide descending more than 1.5 mm within one 50 ms control step (3 cm/s), touched
  or not; and, read every 240 Hz physics step (`_poll`) from the same contact view (the module's body and both screws
  against the six finger links of both grippers), a descent of 2 mm from the module's highest point since a finger last
  touched it.
- Sheet rule, every control step: any corner of the sheet more than 3 mm from where it lay latches 0.
- The end-state table (task.json, "scoring") is read every step from e (the slide's height against the height at which
  the bar, hanging on its stop, touches the sheet's top surface), the screws' and bar's angles, the fingertips'
  distance to the module, bar, screws and sheet (URDF collision hulls against boxes), and each part's movement over the
  last 50 ms step. 1.0 needs its state for 20 steps (1 s), the same 1 s the harness holds `success()` for. A latch
  gives 0 from then on; only a reset clears it.

## Runtime and seeds

- Demo: 1063 steps (53.15 s simulated) to the success hold; about 9 minutes on an RTX 3070 Ti laptop GPU including
  start-up (about 0.45 s a step at 640 x 360).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop.

## Real-world kit

- Laser: Creality CR-Laser Falcon 5W (overall 568 x 632 x 198 mm), complete as sold. Unplugged throughout: the laser
  never fires. The gantry and the carriage are held where they stand by six small spring clamps (two on each side rail,
  either side of the gantry's end plate; one on the beam each side of the carriage).
- Plate: 3 mm aluminium sheet, 300 x 400 mm, taped flat on the table.
- Sheet: 3 mm basswood plywood, 200 x 300 mm.
- Scoring: a phone filming 240 fps slow motion on a small tripod; a 150 mm steel rule; a taper gap gauge (1 to 15 mm);
  a 150 mm digital caliper; a pen and a printed protractor card; masking tape.

## Estimates

Estimates: the module's mass, the slide's damping, the screws' breakaway torque, the magnet's hold, the sheet's mass,
and the module's, screws' and bar's sizes (from the maker's drawings and photos).

## Sources

- Creality, CR-Laser Falcon operation guide (5/10 W), p. 13: the 5W module's assembly and focusing step.
- Creality wiki, CR-Laser Falcon user manual (step 5) and FAQ (item 7, "How to focus? 5W"; item 8, a metal plate under
  cut work).
- Snapmaker wiki, Ray, adjusting the laser height: "before releasing the handle, make sure to hold the laser module
  securely to prevent it from slipping and falling, which may cause damage" (another maker's care for the same step).
- Tom's Hardware, Creality CR Laser Falcon review: the module "slides onto a bracket pre-mounted on the X-axis and is
  hand tightened with thumb screws"; it "can slice through wood up to 5 mm thick".

Every mesh is generated by `focus_usd.py` from the product's published overall dimensions and the maker's drawings.
Textures (basswood, plywood edge, brushed aluminium, fan grille, knurl) are drawn by `make_textures.py`. No third-party
meshes or textures are used.
