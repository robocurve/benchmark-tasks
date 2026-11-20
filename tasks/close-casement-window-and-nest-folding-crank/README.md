# Close a casement window with its folding crank, index the handle to the cover pocket, and fold it flush

Crank-out casement windows are closed by turning a small folding crank on the sill: half a turn pulls the sash onto
its weatherstrip through a geared operator. The handle then folds over into a pocket in the operator cover so
blinds hang clear, but it only fits at one phase of the turn. So the hand has to turn a crank round its circle,
stop it on the pocket's phase (within 5 degrees) without running on into the seal's overtravel, and then flip the
arm 180 degrees over its detent without dragging the crank off phase. Pushing the sash shut by hand is not allowed:
the window must be closed by the crank. A full score needs the sash sealed, the crank on phase, the handle folded
flush and both hands clear.

## Parts: real dimensions and materials

Positions are in cm from the midpoint between the arm bases (x forward, y to the robot's left, z above the table).

| Part | Real part | Size and material | Where |
|---|---|---|---|
| Window stool | painted pine board | 13.2 x 50 x 3.2 cm | x 25.0 to 38.2, centred y 0 |
| Window frame | painted pine, cut from `frame.stl` | 49 cm wide, 7.3 cm deep, 37.7 cm tall (jambs 2.5 cm, head 2.5 cm, frame sill 4.2 cm), 9 mm room-side stop lip | room face at x 38.2 |
| Sash | painted pine, `sash.stl`, 3 mm pale-blue acrylic pane | 43.2 x 30 x 3.8 cm, 5 cm stiles, 6 and 5 cm rails, about 1.2 kg | hinged at its +y edge at x 43.8, y +21.6, bottom 8 cm up; starts 9 degrees open (6.8 cm at the lock edge) |
| Weatherstrip | black EPDM rubber D-section, self-adhesive | 6 mm | on the stop lip |
| Operator | Truth Hardware Encore dual-arm casement operator, 50.10.00 series (left or right hand to match the sash) | die-cast housing, 11/32 in 12-spline shaft | spline axis at x 29.5, y -3.0 |
| Nesting cover | Truth Hardware Encore Tango cover with folding handle (bronze) | 13.6 x 4.1 x 3.8 cm, 2.2 cm handle channel, 2.8 cm knob well, pocket floor 4.2 cm up | x 27.5 to 31.6, y -5.5 to +8.1, rim 7.0 cm above the table |
| Folding crank | the die-cast zinc full-flip handle sold with that cover | arm 7.8 x 1.0 x 0.9 cm, 60 g; knob 1.4 cm across, 1.8 cm tall, 7.8 cm from the spline | starts unfolded, 3 mm above the cover, pointing +y; knob at x 29.5, y +4.8 |

Half a turn of the crank (180 degrees, the knob passing round the room side) closes the sash. The crank turns on to
200 degrees, the overtravel a 6 mm EPDM bulb gives under compression. At 180 degrees the folded arm drops into the
channel with 6 mm per side and the knob into its well with 7 mm per side. Off phase by more than about 4 degrees it
lands on the cover walls and stays proud. The forces are small: about 1.5 N at the knob to turn the crank, under 2 N
to flip the handle.

## The scripted demo

`demo_script` (closed loop, `closedloop.py`): the left arm opens its pads into a 26 mm cradle round the knob and pushes
the crank from 0 to 70 degrees along the room-side arc. It opens and goes home. The right arm then cradles the knob
and pushes it on to 180 degrees, which closes the sash, and trims the phase to within 1.5 degrees. Then it pinches
the knob across x and swings the handle up and over, in the vertical plane through the spline, to 112 degrees, past
the detent's centre. It opens and lets the detent spring and gravity fold the handle into the pocket. Both arms
finish at home with open grippers, well clear of the window. The hands only touch the knob, never the sash.
The demo reads the live crank (spline) angle from the simulator to pace and trim its push, plus its joint encoders
and gripper opening; every other target comes from the fixed layout (object_jitter=0).

## Fidelity

The operator's worm and arm are modelled as a joint coupling (-0.050 rad of sash per rad of crank), not meshing
teeth. A real worm self-locks, so a closed sash cannot be pushed to turn the crank, and the sash can only be closed
through the crank. The coupling works both ways, so the scorer stands in for the lock: any finger pad touching the
sash or pane (over 0.05 N) scores 0 for the whole run. The fold detent is a spring with rests at 0 and 180 degrees.
The crank shaft has 0.12 N m of joint friction (hold), about what a worm operator feels; its exact value in Isaac is
not confirmed and the score does not depend on it. The weatherstrip is look-only; the sash's travel ends at its
joint stop, and the crank's 200 degree stop stands for the seal's compression. All parts are rigid. The simulator
scores the final step without opening the grippers, so a hand still within 3 cm of the operator scores 0.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Crank the casement window shut with the folding handle, then fold the handle flush into its cover pocket and let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=close_casement_window_and_nest_folding_crank.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/casement-dream

# Scripted demo (closed loop)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_casement_window_and_nest_folding_crank.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/casement-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the layout is fixed.
- **Runtime per trial:** the demo takes about 71 s of simulated time (1,426 steps, ending with 10 s parked);
  a Dream-1 trial runs up to 2,400 steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 340 s for the demo and about 400 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds, otherwise `score()` at the end of the run.

## Scoring

Checked every step over the last 1.0 s (20 steps), with both hands at least 3 cm clear of the operator and every
joint moving under 1 degree per second; if that does not hold for the whole last second, the score is 0.

- **Sealed:** the sash's lock edge within 3 mm of the weatherstrip (sash at least 0.150 rad closed from its start).
- **Indexed:** the crank within 5 degrees of the pocket's phase (180 degrees from its start).
- **Nested:** the handle folded at least 175 degrees, its arm top at or below the cover rim.

Score 1.0 if sealed, indexed and nested; 0.5 if sealed and indexed only; 0.25 if sealed only (over-cranked toward
200 degrees, or folded onto the cover walls); 0 otherwise (sash more than 3 mm open, a hand still on the handle,
still moving). Any finger pad touching the sash or pane at any time scores 0 for the run.

A real trial measures the same: a ruler at the sash's lock edge (3 mm), a protractor mark on the cover for the crank
phase, a straightedge across the cover rim (the folded arm must not lift it), a top photo, and the video for sash
contact and release.

## Real-world build and reset

Parts list: Truth Hardware Encore dual-arm casement operator (50.10.00 series, the hand that matches a sash hinged on
its left as seen from the room) with the Encore Tango cover and folding handle sold for it; a 6 mm black EPDM
D-section self-adhesive weatherstrip; two 75 mm steel butt hinges; 3 mm acrylic sheet; pine boards for the stool,
frame and sash; #8 wood screws.

`fabricate.py` writes the custom parts as ASCII STL in mm, from the same dimensions as the scene: `stool.stl` (with
the operator's 4 pilot holes), `frame.stl`, `sash.stl` (with the 3 mm pane groove) and `start_gauge.stl` (a 68 mm
block that sets the start opening). Run `python3 fabricate.py out_dir`; the files serve as print files or as cut
drawings for timber.

Assembly: screw the frame to the back edge of the stool and clamp the stool to the table; fit the pane in the sash
groove and hang the sash on its left (+y) stile on the two butt hinges; stick the weatherstrip on the stop lip;
screw the operator to the stool at the pilot holes and link its arm to the sash bottom rail; snap on the cover and
press the handle onto the spline so it points +y, unfolded, with the sash 68 mm open.

Reset (about 15 s): flip the handle out, turn it back half a turn to its start mark (pointing +y) and check the
opening with the gauge block at the lock edge. Nothing is consumed and the parts stay dry and clean.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
