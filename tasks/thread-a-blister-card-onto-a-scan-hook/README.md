# Thread a blister card onto a scan hook's lower wire, under its overhanging price label

A pegboard scan hook holds two carded fastener packs on its lower wire. A third card stands in a shelf-ready case
beside it. The robot takes that card, threads its closed hang slot onto the lower wire's upturned tip, which the price
label on the upper wire overhangs, slides it back past the upturn beside the stock and lets go. Stockers and order
fillers (O*NET 53-7065, 2.8 million US workers) stock shelves, racks and hooks with merchandise, and retail
salespersons (41-2031, 3.7 million) arrange and display it, all shift; carded goods hang on hooks like this one. Taking stock off a hook is a lift and a pull. Putting it on is the harder
half: a 32 x 20 mm closed slot in a thin card has to find an 8 mm wire tip that the label plate shields from above,
through a vertical window of 6 mm (the card's top edge passes 3 mm under the plate while the slot's top passes 3 mm
over the tip), without knocking off the two cards already hanging there. Too high and the card meets the label plate;
too low and the card's bridge meets the tip. A robot may tilt the card to pass under the label, as stockers do; the
scorer allows it.

## Parts (real size)

Positions are in cm from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table.

- **Pegboard**: hardboard panel 1.2 cm thick, 34 cm wide, 32 cm tall, standing on the table with its front face at
  x 40.5, from y -20 to y +14; 6 mm holes at 2.5 cm pitch (drawn, not cut). Fixed.
- **Scan hook**, chromed steel, 8 mm wire, fixed: mount plate 0.5 x 3 x 6 cm on the board at y -3. Lower wire: level,
  axis 20.0 cm above the table, from the board to x 22, then a 2 cm upturn rising 4 mm (11 degrees) to a round tip
  centred at x 20. Upper wire: level, 4.4 cm above the lower wire, from the board to x 19.4.
- **Price label plate**, white plastic, on the upper wire's end: 5 cm wide, 3.3 cm tall, 0.4 cm thick, front face at
  x 19, lower edge 2.1 cm above the lower wire's axis (1.7 cm over the wire's top, 1.3 cm over the tip's top). A red
  stripe 5 x 0.6 cm marks its lower part.
- **Three carded fastener packs** (a blister of screws and wall plugs on a euro-slot card), 120 g each, loose. Card:
  11 cm wide, 17 cm tall, yellow with an orange header, 0.6 mm board (built 3 mm thick). Hang slot 32 x 20 mm, centred
  1.7 cm below the top edge, leaving a 7 mm bridge. Blister: blue PET, 6 cm wide, 8 cm tall, 4 cm deep with 4 mm
  chamfers, on the front, from 4.5 to 12.5 cm below the slot centre. Centre of mass 1.7 cm in front of the face and
  8.2 cm below the slot centre.
- **Cards A and B** hang on the lower wire with their blisters toward the robot, faces crossing the wire at x 36.5 and
  x 30.0. They rest at the tilt their weight gives them, about 10 degrees, bottoms toward the board.
- **Card C** stands in the case on its bottom edge, blister toward the robot, back against the rear wall, face at
  x 25, centre y -17.
- **Case**: cut-down kraft corrugated shelf-ready tray, 3 mm board, inside 4.6 cm (x) by 13 cm (y), walls 4 cm tall,
  inside x 20.55 to 25.15, centre y -17, taped down. It holds one row of cards and C is the last one left in it.

Fits: slot over wire 6 mm per side vertically (3 mm per side over the upturned tip's rise at threading height), 12 mm
per side across; card top 3 mm under the label plate at threading height.

Real-world build: a stock 20 cm double-wire scan hook with a label holder on pegboard, three identical carded packs
with a euro slot, the label holder's lower edge 1.7 cm over the wire, a cut-down shelf-ready tray taped down. A real
wire is 5 to 6 mm in a 12 to 14 mm tall euro slot (3 to 4 mm per side); here the wire is 8 mm and the slot 20 mm tall
(6 mm per side on the level wire, 3 mm over the upturned tip at threading height).

## The scripted demo

Right arm only; the left arm holds its ready pose. 947 steps (about 47 s) of the 2,400 allowed, the last 9 s parked
and waiting.

1. **Grip.** The gripper comes over the case's front wall pointing away from the robot, leaned 30 degrees down, and
   closes one pad on each flank of card C's blister, 7.5 cm below the slot centre (just above the centre of mass). It
   confirms the grip from the gripper opening and retries once 4 mm deeper if the pads closed on nothing.
2. **Lift and carry.** Straight up out of the case, back toward the robot until the card's face is at x 17.5, across to
   the hook at y -3 in front of the label plate, then down until the slot is level with the tip.
3. **Thread.** With the slot centre 1 mm above the wire's axis, it moves the card 5.7 cm toward the board in 5 s: the
   tip passes through the slot while the card's top edge passes 3 mm under the label plate. Then 1 cm further, to
   4.2 cm behind the tip and past the upturn, lowering the slot's top edge to 3 mm above the wire.
4. **Let down.** It tips the card about 10 degrees, bottom toward the board, to the tilt its weight gives it on the
   wire, lowers the slot's top edge to 0.7 mm above the wire and only then opens the gripper. Let go upright, the card
   would swing 10 degrees each way and strike card B.
5. **Withdraw.** It draws the hand straight back toward the robot under the label, parks at the ready pose with the
   gripper open, waits until the three cards are still, and stays parked for the 8 s that success has to hold.

The demo reads the live pose of card C from the simulator at every step of the carry (its position to cancel the
arm's sag inside the 6 mm window, its tilt while letting it down) and the three cards' speeds while it waits, with the
fixed layout of the hook and case (object_jitter=0), its joint encoders and gripper opening; it never moves or edits a
part. In the offline contact replay (MuJoCo, not Isaac) only the two fingertips touched card C, nothing touched the
hook or the hung cards, and the run scored 1.0.

## Fidelity

Approximate. The hook's sizes follow a stock 20 cm double-wire scan hook with a label holder and a euro-slot card; the
wire is 8 mm for a real 5 to 6 mm, so the slot is 32 x 20 mm: 6 mm per side on the level wire, 3 mm over the upturned
tip. Bodies are rigid: the card face is 3 mm thick where real board is about 0.6 mm and it does not flex, the blister
is a solid block, and density is uniform. A real card stops swinging within a few swings because its slot edge rubs
on the wire; a rigid contact in PhysX rolls without loss, so the three card bodies carry damping (4/s angular, 1/s
linear), authored with the scene and the same for every policy. The pegboard's holes are drawn. The simulator scores
the last step without opening the grippers, so a card still held, or within 3 cm of a finger, scores 0.

## Files

- `thread_a_blister_card_onto_a_scan_hook.py`: scene, `score()`, `success()`, `demo_script()`.
- `mechkit.py`, `closedloop.py`, `belimits.py`: shared kit (geometry, closed-loop demo, limit checks).
- `task.json`: the task's details.

## Run a trial

```bash
INSTR="Hang the carded product from the case on the scan hook's lower wire, with the two cards already there."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=thread_a_blister_card_onto_a_scan_hook.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/hook-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=thread_a_blister_card_onto_a_scan_hook.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/hook-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; cards A and B start resting on the wire.
- **Runtime per trial:** the demo takes 947 steps (about 47 s), its 9 s parked wait included; a Dream-1 trial runs up
  to 2,400 steps (2 minutes at 20 Hz), 2.5 times the demo. Wall-clock time on one L40S, including Isaac Sim start-up: about 130 s for the demo and about 320 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s, otherwise `score()` at the end of the run.

## Scoring

All from the cards' poses, each in its own frame.

- **On:** the lower wire's axis crosses the card's face inside its 32 x 20 mm slot, and the card is within 20 degrees
  of upright.
- **Depth:** the distance along the wire from the tip's centre to where the card's face crosses it.
- **Released:** every finger point of both grippers is 3 cm or more from card C.
- **Still:** all three cards under 1 cm/s for 1 s.
- **K** (latched over the whole run): card A or B not On for 0.5 s (10 steps in a row, after the first 1 s while the
  hung cards settle), or dropped 3 cm. A card cannot leave its closed slot and come back, so a shorter miss is a swing
  past 20 degrees, not a card off the wire.
- **T** (latched): card C ever On at depth 10 mm or more.

| Score | Outcome |
| --- | --- |
| 1.0 | C On at depth 30 mm or more (past the upturn, beside the stock), no K, released, still |
| 0.5 | C On at depth under 30 mm (left on the upturn or near the tip), no K, released, still |
| 0.25 | T, then C off the wire at the end, lying on the table or in the case (lowest point within 1.5 cm of the table top), no K, released, still |
| 0 | K at any time; any card off the table; C still held or not released; cards not yet still at the end, even with C past the upturn; C never threaded; nothing done |

Tilting the card on the way is allowed. The scorer prints one `SCORE step N: ...` line when K or T latches and whenever
a gate changes (still, released, depth), so a run's log shows why it scored what it did. On a real table: a ruler from the tip to C's face for depth, and a photo of the
wire through each slot for On and K.

## Real-world reset

Lift card C off the wire and stand it back in the case against the rear wall. Re-hang A and B at tape flags on the
wire (x 36.5 and x 30.0) if they moved. Nothing is consumed.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
