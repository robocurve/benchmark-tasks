# Change the game in an NES front-loader

An original Nintendo Entertainment System (NES-001) stands on its shelf board with a red-label Game Pak loaded and
the chamber lid shut. A blue-label spare lies flat on one of two wooden stands (plain pine display blocks)
in front of it; the other stand is empty. The loaded Game Pak is held down by a hidden push-push catch. The robot
must open the lid, press the loaded Game Pak *down* until the carrier pops up, draw it straight out and lay it on the
empty stand, slide the spare in to the connector, press it down until the carrier catches, and shut the lid.

Instruction: "Swap the game in the console for the spare one: put the old game on the empty stand, load the spare and
leave the console closed, ready to play."

Results (simulation only, no hardware trial): the scripted demo scored 1.0 in each of lots A and B. Dream-1 scored 0
in both lots: it moved its hands near the spare and the empty stand but never opened the lid or touched either game,
and both trials ran to the 3,000-step limit (no refusal).

## Why this task

- **The hard part is a hidden push-push catch.** A caught Game Pak cannot be pulled: it must first be pressed down
  until it pops up; the manual forbids pulling or prying it. The new Game Pak must be slid fully home along the raised
  carrier, then pressed down past the catch, or the console does not play. None of this is visible from outside; the
  robot must know the procedure.
- **The maker's procedure is the task.** Nintendo's NES Control Deck manual: "Open the Chamber Lid. Gently push down
  on the Game Pak allowing it to pop up. Do not attempt to pry the Game Pak up. Pull the Game Pak out." To load:
  "Insert a Game Pak into the Chamber (label facing up) and push it all the way in. Press down on the Game Pak until it
  locks into place and close the Chamber Lid." ([manual transcription](https://www.world-of-nintendo.com/manuals/nes/control_deck.shtml))
- **Routine home job.** Changing the game is the routine hands-on job with any cartridge console, and the NES-001
  is still played. The lid has no handle: it is lifted by the ledge along its lower edge and swings past its
  balance point to stay open.

## How this differs from close tasks

| Task | What it tests | Difference here |
|---|---|---|
| TK-3UAHWY, game disc into a console slot | Free a disc from its case's hub, stand it up, slide it into a slot | The media here is held by a hidden push-push carrier: pressed down to free it, pulled only after it pops up, slid fully home along the raised carrier, then pressed past the catch. A slot has nothing to toggle and nothing that forbids pulling |
| TK-QBS8FN, capsule machine | Empty the collector and load a fresh capsule | No caught part to release by pressing; no catch to set by pressing |
| TK-JXVV9E, till insert into the cash drawer | Swap an insert, shut the drawer on its catch | The catch there is closed by shutting a drawer; here a press both frees and sets the same catch, and pulling before the press is the failure |
| TK-UKX5MH, carriage onto its rail past a spring plunger | Slide onto a rail past a plunger | No release before removal, no pressing to toggle a catch, no lid |
| TK-FA3EDH, ratchet dispenser cartridge | Swap a cartridge on a dispenser | No spring-return carrier, no push-push catch, no lid |

## Scene (lot A; all sizes in mm unless stated)

| Object | Real product / material | Size and placement | Source of size |
|---|---|---|---|
| Console | Nintendo Entertainment System NES-001, grey ABS | 256 W x 203 D x 89 H, about 2.3 kg; front face centred at (345, 0) from the midpoint between the arm bases, front toward the robot; fixed (real: rubber feet; the task's pushes stay under about 10 N) | dimensions.com NES entry (256 x 203 x 89 mm, 2.27 kg) |
| Shelf board | pine board (TV bench shelf) | 252 deep x 320 wide x 30 thick, under the console, flush with its recessed black front strip | drawing in `fixture/` |
| Chamber lid | the console's top-front panel | 68 deep x 141 wide, hinged along its back edge 87.5 above the board; front face 43.5 tall whose lower edge stands out 8 mm as a finger ledge; 48 g; shut at start; opens to 110 degrees and rests there past its balance point (about 103 degrees, from its shape) | estimate |
| Cartridge carrier | rails, lead-ins and the 72-pin connector, black | rails with 1.5 mm side play on the Game Pak (123 between the rail walls); connector stop face 139.5 behind the console face; pivots about a line 149 behind the face and 56 up; pops up 0.10 rad (label end about 14 higher); 60 g | estimate |
| Old game | NES Game Pak, grey ABS, red label | 120 W x 133 L x 20 thick, 100 g; label up, connector end in, label end 6 behind the console face; carrier caught down | Game Pak 120 x 133 x 20 mm (iFixit, Wikipedia); mass estimate |
| Spare game | NES Game Pak, blue label | same; lying flat, label up, label end toward the robot, 50 over the left stand's front edge | as above |
| Stands (2) | pine display blocks | 80 deep x 100 wide x 60 high, centred at (295, +150) and (295, -150) | drawing in `fixture/` |

The stands hold each game the way a game sits at a shelf edge, label end out, so it can be taken by its 20 mm
thickness; a game lying flat on the bench cannot be gripped that way. Lot B moves the spare to the right stand;
everything else is the same. Trials are run on lots A and B and averaged; the screenshot shows lot A. No object is
jittered (`object_jitter=0`).

## Scoring

**Score 0 for the whole trial** (it stays 0 even if everything is later put right) if, at any moment:

1. any point of the loaded game moves more than 5 mm before its catch has been released (pulled or prised while
   caught);
2. either game tips more than 15 degrees out of the carrier's plane while any part of it is inside the chamber
   (prising one end up or down);
3. the spare slides home onto the connector (from more than 10 mm short) while the carrier is down, caught or held
   down (label end more than 7 mm below its popped-up height): the carrier must be popped up to take a Game Pak.

Otherwise the **final state** counts, and only when both hands are at least 5 mm from both games, the lid and the
carrier, and everything has been still for the last 1 s:

| Score | Final state | How it is read in a real trial |
|---|---|---|
| 0 | The old game is not lying flat on a stand | overhead photo |
| 0.25 | The old game lies flat (within 10 degrees) on either stand, its middle over the stand's top, underside within 3 mm of it; the spare is not fully in | overhead photo, ruler against the stand |
| 0.5 | ... and the spare is fully in: connector end within 3 mm of fully home (3.5 mm of the connector's stop face), square within 5 degrees, label up | side photo through the open lid, ruler on the label end against a mark for "fully home" |
| 0.75 | ... and the carrier is caught down: the spare's label end at its down position (within 1.1 mm) and staying there with the hands clear | after the trial the judge opens the lid and sees whether the Game Pak stays down or pops up |
| 1.0 | ... and the lid is shut: its front edge within 3 mm of closed | photo of the front |

Every outcome falls on exactly one row: each row needs every condition of the rows above it.

### How the simulator measures it (`task.py`)

- `observe()` runs after every control step. It updates the push-push catch (`latch_update()`), the whole-trial
  faults (`moved_in_tray()` for rule 1, `pry_deg()` and `in_chamber()` for rule 2, the spare's arrival at the
  connector for rule 3), the face-sampled hand clearance (`hands_clear()`: every collision face of `gripper`,
  `tip_left` and `tip_right` sampled at 3 mm spacing, 8 mm threshold = 5 mm plus the sampling allowance) and a 1 s
  stillness hold (both games below 3 mm/s and drifting under 0.5 mm / 1 degree, the lid and carrier under 1 degree).
- `score()` returns the ladder value (`tier()`) of the state it is given only if the step record matches that exact
  state, the hands are clear, the hold has run 1 s and no whole-trial fault was recorded. Unknown or non-finite
  states fail closed. The record is cleared only by the simulator's reset hook, never by object positions.
- `seated()` reads the spare's pose in the carrier's own frame (`pak_in_tray()`), so "fully in" holds whether the
  carrier is up or down; `locked_down()` needs the catch engaged and the carrier within 0.008 rad of down.

### Bypasses and near misses (`near_misses.py`, 34 cases, all pass)

| Shortcut or near miss | Score |
|---|---|
| Do nothing | 0 |
| Old game pulled 8 mm with the carrier still caught, then everything finished | 0 (rule 1) |
| Caught old game's label end prised up 9 mm (3.9 degrees) and let back down, then everything finished | 0 (rule 1) |
| Popped-up old game prised up 20 degrees inside the chamber, then everything finished | 0 (rule 2) |
| Empty carrier pressed down and caught, spare pushed in flat, lid shut | 0 (rule 3) |
| Carrier freed but held down by hand, spare slid in flat, then pressed to catch, lid shut | 0 (rule 3) |
| Spare lifts 2.5 mm off its rails for a moment during the catching press, then settles | 1.0 |
| Old game dropped on the bench, spare loaded and caught, lid shut | 0 |
| Old game half off its stand, or propped 12 degrees | 0 |
| Spare in, carrier left popped up, lid pushed shut over it | 0.5 |
| Spare caught down 3.5 mm short of fully home | 0.25 (3.0 mm short still counts) |
| Spare loaded label down, or skewed 6 degrees | 0.25 |
| Caught, then pressed again so the carrier pops back up | 0.5 |
| Fingertip 3 mm from the spare at the end, or hands unknown | 0 |
| Lot B replay of lot A (old game laid on the spare, nothing taken) | 0 |

## Modelling and fidelity

- Rigid boxes at the NES-001's published outline and the Game Pak's published size. The chamber, lid, carrier and
  connector shapes, the carrier's 0.10 rad travel, its spring (2.5 N m/rad toward 0.24 rad: about 5.5 N at the label
  end to press past the catch, about 1.7 N popping up), all masses and frictions (ABS 0.35/0.30, pine 0.5/0.45) are
  estimates; no measured NES-001 internals were available.
- The push-push catch is code (`latch_update()` and `_tray_upper()`): the carrier's upper joint stop is 0 (down)
  while caught and 0.10 rad while free, and it toggles each time the carrier is pressed past -0.018 rad (2.6 mm below
  down at the label end). Over-travel stops at -0.030 rad (4.3 mm).
- The 72-pin connector is modelled as a stop face: its light grip on the Game Pak's board edge (not measured) and
  electrical contact are not modelled.
- The lid has no spring or detent; it stays open because its open stop (110 degrees) is past its balance point.

## Demo

`demo()` is open loop from the published layout of the trial's lot: it reads only which stand holds the spare
(visible in the cameras), never a joint angle, object pose or lot name.

1. The arm on the spare's side lifts the lid by its ledge with the top of its closed fingertips (fingers pointing
   into the console), following the ledge's arc past the balance point; the lid settles on its stop.
2. The other arm presses the old game's label end down with the underside of its closed fingers until the catch lets
   go, then rises while the carrier pops up, grips the label end across its 20 mm thickness (fingers tilted 5.7
   degrees to match the carrier), draws it straight out and lays it on the empty stand.
3. The first arm picks the spare off its stand by its overhanging label end, slides it into the raised carrier to the
   connector, lets go, closes its fingers and presses it down past the catch.
4. The second arm presses the open lid's ledge down from above until the lid swings past its balance point and shuts.

The simulated arm is compliant (a held 100 g Game Pak hangs about 5.5 mm below the commanded grip), so carrying
waypoints aim 5.5 mm higher, and each press is commanded 35 mm past the bottom of the carrier's over-travel; the
force that reaches the Game Pak is limited by that compliance to a few newtons, a firm thumb press. `tilt.py` supplies
the tilted and horizontal waypoints to the stock scripted planner; `parts.py` builds the console, lid and carrier;
`lots.py` sets each trial's layout. None of them changes Dream-1 or the installed harness files.

## Run

From this folder, with object jitter off. Trial k uses lot k mod 2; `TASK_LOT=A` or `B` pins one lot for debugging.

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "Swap the game in the console for the spare one: put the old game on the empty stand, load the spare and leave the console closed, ready to play." --scorer task_score --max-steps 3000 --epochs 2
```

Use the same command with `--policy dream` for Dream-1. Offline scorer tests: `python near_misses.py`.

## Reset

1. Set the console on its shelf board at its mark, front to the robot.
2. Load the red-label Game Pak label up, push it in to the connector and press it down until it catches; shut the lid.
3. Lay the blue-label Game Pak flat, label up, on the lot's stand (A: robot's left; B: robot's right), label end
   toward the robot and 50 mm over the stand's front edge. Leave the other stand empty.
4. Return both arms to the ready pose with the grippers open.

## Licence

Code MIT; documentation and drawings CC BY 4.0. Nintendo, Nintendo Entertainment System and Game Pak are trademarks of
Nintendo, named for identification only. Third-party assets: none; all geometry is generated in `task.py`.
