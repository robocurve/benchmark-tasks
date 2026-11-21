# Square a crooked cabinet door: turn its top cup hinge's side screw with a screwdriver until the gap is even

Two frameless cabinets stand side by side, both doors right-hinged; the left door hangs on the middle partition. It hangs crooked: the gap to the right door is 1.7 mm at the top and 2.6 mm at the bottom. The right hand opens the left door toward the middle by its pull and holds it. The left hand takes a long PH2 screwdriver from the cabinets' top, reaches into the open cabinet, seats the point in the top hinge's side-adjust screw, presses it in (it aims for 13 N; about 12 N is reached) and turns it with one roll of the wrist, about six tenths of a turn. It lays the screwdriver back, the right hand swings the door shut, and both hands let go: the gap is now even, about 2.5 mm top and bottom.

**Why it matters (Impact on jobs).** About 670,000 carpenters (47-2031) and 1.53 million general maintenance and repair workers (49-9071) work in the US (BLS Occupational Employment and Wage Statistics, May 2025). Both adjust cup-hinged cabinet doors after every install and whenever a door drops out of line. O*NET 47-2031.01 (Construction Carpenters) lists as Core work installing "hardware, using carpenters' hand or power tools", and 49-9071.00 lists "Adjust functional parts of devices or control instruments, using hand tools". On a Blum CLIP top hinge the front screw on the hinge arm moves the door sideways at that hinge, so turning the top hinge's screw swings the door's top edge and squares the gap.

**Ability tested:** a closed loop with a hand tool. The screw is reached only with the door open, but the gap shows only with the door shut. The bit must stay seated in a 9 mm screw head inside the cabinet while the wrist turns it, and the right screw must be turned the right way by about the right amount.

**Arms.** The right arm opens and closes the door, the left turns the screwdriver. One arm could do both in turn: the hinge's friction holds the door open above 25 degrees.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "The left door hangs crooked. Use the screwdriver on its hinge to even the gap between the doors, 2 to 3 mm, then shut the door and let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                  # full score, with a video
python demokit.py task.py OUT --no-record --mode MODE      # one of the routes below
python check_score.py                                      # the scorer on hand-built trials, with no simulator
```

What the demo does, each move closed on observed poses:
1. The right hand takes the left door's bar pull, swings the door open 55 degrees toward the middle about its hinge line and keeps hold of it.
2. The left hand takes the screwdriver from the cabinets' top by its handle, the fingers level on two of the handle's flats and the blade along the hand's approach axis.
3. It lifts it, carries the blade out over the cabinets' front, lowers it in front of the left cabinet, brings the tip in line with the top hinge's side screw from the driver's observed pose and moves it in along that line, 10 degrees off the screw's axis so that the blade clears the cabinet's side panel.
4. It winds the wrist back 112 degrees, presses the point into the recess until the head's seat reads its 13 N goal (about 12 N is reached), then rolls the wrist through 225 degrees, the hand turning about the driver's own axis with the tip held in the recess. If a stroke leaves the head short of 0.35 turns (a press under 10 N does not drive it), the bit comes out, the wrist winds back and it presses and turns again.
5. The bit comes out of the cabinet, the wrist unwinds in the air, and the left hand lays the driver back where it lay and moves clear.
6. The right hand swings the door shut and lets go; both hands move clear.

The demo reads the door's pose, the screw head's angle, the driver's pose and the arms' joints; it reads nothing from the scorer. Nothing in the scene is random: `object_jitter=0`.

## The scene and what is simplified

Frame: x forward from the line through the arm bases, y left from the midline, z up from the table top, mm.

| Part | In the scene | Published | Source |
|---|---|---|---|
| Cabinet boxes | 300 wide x 300 deep x 450 high, 18 mm melamine board, on a 30 mm plinth, fronts at x 400, joined at y -250 (the pair stands right of the middle) | frameless (European) carcase, 18 mm board | any 18 mm melamine carcase |
| Doors | 298 x 448 x 19 mm MDF, 1.9 kg, full overlay | | estimate (MDF about 750 kg/m3) |
| Hinges | two per door, 80 mm from its top and bottom; both doors right-hinged (the left door on the middle partition); 110-degree swing | Blum CLIP top 110-degree full overlay, 71T3550 | blum.com; doors handed either way, as IKEA METOD doors mount |
| Mounting plates | 0 mm, 8.5 mm thick | Blum 173H7100 | build.com/blum-173h7100 |
| Side adjustment | front screw on the hinge arm, +-2 mm | +-2 mm | Blum CLIP top assembly instructions |
| Reveal | right door square, 2.5 mm from the left door | | the fitter's usual 2 to 3 mm |
| Screwdriver | long-reach PH2 screwdriver: 300 mm blade, 6 mm shaft, PH2 point; 100 mm octagonal handle, 31 mm across the corners; 0.15 kg; lies on the cabinets' top, tip at (482, -288), the handle 55 mm out past the left cabinet's side | PH2 x 300 mm blade, 405 mm overall (for example Wera 350 PH, 05008730001) | wera.de; generic long PH2 screwdrivers sell for under 15 USD |

- **The door** is a rigid body on one pivot for the hinge's 110-degree swing. Friction holds it where it is left above 25 degrees; below 25 degrees the hinge's self-close spring draws it shut. The door sits on two hidden joints, a sideways slide and a tilt in its own plane, which follow the two hinges' side offsets.
- **The side screws** are real bodies with a cross recess (a pocket 4 mm across and 1.5 mm deep, whose wall holds a seated point sideways), each turning on its own joint and sitting on a stiff seat spring (20 kN/m) along its axis, through which the bit's push is read in newtons. The thread and the bit's drive in the recess are a declared rule (`Thread` in `task.py`). The head turns with the driver's roll about its own axis only while all of these hold: the tip within 1.5 mm of the recess's centre, the tip between 1 mm short of the face and 4 mm into it, the driver within 15 degrees of the screw's axis (beyond that a PH2 bit cams out), and a push of at least 10 N. While it drives, a 0.3 N m torque on the driver resists the turn (an estimate for a small hinge screw). Each turn moves that hinge 1 mm sideways. The pitch is assumed: Blum gives the +-2 mm range, not the thread.
- **Forced past the stop:** a screw turned on past the end of its +-2 mm travel turns no further; half a turn of forcing strips it.
- **Not modelled:** depth and height adjustment (the depth screws and the plate cams are fixed).
- **The hinge's path** is one pivot near the door's front face, so the door's edge clears the right door as it opens; a real cup hinge's linkage throws the door out on a similar path.

## Limits

- The screw's resisting torque (0.3 N m) and the thread pitch (1 mm per turn) are estimates, not measured on a hinge.
- The demo grips the handle at 35 N per finger and aims to push the bit with 13 N (`PUSH_GOAL`; about 12 N is reached), within a YAM gripper's roughly 40 N. The 10 N push and 0.3 N m resist are simulation rules; in a real trial the screw turns by hand-tool feel and the paint mark on the head shows the turn.

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. The state is read after every control step.

| What happened | Score |
|---|---|
| At the end: the left door within 2 degrees of shut; both gaps (20 mm below the top and 20 mm above the bottom) 2.0 to 3.0 mm and within 0.5 mm of each other; released (every point on both whole grippers, fingers, pads, palm and housing, at least 30 mm from the cabinets, doors and driver; the driver's tip at least 20 mm from every screw; door and driver still), held 1 s | 1.0 |
| At the end: shut with those gaps, but not released | 0.5 |
| At the end: the top hinge's side screw turned at least a quarter turn the right way (opening the gap at the top) | 0.25 |
| Any time: a side screw forced half a turn past the end of its travel | 0 |

The tiers read the state at the end; only the stripped screw latches.

The gaps follow from the two hinges' side offsets: at 1 mm a turn, a turn of the top screw opens the top gap by 1.21 mm and closes the bottom one by 0.21 mm. From 1.7 and 2.6 mm, anything from 0.28 to 0.99 turns of the top screw puts both gaps in the window; 0.63 turns makes them equal. Turning the bottom screw instead tapers the gap the other way.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` | The whole task, filmed (RTX 3090 Ti) | 1.0 at step 1508 (75.4 s): a first press of 7 N does not drive the head, the second, at 12.4 N, turns it 0.59 turns (0.61 at the end); gaps about 2.4 mm at the top and 2.5 mm at the bottom |
| Dream-1 | The Dream model, one valid session, seed 1, 16 rows per request, 2,400 steps | 0.0: the doors end shut and the screwdriver lies on the table; the screw is never turned. Film: attached to the PR |
| `none` | The arms wait | 0.0 |
| `no_push` | The bit stops 1.5 mm short of the head and the wrist turns | 0.0 (the head does not turn) |
| `off_centre` | The tip is held 2.8 mm off the recess's centre and the wrist turns | 0.0 (the head does not turn) |
| `door_only` | Opens the door and shuts it again, never touches a screw | 0.0 (gaps stay top 1.67 mm and bottom 2.60 mm) |
| `hover` | The bit seated in the top screw, never turned | 0.0 |
| `bottom` | Turns the bottom hinge's screw instead | 0.0: low in the cabinet the driver tilts in the hand and the bit does not seat; a full turn of the bottom screw is checked in `check_score.py` (0) |
| `reverse` | Turns the top screw the wrong way | 0.0: wound the other way first, the bit never seats and the head does not turn; the wrong-way turn itself is checked in `check_score.py` (0) |
| `open_end` | Turns the top screw right but leaves the door open | 0.25 (head turned about 0.4 turns, door left at 55 degrees) |

`check_score.py` runs the real `Trial` on 19 hand-built trials (13 whole-trial cases and 6 cases of the drive rule), each scoring as the text says: the honest run (1.0); 0.35 turns (1.0); the bottom screw (0); the wrong way (0); 1.3 turns, the gaps out of the window (0.25); the bit seated but never turned (0); the door opened and shut (0); turned, door left open (0.25); opened again after the end state (0.25); a hand left on the pull (0.5); fingertips clear but the palm on the door (0.5); the driver left in the screw (0.5); a screw stripped (0); the drive rule: 14 degrees off the axis turns, 20 degrees off does not, 5 N does not, a hover-turn does not, a tip 2.8 mm off centre does not.

## Runtime and seeds

The full demo takes 75.4 s of simulated time (1,508 steps of the 2,400-step limit), about 6.5 minutes of wall time with the film on one RTX 3090 Ti at 640 x 360, start-up apart. There is no randomness at `object_jitter=0`.

## A real trial

Build two 300 x 300 x 450 mm carcases from 18 mm melamine board and screw them together side by side. Hang a 19 mm door on each with two Blum 71T3550 hinges on 173H7100 plates (35 mm cups bored 80 mm from the door's top and bottom). Set the right door square at a 2.5 mm reveal. Set the left door by its top hinge's side screw until the gap reads 1.7 mm 20 mm below the top and 2.6 mm 20 mm above the bottom. Lay a PH2 x 300 mm long-reach screwdriver on the cabinets' top, its tip at (482, -288), the handle's end 55 mm out past the left cabinet's outer side. Read the gaps with a feeler-gauge set in 0.1 mm steps (about 10 USD) at two pencil marks, taking the thickest blade that slides in; evenness is the difference of the two readings; mark the screw head with paint to read the turn; read the release from the video. Protection for a real trial: pad covers on the gripper fingers, tape on the cabinet edges, and a soft mat on the floor under the cabinets.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Blum CLIP top hinges and adjustment: side +-2 mm by the front screw, height +-2 mm by the plate cam, depth +3/-2 mm by the rear cam (blum.com, CLIP top BLUMOTION assembly; Blum CLIP top frameless hinge instruction sheet). Blum is a trademark of its owner, named for identification only.
- Plate 173H7100: build.com/blum-173h7100.
- O*NET 47-2031.01 (Construction Carpenters), 49-9071.00 (Maintenance and Repair Workers, General).
- Employment: BLS OEWS national estimates, May 2025: Carpenters 47-2031, 670,090 (bls.gov/oes/current/oes472031.htm); Maintenance and Repair Workers, General 49-9071, 1,529,700 (bls.gov/oes/current/oes499071.htm).
- Screwdriver: Wera 350 PH, 05008730001 (wera.de); a generic long-reach PH2 screwdriver of this size is the same shape.

## Real items

- **Blum CLIP top hinge, 110-degree full overlay, 71T3550**, on **Blum plate 173H7100** (blum.com/us/en/products/hingesystems/clip-top/overview/; build.com/blum-173h7100). Its front screw on the hinge arm sets the door's side position, and the 35 mm cup sits in the door's back: the mechanism this task turns.
- **Wera 350 PH PH2 screwdriver, 05008730001** (wera.de/en/tools/350-ph-screwdriver-for-phillips-screws): the long-reach point and blade that reach the hinge arm from the cabinet's front.

## Licence

MIT
