# End-of-day chairs up: lift a preschool chair, turn it over and set it seat down, legs up, on the table

A preschool chair stands on the floor in front of a low table. Both hands take it by the side edges of its seat, lift it, turn it over through half a turn about the line between the hands, and set it down gently on the table, seat down with its legs up, its back hanging over the table's near edge. Then they let go and move away.

**Why it matters (Societal relatability).** At the end of the school day the chairs go up on the tables so the floor can be swept and mopped. Almost everyone has done it, and preschool staff and school janitors do it every day with child-sized chairs and low tables. The task is the physical act inside that routine: turning a light, smooth chair over without letting it swing, knock the table or fall, and landing it flat.

**Ability tested:** a two-hand lift and half-turn of a light, slippery object whose centre of mass is off the seat's middle, then a controlled set-down on a table edge: no fall, no knock, no slide.

**Nearest tasks.** Picking and placing an object keeps it the same way up; here the difficulty is the turn and the landing on a table's edge with the back hanging over.

**Arms.** Both. The lift and the turn need a hand on each side of the line through the centre of mass; with one hand the chair tips in the grip (`onehand` in the table below scores 0: it tips and its back strikes the table).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2000 \
  --instruction "Put the child's chair up on the table: lift it from the floor with both hands, turn it over and set it down gently, seat down with its legs up, the seat's back edge on the table's near edge or up to 3 cm past it, without dropping it, knocking the table with it or sliding it along the top. Then let go and move your hands away."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                            # full score, with a video
python demokit.py task.py OUT --no-record --mode retry,palm,drop,none,onehand,backknock   # the routes below
python check_score.py                                                # the scorer on hand-built trials, with no simulator
```

What the demo does, each move closed on the chair's observed pose:
1. Both hands pinch the seat plate at its side edges, on the line through the chair's centre of mass (5.1 cm behind the seat's middle), fingers pointing in.
2. They lift it and turn it over about the line between the hands while that line rises to 50 to 55 cm over the floor, carry it over the table and lower it until the seat is 6 cm over the top.
3. They tip it onto the seat's far edge, lowering by feedback on that edge's observed height (2 cm/s, then 1 cm/s in the last 2 cm), until the edge rests on the top with the near side about 8 cm up (seat face 20 degrees from flat, inside the 30 degrees the scorer allows a seat face on the top).
4. The right hand lets go of the seat, moves out and round, and pinches the back plate's side edge, which hangs in front of the table's near edge, 18 cm below the seat face; the left hand holds the seat meanwhile and pushes back any slide it reads. Then the left hand lets go and backs out along its approach and up, until each of its fingertips reads more than 4.5 cm from the chair.
5. The right hand lays the chair flat about the far edge, lowering by feedback on the near side's observed height (2 cm/s, 1 cm/s under 3 cm, 5 mm/s under 1 cm). It reads the seat's slide from where the seat first rested and pushes it back; it stops lowering while the slide is over 5 mm.
6. It opens and backs out along its approach until its fingertips read more than 4.5 cm from the chair, then 3 cm more, then moves toward the arms in steps that are kept only while they take the hand further from the chair.

The demo reads the chair's pose, the arms' joint states and the fingertip positions; it reads no contact force and nothing from the scorer. Nothing in the scene is random: `object_jitter=0`.

## The scene and what is simplified

The scene is the robot bench, whose top stands for the classroom floor, with a table on it and the chair. Positions are (x forward from the arm bases, y left) in cm.

| Part | In the scene | Published | Source |
|---|---|---|---|
| Chair width x depth x height | 31.8 x 35.6 x 50.8 cm | 12.5 x 14 x 20 in | Flash Furniture product sheet; staples.com |
| Seat | 25.4 cm square, face 27.3 cm high | 10 x 10 in at 10.75 in | dealer listings (below) |
| Back width | 27.9 cm | 11 in | dealer listings |
| Back height over the seat's underside | 24.7 cm | 10.5 in (26.7 cm), measured into the edge band | dealer listings |
| Chair mass | 1.4 kg | 2-pack 6 lb (quill.com, staples.com), 10 lb carton (webstaurantstore.com) | estimate: 3 lb a chair |
| Shell and leg section | 12 mm plates, 25 mm tubes | not published | estimate |
| Table top | 76.2 x 55.9 cm, 2.9 cm thick, face 30.5 cm high | 30 x 22 in, 1 1/8 in, 12 in leg option | communityplaythings.com C241 |
| Table legs | 45 mm square | not published | estimate |

- **Chair:** Flash Furniture Whitney stackable preschool chair, model YU-YCX-003: one moulded polypropylene shell on four splayed legs. It stands with its seat centre at (10, 0), facing the arms. In the scene the seat and back are flat plates and the legs straight tubes from the seat's corners; the real shell's dished seat, edge band and hand hole between seat and back are not modelled.
- **Table:** Community Playthings Woodcrest rectangular table C241, 12 in high, the lowest of the maker's 12 to 20 in heights, chosen so the arms reach it: solid maple top, no apron. Long side across the arms, near edge at x = 29.8, centred on y = 0. The table is fixed.

`python check_score.py` prints the scene's sizes against the published ones first (scene / published, cm):

```
chair overall width (feet, outside): 31.8 / 31.8 (12.5 in W)
chair overall depth (feet, outside): 35.6 / 35.6 (14 in D)
chair overall height (back top): 50.8 / 50.8 (20 in H)
seat width: 25.4 / 25.4 (seat 10 in W)
seat depth: 25.4 / 25.4 (seat 10 in D)
seat height: 27.3 / 27.3 (seat 10.75 in H)
back width: 27.9 / 27.9 (back 11 in W)
back height (plate, seat underside to top): 24.7 / 26.7 (back 10.5 in H: the shell's back runs down into the seat's edge band, which is not modelled)  DIFFERS
table top across the arms: 76.2 / 76.2 (C241 top 30 in)
table top depth: 55.9 / 55.9 (C241 top 22 in)
table height: 30.5 / 30.5 (12 in legs)
table top thickness: 2.9 / 2.9 (1 1/8 in maple)
chair mass: 1.4 kg (estimate: the 2-pack is listed at 6 lb, 2.7 kg)
```

Friction is set so that the shell can be held by two pinching hands at 35 N per finger.

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. `check_score.py` feeds the real `Trial` hand-built states for the honest placing and each lazy route. The chair's motion is read after every physics step (240 Hz), the grasp from the fingertips at that step.

| What happened | Score |
|---|---|
| Both hands hold the chair (pads on opposite faces of the seat plate, the back plate or a leg) with its lowest point 8 cm or more off the floor, or it rests on the top and not on the floor | 0.25 |
| Also the seat's face within 30 degrees of straight down | 0.5 |
| Also the seat down on the top within 5 degrees, the chair let go and still (under 1 cm/s) for 1 s | 0.75 |
| Also the seat's whole back edge on the table's near edge line or up to 3 cm past it (both rear corners of the seat's face in that band, so the edge lies along the near edge within 7 degrees, and both within the table's width); every fingertip point of both hands more than 3 cm from the chair's surface (the seat and back plates as solids, each leg as a tube), and every arm link's axis, from the upper arm to the palm, more than 5 cm from it; held 2 s | 1.0 |
| Lifted or turned without both hands: the chair leaves the floor or the top (no part within 15 mm of either) and has not been held by both hands, each with both pads within 2 mm of the chair's surface, so a hand beside it, or hovering, does not count, for 0.1 s without a break by its first 0.1 s in the air; or, from lift-off until the end of the turn (its seat's face within 35 degrees of down), while it is off the floor and the top, the two hands together fail to hold it for more than 0.1 s in all; or its seat's face turns past 145 degrees from down (from upright toward down) while any part of it is on the floor or within 5 mm of it. After the turn (seat's face within 35 degrees of down) one hand may let go and re-grip | 0 |
| Dropped: no hand holds it, no part rests on the top or the floor, and it falls more than 12 mm | 0 |
| Set down too hard: any part reaches the top or the floor faster than 0.5 m/s | 0 |
| Struck the table: any point along a leg's whole length or on the back, or on the seat plate (its faces and edges) while its face is more than 30 degrees from down, within 3 mm of the tabletop, or any part within 3 mm of a table leg, while the chair is held or moving. The parts are read as points no more than 7 mm apart, each with its part's radius, so a touch between two points reads within 1 mm of its true gap. The back in front of the table's near edge, where it hangs once placed, is let off only while the seat faces down within 30 degrees, and then only if it arrives there slower than 0.5 m/s | 0 |
| Dragged: the seat's centre moves more than 3 cm, while a seat corner is within 4 mm of the top, from where that set-down began. Lifting the seat 1 cm clear of the top ends a set-down, so a second try starts afresh | 0 |

The tiers are read from the state at the end; only the zeros latch.

A real trial reads the same with a ruler, a photo from above and a 240 fps phone film: the seat's rear edge against the table's near edge, the seat's tilt with a level, the fingers' distance against a 3 cm mark, a fall or a knock frame by frame, and the slide against a tape on the top.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` | The whole task, on each of three lab PCs (RTX 4070 Ti SUPER) | 1.0 at step 1001 (50.05 s), all three; again at the final contact rule (2 mm, 0.1 s hold, two hands in the turn) on a fourth lab PC: 1.0 at step 1001 |
| `full` | The same on an RTX 5090 | 1.0 at step 1003 (50.15 s) |
| `full` | The same on the 5090 at the final contact rule (each pad within 2 mm of the chair, 587604362) | 1.0 at step 1003 (50.15 s) |
| `full --jitter 0.01 --seed N`, N = 1 to 4 | The chair's start shifted up to 1 cm | 1.0 in all four (992 to 1002 steps) |
| Dream-1 | The harness's model, one valid run on the final code: it ran all 2000 steps (100 s) and never ended the trial. Both arms reach to the top of the backrest and hover there; the chair stays upright on the floor and is never lifted (film: `model_run.mp4`) | 0 |
| `retry` | A second try: the far edge is set down 5 cm to the left of the landing (seat centre at y = 4.6 cm), lifted back 6 cm clear, moved back, and the set-down done again | 1.0 at step 1208 (lab PC), 1.0 at step 1205 (5090, final rules) |
| `palm` | The full run, but while the right hand backs out the left comes back from 40 cm above and rests its closed fingertips 2 mm over the upturned seat, 5 cm in from both of its middle lines | 0.0 in every GPU run (fingertips end 1.5 mm from the seat): the left hand's way down pushed the chair 2 to 4 cm along the top, so dragged scores 0 first. The resting hand alone is checked on hand-built states in `check_score.py` (0.75) |
| `ghostseat` | The full run, but the right hand's jaws close to the plate's width with the fingertips 10 mm outside the seat's side edge, never touching; the left hand alone lifts and turns | 0.0 on the 5090 (eaca8da62, same rule): not held by both hands in contact, so lifted without both hands. Before the rule it scored 1.0 in check_score |
| `ghostleg` | The right hand's jaws close to a rear leg's width 2 cm beside it; the left alone does the act | check_score only (0.0; 1.0 under the old rule). The arm cannot reach that pose in the simulator (IK miss at 15 and 20 mm), so no GPU run |
| `hoverlift` | The full run, but the right jaws stop 2.8 mm off the seat and never grip it, so the left hand alone lifts and carries it | 0.0: lifted without both hands (held by the left only, latched 0.1 s into the air); the chair then also strikes the table's near edge with its back (2.6 mm) |
| `hoverlift_19` | The same, but the right jaws stop 1.9 mm off the seat (inside the 2 mm that counts as contact) | 0.0 on the 5090: the pads never touch, so the left hand alone lifts it; lifted without both hands, latched 0.02 s into the air (held by the left only); then the back strikes the table's near edge (2.9 mm) |
| `onehand_carry` | Both hands lift it; the right lets go and the left alone carries it through the first third of the turn; the right takes it again | 0.0 on the 5090: lifted or turned without both hands (the left alone held it for 1.27 s), latched in the carry |
| `floorflip` | Both hands hold it, but the turn's grip line is only 14 cm up, so the chair turns over with its legs and back on the floor | 0.0 on the 5090: the seat's face passes 145 degrees from down with a part within 5 mm of the floor (latched at that moment, both hands holding). A one-hand tip over on the floor, then a two-hand lift to the table, is checked on hand-built states in `check_score.py` (0.0) |
| `onehand_turn` | Both hands lift the chair; then the right lets go and the left alone turns and carries it | 0.0: lifted or turned without both hands (latched while the left held it alone in the turn); its back also strikes the table's near edge (2.9 mm) |
| `drop` | The same, but the fingers open where the open-loop lowering ends, the seat 6 cm over the top | 0.0: dropped, and set down too hard |
| `none` | The arms wait | 0.0 |
| `onehand` | The left hand alone takes the seat and runs the whole path; the right arm stays at its ready pose | 0.0: the chair tips in the one grip and its back strikes the table's near edge (2.8 mm); it falls back onto its feet |
| `backknock` | The full path with the turn made 14 cm lower, so the back swings into the table's near edge half way through the turn | 0.0: struck, the back at the table's near edge (2.9 mm) half way through the turn |

`palm_soft` (the same with the hand stopping 1.2 cm off the seat, expected 0.75) was run twice on GPU and is not reported: the hand's arm pushes the chair along the top in the same way as in `palm` (0.0 both times, the chair ends where it does in `palm`). A resting hand scoring 0.75 is checked on hand-built states in `check_score.py` only.

In the filmed run the far edge rests on the top 20 degrees from flat, the chair lands flat (0.0 degrees) at 0.10 m/s with no fall, its seat's rear edge 7 mm past the table's near edge.

`check_score.py` runs the real `Trial` on 50 hand-built trials and prints the scene's key sizes against the published ones. Each scores as the text says: the honest placing (1.0); the seat settling 11.9 mm (1.0); set down while held at 1 cm/s (1.0) or arriving at 0.8 m/s (0); let go 15 mm, 4 cm or 12 cm over the top (0); the rear edge 25 mm inside the edge (0), 25 mm past (1.0) or 45 mm past (0.75); the seat tilted 20 or 8 degrees (0.5), or 3 or 4.5 degrees (1.0); a hand 2 cm off the seat edge (0.75) or still pinching it (0.5); the end held 1 s (0.75); held seat down 25 cm up (0.5) or upright (0.25); placed then lifted 10 cm (0.5); nothing done, left on the floor, knocked over, lifted by one hand, set back upright or upside down on the floor (0); upright on the top, never turned (0.25); dropped from 20 cm (0); dragged 9 cm (0); pushed off the table (0); put on its side on the top (0.25); carried with its feet on the top (0), with a leg's shaft touching the top's near edge (0), or turned with the seat's edge brushing the top (0); held on its side with a leg's shaft 1 mm off the top's near edge, between two of its sample points (0); a second try, set down 5 cm to the side, lifted 3 cm clear and set down again (1.0), against the same first touch pushed back along the top (0); a hand resting on the upturned seat at a quarter's centre, 6 cm from its edges (0.75), or a forearm resting on a leg (0.75); turned by both hands on the floor, then lifted from the floor and carried to the table by one hand (0); lifted by both hands, one hand letting go for 0.5 s in the turn and taking the seat again (0); a foot 2 mm from one of the table's legs while carried (0); the back swung onto the table's near face half way through the turn (0), or into it at 0.8 m/s seat down (0); the rear edge on the near edge line, the back against the near face (1.0); the seat centred 37.5 cm across, half past the table's side (0.5, not on the top); the chair turned 10 degrees about the vertical, one rear corner 4 cm past the near edge (0.75), or 5 degrees with both corners in the band (1.0).

## Runtime and seeds

The full demo takes 50.05 s of simulated time (1,001 steps of the 2,000-step limit), about 4 to 6 minutes of wall time on one RTX 4070 Ti SUPER at 640 x 360, start-up apart. There is no randomness at `object_jitter=0`.

## A real trial

Use a Flash Furniture YU-YCX-003 chair and a Community Playthings C241 table, 12 in high, or any preschool chair of about that size with a flat seat plate and a table of about that height. Put the chair on the floor with its seat centre 20 cm in front of the table's near edge; read the seat's rear edge against the table's near edge with a ruler, the seat's tilt with a level or a photo, the hands' clearance against a 3 cm mark, and the fall, knocks and slide from a 240 fps film. Tools: a steel ruler (about 5 USD), a bubble level (about 8 USD), masking tape on the top marked every 1 cm (about 3 USD), and a phone's 240 fps slow-motion mode, which resolves a 0.5 m/s landing to about 2 mm a frame.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Chair sizes: dealer listings of the Flash Furniture Whitney stackable preschool chair, 2-pack YU-YCX-003 (webstaurantstore.com item 354CX003GRN; quill.com item 2YUYCX003BLUE) and Flash Furniture's own product page 2-YU-YCX-003-RED-GG. Flash Furniture is a trademark of its owner, named for identification only.
- Table: Community Playthings, Rectangular Woodcrest Table C241, 12 in high (communityplaythings.com; dimension drawing and product guide). Community Playthings is a trademark of its owner, named for identification only.

## Licence

MIT
