# Set a soda can in a seat-back fold-down cup holder: fold down its tray, swing out its ring, set the can in upright

A seat-back panel section stands on the bench facing the arms, with a fold-down drink holder screwed to its face. The holder stows flat: its ring hangs in front of the plate and its tray stands folded up over it. One hand takes the tray by its sides and folds it 90 degrees down to its stop, then takes the hanging ring by its sides and swings it 90 degrees up and out to its stop. The other hand picks up a full 12 oz soda can high on its body, centres it over the ring, lowers it straight through (2 mm of play a side) until it stands on the tray, and lets go. Both hands back away; the can stands upright on the tray inside the ring.

**Why it matters (Societal relatability).** Fold-down cup holders are on car and van seat backs, minivan doors, buses and cinema and stadium seats, and almost everyone has set a drink in one. North American cinemas alone sold about 760 to 780 million tickets in 2025. The US fleet is about 289 million light vehicles (2025). Sources: cinema tickets, Axios, "The 2025 box office comeback that wasn't" (2 Jan 2026): https://www.axios.com/2026/01/02/domestic-box-office-2025-movie-theaters (about 760 to 780 million admissions in the US and Canada, trackers differing slightly); vehicles, S&P Global Mobility, vehicles in operation, May 2025: https://www.spglobal.com/automotive-insights/en/blogs/2025/05/average-age-of-vehicle-in-us .

**Ability tested:** working a stowed two-part folding mechanism to its stops in the order it allows (the tray covers the ring), then seating a tall, top-heavy 66 mm can straight down through a 70 mm ring onto the tray floor 84 mm below it: a can lowered at an angle catches the rim and tips.

**Arms.** The demo uses both: the left opens the holder, the right carries the can. Both arms are not forced. The real holder has no published spring or detent, so each part stays where it is put, and one arm alone can open both, then fetch and set the can (`onearm` below scores 1.0).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 1200 \
  --instruction "Put the can of soda in the seat-back cup holder: fold down its tray, swing out its ring, and set the can upright through the ring onto the tray, then let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                                  # full score, with a video
python demokit.py task.py OUT --no-record --mode onearm,ringonly,beside,hold,drop,none     # the routes below
```

What the demo does:
1. The left hand comes in with its fingers 60 degrees from straight down, toward the panel, and pinches the stowed tray across its side lips (84 mm) at 15 N a finger.
2. It carries the pinch round the tray's hinge line, the grip held; the tray turns between the pads as it folds 90 degrees down to its stop. The hand lets go and backs off.
3. It pinches the hanging ring across its sides (80 mm) and carries the pinch round the ring's pivot line, up and out 90 degrees to its stop; it lets go and moves clear.
4. Meanwhile the right hand picks the can 110 mm up its body, fingers tilted 60 degrees from straight down, jaws across the can, and lifts it over the ring's height.
5. It centres the can's base over the ring by feedback on the can's observed pose, stands it straight if it leans, and lowers it straight through at 20 mm/s until its base is on the tray. Its fingers stay above the ring.
6. The right hand lets go, rises 10 cm and backs off.

The demo reads the can's pose, the holder's joint angles and the arms' joints; it reads no contact force and nothing from the scorer. Nothing in the scene is random: `object_jitter=0`.

## The scene and what is simplified

Positions are (x forward from the arm bases, y left) in cm from the midpoint between the bases; heights above the bench.

| Part | In the scene | Published | Source |
|---|---|---|---|
| Holder plate | 98 x 95 x 4 mm | B 3-7/8 in, A 3-3/4 in | maker's drawing of a marine fold-up holder (A and B read as height and width from photos) |
| Ring inside | 70 mm | holds drinks up to 2-3/4 in (69.9 mm) | maker's listing |
| Ring band, arms | 80 mm outside, 8 mm tall; two side arms to pivots at the plate's top corners | not published | product photo, estimate |
| Tray | 72 deep x 84 wide, 3 mm floor, 7 mm lips on front and sides; hinged at the plate's bottom | not published | product photo, estimate |
| Ring top over tray floor | 84 mm | not published | product photo, estimate |
| Projection open | ring front 100 mm from the plate | D 3-7/16 in (87 mm) | maker's drawing (D read as open projection) |
| Depth stowed | 33 mm (the stowed ring stands 10 mm off the plate) | E 15/16 in (24 mm) | maker's drawing |
| Can | 66 mm x 122.2 mm, 370 g | 12 oz / 355 ml standard 66 x 122.2 mm; 355 g drink + about 15 g can | Crown; Gamut Packaging |
| Seat-back panel section | 45 x 40 x 5 cm | not published | estimate |

- **Holder:** a fold-up drink holder (the kind sold for boats and RVs; its plate, ring and tray are modelled to one such unit's photo and drawing), black ABS, screwed to the panel's face (plate face at x = 35.6). The ring pivots along y at (33.8, 0), 16 cm above the bench; open, its centre is at (29.6, 0) with its top face 16 cm up. The tray hinges along y at (33.0, 0), 7.6 cm up; open, its floor's top is 7.6 cm up. Each part is one link with hard stops at stowed (joint 0) and open (90 degrees). No spring or detent; joint damping 0.05 N m s/rad holds each where it is put (estimate). Their weight (about 20 g each) is left out, so neither falls open by itself; the seller's "store in up position" says the stowed holder stays up. The two links collide with each other, so the stowed tray blocks the ring until it is folded down (`ringfirst` below); they do not collide with the plate they are jointed to, and the joint stops keep them in range. The stowed ring stands 10 mm off the plate, 9 mm more than the real one, so that a fingertip fits behind its sides; the ring's size and the can's 2 mm play are unchanged.
- **Panel:** a rigid board, face at x = 36.0 cm, centred on y = 0, standing on the bench. A real seat-back holder sits about 60 to 70 cm above the floor; the panel is lowered to bench height for the arms' reach, at true scale.
- **Can:** one rigid body, the full can's mass (no fluid), standing on a 10 cm square mat at (22, -17).

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. The tiers are read from the state at the end; only the void conditions latch.

| What happened | Score |
|---|---|
| The tray and the ring each within 10 degrees of fully open | 0.33 |
| Also the can inside the ring: its axis within 10 degrees of vertical and of the ring's axis, crossing the ring's top face within 3 mm of its centre (2 mm play), its base 3 mm or more below that face | 0.67 |
| Also the can's base within 2 mm of the tray floor, every part of both grippers 50 mm or more from the can and from the holder, the can still, for 2 s running | 1.0 |
| Void: once off its mat, the can tilts past 30 degrees, or moves down faster than 0.3 m/s, held or falling (a free fall of about 5 mm; read after every physics step; the speed check is simulation only, a real trial applies the tilt check) | 0 |

The hands-clear test reads a lattice of points 10 mm apart over each whole gripper (fingers, pads, palm, housing and the wrist link 12 cm back), plus the fingertip points, against the can's exact envelope and the holder's ring band and arms, tray, plate and lugs.

A real trial reads the same with a phone level app (the can's tilt, and the 30 degree void) and a phone film from the side: both parts against their stops, the can's base on the tray floor, nothing touching the can for 2 s. It also voids if the fingers open with the can top more than 5 mm above its seated height (38 mm over the ring's top face), read against a tape mark on the board in the side film.

## Results

| Run | What it does | Score (GPU) |
|---|---|---|
| `full` | The whole task, both arms (RTX 3090 Ti, filmed) | 1.0 at step 780 (39.0 s) |
| `onearm` | The right arm alone opens the tray and the ring, then fetches and sets the can | 1.0 at step 872 |
| `ringonly` | The tray and the ring opened; nothing else | 0.33 |
| `ringfirst` | The ring swung out with the tray still stowed over it | 0.0 (the stowed tray blocks the ring at 7 degrees) |
| `hold` | As `full`, but the right hand keeps hold of the can on the tray | 0.67 (hands not clear) |
| `beside` | The can stood on the bench by the panel instead | 0.33 (holder open; can not in the ring) |
| `drop` | The can let go 35 mm over the ring | 0.0 (moves down faster than 0.3 m/s: void) |
| `none` | The arms wait | 0.0 |
| Dream-1 | The command above, the model driving both arms (seed 1, 20 rows per request) | 0.0 (hit the 1,200-step limit) |

## Runtime and seeds

The full demo takes 39.0 s of simulated time (780 of the 1,200-step limit). There is no randomness at `object_jitter=0`.

## A real trial

Screw a fold-up drink holder of this type (about 10 USD, sold by marine and RV suppliers; plate 98 x 95 mm, ring 70 mm inside) to a board standing upright on the bench, its ring pivots 16 cm above the bench, and stand a full 12 oz can on a mat 22 cm out and 17 cm to the right. Stow the holder: ring down flat, tray folded up over it. Reset: lift the can out, put it back on its mat, fold the ring down and the tray up; about 30 s.

## Real items

- **12 oz / 355 ml standard aluminium can**, 66 x 122.2 mm: https://crowncork.com/beverage-packaging/products/beverage-cans/12oz-355ml-standard .
- The holder is a generic fold-up drink holder with a plate, a ring on two side arms and a hinged tray; any unit of that plate, ring and tray size serves.

## Assets

Every mesh is made in code in `task.py` (boxes, prisms and lathed profiles); no third-party meshes, textures or materials. The robot and the station are the harness's own (`yam-isaac-lab-arena`).

## Licence

MIT
