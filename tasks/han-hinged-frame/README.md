# Load four Han-Modular modules into a HARTING hinged frame its spring holds open, then squeeze it shut until it clicks

A HARTING Han hinged frame plus stands open on a bench assembly stand, held open by its own spring. Four assembled Han E connector modules, crimp contacts fitted and a short tail of their wires hanging under each, stand in a rack beside it. The robot sets each module on the open frame's support edges at positions A to D, then squeezes the frame shut round all four until it clicks, every module still seated, and moves its hands clear.

**Ability tested:** setting small loose parts on narrow edges at a close pitch, then closing a sprung frame round all of them at once. Each wired module (8 g, with a 3 g tail) rests on the frame's two support edges, 2.5 mm lips along the walls' inner feet, with only about 1.5 mm of overlap while the frame is open; pushed about 1.5 mm sideways it drops through. The finger pads are wider than the 16 mm pitch, so a hand setting one module must pass over its neighbours' tops: a lower grip drags them along their edges. Each module's 30 mm wire tail hangs under it and goes in first, down between the open walls, so the module is carried in high over its seated neighbours and lowered straight onto its edges; a tail dragged against a wall tips the module over. Then one squeeze must take the frame past its spring to the closed stop: the chamfered windows centre each module's lugs as the walls close, and a module left high stops the frame short of the click. The positions are given and the modules are alike, so nothing is counted or chosen; a picture shows where the modules go, not how to set and keep them.

**Why it matters:** electricians make up industrial connectors when they wire machines and control panels (O*NET 47-2111.00 Electricians, Core task: "Assemble, install, test, or maintain electrical or electronic wiring, equipment, appliances, apparatus, or fixtures, using hand tools or power tools."). HARTING's own instructions for Han-Modular modules: "install the completely assembled ... module in the Han-Modular hinged frame. Make sure that the module's notches fit into the recesses of the hinged frame" and "close it by gently pressing on the outer sides of the frame".

**Hands:** in the demo the right hand sets the modules and then parks; the left hand closes the frame. The score does not check which hand does what. The frame is fixed on its stand, so no hand needs to steady it.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Load the four modules into the hinged frame and close it until it clicks, then move both hands clear."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                 # full score, with a video
python demokit.py task.py OUT --no-record --mode noclose  # cheat check: all four set, never squeezed
python demokit.py task.py OUT --no-record --mode one      # cheat check: D only
python demokit.py task.py OUT --no-record --mode three    # cheat check: D, C and B, then squeezed
python demokit.py task.py OUT --no-record --mode low      # cheat check: each carried in low, as if it had no tail
```

Run each mode in its own session: the demo's reset between modes in one session does not put the wired modules back exactly, and a later mode then starts from a slightly different state.

The demo is a scripted reference. It reads the modules' poses, which the cameras see; a policy gets only the cameras. The right hand takes each module from the rack by its two lugged faces, a 3.5 mm pinch at the top above the lugs, lifts its tail clear of the rails, carries it in high over the modules already seated, then lowers it straight down over 2.4 s and lets go 4.5 mm over its seat on the support edges: D first, then C, B and A, so the hand never passes low over a seated module. It then parks about 25 cm away. The left hand comes straight down over the frame's middle with its fingers outside the two walls, 10 mm up them, and closes at 20 N until the frame clicks; the walls' chamfered windows centre the modules' lugs as they close. It lets go and moves away.

## Building it for a real trial

- **Frame:** a HARTING Han hinged frame plus, size 16B (09 14 016 0361), on a small bench assembly stand that holds it by its two end flanges (two steel blocks 7 x 30 x 35 mm on a 110 x 50 mm plate), its foot 35 mm above the bench so the modules' tails hang free under it, as through a panel's cut-out, at 30 cm in front of the arms' bases.
- **Modules:** four Han E crimp modules (09 14 006 3001), each fully assembled: six Han E crimp contacts crimped onto 0.75 mm2 single wires and pushed in until they latch, the wires cut 30 mm below the module's underside and the six held together as one tail.
- **Rack:** two plastic rails 110 x 7 x 45 mm, 18 mm apart centre to centre, at 20 to 28 cm forward and 16 cm to the right, with 8 mm ribs on top 25 mm apart; each module stands across both rails, its tail hanging between them.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| The frame's opening angle | 15 degrees each wall | **Not measured; approximated** (not published). Protractor on a frame resting open |
| The frame's spring | 0.3 N m/rad each wall (about 8 N at the wall's middle, 4 N at its top, to hold one shut) | **Not measured; approximated.** Push-pull gauge at a wall's middle, frame empty |
| The module's outline | 14.6 x 25 x 34 mm, 2 mm lugs | **Not verified; 14.6 mm modelled** (a distributor's listing gives 15.5 x 25 x 34 mm; the model is 0.9 mm narrower, leaving 1.4 mm between neighbours). Caliper on a module, or HARTING's CAD download |
| The support edges | 2.5 mm lips along the walls' inner feet | **Not measured** (the frame's drawing shows the walls, not the lips). Caliper |
| The wire tail | 30 mm, 3 g with the contacts; bends at 0.0005 N m/rad, damped at 0.005 N m s/rad | **Not measured; approximated.** Weigh an assembled module less the data sheet's 8 g; hold the module on its side and read the tail's droop |

## Safety for a real trial

The wires are cut tails connected to nothing, and the frame is not wired: no voltage anywhere. Keep an emergency stop in reach.

## Scoring

See `scoring` in `task.json`, and `moment`, `seated` and `Trial` in `task.py`. `python3 check_score.py` runs that scorer on nineteen hand-built states (the full result, a hand left near, the partial results including one, two, three and four modules seated on the open frame, a foot 1.4 and 2 mm over the open edges, a module still pinched, two seated in a shut frame, three shut with the fourth out of line or fallen through, and each failure) with no simulator. In a real trial, by eye, caliper and video:

- **Seated, frame open:** a module let go (no fingertip within 5 mm of it) on both support edges at its position, within 1.5 mm along and across, its foot no more than 1.5 mm over the edges, and within 5 degrees of upright (engineer's square). Opened 15 degrees, the edges' inner corners stand about 1 mm higher than when shut.
- **Seated, frame shut:** within 1 mm along, across and in height and 3 degrees, its lugs in their windows.
- **Clicked:** the latch heard, and the walls stay shut when let go.
- **Empty position:** no module within 8 mm along and 6 mm across of it, at any height (in, on or under the frame).
- **1.0:** the frame closed and clicked with all four modules seated shut, every fingertip of both hands at least 3 cm from the frame and the modules, held 1 s.
- **0.6:** closed and clicked with all four seated but a hand still within 3 cm, or with three seated, the fourth position empty and no module fallen through the frame, or all four seated with the hands clear for under 1 s.
- **0.075 per module (up to 0.3):** each module seated on the open frame, the frame not closed: 0.075 for one, 0.15 for two, 0.225 for three, 0.3 for all four. Closed and clicked otherwise: 0.075 for each module seated shut, the same.
- **0** if the frame was forced shut on a module more than 2 mm high (side-on video at the click), or a module fell off the table at any moment. Anything else scores 0.

In the simulator the click is code: once both walls are within 0.5 degrees of shut, they lock, as the real frame's latch does. Hands clear is measured from all 24 fingertip points of both hands to both walls along their length and to each module's faces.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | All four wired modules set on the open frame's support edges, D first: each carried in high over the seated ones, lowered with its tail between the walls and let go 4.5 mm over its seat, landing within 0.7 mm of it; lowering A pushed B and C 1 to 2 mm along, still seated; the frame squeezed shut at 20 N and clicked at step 1126, its windows centring all four within 0.6 mm along and 0.2 mm across; let go, both hands clear; held 1 s | 1.0, at step 1166 (58.3 s) |
| `noclose` | All four set on the open frame as in the film, then the hands park; the frame is never squeezed | 0.3 (all four seated on the open frame, within 1.3 mm along and 1 mm across) |
| `one` | D set on the open frame; the other three left on the rack | 0.075 |
| `three` | D, C and B set, A left on the rack; the frame squeezed shut and clicked with the three seated, the fourth position empty | 0.6 |
| `low` | Each module carried in at 20 mm over its seat, as a module with no tail could be: each tail catches the open walls, D, C and B land 6 to 17 mm off their seats and tip over, A lands 10 mm high; the squeeze stalls with the walls 8 to 10 degrees shut | 0.0 |
| Dream-1 | The arms move about over the table; no module is lifted from the rack (two shift 1 to 2 mm along it), and the frame stays open and empty to the 2,400-step limit | 0 |

No seed is needed: nothing in the scene is random. Runtime per trial, on one GPU: the demo's 1,166 steps take about 6 minutes while recording three cameras; a run to the 2,400-step limit takes about 12 minutes.

## Assets

All meshes are made by the task's own code from plain shapes at the sizes above. No third-party assets, no textures.

The task statement quoted in `task.json` is from the O*NET database (O*NET OnLine, task statements for 47-2111.00, https://www.onetonline.org/link/summary/47-2111.00) by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence. O*NET is a trademark of USDOL/ETA. The assembly steps are quoted from HARTING's instructions for the Han Pneumatic double module (manualslib.de, manual 904709, p. 2); the frame's sizes are from HARTING's drawing of the Han hinged frame plus 16B and the module's mass from HARTING's data sheet for 09 14 006 3001 (https://docs.rs-online.com/e782/A700000015178126.pdf). HARTING, Han and Han-Modular are trademarks of HARTING, named for identification only.

## Licence

MIT
