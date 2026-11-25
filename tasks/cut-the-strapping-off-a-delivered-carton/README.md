# Cut the straps off a delivered carton with a safety cutter

A delivered carton (12 x 9 x 3 in, 2.5 kg) stands on two boards on the table, bound by two black polypropylene
straps round its girth. A concealed-blade safety cutter waits on the table to its right. The instruction is "Cut both straps on this carton with the safety cutter, holding each strap down while you cut it. Do not mark the carton. Then put the cutter down on the table."

Everything is at full size and every part is a stock item (see "Getting the real parts").

## What is physically distinct here

- **A toe that has to get under a strap drawn tight against board.** The straps are under about 48 N and lie flat on
  the carton's top. The cutter's blade sits across a channel 4 mm high between its toe and its upper jaw, out of reach
  of anything that is not inside the channel. The only way to the blade is to lay the toe's tip flat on the carton
  beyond the strap and draw it back under the strap, so that the strap rides up the toe into the channel. A toe held a
  millimetre high goes over the strap and cuts nothing (this happened in the first probe runs).
- **A cut made while the other hand bears on the same strap.** A strapping maker's safety booklet gives the method
  for a single-handle cutter: "Stand to one side of the strap being cut", "Hold the strap firmly against the package -
  above the cutter", "Then, with a firm grip, safely cut the strap." The second hand has to be on the strap, within
  12 cm of the cutter, from before the strap parts until after. Cutting with the strap not held voids the trial.
- **A guard that must not be dug into the carton.** Getting under the strap needs the toe on the board, but pressing
  hard leaves a crease in the carton's top. The sole is laid down with a few newtons; 15 N held for a tenth of a second
  voids the trial.
- **A draw with the right amount of pull.** The strap parts once it presses on the blade with about 5 N. The arm has
  to keep drawing against that without dragging the 2.5 kg carton off its boards.

## Why it matters

Opening delivered cartons is everyday receiving work. Stockers and order fillers (O*NET 53-7065.00, 2,804,300 employed
in 2025) "Receive, unload, open, unpack, or issue sales floor merchandise." and "Pack and unpack items to be stocked on
shelves in stockrooms, warehouses, or storage yards." Cartons strapped for shipping have to have their straps cut
before anything else can be done with them, and a concealed-blade cutter is made for the job: its maker lists
"Plastic strapping band" among what it cuts, with a blade that "is hidden from people and goods".

## Files

| File | What it is |
|---|---|
| `task.py` | the scene, the instruction, the scoring rule and the scripted demo |
| `strapkit.py` | the carton, the two straps (24 links each), the cutter, the cut rule and the contact readouts |
| `test_scoring.py` | 24 offline checks of the strap geometry, the cutter's shape, the cut rule, the scoring rule and the demo plan |
| `task.json` | the task's details for the board |

## Run a trial

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Cut both straps on this carton with the safety cutter, holding each strap down while you cut it. Do not mark the carton. Then put the cutter down on the table." \
      --scorer task_score --max-steps 1500

Layout B: set `STRAP_LOT=B` in the environment (carton, boards and straps 12 mm nearer the robot).

## Scripted demo

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
      -E control_hz=20 --scorer task_score --max-steps 1500 --instruction "..."      # plays task.demo()
    STRAP_LOT=B ...        # layout B
    DEMO_STILL=1 ...       # control: nothing moves
    DEMO_STRAPS=0 ...      # control: the cutter is picked up and put back
    DEMO_STRAPS=1 ...      # control: one strap only
    DEMO_NO_HOLD=1 ...     # control: both straps cut with the left arm still
    DEMO_KEEP=1 ...        # control: both straps cut, cutter never put down
    DEMO_SOLE_MM=80 ...    # control: the sole pressed hard on the carton

What the demo does:

1. **Take the cutter.** The right hand, leaning 46 degrees forward, comes down astride the cutter's handle over the
   blade (fingers on its two flat faces, 13 mm apart; sent shut, 32.5 N a finger at the gripper's 5 N/mm) and lifts it.
2. **Far strap.** The hand tips back to a 30 degree lean, which levels the cutter's sole, and sets the sole on the
   carton with the toe's tip 2 cm beyond the far strap, sent 2 mm into the carton's top (2 to 5 N). The left hand,
   leaning 35 degrees out to its own side, presses the strap down with its closed fingertips 9 cm to the left of the
   cutter (sent 6 mm into the strap: about 2 to 3 N).
3. **Draw.** The right hand draws the cutter toward the robot at about 3 cm/s. The toe goes under the strap, the strap
   rides up into the channel and meets the blade; the hand keeps drawing until the strap parts (5.7 N on the blade
   in the recorded run). It lifts clear; the left hand lets go.
4. **Near strap.** The same again.
5. **Put the cutter down.** The right hand sets the cutter back between its guides and lets go; both hands move away.

The demo is open loop: fixed waypoints from the documented layout, turned into joint targets by the stock inverse
kinematics. It reads nothing from the simulator.

## Scoring

Scored at the end of the trial.

| Score | What is true |
|---|---|
| 0 (whole trial) | A strap was cut while the other gripper was not holding it down: not touching that strap on the carton's top within 12 cm of the cut from 0.1 s before it parted until it parted. Or the cutter was pressed into the carton: a crease, dent or cut in the carton's top. Or the carton ends more than 3 cm from where it started, or tipped more than 10 degrees. |
| 1.0 | Both straps cut through; the cutter put down on the table (its toe within 3 cm of the table top) and at rest; both grippers at least 3 cm from the carton and from the cutter; the carton at rest. |
| 0.75 | Both straps cut, but the cutter not put down, a gripper within 3 cm, or the carton still moving. |
| 0.5 | One strap cut. |
| 0.25 | The cutter was lifted 5 cm from where it waits. |
| 0 | Anything else. |

In a real trial all of this is read from the video, a ruler and a look at the carton: whether each strap is cut;
whether the second gripper's fingers are on the strap, within 12 cm of the cutter (ruler on the frame), in the frames
before and at the moment it parts; whether the carton's top is creased; where carton and cutter end.

The simulator applies the same tests with two force floors, so that a hover does not count as a hold and a light
touch does not count as a dig: the holding gripper's contact force on the strap has to be at least 1 N at every
physics step of the 0.1 s before the strap parts (measured 1.8 to 2.5 N in the demo), and the carton counts as
marked when the cutter's contact force on it is 15 N or more for 0.1 s (the demo's sustained press is 1 to 5 N; the
brief knock as the sole lands reached 16.5 N in the recorded run and lasted under 0.1 s). The 15 N figure is an estimate, not a measured
crush strength. Not scored: where along its top run a strap is cut, which hand does what, and where the cut straps
end up.

## Verification

Scores are read from each run's `result.json` (the scorer's report at the end of the run).

| Run | Score | What happened |
|---|---|---|
| Scripted demo, layout A | 1.0 | full score at step 612 of 1,500; far strap parted at 13.5 s with 5.7 N on the blade; near strap parted at 21.8 s with 5.8 N on the blade; other gripper on the far strap with at least 1.84 N, 9.0 cm from the cut; other gripper on the near strap with at least 2.33 N, 9.0 cm from the cut |
| The same script, layout B (carton, boards and straps 12 mm nearer) | 1.0 | full score at step 612; far strap parted at 13.5 s with 5.6 N on the blade; near strap parted at 21.8 s with 5.9 N on the blade; other gripper on the far strap with at least 1.81 N, 9.0 cm from the cut; other gripper on the near strap with at least 2.53 N, 9.0 cm from the cut |
| Control: nothing moves | 0.0 | carton strapped, cutter between its guides |
| Control: the cutter is picked up and put back | 0.25 | no strap cut |
| Control: the far strap only | 0.5 | far strap parted at 13.5 s with 5.7 N on the blade; other gripper on the far strap with at least 1.84 N, 9.0 cm from the cut |
| Control: both straps cut, the cutter kept in the hand | 0.75 | far strap parted at 13.5 s with 5.7 N on the blade; near strap parted at 21.8 s with 5.8 N on the blade; other gripper on the far strap with at least 1.84 N, 9.0 cm from the cut; other gripper on the near strap with at least 2.33 N, 9.0 cm from the cut; cutter never put down |
| Control: both straps cut with the left arm still | 0.0 | far strap parted at 13.1 s with 5.7 N on the blade; near strap parted at 20.8 s with 5.9 N on the blade; other gripper not touching the far strap; other gripper not touching the near strap |
| Control: the sole sent 80 mm into the carton's top | 0.0 | the cutter pressed on the carton with up to 25.7 N, 15 N or more on 39 control steps; carton counted as marked from step 204; the toe did not get under the strap and nothing was cut |
| Dream-1, 1,500 steps | 0.0 | ran the full 1,500 steps; the right gripper went to the cutter once and came away without it, then both arms stayed low near their bases; the cutter was never lifted and no strap was touched |

`test_scoring.py` (24 checks, no simulator) covers: each strap is a closed loop half a strap's thickness outside
the carton; the sprung slide is under the carton; the straps clear the boards; the cutter's envelope and its 4 mm
channel; the cut needs 5 N for a full 0.1 s; a short knock on the carton does not mark it and a sustained one does;
the hold is the least press over the 0.1 s; every scoring tier and every void; the demo plan's timing, grip force and
path.

## Runtime and seeds

The scripted demo reaches full score after 612 steps (31 s of robot time); the run limit is 1,500 steps
(75 seconds at 20 Hz). The demo run took about 8 minutes of wall-clock time on one L40S GPU with a second
simulation running beside it. Nothing is random: `object_jitter=0`, no seeds.

## What is modelled and what is a rule

- **Straps.** Each strap is a closed loop of 24 rigid links (12.7 mm wide, 1 mm thick, 4 g each) joined by ball
  joints, round the carton's 9 x 3 in girth. One joint under the carton is a slide with a spring (5 N/mm) stretched
  at the start, which puts the loop in tension: 48 N measured at rest. A strapping machine's published range is
  "Adjustable from 5-50 Kgs"; 48 N is its lowest setting. The strap is held on by tension and friction (0.3) alone.
- **The cut is a rule on a measured force.** A rigid body cannot be cut. The blade is its own small body across the
  channel; every physics step the kit reads the contact force between it and each strap link, and when one link has
  pressed on it with 5 N or more for 0.1 s the nearest strap joint is switched off. The 5 N is an estimate; no
  figure for the cutting force of this cutter on this strap was found.
- **The strap does not spring back when cut, so the hazard itself is not simulated.** A real tensioned strap recoils
  when cut. Here the cut ends moved 0.4 to 2.4 cm in a probe run and the rest of the strap stayed taut over the carton's
  edges, because the rigid links catch there. The hold is therefore scored as the safe practice the booklet states (a gripper on the
  strap beside the cut when it parts), not from what the strap does afterwards.
- **Cutter.** Modelled from the maker's published envelope (158 x 13 x 56 mm), weight (48 g) and "Max cutting depth
  4 mm": a handle, an upper jaw, a sole and a wedge-shaped toe, as boxes and one convex wedge, 155 x 13 x 52 mm. The
  profile inside that envelope (toe length 22 mm, where the blade sits) is an estimate from the maker's pictures.
- **Carton.** One rigid box of 2.5 kg; it does not dent. Friction on the boards 0.4.

## Getting the real parts

| Part | Product | Where the size comes from | Price as shown |
|---|---|---|---|
| Carton | Lavex 12 x 9 x 3 in kraft corrugated RSC shipping box, C flute, 32 ECT (WebstaurantStore item 442BRK1293) | listing | $21.49 a case of 25 ($0.86 each) |
| Strapping | PAC Strapping Products 3,600 ft x 1/2 in black polypropylene strapping, 0.026 in thick, 500 lb break strength, hand grade (item 442SPP3600B) | listing | $47.49 a coil |
| Buckles | PAC Strapping Products 1/2 in plastic buckles | listing (name only) | $52.99 a case of 1,000 |
| Cutter | Martor SECUMAX 320 (No. 32000110) | maker's page | not shown on the maker's page; about $10 (estimate) |
| Boards | one 1x4 board (19 x 89 mm actual): two 30 cm lengths under the carton, two 19 cm lengths as the cutter's guides | lumber size table | about $5 (estimate) |

Contents to bring the closed carton to 2.5 kg (reams of paper or bagged sand; estimate of a small delivered case).
Strap each carton round its 9 x 3 in girth, 6.5 cm either side of its middle, with a hand tensioner and buckle or a
strapping machine at its lowest setting (about 5 kg). Screw the two guide offcuts to the table 15 mm apart.

## What the simulation simplifies

- The strap is thicker (1 mm against 0.66 mm) and heavier (96 g against about 4 g) than the real one, for the
  solver's sake; its links are 25 mm long, so it cannot curl.
- No recoil of a cut strap (above). No strap joint (buckle or weld) is modelled.
- The cut is a rule; drawing the blade across the strap's full width is not modelled, only the press.
- The carton is rigid: no denting, so "marked" is a force rule.
- The strap's loose links keep trembling after a cut, so the straps are not checked for rest.
- Disposal of the cut straps (the booklet: "Always dispose used strapping in a container designated for disposal or
  recycling") is left out: a limp 60 cm strap is beyond these grippers.
- Not tried on a real carton.

## Nearest tasks

- "Cut a cable tie off a cable bundle without nicking the cables" (accepted) closes flush cutters on a tie that sits
  slack on a bundle. Here the blade is fixed inside a guard and the work is getting a toe under a strap in tension,
  with the second hand on the strap.
- "Slit a taped carton's 40 cm seam with a box cutter, short of the product 6 mm below, then draw the blade back in"
  (accepted) runs an exposed blade along a seam at a set depth. Here nothing is slit: a strap is hooked and parted,
  and the tool's own guard keeps the blade off the carton.
- "Hold a cord taut and cut it with a knife, away from the holding hand" (accepted) has the second hand make the
  tension; here the tension is already in the strap and the second hand bears on it.
- BEHAVIOR-1K has unpacking activities (`unpacking_moving_van`, `unpacking_suitcase` and others); none cuts strapping.
  No task in LIBERO, RoboCasa, SimplerEnv, DataCenterBench or RoboDojo cuts a strap, as far as their task lists show.

## Sources

- O*NET OnLine, 53-7065.00 Stockers and Order Fillers: "Employment (2025) 2,804,300 employees"; tasks quoted above.
  https://www.onetonline.org/link/details/53-7065.00
- Encore Packaging, "Safety Instructions for Hand Tools, Seals & Strapping", "How to cut tensioned strap", single
  handle strap cutter: quotes above.
  https://www.zoro.com/static/cms/enhanced_pdf/Encore%20Packaging%20LLC_EncorexxPackagingxxHandxxToolxxSafetyxxBookxxe76f6d.pdf
- Martor, SECUMAX 320 No. 32000110: "Dimensions (L × W × H) 158 x 13 x 56 mm", "Weight 48 g", "Max cutting depth
  4 mm", cutting materials include "Plastic strapping band", "The SECUMAX 320 blade is not an open one, but is hidden
  from people and goods."
  https://www.martor.com/en/products/product-finder/product/secumax-320-32000110.html
- Signode, MST semi-automatic strapping machine: "Tension Range Adjustable from 5-50 Kgs"; strap sizes "9mm, 12mm &
  15mm". https://www.signode.com/en-au/productslist/mst-semi-automatic-strapping-machine/
- WebstaurantStore, PAC Strapping Products 3,600' x 1/2" black polypropylene strapping coil, item 442SPP3600B: "Width
  1/2 Inch", "Thickness 0.026 Inch", "Break Strength 500 lb.", "Usage Hand Grade".
  https://www.webstaurantstore.com/pac-strapping-products-3-600-x-1-2-black-polypropylene-strapping-coil-with-16-x-3-core/442SPP3600B.html
- WebstaurantStore, Lavex 12" x 9" x 3" kraft corrugated RSC shipping box, item 442BRK1293: "Flute Type C", "32 ECT rating
  ensures box shape will be retained".
  https://www.webstaurantstore.com/lavex-packaging-12-x-9-x-3-kraft-corrugated-rsc-shipping-box-bundle/442BRK1293.html
- Wikipedia, "Lumber", North American softwood sizes: a nominal 1 x 4 is 3/4 x 3 1/2 in, 19 x 89 mm.
  https://en.wikipedia.org/wiki/Lumber
- Wikipedia, "Strapping": "Polypropylene strap (oriented or tensilized) is an economical material designed for light
  to medium duty unitizing, palletizing, and bundling." https://en.wikipedia.org/wiki/Strapping

## Third-party assets

None. Everything is built from boxes and one convex wedge in `strapkit.py` and `task.py`. `strapkit.py` follows the
structure of the kit in this repository's accepted task "Hold a cord taut and cut it with a knife, away from the
holding hand".
