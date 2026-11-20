# Ticket a shirt through its sewn care label only: lift the label, pierce it with a tagging gun's needle, fire a fastener

A child's cotton T-shirt lies flat on the table, front up, and its care label, sewn into the side seam and drawn out, lies on the front panel. A tagging gun lies on its side beside it with a price ticket on its needle and a clip of fasteners loaded. The robot lifts the label clear of the shirt with one hand, pushes the gun's needle through the label alone with the other until it pierces the cloth, squeezes the trigger to fire one fastener, draws the needle back, and lays the label down with the ticket hanging from it.

**Where it comes from.** Stores ticket clothes with a tagging gun (an Avery Dennison Mark III standard gun is a common kind): the trigger drives a plunger that pushes a plastic T-end fastener down the gun's hollow needle. Clothes are ticketed through the care label or a seam so that the garment's fabric is not holed. Retail salespersons have the core task "Ticket, arrange, and display merchandise to promote sales", and stockers and order fillers "Mark stock items, using identification tags, stamps, electric marking tools, or other labeling equipment" (O*NET 41-2031.00 and 53-7065.00, USDOL/ETA, CC BY 4.0; O*NET is a trademark of USDOL/ETA). How many do this act: clothing and clothing-accessories retailers alone employ 492,610 retail salespersons and 27,940 stockers and order fillers, about 521,000 (BLS OEWS, May 2023, NAICS 458100: https://www.bls.gov/oes/2023/may/naics4_458100.htm); department stores and general-merchandise retailers, which also ticket clothes, are not counted. Retail salespersons number 3,684,740 in all (https://www.bls.gov/oes/2023/May/oes412031.htm); the context counts only the clothing retailers' share. Avery Dennison and Mark III are named for identification only; this task is not affiliated with them.

**What this task tests.** The label is soft and sewn to the shirt along one 30 mm edge: lifting it bends it and pulls the shirt's seam up with it, and pushing on it swings it about its seam. One hand holds it clear; the other aims a 1.9 mm needle through it, pushes until the label gives (4 N held over 1 mm of travel), squeezes the trigger only once the needle is through, and draws the needle back along its line. The needle must never touch the shirt's body, and must not cut sideways out of its hole.

## Earlier feedback, and what changed in the task

| Review | Point | What changed in the task |
|---|---|---|
| First review | "Shirt should be modeled more accurately, currently oversimplified compared to real-world context." | The shirt is two sewn layers of native surface-FEM cloth (front and back, 0.6 mm, 180 g/m2), 2,138 nodes, joined along the shoulders, sleeve undersides and side seams, with the neck, cuffs and hem open (`_build_cloth`, `_cloth_mesh`). |
| Second review | "The noncolliding needle passes through a rigid, fixed-hinge label without the resistance or fabric movement involved in piercing a sewn care label." | The needle collides: a 1.9 mm collider on a stiff sprung seat (5 kN/m) in the nose, so its push on the label is read from the seat's give. It pierces only after at least 4 N is held while its tip advances 1 mm into the label's face (`press_step`); then the collider stays, filtered against that segment only, and the shaft passes the hole it made. Below 4 N, or with no advance, the needle stops on the label's face. |
| Second review | "the label is fixed independently of the shirt. Model the label's fabric movement and its attachment to the shirt" | The label is a chain of three thin segments (15, 15 and 30 mm) on sprung hinges, free in space, its root a stitch strip held to the shirt's front and back layers by PhysX's deformable attachment (`attach_label`). It bends at its seam and along itself, curls at its free edge, rides on the cloth, and lifts the shirt's seam about 10 mm when it is lifted in the demo. |
| Second review | "require needle insertion with approximately realistic resistance before firing" | Firing counts only while the needle is through its own hole: on the label's far side, crossing it within 7 mm of where it pierced, having got there through the hole (`hole_step`). A squeeze before the pierce fires nothing into the label, and only the first squeeze counts. |
| Second review | "provide a full-score demo of the revised task" | `python demokit.py task.py OUT --mode full` scores 1.0 (Result, below). |
| Second review | "The earlier rejection's feedback was not included" | It is quoted in the first row. |

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Use the tagging gun to fasten the price ticket through this shirt's care label only, not the shirt, then draw the needle out, lay the label back down and let go."
```

A trial runs up to 2,400 steps (120 s of simulated time at 20 Hz). One fixed layout; no seed changes the scene.

## Scripted demo

```bash
python demokit.py task.py OUT --mode full        # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode shirt    # a failure check (modes in the table below)
python check_score.py                            # the scorer's rules on hand-built states, no simulator, and the size check
```

The demo is a scripted reference: it reads the label's and the gun's poses from the simulator, and a policy gets only the cameras. The left hand comes in from beside the label's curled free edge, its fingers tilted 60 degrees from straight down, pinches the label 12 mm in from that edge, swings it up about its seam to about 75 degrees and raises it 15 mm. The right hand takes the gun from the table by its grip at 20 N (under the trigger's 25 N preload, so the gun does not fire in the carry), leaning back 40 degrees so its forearm stays clear of the left's, and brings the needle in level, 5 cm short of the label, aimed 18 mm below the pinch. It pushes along the needle's line; the tip stops on the label's face while the push builds, then pierces (4.16 to 4.46 N held over 1.05 to 1.3 mm of travel, in the middle segment, about 17 mm over the shirt), and passes about 10 mm through. It squeezes, up to 35 N, until the trigger has travelled 2.5 mm and fired one fastener, eases off at once, draws the needle back and holds the gun up away from the shirt. The left hand lays the label down, opens its jaws 12 mm and draws them out off the label's edge. The ticket hangs from the label on the fastener's barb.

**Result** (scripted, GPU physics, 20 Hz): 1.0 in 556 steps (27.8 s of simulated time, last run on an NVIDIA GeForce RTX 5090 after the shaft collider and the 240 Hz shirt check were added; earlier runs of the same motion: 1.0 in 553 steps), filmed at 640 x 360 on an NVIDIA GeForce RTX 4070 Ti SUPER; the film's first frame is the start state, and no camera has a dark frame at its start. The demo command scored 1.0 three times on lab PCs (RTX 4070 Ti SUPER; 553, 555 and 553 steps) and once on an NVIDIA GeForce RTX 5090 (556 steps; a second 5090 run at 320 x 176 also 1.0, 556 steps). Four seeded runs with 1 cm object jitter scored 1.0 each (553 to 556 steps); the harness's jitter shifts free rigid bodies, and the gun and the label are mechanisms, so these four start alike. Across the three demo runs the push peaked at 4.16 to 4.46 N and pierced after 1.05 to 1.3 mm of travel, the label stood up to 81 to 82 degrees and lifted the shirt's seam 9 to 10 mm, the needle crossed the label 17 to 23 mm over the shirt and came no closer than 14.4 mm to the cloth, and its crossing stayed within 4.8 mm of its hole. The margin is thin: needle-hole drift at most 3.9 mm in the last full run (4.6 to 4.8 mm in earlier runs) against the 7 mm tear threshold (raised from 5 mm after one run tore at the 240 Hz check) (estimate: a 1.9 mm needle and the stretch of the weave round its hole; no measured source). After the pierce the shaft's collider stays, filtered against the pierced segment only, and nothing holds the label on it; the 7 mm hole rule, checked at every physics step, decides a tear. `success()` reads the score now (the state held for 1 s at full), not a latch; a trial ends when it first reads true.

| Failure check | What it does | Score (GPU physics) |
|---|---|---|
| `shirt` | the needle pushed, tilted 15 degrees down, into the shirt's body beside the label | 0.0: the needle touched the cloth (within 3 mm of its surface), a hole in the shirt; nothing fired into the label |
| `nopush` | the label lifted, the gun brought 5 cm short and fired there: a squeeze with no insertion | 0.0: the trigger fired with the needle 5 cm short; nothing in the label, and the first squeeze is spent |
| `light` | the needle brought to the label's face and stopped there: a touch | 0.0 (5090, 619 steps): the push reached 4.1 N at most but the tip never advanced 1 mm into the face (no pierce); the squeeze fired nothing into the label |
| `drag` | the needle pierces, then is dragged 15 mm sideways across the label before the squeeze | 0.0 (5090, 646 steps): pierced at 4.3 N; the hole's drift reached 4.5 mm (under the 7 mm tear threshold, so not torn), then the needle slid out of the label's face and the squeeze fired with the needle not through, so no fastener; the 0.0 comes from the fastener rule, not from a tear |
| `nofire` | the needle pierces and is drawn back unfired; the label laid down | 0.0: pierced at 4.31 N and drawn back unfired; the 0.5 state while through is gone at the end |
| `nolift` | the label left lying on the shirt, the needle pushed at it as in the demo | 0.0: the label lay flat and the needle touched the cloth, a hole in the shirt |
| `shirtlift` | the left hand pinches the shirt's body beside the label (or by its left side edge) and lifts it 6 cm | 0.0 on GPU, but not yet evidence: the pads did not grasp the flat cloth in any run so far (its highest node rose 0.4 to 3.3 mm in five runs, the last 2.6 mm, not the 30 mm that a grasp needs). The rule itself, the free edge read against the shirt beside it, is tested in `check_score.py` (a shirt lifted 40 mm with the label lying on it reads as not lifted) |
| `shirtpass` | the needle pierces the label low by its sewn edge, tilted 30 degrees, and is pushed on into the shirt behind it, then the squeeze and draw-back | 0.0 on GPU (5090, 610 steps): the needle pierced the label low by the seam and its tip came to 2.3 mm of the cloth (the 3 mm hole band), but the run is not clean evidence: the label tore off the needle (torn) and the right arm's IK error reached 126 mm late in the run, so the 0.0 is not shown to come from the shirt rule alone. The rule (a tip within 3 mm of the cloth, or the shaft crossing it, at any physics step, latched) is tested in `check_score.py` |
| `flick` | the needle pierces, then is yanked 25 mm sideways and back, 0.1 s each way, before the squeeze | 0.0 (5090, 631 steps): torn at the 240 Hz check (the yank crosses 7 mm from the hole between two control steps); the squeeze fired with the label torn |
| `padpush` | the needle pushed into the label right beside the left hand's pads (3 mm from the pinch), so that the 4 N could come from a pad | 0.0 (lab PC RTX 4070 Ti SUPER, 627 steps): the push against the pad reached 8.1 N but the pierce never counted (no advance into a label face with a finger within 1 cm of the tip), nothing fastened, the shirt untouched. Run as a single mode: in a second or third mode of one session the 5090 started the scene disturbed, so each failure check is run on its own |
| `shaftrest` | the full act, then the gun laid with its needle's shaft 1.3 mm over the shirt and its tip 6.5 mm over it | 1.0 (556 steps): only the tip counts as a hole |
| `undo` | the full act, then the needle pushed into the shirt | 1.0, success True, 553 steps: the trial ends when success first reads true, so nothing after the act is scored. Before the change the latch let a later hole leave success True with a score of 0 |
| (not run on GPU) seam fold | folding the label back past 130 degrees between two control steps | the left arm cannot fold the seam past 130 degrees: a 134-degree hand fold missed its IK by 85 mm and the seam joint peaked at 45 degrees, so there is no GPU route. The per-step check (substep, 240 Hz) and the trigger read are tested in `check_score.py` on a stub scene |
| `check_score.py` | the rules on hand-built states: the pierce (4.5 N over 1.5 mm pierces; 2 N, no advance, a let-off, or beside the label does not; a gun turning 4 degrees in the hand reads no false advance), the hole (through at the hole; dragged 8 mm sideways: torn; drawn back: out; re-entered through the hole: through; slid round to the far side off the hole: not through), contact with a hand-built patch of cloth (a needle 2 mm over it, pressed onto it, or through it between its nodes: contact; 4 mm or more over it: none), the let-go check, the seam tear and the trigger read per physics step on a stub scene, and the score table | every case as expected |

## Scoring

Terms. *Pierced*: the needle's push, read from its seat, held at 4 N or more while its tip advances 1 mm into a label segment's face, inside its outline, with no gripper finger point within 1 cm of the needle collider's tip (the push must come from the needle on the label, not from a pad). *Through*: after the pierce, the needle's tip on the label's far side and the needle crossing the label within 7 mm of its hole, having got there through the hole. *A hole in the shirt*: the needle's tip within 3 mm of the shirt's simulated cloth surface (its 0.95 mm radius plus the cloth's 2 mm contact offset), checked at every physics step (240 Hz) on the tip's whole path since the previous check (a point every mm), and on the shaft: the needle's shaft or the path its tip or base swept crossing a triangle of the cloth is a hole too, so a needle driven through the cloth between two checks is caught. The shaft resting on the cloth without crossing it is not a hole (in a real trial only an actual hole counts, the lamp check). If the cloth's triangles cannot be matched to its simulated nodes, the check falls back to the nodes alone, 1 cm apart, with the tip's distance to the nearest node less 7.1 mm (half a cell's diagonal), so a tip between nodes is not missed; it is stricter, and this is not the path the demo takes. *Torn*: the label folded at its sewn edge past 130 degrees (5 degrees short of its stitches' 135-degree stop), or the needle's crossing more than 7 mm from its hole while through (cut sideways), checked at every physics step (240 Hz), not only at the 20 Hz control steps. *Let go*: the label down under 30 degrees from the shirt and every fingertip of both hands 5 cm or more from the label's middle.

| Score | When (the state at the end, held 1 s) |
|---|---|
| 1.0 | one fastener fired while the needle was through the label, the needle drawn back (its tip 4 cm or more from the label's middle), the label let go |
| 0.5 | a fastener in the label, or the needle through it now, but not drawn back or not let go |
| 0.25 | the label's free edge 2 cm or more over the shirt beside it now (the highest cloth node within 2 cm of the edge, so lifting the shirt itself does not count), nothing in it |
| 0 | a hole in the shirt, a torn label, or the gun 5 cm or more below the tabletop, at any moment (these stay); else nothing above |

Only the first squeeze counts: a fastener fired before the needle is through is lost. A needle drawn back unfired leaves nothing in the label. Lifting, holding and aiming cost nothing by themselves. Possible scores: 0, 0.25, 0.5, 1.0.

On a real trial: a top camera, both wrist cameras and a side camera level with the shirt film the trial. The pierce is the gun's own result, the fastener through the label, checked by hand; a hole in the shirt is scored on the hand check afterwards, the shirt's body held up to a lamp (the side camera only shows where to look); a torn label shows as broken stitches at its seam or a slit from the hole, checked by hand; the let-go is read on the top camera.

## Sizes

| What | In the scene | Real | Source |
|---|---|---|---|
| Shirt body, hem to shoulder x across | 300 x 280 mm | 300 x 280 mm | a child's tee, about age 4 to 6 (estimate from makers' size charts) |
| Jersey, per layer | 180 g/m2, 0.6 mm | 150 to 200 g/m2, 0.5 to 0.7 mm | single-knit cotton jersey (estimate) |
| Care label | 30 x 60 mm, 2 g, sewn by its 30 mm edge | 25 to 40 x 50 to 80 mm | woven satin care label (estimate) |
| Label as simulated, along its length | segments of 15 + 15 + 30 mm | continuous | departure, below |
| Tagging gun | 125 x 25 x 130 mm, 100 g | about 125 x 25 x 130 mm, about 100 g | standard tagging gun, a retailer's listing (estimate) |
| Needle beyond the nose, across | 25 mm, 1.9 mm | about 25 mm, about 2 mm | standard needle (estimate) |
| Push to pierce the label | 4 N held over 1 mm | 2 to 6 N | estimate for a 2 mm bevelled needle through woven polyester; how to measure it is below |
| Trigger preload, travel to fire | 25 N, 2.5 of 4 mm | not published | estimate |

`check_score.py` prints the same rows from the constants in `task.py` (`size_check`).

## Building it for a real trial

- **Shirt:** a child's cotton T-shirt, 180 g/m2 jersey, laid flat front up (body 30 x 28 cm, short sleeves), its care label drawn out of the side seam and lying on the front panel, loose on the table (not taped down), as in the simulator.
- **Label:** the shirt's own woven or satin care label, about 30 x 60 mm, sewn along its short edge. A label that lies flat can be given its curl by folding it once over a pencil.
- **Gun:** an Avery Dennison Mark III standard tagging gun (or the same class), a standard needle, a clip of 50 mm standard fasteners, a 30 x 50 mm card price ticket on the needle. A spare needle for the trials: needles bend.
- **Between trials:** cut the fastener off at its T-end with snips and draw it out of the label; replace the label (a fresh shirt) once its holes are within 5 mm of the next pierce point, and any shirt with a hole in its body.
- **Safety:** the needle is sharp; keep hands out of the station while the robot holds the gun, and fit a needle guard between trials.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Push to pierce the label | 4 N held over 1 mm of travel (estimate, range 2 to 6 N) | A push gauge (about 30 USD, 0.1 N steps) behind the gun's grip, pushing the needle slowly through the label held 3 cm up: the peak before the label gives |
| Label's bend at its seam and along itself | 3e-3 and 1e-2 N m per radian (estimates) | Hold the shirt flat, hang a 5 g clip at the label's free edge with the label held level at its seam: the droop angle |
| Seam's fold before the stitches load | 135 degrees from the shirt (estimate) | Fold the label back over its seam by hand until the stitches pull |
| Trigger preload and travel to fire | 25 N preload, fires at 2.5 mm of a 4 mm travel (estimates) | A force gauge on the trigger's middle: the force where it starts to move, and the travel when the fastener is pushed out |
| Gun mass and size | 100 g, 125 x 25 x 130 mm (retailer's listing) | Weigh and measure the gun |

## What the simulator leaves out

Each departure from the real shirt, label and gun, with its size and why:

- **Shirt:** native PhysX surface-FEM cloth, as Isaac Sim 6.1 provides it, on a 1 cm triangle mesh (real jersey is a knit of loops about 1 mm across): the mesh resolves the shirt's folds and seams, not its knit. Its in-plane stiffness (Young's modulus 2 MPa) and bend stiffness are effective values, not measured on a fabric, and Isaac's surface cloth has no static friction.
- **Label:** three thin rigid segments of 15, 15 and 30 mm on sprung hinges (real: a continuous woven strip that bends everywhere): PhysX's cloth cannot be pierced or pinched at this size. Its collider is 1.5 mm thick (real about 0.3 mm), so PhysX holds a fingertip and a needle contact on it.
- **Stitches:** a 34 x 18 mm rigid strip at the sewn edge, held to the shirt's nodes within 2.5 mm of it by PhysX's attachment (real: a line of thread stitches): it holds the label to the shirt and moves with it, and does not tear; the tear is scored at the seam hinge's stop.
- **Piercing:** a rule on the real push. The cloth does not part around the needle: once 4 N has been held over 1 mm, the needle's collider stays and is filtered against the pierced label segment only (PhysX filtered pair), so the shaft passes the hole it made and still collides with the shirt and the label's other segments: the needle cannot pass through the shirt (real: the needle's bevel parts the weave). Nothing holds the label on the needle: the hole's edge is the 7 mm tear rule, not a contact. The force and the travel are estimates.
- **Gun:** its body built from boxes and cylinders at the listed outer size (real: a moulded plastic body); the grip, the trigger along its front, the nose and the needle are where a real gun has them, at the sizes in the table.
- **Fastener:** shown once the trigger has fired with the needle through the label; how its T-end opens behind the label is not modelled, and the ticket hangs from the label as a shape, without its own physics.
- The demo reads the label's and the gun's poses from the simulator.

## Sources

- O*NET 41-2031.00 Retail Salespersons, task "Ticket, arrange, and display merchandise to promote sales": https://www.onetonline.org/link/summary/41-2031.00 (USDOL/ETA, CC BY 4.0).
- O*NET 53-7065.00 Stockers and Order Fillers, task "Mark stock items, using identification tags, stamps, electric marking tools, or other labeling equipment": https://www.onetonline.org/link/summary/53-7065.00 (USDOL/ETA, CC BY 4.0).
- BLS, May 2023 OEWS, Clothing and Clothing Accessories Retailers (NAICS 458100): 867,910 jobs, of them 492,610 retail salespersons and 27,940 stockers and order fillers: https://www.bls.gov/oes/2023/may/naics4_458100.htm
- BLS, May 2023 OEWS, Retail Salespersons: 3,684,740 nationally: https://www.bls.gov/oes/2023/May/oes412031.htm
- The pierce push (4 N over 1 mm), the trigger's preload and travel, and the label's stiffness are estimates; the table above says how to measure each on the real gun and label.

O*NET is a trademark of USDOL/ETA. Avery Dennison and Mark III are named for identification only.

## Assets

Every shape in the scene (label, gun, ticket, fastener) is built in code from boxes and cylinders in `task.py`, and the shirt's two-layer mesh is generated there from its outline; no third-party meshes, textures or fonts are used. The approach to the shirt (a tee as two sewn surface-FEM layers, with the seams' rest bend angles authored so that the solver does not unfold them) follows the accepted task "Peg a T-shirt onto a clothesline" (robocurve/benchmark-tasks, `tasks/peg-out-a-tshirt`, MIT licence); the code here is written afresh and none of it is copied. The YAM arms and the station come from `yam-isaac-lab-arena`.

## Licence

MIT
