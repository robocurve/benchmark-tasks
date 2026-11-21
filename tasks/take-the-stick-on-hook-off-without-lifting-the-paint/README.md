# Take a stick-on wall hook off without lifting the paint

A medium utility hook hangs on a painted drywall surface by a foam stretch-release strip. The strip's pull tab lies flat against the same uninterrupted surface, just below the hook. The robot has to take the hook off without damaging the paint and put it in a tray. Pulling the hook itself, or peeling the strip up off the wall, takes the paint with it; stretching the strip along the wall lets it go cleanly.

Everything here is simulation. No physical robot trial has been run.

## Setup

Coordinates are millimetres on the station table: x forward from the midpoint of the arm-base line, y to the robot's left, table top z=0.

- **Wall.** One continuous 250 x 1000 x 12.7 mm paper-faced drywall panel (the coupon), primed and painted, lying face up with its centre at (270, 0) and its painted face 62.7 mm above the table. About 2.0 kg (8 kg/m2). It rests on two pine battens, 30 x 1000 x 50 mm, under x=185 and x=355, and four steel corner clips keep it in place: at each corner two 30 x 8 mm posts stand 0.5 mm outside the edges, and a 3.5 mm thick lip reaches 15 mm over the corner, 0.5 mm above the face. The robot never moves it. The painted face runs unbroken from x=145 to 395 and y=-500 to +500. There is no edge, seam, cut-out, spacer or gap anywhere near the hook, the tab or the pull path.
- **Hook.** A white plastic medium utility hook: back plate 24 x 62 x 4 mm, stem 14 x 14 x 21 mm. From the stem's top a 14 mm wide, 5 mm thick arm runs 11 mm to the plate's upper end and turns down in a 5 x 6 mm lip. About 15 g.
- **Strip.** One 3M Command Medium Refill Strip (17021), white foam, listed by a distributor as 2-3/4 x 3/4 x 1/20 in, which is 70 x 19 x 1.27 mm. Both liners are removed. 50 mm of the strip is bonded between the wall and the plate. The other 20 mm is the pull tab: the strip's own unbonded end, the same foam, taken as 19 mm wide and 1.27 mm thick like the rest of the strip. The listing gives only the strip's overall size, so the tab's 20 mm length and its thickness are assumptions to be checked on the strip used. The tab is drawn red in the simulation so that it can be seen.
- **Lot A.** Hook centre line at x=270. The bond runs from y=-40 to y=+10 under the plate, the plate from y=-40 to y=+22, and the stem centre is at y=+4. The tab lies flat on the painted face from y=-60 to y=-40, just below the plate's lower end, with nothing under or behind it but the wall. It is not lifted, bent or gripped at the start. When the last of the strip lets go, the tab's end is at y=-365, 135 mm short of the coupon's end.
- **Lot B.** The coupon, hook and strip turned half round in the same clips: the tab lies from y=+40 to y=+60 and the pull goes toward +y.
- **Tray.** Grey plastic, fixed to the table, inside 100 x 150 mm, walls 4 mm thick and 25 mm high, floor 3 mm, centre (78, 0).

Trial 1 uses lot A and trial 2 uses lot B. The screenshot shows lot A at the start. The demonstration video shows trial 1 (lot A) and then trial 2 (lot B), each from its first step until the full score had held, with a label in the corner. The Dream-1 video shows its trial 1 (lot A).

## How the strip and paint are modelled

The 50 mm bond is five 10 mm cells. The strip is five links on 120 N/m springs. The cell nearest the tab lets go once its link has been stretched 61 mm (about 7.3 N), then the next cell can stretch, and so on; all five take 305 mm, 12 inches, of pull. The tab is a rigid plate hinged to the first link along the bond's lower end. It starts at 0 degrees, flat on the face, and cannot go below the face.

Paint comes away, and the trial scores 0, when any of these happens at any time:

- the hook is moved more than 2 mm off the face, or more than 4 mm along it, while any cell is still bonded (the cells still bonded tear);
- a cell lets go while the tab is lifted more than 30 degrees off the face;
- the tab is lifted more than 45 degrees off the face once the front link is stretched 30.5 mm or more (the strip peels).

These limits are rules chosen for the simulation. They are not measured values for this strip or paint.

The hook can slide in three directions but cannot turn. The spent strip stays attached to the hook and springs back to it when let go. Only the tab can be gripped; the stretching links have no collision shape. Nothing in the scene attaches the tab to a gripper: it is held by contact and friction only.

## Scoring

Each trial scores 0, 0.5 or 1.0, and the scores of the two lots are averaged.

- **0** in any of these cases: any paint under the bond came away at any time in the trial; the strip has not let go along its whole bonded length (the hook is still stuck on); the hook has not been let go and still for the last 1 s, with every gripper part more than 5 mm from the hook, its strip and its tab; the hook rests anywhere but on the coupon, the table or the tray floor with its lowest point within 2 mm of that surface (propped on the tray's rim, say); or the hands' positions could not be read.
- **0.5** if none of that applies: the strip has let go cleanly and the hook has been let go and has lain still for 1 s on the coupon, the table or the tray floor, with its lowest point within 2 mm of that surface.
- **1.0** if, as well, the whole hook lies inside the tray's inner walls with its lowest point within 2 mm of the tray floor.

Still means the hook moves at under 10 mm/s. The gripper distance is measured from points on every face of both grippers and all four fingertips. The record of paint damage covers the whole trial and is cleared only when the simulator resets the scene for a new trial. In a real trial the same three levels are read from the video, a photo of the bond area after the trial (any missing paint or bare paper is damage), a photo of the tray from above, and a ruler.

`near_misses.py` checks each rule at and either side of its limit in both lots, the release and stillness tests, and the cases that must score 0 (hook pulled straight off, hook slid sideways, tab peeled up, hook still held, hand positions unknown).

## The scripted demonstration

The demonstration runs open loop from the published layout and reads no simulator state.

1. One hand comes straight down with a finger either side of the flat tab. Its fingertips are commanded 5 mm into the wall, so the arm, which is soft, presses them lightly onto the painted face.
2. The fingers close on the tab's two long edges. The jaw target is 9 mm for the 19 mm tab, which gives about 25 N from each finger (23 to 28 N in the recorded run).
3. The hand draws the tab along the wall, away from the hook, without lifting it: 190 mm commanded over 5.5 s.
4. Once the pulling hand has moved clear, the other hand takes hold of the hook's stem.
5. The pulling hand carries on to 400 mm commanded over 10 s. The arm is soft, so its target leads the tab by about 35 to 50 mm under the strip's pull, and the strip gets the 305 mm it needs with room to spare. The other hand holds the hook while the last cells let go. Once the last cell has let go, the strip's pull draws the held hook about 40 mm along the wall toward the pulling hand, because that arm is soft too, until the strip is let go.
6. The pulling hand lets the spent strip go; it springs back to the hook. The other hand carries the hook to the tray, sets it down, lets go and moves away.

The plan lasts 35.7 s of the 40 s limit. A trial ends early once the full score has held.

## Paint preparation, curing and reset

1. **Coupons.** Smooth paper-faced 12.7 mm drywall, 250 x 1000 mm. One coat of BEHR Drywall Plus Primer & Sealer No. 73, then two coats of BEHR Premium Plus Interior Eggshell Enamel No. 2050 Ultra Pure White, applied with a 10 mm nap roller at each label's spread rate and recoat time. Use the same product batches for every coupon. Log the products, the number of coats and the date of the last coat.
2. **Curing.** Every coupon cures at 20-22 degrees C and 40-60% relative humidity for 30 days after its last coat, or for the paint maker's full-cure time if that is longer. Touch-dry paint is not cured. The 30 days is a control chosen for repeatability, not a maker's guarantee.
3. **Reuse.** After each trial photograph the 19 x 50 mm bond area. If no paint is missing, the same coupon and position may be used again. If any paint lifted, retire the coupon; do not patch or repaint it. Every trial is therefore run on paint prepared and cured exactly as in steps 1 and 2.
4. **Mounting.** Wipe the bond position with rubbing alcohol and let it dry. Take a new strip. Peel one liner and press the strip onto the back of the hook's plate with the pull tab left clear below the plate; peel the other liner and press the hook onto the wall at the lot's marked position for 30 s. Wait 1 hour. Leave the pull tab lying flat against the wall as it falls. Do not lift, bend or thicken it, and do not add a grip aid.
5. **Start.** Clip the coupon in place for lot A or lot B. Empty the tray; it stays fixed at its marks. Photograph the start and record the strip's part number.

## Run

Run on the program's yam-isaac-lab-arena station. `parts.py` and `lots.py` sit beside `task.py`; no installed file is changed.

```sh
I="Take the hook off the painted panel without damaging the paint, and put it in the tray."
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 --instruction "$I" --scorer task_score --max-steps 800 --epochs 2 --store-frames
python near_misses.py
```

For the Dream-1 run use `--policy dream` with 320 x 176 cameras. `TASK_LOT=A` or `TASK_LOT=B` pins one layout. The limit is 800 control steps, 40 s at 20 Hz. The two demonstration trials took about 12 minutes of wall-clock time on one GPU, start-up included. No random seed changes the task: object jitter is off and the two lots are fixed.

## Results

Both runs used this folder's code unchanged, two trials each (lot A, then lot B), with the 800-step limit.

| Run | Lot A | Lot B | Mean |
|---|---|---|---|
| Scripted demonstration | 1.0 | 1.0 | 1.0 |
| Dream-1 | 0.0 | 0.0 | 0.0 |

Scripted demonstration: each trial ended on success after 647 steps (32.4 s). In lot A the five cells let go at 9.2, 10.45, 16.05, 18.25 and 20.5 s, and in lot B at 9.25, 10.5, 16.05, 18.3 and 20.55 s, each with the tab 1 to 3 degrees off the face. In the state logged four times a second, the hook moved about 2 mm along the face while the strip was bonded (limit 4 mm) and at most 0.6 mm off it (limit 2 mm). The gripped tab held the jaw at 18.8 to 18.9 mm throughout the pull, and the fingertips stayed within 0.25 mm of the painted face.

Dream-1: both trials ran to the 800-step limit. The arms moved about over the coupon but never took hold of the tab or the hook. No cell let go and no paint was lifted, so each trial scored 0 under the rule that the strip must have let go. The run raised no error and the model did not decline the task.

## What is simplified

- The coupon lies flat on the table, so gravity acts into the wall rather than along it. On an upright wall a freed hook would drop.
- The tab is a rigid plate on one hinge. A real foam tab bends, and folds up between fingers that pinch it from the sides; the rigid tab cannot. A grip that holds the rigid tab does not prove that the same grip holds the real one.
- The strip's size comes from a distributor's listing for the 17021 refill strip. The split into 50 mm bonded and 20 mm tab, and the tab's thickness, are assumed and should be measured on the strip used. The demonstration's grip depends on the tab being about as thick as the strip: the fingertips' lower inner edges are rounded over about 0.5 mm, and a tab thinner than that is not held.
- The spring rate, the 61 mm release stretch and the paint limits are uncalibrated.
- Paint damage is modelled only under the strip's bond. The demonstration's fingertips press lightly on the painted face (roughly 1 N, estimated from the arm's stiffness, not measured) and slide along it for about 390 mm; any scuffing that would cause on real paint is not modelled or scored.
- The link masses (8 g for each strip link, 1 g for the tab, 20 g for each of the two hidden slide links that carry the hook) are numerical, not weighed.

## Sources

- Strip size: [Uline listing for 3M 17021 Command Refill Strips, Medium](https://www.uline.com/Product/Detail/S-26012/3M-Command/3M-17021-Command-Refill-Strips-Medium).
- Product family and removal guidance: [3M medium refill strips](https://www.command.com/3M/en_US/p/d/b5005604148/), [3M hook application and removal](https://www.command.com/3M/en_US/command/how-to-use/all-other-hooks/), [3M FAQ](https://www.command.com/3M/en_US/command/contact-us/faqs/).
- Paint system: [BEHR gypsum-board coating schedule](https://www.behr.com/binaries/content/assets/behrpro/pdf/specifications/behr-master-spec_section_09-90-00_painting-coating_01232026.pdf), [No. 73 primer](https://www.behr.com/pro/products/interior-products/interior-primers-sealers-and-undercoaters/primers-sealers-and-undercoaters/behr-drywall-primer-sealer), [No. 2050 eggshell enamel](https://www.behr.com/pro/products/interior-products/interior-paints-and-coatings/paints-and-coatings/behr-premium-plus-interior/behr-premium-plus-eggshell-enamel).
