# Swap the hotend on a Bambu Lab A1 mini 3D printer: buckle open, old hotend to its holder, new one seated, locked

A Bambu Lab A1 mini 3D printer stands on the table, powered off and cold, its front toward the robot. The toolhead's
front cover and silicone sock are off and the filament is cut, as Bambu's guide asks before a hotend change; the
toolhead is parked with its part-cooling ducts 20 cm above the table. The installed 0.4 mm hotend sits as on the A1:
its finned heatsink in a recess under the toolhead's body, the front 8.5 mm of it standing proud of the body, the tube
down through the heating assembly and the nozzle 3.5 mm below the ducts. A buckle across the front of the heating
assembly holds it, locked. In front of the printer stands a printed holder with two slots: the new 0.2 mm hotend
(green tag) stands in the right one, the left one is empty. The instruction:
"Change the 3D printer's hotend: unlock the buckle and swing it open, take the old hotend out and stand it in the empty slot of the holder, then put the new hotend in until it sits flush and lock the buckle."

The procedure is Bambu Lab's for A1-series printers
([Hotend Replacement for A1 Series](https://wiki.bambulab.com/en/a1/maintenance/replace-hotend); the A1 mini takes
the same hotend): push the nozzle buckle to the right to unlock it and swing its left side outward, hold the heatsink
and pull the hotend out, put the new hotend in until it sits flush, then flip the buckle's left side back into place
and lock it. The setting is robot building: low-cost arms such as LeRobot's SO-ARM100 and XLeRobot are built from
parts printed on these printers, so a lab building them keeps its printers running as part of the build, swapping the
hotend between parts to clear a clog, replace a worn nozzle or change nozzle size for the next part. What it tests:
a fixed order (unlock, open, out, in, shut, lock) on small parts under the toolhead, where an unlocked buckle or a
hotend left short of its seat is easy to miss.

- **Unlock before opening, lock after closing.** Locked, the buckle's left end sits behind a catch: it opens only
  after it is pushed 2.5 mm to the right, and it counts as locked only once it is shut and pushed back left.
- **Out, and put away.** The old hotend must end up standing in a holder slot. If it is dropped on the way, that part
  earns nothing.
- **Seated, not just put in.** The new hotend counts as seated only within 1 mm of its seat along each axis (flush
  against the mounting plate, on the heating assembly) and upright to 3 degrees. Dropping it voids the run.

## Revision (Nov 26)

Changes for the reviewer's request on PR #4104 (model the buckle's push-to-unlock and locking actions; use hotend
geometry that keeps the real A1 hotend's graspable dimensions and clearance under the toolhead; update the fidelity
note; full-score demo within the action limit):

1. **Hotend at the real size.** 47.3 mm long (Bambu's listing for the Bambu Hotend - A1 Series), 18 g. Proportions
   measured on Bambu's product photograph, scaled to 47.3 mm: heatsink 20 mm front to back, 15.5 mm wide, 16.5 mm
   tall; 4.4 mm inlet stub on top; 6.5 mm tube 25.3 mm long; nozzle 2.8 mm. (Before: a 1.2 x 1.8 x 4.6 cm heatsink,
   6 cm overall.)
2. **Seated as on the A1, with the A1's clearances.** The heatsink sits in a recess under the toolhead's body (1 mm
   under its ceiling, 2 and 5 mm from its side walls), on the heating assembly, its magnet block against the
   assembly's mounting plate; only its front 8.5 mm stands proud of the body (front cover off), which is what a hand
   or gripper can pinch. The tube runs down a groove in the heating assembly; the nozzle ends 3.5 mm below the ducts.
   The hotend goes in and out horizontally, from the front. (Before: the hotend's top went 1.2 cm up into a pocket,
   with the rest of its 4.6 cm heatsink below the toolhead.)
3. **The buckle's push-to-unlock and lock.** The buckle is a 15 mm bar across the front of the tube, on a vertical
   hinge at its right end; the hinge rides a 2.5 mm slide. Locked, the bar's left end sits behind a catch on the left
   duct. Pushing the buckle 2.5 mm to the right frees that end and the bar swings out to the front; closing, the bar
   is swung shut while still pushed right (so its end passes the catch) and then pushed back left, which puts its
   end behind the catch. Scoring requires both: shut, and the slide back at its locked end. (Before: a bar on a
   hinge, no unlock or lock.)
4. Fidelity text, setup, scoring, instruction, demo, video and Dream-1 run updated; the demo scores 1.0 in 1,128 steps
   (limit 2,400).

## Files

| File | What |
|---|---|
| `task.py` | The task: printer (block model) with toolhead, recess, heating assembly, ducts, buckle on its slide and catch; two hotends; the holder; `success()`, `score()`, `report()` |
| `kitchenkit.py` | Parts from boxes and cylinders, joints (hinge, slide), per-step poses and finger pads |
| `demo.py`, `democore.py` | The scripted demo and its variants (`democore.py`: runner, planning and object-servo helpers) |
| `demo_results.json` | Each variant's score, end-state report and per-phase tracking error |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (stops if they no longer match the text) |
| `setup_shot.py` | Writes the top camera's starting view |

Standard `yam_arena` embodiment and `task_score` scorer. Assets of our own: none; every part is boxes and cylinders
built in `task.py`. Third-party assets and licences: the YAM arm model (i2rt, github.com/i2rt-robotics/i2rt, MIT;
`LICENSE.i2rt` in yam_arena); the `station` scene and the `yam_arena` embodiment
(github.com/kairosiann/yam-isaac-lab-arena; its README states it has no licence yet); Isaac Lab-Arena (Apache-2.0);
Isaac Lab (BSD-3-Clause).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Change the 3D printer's hotend: unlock the buckle and swing it open, take the old hotend out and stand it in the empty slot of the holder, then put the new hotend in until it sits flush and lock the buckle." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/hotend-swap-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps; with the default 48 rows a simulator slower than
real time lets the Dream session expire between chunks. `task_score` prints 1.0 on success, otherwise `score()` at the
end. **Seeds:** none; the layout is fixed (`object_jitter=0`). **Runtime:** the recorded full demo ran 1,128 control
steps (56.4 s simulated at 20 Hz) in 50 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU that was also
running three other simulations and other users' jobs (about 2.6 s per step; the same GPU runs this scene at about
0.35 s per step when it is not shared), start-up and video writing included. A model trial runs up to 2,400 steps
plus the policy's response time. The Dream-1 run reached the 2,400-step limit with a score of 0 in 105 min of
wall-clock time on that shared GPU (mean model response 0.59 s). Because the simulator was that slow, every 12-step
chunk was served by a new Dream session (the previous one had expired). Both arms stayed in front of the printer and
neither reached the toolhead: the right gripper pointed at the printer and ended at the holder's front. The top
camera's last frame shows the printer and the holder as at the start (old hotend in the toolhead, new hotend in its
slot). A second Dream-1 run with `-P rows_per_request=2`, so that one session lasted throughout, also never reached
the toolhead: for 1,852 steps both arms stayed near their bases, and then the run stopped with an error when Dream-1
returned a right-gripper command outside 0..1; it has no score.

## Scripted demo

```bash
python demo.py task.py demo_out                       # full: demo.mp4, setup.png, result.json
python demo.py task.py out_no_lock --variant no_lock  # any variant below; --no-record skips the video
python demo.py task.py --plan-only                    # inverse-kinematics check of every variant's plan
```

The right arm pinches the buckle's tab top and bottom, pushes the buckle 2.5 mm to the right (6.2-7.2 s), swings it
out to 80 degrees (7.2-9.8 s) and lets go; the buckle's light spring takes it on to its 95-degree stop. The left arm
pinches the old hotend's heatsink by its sides from the front, as a hand does (17.1-18.1 s), pulls it straight out of
the recess (18.1-20.1 s), carries it to the holder, lowers it into the empty left slot (23.5-25.1 s) and lets go.
Meanwhile the right arm takes the new hotend out of the right slot the same way (20.1-22.6 s), brings it level in
front of the recess, slides it in until its magnet block meets the mounting plate (29.6-33.4 s) and lets go
(34.0-34.8 s). It then pinches the tab again, swings the buckle shut while it is still pushed right, so that its end
passes the catch (43.2-48.7 s), pushes it back left so that its end goes behind the catch (48.7-52.7 s), lets go
(53.3 s) and moves away. The script reads the parts' poses each step (as any scripted policy with full state may) and
shifts the gripper's target so that the part it holds follows the plan: the old hotend by its nozzle's tip, the new
one by the point where its tube meets the heatsink, the buckle by its tab. Parts move only through the arms' grasps
and contacts; nothing is teleported, and no pose or joint of a part is set.

Timeline of `demo.mp4` (planned phases): the buckle is unlocked at 7.2 s and open at 9.8 s; the old hotend is out of
the toolhead at 20.1 s and in its slot at 25.1 s; the new hotend is seated at 33.4 s; the buckle is shut at 48.7 s and
locked at 52.7 s. No finger is near the new hotend or the buckle from about 54 s, and the run ends at 56.4 s (step
1,128) once the finished state has held 1 s plus the harness's 1 s success hold. End state: the new hotend within
0.4 mm of its seat, tilt 0 degrees; the buckle shut (resting on the catch at 3.1 degrees) with its slide 0.08 mm from
the locked end; the old hotend in the empty slot.

| Variant | What it does | Score | Steps | End state |
|---|---|---|---|---|
| `full` | the whole procedure | **1.0** | 1,128 | old hotend in the empty slot; new one seated (within 0.4 mm, tilt 0 deg); buckle shut and locked (3.1 deg, slide 0.08 mm from its locked end); success |
| `no_unlock` | first tries to swing the buckle open without pushing it right, then does it properly | **1.0** | 1,177 | while locked, the catch holds the buckle at 3.4 deg; after the push right it opens, and the run ends as `full` (success) |
| `no_lock` | swings the buckle shut but does not push it back left | 0.6 | 1,172 | old hotend in its slot; new one seated; buckle not locked: its slide stays 2.5 mm right, and once let go its spring swings it back open (24.7 deg) |
| `half_in` | lets the new hotend go 3 mm short of its seat, then closes and locks the buckle | 0 | 1,236 | the new hotend tips forward off the heating assembly and falls (dropped); the buckle locks over the empty recess |
| `drop_old` | lets the old hotend go over the bed instead of in the holder | 0.8 | 1,222 | old hotend on the bed (dropped); new one seated, buckle locked, hands off |
| `drop_new` | lets the new hotend go over the bed on its way to the toolhead | 0 | 758 | new hotend on the bed (dropped); old one in its slot; buckle left open |

From `demo_results.json`.

## Scoring

The rules are in `task.json` (`scoring`) and `score()`'s docstring; `report()` gives the buckle's angle and slide
position, whether it is latched, each hotend's pose relative to the toolhead and the holder, whether it was dropped,
and the hands-off and hold counters.

Score = 0.2 (old hotend standing in a holder slot, not dropped on the way) + 0.4 (new hotend seated) + 0.3 (buckle
shut and locked with the new hotend seated) + 0.1 (that finished state held 1 s with both hands off and the new hotend
still); 0 if the new hotend is dropped. The run is a success, and ends, once the finished state with the old hotend in
a slot has held for 1 s.

A real trial scores the same rules from photos and the video:

- **Old hotend in a slot:** a photo of it standing upright in either slot (not lying, not on the bed or table).
- **Locked:** the buckle shut and pushed back left, latched (end photo; pulling its left side does not open it).
- **Finished:** no part of either gripper touches the new hotend or the buckle in the last second (video).
- **Seated:** after the end photo, a strip of printer paper (about 0.1 mm) does not slide between the heatsink and the
  heating assembly, and the heatsink does not wobble when nudged (Bambu's check).
- **Dropped:** the new hotend falling or set down on the bed, the table or the floor voids the run.

## Real-world trials

Per trial (about 5 min): put the old hotend back and lock the buckle, stand the new one in the holder's right slot,
check the printer is off and cool, run, then photograph the end state and do the paper and wobble checks.

- **Printer:** a Bambu Lab A1 mini (US$199 on sale, US$219 list at Bambu's US store, Nov 2026). Before the trials,
  with the printer on, raise Z so the part-cooling ducts' undersides are 20 cm above the table and park the toolhead
  over the bed's centre, then cut and unload the filament with the toolhead's lever, take off the front cover and the
  silicone sock (Bambu's guide), and switch the printer off. Let the nozzle cool below 50 C. Place the printer so the
  installed hotend's axis is 37.5 cm in front of the arm bases' midpoint, on their midline (y 0); the block model's
  base coordinates in `task.json` are for the simulation only. Mark the printer's feet on the table with tape; it
  stands loose (5.5 kg). Powered off, the toolhead slides along the X arm: tape its parked position on the arm and
  slide it back before each trial.
- **Hotends:** two Bambu Hotend - A1 Series units (about US$10-13 each): a 0.4 mm one installed and locked, a 0.2 mm
  one as the new hotend with a strip of green tape on its heatsink's front. Swap their roles freely between trials.
- **Holder:** print it in PLA (0.2 mm layers, 15% infill, about 200 g): a 4 x 16 x 7 cm block with two bosses on top,
  each 1.2 x 1 x 3 cm (1.2 cm front to back) with a 0.75 cm square hole down its middle for the tube, 13 cm apart
  (positions in `task.json`'s setup); the hotend stands in the hole with its heatsink resting on the boss. It stands
  loose on the table, its front face 23 cm in front of the arm bases' midpoint, slots at y -3.5 (new) and 9.5 (empty).
  Check that a real A1 hotend's tube goes into the hole; widen it if needed. OpenSCAD model (mm; x front to back):
  `difference() { union() { cube([40, 160, 70]); for (y = [15, 145]) translate([14, y - 5, 70]) cube([12, 10, 30]); }
  for (y = [15, 145]) translate([16.25, y - 3.75, 70]) cube([7.5, 7.5, 31]); }`
- **Camera:** a phone on a tripod at the front left, at the toolhead's height (about 20 cm), aimed at the buckle and
  the heatsink, plus the station's top camera.
- **Safety:** the printer is off and cold, nothing is heated; a hotend weighs 18 g; nothing is sharp. The buckle is
  worked with a fingertip (in the model, 1.5 N moves its slide at 1 cm/s); the YAM's ~40 N per finger is far above
  that.

## Simulation fidelity in full

Beyond `task.json`'s fidelity text: every part is a rigid body built from boxes and cylinders. The printer (one loose
5.5 kg body, friction 0.8) keeps what the job touches: base and bed, the Z column at the right rear, the X arm and
carriage behind the toolhead, and the toolhead's underside: the body with its recess and inlet slot, the heating
assembly (a block with a 7.2 mm groove for the tube) and its mounting plate, the two part-cooling ducts and the
buckle's catch. The toolhead is part of the printer body; with the power off the real carriage can be slid along X by
hand. The toolhead's dimensions around the hotend (body front 3 mm in front of the hotend's axis, recess walls,
ceiling 1 mm above the heatsink, duct positions, the buckle's height and length) are estimates from Bambu's photographs
of the A1 toolhead with its front cover off, not measured parts.

The hotend has the real one's length (47.3 mm), its proportions as measured on Bambu's product photo and its listed
weight (18 g; friction 0.8 for the finned heatsink). On the A1 a magnet on the heatsink pulls the hotend back against
the heating assembly; here the magnet block is a plain box: gravity holds the hotend down on the heating assembly, the
recess's walls and ceiling and the mounting plate bound it, and the closed buckle keeps it from sliding forward. A
hotend let go short of its seat is not pulled home as the A1's magnet would pull it: let go 3 mm short, it tips forward
off the heating assembly and falls (the `half_in` variant). On the real printer the magnet may seat such a hotend, so
there the rules still score what the end photo and the paper check show.

The buckle is a bar on a vertical hinge (0 to 95 degrees) whose hinge rides a slide with 2.5 mm of travel to the
right. The slide is damped (150 N s/m): it stays where it is pushed, as the real buckle's right side does until it is
pushed back to lock. A light spring (3e-5 N m per degree) on the hinge holds the opened bar at its 95-degree stop
and a locked bar against the catch. The bar and the slide's carriage weigh 20 g and 50 g (the real stamped buckle is a
few grams) and the carriage is a box inside the mounting plate, out of every other part's reach: the joint solver needs
that mass and inertia to keep the hinge on its axis under the gripper's forces. The real buckle's exact shape and the
spring and detent at its right side are not modelled: here "locked" is the bar's left end behind the catch with the
slide back at its locked end. Finger pads and part poses are read once per control step.
