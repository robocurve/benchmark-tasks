# Repair a fixture that lets a part seat backward

An assembly fixture is supposed to accept a part in one orientation. Here the key in the fixture's guide has broken off, so the part can seat backward. The robot must reproduce the fault and unlock and remove the guide. Then it fits the spare with the part number given on the fixture's data plate and locks it in. Finally it proves that the replacement allows the intended assembly and blocks the reversed one. This is a bench model of [mechanical error proofing in assembly](https://assets.balluff.com/WebBinary1/LIT_BRO_ERROR_PROOFING_EN_K14_DRW_919071_00_000.pdf) and of quick-change locators that slide under hold-down rails and are held by a pin.

The closest Robocurve tasks [choose a pin that fits](https://github.com/robocurve/benchmark-tasks/pull/30), [gauge pins and reject non-fitting ones](https://github.com/robocurve/benchmark-tasks/pull/48), and [flag a nest that blocks a plate](https://github.com/robocurve/benchmark-tasks/pull/155). This task is different in three ways. The original defect gives a successful seating reading. The guide has to be unlocked, slid out, and slid back in and pinned. And the right spare is found from its stamped part number, not by trying spares. Using the *same part* in both orientations shows that the repair fixes this specific failure without making the fixture unusable.

## How the guide is held

Each guide has four lugs on its sides and a tab at its robot-side end. To install a guide, lower it onto the fixture 16 mm short of its stop, where the lugs drop between the lip rails. Then slide it forward until it touches the stop, so all four lugs are under the lips. Finally, push the lock pin down through the slot in the tab into the slot in the base. Once the pin is in, the lips stop the guide from lifting, and the pin stops it from sliding back to where the lugs can clear the lips. A guide therefore cannot be lifted out, or placed from above, without pulling the pin and sliding the guide. The pin slots are 12 mm long along the slide direction, so a pinned guide can creep back up to about 6 mm; releasing it takes 16 mm. The slots are a simulation allowance: the YAM jaws sweep sideways a few millimetres as they close, which jams a pin in a close-fitting round hole. The guide's location is set by the stop and side rails, and the scorer requires the guide within 2.5 mm of the stop.

The fixture's data plate reads **427**. The original guide and one spare are stamped 427 on their tabs. The other spare is stamped **428**, the mirror-hand variant: its key accepts the part only with the arrows opposed.

## Fixed layout and reset

The official YAM station worktop is 747.5 mm above world zero. Coordinates below are in millimetres relative to the midpoint between the robot bases, with X forward, Y toward the left arm and Z above the worktop. Compound part offsets in `task.py` define the geometry. All objects start upright, with no rotation except the test part.

| Object | Materials, dimensions and initial location |
| --- | --- |
| Fixture | Fixed aluminium base, 120 × 88 × 10, spanning X 259–379 and Y −239 to −151; the installed guide's centre is at X/Y 335/−195. Two side rails (inner faces 35.5 mm either side of centre) carry four 12 mm long, 3.5 mm thick lips, centred 15 mm ahead of and behind the guide centre, with their undersides 15.5 mm above the table. An 8 mm tall stop sits ahead of the guide. The pin slot (12 × 9) is centred at X/Y 285/−195. A 26 mm tall parking boss with an 8 mm square bore is centred at 345/−255. Black data plate reading "427" (12 mm digits) at 269/−165; white arrow pointing +X at 269/−223; depth post with green seated and white blocked marks 12 mm apart at 317/−146. |
| Guides (3) | Grey 125 g polymer housings, 60 × 53 × 36, with an integral 24 × 22 × 22 handle on the far end, four 10 × 7 × 4 lugs, and a 28 × 28 × 4 tab on the robot side with a 12 × 8.5 pin slot and the stamped part number. Rectangular bore 39 × 31 at the throat, opening to 55 × 47 over a 9 mm chamfer, with a 3 mm floor. **Original (427, key missing):** installed and pinned, bottom at X/Y/Z 335/−195/10. **Spare 427:** on the table at 370/−335; steel key of three 4 × 4 × 12 mm prongs at local X/Y −10/−7, 10/−7 and 0/8. **Spare 428:** on the table at 465/−210; the same prongs mirrored (−10/7, 10/7, 0/−8). |
| Lock pin | 30 g steel pin: 6 mm across-flats shank, 11 mm long, with a 2 mm lead taper; 20 mm square, 3 mm collar; orange 18 × 26 × 45 handle. Starts fully in, with its axis at X/Y 285/−195 and the collar underside 14 mm above the table, on the original guide's tab. |
| Test part | Blue 62 g polymer piece, 34 × 26 × 70, with three 10 × 10 mm key pockets 16 mm deep in its lower end. Starts upright in the cradle with its arrow pointing −X, bottom at X/Y/Z 225/−110/6. Face marks show depth whichever way it faces. |
| Cradle and quarantine tray | Fixed polymer cradle, 67 × 64 with a 6 mm floor, centred at X/Y 225/−110. Red polymer tray, 150 × 90 with a 6 mm floor and 4 mm retaining lips, centred at 226/−302. |

Reset by putting the original guide and pin back in the fixture, the spares, test part and cradle contents back at the positions above, emptying the tray, and clearing the recorded tests. Nothing is consumed and nothing is randomised. The setup image shows the initial top-camera frame and the right wrist camera's initial view (428 spare). A third panel is the wrist view over the fixture at step 430 of the demo, turned upright, showing the 427 data plate and stamps. In a physical reproduction, keep the fixture unpowered, use blunt parts, and film the part's arrow and depth marks during each hands-off test.

## Scoring

Credit is cumulative, and each stage needs the ones before it. Each test reading must be visible and stable for 20 consecutive control steps (one second). The robot can verify the replacement in either orientation order.

- **0.25:** The original guide is installed and the backward part seats.
- **0.50:** After reproducing the fault, the original guide lies on the red tray floor, completely inside its lips. The 427 spare is installed and stationary.
- **0.75:** With the original in the tray, the same installed 427 spare gives both an aligned seat and an opposed block. Pulling the pin, moving the guide out of its slot, or a contradictory stable reading clears this verification.
- **1.00:** The verified spare stays installed, the original stays in the tray, and the test part stands on the cradle floor. Both arms stay clear for one second.
- **Capped at 0.25:** at any time the 428 spare is put into the fixture slot, meaning its centre is anywhere from 20 mm behind the installed position to 2 mm past it along X, within 3 mm across, with its bottom within 2 mm of the fixture's top surface. Trying spares in the slot is not identification by part number.
- **0 otherwise**, including a blind swap without reproducing the fault. An object falling more than 55 mm below the table (body origin) invalidates the whole trial.

**Installed** means both of the following:
- The guide is slid home: its centre is 0–2.5 mm behind the stop position along X and within 2 mm across, its bottom is within 1 mm of the fixture surface, it is within 3° of square, and upright within 3°.
- It is pinned: the pin collar is within 1 mm of the tab surface, the pin axis is within 3 mm of the slot centre along X and 2 mm across, and the pin is upright within 5°.

Evidence already earned stays while the guide remains within 4 mm (X), 3 mm (Y) and 2 mm (Z) of home, with the pin within 3.5 mm of the slot centre and no more than 4 mm raised.

**A valid part reading** needs all of the following:
- The part is centred within 3.2 mm of the guide along each horizontal axis, upright within 3°, and within 5° of its declared orientation.
- Its bottom is within 1 mm of the guide floor (seated) or 11–13 mm above it (blocked).
- The part and guide move at no more than 4 mm/s and 6°/s.
- Both arms' TCPs are outside a 105 mm radius of the fixture or at least 132 mm above the table.

A bad approach, a tilted jam or holding the part up is not evidence. For the finish, the part bottom must be within 1.5 mm of the cradle floor and its centre within 5 mm on each axis. Clearance heights over the cradle and the red tray are 120 and 102 mm.

## Run on Isaac Lab-Arena

Activate the program's official Isaac Lab-Arena environment and run from this task directory. The launcher adds rigid compound geometry to the official YAM station. It does not change the robots, cameras or policy actions. Readings and retention come from real rigid contact under gravity. The scripted demonstration uses simulated pose feedback to send ordinary 14-channel joint actions; it never moves an object except through robot contact. Dream-1 runs with the standard camera and proprioception interface.

```sh
python -m pytest -q test_scoring.py
python probe.py --output commissioning.json

instruction=$(python -c 'import task; print(task.INSTRUCTION)')
for policy in yam_scripted dream; do
  width=320; height=176
  if [ "$policy" = yam_scripted ]; then width=640; height=360; fi
  python run.py run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width="$width" -E cam_height="$height" \
    --instruction "$instruction" --scorer task_score \
    --max-steps 4000 --seed 0 --no-prompt --store-frames --log-dir "runs/$policy"
done
python export_media.py --demo runs/yam_scripted --dream runs/dream --out media
```

Use fresh log directories for repeats. Each trial allows 4,000 actions (200 simulated seconds at 20 Hz), about twice what the scripted demonstration needs. Arena reports `task_score`, and the launcher writes the depth, pose, pin and evidence state for every action to `proof-1.jsonl`.

## Commissioning results

The gravity probe (`probe.py`) ran on an NVIDIA L40S with Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0 and Inspect Robots 0.60.0. It places each guide slid home and pinned, then drops the test part from above the key. Every reading met the stability and alignment rules for at least 48 consecutive steps after release. Depth is relative to the guide floor.

| Guide | Aligned | Opposed |
| --- | --- | --- |
| Original 427, key broken off | Seated (0.00 mm) | Seated (0.00 mm) |
| Spare 427 | Seated (0.00 mm) | Blocked (12.00 mm) |
| Spare 428, mirrored key | Blocked (12.00 mm) | Seated (0.00 mm) |

Retention, applying a 0.5 s velocity pull to the 427 spare in the slot:
- **Pinned, pulled up:** the guide stays home, moving 0.08 mm.
- **Pinned, pulled back toward the entry:** the guide creeps 5.6 mm, within the slotted pin hole, and stays under the lips.
- **Unpinned, pulled back:** the guide slides 35.7 mm, clear of the lips.

The scorer tests cover:
- a blind swap;
- an unpinned, or not fully slid, guide;
- trying the 428 spare, including only dropping it into the slot;
- a 428 spare installed;
- tilted or off-centre parts and gripper assistance;
- verification in either order;
- clearing verification when the pin is pulled.

The scripted YAM run scored **1.0 in 2,034 actions** (101.7 simulated seconds). It:
1. measured 0.00 mm on the original guide with the arrows opposed;
2. pulled the pin into the parking bore and slid the original out to the tray;
3. slid the 427 spare to the stop and pinned it;
4. measured 0.00 mm with the arrows aligned and 12.00 mm with them opposed;
5. returned the part to its cradle.

The standard Dream-1 run scored **0.0 in 4,000 actions** (200 simulated seconds). It did not reproduce the backward seating fault: no object moved during the trial. Both trials used the same fixed scene and seed 0. These are simulator results.
