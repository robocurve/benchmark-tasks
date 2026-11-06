# Repair a fixture that lets a part seat backward

An assembly fixture is supposed to accept a part in one orientation. Here, an internal key is missing, so the part can seat backward. The robot must reproduce the fault, replace the guide, and prove that the replacement allows the intended assembly and prevents the reversed assembly. A second spare blocks both orientations: showing that a part stops short is not enough to release the fixture. This is a bench model of [mechanical error proofing in assembly](https://assets.balluff.com/WebBinary1/LIT_BRO_ERROR_PROOFING_EN_K14_DRW_919071_00_000.pdf).

The closest Robocurve tasks [choose a pin that fits](https://github.com/robocurve/benchmark-tasks/pull/30), [gauge pins and reject non-fitting ones](https://github.com/robocurve/benchmark-tasks/pull/48), and [flag a nest that blocks a plate](https://github.com/robocurve/benchmark-tasks/pull/155). Here the original defect gives a successful seating reading. Using the *same part* in both orientations verifies that the repair changes the specific failure without making the fixture unusable.

## Fixed layout and reset

The official YAM station worktop is 747.5 mm above world zero. Coordinates below are in millimetres relative to the midpoint between the robot bases, with X forward, Y toward the left arm and Z above the worktop. Compound part offsets in `task.py` define the geometry. All housings start upright and unrotated.

| Object | Materials, dimensions and initial location |
| --- | --- |
| Fixture | Fixed aluminium base, 92 × 84 × 10, centered at X/Y 335/−195, with four 12 mm tall locating rails and a tapered nest. A white arrow points along +X; a fixed post carries green seated and white blocked marks 12 mm apart. |
| Installed guide and two spares | Three identical grey 115 g polymer housings, 60 × 53 × 36 excluding the integral 24 × 22 × 22 handle. The rectangular bore is 39 × 31 at the throat, opening to 55 × 47 over a 9 mm chamfer. Installed guide bottom is at X/Y/Z 335/−195/10 and has no key. Spare at 350/−325/0 has a steel key formed by three 4 × 4 × 12 mm prongs at local X/Y −10/−7, 10/−7, and 0/8. Spare at 465/−210/0 has an oversized 34 × 26 × 12 mm key that blocks both directions. All three have a 3 mm floor. |
| Test part | Blue 62 g polymer piece, 34 × 26 × 70 with three 10 × 10 mm key pockets 16 mm deep in its lower end. Starts upright in the cradle, with its arrow pointing −X and its bottom at X/Y/Z 225/−118/6. Two face marks show depth whichever way it faces. |
| Cradle and quarantine tray | Fixed polymer cradle, 67 × 64 with a 6 mm floor, centered at X/Y 225/−118. Red polymer tray, 135 × 84 with a 6 mm floor and retaining lips, centered at 226/−299. |

Reset by returning the three guides and test part to the positions above, emptying the tray, and clearing the recorded tests. No consumables, randomization, or repositioning between resets. Use the initial camera frame as the setup image. In a physical reproduction, keep the fixture unpowered, use blunt parts, and film the part's arrow and depth marks during each hands-off test.

## Scoring

Credit is cumulative and gated by the prior stages. Each test must be visible and stable for 20 consecutive control steps (one second). The robot can try either spare and can verify the replacement in either orientation order.

- **0.25:** The original guide is installed and the backward part seats.
- **0.50:** After reproducing the fault, the original guide is on the red tray floor, completely inside its lips, and either spare is seated in the fixture.
- **0.75:** With the original in the tray, the same installed spare yields both an aligned seat and an opposed block. Removing it or observing a contradictory stable result clears its verification. A brief rock inside the nest pauses readings but keeps prior evidence: the guide must stay within 6 mm of center per axis, 7 mm of seated height, 14° of upright and 12° of either nest yaw.
- **1.00:** The verified spare remains installed, the original stays in the red tray, and the test part stands on the cradle floor. Both arms stay clear for one second.
- **0 otherwise**, including a blind swap without reproducing the fault. An object falling more than 55 mm below the table at its body origin invalidates the entire trial.

A seated guide is within 1 mm vertically, 3.5 mm along each horizontal axis and 4° of either nest yaw, and upright within 3°. A valid part reading is centered within 3.2 mm of that guide along each horizontal axis, upright within 3°, and within 5° of its declared orientation. The part bottom must be within 1 mm of the guide floor to count as seated, or 11–13 mm above it to count as blocked. The part and guide must move at no more than 4 mm/s and 6°/s. A bad approach, tilted jam, or holding the part up is not evidence. The arms must withdraw from the test area (TCP outside the 105 mm radius or at least 132 mm above the table). The cradle finish also checks the part bottom is within 1.5 mm of its floor and the part center within 5 mm on each axis; clearance heights over the cradle and red tray are 120 and 102 mm respectively.

## Run on Isaac Lab-Arena

Activate the program's official Isaac Lab-Arena environment and run from this task directory. The launcher adds rigid compound geometry to the official YAM station. It does not change the robots, cameras or policy actions. The depth difference is set by real rigid contact under gravity. The geometry is rigid and uses square edges; surface wear and deformation are omitted. The scripted demonstration uses simulated pose feedback to send ordinary 14-channel joint actions; it does not move any object outside robot contact. Dream-1 runs with the standard camera and proprioception interface.

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
    --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir "runs/$policy"
done
python export_media.py --demo runs/yam_scripted --dream runs/dream --out media
```

Use fresh log directories for repeats. Each trial allows 1,800 actions (90 simulated seconds at 20 Hz). Arena reports `task_score` and the launcher writes the depth, pose, and evidence state per action to `proof-1.jsonl`. The six-case commissioning probe presets the guide and drops the part from above the key; it validates the contact outcomes separately from the robot's ability to perform the sequence.

## Commissioning results

The six-case gravity probe ran on an NVIDIA L40S with Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0 and Inspect Robots 0.60.0. Each reading met the stability and alignment rules for 49 consecutive steps after release. Depth is reported relative to the guide floor.

| Guide | Aligned | Opposed |
| --- | --- | --- |
| Original, missing key | Seated (0.00 mm) | Seated (0.00 mm) |
| Correct spare | Seated (0.00 mm) | Blocked (12.00 mm) |
| Faulty spare, oversized key | Blocked (12.00 mm) | Blocked (12.00 mm) |

The scorer tests also cover a blind swap, a tilted or off-center part, gripper assistance, the spare that blocks both orientations, verification in either order, and clearing verification when the spare is removed.

The scripted YAM run scored **1.0 in 1,453 actions** (72.65 simulated seconds, 227 s wall time). It measured 0.00 mm on the original guide with arrows opposed, 0.00 mm on the replacement with arrows aligned, and 12.00 mm on the same replacement with arrows opposed. The guide stayed installed and the test part returned to its cradle. These are simulator results.

The standard Dream-1 run scored **0.0 in 1,800 actions** (90 simulated seconds, 279 s wall time). It did not reproduce the backward seating fault. Both trials used the same fixed scene and seed 0.
