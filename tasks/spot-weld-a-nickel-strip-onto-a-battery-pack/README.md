# Spot-weld a nickel strip onto a 2x2 pack of 18650 cells with a welder's pen on its lead

> Lay the nickel strip on the battery pack, then spot-weld it to all four cells with the welding pen and put the pen back in its holster.

Cylindrical cells are joined into packs by resistance spot-welding nickel strip onto each terminal. On pack lines and repair benches this is done with a welding pen on heavy leads from a bench power unit. Set to automatic mode, the welder fires as soon as both of the pen's needles are pressed onto the metal at their set force. Joining cells is assembly work done by team assemblers (O*NET 51-2092.00), who rotate through every task of their line; BLS counts them with other miscellaneous assemblers and fabricators, about 1.47 million US jobs in 2024.

**What makes it hard for a robot.** One hand lays a 1 g strip with its bars over the cells. The other takes the pen out of its holster and presses both needles square onto the strip over each + terminal, hard enough to fire (200 g on each needle) but without knocking the loose pack, while the pen's lead drags on it. Then it stands the pen back in its holster.

## Lineage: idea TK-PF8K7B

This builds the idea of TK-PF8K7B (PR #1300, "Lay a nickel strip on a 2x2 18650 battery pack and spot-weld it to all four cells"), submitted by another team, which withdrew it; its hold has lapsed. That review raised no importance concern and asked for two things, both done here:

- **"Model the welder as used in pack assembly: a welding pen (or probes) on heavy cables to a power unit, with the tips pressed at a stated force for each weld."** The welder is a bench power unit with a separate welding pen on a 40 cm lead, 5 mm round and 0.2 kg/m. The lead is a flexible cable that drags on the pen as it moves; its plug is seated in a socket on the unit. The pen's two copper needles are each on their own spring, and a weld fires only when both are pushed in 1.5 mm, which takes 200 g on each. That is the bottom of the 200–700 g needle pressure of bench battery spot welders.
- **"Make the pack able to shift if knocked."** The 201 g pack stands loose on the bench. Moving it more than 1 cm, or tilting it over 10 degrees, scores 0 for the trial.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/spot-weld-a-nickel-strip-onto-a-battery-pack/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the nickel strip on the battery pack, then spot-weld it to all four cells with the welding pen and put the pen back in its holster." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/weld-dream
```

For the scripted full-score demonstration, run the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/weld-demo`. Render videos with `inspect-robots video <log>.json`. Set `WELD_TRACE=<file>` to write the scoring state at every control step: welds, strip offset, pen travel and pen-home measures.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states): `python tasks/spot-weld-a-nickel-strip-onto-a-battery-pack/check_task.py`.

## Runtime and seeds

The scripted demonstration takes about 41 s and reaches 1.0 about 1 s after the pen is back. A Dream-1 trial runs until the finished state has held for 1 s, or until the run limit of 2,400 steps (2 minutes at 20 Hz). There are no seeds: `-E object_jitter=0` keeps every object where `task.py` places it, matching the setup screenshot, and the lead is laid the same way at every reset.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart: forward means away from the arms, left and right as seen from behind the arms.

- **Battery pack** (battery_pack, loose, 201 g): a 2x2 pack of 18 x 65 mm cells (yellow wrap, + up) in a black ABS holder, 41.2 mm square with a 20 mm pitch. It is centred 28 cm forward on the centre line. The cells' + caps stand 0.8 mm proud of the holder.
- **Nickel strip** (nickel_strip, loose, about 1 g): an H of 0.15 mm nickel, made of two 8 x 28 mm bars 20 mm apart joined by an 8 mm bridge. A grip tab is folded up from the bridge (8 mm wide, 14 mm tall, 3 mm across). The strip lies in a 40 x 40 mm grey tray with 5 mm walls (fixed), 24 cm forward and 17 cm left.
- **Welder power unit** (welder_unit, fixed): a bench capacitor spot welder, 14 x 23 x 23 cm, centred 58.5 cm forward and 20 cm right. Its display and knobs are on the front. At bench level on the front is the socket its lead's plug sits in; the socket's slot is narrower than the plug.
- **Welding pen** (weld_pen, loose, 110 g): 13 cm long, with a black grip 24 mm across above a red finger guard 42 mm across. Two copper needles, 2.5 mm across and 4 mm apart, stand 18 mm out of its nose, each on its own spring. The pen stands in a grey holster cup (fixed, rim 6.7 cm high) 17.6 cm forward and 20 cm right, its guard resting on the rim.
- **Its lead** (weld_lead, 40 cm, 5 mm round, 0.2 kg/m): runs from the socket back along the bench, rises beside the holster and enters the pen's side near its top.

Nothing else is on the table.

## Scoring

**Defects** score 0 for the rest of the trial:
- **D1:** the pack's centre moved more than 1 cm, or the pack tilted over 10 degrees.
- **D2:** the pen tilted over 60 degrees (dropped).

**Strip on the pack:** the strip lies flat (under 10 degrees) and rests on the + caps (within 2 mm). Its centre is within 4 mm of the pack's centre, and its bars are within 10 degrees of the pack's axis.

**A weld on cell k:**
- At one control step, both needles are pushed in at least 1.5 mm (200 g each).
- Both needle tips are within 3.5 mm of the cell's axis and on the strip's top face.
- The strip is on the pack.

A welded cell stays welded.

**Pen home:** the pen stands in its holster after it was taken out. Its axis is within 6 mm of the cup's centre at the rim, its guard is within 4 mm of the rim, and it leans under 12 degrees.

**Score:**
- 0.2 for the strip on the pack.
- 0.15 for each cell welded.
- 0.1 for the pen home.
- 1.0 when all of these hold, the strip, pack and pen are still, and both grippers' tool points are at least 4 cm from them, held for 1 s. The trial then ends.

## Building the real setup

- **The welder:** a bench capacitor spot welder with a separate welding pen on leads and an automatic (touch-to-fire) mode, its needle pressure set to the lowest setting (200 g). It sits at the back right of the bench with its lead plugged in at the front. Use a holster cup for the pen: a printed square cup, 33 mm inside and 6.7 cm tall, screwed to the bench.
- **The pack and strip:** a 2x2 18650 holder with four cells at storage charge, and pre-cut H strip of 0.15 mm pure nickel for 20 mm pitch, with its centre tab bent up into a loop. A fresh strip is used for every trial.
- **Real-trial scoring:**
  - A weld counts if the welder fired with both needles on the strip over that cell's terminal. The video shows each press, the welder beeps on firing, and each weld leaves two dark spots.
  - After the run, a weld counts only if the strip stays on that terminal when hooked beside the weld and pulled straight up at 10 N for 2 s.
  - The pack's movement (1 cm) is checked against tape marks.

## The demonstration

Scripted, open loop, fingers pointing down.

- **Left arm:** pinches the strip's tab 7.5 mm up, above the tray walls. It lifts the strip out, lowers it until it is 0.5 mm over the caps, lets go and moves back.
- **Right arm:**
  - Takes the pen just above its guard and lifts it 6.5 cm out of the holster.
  - For each cell: brings the needles to 2 cm over the strip, then 2 mm over it, then presses down, aiming 8 mm below the strip. The arm gives about 3.5 mm under the load, so the needles go in about 2 mm.
  - Holds 0.5 s, lifts 2 cm, and goes to the next cell.
  - Stands the pen back in the holster and moves clear.
- **Offsets:** under the pen and its lead, the arm's drives leave the held pen 6 mm short and 6.5 mm right of where they aim. This was measured in a demo run, and the demo aims past every target by that much.

In the scripted run, cells 1 to 4 were welded at about 18.6, 22.7, 27 and 31 s. The strip stayed within 1 mm of the pack's centre, and the pen was home at 36 s.

## Fidelity

Rigid bodies and joints, 20 Hz control.

- **Needles:** each is a slide joint on a spring (1.31 kN/m), so 1.5 mm of travel is 200 g.
- **Lead:** a chain of 20 capsule links with bending springs, plugged into the unit's socket and tied to the pen with a ball joint. Each reset lays it from the socket to the pen and lets it settle for 1.5 s.
- **Welds:** a weld is the needles' travel and position. No current, heat or bond is simulated.
- **Friction:** set so the strip grips the cells' caps harder than the needles grip the strip, as the first weld holds the strip on a real pack.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `weldkit.py`: the `cablekit.py` of the accepted cable tasks (e.g. `tasks/cable-through-panel-grommet`), extended with `attach_to` (the lead's end tied to the pen with a ball joint kept out of the lead's own joints) and an optional set shape.
- `cableshape.py`: from the accepted task `tasks/trace-cable-to-matching-end`, changed only to import from `weldkit` and to count only the lead's bend joints. It lays the lead at every reset.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter`. `taskbase.py` holds this task's per-trial scoring memory.
