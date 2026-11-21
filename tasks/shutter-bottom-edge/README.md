# Restore a shutter's contact edge

Restore the shutter's contact edge and use the orange proof block to demonstrate reversal at both white marks.

## Setup and reset

All components are original primitive geometry in native/shutter.usda. Dimensions below are millimetres. The frame origin is at (335, 0) on the standard YAM station tabletop. +Z points up. +Y is the left mark. The two arms start in the normal ready pose. Recreate the fixture on every reset, with zero object jitter. There are no loose consumables.

| Component | Material represented, dimensions, placement |
|---|---|
| Frame and guides | Aluminium plate 195 × 262 × 10 at (−25,0,5), uprights 20 × 12 × 186 at y = ±119. The two white lines on the base indicate y = ±66. |
| Shutter | Blue aluminium panel 16 × 222 × 47. Its carrier starts 150 mm above the frame origin. The panel centre is 26 mm above the carrier. |
| Two switch shoes | Green elastomer-faced plungers, 15 × 110.8 × 12, along the span. Unloaded plungers sit 14 mm below the carrier at y = ±66. The faces extend inward to a 0.2 mm centre seam. Both have 9 mm upward travel, 300 N/m stiffness, 1.2 N s/m damping. The right shoe rides a sliding cassette with an orange 18 × 20 × 27 handle. It starts offset 48 mm behind the working plane. |
| Proof carriage and witness | A 24 mm orange cube on a 140 × 9 × 6 mm neck is integral to the proof cap. Cube centre x = −161 mm. The guide bushing has 16 mm upward pickup travel. A separate spring returns the 44 × 44 × 10 mm cap after compression, with stiffness 200 N/m and 22 mm downward travel. Lower the bushing to its seat before operating the shutter. |
| Cycle control | The 29 × 27 × 12 mm button is centred at (−155,+210,42) on its pedestal. Five millimetres of button travel starts a cycle. |

## How to judge the trial

The cassette must end within 3 mm of its seat. At each mark (±6 mm), witness actual contact, switch compression of 2 mm, at least 50 mm of descent before reversal, at least 65 mm of reopening and return to within 0.5 mm of open. Count distinct sites. One proved site with the cassette seated earns 0.25. Both sites plus 0.5 s of continuous hands-clear, open, button-released settling earn 1. Otherwise the score is zero, except for earned partial credit. A 15 mm witness excursion, off-guide motion, false triggering, moving the block out of its registered band or manual assistance with the working parts during a cycle latches failure. Pressing the remote start button is allowed. The same public evaluator sees every policy's rigid transforms and contact pairs at each physics step. The policy receives the usual station cameras and joint state.

For a tabletop measurement, use a marked spring cap with a peak-travel collar, white registration lines and side video showing the two toe plungers and travel scale. Quantitative sensor values and time windows are specific to this miniature fixture. Use only a guarded, limited-energy tabletop trainer and a captive proof block for physical replication. No person is an obstruction.

## Reproduce

Activate IsaacLab-Arena-YAM with the Inspect Robots CLI, YAM scripted policy and configured remote Dream SDK. From this task directory run either command. Each starts a fresh process.


```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=native/entry.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 960 --seed 0 --no-prompt --store-frames --log-dir logs/reference --instruction "Restore the shutter's contact edge and use the orange proof block to demonstrate reversal at both white marks."
inspect-robots run --embodiment yam_arena --policy dream -E task=native/entry.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 960 --seed 0 --no-prompt --store-frames --log-dir logs/dream --instruction "Restore the shutter's contact edge and use the orange proof block to demonstrate reversal at both white marks."
```

Read results.metrics.task_score in the official adhoc JSON.

The accompanying runs took 142.6 s scripted and 146.2 s with Dream-1, measured by Inspect.
