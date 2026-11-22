# Unload a keyed shaft

Remove the orange service key and leave the shaft at its resting mark.

A loaded output vane can rebound across the service opening when the retaining key clears. A return to the resting mark can hide the excursion.

At x=335 mm, y=0 on the tabletop, fix the 245 x 215 mm bed, with its 92 mm blue friction-braked input disc under a 60 mm white output hub. The vane reaches 60 mm from the shaft axis. Insert the orange steel-key surrogate with 17 mm overlap. Set the two hub indices to zero with 36 degrees of spring preload. The USD and calibration sheet specify the solid dimensions, materials represented and moving masses.

Score 1 for a key parked at least 40 mm out, with both the output angle and residual twist within 3 degrees and no robot contact on the fixture for 0.5 s. Excessive sweep, loaded disengagement, fixture displacement, inter-arm collision or a non-hand link striking the fixture latches failure. Other outcomes score 0.

See PHYSICAL_CALIBRATION.md for component dimensions, material parameters and limitations.

From this directory with IsaacLab-Arena-YAM activated, run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=5 -E task=native/entry.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Remove the orange service key and leave the shaft at its resting mark." --scorer task_score --max-steps 440 --seed 0 --no-prompt --store-frames --log-dir logs/yam_scripted
inspect-robots run --embodiment yam_arena --policy dream -E task=native/entry.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Remove the orange service key and leave the shaft at its resting mark." --scorer task_score --max-steps 440 --seed 0 --no-prompt --store-frames --log-dir logs/dream
```

## Recorded runtimes

Seed 0, zero object jitter, 240 Hz physics and 20 Hz control. Each trial allows 440 actions (22 s simulated). The episode holds a successful result for 3.6 s where the remaining budget permits.

| Policy or control | Score | Actions | Inspect wall time (s) |
|---|---:|---:|---:|
| idle | 0 | 137 | 28.4 |
| loaded_pull | 0 | 282 | 45.2 |
| reference | 1 | 423 | 62.7 |
| alternate | 1 | 402 | 60.6 |
| unload_only | 0 | 361 | 54.3 |
| dream | 0 | 440 | 81.2 |

The default scripted route is the reference. Set SHAFT_CONTROL to alternate, idle, unload_only or loaded_pull for a scripted control. Leave it unset for Dream. See PHYSICAL_CALIBRATION.md and optical_score.py for independent physical scoring and measurement limits.
