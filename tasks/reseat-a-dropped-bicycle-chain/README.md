# Reseat a dropped bicycle chain

When the front chain drops inboard, pedaling can turn the crank without driving the rear sprocket. Lift the loose span, feed the rollers onto the chainring, withdraw the guide and demonstrate at least two crank turns under rear drag.

## Submitted configuration

Use the `deep_near_light` case and the lift pick. Compared with the reference bench dimensions below, the catcher **radius is 42.46 mm**, the reset offset is **11 mm**, and rear bearing drag is **1 mN·m**. The left arm sits on a 25 mm riser. Its pick has a 16 × 18 × 3.2 mm shelf and a 2 mm wide retaining lip. In the settled simulation the lowest pin is 96.7 mm below the front axle. This is a proposed physical build, validated in simulation only.

## Run and verify

Use Python 3.12, MuJoCo 3.3.7, the Dream Labs dual-YAM assets and FFmpeg with libx264. The scripted run takes 770 robot commands, or 38.5 simulated seconds. The limit is 1,200 actions at 20 Hz. The controller uses simulated roller positions to aim the pick.

```sh
python -m pip install -r requirements.txt -c constraints-tested.txt
MUJOCO_GL=egl python run.py chain --case deep_near_light --tool pick --show-fault --video --out runs/deep
python run.py chain --case deep_near_light --tool pick --replay runs/deep/commands.json --out runs/replay
python verify.py --out runs/sensitivity --jobs 4
MUJOCO_GL=egl python evaluate.py chain --case deep_near_light --tool pick --video --out runs/dream
```

The scripted run writes the setup image, video, command sequence and result. The scorer recorded 2.9148 front and 5.0961 rear revolutions after the guide cleared the chain. All 54 rollers visited front pockets. The saved run ended with 17 front and 10 rear pockets occupied. An independent replay matched the sampled scoring trace. The retained results are in `evidence/`.

## Reference bench and physical reset

This is a proposed bench build with a simulated demonstration. Physical tolerances, friction and robot feasibility have not been verified on hardware.

| Part | Nominal specification |
| --- | --- |
| Chain | Clean single-speed roller chain, 54 pitches in a closed loop (27 inner/outer pairs), 12.70 mm pitch, 7.70 mm roller diameter, 3.20 mm clear inner width. |
| Sprockets | 28 teeth front, 16 teeth rear. Pitch diameters 113.43 and 65.10 mm. Tooth thickness 2.60 mm at the centerline. Use a fixed rear sprocket on its own bearing shaft, with approximately 0.002 N·m rear bearing drag. |
| Frame | Mount parallel horizontal shafts 201.76 mm apart, 112 mm above the base top. Align both tooth center planes within 0.5 mm using a straightedge. Use a 316 × 112 × 16 mm bolted base and slotted bearing mounts to trim center distance. |
| Drop catcher | Concentric stationary smooth sleeve on the inboard side of the front sprocket. Outer diameter 104.93 mm, width 18 mm, center 14 mm from the tooth plane. Rear guide discs are centered 6 mm to either side of the rear teeth. |
| Crank and tools | Counterbalanced 58 mm bench crank with a 16 mm diameter, 112 mm long spindle. The turning arm carries a socket with a 19 mm square opening. The guiding arm carries a flat 28 mm wide replaceable pad. Use a clean chain and cover the gripper pads. |
| Index marks | Mark the tooth pocket at 12 o’clock on both sprockets. Paint one roller and mark one radial line on each hub. Use the marks or encoders to count independent axle turns. |

1. Level and secure the base. Leave the drivetrain unloaded except for the rear shaft drag. Align both marked pockets at 12 o’clock and the crank horizontally away from the rear sprocket.
2. Thread the rear half-wrap between its guide discs. Put the front half-wrap on the smooth sleeve using a removable spacing gauge for the chosen case (11 mm for the submitted case). Place the marked front roller at 12 o’clock. Route the top and bottom spans without a twist.
3. Remove the gauge. Check that the rear rollers sit in the rear pockets, the front rollers clear the teeth, and both tools start clear. Reject the reset if the chain falls off or seats itself at rest.
4. Film approximately along the shaft axis so both index marks, the upper span and both teeth are visible. Use an oblique second view to check the lateral offset and guide withdrawal. On hardware, verify that the crank drives the rear against its drag for the full two turns after the guiding hand clears.

## Simulation and score

Each roller moves in three dimensions. Neighboring pins have pitch constraints, limited bending and passive lateral stiffness. The rollers and plate ends collide with individual teeth. Roller axes stay parallel to the shafts, the plates are simplified, and the sprocket roots use solid collision discs. The pedal socket and chain pad transmit force by contact. No actuator, target angle or kinematic coupling drives either sprocket.

At the 20 Hz control boundaries, the scorer measures each roller’s axial offset, radial position and phase within a tooth pocket. A pocket is counted once. The proof requires a dropped start, a guide at least 10 mm clear, and continuous engagement of at least four front and three rear pockets. Losing that engagement or slipping more than 0.040 rear turns from the 28:16 ratio restarts the count.

| Score | Requirement |
| --- | --- |
| 0.25 | At least four front and three rear pockets contain aligned rollers after a dropped start. |
| 0.50 | At least a quarter crank turn with the guide withdrawn and the drive ratio maintained. |
| 1.00 | Two complete front turns and at least 3.5 rear turns in the same uninterrupted interval. All 54 roller IDs must have visited a front pocket, at least eight front and five rear pockets must remain occupied, and the pedal socket must clear the spindle. |

## Measured sensitivity

Scores are terminal results. Each row uses the same reset and brake setting for the two tools. The lift pick includes its riser and lift routine. Link positions, tools, and both axle angles are measured from the simulator.

| Lateral offset (mm) | Catcher radius reduction (mm) | Rear drag (mN·m) | Flat pad | Lift pick |
| ---: | ---: | ---: | ---: | ---: |
| 13 | 0 | 2 | 1.00 | 0.00 |
| 11 | 0 | 2 | 1.00 | 0.00 |
| 15 | 0 | 2 | 0.00 | 0.00 |
| 13 | 0 | 1 | 1.00 | 0.00 |
| 13 | 0 | 3 | 1.00 | 0.00 |
| 13 | 2 | 2 | 1.00 | 1.00 |
| 13 | 5 | 2 | 0.00 | 1.00 |
| 13 | 10 | 2 | 0.00 | 1.00 |
| 11 | 10 | 1 | 0.00 | 1.00 |
| 15 | 10 | 3 | 0.00 | 0.00 |

All four idle and crank-only control trials scored zero. The filmed deep repair and fresh command replay agree at every recorded sample. Raw results for all 24 trials, including unsuccessful repairs, are in `evidence/sensitivity.json`.

