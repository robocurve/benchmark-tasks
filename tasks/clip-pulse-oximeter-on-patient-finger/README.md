# Clip a fingertip pulse oximeter onto a patient's index finger

**Instruction:** "Put the pulse oximeter all the way onto the manikin hand's index finger without dragging the hand."

A fingertip pulse oximeter is a spring clip. To fit it, you squeeze its back end so the jaws open, slide the
patient's fingertip in until it reaches the finger stop, and let go so the spring clamps the jaws onto the
finger. Nursing assistants (O\*NET 31-1131.00) and medical assistants (31-9092.00) do this at every
vital-signs round. Here the patient's hand is a light, loose task-trainer hand on a foam arm-elevation wedge,
and it slides down the slope if the clip pushes it. So one gripper squeezes and slides the clip while the
other steadies the hand.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=clip_pulse_oximeter_on_patient_finger.py \
    --instruction "Put the pulse oximeter all the way onto the manikin hand's index finger without dragging the hand." \
    --policy yam_scripted --scorer task_score --max-steps 1200
```

- Scene: `station`. Control at 20 Hz. Run limit: 1,200 steps (60 s). The scripted demo takes 28.1 s, and a
  trial takes about 3 to 5 minutes on one GPU.
- No object jitter. The hand, wedge and oximeter start in one fixed layout, and the seed changes nothing.
- `--scorer task_score` gives 1.0 on success and otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_clip_pulse_oximeter_on_patient_finger.py` (generated scorer tests: start,
  goal, partial and cheat layouts).

## Scoring

The scorer reads only the oximeter (its pose and top-jaw angle) and the hand's pose. The release check reads
the robot's joint state to find each gripper's centre point.

| Rule | Value |
|---|---|
| Sticky zero | The hand is ever dragged more than 25 mm (in plan view), lifted more than 15 mm or turned more than 15 deg from its start. Or the oximeter ever drops below the table top. |
| On the finger (required for any credit) | The index fingertip apex is at least 15 mm inside the mouth. The fingertip and a point 20 mm back along the finger both lie within 12 mm of the clip's centre line, at 12 to 24 mm above the bottom face, with the finger pointing in. The top jaw is between -10 and +6 deg. The display normal is within 45 deg of vertical. The clip's lowest point is at least 5 mm above the table. |
| Released and at rest | Each gripper centre is at least 30 mm from the bounding boxes of both the oximeter and the hand. Between 50 ms steps, the oximeter's end points and the hand move less than 0.5 mm and the jaw turns less than 0.25 deg. All of this holds for 1 s. |
| Tiers | On the finger = 0.4. Seated (fingertip within 5 mm of the stop) adds 0.3. Hand on its mark (within 10 mm and 5 deg) adds 0.3. |
| Success | 1.0, held for 1 s (`SUCCESS_HOLD_S`). |

The following all score 0: a clip on another finger, a clip with the display facing down, a clip still held,
a clip shut beside the finger, a clip less than 15 mm on, a clip still moving, and doing nothing.

**Measuring it in a real trial:**
1. Trace the hand's outline on the wedge before the trial.
2. After the robot lets go, take a side photo against a ruler. From it, read the fingertip-to-stop gap through
   the open side of the clip, and the hand's shift from its outline.
3. The photo also shows whether the top jaw rests on the finger and which side the display faces.

## Layout and dimensions (metres, world frame; arm bases at (0, +/-0.305, 0.750); table top 0.7475)

| Object | Type | Material | Size | Mass | Pose |
|---|---|---|---|---|---|
| Arm-elevation wedge | fixed convex prism | firm PU foam, vinyl cover | 250 mm wide (x 0.085..0.335); slope 20 deg rising toward -y; 5 mm thick at the low end (y 0.141), 66.6 mm at the high edge (y -0.028), with a vertical face below the edge; 180 mm along the slope | (fixed) | origin (0.210, 0.141, 0.7475) |
| Trainer hand (right, palm down) | free rigid body, 160 spheres | silicone | see the hand table | 0.300 kg (silicone skin over a foam core) | frame origin (0.270, -0.075, 0.8312), on the slope plane under the index fingertip; frame turned -20 deg about x |
| Fingertip pulse oximeter | free articulation: lower body plus a hinged top jaw | ABS shell, silicone pads | 58 x 32 x 37 mm | 0.050 kg (lower body 0.032 including 2 AAA cells, top jaw 0.018) | origin (0.130, -0.0243, 0.8127), yaw -90 deg, pitched 20 deg with the slope, display up |

**Hand** (hand frame: x right; y along the fingers toward the wrist, 0 at the index fingertip; z normal to the
wedge surface). The sizes are ANSUR II median adult male (hand length 193 mm, breadth 88 mm).

| Part | Geometry |
|---|---|
| Index finger | tip (0, 0), knuckle (0, 0.095). Oval cross-section from two rows of spheres: distal 18 mm wide x 15 mm thick, middle 19 x 16, proximal 21 x 18. The distal joint is 26.6 mm from the tip and the middle joint 55.1 mm. |
| Middle finger | tip (-0.036, -0.010) to knuckle (-0.024, 0.093). Radii 7.8 / 8.3 / 9.3 mm. |
| Ring finger | tip (-0.068, -0.002) to knuckle (-0.045, 0.097). Radii 7.3 / 7.8 / 8.6 mm. |
| Little finger | tip (-0.098, 0.022) to knuckle (-0.064, 0.102). Radii 6.5 / 6.8 / 7.6 mm. |
| Thumb | from (0.042, 0.058), radius 9.5 mm, to (0.016, 0.165), radius 12 mm. |
| Palm | 5 x 7 spheres of radius 13 mm over x -0.060..0, y 0.097..0.176, so 86 mm across and 26 mm thick. Thenar sphere of radius 15 mm at (0.010, 0.150). |
| Forearm stub | 2 x 6 spheres of radius 20 mm at x -0.045 and -0.025, y 0.183..0.225: 60 mm wide, 40 mm thick. |

The index, middle and ring tips reach 50 mm past the wedge's high edge. Friction (static/dynamic) is 0.5/0.4 on
the hand and 0.6/0.5 on the wedge.

**Oximeter** (clip frame K: x toward the back end, mouth at -x; z out of the display; origin on the bottom face
in the hinge plane):

| Part | K extent (mm) |
|---|---|
| Lower shell (grey) | x -38..+20, y +/-16, z 0..10 |
| Finger trough (silicone) | x -38..-4; flat at z 10 for \|y\| < 2, sides rising at 20 deg to z 15.1 at \|y\| = 16 |
| Finger stop | x -4..0, y +/-12, z 10..22 |
| Hinge cheeks | x -4..+4, \|y\| 13..16, z 10..27 |
| Top jaw shell (blue) | x -38..+20, y +/-16, z 27..35.5 |
| Squeeze ridge | x +12..+20, y +/-16, z 35.5..37 |
| Display window | x -32..-4, y +/-11, z 35.5..36.5 |

| Mechanism | Value |
|---|---|
| Hinge | axis +y through (0, 0, 27). Positive angles open the mouth. |
| Stops | -20 deg closed (4 mm gap at the mouth) and +25 deg open (the back ends meet; the back end is then 31 mm thick, measured ridge corner to underside). |
| Torsion spring | 0.104 N m/rad, rest -45 deg (preloaded). It holds the clip shut with 0.045 N m, clamps a 16 mm finger with about 2.3 N, and needs about 8.4 N on the ridge to open fully. |
| Damping, armature | 2e-4 N m s/rad, 1e-5 kg m^2 |
| Friction | 1.0 / 0.9 |

Calibrating the spring on a real unit:
1. Hold the lower body in a vise.
2. Hang a spring scale on the top jaw 34 mm in front of the hinge.
3. Read the force with the jaws resting on a 16 mm dowel (target 2.3 N).
4. Check the squeeze needed at the ridge to reach the open stop (target about 8 N).

## Scripted demo (`demo()`, open loop, start layout and time only)

**Right gripper.** The wrist is tilted 20 deg, so the gripper points down the slope and the pads close
square to the clip.
1. Come in behind the clip's back end.
2. Close the pads 4 mm past contact (about 20 N per pad) on the squeeze ridge and the underside. This holds
   the jaws at their open stop.
3. Lift 6 mm, draw back up the slope, and swing across to line up with the index finger.
4. Slide the clip down the finger axis with the bottom jaw 2.5 mm below the finger. Stop at the finger stop, with
   no push past it.
5. Open the gripper so the jaws clamp the finger, then back away along the finger axis and rise.

**Left gripper.** Straight down, pads closing across the forearm stub: 60 mm wide, commanded to 52 mm. It
holds the stub from before the insertion until the right gripper has backed away, then opens and rises.

Both arms then hold still for 3.5 s.

Offline checks: IK error is at most 0.1 mm, planar reach at most 0.343 m, and the gripper body stays at least
65 mm above the table.

Offline-checker notes:
- The offline replay models every grasp as straight down and cannot open the clip's jaws. So
  `PRESSED_PARTS = ("oximeter", "hand")` turns its finger-contact findings into advisories, and the top jaw's
  offline proxy is a single sphere inside its shell.
- The wedge's offline proxy is an inscribed staircase 7 mm under the slope, because the checker bounds each
  sphere by an axis-aligned box.
- None of this changes the simulated colliders: every mesh is a convex collider on the GPU.

## Real-world parts list

- Fingertip pulse oximeter: any CE/FDA-cleared clip-type unit of about 58 x 32 x 34-37 mm and 50 g with 2 x
  AAA cells (for example the ChoiceMMed MD300C series: 58 x 32 x 34 mm and 50 g for the MD300C19, 58.3 x 33.2
  x 37.0 mm for the MD300C15D).
- Adult right-hand task trainer, silicone, life size (193 mm wrist crease to middle fingertip), about 0.3 kg,
  with a short forearm stub.
- Arm-elevation wedge: firm polyurethane foam (about 35 kg/m^3) in a vinyl cover, cut to 250 mm wide with a
  20 deg slope, 5 mm to 67 mm high over 180 mm, taped to the table.
- Safety: a trainer hand only, with no person in the cell. The oximeter is harmless. Use the robot's usual
  guarding and e-stop.

## Files and licences

- `clip_pulse_oximeter_on_patient_finger.py`: the task (scene, scorer, demo).
- `yam_parts.py`, `yam_joints.py`, `yam_mesh.py`: our own primitive, joint and mesh kits, unmodified copies.
- `task.json`, `README.md`, and `test_clip_pulse_oximeter_on_patient_finger.py` (generated scorer tests).
- Every shape is a simulator primitive (sphere, box) or a convex mesh generated in code. There are no
  third-party assets, textures or fonts.
