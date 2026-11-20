# Fit a mallet-finger splint onto a drooping fingertip

**Instruction:** "Fit the splint fully onto the drooping tip of the trainer hand's index finger so that it holds the fingertip straight, without bending the fingertip further down or dragging the hand."

In a mallet finger the tendon that straightens the fingertip is torn, so the last joint (DIP) droops and cannot be
lifted by the patient. Treatment is a splint that keeps the fingertip straight day and night for 6 to 8 weeks; it
is refitted by the patient or a carer, and the rule is that the fingertip must never be allowed to bend while doing
so, because a single bend restarts the healing. A Stack-type splint is a small moulded shell, closed at the
fingertip end: the fingertip goes in, the hood covers the nail and the joint, and the floor runs on under the
middle segment so that the fingertip is held level with it.

Here the patient's hand lies on an arm-elevation wedge with the fingertips past its edge, the index fingertip
drooping 40 degrees. A splint aimed straight along the finger would hit the drooping fingertip and bend it further
(scored 0), and the floor's extension stops the splint from going on fully while the tip droops. So the robot has
to slide the splint onto the fingertip at the fingertip's own angle, swing the splint (and the fingertip in it) up
about the joint until the finger is straight, slide it home against the fingertip, and let go, while the other hand
steadies the forearm so the light hand is not pushed down the wedge.

A task file for the `yam_arena` embodiment on the station scene:
[slide_mallet_finger_splint.py](slide_mallet_finger_splint.py) holds the scene, the scorer and the closed-loop
scripted demonstration. Helper kits, unchanged, from the accepted task `tasks/clip-pulse-oximeter-on-patient-finger`:
[yam_parts.py](yam_parts.py), [yam_joints.py](yam_joints.py) (the hinged fingertip), [yam_mesh.py](yam_mesh.py)
(the wedge); the hand, wedge, steadying grip and release-and-stillness code follow that task.
[cabledemo.py](cabledemo.py) (unchanged, from the accepted cable tasks) runs closed-loop demos that read object
states.

## Run a trial

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/slide-mallet-finger-splint/slide_mallet_finger_splint.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the splint fully onto the drooping tip of the trainer hand's index finger so that it holds the fingertip straight, without bending the fingertip further down or dragging the hand." \
  --scorer task_score --max-steps 2400 --store-frames --no-prompt --log-dir ~/yam-eval/mallet-splint-dream
```

Scripted demo (closed loop):

```bash
cd tasks/slide-mallet-finger-splint
python cabledemo.py slide_mallet_finger_splint.py ~/yam-eval/mallet-splint-demo --max-steps 2400
```

It reaches 1.0 in 549 steps (27 s) and the episode ends on success; `success()` is a score of 1.0. Runtime per
trial on one L40S: about 2 minutes with cameras. Zero object jitter, so no seed is needed.

## Scoring

| Score | Condition |
| --- | --- |
| 0 (sticky, whole trial) | The injured joint is ever bent more than 10 degrees past its starting droop (beyond 50 degrees); or the hand is ever dragged more than 25 mm, lifted more than 15 mm or turned more than 15 degrees; or the splint falls off the table. |
| 0.3 | The splint is on the fingertip: the fingertip's apex no more than 20 mm from the closed end (at least 11 mm under the hood), the fingertip's axis within 5 mm of the splint's centre line, the hood on the nail side (within 30 degrees). |
| 0.6 | In addition, seated: the apex within 4 mm of the closed end. |
| 1.0 | In addition, the joint held straight (within 8 degrees of full extension) and the hand on its mark (within 10 mm and 5 degrees of where it lay). |
| 0 | Otherwise: no splint on the index fingertip, a splint on another finger or upside down, or less than the 0.3 condition. |

Every credit counts only once both grippers are at least 30 mm from the splint and the hand, and the splint and hand
have stayed still (under 0.5 mm per 50 ms step) for 1 s; while either is held, the score is 0.

**Real-world scoring:** a ruler, a protractor and a photo from the side. Mark the hand's outline on the wedge. After
the robot lets go: the gap between the fingertip and the splint's closed end (seen through the shell, or by
marking the fingertip's position on the shell), the joint angle between the middle segment and the splint's line
(protractor on the side photo), and the hand's shift from its outline. During the trial, a side video shows
whether the fingertip ever bent further than its starting droop (protractor on the frame).

## Checks

Run in the simulator with this task file (`object_jitter=0`):

| Run | Result |
| --- | --- |
| Scripted demo | 1.0 in 549 steps: apex 1.8 mm from the closed end, joint 7.6 degrees from straight, hand 5.7 mm from its mark |
| Nothing done | 0 |
| Splint slid onto the drooping fingertip (apex 21 mm from the closed end), then let go without straightening | 0: the splint slips off the drooping tip |
| Swung straight with the fingertip in it, but let go before sliding it home | 0: the splint falls off |
| Slid home only part way (apex 6.3 mm from the closed end), then let go | 0.3: on the fingertip but not seated; the fingertip sags to 13 degrees |
| Splint aimed straight along the finger while the fingertip still droops (no swing) | 0: it shoves the fingertip and hand and ends off the fingertip |

`test_scoring.py` (run where yam_arena is installed) checks the rules on synthetic layouts: the start, seated and
straight, seated at 7 degrees, seated but bent 15 degrees, 10 mm short, only the tip in, upside down, on the middle
finger, the hand 15 mm off its mark, the hand dragged 30 mm, the joint forced to 55 degrees, a dropped splint, and a
splint still held.

## Scene and reset

Positions are from the midpoint between the two arm bases: x forward, y to the left, on the station table (top
74.75 cm). The robot, table and cameras are the standard YAM station.

| Object | Position (cm) | Size and material |
| --- | --- | --- |
| Arm-elevation wedge (fixed) | x 8.5 to 33.5; low end y 14.1, high edge y -2.8 | Firm foam, vinyl covered; 25 cm wide, sloping 20 degrees up toward -y, 0.5 cm to 6.66 cm thick, 18 cm along the slope. |
| Task-trainer right hand (free, 298 g) | palm down on the slope, fingers up the slope, fingertips 5 cm past the high edge; frame origin (27.0, -7.5, 83.1) under the straight index fingertip | Silicone skin over foam: 86 mm across the knuckles, 26 mm thick palm, index 95 mm (distal segment 18 x 15 mm, middle 19 x 16, proximal 21 x 18), thumb beside the index, 60 x 40 mm forearm stub. The index fingertip is hinged at the DIP (26.6 mm from the tip): droops 40 degrees at rest, moves from 5 degrees past straight to 70 degrees bent, weak spring (0.49 N at the fingertip apex holds it straight). |
| Mallet-finger splint (free, 6 g) | on the table at (30.0, -20.0), hood up, open end toward the hand | Stack-type, light-blue moulded plastic 1.5 mm thick: 23.5 x 20 x 47.5 mm outside, 20.5 x 17 mm inside, closed at the fingertip end; hood over the first 31 mm; then 15 mm of floor with 8.5 mm side walls. |

Friction (static/dynamic): wedge 0.6/0.5, hand 0.5/0.4, splint 0.35/0.3.
Reset: lay the hand back on its outline with the fingertip drooping, the splint back at its mark.
Safety: a trainer hand, a 6 g plastic splint; nothing sharp, hot or heavy.

## Fidelity

The hand is rigid spheres (the accepted oximeter task's trainer hand) except for the hinged index fingertip with a
weak passive spring. The splint is built from flat plates where a real one is a rounded shell, and has 1 mm of play
around the fingertip, so a seated fingertip rests a few degrees from fully straight (7.6 degrees in the demo, within
the 8 degree limit). The tape that would secure the splint is not modelled. The hand rests on a wedge with the
fingertips past its edge (so that the fingertip can droop and the gripper can reach around it); a carer would hold
the patient's hand on a table edge the same way.

## Reproducing the trainer hand

The one custom mechanism is the hinged index fingertip. A reference build (printable STL files for the fingertip, the finger stub and the splint, a spring specification, assembly steps, and how to set and check the 40 degree droop and the 0.49 N spring force) is in [trainer_build/BUILD.md](trainer_build/BUILD.md); `trainer_build/make_parts.py` regenerates the parts and runs a collision sweep of the hinge. The simulation, the scorer and the scripted demo are unchanged by this documentation.
