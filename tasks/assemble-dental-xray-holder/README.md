# Finish assembling a dental X-ray sensor holder

**Instruction:** "Finish assembling the X-ray sensor holder: the arm seated in the bite block and the aiming ring down on the grip."

Before each intraoral X-ray, dental assistants (O\*NET 31-9091.00) and dental hygienists (29-1292.00) assemble a
sensor holder, a paralleling instrument like the XCP: a bite block that holds the digital sensor, an indicator arm
whose prongs plug into the bite block, and an aiming ring that slides onto the arm so that the X-ray cone, lined up
with the ring, points straight at the sensor. Here the bite block, with the sensor in it, sits in a tray slot. The
robot has to pick the arm out of its stand, put two 3 mm prongs into 3.2 mm holes and push them fully home, then pick
up the ring by its finger tab, turn it so its sleeve lines up with the arm's 2.5 x 5 mm upright (0.5 mm of play), and
thread it down until it rests on the grip hub. A ring that ends up tilted, or an arm leaning in half-seated prongs,
moves the aiming line off the sensor: in a real radiograph, that is a cone cut and a retake.

A task file for the `yam_arena` embodiment on the station scene:
[assemble_dental_xray_holder.py](assemble_dental_xray_holder.py) holds the scene, the scorer and the closed-loop
scripted demonstration. Helpers: [yam_parts.py](yam_parts.py) (rigid multi-shape parts; unchanged, from the accepted
task `tasks/take-ear-temperature-on-manikin`) and [cabledemo.py](cabledemo.py) (runs closed-loop scripted demos that
read object states; unchanged, from the accepted cable tasks). The release-and-stillness code is the accepted
`tasks/clip-pulse-oximeter-on-patient-finger` block.

## Run a trial

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/assemble-dental-xray-holder/assemble_dental_xray_holder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Finish assembling the X-ray sensor holder: the arm seated in the bite block and the aiming ring down on the grip." \
  --scorer task_score --max-steps 1800 --store-frames --no-prompt --log-dir ~/yam-eval/dental-holder-dream
```

Scripted demo (closed loop: it reads the parts' positions as it goes):

```bash
cd tasks/assemble-dental-xray-holder
python cabledemo.py assemble_dental_xray_holder.py ~/yam-eval/dental-holder-demo --max-steps 1800
```

It reaches 1.0 in 803 steps (44 s) and the episode ends on success. `success()` is a score of 1.0. Runtime per trial
on one L40S: about 3 minutes with cameras. Zero object jitter, so no seed is needed.

## Scoring

| Score | Condition |
| --- | --- |
| 0 | The bite block has tipped more than 5 degrees or moved more than 2 cm, or the sensor is more than 2 mm or 5 degrees out of its pocket position. |
| 0.5 | The indicator arm is seated: its prong tips within 1.5 mm (sideways and in depth) of fully home in the connector's holes, and the arm within 8 degrees of upright. |
| 1.0 | In addition, the aiming ring is fitted: the upright passes through its sleeve (the slot's centre within 3 mm of the upright's axis), the ring rests on the grip hub (its sleeve no more than 4 mm above the hub's top), it is level within 8 degrees, and the line through the ring's centre along its axis meets the sensor's plane within 10 mm of the sensor's centre. |
| 0 | Otherwise (arm not seated). |

Every score counts only once both grippers are at least 3 cm from the arm and the ring and both have stayed still (under
0.5 mm per 50 ms step) for 1 s; while either is held, the score is 0. The 10 mm aiming limit is the margin of a
standard 7 cm round cone over the 30 x 40 mm sensor (35 mm cone radius minus the sensor's 25 mm half-diagonal):
further off, the cone's edge cuts a corner of the image.

**Real-world scoring:** a ruler, a phone level and a plumb line. Arm seated: the prongs' collar is flush with the
connector top (no gap over 1.5 mm) and the upright is within 8 degrees of vertical. Ring: it sits on the hub
(gap under 4 mm), the level reads 8 degrees or less on the ring, and a plumb line hung from a thread across the ring's
centre lands within 10 mm of the sensor's centre mark. Block: within 2 cm of its tray mark, flat.

## Checks

Run in the simulator with this task file (`object_jitter=0`):

| Run | Result |
| --- | --- |
| Scripted demo | 1.0 in 803 steps: prongs 0.2 mm from home, arm 1.8 degrees from upright, ring slot 0.4 mm from the upright's axis, ring tilted 0.8 degrees, aiming line 4.5 mm from the sensor's centre |
| Nothing done | 0: the arm stays in its stand |
| The demo stopped with the prongs half way into the holes, then let go | 0: the arm topples out (86 degrees) |
| The demo stopped once the arm is seated (the ring never picked up) | 0.5 |
| The demo let go of the ring as soon as its sleeve was on the upright's top | 1.0: the ring slides down the upright under its own weight onto the hub; threading it on is the skill, and letting it drop the last few centimetres is a legitimate finish |

`test_scoring.py` checks the rules offline on synthetic states (`python test_scoring.py`): each stage, the ring
slid only part way down, 3 mm above the hub, tilted 10 degrees, turned 180 degrees on the upright, 6 mm off the
upright's axis, the aiming line 11 mm off the sensor, prongs 6 mm and 1 mm high, a leaning arm, a tipped or pushed
bite block and a sensor slid out of its pocket.

## Scene and reset

Positions are from the midpoint between the two arm bases: x forward, y to the left, on the station table (top
74.75 cm). The robot, table and cameras are the standard YAM station.

| Object | Position (cm) | Size and material |
| --- | --- | --- |
| Tray slot (fixed) | around the bite block's paddle | Grey plastic recess, 0.3 mm play around the paddle, 7.5 mm walls 6 mm thick, a 3 mm retaining lip over the paddle's far edge. |
| Bite block (free, 12 g) | paddle centre x 28, y 2, paddle-down; bite plate on the robot's right | Blue plastic: 35 x 45 x 4 mm paddle with a sensor pocket (two 9 mm rails with 2 mm lips); 4 x 20 x 45 mm bite plate; 12 x 20 x 20 mm connector on top with two 3.2 mm holes 8 mm apart and an 8 x 16 mm, 3 mm deep lead-in. |
| X-ray sensor (free, 20 g) | in the pocket, against the bite plate | Black, 30 x 40 x 6 mm. |
| Arm stand (fixed) | centre x 20, y -16 | Grey plastic, 30 x 24 x 30 mm, two 3.2 mm holes 18 mm deep. |
| Indicator arm (free, 20 g) | prongs in the stand | Stainless flat bar 5 x 2.5 mm: two 3 x 18 mm prongs 8 mm apart, a 31 mm crossbar, an 85 mm upright; grey plastic grip hub 10 x 14 x 20 mm, centre 50 mm up the upright. |
| Aiming ring (free, 8 g) | centre x 37, y -10, flat; the lug pointing back and to the right (yaw -145 degrees) | Yellow plastic annulus, 46 mm inside, 60 mm outside, 5 mm thick; a lug with a 22 mm-tall sleeve (3.5 x 6.0 mm slot, centre 48 mm from the ring's centre) and a 6 mm-thick, 12 mm-tall finger tab out to 90 mm. |

Reset: stand the arm back in its stand, lay the ring at its mark, seat the bite block (sensor in) in the tray slot.
Safety: no X-ray source is involved; the parts are light plastic and steel.

## Fidelity

Rigid parts; moulded plastic parts are built from boxes and the ring is a 24-sided polygon. The prongs slip into
3.2 mm holes (0.1 mm a side) with a lead-in where a real connector is a light press fit (rigid bodies cannot press-fit). The bite block sits in a tray slot (a person
would hold it in their fingers), and the sensor starts in its pocket. The grip hub stands in for where the ring would
come to rest near the patient's face. The X-ray beam is not simulated: aiming is the geometric line through the ring's
centre.

## Real-world parts

An obtainable Dentsply Rinn XCP-DS FIT kit (measure its prongs, holes and insertion force first) or a printable equivalent with a defined 2.95 mm press-fit hole for 3 mm steel dowel pins is described in [holder_build/BUILD.md](holder_build/BUILD.md), with STL files for the bite block, aiming ring and grip hub and a generator, `holder_build/make_parts.py`. The simulation uses a 3.2 mm hole because rigid bodies cannot press-fit; its press-fit force is not modelled.
