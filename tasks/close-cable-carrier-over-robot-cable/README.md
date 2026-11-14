# Lay a robot axis's two cables into its cable carrier and snap every crossbar shut

**Instruction:** "Lay both cables side by side into the cable carrier, grey on the far side, without crossing, and
snap every crossbar shut."

A cable carrier (energy chain) guides the motor and encoder cables of every linear axis, gantry and robot track.
Filling one is a manual step of building the axis: the chain is laid out straight, each link's crossbar is swung
open, the cables are laid in side by side in a fixed order, and each crossbar is swung back and pressed until its
catch snaps. Here the carrier is a real igus E2 mini B15 (25 mm inner width, 12 links, 366 mm, which suits a
linear axis of about 400 mm stroke: S/2 + pi R + 2 pitches = 200 + 119 + 61 mm). Two cables are tie-wrapped to a
strain-relief plate behind the fixed end and lie loose on the table beside the carrier. Each arm can reach only its
half of the carrier, so both arms close crossbars.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=close_cable_carrier_over_robot_cable.py \
    --instruction "Lay both cables side by side into the cable carrier, grey on the far side, without crossing, and snap every crossbar shut." \
    --policy yam_scripted --scorer task_score --max-steps 3600
```

- Scene: `station`. Control at 20 Hz. Run limit: 3,600 steps (180 s). The scripted demo plan is 67.2 s and
  reaches success at step 1,274 (64 s). A demo trial takes about 9-22 minutes on a shared GPU (0.4-1 s per step).
- No object jitter: the carrier and cables start in one fixed layout, and the seed changes nothing.
- `--scorer task_score` gives 1.0 on success and otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_close_cable_carrier_over_robot_cable.py` (generated scorer tests: start, goal,
  partial and cheat layouts).

## Scoring

The scorer reads the carrier's crossbar and catch joints and the cables' segment poses. The release check reads
the robot's joint state to find each gripper's centre point.

| Rule | Value |
|---|---|
| Link k covered | Over the middle 24 mm of link k, both cables are inside the channel: each cable's centre is within its own radius + 1.5 mm of the inner side plates, below the crossbar line, and down on the bottom crossbars (within 2 mm). And the crossbar is latched: within 2.5 deg of shut (free end at most 1.4 mm proud) with its catch back within 0.5 mm of rest, over the bar. |
| Per covered link | 0.05 (12 links: up to 0.6) |
| Cables in order | + 0.2 if both cables are inside all twelve links and, at every 5 mm along the carrier, grey's centre is at least 1 mm farther from the robot than orange's (not crossed). |
| Finished | + 0.2 if all twelve links are covered and both cable ends come out at least 60 mm past the moving end. |
| Released and at rest | Required for any credit: each gripper's centre point at least 30 mm from the bounding boxes of the carrier and of both cables; cable points moving under 0.5 mm and crossbars under 0.25 deg per 50 ms step; held for 1 s. |
| Permanent 0 | Either cable is ever pulled more than 5 mm through its tie-wrap (the strain relief is torn). Sticky for the rest of the trial. |
| Success | 1.0, held for 1 s (`SUCCESS_HOLD_S`). |

These score 0: doing nothing; crossbars shut over an empty carrier; only one cable laid; a cable still held;
a cable pulled through its tie-wrap. A crossbar resting on its catch (4.1 mm proud) or shut on a cable that rides
a side plate is not latched.

**Measuring it in a real trial:**
1. Mark each cable with tape at its tie-wrap before the trial; any slip shows against the tie.
2. After the robot lets go, lay a straightedge across each crossbar's two side plates. A latched bar is flush
   (with 0.8 mm of play); an unlatched one rests on its catch 4.1 mm proud. Then lift each bar's free end with a
   fingernail: a latched bar does not come up.
3. Take a top photo. Between the 10 mm crossbars there are 20 mm gaps, so both cables, and their order, can be
   read in every link.
4. Measure how far each cable end sticks out past the moving end with a ruler.

## Layout and dimensions (metres, world frame; arm bases at (0, +/-0.305, 0.750); table top 0.7475)

| Object | Type | Material | Size | Mass | Pose |
|---|---|---|---|---|---|
| Cable carrier | fixed articulation: rigid chain body + 12 hinged crossbars + 12 sprung catches | Igumid G (glass-filled nylon), black | 12 links x 30.5 mm = 366 mm; inner 25 x 17 mm; outer 36 x 23 mm | about 0.15 kg (0.40 kg/m) | centreline x = 0.170, fixed end y = +0.183, moving end y = -0.183, on the table, openable side up |
| Fixed-end bracket | part of the carrier body, screwed down | nylon | 36 x 22 x 3 mm plate, two 12 mm lugs | - | y +0.183 .. +0.205 |
| Moving-end bracket | part of the carrier body | nylon | 36 x 22 x 3 mm plate, two lugs | - | y -0.183 .. -0.205 |
| Strain-relief plate | part of the carrier body, screwed down | steel | 40 x 16 x 3 mm | - | centre (0.170, 0.283) |
| Grey motor cable | articulation, 28 capsule segments | PVC jacket, copper cores | 7 mm diameter, 560 mm | 42 g | tie-wrapped at (0.176, 0.283), axis 6.8 mm above the table |
| Orange encoder cable | articulation, 28 capsule segments | PVC jacket | 5 mm diameter, 560 mm | 23 g | tie-wrapped at (0.164, 0.283), axis 5.8 mm above the table |

**Carrier cross-section** (x from the centreline, + away from the robot; z above the table):

| Part | Geometry |
|---|---|
| Side plates (per link) | x +/-12.5 .. +/-18 mm, z 0 .. 20 mm, 29 mm long (1.5 mm gap between links) |
| Bottom crossbar (fixed, per link) | x -12.5 .. +12.5 mm, z 0 .. 3 mm, 12 mm along the chain |
| Openable crossbar (per link) | x -15 .. +18 mm (hinge knuckle radius 1.5 mm at x +16.5, z 21.5), z 20 .. 23 mm, 10 mm wide; 1.1 g; opens 0 .. 100 deg; damping 3e-4 N m s/rad |
| Catch (per link) | 2 x 8 mm tongue sliding in x on the near side plate, 4.5 mm travel, spring 300 N/m preloaded 3 mm (0.9 N at rest, 1.7 N when pushed clear); hook x -18 .. -12.5 mm, flat underside at z 23.8 mm, 45 deg lead-in from (-12.5, 24.6) to (-16.5, 28.6) mm |
| Bracket and plate | as in the table above |

The catch overhangs the crossbar's free end by 2.5 mm. Pressing the free end down onto the lead-in pushes the
catch out; once the bar is past, the catch springs back over it. A shut bar has 0.8 mm of play under the catch
(1.45 deg); a bar resting on the lead-in sits 4.1 mm proud (7.5 deg). The closing force at the free end is
about 3 N (1.7 N spring through a 45 deg ramp with friction 0.3).

**Cables:** 20 mm segments (a cylinder plus a ball at each joint), joints alternately bending sideways (z) and
up/down (x), +/-60 deg. Bending springs 0.030 N m/rad (grey) and 0.015 N m/rad (orange) per joint, damping
0.002 N m s/rad, armature 5e-6 kg m^2. Calibration on a real cable: a 100 mm horizontal overhang droops about
15 mm (grey) and 17 mm (orange). The tie-wrap grips the first segment through a stiff axial spring (3,000 N/m): a 15 N pull draws the
cable 5 mm through, which the scorer counts as the strain relief torn out. Start path: from the tie-wrap along -y for 20 mm, then 30 deg toward the robot for 80 mm (grey) or
120 mm (orange), then straight along -y on the table at x = 0.136 (grey) and x = 0.104 (orange).

Friction (static/dynamic): carrier 0.30/0.25, cables 0.45/0.35.

## Scripted demo (`demo()`, open loop: start layout and time only)

**Both arms, cables (about 26 s).** Grey first, then orange. The left gripper pinches the cable 190 mm out of
its tie-wrap (beside link 3), lifts it 40 mm and holds it over the channel line, so the cable between the tie-wrap
and that hand clears the near side plates. The right gripper pinches it 60 mm from its end, lifts it to 150 mm while
carrying it over the channel line beyond the moving end (grey 6 mm to the far side of the centreline, orange 6 mm
to the near side, in line with their tie-wraps), and lowers it to 8 mm above the table just past the moving end so it settles into the channel.
The right gripper lets go, the left lowers its part to 3 mm above the side plates and lets go, and both rise.

**Both arms, crossbars (about 37 s).** Left closes bars 6 to 1, right bars 12 to 7, at the same time (the two
grippers stay 183 mm apart). Each bar: closed pads come down outside the open bar, push it over past upright,
follow it down onto its catch, let it settle on its catch, then press 10 mm in from the free end (pads 4 mm clear of the catch), commanded 13 mm past flush (about 7 N), and hold 0.6 s.

Both arms then rise clear and hold still for 4 s.

## Real-world parts list

- Cable carrier: igus E2 mini series B15, part B15.025.038.0 (inner width 25 mm, bend radius 38 mm, pitch 30.5 mm,
  inner height 17 mm, outer 36 x 23 mm), 12 links, with the matching mounting brackets. Its crossbars open along
  the outer radius; lay it openable side up.
- Grey cable: 4-core servo-motor cable, about 7 mm, PVC jacket, continuous-flex type; orange cable: encoder cable,
  about 5 mm. 560 mm each.
- Strain relief: a 40 x 16 mm steel tie-wrap plate screwed to the bench 100 mm behind the fixed bracket, two 3.6 mm
  tie-wraps pulled hand-tight (the cable must not slip at 15 N on a spring scale).
- Calibrating the catches: swing a crossbar down onto its catch and press its free end with a push gauge: it
  should snap at about 3 N. igus does not publish a crossbar closing force, so
  this value is ours; measure the chain you use and note it.
- Safety: no hazards; the usual robot guarding and e-stop.

## Files and licences

- `close_cable_carrier_over_robot_cable.py`: the task (scene, scorer, demo).
- `yam_parts.py`, `yam_joints.py`, `yam_mesh.py`: our own primitive, joint and mesh kits, unmodified copies.
- `task.json`, `README.md`, and `test_close_cable_carrier_over_robot_cable.py` (generated scorer tests).
- Every shape is a simulator primitive (box, cylinder, sphere) or a convex mesh generated in code. There are no
  third-party assets, textures or fonts.
