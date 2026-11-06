# Damp a hanging load through its trolley

Bring the hanging test weight to the service mark and leave it hanging still.

Overhead crane and monorail hoist operators stop load sway by moving the trolley, never by steadying the load by hand; a swinging load strikes people and structures. Done by: Crane and Tower Operators (O*NET 53-7021.00), Hoist and Winch Operators (53-7041.00).

## Run and score

From this folder, with the official simulator installed:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=damp_trolley_load_swing.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Bring the hanging test weight to the service mark and leave it hanging still." \
  --scorer task_score --max-steps 600 --store-frames --log-dir demo-run
```

For a model trial use `--policy dream` with Dream credentials configured and a separate log directory. `inspect-robots video <stored-run.json>` converts stored frames to MP4s. `task_score` is the last-step score. The run limit is 600 steps (30 seconds at 20 Hz); one fixed reset, no seed-dependent geometry, `object_jitter=0`. Run the generated offline tests with `pytest .` in this folder.

| Score | Current condition, absent any sticky violation |
|---|---|
| 1.0 | Pointer within 5 mm of world x=200 mm, released, hanger within ±1.5 degrees, continuously 1.0 s |
| 0.5 | Same position and release, hanger within ±5 degrees, continuously 1.0 s |
| 0.25 | At mark and released now |
| 0 | All other outcomes, held trolley, or any sticky violation |

Highest current tier, never best-so-far. Let theta be the hanger angle from vertical and pointer_x the orange pointer flag centre world x. At mark means |pointer_x - 0.200| <= 0.005 m; the weight's own x is not measured, only the pointer flag (at 1.5 degrees the weight centre hangs up to 7 mm from the mark). Released means both gripper TCPs (centres between pads, from robot joint-state FK) at least 0.030 m from the trolley link bounding box, regardless of opening. Score 1.0 when at mark, released, and |theta| <= 1.5 degrees continuously for 1.0 s; 0.5 when at mark, released, and |theta| <= 5.0 degrees continuously for 1.0 s; 0.25 when at mark and released now; 0 otherwise, including a held trolley. Success requires score 1.0 for a further 1.0 s. A tier disappears as soon as its conditions fail. Sticky zero for the rest of the episode if any part of either robot (TCP, fingers, palm, wrist or arm links) ever comes within 0.005 m (inclusive) of the rod box or the weight box, measured with two envelopes from robot joint-state FK: each hand is a cylinder of radius 0.080 m around the tool axis from 0.013 m past the TCP to 0.100 m behind it, which holds both fingers at full opening (73 mm from the axis) and the 68 x 142 mm palm (78.7 mm) at any wrist turn, so the tool axis must stay more than 0.085 m from both boxes; each arm is a capsule of radius 0.045 m around the line through its joint centres from the base to the top of the hand cylinder, which must stay more than 0.050 m from both boxes. Also sticky zero if |theta| ever exceeds 30 degrees, or the weight box ever comes within 0.002 m of the tabletop or either post box (including contact/penetration). Rod and weight boxes are exact rotated-cylinder world AABBs at the reported hanger pose, not the whole hanger link box. Post boxes use the base pose. No velocity cutoff is added: the continuous 1.0 s angular band spans about one pinned swing period and bounds sway; the 5 mm position band must also hold throughout that interval. Offline layouts assume infinite elapsed time and release unless TEST_GRIPPERS supplies the hands; offline test hands point straight down and have no arm links.

On a real rig, read a trial from a side video at 30 fps or more taken square to the beam: theta against lines drawn on the frames from the hinge pin at ±1.5 and ±5 degrees from vertical, held for 30 consecutive frames (1.0 s); at mark when the pointer flag centre stays within the white band, whose edges are 5 mm either side of the mark centre; released when the point midway between each gripper's pads is at least 30 mm outside the box around the blue trolley, its bracket and knob included; and the sticky violation when any part of the robot touches or passes within 5 mm of the grey rod or red weight. The 85 mm hand and 50 mm arm FK envelopes are the simulator's conservative stand-in for that video check.

## Setup and reset

A hand-pushed monorail trainer stands in front of the left arm. A yellow I-beam runs along x on two posts. A blue trolley has a drag brake and a black push knob on a J bracket toward the left arm. A grey hanger rod carries a red cast-iron test weight. A white service mark bands the beam, and an orange pointer flag projects from the trolley beside it. The weight starts displaced 12 degrees toward +x, away from the mark, as if just bumped.

All dimensions and centres below are in mm: x forward from the arms, y left, z up. The tabletop is at world z=747.5. The anchored monorail origin is world (320, 120, 747.5), yaw 0 degrees. Shape centres are local to that origin at trolley slide 0 and hanger angle 0 degrees; apply the initial -12-degree hanger rotation about the hinge to obtain the starting layout. Box sizes follow x/y/z. The four end stops at the end of the table are real-rig parts; the simulation replaces them with the slide's joint limits at the same positions. There are no other task objects.

| Shape | Local centre (mm) | Size (mm) and axes | Material and colour |
|---|---|---|---|
| beam bottom flange | (-60, 0, 328) | box size (450, 60, 6), axes x/y/z | yellow painted structural steel |
| beam web | (-60, 0, 356) | box size (450, 6, 50), axes x/y/z | yellow painted structural steel |
| beam top flange | (-60, 0, 384) | box size (450, 60, 6), axes x/y/z | yellow painted structural steel |
| left post | (-270, 0, 162.5) | box size (30, 30, 325), axes x/y/z | yellow painted square steel tube |
| right post | (150, 0, 162.5) | box size (30, 30, 325), axes x/y/z | yellow painted square steel tube |
| mark band top | (-120, 0, 387.5) | box size (10, 60, 1), axes x/y/z | white paint |
| mark band side | (-120, -30.5, 356) | box size (10, 1, 62), axes x/y/z | white paint |
| trolley side plate (-y) | (0, -37, 318) | box size (44, 8, 34), axes x/y/z | blue painted steel |
| trolley side plate (+y) | (0, 37, 318) | box size (44, 8, 34), axes x/y/z | blue painted steel |
| trolley bottom plate | (0, 0, 302) | box size (44, 82, 6), axes x/y/z | blue painted steel |
| trolley wheel (-x, -y) | (-14, -19, 341) | cylinder diameter 20, length 10, axis y | blue painted steel |
| trolley wheel (-x, +y) | (-14, 19, 341) | cylinder diameter 20, length 10, axis y | blue painted steel |
| trolley wheel (+x, -y) | (14, -19, 341) | cylinder diameter 20, length 10, axis y | blue painted steel |
| trolley wheel (+x, +y) | (14, 19, 341) | cylinder diameter 20, length 10, axis y | blue painted steel |
| pointer flag | (0, -50, 335.5) | box size (4, 20, 15), axes x/y/z | orange painted steel |
| bracket arm | (0, 40.5, 300) | box size (12, 33, 12), axes x/y/z | blue painted steel |
| bracket leg | (0, 55, 207.5) | box size (12, 4, 185), axes x/y/z | blue painted steel |
| bracket foot | (0, 87.5, 115) | box size (12, 73, 6), axes x/y/z | blue painted steel |
| knob stem | (0, 120, 121.5) | cylinder diameter 8, length 7, axis z | blue painted steel |
| knob | (0, 120, 150) | cylinder diameter 20, length 50, axis z | black rubber-covered steel |
| clevis | (0, 0, 299) | box size (12, 14, 12), axes x/y/z; passes through a 30 x 16 mm (x by y) slot cut through the centre of the trolley bottom plate | bright grey steel |
| rod | (0, 0, 183) | cylinder diameter 8, length 220, axis z | bright grey steel |
| test weight | (0, 0, 50) | cylinder diameter 44, length 46, axis z | red-painted cast iron |
| end stop (-x, -y), real rig only | (-204, -16, 341) | box size (20, 22, 20), axes x/y/z | black painted steel block, bolted through the bottom flange onto its top face |
| end stop (-x, +y), real rig only | (-204, 16, 341) | box size (20, 22, 20), axes x/y/z | black painted steel block, bolted through the bottom flange onto its top face |
| end stop (+x, -y), real rig only | (84, -16, 341) | box size (20, 22, 20), axes x/y/z | black painted steel block, bolted through the bottom flange onto its top face |
| end stop (+x, +y), real rig only | (84, 16, 341) | box size (20, 22, 20), axes x/y/z | black painted steel block, bolted through the bottom flange onto its top face |

The base is anchored. The trolley link mass is 1000 g; the hanger link mass is 500 g, distributed uniformly by collider volume: clevis 12 g, rod 67 g, test weight 421 g (component masses rounded to 1 g). Both joint anchors are local (0, 0, 305) mm. The trolley slides along +x, with limits -170 to +50 mm and initial position 0 mm. On the real rig the end stops set these limits. Each stop sits in a wheel track, between the web (3 mm from the centre line) and the flange edge (30 mm). At slide -170 mm the leading edges of the -x wheels (local x -24 mm at slide 0) meet the +x faces of the -x stops at local x -194 mm; at slide +50 mm the leading edges of the +x wheels (local x +24 mm) meet the -x faces of the +x stops at local x +74 mm. No other trolley part comes within the stops' y range (5 to 27 mm either side of the centre line) and z range (local 331 to 351 mm). The scripted demo moves the trolley only from 0 to the mark at -120 mm, so it reaches neither limit. Slide damping, unitless joint friction and armature are 0. Its drag brake has 3.0 N static and 2.5 N sliding friction. The hanger hinges about +y, with limits -60 to +60 degrees, initial angle -12 degrees, armature 0 and unitless joint friction 0. Hinge damping: build the real hinge to about 0.0012 N m s/rad, which with the hanger's 0.0288 kg m2 moment of inertia about the pin gives an amplitude decay rate of 0.021/s (time constant 48 s); the simulation sets 0.00332 N m s/rad (3 significant figures) to reach that same measured decay, because the solver adds energy to this lightly damped hinge at about 0.0021 N m s/rad (0.00332 - 0.0012, inferred from the measured decay; see Fidelity). All joint springs and rest values are 0. Hinge construction, which gives the stated low damping: a 6 mm diameter, 82 mm long hardened steel hinge pin runs along y on the hinge axis at local z 305 mm. It is pressed into a 6 mm bore through the clevis and turns in two shielded (non-contact) deep-groove ball bearings, 6 mm bore x 10 mm outside diameter x 3 mm wide, lightly oiled. One bearing is pressed into a 10 mm bore centred at local (0, -37, 305) mm in the -y side plate and the other at (0, 37, 305) mm in the +y side plate and the bracket arm welded to it (inner faces of the side plates 66 mm apart). Do not use plain bushings, contact-sealed bearings or grease-packed bearings: they add friction. On the real rig the clevis is 6 mm taller than the simulated box (local z 293 to 311 mm), so the pin bore has 3 mm of wall above it; above the axis its corners stay within 8.5 mm of the pin at any angle, inside the slot and clear of the wheels. The bottom plate and the bracket arm carry a 7 mm wide, 3.5 mm deep clearance groove along the pin axis, so only the two bearings touch the pin. The pin and the clevis extension add about 26 g on or next to the pin axis, which leaves the swing period unchanged. These hinge parts are not simulated colliders. The clevis passes through the 30 x 16 mm slot in the bottom plate; its corners reach at most 13.4 mm from the centre line along x at 60 degrees, so the trolley and the hanger never touch each other. In the simulation the bottom plate is a solid box and collision between these two jointed links is switched off, which stands in for the slot. Surface friction is 0.8 static and 0.6 dynamic; all other parts collide with each other. The white top band spans the full 60 mm flange width; the side band spans the full 62 mm beam depth on its -y face. Both are 10 mm wide along x and 1 mm thick. The pointer flag is 4 mm thick along x and 15 mm tall, extending to local y=-60 mm, 30 mm beyond the flange edge. Its centre starts at world x=320 mm; both mark bands are centred at world x=200 mm, so the target slide is -120 mm. The knob centre starts at world (320, 240, 897.5) mm.

Fix the base at its specified origin and yaw; restore trolley slide 0 m and hanger -12 degrees with zero velocities. The harness settles for 0.5 s before policy time zero. Use object_jitter=0. Nothing is consumed.

Hinge acceptance check, before each session of real trials: with the trolley at rest at slide 0 and the robot clear, hold the weight so the hanger is 12 degrees from vertical and release it. Ten full swings must take 10.0 ± 0.1 s, and 30 s after release the swing amplitude, read from the side video against drawn angle lines, must still be 6 to 7 degrees (the simulation gives 6.4 degrees). If it is under 6 degrees the hinge has too much friction: clean or replace the bearings and repeat the check. A hinge that damps faster would let "push to the mark and wait" reach the 5-degree or 1.5-degree band without any sway control.

## Open-loop demo

Only the left arm moves; the right plan is empty and retains READY. Yaw 90 degrees aligns the pads across the black knob. Turn, hover, descend and pinch finish at 3.6 s, followed by a hold. The 0.30 s cubic-smoothstep cancel step starts at about 4.808 s, with commanded centre at about 4.958 s and expected physical centre at 5.000 s. Its displacement is about 52 mm toward -x. Hold through 7 s; transport in 60 minimum-jerk waypoints at 0.05 s intervals through 10 s; hold until 11 s; open for 0.5 s, rise 40 mm over 0.6 s, then park at world (150, 300, 947.5) mm over 1.0 s. Hold through 15.8 s. The plan plus two seconds is 17.8 s, below 60% of the 30 s run limit. The park is at least 45 mm from the trolley box, and its tool axis is at least 0.15 m from the rod and weight boxes. While the pads hold the knob the tool axis is 116 mm from the rod box and 98 mm from the weight box (both across y, which the hinge does not change), outside the 85 mm hand limit.

The rounded build dimensions give a damped pinned period of about 1.001 s, within 0.2% of 1.000 s. The measured amplitude decay rate is 0.021/s (time constant about 48 s), which corresponds to a net hinge damping of about 0.0012 N m s/rad (2 x 0.0288 kg m2 x 0.021/s); the simulator's 0.00332 N m s/rad is that 0.0012 N m s/rad plus about 0.0021 N m s/rad (0.00332 - 0.0012) that offsets the energy the solver adds to the hinge, and a real hinge of about 0.0012 N m s/rad gives the same decay. The one-second angular band spans about one pinned swing period. Waiting through the 30 s policy limit from the original release leaves about 6.4 degrees of sway amplitude.

The calibrated arm lag is 42.43 ms, measured by comparing commanded and measured TCP x over 4.80–5.35 s. The mark command includes compensation for transport undershoot. Cancellation magnitude is recomputed from the fixed hanger geometry. These constants use the known reset and time; the demo reads no object state. A 0.8 duration factor puts the cancel near four pinned periods.

Offline replay contact advisories arise because the pads pinch the trolley knob while replay holds mechanism joints fixed. Offline scoring and geometry checks cover the named layouts, temporal violations, arm meshes, fingers, palm, tabletop clearance, reach and inverse kinematics. They do not measure the physical demo's final score. Scores: Dream-1 run 0.0; demo 1.0.

## Fidelity

Rigid uniform-density unions on a passive slide and hinge; the drag brake is joint friction (3.0 N static, 2.5 N sliding). The solver adds energy to the lightly damped hinge at about 0.0021 N m s/rad (inferred from the measured decay), so the sim's 0.00332 N m s/rad nets the measured 0.021/s decay (time constant 48 s); a real rig needs about 0.0012 N m s/rad for the same decay. No rope stretch or wear. Release and robot envelopes come from joint-state FK.

All assets are original analytic primitives and the supplied kits; no third-party mesh assets are shipped.
