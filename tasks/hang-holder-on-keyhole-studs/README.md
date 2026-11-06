# Hang a holder on its keyhole studs

Hang the orange holder on both keyhole studs of the grey panel.

Facilities installers mount tool holders and safety-equipment holders on headed studs. Both heads go through the wide openings, then the holder drops into the narrow slots and hangs on its own. Done by: facilities maintenance workers and equipment installers.

## Run and score a trial

With the official `yam-isaac-lab-arena` simulator installed, run from this folder:

```bash
# Dream-1 (with the Dream credentials configured)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hang_holder_on_keyhole_studs.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the orange holder on both keyhole studs of the grey panel." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/keyhole-dream

# Scripted insertion and release
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hang_holder_on_keyhole_studs.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the orange holder on both keyhole studs of the grey panel." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/keyhole-demo

# Convert stored frames to one MP4 per camera
for log in ~/yam-eval/keyhole-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`. Full credit requires 1.0 s of released stillness in the hung pose.
It ends in success after full credit holds for 1.0 s more;
otherwise the score is the last step's score. The limit is 1,200 control steps,
60 s at 20 Hz. Run the tests with `pytest .` in this folder, with
`yam_arena` installed. In simulation, the scorer checks insertion and bearing
geometry, gripper clearance and continuous stillness in the hung pose.

## Setup and reset

All dimensions and coordinates are metres; x is forward from the arm bases,
y is left of their midline, and z is up. The table top is z=0.7475. All part
origins start at yaw 0. The only loose body is the 0.120 kg orange painted-aluminium
holder. The panel and rest are anchored painted-steel fixtures without assigned
rigid-body masses. There are no task objects or marks beyond these three parts.

| Part | Geometry and starting layout |
|---|---|
| Grey panel | Plate 0.012 x 0.260 x 0.130 (x,y,z), centre (0.340, 0.180, 0.8125), near face x=0.334, back x=0.346. Two steel studs along x at y=0.130 and 0.230 (100 mm apart), z=0.8725. Each shank: radius 0.0025, length 0.016, x=0.318..0.334. Each head: radius 0.006, thickness 0.003, x=0.315..0.318. |
| Orange holder (`bracket`) | Plate 0.004 x 0.160 x 0.090, origin (0.220, 0.200, 0.7975). Entry windows 0.024 wide x 0.024 tall, local centres y=+/-0.050, z=0.014. Slots 0.008 wide x 0.012 tall above the entries, local z=0.026..0.038. Handle 0.020 x 0.160 x 0.020, local centre (0,0,0.055); `grip=0` selects it. |
| Dark blue rest | U-channel 0.030 x 0.180 x 0.050, origin (0.220, 0.200, 0.7475). Floor 0.030 x 0.180 x 0.004 at local z=0.002; walls 0.009 x 0.180 x 0.046 at local x=+/-0.0105, z=0.027. The slot is 0.012 wide. |

The plate consists of eight non-overlapping strips, all 0.004 thick along x:
a 0.160-wide, 0.047-tall bottom strip at local z=-0.0215; three 0.024-tall entry
strips with widths 0.018/0.076/0.018 at local y=-0.071/0/0.071, z=0.014; three
0.012-tall slot strips with widths 0.026/0.092/0.026 at y=-0.067/0/0.067,
z=0.032; and a 0.160-wide, 0.007-tall top strip at z=0.0415.
`task.json` repeats every shape's dimensions, material, mass and pose.

Reset the holder upright in the channel at the stated origin and yaw, centred
with 4 mm clearance to each wall, with its bottom 1 mm above the floor; let it
settle. The holder may lean up to about 9 degrees in the channel at the start;
the gripper squares it when it closes. Keep both fixtures anchored. Nothing is consumed. Use `object_jitter=0`: one fixed layout,
so the random seed does not change the start.

## Scoring

Credits are alternatives: take the highest satisfied tier. All measurements use
the holder's root origin and rotation; its bounding-box centre is not a feature.
The full-score origin is **(0.3260, 0.1800, 0.8370)**. Its plate bottom is 0.7920,
44.5 mm above the table. Entry-aligned origin height is 0.8585. The locked height
is `0.8725 - (0.038 - 0.0025) = 0.8370`: both shanks touch the slots' closed upper
ends. Lowering the holder moves the shanks upward in the holder frame.

| Score | Condition |
|---|---|
| 0 overall | Tilt exceeds 5 degrees; lowest collider point is below table top + 0.020; any plate corner is beyond x=0.336 or the origin is behind panel face x=0.334; or any plate width extends outside panel y=0.050..0.310. These reject table-supported, leaning, flipped, behind-panel and off-table layouts. |
| 0.25 | The upright holder is out of the rest and within 0.050 of the finite panel face: Euclidean distance from its plate centre in y,z (clamped to the panel rectangle) and rear plate face in x. |
| 0.50 | Inserted: the whole plate is inside the shank span (minimum plate x >= 0.316, maximum plate x <= 0.336, i.e. between the heads' back faces x=0.318 and the panel face x=0.334 with 0.002 contact tolerance), and both shank centres are inside their keyhole openings. Each shank centre is measured in the holder frame where its axis crosses the plate mid-plane: local z 0.0045..0.0375, and its local y offset from its opening's centre line (y=+/-0.050) is at most 0.0095 while local z < 0.026 (in the entry) and at most 0.003 at local z >= 0.026 (in the slot). This is one continuous band from the entry floor to the slot end: for an upright, centred holder, origin z 0.8350..0.8680. The holder may be held or moving. |
| 1.00 | Hung: the 0.50 conditions, plus each shank centre at local z 0.0350..0.0375 (at most 0.0005 below its slot's closed end at local z=0.0355, or up to 0.002 into it) with local y offset at most 0.003; for an upright holder, origin z 0.8350..0.8375. Each slot centre has at most 0.003 of yaw-induced x displacement from the holder centre plane (yaw within about 3.44 degrees of 0 or 180). Released: each gripper's centre point between the pads is at least 0.030 m from the holder's world bounding box. At rest: speed below 0.010 m/s and spin below 0.500 rad/s. All hung, release and rest conditions must hold continuously for 1.0 s to earn 1.00. Success requires full credit for 1.0 s more. |

The bearing rule is checked per shank and is one-sided: a shank more than 0.5 mm
below its slot end does not bear, so a holder held still 0.8, 1.5 or 2 mm below
the seat, or rolled 2 degrees so that one shank bears and the other sits 3.5 mm
below its slot end, earns 0.50. The 2 mm upper allowance is solver penetration
of the resting contact; a real holder cannot sit above the bearing point.
A holder in the hung pose that is held by a gripper or still moving earns 0.50, as does
a released, resting holder before the 1.0 s stillness interval finishes. Release
uses gripper centre points computed from the robot's joint state by forward
kinematics; open fingers alone do not establish clearance. In simulation, full
credit requires both grippers to clear the holder throughout the stillness
interval. Timers reset on interrupted conditions,
skipped control steps and episode resets.

Full credit uses the entire plate bounds: for an upright holder, full credit
allows origin x 0.318..0.334 (the plate anywhere between the head backs and the
panel face, 2 mm contact tolerance), which is -8/+8 mm around the centred goal
x=0.326; for example, 5 mm shifts forward or back (x=0.321 or 0.331) earn 1.00. A 4-degree yaw at the goal earns 0.50:
the long shanks can physically accommodate that yaw, so squaring the slot pair is
an explicit requirement, `abs(R[0,1]) * 0.050 <= 0.003`.

Examples: inserted with the shanks 3 mm above the entry floors (origin 9 mm
above entry height), 9 mm below entry height, halfway from the entries to the
slots, 5 mm off centre sideways in the entries, 30 mm above lock, or 5 mm below
bearing earns 0.50; one moving at 0.2 m/s at the seat earns 0.50. Staging beyond
50 mm earns zero. A plate in front of the heads, including the handle resting on
both heads without insertion, earns 0.25; hanging on one stud at 10 degrees
earns zero. A 45-degree yaw at the goal fails the corner check; a 180-degree yaw
is equivalent for this symmetric, two-sided holder. A ruler, side and front
photographs and a level check the geometric conditions; repeated photographs
check stillness.

## Scripted demo and runtime

The left gripper points down, yaw pi/2, closing across the 20 mm handle. The TCP
is 7 mm above the handle centre, so 16 mm of pad height engages the bar. The first
four waypoints, including a 60 mm straight vertical lift, come from
`pick_place_waypoints`; the helper has no per-waypoint duration argument, so a 1.0 s hold at the lifted pose settles the holder before
carry. Insertion uses explicit `Waypoint`s, including a 0.8 s closed-grip pause
before insertion, a 2.5 s pass, and a 1.0 s closed-grip pause
after passing both heads. Carry, approach, pass and settle use
`ENTRY_TCP_Z + 0.003`: the holder rides about 3 mm low in the pads, so it is carried 3 mm higher to centre the heads in the entries. The right arm parks at
(0.180, -0.300, 0.8975) over 1.5 s,
then waits there through 24.3 s, clear of the holder and panel.

| Left segment | Duration (s) | End time (s) | TCP (x, y, z) | Grip |
|---|---:|---:|---|---|
| Hover | 1.5 | 1.5 | (0.2200, 0.2000, 0.9185) | open |
| Descend | 1.5 | 3.0 | (0.2200, 0.2000, 0.8585) | open |
| Close | 0.8 | 3.8 | (0.2200, 0.2000, 0.8585) | closed |
| Lift 60 mm | 1.2 | 5.0 | (0.2200, 0.2000, 0.9185) | closed |
| Hold lifted | 1.0 | 6.0 | (0.2200, 0.2000, 0.9185) | closed |
| Carry to entry height | 2.5 | 8.5 | (0.3000, 0.1800, 0.9235) | closed |
| Approach +x | 1.5 | 10.0 | (0.3180, 0.1800, 0.9235) | closed |
| Settle before insertion | 0.8 | 10.8 | (0.3180, 0.1800, 0.9235) | closed |
| Pass both heads +x | 2.5 | 13.3 | (0.3290, 0.1800, 0.9235) | closed |
| Settle after insertion | 1.0 | 14.3 | (0.3290, 0.1800, 0.9235) | closed |
| Lower 14 mm | 1.8 | 16.1 | (0.3290, 0.1800, 0.9095) | closed |
| Release | 0.8 | 16.9 | (0.3290, 0.1800, 0.9095) | open |
| Retreat vertically | 1.2 | 18.1 | (0.3290, 0.1800, 0.9235) | open |
| Retreat -x | 1.2 | 19.3 | (0.2690, 0.1800, 0.9235) | open |
| Park | 2.0 | 21.3 | (0.2200, 0.3000, 0.9235) | open |
| Hold clear | 3.0 | 24.3 | (0.2200, 0.3000, 0.9235) | open |

After opening, the TCP rises vertically before moving 60 mm back along -x at
that height, then parks. The open pad tips clear the seated handle by 8.5 mm
before sideways travel, avoiding a sweep through the handle. For the goal
holder box, low (0.316, 0.100, 0.792), high (0.336, 0.260, 0.902):

| Retreat waypoint | TCP (x, y, z) | Left planar reach (m) | TCP-to-box distance (m) |
|---|---|---:|---:|
| Release | (0.3290, 0.1800, 0.9095) | 0.351946 | 0.007500 |
| Rise vertically | (0.3290, 0.1800, 0.9235) | 0.351946 | 0.021500 |
| Retreat -x | (0.2690, 0.1800, 0.9235) | 0.296624 | 0.051684 |
| Park / hold clear | (0.2200, 0.3000, 0.9235) | 0.220057 | 0.106199 |

The TCP clears 0.045 m by the -x endpoint at 19.3 s, leaving 2.0 s of
parking and a final 3.0 s hold (5.0 s total) for the 1.0 s stillness test,
1.0 s success hold and margin. The plan is 24.3 s. After the plan, the
scripted policy holds for another 3.0 s, stopping at 27.3 s, below 36.0 s
(60% of the 60 s run limit). In simulation, the scorer observes arm release and
measures the continuous stillness interval before awarding full credit.
A full
Dream trial uses at most 1,200 calls to the 20 Hz controller. Wall-clock runtime
depends on the simulator host and policy latency; 60 s is the simulated trial
budget, not a measured wall-clock duration.

## Geometry and motion margins

- Slots are above the entries: lowering the holder moves the shanks toward their
  closed upper ends. Entry centres stay 14 mm above the plate centre. A 24 mm
  entry plus a 12 mm slot ends at local z=38 mm, preserving the 7 mm top web.
- The panel back is x=0.346, near face x=0.334, centre y=0.180. Studs and holes
  are 100 mm apart at y=0.130 and 0.230.
  The rest stays at (0.220, 0.200); the holder starts centred at x=0.220 in its
  12 mm slot, with 4 mm clearance per side and a 1 mm gap above the floor.
- Carry x=0.300 and approach x=0.318 retain their 34 mm and 16 mm offsets from
  the panel face. Another 11 mm +x passes both heads to x=0.3290,
  3 mm past goal x=0.3260 to compensate arm lag. Lower, release and
  vertical retreat also use `PASS_X`; the next retreat uses `PASS_X - 0.060`.
  Every left TCP target is within 0.351946 m of the left base in the table plane,
  below the 0.36 m margin limit, including the entry TCP at table top +170 mm.
- The 160 mm wide plate clears the 50 mm tall rest by 14 mm after the 60 mm
  lift (11 mm with 3 mm of gripper hang). At carry height the plate bottom is
  69 mm above the table (66 mm with hang), so it remains clear throughout the
  slightly rising carry (lift TCP 0.9185 to carry TCP 0.9235). Before
  insertion its rear face stays at least 14 mm ahead of the panel, then stops 3 mm ahead of it at `PASS_X`.
  At the approach waypoint the plate's front face is at x=0.316, so both heads
  (x=0.315..0.318) are already 2 mm into the entry windows during the 0.8 s
  settle; the pass then carries the plate the remaining 11 mm. Where the 20 mm deep
  handle overlaps the panel in x, its bottom clears the panel top by 29 mm
  at the compensated entry height (26 mm with 3 mm hang). Neither carry segment
  sweeps a fixture.
- A 12 mm head in a 24 mm entry has 6 mm clearance per side. A 5 mm shank in
  an 8 mm slot has 1.5 mm clearance per side; contact offsets remain 1 mm.
  The 16 mm shank span minus the 4 mm plate leaves 12 mm total axial play,
  split into 6 mm on each side at the goal. At 5 degrees of yaw the plate's
  rear x extent is 8.965 mm from its origin, so the corner interferes with the
  panel by 0.965 mm at GOAL_X (within the 2 mm solver penetration allowance).
- The geometric entry-to-seat drop is 21.5 mm. The insertion TCP is 3 mm high
  to offset the hang; after the commanded 14 mm descent the release TCP is
  0.9095, putting the origin at 0.8475 (0.8445 with 3 mm hang) against the
  locked height 0.8370, so the holder drops about 7.5 mm onto the shanks when
  released (10.5 mm without hang). At release the shank centres are at local
  z=0.028 (0.025 without hang), at the slot mouths (local z=0.026), so the
  slots guide the drop to the closed ends.
- The panel stays 130 mm tall: its top clears the seated handle bottom by
  4.5 mm. Retreat stays at table top +176 mm so the open pad tips clear the
  seated handle by 8.5 mm before sideways travel.

## Fidelity

The three parts are rigid unions of analytic colliders; holes are gaps between
strips, with 24 mm square entries, 8 mm wide by 12 mm long slots and a 12 mm rest slot. `yam_parts.py` is an
unchanged copy of the compound-parts kit. Mass properties come from the colliders at uniform density.
There is no flex, wear or damage. All parts use rubber-like friction (1.2 static,
1.0 dynamic), grippier than bare metal, and 1 mm contact offsets. Fixtures do not
move. Explicit goal and cheat layouts test the scorer independently of the
kinematic replay, which cannot model hanging.

In simulation, the scorer checks that both shanks pass through their keyholes
and bear on the closed slot ends, with the plate between the heads and panel.
PhysX models contact, hanging and retention. Release uses robot joint state
(gripper centre points by forward kinematics); full credit requires both
grippers clear of the holder and one continuous second of released stillness.
