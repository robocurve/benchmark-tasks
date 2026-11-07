# Physical scoring and exact predicates

Safety failures at any time permanently score 0: panel lean outside 10–32 degrees from vertical, foot horizontal drift over 18 mm or foot height change over 10 mm; either moving object more than 5 mm below the table or outside x=0.05–0.50 m, |y|<=0.40 m; invalid/nonfinite states. Otherwise 0.5 for the stopper centre within 4 mm horizontally of (0.233,0), bottom within 3 mm of z=0.7515, tilt/yaw <=5 degrees, speed <15 mm/s, spin <0.15 rad/s. Full score requires both grippers >=80% open and TCPs >=8 cm outside both bounding boxes, then a 0.6 N pull toward -X at the panel bottom-edge centre for 0.4 s, followed by 2 s with panel and stopper position range <=3 mm per axis and both objects have final speed <10 mm/s and spin <0.1 rad/s. Losing geometric seating or returning hands during proof scores 0. Transient motion inside the socket during the pull is allowed; the stillness requirement applies after it. Full proof passed: 1.0; otherwise valid seated stopper: 0.5; all remaining outcomes: 0. Final-state scoring.

All dimensions and initial poses are in `scene_inventory.json` and `task.json`.
The panel starts 20 degrees from vertical. Shallow side guides constrain sideways
motion but leave the foot free to slide toward the robot. The stopper is a
rectangular block, an allowed stopper variant of the proposed wedge task.

A seated stopper has its centre within 4 mm in x and y of its target, its bottom
within 3 mm of the base surface, and tilt and yaw within 5 degrees of the socket
axes. Translation must be below 15 mm/s and rotation below 0.15 rad/s.

Safety is irreversible within an episode: panel lean must remain from 10 to 32
degrees, its foot's horizontal displacement from its initial mark at most 18 mm,
and its foot height within 10 mm of the initial value. A moving object below the
table by more than 5 mm, outside x 5–50 cm or |y| 40 cm, or a nonfinite state
also fails the episode. Zero is retained even if the robot restores the layout.

Both grippers must be at least 80% open, with each TCP at least 8 cm outside both
objects' complete bounding boxes. Once the stopper is seated and both grippers
clear, the test applies a 0.6 N horizontal pull toward the robot at the centre
of the panel's bottom edge for 0.4 seconds. It then checks 2 seconds of stability:
panel and stopper position range at most 3 mm in each axis, panel translation
below 10 mm/s and rotation below 0.1 rad/s, with the same final speed and spin limits for the stopper. Returning a hand or losing the stopper's geometric seating during this test fails the episode. Brief motion under the pull is permitted; both objects must meet the final stillness check. The pull is a world-frame external
wrench, not a pose edit or scripted panel motion.

Score 0.5 for a seated stopper while all safety limits hold, awaiting completion
of the release test; 1.0 after the whole test passes; 0 otherwise. Untouched start
scores zero. Scoring uses the final state, not the best historical score.

For a tabletop reproduction use only a small, low-mass panel and a bounded
catch area. Match the dimensions, mass, measured friction and starting angle;
mark the foot location and stopper target. Record a side video against angle
and displacement marks, plus a top view of the socket. Apply the test pull at
the foot using a 0.6 N spring scale or a calibrated 61 g hanging mass with a
low-friction pulley, for 0.4 s. Use a ruler in the video for displacement and the
2 s stability interval. These proof-test tools are examiner equipment, not props
manipulated by the robot. This scale analogue does not establish full-size panel
handling safety.
