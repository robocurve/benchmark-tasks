# spin-lazy-susan-bring-back-jar-to-front build deviations (2026-11-21)

1. **The tray is turned by pinching its lip, not by pushing its rim.** A person usually spins a lazy Susan by pushing the
   rim with the fingertips. In MuJoCo a closed-finger push on the 10 mm lip was erratic: 0 to 73 degrees per three to
   four strokes, with the tray dragged back as the fingers left it. The demo therefore turns it the way one turns a
   loaded tray carefully, hand over hand: the gripper pinches the lip 5 mm inside the rim, carries it round up to 45
   degrees, lets go, moves back and pinches again. The stroke that completes the turn keeps hold of the lip, and that hand
   holds the tray still while the other hand takes the jar out.

2. **The demo is closed loop.** Each stroke's size is decided when the stroke starts, from the tray's angle (at most 45
   degrees, only what is still needed to bring the red-capped jar to the front); the taking hand goes to the jar where it
   then stands. Both stand for what the cameras show; the scripted demo reads them from the simulator state. Strokes land
   about 30 to 34 degrees, not 45, because the pads slip on the lip; up to seven strokes are planned and the unused ones
   hold.

3. **The bearing's friction is not published.** The tray is a hinge with Coulomb friction. OXO gives no friction torque
   for the 1071352's steel ball bearing, so the demo is run at 0.005, 0.02 (the shipped value) and 0.05 N m and scores 1.0
   at each in offline MuJoCo.

4. **Parts are boxes.** mechkit builds parts from boxes: the round tray and its lip are 24 segments, the fixed base a
   12-segment disc, the jars 45 x 45 mm square boxes (the McCormick bottle is 1.77 x 1.77 in with rounded corners).
   Of the turntable's 0.39 kg, 0.30 kg is given to the tray; the rest is the fixed base.

5. **The side units end at the upper shelf.** IVAR side units are 124 cm tall. At that height the left side unit's front
   post would stand where the left arm's fingertips are at the ready pose (8 mm into them), so in the scene the posts end
   at the upper shelf's top, and in the real trial the two side units are sawn off (solid pine) just above the upper
   shelf. The bottom shelf lies on the counter (the station table).

6. **Where the shelf stands and where the hand turns the tray were set by the arm's reach.** Pinches on the tray's
   front-right (azimuth 202.5 degrees) were 8 to 15 mm outside the official IK at any shelf position tried (y 0 to 100 mm,
   x -40 to +20 mm). The shelf section stands 20 mm left of the station's centre line, and the turning hand works on the
   tray's front-left, from azimuth 172.5 to 127.5 degrees; every pinch and back-off pose there solves within 0.07 mm.

7. **The top cameras see little under the upper shelf.** The upper shelf is 40 mm over the jar tops, so from the top
   cameras most of the tray and the jars at the back are hidden; that is the point of the task (the jar wanted cannot be
   seen or reached from the front until the tray is turned). The scorer reads poses; in the real trial the jars are
   checked by looking in from the front after the run (README, Scoring).

8. **"At rest" is measured from poses.** The stock rest test reads the bodies' reported velocities (under 1 cm/s and
   0.5 rad/s). In the GPU simulator a jar standing on the tray kept reporting about 1.4 cm/s (mostly vertical) for the
   last 8 s of the run while its position changed by under 1 mm, so the run could never finish. The scorer
   therefore calls the jars still when each moved under 2 mm and turned under 2 degrees over the last second, which is
   also what the real trial reads from the video.
