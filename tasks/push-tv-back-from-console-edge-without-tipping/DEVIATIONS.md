# push-tv-back-from-console-edge-without-tipping build deviations (2026-11-21)

1. **The dresser top is the station table top.** The arms are mounted on the table, so the TV stands on it. The
   dresser's front edge is a taped line (x 32), not a physical drop; a foot over the line scores 0 (latched). A real
   dresser of the same height can be pushed against the table for the real trial.

2. **The TV is one rigid body of boxes.** The panel is a 732.9 x 432.6 x 78.5 mm box, its mass (4.0 kg, set with stand)
   spread evenly, so its centre of mass is at the panel's centre, 24.7 cm up. A real set's electronics sit low and
   behind the screen; that moves the centre of mass a little down and back, which does not change the tipping rule
   (tipping height (D / 2) / mu does not depend on the centre-of-mass height) and makes a screen push tip it backward
   no later. Each Mini Arc foot is a 137 x 165.8 x 30.2 mm block (the arc's footprint); the real foot touches the top
   at the arc's ends.

3. **Friction is not published.** No source gives a friction coefficient for the stand's plastic on a laminate or wood
   top. The demo is run in MuJoCo with the TV-top friction set to 0.4, 0.6 and 0.85 (dev/mu_sweep.py) and scores 1.0 at
   each; the screen push scores 0 at each.

4. **mj_check START 'unreachable tv'.** The offline checker's start check asks for a straight-down grip at the object's
   centroid height (here the panel centre, 25 cm up). The TV is not to be grasped; the demo's push poses solve the
   official IK within 0.1 mm (DEMO check). VISUAL 'outside_frame tv': the TV's top edge leaves the top camera's frame;
   the scored region (both feet and the edge tape) is in frame of both top cameras (tested).

5. **The hands end out beside the TV.** The run ends when the score holds 1.0; the first demo ended with the hands
   rising over the edge tape, where the 'demo' top camera lost a foot corner and the tape behind a finger (occlusion test
   on the Isaac record). The hands now move out to y +-42 cm after backing off.
