# Simulation limitations and declared parameters

- This is a function test of a sound e-stop, not electrical energy isolation, hazardous jam clearing, a diagnostic inspection, or a safety certification. The final physical state matches the initial state; observed trajectory evidence is necessary.
- The operator is the catalogue 16 mm-panel XA-class, with a 40 mm mushroom, 4.5 mm stroke, 10.5 N push-to-lock force and 0.16 N*m turn-to-release torque. Release angle 30 degrees and the START spring force of 5 N are declared rather than manufacturer specifications.
- The latch uses a contact-driven joint and a hook-moved spring detent, not a fully modeled pawl/cam. Twisting resets only the e-stop device. A separate green START actuator requires a fresh press after the head returns. No separate safety-relay reset is modeled. Legacy reset joint names refer to START.
- The platen is a continuous position-target actuator. Stopping holds the target and models servo braking, not electrical disconnection and free coast. There is no teleport or quarter-turn wrap.
- Actual lens and platen positions are observed for scoring. Stop tolerance is 0.20 rad/s, run threshold is 0.50 rad/s and lens tolerance is 2 mm. These are declared observation thresholds, not manufacturer safety limits. Running credit proves rotation, not exact 60 rpm accuracy.
- The caliper, solvent can, glove box and cables are visual-only dressing. The grinder-polisher dimensions are declared generic values, not verified against a particular machine.
