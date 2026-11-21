# Remove the middle feeder from a packed feeder rail on a pick-and-place machine

**Instruction:** Take the middle feeder off the rail and lay it on its side on the blue mat. Leave the other two
feeders in place.

**Category:** Impact on AI timelines.

## Why it matters

Robot builders assemble their motor-driver, encoder and sensor boards on pick-and-place machines; small makers and labs
use desktop machines such as the LumenPnP, which take the same kind of tape feeders on a rail as production lines. Each
board needs a different set of parts, so every board change means taking feeders off the machine's rail and putting
others on. A robot that runs such a machine between jobs has to do this feeder change itself, at the machine. The
feeders sit side by side at the rail's slot pitch, so the job is always "take one unit out of a packed row without
disturbing the ones next to it": a neighbour knocked off its slot loses its slot address and its pick position, and the
next board is placed wrong.

## What makes it hard for a robot

- **No grip is available at the start.** The three feeders sit at the rail's 15.8 mm slot pitch: 0.8 mm between the
  15 mm bodies and 2.8 mm between the 13 mm reels. A YAM finger is 6.4 mm thick, so it cannot go down beside the middle
  feeder or its reel, and a pinch from above closes on all three. Nothing on the middle unit can be grasped until the
  unit has been moved out of line.
- **It is held by two features at once.** A round hook nose under the body sits in the slot along the rail's top
  face, and a yellow sprung lever on the clip hooks two teeth under the slot blade on the rail's front face, with the
  clip fitting closely round the blade's peg. The unit only comes free by turning about the hook (reel end up, inner
  end down) once the catch lets go, then lifting the hook out of the slot.
- **The release and the grasp need two hands.** Whatever turns the middle unit out of line has to keep it turned while
  the other hand takes the part that now stands proud of the bank. Let go, and the reel's weight drops the unit back
  into its catch.
- **The machine limits the approach.** The bank sits on the machine's front rail, as on the real machine: the bed
  starts 5 mm behind the feeders' inner ends, the front left leg and side frame stand about 75 mm to the near side, the
  control box sits between the robot and the machine, and the slot blade fills the space under the feeders' middle
  span, in front of the rail. The inner end can only be reached from above, over the bed's edge, and the unit can only leave toward the
  reels.
- **It must leave the bank along its own plane.** The unit is 345 mm long (reel edge to inner end) and 0.8 mm from its
  neighbours. It can only slide out sideways once its inner end has passed their reels; any twist rubs the neighbours,
  and pushing or prying on a neighbour unseats it.
- **The wrong ways score 0:** pressing, pulling or dragging a neighbour until its catch lets go or its hook leaves the
  slot (even if it then drops back into its seat), pulling the middle unit up while its catch or hook is still
  engaged, dropping it, or leaving it on the rail.

The maker's mounting guide documents both release routes for a feeder between occupied slots: reach under the spools
to the lever and "push forward to tip the feeder", or remove it "without depressing the lever arm" by pushing or
pulling the spool arm ([mounting guide](https://docs.opulo.io/feeders/4-mounting/mounting/)). Both are open here (the
catch is a sprung tooth that cams out under load). Neither is required, and the scoring only reads the end state and
the whole-trial failures.

**How it differs from accepted tasks.** Tipping a recessed book off a packed shelf (TK-E6ECW7) also has no grasp until
the item is tipped out, but the book is free: once tipped it stays tipped and is then taken with one hand. Taking one
bottle from a row (TK-HAH39Z) and swapping a module from a row of sockets (TK-KHHQD4) start with a graspable item. Here
the unit is latched by a hook and a sprung catch and falls back into its catch the moment it is let go, so one hand
has to hold it tipped while the other takes the end that now stands proud; it is then lifted off its hook and slid out
along its own plane, past two neighbours 0.8 mm away that must stay seated, inside the machine's frame.

## Scene and parts (true size)

Positions are in task.json's `setup` (robot frame): the machine's left outer face is 150 mm in front of the arm bases,
its front rail's centre line 70 mm to the right of the robot's centre line, and the bank fills its first three front
slots, so the robot works at the machine's front left corner.

| Item | Real product | Source | Modelled as |
|---|---|---|---|
| Machine | Opulo LumenPnP desktop pick-and-place machine | maker's assembly CAD (`src/assembly.FCStd` in the open-source machine repository) | fixed boxes at their CAD positions: front frame rail, front and back legs and feet, lower and upper side rails with their guide rails, back frame rail, bed plate (3 mm, starting 85 mm behind the rail so it clears the feeders by 5 mm, as the maker specifies), bottom camera, control box, gantry beam, carriages and toolhead parked at the back of travel |
| Front rail | 20 x 20 mm V-slot aluminium extrusion, 600 mm, the machine's front frame member | assembly CAD: centre line 110 mm above the bench (machine on its leg extensions) | core with its top-face slot, simplified to a 5 mm wide, 4 mm deep channel (a real V-slot groove is 6.2 mm wide at the face with 45 degree lips) |
| Slot blade | LumenPnP front slot blades (25 slots, x 103 to 498 mm along the rail) | assembly CAD | yellow block 28 mm deep, 45 mm tall on the rail's front face, one 5 x 5.5 x 6 mm peg per slot at a 15.8 mm pitch |
| Tape feeders (3) | Opulo LumenPnP 8 mm feeder with its spool arm | [maker page](https://opulo.io/products/feeder), [mounting guide](https://docs.opulo.io/feeders/4-mounting/mounting/), [overview](https://docs.opulo.io/feeders/1-overview/feeder-overview/), design files (the `opulo-inc/feeder` repository on GitHub; frame-8 envelope 164.8 x 82.8 x 15 mm) | black body 165 long (70 mm in from the rail centre, 95 out), 15 wide and 73 tall (the frame's 83 mm envelope less the bulge under its drive gear, left out), hook nose, clip round the peg, yellow two-tooth sprung lever; body 0.13 kg (estimate) |
| Reels (3) | Standard 7 inch (178 mm) reel of 8 mm component tape, EIA-481 | maker spec sheet ("standard 7 inch reel") | white disc 178 x 13 mm on a revolute joint at the spool arm's hub, 0.13 kg (estimate), light hub drag; the threaded tape is a one-way stop (see Fidelity) |
| Mat | Anti-static bench mat, blue, cut to 300 x 420 x 3 mm | any electronics supplier | loose slab, 0.5 kg |

Drawings of the slot blade, peg, rail section and the feeder's interface: `fixture/`.

Estimates, each marked as such in `task.py`: the hook, clip and lever dimensions (sized from the maker's photos; the
exact printed geometry is not reproduced), the masses, the lever spring and the reel's hub drag. Layout: lot A, the
screenshot; lot B moves the mat 30 mm toward the robot.

## Scoring (what the code checks)

Whole-trial zeros (0 at any moment, and it stays 0):

- **A neighbour knocked loose.** On the real machine a feeder whose hook leaves the rail's slot, or whose catch lets
  go, has lost its seat, even if it then drops back: it has to be checked and reseated before the next board. A brief
  rock that leaves it hooked and latched does no harm. So the code checks the two holding features of each outer
  feeder at every control step, from its body pose and its lever's pose in the rail's frame: the **hook** is out when,
  at either end of the nose, the nose's lowest point is above the rail's top face or its axis is outside the slot's
  5 mm width; the **catch** has let go when either lever tooth is no longer under the slot blade (the tooth's centre is
  above the blade's bottom face, or its inboard edge is in front of the blade's front face, that is, past its last
  point of contact with the blade's bottom corner). With the lever closed, the catch lets go once the feeder rocks
  about 3.5 to 4 degrees about its hook, reel end up (depending on how it settled in its seat); the near misses check
  both sides of that edge. As a backstop for sliding along the rail or turning about a vertical axis, which the
  simplified clip does not stop, a neighbour whose origin (on the rail's centre line under its hook) moves more than
  5 mm, or whose body turns more than 10 degrees, also counts as knocked loose.
- **A feeder dropped or off the bench.** Any feeder's body speed exceeds 1.2 m/s (as in a free fall from about 7 cm;
  the demo's set-down peaks well below that), or any feeder's origin drops 2 cm below the bench top.

A missing or malformed observation of the scene or the hands scores 0 while it lasts (fail closed). The record resets
only at a scene reset. Missing or malformed observations, a native hand-read failure, or a gap greater than
one 20 Hz control interval restart the released/still timer. Recovery must show a fresh continuous second.

End state:

| Score | Condition |
|---|---|
| 0 | either outer feeder's origin (on the rail's centre line under its hook) more than 2 mm, or its body more than 3 degrees, from its starting pose; or any point of the middle feeder (every box of the body and the lever sampled at about 4 mm on its faces; the hook nose, teeth and reel sampled on their rims and axes) within 5 mm of the rail, the slot blade or a peg |
| 0.5 | every point 5 mm or more clear of them |
| 0.75 | and: no gripper or fingertip surface point (face-sampled hulls) within 5 mm of any collider of the middle feeder (exact distance to each box and to each solid cylinder, reel faces included); the feeder's across axis within 20 degrees of vertical (lying on a broad side); its lowest point within 5 mm above the bench or the mat top; still (under 5 mm/s and 0.05 rad/s) continuously for 1 s |
| 1.0 | and every sampled point lies inside the mat's outline (seen from above) |

`success()` is `score() == 1.0`. Trials are run on lots A and B and averaged.

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted --scorer task_score \
        -E task=tasks/remove-the-centre-feeder-from-a-populated-bank/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "Take the middle feeder off the rail and lay it on its side on the blue mat. Leave the other two feeders in place." \
        --epochs 2 --max-steps 800
    inspect-robots run --embodiment yam_arena --policy dream --scorer task_score \
        -E task=tasks/remove-the-centre-feeder-from-a-populated-bank/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "Take the middle feeder off the rail and lay it on its side on the blue mat. Leave the other two feeders in place." \
        --epochs 2 --max-steps 2400

Trial k uses lot k mod 2 (A first). `TASK_LOT=B` in the environment pins every trial to lot B. Run with
`object_jitter=0`. The final scripted trials take about 30 s of simulated time each; the task allows up to 120 s
per trial. Wall time depends on the simulator host.

## Demo

`demo()` is open loop from the published layout (the only thing it reads per lot is the mat's starting position). The
right hand reaches over the bed's front edge and presses the middle feeder's inner end down with one open finger until
its catch lets go and the unit turns about its hook, reel end up. The left hand comes down over the feeders' outer
ends and pinches the middle feeder's body where its outer end now stands proud of the others, while the right finger
still holds the unit turned. It grips the body, not the reel: the reel turns freely on its hub, so a reel grip would
leave the extraction depending on how much load the threaded tape can hold. The left hand commands a 22 mm lift to take the hook out of the
rail's slot and holds the unit while the right finger lifts away, then moves it 20 mm out, levels it with a 15 degree
lean of the wrist, slides it out along its own plane until its inner end has passed the neighbours' reels, carries it
over the mat, leans it and lets go so that it settles on its side. The demo uses `tilt.py` for leaned grips, `lots.py`
for the two mat positions and `parts.py` for the jointed parts; they patch the stock embodiment in this process only
and change nothing on disk.

## Results

With the interrupted-hold timer corrected, the final scripted demo scores **1.0 in lot A and 1.0 in lot B**
(mean 1.0). Trials last 30.25 s and 30.35 s of simulated time. Both neighbours remain hooked and latched
throughout. Maximum neighbour movement/rotation: 1.16 mm / 1.59 degrees in A; 1.36 mm / 1.99 degrees in B.
Both finish released and still for 1.95 s with at least 30 mm hand clearance (the distance report is capped there).
Peak feeder-body speed is 0.638 / 0.670 m/s, below the 1.2 m/s drop threshold. Every 20 Hz observation is valid,
with no missing control interval. The combined demo video shows A then B.

The middle reel joint spans -0.020 to 0.371 rad in A and -0.020 to 0.284 rad in B, reaching the modelled winding
stop while the hand grips the body. These are simulated joint measurements, not physical test results.

Dream-1 used this same frozen runtime, the same instruction and two epochs: **0.0 in lot A and 0.0 in lot B**; both trials ran to the 2,400-step limit with the middle feeder still seated. No instruction was reworded. (An earlier Dream-1 attempt on the same source was cut short by a `DA-2000 NO_CAPACITY` service error in lot A; it was re-run in full.) No physical hardware test has been performed.

## Near misses

`python near_misses.py` checks every level and its boundaries: seated; lifted 3 mm; tipped 15 degrees with the hook
still in; lifted 30 mm with the clip still over the blade (0); 4.9 mm and 5.1 mm clear (0 / 0.5); held clear (0.5);
lying flat on the mat released for 1 s (1.0) or 0.9 s (0.5); a fingertip 4 mm away (0.5) and exactly 5 mm away (1.0);
fingertips pinching the reel faces at mid radius (still held: 0.5); 0.5 mm inside and 10 mm over the mat edge
(1.0 / 0.75); flat on the bench (0.75); tilted 25 degrees (0.5); 8 mm above the mat (0.5). Neighbours, each for one
step and then put back: rocked 2 degrees about its hook, and rocked +1.5 / -0.5 degrees, still hooked and latched (no
zero); rocked 0.2 degrees short of and past the angle at which a lever tooth comes out from under the blade (no zero /
0); inner end lifted about the teeth until the nose is just inside or just out of its slot (no zero / 0); lever opened
0.01 rad short of and past releasing (no zero / 0); lifted 3.6 mm or 3.8 mm with its teeth on the blade's corner (no
zero / 0, the hook alone); lifted 2.6 mm with the lever closed (0, the tooth height alone); slid 4.5 mm or 5.5 mm along
the rail (no zero / 0) or turned 2 or 11 degrees about a vertical axis (no zero / 0), the backstop; pulled 6 mm out of
its slot, or tipped 11 degrees (0). A neighbour
ending 2.5 mm or 4 degrees off its seat (0) or 1.9 mm or 2.9 degrees (no zero); unknown or NaN hand geometry (0).

## Fidelity

See the `fidelity` field. In addition:
- **Reel and tape.** Each reel is its own body on a revolute joint at the spool arm's hub, with light drag (estimate
  0.002 N m s/rad). The model assumes that the stopped tape drive holds the threaded tape: the reel can unwind
  and make slack but cannot wind taut tape back through the feeder. A one-way angular stop permits 3 rad of
  unwinding and 0.02 rad of winding play. The tape strip is visual only; tape stretch, tearing and drive compliance
  are not simulated. The body grasp does not rely on reel friction or tape tension to support the feeder.
  The real-trial procedure below checks these assumptions; it has not been performed.
- **Friction.** The feeders' printed surfaces have a friction coefficient of 0.38 static and 0.30 sliding. The static
  value is the top of the 0.12 to 0.38 range measured between pairs of printed PLA parts (printed PETG pairs: 0.13 to
  0.21) by Santana, Alves and Netto, "Effect of the angle of the laid-up filaments on the surfaces on the coefficient
  of static friction between parts manufactured by extrusion-based 3D printing", Proc. IMechE Part L 235(6), 2021; the
  sliding value is an estimate. The earlier value, 0.9, had no source. Taking the top of the measured range errs
  toward more drag between neighbours. The simulator
  averages the two surfaces' coefficients at a contact: feeder on feeder 0.38, on the aluminium rail about 0.34, on
  the gripper fingers (the simulator's default 0.5) about 0.44, on the mat about 0.64.
- **Packed contacts.** The declared reset has 0.8 mm between feeder bodies and at least 0.05 mm clearance at the
  rail interface. Every task collider uses a 0.2 mm contact offset and zero rest offset; the two offsets sum to
  0.4 mm, below the packing gap. Articulations use 64 position and 8 velocity solver iterations and a 0.1 m/s
  depenetration-speed cap. After placing the mat for the lot, the arms stay at their ready poses for 2 s to settle
  the scene before policy actions. These numerical settings leave the geometry, friction, hook/catch tests and
  whole-trial failure rules unchanged.
- **Machine.** The machine's printed parts (legs, control box, carriages, toolhead) are their bounding boxes from the
  CAD, which is conservative for clearance. The gantry is parked at the back of its travel, as an operator would before
  a feeder change. The printed frame's flex and wear-in are not modelled; the lever is a single hinged link with a
  preloaded spring.

## Real-trial check procedure (not performed)

The simulation gives each standard 7 inch (178 mm diameter, 13 mm wide) reel its own revolute joint, with
estimated hub drag of 0.002 N m s/rad. Threaded tape is represented by a one-way angular stop: up to 3 rad of
unwinding slack and 0.02 rad of winding play. The visible tape strip has no flexible-body physics; tape stretch,
tearing and sprocket compliance are not modelled. The extraction demo grips the feeder body, leaving the reel
free to rotate. These are stated approximations, not measurements from a physical test.

A future real trial can check them as follows:

1. Use an Opulo LumenPnP, three Opulo 8 mm tape feeders in adjacent front-rail slots, standard 7 inch reels loaded
   with 8 mm component tape threaded according to the maker's mounting guide, and the 300 x 420 mm service mat.
   Park the gantry at the back, stop the feeder drives, and check that all three hooks and catches are seated.
   Use the two mat positions in the setup. Record an overhead video and a close side view of the hooks/catches;
   include a millimetre rule and angle reference in view.
2. Mark the middle reel and its hub. Support the feeder body, leave its tape threaded with the sprocket stopped,
   and gently turn the reel in each direction. Record unwinding rotation, drag, slack and any winding play or
   back-driving. This tests whether the one-way-stop approximation is appropriate; do not force or tear the tape.
3. With the body independently supported, compare a reel-face grasp with a body grasp. Record relative reel/body
   rotation and whether load is transferred through the tape. Do not rely on the tape to support the feeder.
4. Perform the body-grasp extraction shown in the simulated demo: press the inner end down, hold it tipped,
   pinch the exposed outer body, lift until the hook clears, and slide out along its plane past the neighbouring
   reels. Lay it on its side on the mat. Check the whole-trial hook/catch rules and the 2 mm / 3 degree end limits
   from the videos; confirm the feeder is inside the mat, hands at least 5 mm clear, and still for 1 s.
5. Record any difference from the simulated reel/tape behavior and revise the approximation before claiming
   physical validation. This procedure has not been run; no hardware result or measured release force is claimed.

## Answers to the review

| Review point | How the task answers it |
|---|---|
| "The feeder bank is mounted on an isolated bench stand rather than the machine where the changeover occurs, omitting surroundings that constrain access." | The bank is now on the machine's own front frame rail, in its first three front slots, with the machine around it at the positions in the maker's assembly CAD: legs, side frame, back frame, bed plate, bottom camera, control box and the parked gantry (setup; `MACHINE` in task.py). |
| "Mount the feeder bank on the pick-and-place machine, or reproduce the machine structures that constrain access, and update the setup and demo accordingly." | Done as above. The rail height, slot blade, slot pitch (15.8 mm, from the CAD) and the bed's 5 mm clearance to the feeders' inner ends now come from the maker. The demo was rebuilt for this access: the inner end is pressed from above over the bed's edge, and the unit leaves toward the reels between the machine's front left leg and the bank. |
| "Also verify the demonstrated reel-grasp extraction with a real rotating reel and threaded tape; represent their effects in the simulation if they change the grip or motion..." | The simulated reel is a separate rotating body with estimated hub drag; threaded tape is approximated by a one-way stop. The demo grips the feeder body. The real-trial check procedure above describes how to check the approximation with named parts; it has not been performed and no physical verification is claimed. |
| "...then provide full-score demos for both lots." | The corrected runtime scores 1.0 in each lot; both trials are included in the combined demo video (Results). |

## Licences

All geometry is modelled from scratch with primitive shapes; no third-party asset is used. The dimensions and positions
come from the maker's public documents and open-source design files, cited above.
