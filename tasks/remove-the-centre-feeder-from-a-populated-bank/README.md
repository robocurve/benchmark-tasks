# Remove the middle feeder from a packed feeder rail on a pick-and-place machine

**Instruction:** Release the middle feeder, lift its hook clear and withdraw it past its neighbours. Lay it on its side on the blue mat and move both hands clear. Leave the outer feeders seated.

The recorded Dream-1 run uses this instruction.

**Category:** Impact on AI timelines.

## Why it matters

Robot builders assemble their motor-driver, encoder and sensor boards on pick-and-place machines; small makers and labs
use desktop machines such as the LumenPnP, which take the same kind of tape feeders on a rail as production lines. A
board that needs parts not already loaded on the rail means taking feeders off it and putting others on. A robot that runs such a machine between jobs has to do this feeder change itself, at the machine. The
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
- **The demonstrated release and grasp use two hands.** The pressing hand keeps the middle unit turned while
  the other hand takes the part that now stands proud of the bank. Let go, and the reel's weight drops the unit back
  into its catch. This explains the scripted approach, not a proof that every one-hand strategy is impossible. The maker also documents spool-arm removal by hand.
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
the unit is latched by a hook and a sprung catch and falls back into its catch when let go. The demonstrated
approach holds it tipped with one hand while the other takes the exposed end, then lifts it off its hook and slides it out
along its own plane, past two neighbours 0.8 mm away that must stay seated, inside the machine's frame.

## Scene and parts (true size)

Positions are in task.json's `setup` (robot frame): the machine's left outer face is 150 mm in front of the arm bases,
its front rail's centre line 70 mm to the right of the robot's centre line, and the bank fills its first three front
slots, so the robot works at the machine's front left corner.

| Item | Real product | Source | Modelled as |
|---|---|---|---|
| Machine | Opulo LumenPnP desktop pick-and-place machine | maker's assembly CAD (`src/assembly.FCStd` in the open-source machine repository) | fixed boxes at their CAD positions: front frame rail, front and back legs and feet, lower and upper side rails with their guide rails, back frame rail, bed plate (3 mm, starting 85 mm behind the rail so it clears the feeders by 5 mm, as the maker specifies), bottom camera, control box, gantry beam, carriages and toolhead parked at the back of travel |
| Front rail | 20 x 20 mm V-slot aluminium extrusion, 600 mm, the machine's front frame member | assembly CAD: centre line 110 mm above the bench (machine on its leg extensions) | core with its top-face slot, simplified to a 5 mm wide, 4 mm deep channel (the model uses a plain channel; the selected real V-slot profile and its sloped lips must be measured during enrollment) |
| Slot blade | LumenPnP front slot blades (25 slots, x 103 to 498 mm along the rail) | assembly CAD | yellow block 28 mm deep, 45 mm tall on the rail's front face, one 5 x 5.5 x 6 mm peg per slot at a 15.8 mm pitch |
| Tape feeders (3) | Opulo LumenPnP 8 mm feeder with its spool arm | [maker page](https://opulo.io/products/feeder), [mounting guide](https://docs.opulo.io/feeders/4-mounting/mounting/), [overview](https://docs.opulo.io/feeders/1-overview/feeder-overview/), design files (the `opulo-inc/feeder` repository on GitHub; frame-8 envelope 164.8 x 82.8 x 15 mm) | black body 165 long (70 mm in from the rail centre, 95 out), 15 wide and 73 tall (the frame's 83 mm envelope less the bulge under its drive gear, left out), hook nose, clip round the peg, yellow two-tooth sprung lever; body 0.13 kg (estimate) |
| Reels (3) | Standard 7 inch (178 mm) reel of 8 mm component tape, EIA-481 | [maker's v1 spool-arm mounting guide](https://docs.opulo.io/feeders/4-mounting/mounting/) (standard 7 inch spool); 13 mm model width and 0.13 kg model mass are estimates requiring specimen enrollment. | white disc 178 x 13 mm on a revolute joint at the spool arm's hub, 0.13 kg (estimate), 0.3 N rim drag mapped to hub torque; ideal threaded-tape winding stop (see Reel and tape) |
| Mat | Anti-static bench mat, blue, cut to 300 x 420 x 3 mm | any electronics supplier | loose slab, 0.5 kg |

Pinned machine assembly and feeder design sources, including the exact feeder CAD path: [fixture/BUILD.md](fixture/BUILD.md#scene-and-parts-sources). The hook, clip and lever dimensions are photo-based model estimates.

Supplier profiles differ: https://ratrig.com/products/v-slot-2020-standard specifies a 6 mm opening; that is not a measurement of the recommended ABRA rail.

Drawings of the slot blade, peg, rail section and the feeder's interface: `fixture/`. A low-cost build of the rail and
the frame parts that limit access, for a real trial with genuine feeders: `fixture/BUILD.md` (below).

Estimates, each marked as such in `task.py`: the hook, clip and lever dimensions (sized from the maker's photos; the
exact printed geometry is not reproduced), masses, the catch lever arm and spring target, additional reel viscous
drag and tape play. Catch pull and reel rim drag use the phone figures in Reel and tape. Layout: lot A, the
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
`object_jitter=0`. The final scripted trials take about 32 s of simulated time each; the task allows up to 120 s
per trial. Wall time depends on the simulator host.

## Demo

`demo()` is open loop from the published layout (the only thing it reads per lot is the mat's starting position). The
right hand reaches over the bed's front edge and presses the middle feeder's inner end down with one open finger until
its catch lets go and the unit turns about its hook, reel end up. The left hand comes down over the feeders' outer
ends and pinches the middle feeder's body where its outer end now stands proud of the others, while the right finger
still holds the unit turned. It grips the body, not the reel: the reel can turn relative to its body, so a reel grip would
leave the extraction depending on how much load the threaded tape can hold. The left hand commands a 40 mm lift over 2 s with a 5 degree wrist lean to take the hook out of the
rail's slot and holds the unit while the right finger lifts away, then moves it 20 mm out, levels it with a 15 degree
lean of the wrist, slides it out along its own plane until its inner end has passed the neighbours' reels, carries it
over the mat, leans it and lets go so that it settles on its side. The demo uses `tilt.py` for leaned grips, `lots.py`
for the two mat positions and `parts.py` for the jointed parts; they patch the stock embodiment in this process only
and change nothing on disk.

## Results

Frozen `task.py` SHA256: `1489a2c1b6b305d97fe5b8c914773482a44c2fd2a415243680ec5cfcb67dbb66`.
Source 1489a2c1: scripted demo A/B 1.0/1.0; Dream-1 with this instruction A/B 0.0/0.0. In the first trial the catch spring starts without its preload; the second trial starts preloaded.

Two fixed lots, A then B, `object_jitter=0`:

| Run | Lot A | Lot B | Status |
|---|---:|---:|---|
| Diagnostic trace 20261124T003613 | 1.0 | 1.0 | Completed on frozen source |
| Scripted demo 20261124T004031 | 1.0 | 1.0 | Completed on frozen source |
| Dream-1 20261124T014656 (this instruction) | 0.0 | 0.0 | Both hit 2400-step limit; zero errored trials |

Dream-1 completed with both trials reaching the step limit, zero errored trials and no refusal. The videos show both complete lots.
Checked by phone with a technician who works with these feeders, on real feeders with loaded, threaded reels (Nov 2026): the reel turns about 5-15 degrees during removal; the tape stays threaded; catch release about 8 N on a new feeder.

**Simulated motion effect.** Applying both phone resistance figures to the original 22 mm lift produced A 0.0 / B 0.0:
the hook lift stalled and the feeder reseated. The revised motion commands 40 mm lift over 2 s with 5 degree wrist
lean, holds the pressing hand 1 s longer, then returns to the original carry height. This clears the hook in both
trace lots. Both catch resistance and hub drag changed, so this comparison does not isolate either one's effect.
The 5 degree lean also makes the lift reachable; a straight-down 40 mm command failed the offline IK check.
No scoring threshold was loosened.

| Final-source trace quantity (simulation only) | Lot A | Lot B |
|---|---:|---:|
| Reel joint range, rad | -0.02000 to 0.13303 | -0.02001 to 0.14955 |
| Reel excursion, max minus min, degrees | 8.77 | 9.71 |
| Sample duration at winding stop, q <= -0.0195 rad | 4.65 s | 1.40 s |
| Released/still hold at final sample | 1.95 s | 1.95 s |

The simulated reel excursions reproduce the scale of the 5-15 degree turn observed in the technician
extraction check. The winding-stop durations show that the ideal tape constraint participates in the simulated motion;
they are not tension measurements. The rotating reel's incoming wrench includes hub support, friction and constraint
loads. Logged peak catch-lever deflection was 0.0690 / 0.0545 rad. The trace computes spring torque using the intended -0.15 rad target; it does not read back the applied target. These traces do not establish the claimed preload in the first trial. No whole-trial
failure occurred and both neighbouring hooks/catches stayed engaged throughout both traces.

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
- **Reel and tape.** Each reel is its own body on a revolute joint at the spool arm's hub, with hub friction and
  drag, and the threaded-tape winding constraint reacts on the feeder as set out in "Reel and tape" below. The tape strip
  is visual only; tape stretch, tearing and drive compliance are not simulated. The spool arm now has a collider;
  it is part of the feeder body's sampled geometry for clearance, hand-release and mat containment. The demo's grasp holds the body, not the reel.
- **Friction.** The feeders' printed surfaces have a friction coefficient of 0.38 static and 0.30 sliding. The static
  value is the top of the 0.12 to 0.38 range measured between pairs of printed PLA parts (printed PETG pairs: 0.13 to
  0.21) by Santana, Alves and Netto, "Effect of the angle of the laid-up filaments on the surfaces on the coefficient
  of static friction between parts manufactured by extrusion-based 3D printing", Proc. IMechE Part L 235(6), 2021. Source: https://doi.org/10.1177/1464420720978997. This paper measured printed-material pairs, not these feeders or their contacts with aluminium and gripper pads. The
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

## Reel and tape: model and effect on the motion

Checked by phone with a technician who works with these feeders, on real feeders with loaded, threaded reels (Nov 2026): the reel turns about 5-15 degrees during removal; the tape stays threaded; catch release about 8 N on a new feeder. Reel turning drag is about **0.3 N at the reel rim**; catch release pull is lower as the feeder wears in. These phone readings replace the earlier resistance estimates.

The simulation reproduces the observed turn with a rotating reel with hub drag and a threaded-tape winding stop. The tape stays threaded while the reel turns slightly. The demo grips the body so the reel can rotate independently, keeps the feeder in its own plane past the neighbours, and uses the 40 mm lift with 5 degree lean described in Results. The recorded simulated reel excursions are 8.77 / 9.71 degrees.

The call did not specify the catch-pull lever arm. The model conservatively maps 8 N through the tab's maximum
pivot radius: `CATCH_PULL_ARM = hypot(6, 10.3) = 11.92 mm`. Intended closed-stop preload is **0.0954 N m**; with the
estimated -0.15 rad spring target this gives **0.636 N m/rad** stiffness. With that intended target applied, torque grows during opening, and torque
divided by any tab moment arm is at least 8 N at or beyond the closed stop. First-trial application of the target requires the correction and readback stated in Results. This mapping is an engineering
assumption, not a phone-reported torque or a measurement of the inner-end press force. The spring resists release;
release itself requires tooth/blade contact and relative motion. Its reaction acts on the parent feeder.

Each reel is a separate rotating rigid body (estimated 0.13 kg). The 0.3 N rim force at the **89 mm reel radius**
gives `REEL_HUB_FRICTION = 0.0267 N m`, used for both static and dynamic hub friction, with equal reaction on the
parent feeder. Additional viscous drag remains an estimated **0.002 N m s/rad**. At the estimated 70 mm tape
radius, torque/radius is **0.381 N equivalent resistance** (`TAPE_TENSION`); this is not an applied tensile force
in slack tape and not a measured tape tension. All phone figures and model assumptions are named constants near
the top of `task.py`. The 5-15 degree figure is a comparison for traces, never a scoring requirement or rotation clamp.

Threaded tape is an **ideal one-way angular constraint** between reel and feeder: 0.02 rad winding play before a
joint limit makes it taut, with equal reaction on the parent feeder. It allows up to 3 rad unwinding (210 mm at
70 mm radius), treated as slack payout. The winding limit assumes the stopped drive holds the tape. Its strength
is unlimited in the solver: there is no measured tensile limit, back-drive threshold, stretch or tearing model.
The visible strip is not a flexible tape collider, so it cannot predict a tape snag. Trace `reel_wrench` is the
total incoming joint wrench, including support/friction/constraint effects; it is not an isolated tape force.
The upper unwind bound is also a numerical limit, not a real slack capacity.

The demo grips the **body**, keeping reel rotation separate from the pinch. A reel-face grasp could transfer
feeder load through the estimated hub resistance or the ideal tape stop; it is therefore not evidence that a
real reel grasp works. No new waypoints have been chosen to rely on that stop. The maker's
[mounting guide](https://docs.opulo.io/feeders/4-mounting/mounting/) documents removal between mounted neighbours
and an onboard 7 inch reel on the **v1** spool-arm feeder. It does not give a reel torque, unpowered tape-hold
force or evidence that a reel-face grasp is safe. The same guide says v2 has a longer nose; v2 is not assumed to
match this scene.

**Diagnostic sensitivity, simulated only:** inherited base hub-resistance runs completed the same body-grasp
motion in both lots. Increasing the equivalent force scale to 1.0 N (hub torque 0.070 N m) gave A 1.0 / B 0.0.
In the failed B trace, the commanded hook lift stalled and the unit fell back into its catch when the pressing
hand withdrew. This is evidence that success is sensitive to the model/trajectory; one run does not isolate the
reel torque from contact variability. The 1 N value is an exploratory estimate, not a published Opulo back tension.
It must not be confused with cover-tape peel strength. Final-source joint ranges and outcomes belong in Results.

## Real extraction check and fixture enrollment

Checked by phone with a technician who works with these feeders, on real feeders with loaded, threaded reels (Nov 2026): the reel turns about 5-15 degrees during removal; the tape stays threaded; catch release about 8 N on a new feeder. The technician performed the extraction and reported the reel turn and retained tape threading. The simulation represents those effects with reel rotation, drag and the tape stop; the body grip leaves the reel free to turn during lift and withdrawal.

The following procedure is for enrolling the substitute fixture and recording additional specimen-specific details. The substitute frame remains a construction plan.

1. Use genuine v1 8 mm spool-arm feeders on the machine or the substitute rail/frame in `fixture/BUILD.md`.
   Keep the original hooks, catches and mating blades; use adjacent slots, loaded 178 mm reels and threaded tape.
   Record version, tape type, fullness, masses and where spent tape exits; disclose any external tape connection.
2. Mark the middle reel relative to its hub and film both sides with a ruler/angle reference. With the body
   supported and drive stopped, turn gently both ways. Record winding play, unwinding slack and any tape back-drive.
   Measure tangential start/steady force with a spring scale at a measured radius; stop before damage. Do not use
   the simulation's infinite tape stop as a physical acceptance limit.
3. Film the body-grasp extraction: press the inner end, keep it tipped, pinch the exposed outer body, lift the hook
   clear, withdraw along its plane past neighbouring reels and lay on the mat. Record reel rotation at each phase,
   tape taut/slack/slipped/snags, neighbour motion and whether speed, tilt or grip must change. Separately supporting
   the body, inspect what a reel-face grip does; never rely on tape to support it.
4. Check the whole-trial hook/catch and end limits using the scoring section, photos/ruler, angle reference and video.
   Record all failures. Reconcile the phone figures, model mapping and tape law with the trial observations before
   reporting a physically validated substitute fixture. The completed technician extraction check is recorded above.

## Low-cost substitute

`fixture/BUILD.md` gives numbered steps, cut list, sources, genuine-interface requirements and enrollment checks.
`fixture/frame-plan.svg` and `machine-boxes.csv` come directly from `MACHINE`; every machine envelope is retained.
The rail/frame plan is about **USD285.26** (USD262.91 observed product lines plus USD22.35 catalogue hardware), excluding feeders, reels, mat, tools, tax and shipping. Including the USD59 sourced mat and three loaded
reels gives about USD369.76 with already-owned suitable v1 feeders/tools. Adding the current USD495 v2 five-pack
would give USD864.76 and still leaves v1 equivalence unverified. This is not a complete new kit for a few hundred
dollars. Genuine blade delivery is stated as four weeks. Price/stock snapshots and estimates are distinguished in
`fixture/bom.csv`; borrowing suitable existing v1 parts is required for an immediate real trial and is unconfirmed.

## Answers to the review

### TK-H47YS7, original verdict (November 20)

> **Not accepted** (TK-H47YS7).
>
> Construct validity (not met): The feeder bank is mounted on an isolated bench stand rather than the machine where the changeover occurs, omitting surroundings that constrain access.
>
> What to change: The packed-feeder extraction is a useful idea, but the current setup leaves out important features of the real changeover. Mount the feeder bank on the pick-and-place machine, or reproduce the machine structures that constrain access, and update the setup and demo accordingly. Also verify the demonstrated reel-grasp extraction with a real rotating reel and threaded tape; represent their effects in the simulation if they change the grip or motion, then provide full-score demos for both lots.

### TK-GGKDVY, latest verdict (November 23)

> **Not accepted** (TK-GGKDVY).
>
> The machine surroundings are now in. Still open from the earlier review: please check the extraction with real rotating reels and threaded tape and note any effect on the motion. Please give a low-cost substitute (the feeder rail and the frame parts that limit access, with a parts list and build steps) that keeps the real feeders and their catches.

Neither verdict lists additional flags.

| Verdict requirement | Answer and evidence |
|---|---|
| Isolated stand omits access; mount on machine or reproduce structures and update setup/demo | Machine surroundings retained from the previous revision: all 20 `MACHINE` envelopes, rail/blades at CAD positions, 5 mm bed gap, legs, side frame and parked gantry. Setup and demo use this scene. The new fixture plan now reproduces every envelope instead of omitting distant parts. |
| Verify real rotating-reel/threaded-tape extraction; note any effect on motion | Checked by phone with a technician who works with these feeders, on real feeders with loaded, threaded reels (Nov 2026): the reel turns about 5-15 degrees during removal; the tape stays threaded; catch release about 8 N on a new feeder. Reel drag is about 0.3 N at the rim. A rotating reel with drag and the tape stop reproduce the slight turn with tape retained; the body grasp leaves the reel free to rotate during lift and withdrawal. |
| Represent effects on grip/motion | Reel rotation, 0.0267 N m hub friction from 0.3 N rim drag, viscous drag and taut-tape winding reaction are included; spool arm collides. The intended catch preload maps the 8 N phone pull; first-trial applied preload requires correction and readback. Body grasp avoids relying on a reel grasp supported by tape. Snags, stretch, tearing and drive back-drive remain unmodelled. Results must be interpreted within those limits. |
| Full-score demos in both lots | Source 1489a2c1: scripted demo A/B 1.0/1.0; Dream-1 with this instruction A/B 0.0/0.0. In the first trial the catch spring starts without its preload; the second trial starts preloaded. |
| Priced low-cost substitute rail/frame, parts list, build steps, genuine feeders/catches | `fixture/BUILD.md`, `bom.csv`, `cut-list.csv`, `machine-boxes.csv`, `frame-plan.svg` and generator. Real V-slot/genuine blades keep the catch interface. About USD285.26 for rail/frame, catalogue-priced; existing v1 feeders required. Complete new low-cost v1 specimen availability and physical fit remain unverified. |

## Licences

All simulated geometry is modelled from scratch with primitive shapes; no third-party asset is used in the simulation.
The dimensions and positions come from the maker's public documents and open-source design files, cited above. The
substitute build in `fixture/BUILD.md` points to the maker's slot blade design files for printing (route A); the
maker's feeder design files are published under CERN-OHL-W v2. The pinned machine licence names CERN-OHL-W v2
for legacy `pnp/cad` and `pnp/pcb` paths; applying that notice to the current `src` paths is an inference, not an
explicit path grant. The recommended fixture uses genuine purchased blades. Any future exported/printed derivative
must retain the maker's notices and remove its branding under the source terms. No CAD asset is copied into this folder.
