# Ratchet a tie-down strap taut over a crate to the deck's D-rings and latch its handle before the deck is jolted

Instruction given to the policy: `Strap the crate down to the deck.`

Files: `task.json`, `ratchet_strap_latch_load.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`, the
sim controls and an offline self-test), `mechkit.py` (hinges, sliders and per-step rules for yam_arena, imported by the
task file), `README.md`.

## Response to review

This revision answers both earlier verdicts on this pull request.

**Review of TK-72ZFXG, construct validity (the strap ran horizontally round the crate and pressed it toward the
stake board instead of tying it down).** The rig is rebuilt as a real top-over tie-down.
- **Routing.** The strap now runs from a floor D-ring on the deck's left edge, up the crate's left side, across its
  top, and down its right side to a lever ratchet hooked into a second floor D-ring on the right edge. The legs are
  15.7 cm long at 58 deg to the deck. The stake board is gone; the crate stands free on painted marks.
- **What holds the crate.** Only the strap pressing it down. At 60 N in the strap, the two legs push the crate onto
  the deck with about 100 N, so friction holds it against the jolt. The crate is tall and narrow (14 cm high on a
  5.5 cm base), so a strap that is not tight lets it pitch over its front edge.
- **Simulation.** The strap is no longer a band on a slider. It is a chain of rigid webbing links hinged across their
  width: one link on each leg and one over the top, plus the strap end inside the ratchet. The links bend over the
  crate's edges and push down on its top through contact.
  - The tension is the force of the take-up spring (8 kN/m) between the strap end and the wound length. In the
    physics, only the strap pressing on the crate holds the strap end short of the wound length, so a tension reading
    means the crate is being held down.
  - The jolt is a hold-down test: the crate must not slide more than 2.5 mm or tip more than 2 deg.
- **Real rig, reset and scoring** are updated to match (sections below). The load cell now sits in line at the strap's
  fixed end on the left D-ring.
- **Full score:** the scripted demo scores 1.0 on the revised rig. A slack strap fails the hold-down test. Evidence is
  under "Sim evidence".

**Review of TK-AUMWWL (scoring, and clarity/reproducibility).** These answers were added in the previous revision and
are kept here, updated for the new rig:
- **Calibrated 50 N.** The rig no longer counts strokes. An inline S-type load cell at the strap's fixed end reads the
  tension. It is calibrated with dead weights, and the slack mark is set from load-cell readings ("Real-rig
  calibration").
- **Geometry for reproducing the rig.** The README and `setup` give:
  - the strap's full path and lengths (33.3 cm ring to mandrel when taut, plus 12 mm of slack);
  - the D-rings' positions;
  - the ratchet's mounting, its mandrel axis and its handle;
  - the crate.
- **Slide, actuator and end stop.** These are specified in "Real-rig build".
- **Guarding.** Guarding and safe operation of the 20 kg deck are in "Guarding and safe operation".

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py
```

It does three things:
- checks the rig's geometry (the slack strap's spawn pose closes the chain);
- asserts the scoring rules on hand-built states;
- replays every arm's demo plan through the kinematics at 20 Hz (every joint inside the URDF limits, IK error under
  0.5 mm).

Expected output:

```
strap: 33.3 cm D-ring to mandrel when taut (legs 15.7 / 7.2 cm at 58 deg, 10.3 cm over the top), 12 mm slack (top link's ratchet end 11.9 mm up at spawn); grip TCP 5.3-15.3 cm above the table
scoring: 17 cases pass (demo winds 4 teeth: 60 N in the strap)
left: 36.0 s plan, worst IK 0.090 mm / 0.013 deg
right: 36.0 s plan, worst IK 0.089 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Strap the crate down to the deck." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/ratchet-strap-latch-load-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Strap the crate down to the deck." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/ratchet-strap-latch-load-demo
```

The demo is `demo()` in the task file. It is open loop and deterministic.
- The right gripper points straight down with its fingers closing along y. It comes down round the round grip bar,
  closes on it lightly and follows the bar's circle about the mandrel's axis. The grip force is set in the task file
  by the finger targets against the gripper's 5000 N/m finger stiffness: `GRIP` puts each pad 2 mm inside the bar's
  surface on the strokes (about 10 N a pad), and `GRIP_SHUT` 0.5 mm while closing the handle (about 2.5 N a pad), so
  the bar can roll between the pads as the handle turns.
- It makes four full strokes (out to 100 deg, back to 22 deg), then presses the handle shut through its detent.
- It eases off onto the shut bar, lets go, lifts clear and parks while the deck is jolted.

The plan is 36.0 s; in sim the episode ends at about 606 steps (30 s) with the success hold after the jolt, inside the run
limit of 1,200 steps.

The sim controls are the same file with `VARIANT` set:
- `"slack"`: 1 stroke, then latched.
- `"overswing"`: the first stroke swung out to the release position.
- `"knock"`: the crate shoved before strapping.
- `"open"`: 4 strokes, handle left up.

## Videos and screenshot

```bash
for log in ~/yam-eval/ratchet-strap-latch-load-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/ratchet-strap-latch-load-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on the first
  run.

## The rig (from the task file)

Positions in cm from the midpoint between the arm bases (x forward, y left). Heights are above the deck's top.

| Part | Where and what | In `ratchet_strap_latch_load.py` |
|---|---|---|
| Deck section | Steel chequer plate 24 x 32 x 1 cm, centred at x 29, y -11, top 1.2 cm above the table. With its ballast and carriage it moves 20 kg. | `DECK_O`, `DECK_SIZE`, `DECK_KG` |
| Slide | Along x, 15 cm of travel from the rear stop (the start) to the end stop. | `RUN`, `kit.slider("deck_run", ...)` |
| Drive | Holds the deck on its rear stop with 10 N, then pulls it forward with 10 N: about 0.47 m/s^2, reaching the stop at about 0.38 m/s. | `K_DECK`, `FAR` |
| Floor D-rings | Zinc-plated, bolted to the deck, bars 4.4 cm long along x, 0.8 cm up, at x 28.3, y +2.5 (left) and y -24.5 (right). | `A_L`, `A_R` |
| Crate | Pine, ballasted to 1 kg, 5.5 (front to back) x 10 x 14 cm tall, centred at x 28.3, y -11 on painted corner marks. | `CRATE`, `CX`, `CY` |
| Strap | Polyester webbing 3 cm wide, 100 g, centre line at x 28.3. Fixed end looped through the left D-ring; it runs 15.7 cm up to the crate's top left edge, 10.3 cm over the top, then 7.2 cm down the right leg into the ratchet's mandrel. That is 33.3 cm ring to mandrel when taut; the start has 12 mm more paid out. | `L_L`, `L_T`, `L_R`, `GAP` |
| Ratchet | A steel lever ratchet for 3 cm webbing, hooked into the right D-ring, its frame lying along the right leg (15.7 cm, ring to crate edge, 58 deg). The mandrel's axis runs along x, 8.5 cm up the leg from the ring (y -20.0, 7.9 cm up). | `M_UP`, `M`, `U` |
| Handle | Two steel side plates 5 cm apart on the mandrel's axis, 60 g, joined by a round red PVC-sleeved grip bar 1 cm thick and 4.6 cm long along x. Shut (0 deg) it lies back down the leg, the bar 6.4 cm along the leg from the axis and 2.5 cm out from it. It opens up and away from the crate and starts 40 deg open. Its pawl springs hold it where it is left. | `R_ARM`, `CRANK`, `BAR_L`, `BAR_R`, `START_DEG`, `H_FRICTION` |
| Ratchet angles | One tooth (5 mm wound) per full stroke: out past 75 deg, then back under 35 deg. Detent ridge below 12 deg; latches when pressed within 1 deg; releases (pawls lifted) when opened to 130 deg. | `STROKE_OPEN`, `STROKE_BACK`, `RIDGE_DEG`, `LATCH_DEG`, `RELEASE_DEG` |

## Reset between trials

Engage the deck's catch first, then:
1. Pull the ratchet's release, open the handle and pay the strap out to its slack mark: 12 mm past taut in sim, or
   the mark set in calibration step 3 on a real rig.
2. Set the handle 40 deg open.
3. Stand the crate on its marks under the strap.
4. Run the deck back by hand onto its rear stop, raising the drive weight, until the catch holds it.

## Real-rig build

- **Slide.** A profile linear guide (one rail along x, two carriages) under a base plate. The ballast and the
  chequer plate are bolted to the base plate. Hard rubber blocks set the rear stop at the start position.
- **Actuator.** A 1.0 kg weight on a cord over a pulley in front of the stop. This gives 10 N, as in sim. It cannot
  push harder than its own weight, and with 15 cm of run it cannot go faster than about 0.4 m/s. The weight's drop is
  shorter than the cord, so it lands on a pad before the cord goes slack.
- **Catch.** A spring latch holds the deck on its rear stop. It is released by a solenoid that must be powered to
  release, so a power cut or the e-stop leaves the deck held. It plays the part of the sim's trip: the operator, or
  the robot controller's timer, releases it once both grippers have been 3 cm clear and still for 1 s.
- **End stop.** A steel block bolted to the table, faced with a 3 mm hard polyurethane pad. The sim's stop is
  rigid; the pad only stops metal-on-metal ringing.
- **Anchors and strap.** Two floor D-rings (bolt-on, rated far above the loads here) through the chequer plate.
  - The strap is a light-duty ratchet tie-down for 25-30 mm webbing, cut so its loose end reaches the mandrel with the
    slack mark showing.
  - Its fixed end is sewn into a loop. The loop hooks to the left D-ring through an inline S-type load cell (0-200 N,
    eye ends), so the cell lies along the left leg.
  - The ratchet's hook sits in the right D-ring.
- **Marks.** Corner marks for the crate are painted on the deck, with reference marks beside each corner for the
  video measurement.

## Real-rig calibration

In sim the tension is the take-up spring's force. Each full stroke winds exactly 5 mm, and each mm the strap end is
held short adds 8 N. That 8 N/mm is a lumped stiffness for the webbing's stretch, its bedding on the crate's edges and
the mandrel's slack, not a property of the webbing alone. On a real ratchet, the strap wound per stroke changes as
the wrap on the mandrel grows, so the rig does not count strokes: it reads the tension.

1. **Load cell, once and after any knock.** Off the rig, hang known masses from the cell: 1.02, 2.04, 5.10 and
   10.20 kg (10, 20, 50 and 100 N). The readings must be within 1 N. At the start of each session:
   - zero the cell on the rig with the strap slack;
   - check it with the 5.10 kg mass on a cord over a pulley, in line with the leg: 50 +/- 1 N.
2. **Taut means at least 50 N at the fixed end.** Friction where the strap turns over the crate's edges leaves the
   fixed-end leg the less loaded of the two legs, so 50 N there means at least 50 N in the ratchet leg too.
3. **Starting slack.** From fully slack, give full strokes and read the cell after each one. Set the slack mark (where
   the webbing enters the mandrel's slot) so that 3 strokes read under 50 N and 4 read 50 N or more, as in sim (about
   20 N and 60 N). Repeat 3 times.
4. **Handle angles.** Fix a protractor plate round the mandrel's axis. Mark this buckle's real angles:
   - where a stroke clicks a tooth;
   - where the handle locks shut;
   - where it releases the strap.

   The scorer's 1.5 deg "shut" and its release position are read against these marks.
5. **The jolt.** Release the catch with the crate removed and the strap slack. Time the deck over the last 5 cm
   before the stop (two marks, video at 60 fps or more). It must arrive at 0.38 +/- 0.04 m/s; add or remove weight
   until it does.
6. **Hold-down measurement.** Film the crate side-on at 120 fps or more against the reference marks. "Held" means
   its base moves no more than 2.5 mm and it tips no more than 2 deg, measured from the frame at the stop to 1 s
   after it.

## Guarding and safe operation of the moving deck

The deck stores at most about 1.5 J (21 kg at 0.38 m/s). The hazards are the pinch between the deck and the end
stop, the pinch at the rear stop, and the falling weight.

- **Fence.** A fixed polycarbonate guard round the deck's sides and front covers both stops and the pulley. It stands
  as high as the deck's top plus 10 cm. The weight runs inside a closed tube. The top stays open for the robot.
- **E-stop.** One e-stop cuts the robot's power and the catch solenoid together. With no power the catch stays
  engaged, so the deck cannot be released.
- **Limited force and speed.** The gravity drive is limited to its weight (10 N) and its 15 cm run. There is no
  motor to drive the deck harder.
- **Operation.**
  - Release the catch only with the robot parked and no one's hands inside the fence.
  - After the trip, wait for the deck to come to rest on the stop.
  - Engage the catch before anyone reaches in to reset the strap or the crate.
  - Run the deck back by hand only.

## Reach and forces (why this fits the arm at full size)

The crate, webbing, ratchet and D-rings are full-size parts; nothing is scaled down.
- **Reach.** The grip bar's path over the strokes (0-100 deg) keeps the TCP 5.3-15.3 cm above the table, at x 28.3 and
  y -23.1 to -26.9, inside the arm's straight-down reach (about 18 cm at x 20-30). The strap over the crate's top is 15.5 cm up.
- **Real forces.** At 60-64 N, a ratchet spool of about 10 mm effective radius (bare spindle plus wraps) needs about
  0.64 N m. With the 6.9 cm lever to the grip bar and 80 % efficiency, that is about 11.6 N along the stroke at the end
  of the 4th stroke.
  - Over the winding stroke this splits into at most 4.4 N horizontally and at most 11.3 N vertically.
  - Pressing the handle shut through its detent takes about 5 N.
  - Both are within the arm's sustained capability (about 10 N horizontal, about 45 N press).
- **In sim** the handle carries only its pawl-spring friction (0.04 N m) and the detent's push (0.04 N m). The
  winding torque is not applied to it (see fidelity).

## Sim evidence

Recorded pass (the media): the scripted demo scores 1.0 (episode ends by success after 30.3 s), and Dream-1 scores 0
(it ran all 1,200 steps).

These are debug runs of this task file in yam_arena with `DEBUG = True` (the mechanism's event lines), and `VARIANT` set for
the controls. Each line is quoted from the run's log.

| Run | What happened | task_score |
|---|---|---|
| Demo | Teeth at steps 195, 268, 340 and 412. Tension 25 N after 3 strokes and 59 N after 4; the strap end was drawn 12.6 mm into the mandrel, and the top link came down onto the crate. "handle LATCHED at 0.80 deg, 4 teeth, tension 59 N". "TRIP: deck pulled forward; tension 59 N, handle -0.0 deg, latched True". "deck on its stop at about 0.35 m/s". "trip over: crate moved at most 0.3 mm / 0.2 deg, peak tension 59 N". | **1** |
| Control: slack strap (1 stroke, then latched) | "handle LATCHED at 0.86 deg, 1 teeth, tension 0 N", then "LATCH shifted: the crate moved 5.8 mm / 2.3 deg on the deck in the jolt" | 0 |
| Control: handle swung out to its release position on the first stroke | "LATCH released: handle opened to 130.3 deg, pawls lifted, strap paying out", then "LATCH shifted: the crate moved 2.8 mm / 2.6 deg on the deck in the jolt" | 0 |
| Control: crate shoved back at the top of its front face before strapping | "LATCH knocked: crate moved 4.4 mm / 1.8 deg before the trip" | 0 |

So every fail latch fires in sim: released, knocked, and not held down (shifted).

The hold-down test separates a taut strap from a slack one. A taut strap (59 N) held the crate within 0.3 mm and
0.2 deg. A slack one let it move 5.8 mm and tip 2.3 deg. Development runs of the same strap and jolt agree:
- no strokes: 9.2 mm / 3.6 deg;
- 2 strokes (still 0 N): 2.5 mm / 2.1 deg;
- 4 strokes but the handle not latched: 0.3 mm / 0.2 deg, score 0.25.

The other partial-credit case (latched, but a gripper left on the handle) is covered by the offline self-test.
