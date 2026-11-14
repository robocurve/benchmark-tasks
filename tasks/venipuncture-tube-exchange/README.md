# Fill a blood tube on a needle already in a training arm's vein and take it off without moving the needle

Instruction given to the policy: `On the practice arm, fill the blood tube, then lay it in the tray.`

Files: `task.json`, `venipuncture_tube_exchange.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`,
two sim controls and an offline self-test), `mechkit.py` (unchanged helper: sliders with friction, per-episode memory),
`README.md`.

## Response to review

Review 1, Scoring (not met): the 2 mm limit measured only axial travel, so a needle displaced sideways or pivoted
could still earn full credit. Changes in this revision:

- **Numeric limits added, over the whole trial.** Besides the 2 mm along the needle, the trial is now latched 0 if
  the needle's hub ever moves more than 2 mm sideways off the needle's start line (any direction across it), or the
  needle ever pivots more than 3 deg off its start direction (any direction). They apply at every moment from the
  start to the end of the trial: before and during the push, while the tube fills, after it is full, while it is
  pulled off and after. 2 mm is about the radius of the cubital vein (3.9 mm across at rest, 4.8 mm under a
  tourniquet; source below); 3 deg swings the tip (9 mm in) about 0.5 mm and the hub about 0.9 mm.
- **Video measurement for the real trial.** A pen mark on the skin at the hub, a paint dot on the hub, and two fixed
  cameras at right angles, both square onto the needle (a side camera beyond the training arm, and a camera above the
  site tilted 25 deg toward the shoulder), with the barrel's 19 mm width as the scale. The start line and hub dot are
  overlaid from the last frame before the robot touches anything; the sideways offset and the barrel's turn are
  combined from both views (root of the sum of squares) and read at least every 5th frame to the end of the trial.
  Full procedure under Scoring rules and in task.json.
- **In the sim, the same gates are in the scorer**, computed every physics step from the holder's pose
  (`needle_pose()`, `_check_side()`); either one latches the score to 0 and stops the fill.
- **Stated plainly in the fidelity note and here:** the simulator constrains the needle to axial motion (a slide on
  its own line), so it cannot show the sideways or pivot failure modes. They are checked in the real trial. The sim
  gates are kept so the scorer is the real-trial rule, but they cannot trip in the sim; `self_test()` shows them
  firing on hand-built poses after the fill.
- **No sim outcome changed, so no re-record.** The scorer change only adds gates that read the holder's pose. Offline,
  the recorded demo's pose log (run with the same code, sampled every 0.25 s) peaks at 0.06 mm sideways and 0.08 deg
  of pivot, and both sim controls stay under 0.06 mm and 0.06 deg. One debug run of the revised task file (no frames)
  computed both gates in the sim and passed: hub sideways at most 0.043 mm, pivot at most 0.084 deg, no latch,
  task_score 1, with the pierce and the tube's removal at the same physics steps as in the recording (2256 and 7426).
  The videos and scores in task.json are from the recording made before this revision.
- Also in this revision (text only, no sim value changed): the context now names medical assistants first (that
  occupation alone is over the 500k bar) and drops the summed figure, and the README lists the sources, or the proxies where
  no direct figure was found, for each force and time.

## Environment

Isaac Lab-Arena 0.3.1 with yam-arena and inspect-robots, as the program's launchable installs them. Before every shell:
`source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/venipuncture-tube-exchange/venipuncture_tube_exchange.py
```

It checks the geometry (needle tip under the skin, stopper through the bore, the squeeze span inside the gripper's
94 mm, the flange wing where the squeeze expects it), asserts the scoring rules on hand-built states (including the
sideways and pivot gates firing on hand-built holder poses after the fill, which the sim itself cannot produce), and
replays every plan (the demo and both sim controls) through the kinematics at 20 Hz, every joint inside the URDF
limits, IK error under 0.5 mm. Expected output:

```
[vp] LATCH needle-sideways: the hub moved 2.10 mm sideways off the needle's start line (limit 2 mm) at physics step 4000, waypoint ''
[vp] LATCH needle-pivot: the needle pivoted 3.10 deg off its start direction (limit 3 deg) at physics step 4000, waypoint ''
[vp] LATCH needle-sideways: the hub moved 3.35 mm sideways off the needle's start line (limit 2 mm) at physics step 4000, waypoint ''
[vp] LATCH needle-pivot: the needle pivoted 8.00 deg off its start direction (limit 3 deg) at physics step 4000, waypoint ''
scoring: 21 cases pass
demo left: 48.4 s plan, worst IK 0.088 mm / 0.013 deg
demo right: 42.4 s plan, worst IK 0.099 mm / 0.014 deg
control a (not the demo) left: 10.0 s plan, worst IK 0.000 mm / 0.000 deg
control a (not the demo) right: 10.0 s plan, worst IK 0.083 mm / 0.014 deg
controls c+b (not the demo) left: 24.0 s plan, worst IK 0.088 mm / 0.013 deg
controls c+b (not the demo) right: 21.7 s plan, worst IK 0.098 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/venipuncture-tube-exchange/venipuncture_tube_exchange.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "On the practice arm, fill the blood tube, then lay it in the tray." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/venipuncture-tube-exchange-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`), otherwise
`score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/venipuncture-tube-exchange/venipuncture_tube_exchange.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "On the practice arm, fill the blood tube, then lay it in the tray." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/venipuncture-tube-exchange-demo
```

The demo is `demo()` in the task file, 48 s of plan. In the sim run it ended on success at about step 830 (41 s), inside the run limit of 1,800 steps: the stopper pierced under a 4.4 N push, the tube went home 7.5 mm and filled to 100 %, it was drawn off and laid in a tray slot, and the needle stayed within 0.55 mm of its start throughout. In a debug run of this revision's task file (same outcome, task_score 1) the scorer logged
`[vp] k=800 needle sideways/pivot so far: hub max 0.043 mm (limit 2), pivot max 0.084 deg (limit 3), latched False`.

## The mechanism (what is physical, what is a rule)

- **Needle in the arm.** Holder and needle are one rigid body on a slide along the needle's own line, anchored in the
  training arm, with 0.5 N of Coulomb friction (tissue drag; a force-capped joint drive whose stick point follows the
  needle every physics step). The vein's lumen is +-2 mm of that slide. Any push or pull on the holder over 0.5 N moves
  the needle; nothing else holds it. **The simulator constrains the needle to axial motion:** it is a slide, so the
  needle cannot move sideways or pivot in the sim, and the sim cannot show those failure modes (a needle levered or
  dragged sideways in the vein). The scorer still reads both from the holder's pose every physics step, as gates for
  the whole trial (hub 2 mm off the needle's start line, needle 3 deg off its start direction); in the sim they stay
  near zero (see the demo), and they are checked in the real trial on video (Scoring rules below).
- **Stopper.** The tube rides a slide joint along the needle's line in the holder's bore (as the bore guides it). Its
  drive is a spring whose anchor stands for friction (mechkit's model): seated at the guide line, the unpierced tube is
  held there; a push that compresses the spring past 4 N (the tube 1.3 mm in) means the stopper has given way, and from
  then on the tube slides on the needle with up to 1.5 N of drag, ramped over the 4 mm of stopper the needle passes
  through, to a stop 8 mm in. Drawn back clear of the needle the drag is the bore's 0.3 N; once the stopper is 2 mm out
  of the holder's mouth the joint is switched off and the tube is free. The anchor moves only while a finger tip
  touches the tube (static friction: with no applied load the tube stays where it is). This is needed in the sim: the
  light holder and tube carry small joint-error forces when a gripper squeezes the holder, which otherwise walk the
  tube along its slide. The drive's force caps and anchor are updated once per control step (20 Hz).
- **Fill.** A counter, not a fluid: it runs (4 mL in 8 s) only while the needle tip is through the stopper and the
  needle is in the vein. The tube's body turns red in fifths as it fills (looks only). On a real training arm the
  artificial blood flows only from the vein, so the fill line checks the needle position as well.
- **Holding the holder.** Read from contacts: a gripper holds the holder when both its finger tips touch it, or one
  touches it and the other the tube (the thumb-and-fingers squeeze of flange and tube end).

Forces and times, with sources:

- Stopper pierce 4 N: no blood-tube figure was found. A 21G needle into vial stoppers takes 2.4-3.4 N (Aptar,
  ONdrugDelivery 2021), ISO 8871-5 cl. 4.1 caps closure piercing at 10 N, and a blood-tube stopper maker specifies
  "puncture strength not exceed 20N" (xindongrubber.com). A higher value would only make an unbraced push move the
  needle more.
- Tissue drag 0.5 N: an in-human venipuncture device measured a resting force of ~0.4 N on the needle (Leipheimer et
  al., Technology 2020, PMC7156113), and subclavian-vein withdrawal friction was 0.160 N (J. Mech. Med. Biol.,
  10.1142/S0219519422500130). A lower drag would only make the task harder.
- Stopper drag 1.5 N: unverified. The maker's ceiling is 15 N ("needle pulling strength"), and 21G friction through
  0.4 mm PU foil is 0.06-0.24 N (GBO/Melab, DIN 13097-4).
- Fill 8 s for 4 mL: unverified for a 21G multi-sample needle. A 23G butterfly set filled 5 mL in 26.7 s on a bench
  rig (Mouser et al., JALM 2017), and a Poiseuille estimate for 21G gives 4-7 s. Bore drag 0.3 N: estimate.
- Vein size, for the sideways limit: cubital vein outer diameter 3.9 mm (IQR 3.4-5.1) at rest and 4.8 mm (4.1-5.7)
  under a tourniquet, by ultrasound in 57 adults (Berger et al., BMC Anesthesiology 2024, 10.1186/s12871-024-02765-6).
  A 2 mm sideways move of the needle is about that vein's radius; a 3 deg pivot about the skin swings the tip
  (9 mm in) about 0.5 mm and the hub (16.4 mm out) about 0.9 mm.

## Scoring rules

- Latched 0, judged over the whole trial (before, during and after the fill, while the tube is pulled off and after):
  the needle ever moved more than 2 mm along its line (either way); its hub ever moved more than 2 mm sideways off the
  needle's start line (any direction across it); the needle ever pivoted more than 3 deg off its start direction (any
  direction; spin about its own axis not counted); the holder let go of after it was first held (no hand on it for
  0.25 s); the stopper never pierced.
- 0.25 pierced with the needle in the vein; 0.5 filled to the mark (>= 90 %); 0.75 that and the tube off the needle and
  out of the holder; 1.0 that and the tube lying in a tray slot, let go and still, the holder still held (held 1 s).

In the sim the needle's travel comes from its slide, and the sideways offset and pivot from the holder's pose (the
hub's face, 16 mm out along the needle, against the needle's start line; the holder's axis against its start
direction), every physics step: `needle_pose()` and `_check_side()` in the task file. Because the sim's needle can
only slide, those two gates cannot trip in the sim; `self_test()` shows them firing on hand-built poses.

In the real trial, from video (as in task.json):

- Marks: a pen mark on the skin at the hub, a paint dot on the green hub.
- Two fixed cameras, 30 fps or more, each with its lens 20 cm from the hub's face (x 25, y 4.55, 6.7 cm above the
  table) and its optical axis along a normal to the needle, so the needle crosses each view square on:
  - Side camera: beyond the training arm, lens at x 45, y 4.55, 6.7 cm up (the hub's height), looking toward the robot
    (-x), along the normal to the needle's vertical plane.
  - Overhead camera: lens at x 25, y 13.0, 24.8 cm up, on a stand or boom above the site, aimed down and back at the
    hub. Its axis is the needle's upward normal in the needle's vertical plane: tilted 25 deg from straight down
    toward the shoulder, so it looks perpendicular to the needle (which descends 25 deg into the arm).
  The two views are at right angles, both across the needle. The mm scale in each view is the holder barrel's 19 mm
  width, which lies in the needle's plane.
- On the last frame before the robot first touches anything, overlay the needle's start line (the barrel's axis) and
  the hub dot in each view (any video player with a line overlay, or free video-analysis software).
- Read every 5th frame, and frame by frame wherever the holder visibly moves, to the end of the trial.
- Blocked frames: where a gripper blocks a camera's view of the hub dot or the barrel's edge, use that camera's nearest
  unblocked frame within 0.5 s (before or after). If there is no unblocked frame within 0.5 s, that interval fails:
  the trial is latched 0.
- Latched if: the hub-to-mark gap along the needle (16.4 mm at the start: 1 inch needle, 9 mm in) is under 14.4 or
  over 18.4 mm; the hub dot is more than 2 mm off the start line, combined from the two views (square root of the sum
  of squares); or the barrel's axis, read along at least 20 mm of its visible edge, has turned more than 3 deg from
  the start line, combined the same way (1.05 mm across per 20 mm along).

## Sim controls

The task file has two control plans besides `demo()` (each was run as `demo` in a debug copy, no frames):

- `controls_push()` (a): nothing holds the holder; the right hand's closed finger tips push the tube in by its end.
  Logged: `[vp] LATCH needle-out: the needle moved +2.02 mm along its line (deeper, through the vein; lumen +-2 mm) at
  physics step 1325, waypoint '[ctl-a] right: push the tube in, holder free'`, task_score 0.
- `controls_letgo_yank()` (c, then b): as the demo up to the tube grip, then the left hand lets go of the holder and the
  right hand yanks the tube out. Logged: `[vp] LATCH holder-let-go: neither hand on the holder for 5 control steps, at
  physics step 4656 (needle +3.19 mm), waypoint 'right: hold the tube (left lets go)'`, task_score 0. In that run the
  needle had already gone deeper (+2.28 mm) as the left hand released, before the yank; the yank itself moved it only
  0.4 mm (it cleared the stopper in about three control steps, faster than the 20 Hz friction update).

## How the demo does it

The left gripper holds the holder's barrel near the hub, closing across it. The right gripper does the push the way a
phlebotomist's thumb and fingers do: one pad behind the flange's upper wing, the other on the tube's end, its closing
axis tipped 13 deg off the needle's line so the flange pad clears the barrel. Closing it pushes the tube onto the needle
with no net force on the holder. It then lets go, grips the tube's body, waits for the fill, eases the tube off (2 mm/s
over the stopper, then 10 mm/s), turns it level and lays it in a tray slot. The scripted demo reads sim state, as the
Guide allows: the left hand's target is corrected every control step against the needle's measured travel, so it keeps
the holder still while the tube's drag pulls on it.

## Dream-1 run

One Dream-1 trial (1,800 steps, 90 s): task_score 0. It moved toward the arm but never pierced the stopper.

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz). The demo plan is 48 s.
- One fixed layout: `-E object_jitter=0`. Dream-1 is hosted and not seeded; the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up.

## Reset between trials

Set the needle back to its start mark (9 mm in), replace the tube with a fresh evacuated one (a filled or pierced tube
has no vacuum left) pushed into the holder to the guide line only, and empty the tray. The needle is sharp: keep people
clear of the training arm while the robot works.

## Assets

All geometry is built from boxes in the task file (no third-party meshes). `mechkit.py` is this repository's helper.
