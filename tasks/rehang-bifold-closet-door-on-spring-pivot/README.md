# Rehang a bifold closet door that popped off its floor bracket

A full-size two-panel bifold closet door (a louvred pine pair at its listed 6.90 kg) hangs from its spring-loaded top pivot and its guide in the track, but its bottom
pivot has popped out of the jamb bracket on the floor. The door stands folded open in front of the left jamb, its bottom pin
resting on the floor 5.5 cm toward the room and 1.3 cm along the opening from the bracket's centre hole. The instruction
is only: **The bifold closet door has popped off its floor bracket. Put it back so it hangs plumb.**

The knack: the pin cannot be slid back, because the bracket's toothed rails stand 5 mm proud of the floor. The whole door
(two 78-3/4 in panels, 6.90 kg) has to be lifted 5 to 10 mm, compressing its top pivot spring until the door's top nearly meets the
track, carried back about 5.5 cm with its bottom pin 2 m below the hands that lift it, set over a 10 mm slot under the
door where neither the camera nor the hands can see it well, and lowered so the pin drops in at the spot directly under the
top pivot. Dropped straight back, the pin lands 13 mm off that spot and the door hangs out of plumb. Once the wheel is down,
its teeth lock it in place along the slot, so a misplaced pin can only be corrected by lifting the door again.

## Response to review

The review (TK-Z4CTFX, 2026-11-21) asked: "Demonstrate full completion with a realistically weighted door and a robot
embodiment capable of gripping and lifting it; update the simulated mass and provide the corresponding model run if the
embodiment changes."

| Request | Change |
|---|---|
| A realistically weighted door | The door is now a real, listed product at its listed weight: a ReliaBilt 24 x 80 in louvred primed pine bifold, actual 23.63 x 78.75 x 1.375 in, **15.211 lb (6.90 kg)**, for a 24 x 80-3/4 in finished opening ([listing](https://superarbor.io/products/reliabilt-24-in-x-80-in-white-louver-solid-core-primed-pine-wood-bifold-door-hardware-included)). Simulated at 3.45 kg a panel, 6.90 kg the pair (was 2.4 kg). Panel height 78-3/4 in (was 79 in), louvre faces drawn; the opening, track, bracket, start pose and scoring are unchanged. The earlier 9-10 kg was our own estimate for a hollow-core molded pair, not a listing; we changed the door to one whose weight we could verify. |
| An embodiment capable of gripping and lifting it | Two **i2rt YAM Ultra (v2)** arms, the higher-payload member of the same YAM family (still two arms with parallel grippers). From i2rt's own repository ([`yam_ultra_v2.yml`](https://github.com/i2rt-robotics/i2rt/blob/main/i2rt/robots/config/yam_ultra_v2.yml), [`yam_ultra/v2/README.md`](https://github.com/i2rt-robotics/i2rt/blob/main/i2rt/robot_models/arm/yam_ultra/v2/README.md)): DM4340 motors at joints 1-4 (the standard YAM has DM4310s at joint 4), joint 4 gains kp 40 / kd 3, link masses 0.94 / 0.10 / 1.63 / 1.06 / 0.46 / 0.40 kg, and kinematics identical to the standard YAM's to within 5 mm (joint 1 height +5.3 mm, wrist -3.6 mm, flange +4.9 mm). `task.py` applies these to both arms at every reset (`_ultra_arms`): joint 4 gets the DM4340 limits that `yam_arena` already uses for joints 1-3 (27 Nm peak, 5.5 rad/s), and the links get Ultra's masses. i2rt's driver encodes DM4340 torque in a +/-28 Nm range and DM4310 in +/-10 Nm (`i2rt/motor_drivers/utils.py`). Resellers list YAM Ultra at 4 kg nominal payload per arm ([roboticscenter.ai](https://www.roboticscenter.ai/store/product/i2rt-yam-ultra-2-6dof)); i2rt's site gives no payload figure we could open. |
| Gripping | Each finger is now capped at 40 N in the simulation (was 50 N). The door is held by friction alone: two pinches of 2 x 40 N. |
| The demo, at the real mass | Full score twice at 6.90 kg (Validation). The demo now feeds each hand's half of the door's weight forward to its joint targets (`load_torque`, a static model on the URDF), as a load-compensating controller does; without it the position-controlled joints sag under the load and the fingers slip. |
| Margin | The same Ultra arms and demo also reached 1.0 at 7.7 kg and 8.42 kg (heavier louvred pine pairs) in check runs on an earlier geometry. |
| The model run | Dream-1 rerun on the Ultra arms (Validation). |

## Files

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the closet, door, track and bracket, the pivot spring and the bracket's teeth, the scorer (`score`, `success`), the YAM Ultra arm settings (`_ultra_arms`) and the closed-loop `demo_script` |
| `task.json` | The task's details |
| `demokit.py` | The closed-loop demo runner, unchanged |
| `mechkit.py` | The mechanism kit (articulations from plain-text USD), unchanged |
| `simready.py` | Loads NVIDIA SimReady props, downloading them once from Hugging Face |

## Run and score a trial

From this folder, in the environment that has `yam_arena` and `inspect-robots`:

```bash
# a model trial (Dream-1 or another policy), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The bifold closet door has popped off its floor bracket. Put it back so it hangs plumb." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/bifold-model
```

`task_score` is 1.0 when `success()` comes true (the tracker counts its own 2 s hold, `SUCCESS_HOLD_S = 0`), and otherwise
the last step's `score()`. Nothing is randomised: every seed gives the same scene (`object_jitter=0`).

## The demo

```bash
python demokit.py task.py OUT                                                                # the full-score run, with video (GPU)
python demokit.py task.py OUT_checks --no-record --mode none,drag,straight_back,hold,drop_early,onearm   # the checks
python demokit.py task.py OUT_fast --fast --mode full                                         # physics on the CPU, no cameras
```

`OUT/full/result.json` holds the harness's `task_score`; `OUT/full/demo.mp4` and `setup.png` are the video and screenshot.
The demo is a scripted reference, not a policy. It reads only what a camera-based policy could estimate (the door's pose and
its visible joints) and its own arms' joints, never the scorer's state. Both hands, level, pinch the two folded panels
together at their hinged edge, the left 14 cm and the right 34 cm above the door's bottom, as a person takes a door at its
edge with both hands. With the pinch made, each arm's joint targets take on, over 1.5 s, the extra lag that half of the
door's weight and spring (about 38 N a hand) needs at that pose (`load_torque`): the arms' joints are position
controlled, so this is what lets them carry the load instead of sagging under it. Steering by the bottom pin's position, both hands lift until the pin is 8.5 mm up (the door's top is
then close under the track), carry it back over the slot at the mark and lower it in; they then press down a few
millimetres while the fed-forward load ramps back to zero, so that the bracket rather than the arms carries the door, let
go, and back away along the panels. On the GPU
a run reaches full credit (success, after the 2 s hold) at 20.1 simulated seconds (402 steps); `run_limit` is 1,200 steps
(60 s), about 3.0 times that. On a shared GPU a trial ran at 1 to 2 steps a second with cameras (about 4 to 7 minutes of
wall time for the demo, up to about 20 minutes for a 1,200-step model trial).

## Importance and sources

Category: **Societal relatability.** Bifold doors are the stock closet door in North American homes; every major interior
door maker sells them as 24 to 36 in sets with their hardware, and the makers' own instructions describe this exact
operation:

- JELD-WEN, *Installation Instructions for Bifold Doors (JII103)*
  ([Lowe's copy](https://pdf.lowes.com/productdocuments/5f7d7313-4b2d-4b77-aee4-2799f863d26e/61295829.pdf),
  [Home Depot copy](https://images.thdstatic.com/catalog/pdfImages/c6/c683cfbc-5706-4de0-8a4f-21ce3778dab0.pdf)): "Place
  the upper pivot pin in the hole in the bracket and the guide in the track ... Lift the door assembly slightly and drop
  the lower pin into the lower bracket slot. Verify there is a 1/4" clearance between the pivot door and the jamb." For
  adjustment: "Lift assembly slightly until the adjustment wheel clears the slot in the lower bracket ... Be sure the teeth
  lock firmly into bracket when finished." It also gives the pivot holes "1 1/4" from edges" and 1-3/8 in thick doors.
- A bifold hardware set's instructions
  ([Home Depot PDF](https://images.homedepot-static.com/catalog/pdfImages/cb/cb077847-5307-4e99-a07d-5b7ef14344fc.pdf)):
  "place top pivot pin (C) in hole, lift doors (compressing pin spring) and seat bottom pivot into jamb bracket slot";
  "Place jamb bracket tight to floor so that center hole of bracket is plumb with hole in the top pivot bracket (in
  track)"; "Lift door and slide bottom pivot bolt into desired position"; the knob goes "approximately 36-40" from floor".
- Masonite's bifold instructions
  ([Home Depot PDF](https://images.thdstatic.com/catalog/pdfImages/4a/4a074e91-61cf-431b-a99e-99266d852a5e.pdf)): the
  standard finished opening for a 79 in panel is "24" x 80-3/4"", which "allows for a 1/8" (3.2 mm) gap at the top of the
  bifold and a 1/4" (6.4 mm) space on the sides"; "Masonite bifolds are hinged with 1" top track"; "Install the door by
  inserting the top pivot into the pivot bracket in track. Lift the door and seat bottom pivot into jamb bracket."
- Another maker's sheet ([Lowe's PDF](https://pdf.lowes.com/productdocuments/c6606070-9f85-4724-b3fa-9aa0625787c7/12059312.pdf)):
  "With the top pivots inserted, lift upward on the panels and position the bottom pivot into the jamb bracket on the
  floor. Let the bottom pivot sit onto the jamb bracket firmly with the knurled teeth engaged."
- Family Handyman, [How to Fix Bifold Closet Doors](https://www.familyhandyman.com/project/how-to-fix-stubborn-bifold-closet-doors/):
  "After a few years of use, bifold doors often start to sag, scrape against the floor or pop out of their brackets";
  "The bottom pivot often loosens and slips"; "you can either raise the door slightly to shift the pivot in or out".

Limits of this evidence: the makers' instructions establish the operation and the hardware; Family Handyman establishes
that the fault is common. No source gives a count of households with bifold doors; the claim that most adults have met a
bifold door off its track is ours.

Why it is a robot problem: the hands work 14 to 34 cm above the floor on a 2 m door that pivots about a point at its
top, so moving them moves the bottom pin by about the same amount, but the pin is out of sight under the panels; the
lift has a window of about 5.5 mm (the rails need 5 mm of lift, the track stops the door at 10.5 mm), and the pin must go into a
10 mm slot within 4 mm of one spot along it.

### Distinct from

| Task | What it tests | Difference |
|---|---|---|
| TK-LVMJWW (#3188, pending): re-hang a sliding cabinet door, top up into its channel then foot onto the rail | A 29 cm cabinet panel lifted into a deep top groove, then its grooved foot dropped onto a floor rail | A sliding panel with no pivot or spring; here a 2 m hinged pair hangs on a spring-loaded pivot, and the task is a hidden pin into a slot at one spot (plumb), with teeth that lock it |
| TK-ZHRV7N (#1410, rejected): hang a bypass closet door on its track | Hanging a 14 cm model bypass door on rollers | Rejected as a miniature; this door is full size (78-3/4 in), and a bifold on pivots, not a bypass door on rollers |
| TK-V76PZX (#3313, rejected), TK-NVTZYW (#3094, pending), TK-JG9NPW (#166, rejected) | Hanging or lifting doors on removable-pin or lift-off hinges | Butt-hinge pin alignment; no spring pivot, floor bracket or plumb set |

External sets: LIBERO, RoboCasa, BEHAVIOR and SimplerEnv open and close cabinet and room doors; none rehangs a door on its
pivots or seats a pivot pin in a floor bracket.

## Scale and mass

**Scale: 100%.** Nothing is rescaled: the robot works at the door's bottom, where the arms reach, so the door, opening, track
and bracket are full size.

| Part | Real (source) | Modelled |
|---|---|---|
| Panel | ReliaBilt 24 x 80 in louvred pine bifold: actual 23.63 x 78.75 x 1.375 in for the pair, 24 x 80.75 in finished opening (listing linked above) | 200.0 x 29.8 x 3.5 cm each (two panels and a 3 mm hinge gap: 23.62 in) |
| Opening | 24 x 80-3/4 in (Masonite, for 79 in panels; the ReliaBilt listing gives the same finished opening for its 78-3/4 in pair) | 61.0 x 205.1 cm |
| Track | 1 in (Masonite) | 2.54 cm channel |
| Pivot and guide holes | 1-1/4 in from the panel edges (JELD-WEN) | 3.18 cm |
| Jamb gap | 1/4 in (JELD-WEN, Masonite) | 6.35 mm |
| Top gap | 1/8 in after adjustment (Masonite); the hardware adjusts | 3/8 in (9.5 mm), adjusted up so the door can be lifted to clear the rails |
| Knob | 36-40 in from the floor (hardware sheet) | 36 in |
| Door mass | 15.211 lb (6.90 kg) for the pair (same listing) | 6.90 kg (3.45 kg per panel) |
| Bottom pivot bolt | adjustable height (makers' sheets: "slide bottom pivot bolt into desired position") | shank 7.9 mm, so the 78-3/4 in panel seats with its top 3/8 in under the track and its bottom 5/8 in above the floor |

**Mass:** the real door's listed mass, not reduced. Lifted, the door and its spring put about 76 N on the hands, 38 N
(3.9 kg) on each, just under the YAM Ultra's listed 4 kg nominal payload; the joint torques it takes are below.

**Embodiment:** two i2rt YAM Ultra (v2) arms, applied by `task.py` on top of `yam_arena`'s standard YAM (see Response to
review for the sources). What differs from the standard YAM in the simulation: joint 4 is a DM4340 (27 Nm peak, 5.5 rad/s,
kp 40, kd 3) instead of a DM4310 (7 Nm); links 2-5 and the base have Ultra v2's masses; each finger is limited to 40 N.
What is approximated: the kinematics are the standard YAM's, which match Ultra v2's to within 5 mm, and the link inertias
are the standard YAM's. The 27 Nm figures are the motors' peak torque, which the simulation allows for as long as it is
asked. In a logged demo run (402 steps) the door was off the floor for 6.9 s; the right arm's shoulder (joint 2) was above
20 Nm for 7.7 s and at its 27 Nm limit for 1.5 s, its elbow (joint 3) peaked at 25 Nm; the left arm's joints 2 and 3
peaked at 22 and 21 Nm. Joint 4 needed up to 8.5 Nm, more than a standard YAM's DM4310 gives (7 Nm), which is why the
Ultra's DM4340 there matters. A real arm working this close to its peak torque is at the edge of its capability: the
door is about as heavy as these arms can lift.

**Not modelled from a source:** the top pivot spring (300 N/m, 5 N preload when seated: an estimate), the pin's tilt play
at the top (joint limits 5 degrees either side of the starting tilt), the jamb bracket's exact plate and rail dimensions, the wheel (22 mm) and
pin (8 mm, protruding 4 mm), and the closet's depth behind the wall (26 cm instead of about 61 cm, because the bench ends
there; nothing in the task happens inside the closet).

The table is the floor: the closet's floor is the table top (z 0), and the whole door, wall and track stand on it at
their real heights. Nothing is lowered.

## The mechanism, and how the simulator judges each rule

The door is one articulation (`mechkit.mechanism`) fixed at the top pivot:

- `tilt_x`, `tilt_y`: free hinges about two horizontal axes through the top pivot, limits 5 degrees either side of the
  starting tilt (which is 1.6 degrees from plumb): the play of the pin in the top bracket and the track that lets the
  bottom swing out. No spring; gravity and the floor hold
  the door where it is.
- `lift`: a slider along the pivot axis with a 300 N/m spring that pushes the door down off the top pivot pin (the spring
  pushes the pin up into the bracket), 5 N when seated. Its upper limit is where the door's top meets the track (9.5 mm
  above seated).
- `pivot_panel`: the door's swing about the pivot axis, from closed to 95 degrees open. The panel carries the bottom pivot
  bolt (wheel and pin, with a rounded-off tip) as colliders that rest on the floor or on the bracket.
- `lead_panel`: the fold hinge on the panels' closet faces, from flat to folded. Its guide sits in the track between the
  channel's walls as a collider, so the pair folds and slides as a real bifold does.

The bracket is a fixed part: two plate halves either side of the slot and two rails with rounded inner edges, as convex
colliders. **The teeth** are modelled as a lock: once the pin is inside the slot and its tip is within 0.8 mm of the seated
height, the `tilt_x` joint (which moves the pin along the slot) is held where it is (`mechkit.lock`), as meshed teeth hold
the wheel; it is let go once the tip is lifted 1.5 mm above seated. The lock never acts across the slot, where the slot's
walls hold the pin, and never moves the pin.

`Trial.step` runs once per control step. It computes the pin's tip from the door's pose and joints; the guide's position
from the lead panel's pose; the panels' speeds; and the smallest distance from mechkit's 24 fingertip points to either panel
(each taken as its full slab). `score()` and `success()` only read the result. The record that the guide left the track is
cleared only in mechkit's `on_reset` hook.

## Scoring

- **Seated:** the pin's centre within 1.5 mm of the slot's centre line across the slot, inside the slot's length, and its
  tip at most 2.5 mm above the floor (seated it reads 1 mm; standing on the rails it reads 5 mm).
- **Plumb:** the pin's centre within 4 mm, along the slot, of the painted mark under the top pivot (the door then leans
  less than 0.12 degrees).
- **Over the slot:** the pin's centre within 6 mm of the slot's centre line and inside its length.
- **1.0:** seated and plumb; both panels still (under 5 mm/s at the top pivot, under 0.01 rad/s); every sampled finger
  point more than 3 cm from both panels; all continuously for 2 s.
- **0.5:** seated, but not 1.0: off the mark by more than 4 mm, or still held, or moving.
- **0.25:** over the slot but not seated: resting on the rails' inner edges, or held above the slot.
- **0:** anything else, such as the pin on the floor beside the bracket, or on the plate or a rail more than 6 mm off the
  slot's centre line. And 0, whatever the end state, if the lead panel's guide ever left the track (more than 14 mm off
  its centre line).

Possible scores: 0, 0.25, 0.5, 1.0. Lifting, tilting and regrasping the door are not penalised; folding or swinging it is
allowed; the door can end at any swing angle.

## Real trial

**Build.** Use a real 24 x 80 in two-panel louvred pine bifold with its maker's hardware (the ReliaBilt pair above, about
6.9 kg, which comes with track, top pivot bracket, spring top pivot, bottom pivot and jamb bracket), and two i2rt YAM
Ultra arms. Frame a test opening 24 x 80-3/4 in in a freestanding stud wall (2x4 studs, 1/2 in drywall or
plywood skin) screwed to a plywood floor platform, with the robot's bench beside it so that the floor platform is level
with the bench top and the arms stand where the table edge is here. Hang the door to the maker's instructions, set the
bottom pivot bolt so the top gap is 3/8 in, and paint a mark on the bracket plate beside the slot directly under the top
pivot (use a plumb bob from the top pivot bracket's hole). Distractors: a folded hand towel, a small cardboard box inside
the closet and a small waste bin, as in `setup`.

**Prepare each trial.** Lift the door out of the bracket and set the pin's tip on the floor on a tape mark 5.5 cm toward the
room and 1.3 cm away from the jamb from the painted mark, the door folded open about 85 degrees with its guide in the track.
Check the pin is on the mark and the top pivot and guide are in the track. This takes about a minute; nothing is consumed.

**Measure,** before anyone touches the door: seated if the wheel stands on the rails with its teeth engaged and the pin is
in the slot (check that a 2 mm feeler gauge does not pass under the wheel); plumb if a ruler shows the pin's centre within
4 mm of the painted mark; still and hands clear from the video's last 2 s; guide in the track by eye. A level held against
the pivot panel's edge after the reading is a cross-check.

**Safety.** The real trial uses the real door at its real mass (6.9 kg), the mass simulated. Each hand carries about 3.9
kg, close to the arms' nominal payload, so run the arms with their torque limits and gripper force limit (40 N a finger)
set; an arm that cannot hold the door should fail the lift, not force it. The door stays
captive at its top pivot and guide whatever happens at the bottom, so it cannot fall: check that both are in the track
before every trial. Keep people out of the door's swing, and stop the trial if the guide comes out of the track.

## Validation

All runs below are demokit runs on the GPU (PhysX on the GPU, as a model trial runs), with no seed-dependent randomisation,
on this `task.py`: the 6.90 kg door and two YAM Ultra arms with 40 N fingers. The two full demos were recorded with
cameras at 640 x 360; the checks ran with `--no-record`.

| Run (`--mode`) | What it does | `task_score` | Steps |
|---|---|---:|---:|
| full (1) | the two-handed demo described above | **1.0** (success) | 402 |
| full (2) | the same, a fresh run | **1.0** (success) | 402 |
| none | does nothing | 0.0 | 440 |
| drag | careless: pinches the panels and pushes the bottom back toward the mark without lifting; the pin stops against the bracket | 0.0 | 430 |
| straight_back | lifts and seats the pin straight back, 13 mm along the slot from the mark; the teeth hold it there | 0.5 | 596 |
| hold | seats the pin at the mark but keeps both hands on the door | 0.5 | 521 |
| drop_early | cheapest shortcut: lifts, carries the pin back to 6 mm short of the slot's centre line, sets the door down and lets go | 0.25 | 417 |
| onearm | the full demo with the left hand alone | 0.0 | 720 |

In drop_early the pin came to rest on the rails' inner edge over the slot without dropping in: the 0.25 the scoring gives
for "over the slot but not seated". With one hand the arm cannot carry the 6.90 kg door: it stopped 27 mm from the mark,
its IK out of reach. A further full run with joint torques logged also scored 1.0 in 402 steps (the torque figures under
Scale and mass come from it). Check runs on the previous door geometry (79 in panels), with the same Ultra arms, 40 N
fingers and load feed-forward demo, also scored 1.0 at 7.7 kg and 8.42 kg. No run printed a joint blow-up. The recorded demo's top and wrist
camera frames were inspected: the door stays on its top pivot and guide throughout, the hands leave the panels at the end.

One Dream-1 run on the YAM Ultra arms (1,200 steps, 320 x 176 cameras) scored 0.0: its left arm pushed the waste bin and
reached up beside the door's hinged edge without closing on it, its right arm hovered over the towel, and the pin stayed
on the floor beside the bracket. (A Dream-1 run on the previous version, standard YAM arms and the 2.4 kg door, also
scored 0.0.)

## Assets and licences

The closet, door, track, bracket and hardware are authored structural and mechanical parts (plain meshes and convex
colliders) built in `task.py`. The distractors are NVIDIA SimReady props, downloaded at first run from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`consumable_microfiber_towel_folded_small_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_microfiber_towel_folded_small_b01): mass set to 0.12 kg.
- [`box_cardboard_d01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/box_cardboard_d01): mass set to 1.2 kg.
- [`bin_waste_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/bin_waste_a01): mass set to 1.3 kg; its lid joints welded into one rigid body by the loader.

Nothing large is committed: files download at first run. `simready.py` is adapted from the open-source
`cutters-off-pegboard` loader in this repository. `mechkit.py` and `demokit.py` are taken unchanged from another team's
open-source task in this repository, `tasks/heat-shrink-lug/`.
