# Put the laptop on the laptop stand

**Instruction:** Put the laptop on the laptop stand.

Someone has propped their open laptop on a stack of three books. Its screen shows an illustrative upload window
with a progress bar; this is a static image, not a functioning upload. The transfer is performed with the lid open. An empty wooden laptop stand stands to the right.
The laptop has to be lifted off the books, carried and set on the stand.

This is a familiar office and home handling skill: transfer an open laptop by its thin base while keeping it open. The handling constraint is how it is held. On an open laptop the screen is the part that stands up and is easiest to get a
gripper around, and lifting it by the screen puts the whole base's weight on the hinges and the display panel and
lets the heavier base swing and fall. The maker's user guides say: "When picking up your open computer, hold it by
the bottom. Do not pick up or hold your computer by the display." (Lenovo ThinkPad T590 / P53s user guide,
"Cleaning and maintenance";
download.lenovo.com/pccbbs/pubs/t590_p53s/html_en/en/Cleaning_and_maintenance_(topic)_T0000683157.html; the same
sentence is in the ThinkPad L530, G460 and other guides.) The instruction does not mention the screen. Nobody is in
the scene.

What the robot has to do with its hands: hold the 0.97 kg laptop by its base, lift
it off the books with both arms together, carry it to the stand and set it down level, with the lid staying where it
is. The base is 10 mm thick and lies flat, so a parallel gripper cannot take it from above: both books and stand are
narrower than the laptop. The stand is also shorter front to back; the books are longer. The laptop is shifted
forward to expose its front edge, where it can be pinched top and bottom.

## Run

From this task folder in the program's yam_arena launchable:

```bash
laptop_instruction='Put the laptop on the laptop stand.'
inspect-robots run --embodiment yam_arena --policy dream \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$laptop_instruction" \
  --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir logs/dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 --instruction "$laptop_instruction" \
  --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir logs/demo
```

Trial k uses lot k mod 2, A first. `--epochs 2` runs both layouts and reports their mean. Set `TASK_LOT=A` or `B` to pin one. The submitted videos concatenate the complete A and B recordings in that order, with layout labels and no omitted captured frames; the screenshot shows A. Use `object_jitter=0`.

Offline checks: `python near_misses.py` (every score level, both sides of each limit, and the shortcuts).

## Files

- `task.py`: scene, scoring, the step hook (whole-trial rules, let-go distance and stillness hold) and the scripted
  demo.
- `parts.py`: multi-shape rigid parts with joints (the lid's hinge). `lots.py`: per-trial layout lots. `tilt.py`:
  grip rotations for the scripted demo only. Each patches the stock embodiment in the task's own process; nothing is
  changed on disk, and `tilt.py` never affects Dream-1 or scoring.
- `near_misses.py`: scoring counterexamples.

## Scene (all from task.py)

World frame: x forward from the line through the two arm bases, y to the robot's left, z up. The desk is the
station table, representing the top of a thin protective pad around the books and stand. Pad compression is not modelled; the existing desk contact plane and scene geometry are unchanged.

| object | real product, true size | in the simulator |
| --- | --- | --- |
| Laptop | Lenovo ThinkPad X1 Nano Gen 3, 13 in: 293.2 x 208.0 x 14.4 mm (non-touch, Lenovo PSREF); weight "starting at" about 0.97 kg (Lenovo; reviews weigh 0.96 to 1.0 kg) | loose; a base 293.2 x 208.0 x 10.0 mm, 0.60 kg, and a lid 4.4 mm thick, 0.37 kg (the split of thickness and mass is an estimate) on a hinge along the base's back top edge. The lid starts open 110 degrees from closed (20 degrees back past upright). The hinge holds it there with up to 1.5 N m (a spring of 20 N m per radian capped at 1.5 N m, standing in for the friction hinge; 13 in laptop hinges hold roughly 0.5 to 1.5 N m, an estimate). Keyboard, touchpad and a red TrackPoint are drawn on the base; the screen shows a white upload window with a part-filled blue progress bar. Friction 0.6 static, 0.5 dynamic (estimate) |
| Books | three hardcover books, 235 x 155 x 35 mm each (any 6 x 9 in hardcovers of that thickness) | fixed; the physical stack is bonded to itself and to the bench as below; stacked flat to 105 mm, lengthwise front to back |
| Laptop stand | a plain beech riser, built from the drawing below | fixed; top board 150 mm front to back, 160 mm wide, 18 mm thick, on two 18 mm side boards; 120 mm tall overall |

Layout (cm; forward, left; angles counterclockwise from straight ahead, seen from above):

| | laptop's middle | laptop turned | books | stand |
| --- | --- | --- | --- | --- |
| lot A (screenshot) | 41.0, 12.0 | 0 (keyboard toward the robot) | front edge at 36.1, centred 12.0 left | front edge at 36.1, centred 14.0 right |
| lot B | 41.5, 10.8 | 6 | same | same |

In lot A the laptop's front edge is 5.5 cm in front of the books' front edge and its sides reach 6.9 cm past the
books' sides. Both supports are narrower than the laptop; the stand is also shorter front to back. The laptop's
forward offset leaves its front edge free to be held top and bottom. In lot B the front-edge midpoint is about
49.4 mm in front of the books' front edge.

### Laptop stand drawing (all mm, beech, glued and screwed)

    top board     150 x 160 x 18      (front to back x side to side x thick)
    side boards   2 x 150 x 102 x 18  (front to back x tall x thick), flush with the top's left and right edges
    overall       150 deep, 160 wide, 120 tall; all edges eased 1 mm

### Stand assembly and fixed-support mounting

Parts: the three beech boards in the drawing, PVA wood glue and four 3.5 x 35 mm countersunk wood screws. Fit the two side boards below the top, flush with its side and front/rear edges. Their centre lines are 71 mm either side of the top's centre. Drill four clearance holes through the top at front/back offsets +/-50 mm and side offsets +/-71 mm from its centre, with 2.5 mm pilot holes into the side-board ends. Glue the joints and install the screws flush; cure according to the glue label. Check the assembled 150 x 160 x 120 mm envelope and ease exposed edges 1 mm.

1. Use a level rigid wood/laminate worktop 1100 mm front-to-back by 1300 mm side-to-side, centred at x 360 mm,y 0 in the arm-base frame (edges x -190/+910,y -650/+650 mm). Fit a nominal 1 mm dense closed-cell neoprene sheet flat over it. Its upper face is the common height datum; the model's table plane represents that pad top. Compression and impact absorption are not simulated.
2. Secure the three closed books together with thin acrylic transfer adhesive film, no foam carrier, at most 0.10 mm per bond layer: two 30 x 180 mm strips per cover interface, lengthwise, centred 40 mm either side of the width centre. Bond the bottom book to a rigid plastic shim and the shim to the worktop inside its footprint. Bond each stand foot to a separate rigid shim and the shim to the worktop using 12 x 140 mm strips inside the 18 x 150 mm foot. Adhesive may mark surfaces; use sacrificial books. Keep every film/shim edge at least 2 mm inside its footprint, without tabs or exposed adhesive.
3. Cut the neoprene away only under the book footprint and stand feet. Choose hidden shim thickness allowing for the actual bond layers so the assembled book and stand tops are 105 and 120 mm above the surrounding pad, respectively, within 0.5 mm. Do not add padding to their upper faces. This is a proposed assembly tolerance, not a measured result.
4. Align the published support front edges at x 361 mm, books centred y +120 mm and stand y -140 mm. All mounting material remains hidden below and inside the support volumes, leaving the front overhang and finger approaches clear. Check the actual assembly's clearance before a physical trial.
5. Add witness lines across book interfaces and support/worktop positions. Check heights, marks, bonding and pad flatness before each trial, and witness lines afterward. Invalidate a physical trial if a fixed support shifts, separates or rocks; repair/reset it before retrying. This proposed attachment has not been built or strength-tested and does not establish resistance to arbitrary robot forces.

## Scoring (exactly as task.py)

Whole-trial rules are sampled every 0.05 s and never undone. The trial scores 0 once:

- two distinct fingertips contact opposite display faces within the 1 mm geometric tolerance, with their contact witnesses within 10 mm of one another along the display plane, including its free rim; this is the observable display-grip proxy, whether or not the grip lifts anything;
- the laptop has lid contact and no hand support against its base underside, and is either unsupported by books, stand or padded desk, or its base middle has risen more than 5 mm during that lid-contact episode; this catches a unilateral display hook, partial display lift or display-only carry;
- the lid turns more than 25 degrees from its start, or the base tilts more than 25 degrees;
- the laptop descends during an observed interval without base-hand or surface support: downward speed over 0.02 m/s or height loss over 2 mm between samples, including a landing or catch after an observed free interval;
- the laptop leaves the desk (its base origin more than 2 cm below the top).

A one-sided lid brush is allowed alongside normal base support. Controlled set-downs on the padded desk, pauses and regrips are allowed. A laptop left on the desk scores 0 by the end-state rules. Base support means hand geometry within 3 mm of the underside, not just a hand near an edge. Surface support uses the actual support footprint, 3 mm above / 2 mm below its top as contact tolerance. Fast held lowering is allowed; speed alone does not prove a drop.

The predicates use measured laptop/arm body-link geometry; fixed-support states are declared by the scene and validated for finite poses and velocities, not independently sensed. Missing or malformed states permanently invalidate credit and are logged separately from a detected handling failure. Contact is a geometry proxy, not a force or damage measurement. At 20 Hz a release and landing entirely between observations can be indistinguishable from opening at first contact; the scorer does not invent a drop from that ambiguity. The hull face samples (3 mm on tips, 5 mm on hands, 8 mm on forearms) also limit contact resolution. Native traces and close boundary cases must be checked before treating this as an accepted revision.

Otherwise, at the end of the trial:

| score | state |
| --- | --- |
| 0 | still on or partly on the books, or lifted less than 10 mm off them |
| 0.25 | lifted clear of the books (the base's underside 10 mm or more above their top, or no part of it over them in the top photo) but not over the stand |
| 0.5 | the base's middle above the stand's top outline seen from above and the middle of its underside no lower than the stand's top, but not meeting 0.75 (held over the stand, or resting on it off the marks below) |
| 0.75 | on the stand the right way round, held or let go: turned at most 15 degrees from square (keyboard toward the robot); its front edge 20 to 90 mm in front of the stand's front edge, measured at the base's middle (every placement in that range keeps its centre of mass over the stand, a centred laptop overhangs 29 mm); its middle within 25 mm of the stand's middle side to side; the middle of its underside -2 to +3 mm from the stand's top (the negative tolerance allows contact give in simulation); level within 5 degrees; the lid within 25 degrees of where it started |
| 1.0 | as 0.75, every sampled hand point at least 30 mm from the laptop, and no base or display corner moving more than 1 mm for 1 s |

The simulator also gives 0 for the trial if the objects or the arms cannot be read at any step (fail closed). The
episode ends 1.5 s after the full score first holds. Trials on lots A and B are averaged.

How a person scores a real trial: use side/front video to distinguish an opposing display grip or display-only lift from a brush, and to see whether the base is supported during lowering and release. A visible free fall is a drop; a controlled padded-desk set-down is allowed. Use the hinge angle card and phone level for the 25 degree limits. Use a calibrated close 1080p top/side video with a 1 mm scale for corner stillness. Use a tape measure for stand overhang (20 to 90 mm), side offset (25 mm), turn (15 degrees), the top photo for the stand outline, and a 30 mm gauge for hand clearance. Final stillness must last 1 s. Contact/force and events wholly between video frames remain measurement limits.

`near_misses.py` preserves all final-placement boundaries and adds direct geometric pinch/brush/edge tests in both lots, mixed base support plus lid contact, unilateral lid hooks, controlled desk rest and completion, held fast lowering versus unheld falling, a drop between supports, first-contact release, and permanent failure on unreadable state. The trajectories are offline scorer checks, not new physical demonstrations.

## The scripted demo

`demo()` reads only the lot's published layout (where the laptop lies and which way it is turned), visible to the
top camera at the start. It is open loop: fixed waypoints from that layout, no object poses read back. Both arms
work together, with matching timings:

1. each arm comes in from the front, leaned 80 degrees forward from straight down, its fingers opened 40 mm and
   closing nearly up and down (10 degrees off vertical), 90 mm to its side of the laptop's middle;
2. it slides its fingers over and under the base's front edge until the pads reach 43 mm in (12 mm short of the
   books) and closes on the 10 mm base (about 25 N a finger);
3. both raise the front about 85 mm. The 0.97 kg laptop is held at its front edge, about 160 mm in front of its centre of
   mass, and turns nose up in the pinch: its back edge stays on the books until the front is about 70 mm up, then
   it hangs about 15 degrees nose up. The arms wait 1.2 s, then carry it to above the stand;
4. both lower it until the base rests on the stand (the back edge first, then flat), open, draw straight back
   toward the robot and return to rest.

The default demo keeps its base grips far from the display. The existing `LAPTOP_DEMO=naive` variant attempts a side-edge display grab. Its 0/0 result alone does not establish a forbidden grip, because it may fail to clamp the panel. [The aligned-grip control](controls/README.md) adjusts only that test controller's closing direction to the display normal and adds read-only diagnostics in a separate copy.

### Revised native results

The frozen runtime was run in Isaac Sim with two epochs, zero object jitter and the unchanged instruction:

| Run | A | B | Result |
| --- | --- | --- | --- |
| Scripted reference |1.0|1.0|success after 550/549 control steps; first full score 26.05/26.00 s; both completed within 1200 steps |
| Dream-1 |0.0|0.0|each trial reached the 1200-step limit; evaluation completed without errors |
| Aligned display-grip negative control |0.0|0.0|the opposing-fingertip zero latched at 8.15 s in each 260-step control trial |

Every expert observation had readable required state, no handling zero and no unsupported interval. Hands supported the base through carry and lowering; the stand supported it at release. The mean expert score is 1.0 and Dream score 0.0. Both submitted videos contain complete A then B recordings, with layout labels only. The negative-control copy, its patch and its 13 s limit are separate from the full-score reference controller.

These are Isaac Sim results. The physical mounting recipe and robot execution have not been built or tested. Contact-force equivalence, thin-lid sampling and events wholly between 20 Hz observations remain limits.

## Real trials

Use a powered-off laptop of this model with a printed still image of the upload window over the screen, or a dummy of the
same size and masses (two plywood plates on a friction hinge, weighted to 0.60 and 0.37 kg, the screen face
printed). The dummy's friction hinge holds the lid at any angle and starts to turn at 0.5 to 1.5 N m (2.4 to 7.2 N pulled
at the lid's top edge with a spring scale). Use the specified 1 mm neoprene pad and hidden support mounting. Controlled rests on that pad are allowed; observed drops remain zero. Pad compliance and impact protection have not been measured or simulated. Reset: put the laptop back on the books at the lot's
published middle position and yaw, with the lid at 110 degrees on the angle card; reset the pad and laptop pose.
The front-edge midpoint is 55 mm before the books in A and about 49.4 mm before them in B.

## Limits of the simulation

- The friction hinge is a capped spring: pushed hard it gives and springs back, where a real hinge stays put. Any
  lid movement over 25 degrees scores 0 either way.
- The laptop is two rigid slabs; ports, rubber feet and the screen's flex are not modelled. Display or hinge damage
  is not simulated: opposing display grips and display-only support are geometric handling rules.
- The pinch holds the laptop about 15 degrees nose up; a person would hold it level from below with spread hands.
- The model fixes the supports. The proposed concealed mounting reproduces that assumption; a physical trial is invalid if its mounting shifts. Attachment strength is untested.
- Friction values and the base/lid split are estimates.
- Two lots stop a single memorised replay; a policy that memorised both layouts is not ruled out.

## Assets and licences

No third-party assets. Every shape is built in `task.py` from boxes.
