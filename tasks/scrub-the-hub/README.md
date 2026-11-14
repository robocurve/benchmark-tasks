# Scrub the IV connector's hub for 15 seconds before attaching the flush syringe

Four files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. `yamkit.py` spawns the modeled props: visible
meshes with simple box colliders underneath. It patches `yam_arena.scene` in memory at import and changes no files. The
syringe is a loose yamkit `Mechanism`: its barrel is a free body and its plunger a link on a 6 cm slide along the
barrel's axis. `task.py` moves the articulation's root onto the barrel (`_free_root`). `flushkit.py` models the saline
(see "The saline and the plunger force" below): the volumes in the barrel, the primed line and the graduated cylinder,
the flow's resistance and the stopper's glide on the plunger's slide, and the cylinder's saline level. `guardkit.py`
(trimmed to its one check) judges the grip on the pad by contact: both fingers' own shapes, from the gripper's collision meshes at their present opening,
touching the pad, with the opening stopped by its 3.5 cm width.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/scrub-the-hub/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Flush the IV line on the training arm." --scorer task_score --max-steps 1500

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

The scorer remembers what happened during the run: `success()` and `score()` are called after every control
step. It adds up the steps the pad spends rubbing the connector's access face, and the syringe's barrel or tip coming
within 2 mm of that face before 15 s of scrubbing sets a flag that nothing later clears. Seating the syringe means its
luer tip touching the access face: the tip end from 0.5 mm short of the face (the resting gap between two parts) to
pressed 3 mm in (the colliders' give), within 1.5 mm of the connector's axis and lined up within 5 degrees, for 0.5 s
in a row. A tip hovering off the face, however close, is not seated. The sim's connector is solid, so the tip pressed
square on the port stands in for the luer slipped into the valve; a real trial checks it with a gentle tug on the
barrel. Seating is latched once reached. The flush is scored on the modeled saline, by the real trial's rule: at least
5 mL gone from the barrel and at least 4.5 mL collected in the graduated cylinder, with at most 0.5 mL of the
difference lost (spilled at the tip). Saline pushed while the tip is seated enters the primed line, so the same volume
runs out into the cylinder; pushed while it is not seated, it spills. Drawing the plunger back while seated pulls the
line back (the sim counts the net volume pushed through). The full flush is latched once reached.

`python test_task.py` replays step-by-step histories offline (no simulator), including rubbing the connector's top
instead of its face, a leaning pad whose far corner passes the face while the face itself is 4 mm short, a syringe
left 8 mm off the hub, a tip hovering 0.8 or 1.5 mm off the port (not seated), a tip touching or pressed 1 mm in
(seated), a plunger pushed before the syringe is seated, a push split between the tray and the seated hub, a seated
flush finished with the tip pulled off (spilled), and the computed plunger forces. For the grip it checks a hand holding the pad
by its far end (counts), a hand closed to the pad's width with its fingertips 5 mm off it, one finger on the pad, an
open hand around it and an empty fist on it (none count). Each scene reset by the simulator (counted by
yamkit) starts a new history.

## Real-world build

An IV training arm (a nursing-skills forearm whose practice veins run out of its upper end as tubing, for example
the Laerdal Multi-Venous IV Training Arm, 270-00001) on a padded armrest, with a 22 G x 25 mm catheter (BD Insyte
Autoguard, 381423) taped into a practice vein. A small-bore extension set (B. Braun 471991, 6 in) runs from the catheter
around the arm's right side to a split-septum needleless connector (BD Q-Syte, 385100). Tape the connector to the bench
along y at the position in the setup, its open end facing the robot's right. The vein's outflow tube runs across the
bench and down into a 25 mL polypropylene graduated cylinder (Nalgene 3662-0025: 0.5 mL graduations, 147.5 mm tall;
the sim draws only its 5 mL marks) standing on the bench at the cylinder's position in the setup, left of the armrest.
Large 70% isopropyl alcohol prep pads in foil packets, opened and folded once to about 6.5 x 3.5 cm, and a prefilled
10 mL 0.9% sodium chloride flush syringe (BD PosiFlush normal saline syringe, 10 mL fill in a 10 mL syringe, 306546;
0.5 mL graduations) on a 15 x 15 cm tray. The line is not connected to anyone.

Before each trial:
1. Prime the line: push saline through the connector until it drips from the outflow tube, then empty the cylinder.
   The line is now full, so every mL that later enters the connector comes out into the cylinder.
2. Prime a new flush syringe: uncap it, hold it tip up and push out the air until saline reaches the tip (the plunger
   ends just under the 10 mL mark). Lay it on the tray.
3. Dry the connector's access face and put a dot of water-soluble ink on it.

Score with:
- the video and a stopwatch for the scrub time (only time the pad rubs the face) and for whether the syringe touched
  the face before 15 s of scrubbing;
- the pad, which must show the ink, and the access face, which must be clean;
- a gentle tug on the barrel for seating (the syringe stays on);
- the flush: at least 5 mL gone from the barrel by its mL marks and at least 4.5 mL in the cylinder, read at the
  meniscus to 0.25 mL. The 0.5 mL margin covers the reading and drops left in the tube. Less than 4.5 mL collected,
  or more than 0.5 mL of what left the barrel missing from the cylinder (saline running out at the hub rather than
  into the line), is not a flush: the trial scores at most 0.5. The sim scores its modeled volumes by the same rule.

Safety: no needle and no patient are involved; the saline is sterile and harmless, and spills are wiped up. Nothing
is pressurized beyond what a thumb puts on a syringe.

### The saline and the plunger force

The saline is a volume and flow-resistance model (`flushkit.py`), not particle fluid.

- Volume. The primed syringe starts with 10 mL in its barrel (10 mL over the 6 cm stroke), the line and the training
  arm's vein start full and the 25 mL graduated cylinder starts empty. Plunger travel pushes saline out of the barrel.
  While the luer tip is seated on the connector, that saline enters the primed line and the same volume runs out of
  the outflow tube into the cylinder; while it is not seated, it spills at the tip. The video shows the barrel's
  saline as a real syringe does, by the dark stopper against the mL marks, and the cylinder's as a blue column whose
  top follows the collected volume (a slide inside the cylinder with no collider, set each step). The column is not
  visible in the camera frames, even at 1280 x 704: the top camera looks down into the clear cylinder, and 5 mL
  stands only about 2.4 cm at its bottom. The collected volume is in the scorer, not on the video. Spilled saline is
  counted, not drawn.
- Force. The plunger's slide carries the force a real primed syringe needs:
  F = A x dP + glide, with A the stopper's area (14.5 mm bore, 1.65 square cm) and dP the Hagen-Poiseuille pressure drop
  8 mu L Q / (pi r^4) summed over the connector, the extension line and the catheter, with saline's viscosity
  mu = 1.0 mPa s (water is 1.00 mPa s at 20 C; 0.9 % saline is about 2 % more). The flow term is a damper on the slide
  (A^2 R = 377 N s/m when seated), applied by PhysX's own drive. The glide is a friction against the plunger's motion,
  applied the same way as a damper of 2.5 N divided by the plunger's speed (averaged over about 0.5 s, and capped at
  1,250 N s/m below 2 mm/s so a plunger at rest stays put). Unseated, only the luer tip's 1.8 mm bore resists (under
  0.02 N of flow force).

| Part | Length | Inside diameter | Pressure drop per mL/s | Source |
|---|---|---|---|---|
| 22 G x 25 mm catheter | 25 mm | 0.648 mm | 5.8 kPa | BD Insyte Autoguard 22 G (product 381423): 0.648 mm ID, 25 mm |
| Small-bore extension set | 18 cm | 1.0 mm | 7.3 kPa | B. Braun small-bore set, 15.2 cm with 0.2 mL priming volume (so at most 1.29 mm inside, hubs included); 1.0 mm and 18 cm taken as the conservative case |
| Split-septum connector | 2.6 cm | 1.1 mm | 0.7 kPa | BD Q-Syte (385100): straight, open fluid path, 0.16 mL priming volume; the bore is an estimate |
| In series | | | 13.8 kPa | |

| Flush rate | Pressure drop | Force from the flow | Plus stopper glide (2.5 N) |
|---|---|---|---|
| 0.5 mL/s | 6.9 kPa | 1.1 N | 3.6 N |
| 1 mL/s | 13.8 kPa | 2.3 N | 4.8 N |
| 2 mL/s | 27.7 kPa | 4.6 N | 7.1 N |

What the sum leaves out (not modeled in the sim). Hagen-Poiseuille assumes laminar, fully developed flow. In the
catheter, the narrowest part, the Reynolds number is about 1,000 at 0.5 mL/s, 2,000 at 1 mL/s and 3,900 at 2 mL/s
(turbulent), and the flow's entry length is longer than its 25 mm. The entry and exit losses there come to about 2 to
3 velocity heads (1.2 kPa each at 0.5 mL/s), and lifting the saline from the connector to the outflow's end in the
cylinder's mouth (about 12 cm) adds 1.2 kPa. Together they add about 0.6 to 0.8 N at 0.5 mL/s (about 4.2 to 4.4 N
in all) and about 1.7 to 2.5 N at 1 mL/s (about 6.5 to 7.3 N). The 2 mL/s row is a lower bound. The training vein and
outflow tube are wide and add little.

The glide force is the least certain number. No published measured glide force for a 10 mL prefilled flush syringe
was found (the FDA 510(k) summaries for these syringes list the break-loose and sustaining force tests only as
equivalent to a predicate), so 2.5 N is an estimate (a judgment for a siliconized 10 mL stopper, not a measurement); every 1 N more raises each row by
1 N. Before trials, check the setup: press the primed syringe's plunger on a kitchen scale held behind the thumb press,
at about 0.5 mL/s, and confirm it stays under about 500 g.

The demo flushes the way nurses are taught for a peripheral line, push-pause (pulsatile): seven pushes of about
0.8 mL, each over 1.6 s, with 0.4 s pauses (one hospital teaching sheet describes "10 short 1 ml boluses with short
pauses in between"). In the demo run the plunger moved 3.2 cm in 14.2 s, 5.4 mL into the cylinder and none
spilled: 0.38 mL/s on average and at most 0.53 mL/s over any 1 s. The force it met in the sim averaged 3.2 N and
never exceeded 3.9 N (computed for those rates: 3.4 and 3.7 N), within the YAM's press of about 5 N. With the losses
the sum leaves out, a real push at the demo's rate needs about 4.2 to 4.9 N, so the margin is under 1 N and rests on
the glide estimate: the kitchen-scale check decides it.

## Category

Impact on jobs: registered nurses (more than 3 million US jobs) and licensed practical nurses (more than 600,000)
access and flush IV lines many times a shift.

## Runtime and seeds

- Demo: 1,284 control steps (64 s simulated), including a 20 s scrub and a 14 s push-pause flush; it ends on success.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
