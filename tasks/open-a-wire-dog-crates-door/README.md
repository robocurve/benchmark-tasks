# Open a wire dog crate's front door: lift and slide back both slide-bolt latches, then swing it open

An 18-inch folding wire dog crate stands on the table with its front door facing the robot. The door is shut and held by
its two slide-bolt latches, one near its top and one near its bottom, both on the door's right-hand edge (the robot's
right); the door is hinged on its left-hand edge and swings out toward the robot. The instruction asks for the door to
be opened fully and let go of; it does not say how.

Each latch is a steel bolt that slides sideways in two guides on the door. Its outer end sits in a wire keeper on the
crate's front frame, which holds the door shut. Its inner end is bent down into a small wire-loop handle that hangs
straight down by its own weight. Next to the keeper the bolt carries a short locking tip that, with the handle down,
reaches back behind the front frame's vertical wire, so the bolt cannot slide. Lifting the handle out and up through
about 90 degrees turns the bolt in its guides and swings the tip down clear of that wire; only then can the bolt be slid
back out of its keeper. Let go, the handle drops again; a bolt that has been slid back stays back. The door opens only
once both bolts are out of their keepers.

The procedure is the maker's. MidWest Homes for Pets' wire crate instructions ([New World crate instructions][nw]):
"Lift the handle 90° so the locking tip will clear the wire on the front panel, then slide the bolt to the left and swing
the door open." Their assembly instructions for another of their crates ([model SL54][sl54]) describe the lock from the
other side: "be sure the bolt slides all the way, so when the handle is in the down position the reinforcing hook on the
bolt is behind the vertical wire." MidWest's [iCrate page][icrate] lists the door's "slide-bolt latch with Locking Tip",
seven sizes from 18 to 48 inches; one buyer's review on the same page says "The door has latches at the top and bottom of
the door". The modelled crate is the smallest of these sizes, 18 x 12 x 14 in (45.7 x 30.5 x 35.6 cm, as listed by
retailers for the 18-inch iCrate), at true size.

What makes it hard for a robot: each handle is a 3 cm wire loop hanging 1 cm in front of the door; the gripper has to
pinch it across its two legs, swing it up through most of a right angle about the bolt's axis (a lift of less than about
67 degrees leaves the tip behind the wire, and the bolt does not move), and slide it 2 cm or more sideways while
keeping it up, then do the same at the other latch 21 cm higher or lower, and then swing a 25 x 29 cm door through 90
degrees about a hinge 25 cm away from where it was held. Two arms are used in the demo because the door's swing
carries the hand that pulls it round toward the other side of the table, out of that hand's reach before 90 degrees:
the right arm works both latches, the left arm pulls the door open by its top handle to about 75 degrees, and the right
arm then pushes it on from the inside, past 90 degrees. The task is the task file
[`open_a_wire_dog_crates_door.py`](open_a_wire_dog_crates_door.py) for the `yam_arena` embodiment (scene, scoring and
the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no third-party
assets: every shape is built from boxes, cylinders and capsules in the task file.

`yam_arena` task files place single boxes, cylinders and spheres, upright, and cannot join them. The crate, the door and
the two bolts are bodies built from several shapes; the door turns on a hinge joint and each bolt slides along and turns
about its own axis on a joint to the door (a "cylinder" joint, added to this copy of `bodykit.py`). Importing `bodykit.py`
hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives `score()` each body's pose.

## What is modelled and what is not

Positions are from the midpoint between the arm bases (x forward, y to the robot's left), heights above the table.

- Crate (fixed): 45.7 x 30.5 x 35.6 cm, front frame in the plane x 31 cm, back at x 76.7 cm, sides at y -15.25 and
  15.25 cm, standing on a 3 cm plastic pan. Its frame wires (5 mm) collide; the side, top and back mesh (3 mm wires 2.5
  cm apart) is drawn only. The door opening is between the front frame's vertical wires at y -13.5 and 13.5 cm.
- Door: 0.40 kg, 25 x 29 cm (5 mm frame, 3 mm vertical wires 2.5 cm apart, two horizontal wires), from 4 to 33 cm up,
  hinged about the vertical line x 31, y 12.8 cm; it opens outward from 0 to 120 degrees (a stop at 120 degrees; the real
  door swings on round to the crate's side) and not inward. Hinge damping 0.05 N m s/rad.
- Bolts (0.025 kg each): 4.5 mm steel rods 8.9 cm long, their axes 1 cm in front of the door at 8.5 cm (bottom) and
  29.5 cm (top) up. Shut, a bolt's outer end is at y -16.8 cm, inside a keeper loop (3 mm wire) at y -16.5 to -15.0 cm
  that stands out of the front frame and wraps the bolt in front, above and below with 2 mm clearance. The handle at
  the bolt's inner end is a loop of the same rod: two legs 3 cm long, 1.7 cm apart (2.15 cm across their outsides), at
  y -9.65 and -7.95 cm, joined at the bottom. The locking tip is a 1.4 cm stub of 4 mm rod at y -14.2 cm reaching back
  from the bolt's axis; with the handle down it stands 2.5 mm to the right of the frame's vertical wire at y -13.5 cm,
  so a bolt pushed toward the hinge moves 2.5 mm and stops. Each bolt slides 0 to 3.2 cm toward the hinge (+y) with a
  drag of 120 N s/m in its guides (standing for their friction) and turns 0 to 95 degrees; its weight hangs its handle
  down. Once a bolt is slid back 2.4 cm or more and its handle drops, the tip stands just inside the door's edge bar,
  which keeps the bolt from sliding back into its keeper while the door swings (in the simulator the door's edge bar
  beside each bolt is a separate body fixed to the door, because a joint keeps its two bodies from colliding).
- The real bolt's guides and the hinge loops are drawn but do not collide (the joints stand for them). The real crate's
  newer "Paw Block" cover over the latch is not modelled. The door's exact size on the 18-inch crate is not published;
  25 x 29 cm fits the 30.5 x 35.6 cm end panel. Whether the 18-inch door has one latch or two is not stated by the maker;
  the two-latch door is modelled, as on the reviewed crate.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. The door
is forced if it swings out past 8 degrees while a bolt was still in its keeper (slid back less than 18 mm) the last time
the door was shut (within 2 degrees); in the simulator that can only happen by pushing a bolt through its keeper's wire.
That sets the score to 0 for the rest of the run.
Otherwise the score is the best stage reached:

- 0.25: one bolt slid back 20 mm or more (its end 2 mm clear of its keeper);
- 0.5: both bolts slid back 20 mm or more;
- 0.75: the door swung open 45 degrees or more;
- 1.0 (success): the door open 90 degrees or more, both grippers' centres 4 cm or more from every point of the door and
  of both handles, the door still (turning less than 0.5 degrees over 1 s) for 1 s.

Doing nothing scores 0. Pulling the door without opening the latches scores 0: each bolt is caught in its keeper.
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_a_wire_dog_crates_door.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Open the dog crate's front door fully, at least 90 degrees, and let go of it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/crate-dream
```

The scripted demo, which reaches full score (the right gripper, tipped forward, pinches the bottom handle across its
legs, swings it up while tipping itself further down, slides the bolt back 3.8 cm (it stops at 3.2) and lets go; then
the same at the top latch; the left gripper pinches the top handle, pulls the door open by it to about 75 degrees,
turning with the door, lets go and rises over the door's top; the right gripper, fingers closed, comes into the open
doorway and pushes the door's inside face on to about 105 degrees; both move away):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_a_wire_dog_crates_door.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Open the dog crate's front door fully, at least 90 degrees, and let go of it." \
  --scorer task_score --max-steps 1150 --store-frames --log-dir ~/yam-eval/crate-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/crate-dream/*.json; do inspect-robots video "$log"; done
```

`CRATE_DEBUG=1` prints, every 10 steps (`CRATE_DEBUG_EVERY=5` for every 5), the door's opening angle, each bolt's slide
(mm) and handle angle (degrees), how close the nearest gripper is to the door, whether the door is still, the stage
reached, whether the door was forced, and where each gripper is.

`CRATE_YANK=1` runs the demo the wrong way: nobody opens the latches; the left gripper pinches the top handle and pulls
the door straight out by it. The door does not move (both bolts are caught in their keepers) and the run scores 0.
`CRATE_NOLIFT=1` checks the lock: the right gripper pinches the bottom handle and pushes the bolt toward the hinge
without lifting the handle; the bolt moves about 3.5 mm and stops at the locking tip, and the run scores 0.

## Runtime per trial

On one RTX PRO 6000 (Blackwell), on a shared machine (other runs on the same GPU at the time): about 5.5 minutes for
the scripted demo while rendering 1280 x 720 video (1,070 steps, 53.5 s of simulated time; 331 s), and about 9.5
minutes for a Dream-1 trial of 2,400 steps at 320 x 176 (565 s). Isaac Sim's first start on a new machine adds a few
minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

An 18-inch two-door or single-door wire crate (MidWest iCrate 1518 or similar, with two slide-bolt latches on the front
door) on the table, its pan in, door shut and both handles down, the crate held in place (clamps or a non-slip mat). A
trial is scored with a ruler and a protractor (or a top photo): each bolt's travel from its shut position (20 mm or more
is out of its keeper), and the door's angle from the front panel (45 and 90 degrees), with the grippers off the door. A
door that came open while a bolt was still in its keeper (bent keeper or bolt) scores 0.

[nw]: https://m.media-amazon.com/images/I/A1bYoMHpgGL.pdf
[sl54]: https://www.midwesthomes4pets.com/wp-content/uploads/INST_SL54-195.pdf
[icrate]: https://www.midwesthomes4pets.com/?p=2859
