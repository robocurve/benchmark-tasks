# Slit a sealed envelope open along its top fold with a letter opener, holding it down, without cutting the letter inside

A sealed #10 business envelope (24.1 x 10.5 cm) with a letter folded in thirds inside lies flat on the desk, flap side
up, its top fold on the far side. A 22.9 cm (9 in) steel letter opener lies on the desk beyond it. The instruction asks
for the envelope to be opened along its top edge with the letter opener without cutting the letter inside, and for the
opener to be put down on the desk afterwards.

Opening incoming mail is a listed task of mail clerks (O*NET 43-9051.00, Mail Clerks and Mail Machine Operators,
Except Postal Service: "Seal or open envelopes, by hand or by using machines." ([O*NET][onet])) and an everyday office
and home chore. The [European patent description][opener] describes the familiar pointed hand opener and its use.
The usual way: hold the envelope flat with one hand; with the other slide the blade's point into
the small gap at a corner of the flap, where the flap is not glued, between the two layers of paper; then draw the
blade along the top fold so its edge slits the fold from inside, all the way to the far corner. A letter folded in
thirds (93 mm tall) sits below the fold of a 105 mm envelope, so the point must stay in the band above the letter's
top edge: pushed deeper, it cuts the letter.

## The hand skill, and why two hands

- The blade's point (1.5 mm thick, 3 mm wide) goes into a corner gap 12 mm wide and 4 mm tall, between the two layers.
- For the whole 241 mm the blade is held at a shallow angle across the fold (10 deg in the demo), point inward and
  slightly down, its outer edge pressing on the fold 15 mm behind the point. The point then runs 4-7 mm inside the
  fold; the letter's top edge is 12 mm inside. Too shallow and the point leaves the envelope; too deep and it runs
  into the letter (the wrong way below).
- The fold resists the blade with 1.5 N. The envelope with its letter weighs 80 g and slides on the desk under far less,
  so it slides away with the blade unless the other hand pins it down. That is visible in the scene: the envelope is
  loose on the desk, nothing else holds it.

The task is the task file [`slit_open_a_letter_with_a_letter_opener.py`](slit_open_a_letter_with_a_letter_opener.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the envelope, the letter and the opener are built in these files from boxes.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint. The
envelope (a loose articulation: the two paper layers, the side seams and the bottom fold as its base; the letter on a
stiff slide; 24 pieces of the top fold on sprung hinges) and the opener (one rigid body of several boxes) are built with
`bodykit.py`, which adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, and gives the task file each body's pose, each joint's position and PhysX contact forces
between chosen parts. Every control step the task file reads the contact forces between the opener and each fold piece
and the letter, and sets the fold pieces' hinge targets through Isaac Lab's articulation API, as described below.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Sealed #10 business envelope (4 1/8 x 9 1/2 in, 24.1 x 10.5 cm), white paper, flap side up, loose on the desk,
  centred at x 19, y 0, its long sides along y, its top fold (the flap's fold) on the far side at x 24.25. Inside, a
  letter of about 12 sheets of letter paper folded in thirds (21.6 x 9.3 cm), its top edge 1.2 cm below the fold.
  Envelope and letter are 0.5 cm thick and weigh 80 g. The flap is glued except at its two corners: at each end the
  last 1.2 cm below the fold is open (the band above the letter's top edge, 4 mm tall between the layers).
- Steel letter opener, 22.9 cm (9 in), 40 g: wooden handle 10.5 x 1.5 x 0.9 cm; steel blade 12.4 cm long, 1.5 mm
  thick, 1.6 cm wide at its root, tapering to a 3 mm point, in the plane of the handle's middle. It lies flat on the
  desk beyond the envelope's fold, handle centred at x 30, y -25, point toward +y (the robot's left).
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled:

- The envelope's two layers are rigid plates 0.5 mm thick, 4 mm apart; the letter between them is a rigid block on a
  stiff slide (3 N/mm, within 1 mm) with a compliant contact (3 N/mm), the give of a stack of paper.
- The top fold and the top layer's last 14 mm next to it are 24 pieces 1 cm long, each on a hinge along the fold, held
  shut by a spring. Paper cutting is a stated rule: a piece is slit when the blade's point is inside the envelope (in
  the band above the letter, at most 2 mm past it), the piece before it toward the start corner is already slit (the
  first piece is at the start corner's gap), and the blade pushes on that piece with 1.5 N or more (PhysX contact
  force). The slit piece then swings up 25 deg about its hinge, opening the slit, as a slit envelope's edge gapes.
- The letter is cut when the blade pushes on it with 1.0 N or more (PhysX contact force).

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run.

- 0.25: at least 6 of the 24 fold pieces slit (a quarter of the edge).
- 0.5: at least 12 slit (half).
- 0.75: all 24 slit (the whole top edge, corner to corner).
- 1.0 (success, held 1 s): all 24 slit and the opener put down: resting on the desk (its lowest point within 5 mm of
  the table top), still (its middle moved less than 0.5 mm in the last step), and every point of both grippers'
  finger collision hulls 1 cm or more from it.

Once the letter has been cut (the blade pushed on it with 1.0 N or more), the score is 0 for the rest of the run. Doing
nothing scores 0. A real trial is scored with a ruler laid along the slit (each 1 cm of the fold, from the start
corner, counts while the slit is continuous) and by taking the letter out and checking it for cuts.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=slit_open_a_letter_with_a_letter_opener.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Open the envelope along its top edge with the letter opener without cutting the letter inside, then put the opener down on the desk." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/letter-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The left gripper, closed, presses the
envelope onto the desk near its bottom edge (at x 16, y 9) and holds it there. The right gripper takes the opener's
handle from above, lifts it and turns it so that the blade lies across the fold at 10 deg, point inward and tilted
2.4 deg down, the point level with the middle of the corner gap; it slides the point into the gap at the start (-y)
corner and draws the opener along +y at 1.5 cm/s, the blade's outer edge on the fold, on past the far corner, where
the point comes out through the other gap. The fold pieces are slit one after another as the edge reaches them. It
lifts the opener clear, lays it on the desk beyond the envelope (handle at x 30, y -6) and lets go; the left hand lets
go of the envelope; both arms go back to the ready pose. It reaches success at about step 880 (44 s).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=slit_open_a_letter_with_a_letter_opener.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Open the envelope along its top edge with the letter opener without cutting the letter inside, then put the opener down on the desk." \
  --scorer task_score --max-steps 1080 --store-frames --log-dir ~/yam-eval/letter-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/letter-dream/*.json; do inspect-robots video "$log"; done
```

`LETTER_DEBUG=1` prints, every 5 steps (`LETTER_DEBUG_EVERY=1` for every step), how many fold pieces are slit, whether
the letter is cut and the blade's force on it, the largest force on a fold piece, whether the blade's point is inside
the envelope and where it is in the envelope's frame, the opener's pose and lowest point, how close the nearest finger
is to it, how far the envelope has moved, and where each gripper is and how far it is open.

`LETTER_DEEP=1` runs the demo the wrong way: after the first 6 cm the blade is pushed 1.5 cm deeper into the
envelope, so its point runs into the letter's top edge. In the simulation the blade pushed on the letter with 1.0 N at
step 456 (the letter is cut), after 4 of the 24 fold pieces were slit: score 0.

`LETTER_NOPIN=1` runs the same right-hand moves with the left arm left at its ready pose (nothing holds the envelope).
The fold's resistance pushes the 80 g envelope along the desk with the blade: it slid 19 cm and only 2 of the 24
pieces were slit: score 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: a debug run of the scripted
demo without video (1,080 steps, 54 s of simulated time; success at about step 880) took about 6-7 minutes from start
to finish. The Dream-1 run has a 60-second simulation limit.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A box of #10 business envelopes (4 1/8 x 9 1/2 in) with gummed flaps, letter paper folded in thirds (about 12 sheets per
envelope, to match the 80 g and 5 mm here, or fewer), and any 9 in (229 mm) letter opener with a flat blade about
1.5 mm thick and a blunt point. Seal each envelope with its gum, leaving the flap's corners unglued as they come (check
that the corner admits the blade's point). Mark the envelope's position on the desk (centre 19 cm in front of the
midpoint between the arm bases, long sides across the robot, flap up, fold on the far side) and the opener's (handle
middle at x 30, y -25, point toward the robot's left). Each trial uses a fresh envelope; after it, measure the slit
with a ruler and check the letter.

[onet]: https://www.onetonline.org/link/summary/43-9051.00
[opener]: https://patentimages.storage.googleapis.com/da/46/22/964908216d0b9b/EP0190849A1.pdf
