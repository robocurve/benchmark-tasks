# Raise a corded mini blind hand over hand until its bottom rail is level with a tape mark, and let its cord lock hold it

A corded 1-inch aluminium mini blind hangs fully lowered inside a window opening in a wall section at the back of the
table. Its pull cord hangs from the cord lock under the right end of the headrail, with its tassel. A strip of blue
painter's tape on the wall beside the window marks a height 30 cm above the bottom rail. The instruction asks:
"Raise the blind until its bottom rail is level with the blue tape, and leave it there."

Raising a mini blind by its cord is something most people have done. The blind's maker says how it works
([Levolor, stock metal and vinyl blinds, installation and operating instructions][levolor]):

> To raise or lower the installed blind, pull the cords to the LEFT (Fig. I) to disengage. Gently raise or lower to
> desired position. [...] To lock blind into desired position, pull the cords to the RIGHT (Fig. J).

Another maker's sheet for 1-inch aluminium mini blinds says the same in other words: "TO RAISE BLIND: Pull cord
downward until blind reaches desired height. Release cord and blind locks automatically." and, to lower it, "Move cord
to left pulling down gently until cord lock releases".

The lock is at the right end of the headrail. Pulled toward the blind's centre (to the left), the lock is held open
and the cord runs both ways; hanging straight or swung to the right, the lock's cam catches as soon as the cord starts
back up, so the blind stays where the hand left it.

## The hand skill

- The cord hangs free, 7 mm in front of the slats' front edges: a 3 mm cord (the blind's two lift cords, which run
  side by side through the lock) 17.5 cm long below the lock, with a plastic tassel. It swings when touched. The
  gripper closes on it with its fingers across the cord, so the cord lies along the finger pads.
- 30 cm of rise needs 30 cm of cord, and the arms can hold the cord only between about 44 cm and 14 cm above the table
  (the lock is 49 cm up), so one stroke is not enough. The natural way is hand over hand: one hand pulls the cord down
  and keeps it taut while the other hand takes a new grip higher up, then the first lets go. (One hand can also do it
  in strokes, letting the lock catch between them and taking a new grip each time; a person does either.)
- The pulled cord must stay clear of the rising bottom rail: the rail and the slats it gathers come up behind the
  cord, so the hand pulls down and a little out toward the robot.
- The rail has to stop level with the tape (within 2 cm), and the lock catches about 8 mm below where the cord is let
  go, so the hand has to stop at the right place.
- Then the cord is let go hanging straight (or to the right). Let go while it is pulled toward the centre, the lock is
  open and the blind runs back down.

Two arms are not strictly needed: the reason to use two is the same as a person's, to keep the cord taut through a
long pull. The task tests catching a hanging cord, pulling it a set length in more than one stroke without losing it,
and letting it go so the lock holds.

The task is the task file
[`raise_a_corded_mini_blind_to_a_mark_and_lock_its_cord.py`](raise_a_corded_mini_blind_to_a_mark_and_lock_its_cord.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`blindkit.py`](blindkit.py) and
[`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the wall, the window and the blind are built in these files from boxes, cylinders, spheres and
capsules.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint. The blind
is one PhysX articulation built by `blindkit.py` (prismatic, ball and mimic joints, self-collision on); `bodykit.py`
supplies the shapes and the per-step hooks. Importing them hooks `yam_arena.scene` and the embodiment when the
simulator starts, without changing yam_arena.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, heights above the table top.

- Wall section (painted plywood, fixed to the table): front face at x 29, 11 cm deep, from y -36 to 66, 90 cm tall.
  Window opening 59 cm wide (y -14.5 to 44.5), from 2 to 52 cm up, with a sill board (2 cm thick, 2 cm proud of the
  face) and a pane at x 39.
- Blind: a white 1-inch aluminium mini blind with corded lift, 22.5 in (57.2 cm) wide, such as Levolor's corded 1 in
  aluminium mini blind (sold 23 in x 72 in), shortened to 20 slats with the maker's own shortening steps (bottom rail
  moved up the ladders, lift cords re-knotted). It is mounted inside the opening and fully lowered:
  - steel headrail 57.2 x 3.5 x 2.5 cm (x 30 to 33.5, y -13.6 to 43.6, 49.5 to 52 cm up), the cord lock under its
    right end, the tilt wand hanging at its left end;
  - 20 aluminium slats, 56.6 x 2.5 cm, 8 g each, on two ladders, 2.2 cm apart, the lowest 2.2 cm above the rail;
  - steel bottom rail 57 x 2.5 x 1 cm, 100 g, its underside 3 cm up (1 cm above the sill).
- Pull cord: leaves the lock at x 29.8, y -7, 49.3 cm up, and hangs 17.5 cm, then a white tassel 1.2 cm across and
  3.6 cm long on an 8 mm stem, from 31.8 down to 27.4 cm up.
- Blue painter's tape on the wall face right of the opening, y -21.5 to -15, 1.2 cm tall, centred 33 cm up: 30 cm
  above the rail's underside at the start.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled:

- The rail and every slat sit on vertical slides. Each slat rests on its ladder rung (the slide's lower stop) and the
  rising rail lifts it from below, so the slats gather into a stack on the rail as it goes up (2 mm per slat), as the
  ladders let them.
- The cord is a rigid top length on a ball joint at the lock, on a slide along itself (the pay-out), then nine 15 mm
  lengths on ball joints and the tassel. The top length runs up through the lock into the wall, out of sight, and comes
  out as cord is pulled. A PhysX mimic joint ties the pay-out to the rail's slide one to one: this stands for the lift
  cords running from the rail up through the slats, along the headrail over its cradles and out through the lock. So
  the rail and the slats it carries pull back on the cord, and the cord pulls the rail up.
- The cord lock is a rule applied every control step, as a lower limit on the rail's slide: while the cord's top length
  hangs less than 15 degrees toward the centre (+y), the cam is engaged: the rail can rise freely but not fall more than
  8 mm below its highest point since the cam engaged (the catch). With the cord 15 degrees or more toward the centre,
  the cam is held open and the rail runs both ways.
- Not modelled: the slats' tilt (they stay level; the wand is fixed), the ladder tapes and lift cords between slats
  (drawn only), cord friction in the lock beyond the rail's slide friction (0.3 N), and cord stretch.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the rail's lift, the cord's pose and the points of each finger's collision hulls.

- 0 if the blind ever runs down: the rail falls 4 cm or more within 0.5 s at any moment (latched). A hand lowering it
  more slowly is not a fall.
- At the end, with the cord let go (no finger point within 15 mm of the cord or the tassel) and the rail still (moving
  less than 2 mm) for the last 5 s:
  - 1.0 when the rail's underside is within 2 cm of the tape's centre line (33 cm up);
  - 0.5 when it is elsewhere but at least 5 cm above where it started.
- Otherwise 0 (never raised, still held at the end, or not still).

A real trial is scored with a tape measure from the table to the rail's underside after the robot has let go and 5 s
have passed, and from the trial video (a run-down is plain to see, and whether the cord was let go).

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=raise_a_corded_mini_blind_to_a_mark_and_lock_its_cord.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Raise the blind until its bottom rail is level with the blue tape, and leave it there." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/blind-dream
```

The scripted demo, which reaches full score (open loop from the known layout). Both grippers are level, pointing
forward, fingers opening across y. The right hand takes the cord 42 cm up and pulls it down to 17 cm, 4 cm out toward
the robot (clear of the rising rail), then brings it back under the lock: about 24 cm of cord. The left hand, which has
waited up and to the left, takes the cord 40 cm up, above the right hand; the right hand lets go and moves away low to
the right. The left hand pulls on down to 33.5 cm (4 cm out toward the robot), holds, lets go with the cord hanging
straight (the lock catches about 8 mm lower) and moves away. The rail ends at 30 cm of lift, its underside within 1 mm
of the tape's centre line.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=raise_a_corded_mini_blind_to_a_mark_and_lock_its_cord.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "scripted demo" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/blind-demo
```

Videos: `inspect-robots video <log>.json` in the log directory.

Debug and wrong-way variables:

- `BLIND_DEBUG=1` prints a `DBG` line every 5 control steps (lift, pay-out, the cord's angle toward the centre, the
  lock's state and limit, finger clearance, score).
- `BLIND_WRONG=centre`: the same demo, but before letting go the left hand swings the cord 6 cm toward the blind's
  centre. The cord's top length reaches 15-18 degrees toward the centre (the hand lags its target) and the lock
  opens. Let go, the blind runs down 13 cm (from 32.7 to 19.4 cm of lift; the lock catches again only as the cord
  swings back to straight): the run-down is latched and the score is 0.
- `BLIND_ONE=1`: the right hand alone, in two strokes, letting go between them with the cord hanging straight. The
  lock caught each time (8 mm below the hand) and the rail rose 24 cm: a third stroke would be needed to reach the tape
  (that run scores 0.5). A single hand is slower here but not ruled out, as for a person.

## Runtime per trial

The scripted demo reaches success at step 641 (32 s simulated; 2,400 allowed). On one NVIDIA L40S-class GPU shared
with other runs, a 720-step demo at 320 x 176 took about 6 minutes, and the demo at 1280 x 720 with its videos about
9 minutes. The Dream-1 run (2,400 steps at 320 x 176, sharing the GPU) took about 17 minutes. It scored 0.0: both arms drew
back toward their bases and stayed there; neither touched the cord and the blind stayed down.

## Seeds

None: the layout is fixed (`object_jitter=0`) and the simulation is deterministic for the scripted demo.

## Building it for real

- Wall section: a plywood frame 102 x 90 cm with a 59 x 50 cm opening 11 cm deep, its sill 2 cm above the table,
  clamped to the far edge of the table so its face is 29 cm in front of the arm bases; a clear or tinted acrylic sheet
  as the pane (optional).
- Blind: any corded 1 in aluminium mini blind 22.5-23 in wide (for example Levolor's corded 1 in aluminium mini blind,
  23 x 72 in), shortened to 20 slats with the maker's shortening steps, inside-mounted with its brackets under the
  opening's head. Cut or knot the pull cord so it hangs 17.5 cm below the lock with the tassel on.
- Mark: blue painter's tape on the wall face beside the opening, its centre line 30 cm above the rail's underside
  when the blind is fully lowered.
- Score with a tape measure from the table (or the sill) to the rail's underside 5 s after the robot lets go.
- Cost: the blind about 25-40 USD; plywood and tape.

[levolor]: https://www.blindsgalore.com/pdf/installation/levolor/StockMetalandVinylBlind.pdf
