# Fish a wire through a conduit run with a fish tape

A 59 cm run of 1-inch EMT conduit (galvanised steel tubing, 29.5 mm outside, 26.6 mm bore, 1.45 mm wall) is strapped
on two steel channels on the table. From its right-hand mouth, which faces the robot, it runs 12 cm straight away
from the robot, turns left through a 90° sweep of 146 mm centreline radius, and runs 23 cm straight to its left-hand
mouth, which faces left. The tube sizes are the catalogue sizes of 1-inch EMT, and 146 mm (5.75 in) is the
minimum bend radius for 1-inch EMT bent with a one-shot hand bender.
A yellow fibreglass fish tape is already pushed through the run. Its brass tip, with an
eyelet, sticks out of the right-hand mouth, and its handle end sticks out of the left-hand mouth. An orange 6 mm
insulated wire lies on the table in front of the right arm. Its last 16 mm is bare copper, bent down and folded back
into a J. The robot hooks the J through the eyelet at the right-hand mouth while the left arm holds the tape at the
other end. Then the left arm pulls the tape back hand over hand. The wire is drawn through the straight, the sweep
and the second straight, until its end is out of the left-hand mouth.

This is how electricians pull wire into a conduit they cannot reach inside. They fasten the stripped wire to the
eyelet of a fish tape already pushed through the run, and then pull the tape back. Neither arm can do it alone.
The right-hand mouth is only within the right arm's reach, and the left-hand mouth only within the left arm's reach
(offline IK: the left arm ends 11 mm or more short of the eyelet, the right arm 5 cm or more short of the tape beyond
the left-hand mouth). Pushing the wire in by hand is scored 0; neither gripper reaches anywhere near the sweep.

## Ability tested

Ability tested: making a pulling joint between a wire's bare end and a tool's eyelet at one end of a conduit, then
moving the wire through a bent run that no gripper can enter, only by pulling the tool from the other end, so that
the joint stays made the whole way.

## Closest tasks

Each shares some elements with this task (two arms, a cable, a pull); the differences are in what is scored.

- TK-UC3UAQ (pass a cable through a panel grommet): there the gripper pushes the cable's own plug through a short
  hole and the other arm takes it on the far side. Here the wire is never pushed: it is joined to a tape at one
  mouth and moved only by pulling the tape, through 59 cm of conduit with a 90° sweep.
- TK-R6AVUY (rod a blocked duct): there a rod is pushed into a duct to find a blockage and withdrawn. Here the tape
  is already through, and what is scored is a joint made at one mouth that is carried through the bend and out of
  the other mouth.
- TK-EBGGYD (pull a cable out to a length mark): there a cable is pulled out of an open channel to a measured length.
  Here nothing can be gripped inside the run, and the wire must be joined to a tape before any pulling can happen.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (the cable simulation, the pre-shaped starting
  layout and the scripted-demo runner). `task.py` adds three things on top of the kit. It builds the conduit as fixed
  colliders: a 12-sided lining whose flats sit at the 26.6 mm bore, plus the two bushings' flared throats. It adds
  rigid parts to two end plugs: the eyelet on the tape's brass tip and the copper J on the wire's end. At every
  reset, it also lays the tape through the conduit and the wire round its arc on the table.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Pull the orange wire through the conduit with the yellow fish tape: hook the wire's bent copper end through the eyelet on the fish tape's tip at the conduit's right-hand end, then pull the tape back out of the left-hand end until the wire's end is at least 5 cm out of the conduit."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo reads simulator object poses (it is not camera-driven). Scripted demos may read the simulator's object poses to plan. This one plans its motions from the fixed layout and
the arms' own poses. It reads the wire end's and the eyelet's poses from the simulator to place the J in the eyelet,
and it reads the tape's height outside the left-hand mouth before each grip. It does not use the cameras, so it shows the task is feasible, not that a camera policy can do it.
1. The left arm pinches the bare tape 4 cm beyond the left-hand mouth and holds it, so the tape cannot be pushed
   back while the right arm works.
2. Meanwhile the right arm pinches the wire's stiff end where it lies, lifts it and turns it to point at the
   right-hand mouth.
3. It holds the J 8 mm above the eyelet, with the leg on the mouth side of the hole, and corrects its position
   (up to 3 times) from the measured leg position. It lowers the J through the hole, slides it 6.5 mm back so that
   the folded tail goes under the eyelet's outer bar, and lets go (score 0.25).
4. The left arm pulls the tape back hand over hand in four 20 cm strokes, regripping 4 cm beyond the left-hand mouth
   each time. The first stroke brings the wire's end into the right-hand straight (0.5); the strokes then carry it round the sweep and out of the left-hand mouth (0.75), and well past it (1.0).

In the recorded run the J went through the eyelet after one correction of about 1.5 mm. The wire end was in the sweep after the second stroke, past it during the third, and stayed hooked the whole way. Success holds for 1 s and ends the episode at step 883
(~44 s); task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Unintended strategies, selected with `FISH_DEMO=<mode>`. Each was run in the sim and scored by `score()`:

| `FISH_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 (the J is not through the eyelet) |
| `tape_only` | the left arm pulls the tape all the way out without the wire hooked on | 0 (the tape leaves the conduit before any wire is in; the wire never moves) |
| `push` | the tape is pulled out first, then the right arm tries to push the wire's end into the empty right-hand mouth | 0 (tape withdrawn early; the J snagged on the mouth and the end did not get in) |
| `away` | the right arm slides the tape 10 cm further out of the right-hand mouth by its tip (it stays threaded) and hooks the wire on there | 0 (hooked and threaded, but the eyelet is 15 cm from the mouth) |

Partial credit, also run in the sim (`FISH_STROKES` and `FISH_LAST` change the number and length of the pull
strokes): hooked with the tape still held and nothing pulled scores 0.25; two strokes (wire end 33 cm along the conduit, still in the sweep) score 0.5; four strokes with a 10 cm last stroke (end 3 cm out) score 0.75.

A hook-on-the-left-arm strategy is not possible: offline IK puts the eyelet 11 mm or more out of the left arm's
reach. The "pushed by hand" rule (end 2 cm inside while off the eyelet) is checked by offline tests of the scorer on
built wire and tape states, together with a wire carried round the outside of the run, a leg 13 mm off the eyelet
and a hook made 15 cm from the mouth: all score 0.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

Distances along the conduit are measured along its axis (the straight, the sweep and the straight), from the
right-hand mouth. "Hooked" means the J's leg passes through the eyelet's hole. In the sim this is judged
geometrically from the two end plugs' poses. The leg's axis must cross the eyelet's plane within 9 mm of the eyelet
centre, measured along and across the eyelet. A leg through the 14 mm hole crosses at most 5.5 mm from the centre;
a leg outside the eyelet crosses at least 11.5 mm from it. `task.py` reads the end plugs' orientations from the sim
(through a hook on the embodiment's step), because the kit's cable state has positions only.

Two rules are checked at every control step. Once broken, they hold the score at **0** for the rest of the trial
(tracked in module state that resets at every episode start):

- **Pushed by hand**: the wire's end (the middle of the J's leg) was 2 cm or more inside the right-hand mouth while
  off the eyelet for 0.3 s or longer.
- **Tape withdrawn early**: no part of the fish tape (no segment centre and neither end plug) was inside the bore at
  a step before the wire's end had first been 2 cm inside the right-hand mouth.

At the end, the J must be hooked through the eyelet (otherwise **0**). Then:

- **Wire not yet entered** (the wire's end is less than 2 cm inside the right-hand mouth):
  - **0.25** if all three hold: the tape is still threaded through the whole run (handle end beyond the left-hand
    mouth, tip beyond the right-hand mouth, every part between the mouths inside the bore); the eyelet centre is
    within 10 cm (straight line) of the centre of the right-hand mouth; and no part of the wire is inside the bore.
  - **0** otherwise.
- **Wire entered** (its end 2 cm or more inside): the wire must run from its end back through the bore and out of the
  right-hand mouth (not carried round the outside, not in through the left-hand mouth), otherwise **0**. Then:
  - **1.0** if the wire's end is at least 5 cm past the left-hand mouth;
  - **0.75** if it is past the left-hand mouth by less than 5 cm;
  - **0.5** if it is still inside the conduit.
- `distance()`: how far (m, along the conduit) the wire's end still has to travel to be 5 cm past the left-hand
  mouth; 1 m after a latched 0.
- `progress()`: stage diagnostics (hooked, tape threaded, hooked near the mouth, entered, past the sweep, out,
  routed, not pushed, tape not withdrawn early). They do not affect the score.

## What is simplified

- The eyelet head is raised 6 mm above the tape's axis (flat, 20 mm square, 14 mm hole) so the J can be lowered through it; real fish tape eyelets are smaller and sit closer to the tape.
- The wire's bare end is bent into its J before the trial starts. Real copper is bent round the eyelet after it is
  passed through, then twisted and taped. The sim has no plastic bending, so the robot hooks the ready-bent J through
  the eyelet and slides it back so that the fold closes round the eyelet's outer bar. There is no twisting or tape
  wrap: the joint holds by contact.
- Wire and tape are chains of 2 cm rigid segments with bending springs (no twist, no plasticity).
- The bore is a 12-sided lining whose flats are at the true bore radius; the outside of the tube is drawn round.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes about 1 h of wall time on a busy shared GPU (0.4 to 0.5
  steps/s), including the 1–2 min Isaac Sim start-up.
- The layout is fixed with `object_jitter=0`. The tape and the wire are laid by `cableshape` at every reset, the
  same way each time. `VARIATION` in `task.py` lists the seeded ranges a later runner may apply.
