# Grease a loader's pivot pin from a pail pump without leaking grease at the nipple

On the table stands the pivot of a loader arm: a yellow steel lug with a steel pin through it and, on its top, a
straight grease nipple (DIN 71412 form A: M10x1 taper thread, 11 mm hex, 6.5 mm ball head). To the left stands a
grease pail with a lever pump on its lid; its black hose runs over the table to a jawless push-type coupler standing in
a steel holder cup. The job is to grease the pin through its nipple without leaking any grease at the coupler or the
nipple, and to finish with the coupler back in its holder and both arms back where they started. The task is one task
file, [`grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py`](grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: seating a coupler squarely over a small ball-head fitting and keeping it pressed on against a push
while the other hand works a lever through repeated full strokes; timing the two hands (pump only while the coupler is
sealed); putting the coupler back in its holder.

Why two hands: the push-type coupler has no jaws, so the grease pressure pushes it back off the ball head during every
stroke unless it is pressed on, and the pump's lever is 29 cm from the nipple. Two checks:

- **One hand.** The right takes the coupler and presses it onto the nipple as in the demo; the left never moves. The
  trial scored 0.25: the coupler was sealed, but nothing was pumped.
- **One hand after the other.** The right seats the coupler, lets go and goes home, then the left pumps. The trial
  also scored 0.25. Once let go, the coupler rocked past 8 degrees on the ball within a quarter second, pulled over
  by the hose on its side outlet, and fell off. The strokes then leaked.

## The real parts

- **Nipple:** DIN 71412 form A (straight), yellow-zinc-plated steel, M10x1 taper thread (5.5 mm long, in the boss),
  11 mm hex 3.5 mm tall, 4 mm neck, 6.5 mm ball head; 16 mm long, its top 10.5 mm above the boss.
- **Coupler:** a jawless push-type coupler for ball-head nipples (70 g, 4.8 cm tall). Its zinc-plated hardened steel
  nose is 18 mm across and 12 mm tall. The mouth is a 7.2 mm bore with a 45-degree lead-in (10.4 mm across at the
  face), running 5 mm up to a nitrile seal ring 2 mm thick with a 4.6 mm hole. Above the nose sit a black-oxide steel
  body (13 mm hex, 2.6 cm tall) and a brass swivel with the hose's side outlet. Pressed on, the ball head goes fully
  into the mouth and the seal's edge sits on the ball about 1 mm below its top; the nose's face is then 1 mm above the
  hex.
- **Why the seat tolerances:** the bore leaves 0.35 mm round the ball, so a coupler with the ball in its mouth is at
  most about 0.4 mm off the nipple's axis (the scorer allows 1.5 mm for the simulation's contact give). The seal ring
  sits on the ball like a socket, so it keeps sealing as the coupler rocks, until the nose's face meets the hex's
  corners at about 10 degrees and the seal lifts off the far side of the ball; the scorer allows 8 degrees.
- **Hold-on force:** pushing grease through a free pin bushing takes roughly 30-60 bar. On the seal's 4.6 mm circle
  (16.6 mm²) that pushes the coupler off with 50-100 N, so a person leans about 100 N on it while pumping. A YAM arm
  is rated for about 2 kg (about 20 N), so the simulation pushes the coupler off with 15 N, and the demo's arm leans
  about 20 N on it.
- **How much grease:** the pin's drilled passage and its bushing's groove hold about 1.5 cm³, and the lever pump gives
  about 1 cm³ per full stroke. Four strokes (about 4 cm³) fill them and purge fresh grease at both ends of the bushing.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The pivot, the pail with its pump and the holder
cup are kinematic, fixed where they are put; the lever, the five hose segments and the coupler are loose rigid bodies
made of shapes from `bodykit.py`.

- **Nipple:** a hex (three boxes), a neck and a 6.5 mm sphere, kinematic. The pivot is greasy steel: slick, with
  friction 0.15.
- **Coupler's mouth:** real collision geometry made of 12-sided rings of boxes. They form the bore wall, the 45-degree
  lead-in and the seal ring, with a plug above the seal. Its outside is drawn as a round nose. The lead-in and the ball
  centre the coupler as it is pressed down, and only a coupler whose mouth has gone fully over the ball has its seal on
  the ball.
- **Lever:** it turns on a sprung hinge (a `Hinge` from `bodykit.py`) about a side-to-side axis on the pump head. Its
  spring holds it 15 degrees up, and it presses down 26 degrees at most.
- **Hose:** five stiff 12 mm segments joined by ball joints (`Ball`), from the pail's outlet to the coupler's side
  outlet.
- **Holder cup:** smooth steel (slick, friction 0.15), 4.6 cm across and 1.6 cm tall. A 45-degree funnel, 4 cm across
  at the rim, leads down to a 22 mm seat 3 mm deep, all made of 12-sided rings. A coupler set down within about 1 cm of
  its middle slides into the seat and stands upright. It counts as in its holder with its mouth within 6 mm of the
  middle and at most 6 mm above the floor, tilted less than 35 degrees.
- **Grease pressure:** after every control step the task file checks the lever and the coupler. While the lever is
  pressed more than 6 degrees from rest, the pump is pushing grease. If the coupler's seal is then on the ball (its
  axis within 2 mm of the ball's centre, the seal within 1 mm of the ball's top, tilted at most 20 degrees), the task
  file pushes the coupler back off along its axis with 15 N.
- **Sealed:** the ball's centre within 1.5 mm of the coupler's axis, the seal from 1 mm into the ball's top to 0.5 mm
  off it, and the coupler tilted at most 8 degrees.
- **Shots and leaks:** a stroke is the lever pressed down past 18 degrees; it has to come back to within 10 degrees of
  rest before the next press counts. A stroke reaching 18 degrees with the coupler sealed is a shot, and for each of
  the first four a drawn dab of grease (kept under the table until then) is moved to the gap between the lug and the
  arm's side plates. A leak is the lever pressed past 6 degrees at any moment while the coupler is not sealed. It is
  latched, and a fifth dab shows it: on the nipple's boss if the coupler is within 6 cm of the nipple, otherwise under
  the coupler's mouth.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: full score, which includes both arms back
where they started, with the coupler, the lever and the hose still. Otherwise it gives `score()` at the end of the
episode: 0, 0.25, 0.5 or 1.0 (see `task.json`). A leak removes the greasing credit (0.5 and 1.0) for the rest of the
trial. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grease the loader's pivot pin through its grease nipple from the pail pump, without leaking any grease at the coupler or the nipple. Leave the coupler back in its holder cup and both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grease-dream
```

The scripted demo reaches full score:

1. The right hand comes down round the coupler's hex body in its holder, takes it, lifts it out and carries it over the
   nipple.
2. It lowers the coupler over the ball head and presses it on. It aims 4.7 cm below the seat, so the soft arm leans
   about 20 N on it, and holds it there.
3. The left hand comes down on the lever's grip pad, fingers closed, and pumps the lever down five full strokes,
   following its swing. Then it rises and goes back to where it started.
4. Once the lever is back up, the right hand eases off and pulls the coupler off the nipple. It carries the coupler
   back, steadies it just above the holder cup and lowers it in. Then it lets go and goes back to where it started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grease the loader's pivot pin through its grease nipple from the pail pump, without leaking any grease at the coupler or the nipple. Leave the coupler back in its holder cup and both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grease-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/grease-dream/*.json; do inspect-robots video "$log"; done
```

Set `GREASE_DEBUG=1` to print what the score is built from every quarter second (every step while the lever is pressed
with the coupler near the ball):

- how far the lever is pressed down;
- the strokes and shots so far, and whether a leak has happened;
- where the coupler is: its offset from the ball's centre, its seal's lift off the ball, and its tilt;
- whether it is sealed, on the ball or in its holder;
- where each TCP is.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,200 steps (353 s on the clock, with Dream answering in 0.83 s on average), and about 9 minutes for the scripted demo (698 steps, 34.9 s of simulated time, 555 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-RJSVE3. Its review said:

> Construct validity (not met): A coupler can satisfy the 9 mm seating tolerance while misaligned enough to leak, so four counted strokes need not grease the pivot.
>
> Scoring (not met): The primary score defines greasing by lever strokes and coupler pose rather than requiring actual grease delivery or an intact seal.
>
> What to change: Specify the real push-on coupler and nipple sealing geometry, then revise greasing credit to require grease reaching the pin bushing without leaking at the coupler. Tighten the simulated seating proxy to match that seal and document the approximation in the fidelity note.

What changed in response:

- **Real parts and sealing geometry.** The nipple is now a DIN 71412 form A straight nipple: M10x1 taper thread, 11 mm hex, 6.5 mm ball head on a 4 mm neck, 10.5 mm above the boss. The coupler is a jawless push-type coupler. Its 18 mm nose has a 7.2 mm mouth bore with a 45-degree lead-in, running 5 mm up to a nitrile seal ring with a 4.6 mm hole that sits on the ball's top. Both are modelled at these sizes, with the bore, lead-in and seal as real collision geometry.
- **Tighter seat.** Sealed now means the ball is fully inside the mouth: within 1.5 mm of the coupler's axis, with the seal within 0.5 mm of the ball's top, and the coupler tilted at most 8 degrees. The numbers come from the parts: the bore leaves 0.35 mm round the ball, and at about 10 degrees the nose meets the hex and lifts the seal. The demo seats within 0.4 mm and 4 degrees.
- **Grease delivery and leaks.** Greasing credit needs at least four strokes made through a sealed seat. That is about 4 cm³, enough to fill the pin's passage and bushing groove (about 1.5 cm³) and purge fresh grease. Pressing the lever past 6 degrees while the coupler is not sealed, askew on the nipple or anywhere else, is a leak. It is latched and removes the greasing credit. A real trial checks fresh grease purging at the bushing, and none at the coupler or nipple on a white cloth or in the video. The drawn dabs are disclosed in fidelity as the simulation approximation.
- **Hold-on force.** A real coupler needs 50-100 N (30-60 bar on its 4.6 mm seal), which is beyond a YAM arm's ~20 N rating. During each stroke the simulation now pushes the coupler off with 15 N, up from 2.5 N. The demo arm leans about 20 N on it, and fidelity discloses the real figure.
- **Instruction and title.** The instruction is now goal plus constraints only: grease the pivot pin from the pail pump without leaking grease, coupler back in its holder cup, arms home. The title is method-free, and the category stays Impact on jobs. New demo and Dream-1 videos.
