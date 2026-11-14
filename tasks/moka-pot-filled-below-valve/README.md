# Make a moka pot ready to brew: water just below the safety valve, the coffee basket in, the top screwed on snug

A 3-cup aluminum stovetop moka pot stands taken apart on a gas hob: its empty water chamber (the base, with the
brass safety valve on its side toward the robot) is already on the cold burner grate, its upper chamber stands on the
counter to the left, a glass carafe holding 175 mL of water stands to the right, and the funnel basket, filled level
with ground coffee, stands in an espresso cup behind the carafe. "Get the moka pot ready to brew." means doing what
anyone who makes coffee at home on one knows: fill the base with water up to just below the safety valve, never over
it, set the basket in, then screw the upper chamber on until it is snug. The order matters: water poured onto the
basket runs over the grounds and off the rim, and the basket cannot go in once the top is on. Emptying the whole
carafe in (19 mL over the valve), leaving the basket out, and setting the top on without screwing it tight are the
tempting wrong answers.

## How it differs from nearby tasks

- The board's tamp-and-lock-the-portafilter task: an espresso machine's portafilter is held level, tamped and locked
  in with a quarter turn. Here nothing is tamped (a moka pot's basket is never tamped), there is no machine, and the
  core is a fill level capped by a safety valve, a basket that has to go in between the water and the top, and a
  multi-turn screw joint whose tightness is scored.
- Our kettle-fill-between-lines: also a pour stopped inside a window, but there the window is two printed lines and
  the closing act is a hinged lid and a switch. Here the upper limit is a safety part (the valve's bore, nothing
  printed), and the pot has to be closed by screwing a free upper chamber onto the base with regrasp strokes, scored
  for being on straight and turned down to the seat, against a hard stop.

## Spec

| Item | What it is |
|---|---|
| Instruction | "Get the moka pot ready to brew." (the rule is never stated) |
| Who | Anyone making coffee at home on a stovetop moka pot (category: Societal relatability) |
| Rule | Water up to just below the safety valve (not over it, not more than 1 cm under it), then the coffee basket set in, then the upper chamber screwed on snug over it and let go; the carafe set down off the stove |
| Why | Over the valve, the valve stands in water and can no longer vent steam if the funnel clogs (the pot can over-pressure); far under it, too little water for the grounds makes weak, under-extracted coffee; a loose or crooked top leaks steam and hot coffee at the gasket |
| Traps | Emptying the carafe (175 mL, the valve is at 155.8 mL); leaving the basket out or putting it in before the water; tipping it on the way (the grounds fall out); setting the top on without screwing it; turning it one turn and stopping |
| Water | `pourkit.py` flow model (copied from the kettle task, with its `accepts` hook for a covered mouth) |
| Screwing | Scored from the upper chamber's turning on the base, credited with twistkit's signed wrist-roll budget (see Scoring) |

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

| Object | Type and size | Material | Position |
|---|---|---|---|
| Hob plate | 14 x 14 x 0.4 cm with a black burner cap 5.6 cm across; fixed | stainless steel | centered (27, 5) |
| Burner grate | four diagonal fingers (1.2 cm wide, 1.4 cm tall, from 2.2 to 6.6 cm off the axis) and a ring 13.2 cm across; fixed | cast iron | on the plate |
| Moka pot base | 3-cup water chamber, 7.2 cm tall, 8.2 cm across at the foot tapering to a 6.6 cm threaded collar (top 1.2 cm); inside it steps from 7.2 to 6.6 to 6.2 cm across (floor 4 mm up, steps at 2.4 and 4.4 cm); in a fixed holder on the grate, so it cannot turn, tip or move (in a kitchen the cook's other hand steadies it) | cast aluminum | standing on the grate, axis at (27, 5), foot at z 1.8 |
| Safety valve | brass hex 1.3 cm across with a pin, its center 5.0 cm above the foot; bore 8 mm across, bottom edge 4.6 cm above the foot (155.8 mL) | brass | on the base's side toward the robot (-x) |
| Upper chamber | 8.7 cm tall, 7.0 cm across on its lower body, 8.0 cm at the top, with a lid knob, a small spout and a black handle 45 degrees off the x axis; a skirt 1.2 cm deep (female thread) under it, 8.5 cm across, with eight round ribs inside 2 mm clear of the collar, so once down over the collar it stays centered on it; 200 g; free | aluminum, black plastic handle | standing on the counter at (24, 19) |
| Carafe | cylinder 4.9 cm across, 13.5 cm tall, 4.4 cm inside, 200 g | glass | standing at (23, -12) |
| Funnel basket | flange 6.5 cm across and 1.5 mm thick, cup 6.1 cm across tapering to 4.6 cm at its perforated floor 2.0 cm down, stem 1.0 cm across and 3.8 cm long; 40 g with its coffee; free | stainless steel (a replacement funnel) | standing in the espresso cup |
| Ground coffee | 15 g, level with the basket's flange, drawn as a dark-brown disc | | in the basket |
| Espresso cup | 5.4 cm across, 5.0 cm tall, 4.2 cm across inside its mouth; fixed. The basket's floor rests on its rim and its stem hangs inside | white ceramic | at (36, -7) |
| Water | 175 mL in the carafe, none in the base | | |

In the base the basket hangs by its flange on the rim (the base's wall inside the collar is modeled as 24 thin posts,
so the cup and stem go down inside it), its stem ending 1 cm above the floor, and the upper chamber screws down onto
the flange. Assembled the pot is 16 cm tall, as a real 3-cup pot.

## Scoring

`success()` records the run every step, after `pourkit` has moved the water.

| Outcome at the end | Score | Why it is worth that |
|---|---|---|
| More than 155.8 mL in the base (above the bottom of the valve bore) | 0 | The valve stands in water and cannot vent steam if the funnel clogs: the one unsafe outcome |
| Under 30 mL in the base (nothing to brew; includes doing nothing) | 0 | Nothing was done that brews coffee |
| 30 to 122.5 mL and the pot not otherwise ready (no seated basket with its coffee under a snug top) | 0 | A short pour alone is neither safe-and-right nor a pot ready to heat |
| 30 to 122.5 mL (more than 1 cm under the valve: weak coffee), basket in and top screwed snug over it | 0.25 | The pot would brew, but a fifth less water than the 15 g dose needs makes weak, under-extracted coffee |
| 122.5 to 155.8 mL, basket not in the base with its coffee: left out, propped crooked, or tipped on the way | 0.25 | The safety-critical fill is right, but there is nothing to brew |
| 122.5 to 155.8 mL, basket in, upper chamber not screwed snug over it: not on, crooked, put on before the basket, or not screwed down to the seat (450 degrees) | 0.5 | Water and coffee are in, but a loose or crooked top leaks steam and hot coffee at the gasket, so the pot cannot go on the heat |
| Filled, basket in, top snug, but over 5 mL spilled, the carafe not set down upright, let go (no gripper touching it) and off the stove, or a gripper still touching the upper chamber | 0.75 | The pot is ready, but the job is not finished: a spill wets the hob, a carafe still held or left on the plate can tip or be heated, a base off the grate sits off the flame, and a pot still held has not been let go to heat |
| All of it (where the arms end up is not scored) | 1.0 | Ready to brew and safe to light the burner |

Possible scores: 0, 0.25, 0.5, 0.75, 1.

Definitions and why each number:

- Valve line, 155.8 mL: the bore's bottom edge, 4.6 cm above the foot (4.2 cm of water). Water above it stands in the
  valve. Low line, 122.5 mL: 1 cm lower, a fifth less water than the dose; the 3-cup basket's 15 g of coffee then
  brews strong and under-extracted, the usual "weak coffee" complaint. The band is 33 mL wide (8 mm of level).
- Basket in: its flange's bottom within 2 mm of the rim (resting on it, not propped on something), its axis within 4 mm
  of the base's (the modeled cup, a band 5.6 cm across, can sit 2.9 mm off center in the 6.2 cm opening) and within 5
  degrees of it.
- Coffee: the grounds fill the 2 cm cup to its brim, so a basket tipped past 45 degrees for two control steps running
  (one step can be a physics pop) has lost them; this is latched for the rest of the run, and the camera then shows the
  basket bare and a brown patch where they fell. The real trial checks the same thing with a photo of the basket in the
  base before the top goes on.
- Order: water poured while the basket is in the base or just over its mouth (within 3.5 cm of its axis and 4 cm above
  the rim) lands on the grounds and runs off over the rim, so it counts as spilled. The upper chamber's screwing counts
  only if the basket was already in when it went on, and only while it stays in.
- On the base: the upper chamber's bottom face within 4 mm of the rim height, its axis within 4 mm of the base's (the
  skirt's 1.5 mm clearance plus the thread's play) and within 4 degrees of it (more is cross-threaded).
- Screwed: once the upper chamber has sat on the base (on the basket's flange, 1.5 mm over the rim), and while its skirt stays round the collar (bottom face under
  1.2 cm above the rim, axis within 6 mm and 8 degrees), its turning relative to the base counts. Clockwise turning
  (seen from above) is credited only while a gripper's fingers are closed on it (touching it, judged with the
  gripper's own shape at its present opening within 2 mm, with an opening between 0.45 and its 7 cm width + 3 %, i.e.
  stopped by its body: an empty closed hand reads 0, an open one 1) and only up to 1.25
  times that gripper's own wrist roll in the same direction (twistkit's signed budget; up to 15 degrees of unused
  clockwise roll carries over, counterclockwise roll is kept as debt). Another arm swiping the top round, a hand
  resting on it, or a closed hand wiggling back and forth earns nothing. Counterclockwise turning counts against it
  whoever does it, and lifting it off the collar restarts the count.
- Seat, 450 degrees: a 3-cup pot's thread catches and runs about 1.25 turns before the gasket meets the filter
  plate; snug is reaching it by a gripper's own turning and staying there: turned back off the seat (by anything) it
  is loose again, while a few degrees of give at the stop that the next stroke turns back do not count against it.
  The thread has a hard stop a quarter turn (90 degrees) past the seat, where a firm hand has squeezed the gasket:
  the upper chamber turns no further clockwise on the base, so it cannot be over-turned, and a robot that keeps
  turning just stalls there. With no modeled thread (the top is a free body), the sim applies the
  thread itself from the seat on: before every physics step it holds the top where it came down on the seat, free
  only to turn about the base's axis (a seated thread lets it turn, not lift, tip or slide off the collar), turns it
  back to the stop if it went past and removes its clockwise spin there. Turned back counterclockwise off the seat,
  it is free again.
- Spill: 5 mL, a teaspoon on the hob (a stream falling more than 10 cm splashes a quarter of itself out).
- Upright: the carafe within 10 degrees of upright (a glass standing on its own on a flat counter is within a degree
  or two, so 10 means tipped or leaning on something). The base is in a fixed holder, so it has no such rule.

In a real trial: weigh the water in the base before the basket goes in (or read the level against the valve bore with
a torch from above, as the camera does), photograph the basket in the base from above before the top goes on (grounds
still level in it, flange flat on the rim), judge snug by the seat, not by counting turns: no light through the seam
between the upper chamber and the base with a torch behind it (the gasket pressed on the filter plate all round), and
the top will not turn further clockwise with a light twist and does not lift, and weigh a towel under the hob for
spills.

## Run a trial

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/moka-pot-filled-below-valve/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Get the moka pot ready to brew." --scorer task_score --max-steps 4000

Model run (Dream-1): the same command with `--policy dream --max-steps 4000`. Offline scorer checks:
`python test_task.py` (prints "all pass").

## The demo

The right arm grips the carafe from the side, brings it over the base already tipped 49 degrees, tips it about its lip 5
cm over the rim to 80 degrees (1.40 rad; tipped that far the carafe keeps 41.7 mL), holds, tips it back and sets it down
where it stood: the base ends with 132.5 mL (8 mm under the valve bore's bottom edge), between the lines, with 0.5 mL
spilled. The same hand then grips the funnel basket straight down across its cup, lifts it out of the espresso cup,
carries it level over the base and lowers it until its stem is inside and the cup's floor is 1.2 cm over the rim. The
pads hold the cup under the flange, so the hand cannot follow it into the 6.2 cm mouth: it lets the basket drop the last
3.2 cm, as a person lets it fall in from their fingertips, and the tapered cup centers itself and the flange settles
flat on the rim. In the grip the basket sits 1.5 cm off the tool point (the first finger to touch it pushes it along
until its stem meets the espresso cup's wall, and the two pads then hold it wherever it is), and opened all at once the
fingers flicked it back toward the tool point so that it landed crooked on the rim. So the hand is carried with its
tool point, not the basket, over the base's axis, and eases its grip in two steps over 2 seconds: the basket slides
back between the pads onto the axis, still held, then drops in. Meanwhile the left arm picks the upper chamber up by
its body with a straight-down grip and holds it aside, 2 cm further back and to the left than where it stood (nearer,
the right arm reaching over the base with the basket brushed it). Once the basket is in it lowers the chamber over the
base's collar onto the basket's flange, lets go, and rolls its wrist clockwise in 90-degree regrasp
strokes; the pads slip a little, so after six strokes (540 degrees of roll) the gripper's roll has screwed the top
about 514 degrees (1.43 turns), past the 450-degree seat, and once the hand lets go the run ends in success (a seventh stroke is planned as margin). Between strokes the open
hand centers itself on the top, rises 8.5 cm until its pad tips clear the handle, spout and lid, rolls back up there
and comes down round the body again (after the last stroke it rises the same way before going home), as a person lifts
their hand off to regrip: a top turning more than 45 degrees per stroke always brings its handle under the fingers' way
back. Its grip commands are offset so the wrist turns about the top's axis, not the tool point (the held top sits about
1 cm off the tool point in the sim). The burner stands at (27, 5), toward the left arm, because only there can that arm
lift its hand that high over the pot. The right arm goes home after the basket; the run ends with the left hand just off the top.

## Runtime and seeds

- Demo: 1,969 control steps, ending in success (score 1.0). Dream-1: not yet run on this version (it scored 0 at
  the old 2,400-step limit before the basket was added).
- Model run limit: 4,000 steps (3 minutes 20 seconds at 20 Hz, about twice the demo). One fixed layout, `object_jitter=0`.

## Fidelity

Real size. No fluid simulation: the water is pourkit's deterministic flow model, and its level is drawn as a blue
column inside the base. There is no physical thread: the upper chamber is a free body that drops over the base's
collar (ribs in its skirt keep it centered), and screwing is scored from its turning there (see Scoring), so the sim
gives no torque rise at snug. The ground coffee is not particles: it stays in the basket unless the basket tips past
45 degrees. No heating or steam. The base sits in a fixed holder on the grate, in the sim and in the real build (in a
kitchen the cook's other hand steadies it; this task does not ask for that). What the thread does from the seat on is
applied by a physics-step hook in task.py, since the top is a free body: it holds the top on the seat, free only to
turn, up to the hard stop 90 degrees past it. The funnel basket and the base
are given bare-metal friction, 0.6 static and 0.47 sliding (a handbook dry value for steel on aluminum: the stainless
basket on the aluminum base), instead of yam_arena's rubber-like 1.2 and 1.0 for parts a gripper picks up (a
`friction` option added to this task's copy of yamkit's `Prop`): with the rubber value a basket dropped 14 degrees
crooked wedged 7 mm up on the square-edged posts that model the mouth, where a real basket's rounded rim slides down.

## Real-world build

A 3-cup aluminum moka pot (Bialetti Moka Express 3-cup or similar, 15-16 cm tall) with a stainless-steel replacement
funnel basket of its size, filled level with 15 g of ground coffee and stood in a ceramic espresso cup whose mouth is
about 4.2 cm across inside, taped to the counter at (36, -7); a gas hob or a single-burner camping stove with a
cast-iron grate, unlit and cold, with the base held in a fixed holder on it (a hose clamp round its foot screwed to a
plate bolted to the grate, or a printed collar) so it cannot turn, tip or move; a 250 mL glass carafe with 175 mL of
water. Reset: empty and dry the base, unscrew the top and stand it at (24, 19), tip out and refill the basket and
stand it in the cup, refill the carafe to 175 mL.

## Licenses

All geometry is generated in task.py; no third-party assets.
