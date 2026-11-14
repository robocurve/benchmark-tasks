# Mount a full malt cup on a spindle drink mixer

Mix the milkshake: put the filled steel cup on the drink mixer with the spindle inside it, its base on the upper step
of the cup rest and its rim in the cup guide, so the mixer runs. Do not spill the mix.

Ice cream shops, diners and burger counters blend milkshakes on single-spindle drink mixers (O*NET 35-3023.00: "Prepare
and serve cold drinks, frozen milk drinks, or desserts, using drink-dispensing, milkshake, or frozen custard
machines."). The mixer has no button: the cup is tipped so its mouth takes in the agitator hanging from the head,
lifted around the spindle with its back rim going up behind the cup guide, and set down on the cup rest. On the rest's
upper (run) step the rim holds the guide's switch pin up and the motor runs; on the lower (standby) step the cup hangs
in the guide with the motor off. The cup weighs 0.58 kg full, its mouth is 96 mm across around a 27 mm agitator, the
rim has a 10 mm slot between the guide plate and the column, and the mix spills past about 69 degrees of tip.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mount_malt_cup_on_spindle_mixer.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Mix the milkshake: put the filled steel cup on the drink mixer with the spindle inside it, its base on the upper step of the cup rest and its rim in the cup guide, so the mixer runs. Do not spill the mix." \
  --scorer task_score --max-steps 3600 --epochs 2 --store-frames --log-dir ./demo-run
```

For a model trial, replace `--policy yam_scripted` with `--policy dream`. The output of `--scorer task_score` is the
score, taken at the end of the run. Each trial allows 3,600 control steps (180 seconds at 20 Hz). There are no seeds:
the layout is fixed and nothing is jittered. The scripted demo lasts 35.1 s. `success()` requires a score of 1.0.

## Scoring

Read from the cup's pose and velocity, the switch pin's lift and the grippers, at 20 Hz; the score is the value at
the end. In the mixer's frame (origin on the counter under the spindle, +x toward the column):

- **Seated on the run step:** the cup's bottom centre 2 mm below to 3 mm above the run step's top (160 mm), its
  bottom's back edge at least 2 mm over the step's front edge (x 26 mm), its axis within 5 degrees of vertical, its
  bottom centre within 10 mm of the spindle's line across the rest (|y|), the rim's inner top edge on the column side
  at least 3 mm above the guide plate's lower edge, no more than 1.5 mm in front of the plate's back face and within
  18 mm of y = 0, and the switch pin lifted at least 2 mm (the switch closes).
- **Seated on the standby step:** the same on the standby step (152 mm, front edge x 12 mm) with the pin under 2 mm.
- **Spindle in:** the agitator's centre inside the cup at least 40 mm below the lip and above the mix layer, with 2 mm
  between it and the wall, and the cup's bottom at least 30 mm above the counter.
- **Released:** both gripper TCPs at least 30 mm from the cup's bounding box. **Still:** under 10 mm/s and 0.5 rad/s.
- **Zero for the rest of the run:** the cup tips more than 65 degrees from upright (the mix spills), or its bottom
  centre leaves the counter (more than 20 mm below its top or outside its edges).

Score: 1.0 seated on the run step, released and still for 1 s; 0.5 seated on the run step otherwise (held or
moving), or seated on the standby step released and still for 1 s; 0.25 seated on the standby step otherwise, or
spindle in; 0 otherwise. In a real trial the motor running and a side photo against a scale give the same readings.

`pytest` in this folder runs the generated offline checks (no GPU): the scorer on the start, the goal and the named
layouts, and the demo's reach and length.

## The demo is open loop

The scripted demo uses only the known starting layout, time and a model of the arm. It never reads the cup, the
mixer or the scorer. The right arm does the work; the left arm parks. Times are seconds from the start.

| Time | Right arm |
| --- | --- |
| 0.00-3.70 | Park, then hold the open hand 70 mm behind and above the grasp, along the line from the arm's base to the cup |
| 3.70-7.60 | Bring the pads down around the cup's lower body and close across it (pads 52-60 mm up, 12 mm short of its axis, gripper pitched 20 degrees below horizontal), settle |
| 7.60-9.80 | Take the cup's weight, lift it 40 mm |
| 9.80-14.20 | Carry it in front of the mixer tipped 30 degrees toward the column, slide its mouth under the agitator |
| 14.20-17.40 | Raise it around the spindle while it straightens to 15 degrees, its back wall passing over the cup rest's steps |
| 17.40-20.00 | Bring the rim under the guide plate (10 degrees), then straight up behind it |
| 20.00-23.00 | Lift the switch pin with the rim while straightening; upright, the bottom 2.5 mm over the run step |
| 23.00-25.60 | Lower the bottom onto the run step, hand its weight to the step |
| 25.60-29.10 | Open over 1.9 s, back the open hand out along its approach |
| 29.10-35.10 | Park and hold still |

The mount follows keyframes of a path planned offline through the clear space between the mixer and the cup's walls,
lip, bottom and the gripper: 4 mm to spare before the guide, 2.5 mm behind it. While the pin is lifted the rim's inner
edge bears on it at least 3 mm in front of its back face. Released on the run step, the cup's centre of mass is in
front of the step's edge, so it tips forward about 1.5 degrees until its inner wall rests on the guide plate; the
whole width of the rim is then under the pin, which it holds up about 4.6 mm.

The parallel gripper's two fingers both close at their effort limit, so the cup does not stay centred between them: it
ends against the pad of the finger that stays at its open stop, 8.2 mm off the hand's centre line, and the grip holds
it there without slipping (under 0.4 degrees of turn) until the release. The demo's carried poses allow for this
measured offset, so the cup itself keeps to the planned path.

The arm is a stiff spring, not a rigid positioner: each joint is driven by PD (80 N m/rad for joints 1-3, 10 for
4-6), and gravity compensation covers the arm's own links but not a held object. The full cup bends the arm about
50 mm and 10 degrees at the mixer, and the switch pin's push on the rim adds to it. While the arm carries the cup the
demo commands each pose that far past the wanted one, from the joint stiffnesses, the cup's mass and centre of mass
and the pin's spring (q_cmd = q - J^T F / K); the moves in the mixer are cut into steps of at most 3 mm and 1 degree,
each with its own correction. These are fixed numbers computed from the arm's model, not feedback. All demo targets
are within 0.352 m (planar) of the right arm's base.

## Rebuild the setup

Station tabletop (the counter, stainless steel) z = 0.7475 m, spanning x -0.19 to 0.91 m, y -0.65 to 0.65 m; x
forward, y left; arm bases (0, +/-0.305, 0.75) m. Millimetres in the mixer's frame unless stated.

- **Drink mixer:** single-spindle commercial drink mixer (Hamilton Beach HMD200 class: 20.5 by 6.5 by 6.75 in,
  6.4 kg), die-cast housing, fixed to the counter by its suction feet. Origin (0.277, -0.145, 0.7475) m on the
  counter under the spindle, turned 30 degrees about z so that its +x points straight away from the right arm's base
  (0.32 m from it, 30 degrees to its left); the column is at the back.
  - Base: 25 tall, 165 wide (y), x -60 to 111 with a rounded front (radius 82.5).
  - Column: x 60 to 111 (front face 96 wide with 12 mm rounded edges, 120 wide behind x 72), from the base to the head.
  - Head: z 375 to 521, 165 wide, x -75 to 111 with a rounded front.
  - Spindle, stainless steel, free to spin about its axis (x = y = 0): an 8 mm shaft from the head down to a solid
    27 mm agitator disc 6 mm thick, 208 to 214 above the counter; a 20 mm chuck from 340 to 375.
  - Cup rest, chrome-plated steel J bracket on the column face, 18 wide (y): back plate x 56 to 60, z 130 to 178; run
    step x 26 to 56, top at 160; standby step x 12 to 26, top at 152; both steps from z 130.
  - Cup guide, chrome-plated steel plate 3 thick hanging from the head, z 326 to 375, inside the seated cup's mouth,
    stepped to follow the cup's curve: its back face at x 45.9 for |y| up to 6, 44.7 for 6 to 12, 42.7 for 12 to 18.
  - Switch pin, white plastic, 9 (x) by 8 (y) by 20 tall at x 47 to 56, its lower end 336.6 above the counter at rest
    (4 mm below the rim of a cup on the run step, 4 mm above one on the standby step). It slides up 0 to 12 mm on a
    spring of 120 N/m preloaded 1.2 N (1.7 N when a seated cup holds it 4 mm up), damping 2 N s/m, 4 g; the switch
    closes at 2 mm.
- **Malt cup:** 32 oz stainless steel drink-mixer cup (Hamilton Beach 110E size): 180.6 tall, 96.5 across the top
  under a rolled lip 102.4 across and 3 tall, 70 across the base. Modelled as 32 flat wall panels 1 mm thick (the
  outside surface true) and 32 lip panels, a 1 mm bottom and the mix as a 7.4 mm layer on the bottom. 0.58 kg in all
  (about 0.28 kg cup and 0.30 kg of mix: the maker's recipe of 3 No. 24 scoops of ice cream, 4 fl oz milk and
  1.5 fl oz syrup, 281 ml); centre of mass 64 mm above the bottom. It stands upright on the counter, bottom centre at
  (0.301, -0.479, 0.7475) m (0.347 m from the right arm's base, 30 degrees to its right), turned -30 degrees about z
  so one panel faces that base.
- **Contact:** friction static/dynamic 0.30/0.25 for the cup and the mixer (estimates for stainless and chrome, not
  measured); contact offsets 1 mm, rest offsets zero.

### Sources and estimates

From the maker's documents: the mixer's size and mass, the cup's height and diameters, the recipe, the two-position
cup rest on the column face below the agitator, and the guide pin that the cup's rim lifts against the actuator
switch in the top position ("Position the container into the container guide and move the cup upward to
automatically turn the motor on. Set the container down onto the cup rest."). Scaled from the maker's photographs:
the agitator, the cup rest's steps and the guide's position. Estimated and to be measured on a real unit before real
trials: the cup's base diameter, wall thickness and mass; the cup rest's step heights, depths and width; the
agitator's height and diameter; the guide plate's shape, lower edge and gap to the cup; the pin's position, travel,
spring and switch point; the column face's position; friction.

## Fidelity and simplifications

Rigid bodies. The mix is a rigid layer on the cup's bottom that gives the full cup its mass and centre of mass; it
does not slosh, and the agitator passes above it. Spilling is scored from the cup's tilt: treated as a liquid, the
recipe's surface reaches the lip at about 69 degrees of tip, and the scorer stops at 65 for slosh. The spindle spins
freely but the motor is not simulated: it runs when the pin is lifted 2 mm. The cup's wall is modelled 1 mm thick
(about 0.6 mm real), with the outside surface true.

## Reset

Lift the cup off the rest and out of the guide, tip it clear of the spindle and stand it back on its mark, turned the
same way. Nothing is consumed in simulation. Real trials: refill to the recipe (or a 0.3 kg thick stand-in) and wipe
the spindle.
