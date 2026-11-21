# Scoop a ball of ice cream onto a hand-held sugar cone without cracking it

A full supermarket tub of hard ice cream, a Zeroll #20 scoop on a plate and a JOY sugar cone standing tip down in a wire cone stand. The left hand takes the cone out of its stand and holds it upright, gently, because the shell is brittle. The right hand takes the scoop by its handle, sets the bowl's rim about 1 cm into the ice cream and draws it 10 cm along the tub against a drag of a few newtons; a ball curls up in the bowl after 8 cm. It lifts the ball clear and turns the bowl over the cone's mouth until the ball rolls out onto the cone. The left hand then holds the cone upright with the ball on it.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-cameras --mode none,one_hand,hover,shallow,short,slow_draw,back_draw,high_drop,squeeze,bare_left,stand_then_lift,same_hand
```

The harness's instruction (Dream-1 run): `--instruction "Scoop a ball of ice cream from the tub with the scoop and put it on the sugar cone. Hold the cone upright in your other hand, and don't crack it."`

**Result** (scripted, GPU physics, 20 Hz): `full` scores 1.0 (success at step 813, 40.7 s; RTX 3090 Ti). The ball forms once the bowl is 8.0 cm from where it went in; it reaches the cone's mouth, the cone then in the left hand out of its stand; the left hand holds the cone at about 9 N a finger (10 N commanded); the ball comes to rest in the cone's mouth, off the scoop, and stays there, the cone upright in the hand, for 3 s.

**Dream-1 run** (the harness's own policy, 1,200 steps, one trial): 0.0. It reached no tier before the step limit.

## Scoring, and the routes that score less

The score is read each step from the state then; only the crack and an unheld serve latch. 1.0 needs the 1.0 state for 3 s in a row.

| Tier | What the code reads |
|---|---|
| 0.3 | a ball has formed (the bowl drawn 8 cm, its centre read in a straight line from where it went in, while its lowest point stays 6 to 15 mm under the surface; drawing it to and fro adds nothing, and nor does a step at the end of a half second in which the bowl moved under 5 mm, so a creep under 1 cm/s counts nothing; against a drag of 0.3 N per mm of depth) and sits in the bowl (centre within 12 mm of the bowl's), the bowl's lowest point 1 cm over the tub's rim (the bowl lifted clear of the tub); the draw counts only while the bowl's opening leads its motion (drawn backwards, it counts nothing), and a ball that moves faster than 1.2 m/s between leaving the bowl and reaching the cone (a flick or a drop from height) caps the score at 0.3 |
| 0.7 | the ball rests in the cone's mouth (centre within 12 mm of the cone's axis, within 20 mm of the rim's height), 2 mm or more from the scoop's inner and outer surfaces, while the left hand holds the cone: both of its fingers within 8 mm of the cone's outer surface, the cone lifted 6 cm or carried clear of its stand, its tip 1 cm or more over the bench |
| 1.0 | as 0.7, and the cone's axis within 20 degrees of vertical, for 3 s in a row |
| void (0) | on the first step the formed ball is in the cone's mouth, the left hand is not holding the cone out of its stand (the ball set on a cone in its stand, or on one in the scoop's hand, and the cone lifted after); or a hand with both fingers on the cone pressing over 20 N a finger for two steps: the cone cracked |

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | left takes the cone from the left side, tilted 60 degrees, and lifts it 8 cm; right takes the scoop by its handle, tilts the bowl 35 degrees, sets its lowest point 10 mm in and draws it 10 cm (the ball forms at 8 cm), lifts it out, levels it and holds it up between the hands, turns the bowl 55 degrees over about its handle; left brings the cone's mouth under the bowl's lip, 35 mm under its centre; right turns on to 110 degrees and the ball rolls out onto the mouth (about 2.5 cm of fall); right draws the scoop away; left lifts the cone 5 cm and holds it | 1.0 |
| one_hand | the left hand does nothing; the right scoops and turns the ball out over the cone left in its stand | 0.0 |
| hover | the scoop held 4 cm over the tub, not drawn | 0.0 |
| shallow | the scoop drawn 10 cm at 3 mm deep | 0.0 |
| short | the scoop drawn 4 cm at 10 mm deep | 0.0 (3.5 cm counted) |
| back_draw | the scoop drawn 10 cm with the bowl's back leading, then the full hand-over | 0.0 (the draw counts nothing) |
| high_drop | as full, but the bowl lifted 12 cm before the turn, so the ball is dropped from height | 0.0 (the ball bounces off the cone and ends 3.6 m away; a ball that did land after moving faster than 1.2 m/s in flight would score at most 0.3) |
| slow_draw | the scoop drawn 9 cm at 10 mm deep, at 2.9 mm/s | 0.0 (0.3 cm counted) |
| squeeze | as full, the cone held at 40 N a finger | 0.0 (cracked at step 113, 33 N) |
| bare_left | the left lifts the cone and holds it; no scoop | 0.0 |
| stand_then_lift | as one_hand, then the left hand takes the cone out of its stand and holds it up | 0.0 (the right arm cannot turn the ball out over the cone in its stand: it falls 58 cm away; had it landed, the unheld serve voids the run, as `check_score.py` shows) |
| same_hand | as one_hand, then the right hand lets the scoop go, takes the cone out of its stand and holds it up | 0.0 (as stand_then_lift) |
| none | nothing | 0.0 |

`check_score.py` runs the tier table on hand-built moments with no simulator.

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Scoop | 184 mm long; bowl 51 mm across, 1.5 mm wall; handle 25 mm across; 150 g | 7.25 in, 2 in bowl | Zeroll 1020 Original #20 listings (chefsupplies.ca, restaurantsupply.com); handle and mass estimates |
| Ball | 44 mm across, 40 g | a #20 scoop is 1/20 quart, 47 ml | sphere of 47 ml; mass estimate |
| Cone | 117 mm tall, 48 mm across the top, 43 mm inside, 12 g | 4 5/8 in, 1 7/8 in | JOY #310 sugar cone (webstaurantstore.com); mass estimate |
| Tub | 180 x 130 x 90 mm, full, surface 5 mm under the rim | 1.42 L (48 fl oz) rectangular tub | estimate |
| Cone stand | RAK Porcelain PIRC08 stainless rod stand: 79 mm (3 1/8 in) base ring and one slanted rod of 4 mm rod, a 6 mm top ring 22 mm across inside, its top 60 mm up | | base ring from the listing; top ring and height estimated from its photo |
| Drag | 0.3 N per mm of depth: about 3 N at 1 cm | 2.5 to 8 kPa yield stress over about 5 cm2 of rim gives 1.3 to 4 N | vane-method rheology of ice cream; estimate |
| Crack | 20 N a finger | | estimate, to measure |

## Colliders

| Body | Colliders | Free or fixed |
|---|---|---|
| Tub | five boxes (walls and floor); the ice cream itself has no collider (see below) | fixed |
| Plate, cone stand | a cylinder; the stand's top ring as 16 thin boxes (the cone sits on its inner top edge), its rod and base ring as short hulls | fixed |
| Scoop | the bowl as 64 convex patches whose inner faces lie on or outside the bowl's inner surface; the handle as one hull starting at the bowl's rim | free, 150 g |
| Ball | a sphere | free, 40 g; parked under the ice cream's surface until it forms |
| Cone | 36 convex pieces of the hollow shell | free, 12 g |
| Fingers | the harness's YAM finger colliders | the robot |

## Robot figures the demo keeps within

The YAM's jaws open to 94 mm and grip about 40 N a finger; each arm pushes about 20 N and reaches about 60 cm. The left holds the cone at about 10 N a finger (half the crack figure). The right pinches the 25 mm handle at 40 N and draws the scoop against about 3 N.

Why it matters: U.S. residents ate 12.1 lb of regular ice cream each in 2019 and 21 lb of frozen dairy products in all ([USDA ERS, "U.S. residents scooped more ice cream in 2000 than in 2019"](https://www.ers.usda.gov/data-products/charts-of-note/101589)), about eight 1.42 L tubs a person a year (estimate, at roughly 1.5 lb a tub), most of it scooped by hand at home or in a shop. In [IDFA's 2022 survey of 1,002 US consumers](https://www.idfa.org/wordpress/wp-content/uploads/2022/06/IDFA-Ice-Cream-Frozen-Novelty-Trends-2022-Report_6-21.2022-for-Press.pdf) (p.23, p.31), 84% buy ice cream at a grocery store and eat it at home, and almost half prefer half-gallon tubs; only one in ten eat it straight from the carton ([IDFA 2024](https://www.idfa.org/wordpress/wp-content/uploads/2024/05/2404080_IDFA_Ice_Cream_Analysis_Final3.pdf), p.12).

## Real trial

Use one fresh 1.42 L (48 fl oz) tub of a widely sold vanilla ice cream, for example Breyers, per trial, lid off. Keep it at -18 C (0 F, a standard freezer setting) overnight, then temper it on the bench until a probe thermometer 2 cm below the surface reads -14 to -12 C (6 to 10 F; the target and the 10 to 15 minute estimate are procedure choices to confirm by the probe, not sourced figures). The 0 F freezer setting is in the range a commercial freezer manual gives: "Freezers are set to cycle between a minimum temperature of -15°F and a maximum temperature of -8°F" ([Avantco CP series manual](https://www.webstaurantstore.com/documents/pdf/avantco_cp_series_updated_manual.pdf)). Level the surface with a spatula so it sits about 85 mm up, and start every trial from an untouched surface with the bowl entering at the centre, 2 cm from the near wall. Use a fresh cone each trial. Dip the scoop in warm water and dry it. Stand the cone tip down in its stand and lay the scoop on its plate, handle toward the robot. Melting ice cream may drip on the cone hand: fit a nitrile finger cover.

What the video and the top photo see, by the scorer's terms:
- SCOOPED: a round ball in the bowl, lifted clear of the tub.
- ON THE CONE: the ball lands in the cone's mouth while the left gripper holds the cone out of its stand, and rests there, the scoop away from it.
- SERVED: the cone upright with the ball on it for 3 s.
- Judge with a side-on phone camera on a tripod for the 1 cm rim clearance and the 20 degree upright; inspect the cone by hand after each trial for a crack.
- Void: the ball put on the cone before the left gripper has it out of its stand; a visible crack or a broken piece of cone.

| Real-trial kit | Use | Price (about) |
|---|---|---|
| Zeroll 1020 #20 ice cream scoop | the scoop | $25 |
| JOY sugar cones (12) | cones | $4 |
| 1.42 L tub of ice cream | the tub | $5 |
| RAK Porcelain PIRC08 stainless cone stand (webstaurantstore.com 772PIRC08, sold 12 to a case, about $34 each), or any single-cone wire stand with a ring about 22 mm across, or a 4 mm steel rod bent to the sizes above | the stand | $10 to $34 |
| Plate, phone and tripod | scene, top photo and video | $0 to $15 |
| Total | | under $85 |

## Safety for a real trial

Food only; the ice cream is not eaten after a trial. Wipe the grippers between trials.

## What is modelled, and what is not

- The ice cream is not a body. While the bowl's lowest point is under the surface, a drag of 0.3 N per mm of depth (an estimate) acts against the bowl's motion at the hand's grip on the handle (scaled down below 1 cm/s). After the bowl has been drawn 8 cm (its centre's straight-line distance from where it went in, counting only a draw of 1 cm/s or more over half a second) with its lowest point 6 to 15 mm deep, a rigid 44 mm ball is put in the bowl. A real ball is formed by the scoop's curl, softens and sticks a little; the sim's ball rolls out of the bowl once its opening faces down past level, so it falls about 2.5 cm onto the mouth where a server would touch the ball to the cone first.
- The cone is a rigid hollow shell. Its crack is read from the finger force, not from a fracture model.
