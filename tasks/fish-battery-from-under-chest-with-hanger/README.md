# Fish an AA battery out from under a chest of drawers with a coat hanger

**Instruction:** "A battery rolled 11 cm under the chest, left of centre. Using the hanger, get it out, then let go. Leave the chest in place."

A spare AA battery has rolled under a two-drawer pine chest of drawers. The chest's bottom rails stand 27 mm off
the floor, so a hand cannot follow it (an adult hand is about 28 mm thick across the knuckles and the fingers alone
reach about 8 cm), and the chest is full of clothes and stands against the wall. What nearly everyone does is take a
coat hanger, slide it in flat and draw the battery out. The hard part is physical: the hanger has to be slid along
its own bar past the battery's end without touching it (a battery nudged on its near side rolls away, deeper), and
then swung round so the bar rolls the battery out ahead of it, slowly enough that it is not flicked away and with
the bar behind the battery's whole length so it does not slip off the bar's end.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=fish_battery_from_under_chest_with_hanger.py \
    --instruction "A battery rolled 11 cm under the chest, left of centre. Using the hanger, get it out, then let go. Leave the chest in place." \
    --policy yam_scripted --scorer task_score --max-steps 1800
```

- Scene: `station`. Control at 20 Hz. Run limit: 1,800 steps (90 s). The scripted demo plan is 27.4 s (548 steps)
  and reaches success at step 463; a trial takes about 2 minutes on the GPU.
- No object jitter: one fixed layout; the seed changes nothing.
- `--scorer task_score` gives 1.0 on success and otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_fish_battery_from_under_chest_with_hanger.py` (scorer tests on the start, goal,
  partial and cheat layouts, and the demo's reach).

## Objects (real products, real size)

| Object | Real product | Figures used | Source |
|---|---|---|---|
| Chest of drawers | IKEA KRONÖREN chest of 2 drawers, untreated solid pine (906.107.99), about $100 (price an estimate) | 76 x 48 x 55 cm; free height under the furniture 2.7 cm; leg span 64 x 39 cm; drawers 60 x 38 cm inside, 19 cm deep; package 14.78 kg | ikea.com product page "Weight & measurements" and its measurement drawing; assembly instructions AA-2590395-5 |
| | | legs 28 x 24 mm; bottom rails 22 mm tall, 18 mm thick, on all four sides; side rails at 0.27 and 0.48 m; top 18 mm; lower drawer underside 63 mm above the floor | read off the maker's measurement drawing (estimates, about 2 mm) |
| | | 23 kg loaded: 13.5 kg chest + about 9.5 kg of folded clothes (the maker allows 7 kg per drawer) | estimate |
| AA battery | Duracell alkaline AA, MN1500 / LR6, about $1 | 14.5 mm x 50.5 mm, 23.6 g; black jacket, copper band at the positive end | Duracell MN1500 datasheet |
| Coat hanger | Uline S-22340 heavy-duty tubular plastic hanger, white polypropylene, about $0.75 each | 16 5/8 in (422 mm) wide outside, 3/8 in (9.5 mm) tube; 18 lb per carton of 120 (68 g each, carton included) | uline.com S-22340 |
| | | bar 408 mm between corner centres, apex 124 mm above the bar, hook reaching 219 mm, centre loop 22 mm radius; 65 g | measured off the maker's photo scaled to the 422 mm width (estimates) |
| Wall | painted wall with a 10 x 70 mm skirting board | | |

Total real cost of the props: about $102 (chest about $100, battery about $1, hanger $0.75).

The chest is modelled as the real frame, not a box: four legs, bottom rails on all four sides, side-frame rails,
two shut drawers and the overhanging top. Under the lower drawer the floor is open from rail to rail, 63 mm high,
as on the real chest. The battery is three coaxial cylinders (black can, copper band, positive nub) plus the steel
negative end. The hanger is its real outline: a round bar (three coaxial cylinders, the middle one 20 mm long where
the gripper pinches), shoulder tubes, hook and centre loop as convex octagonal tubes 9.5 mm across the flats.

## Layout (metres; x forward, y left, z up from the table top, which stands for the floor)

| Item | Pose |
|---|---|
| Chest | front faces of the front legs and front rail at x 0.355; centred on y 0; back legs to x 0.745; top from x 0.310 to 0.790 |
| Wall | front face at x 0.800 |
| Battery | spawns at (0.485, 0.090), axis 38 deg left of straight ahead; in the simulator it settles in the first 3 s to (0.495, 0.077), its nearest point 11.6 cm behind the rail face |
| Hanger | flat, bar across the chest's front at x 0.182 from y -0.263 to +0.145, hook toward the chest, hook tip 5 cm under the rail |

The chest stands at x 0.355 because at x 0.30 its front would cut into the arms' ready pose (the ready pose's
fingertips reach x 0.32). Nothing is shrunk or lowered.

## Rolling resistance

The simulator has no rolling resistance, so a touched cylinder rolls on indefinitely. The battery gets angular
damping 4 /s, authored at spawn by the task module (the kit has no option for it; the task module wraps the kit's
spawner, the simulator is unchanged). For a rolling solid cylinder this slows it at (4/3) v per second, the same as
a rolling-resistance coefficient of 0.01 at 5 cm/s (estimate: published rolling-resistance figures for hard rollers
on hard floors span 0.001 to 0.01 and are noted as inconsistent between sources). On the GPU a battery left rolling
at 3.9 cm/s by the hanger rolled on 3.1 cm and stopped (run fb-p2).

The task module also gives the hanger's tubes a matte (non-metallic) look; the kit's mesh look is metallic.

## Scoring

The scorer reads the battery, hanger and chest poses, and the robot's joint state for the gripper centres and
openings.

| Rule | Value |
|---|---|
| OUT | every part of the battery in front of the front rail's face (x < 0.355; the can's extent along x from its pose), and the battery lying on the floor (centre within 3 mm of its resting height) |
| STILL (needed for any credit) | over the last 1 s, the battery's centre and 26 points along the hanger's tubes each move less than 2 mm |
| HANDS CLEAR | both grippers at least a quarter open and both gripper centres at least 30 mm from the battery's surface and from every hanger tube's surface |
| 1.0 | OUT, hanger lying flat on the floor (tilt at most 10 deg), HANDS CLEAR |
| 0.7 | OUT, but the hanger held, lifted or a gripper within 30 mm |
| 0.4 | battery on the floor, centre x < 0.395 (10 cm nearer than where it lay), not OUT |
| 0.2 | battery on the floor, centre x < 0.455 (4 cm nearer) |
| 0 | otherwise, including a battery held or resting off the floor |
| Sticky zeros (every step) | battery centre ever behind x 0.615 (12 cm deeper); battery ever beyond y -0.32 or +0.32 while behind the rail face (out under a side rail); any chest foot ever more than 10 mm from its start or lifted more than 3 mm; battery, hanger or chest ever 20 mm below the floor |

Success (`success()`) is a score of 1.0 held for 1 s (`SUCCESS_HOLD_S`).

**Real trial.** Tape L-corners round the four feet; stick a paper tape ruler to the floor along the battery's line,
from 10 cm in front of the chest to the back rail; tape lines along the rail face (x 0.355) and at y -0.32 and
+0.32. A phone lying on the floor at the right front leg, looking along the gap, records the battery against the
ruler (read to about 5 mm). A top video shows release and stillness. Reset: lift the chest's front by its top edge
about 3 cm, lay the battery on its taped outline, lower the front feet back into their corners (the rails clear the
battery by 12 mm); lay the hanger on its taped outline.

## Scripted demo (one arm; the left arm has no job and parks out of the way)

Open loop from the start layout and time:

1. The right gripper opens to 25 mm and pinches the bar 60 mm from its right corner (the pads clear the shoulder
   tube, 36 mm off the bar there).
2. It swings the hanger flat on the floor about the pinch, 52 deg clockwise, so the bar lines up with the battery
   (commanded 2 deg further: the hanger drags on the floor and lags the wrist by about 2 deg).
3. It slides the hanger 12 cm along its own bar, under the rail and past the battery's right end, so the bar lies
   parallel to the battery, 16 mm behind it, its far corner 24 mm past the battery's end.
4. Over 7 s it swings the bar 48 deg back toward the chest's front while the pinch moves 25 mm back and 45 mm left
   (so the battery stays on the bar instead of drifting off its far end). The battery rolls out ahead of the bar.
5. It waits 1.5 s, opens, lifts and moves aside.

The plan keeps the pinch within 0.335 m of the arm base and the gripper's palm in front of the drawer fronts.

## GPU runs

| Run | What | Result |
|---|---|---|
| fb-p1 | first build (battery spawned 0.3 mm high, bar 14 mm behind it) | 0.0: the battery settled 15 mm toward the bar, the bar hit it while sliding in and pushed it 7 cm deeper |
| fb-p2 | spawn on the floor, 20 mm clearance | 1.0, success at step 463; battery out to (0.273, 0.271) |
| fb-n1 | demo planned from the settled battery pose | 1.0, success at step 463, but the battery slipped off the bar's far end and stopped only 5 mm in front of the rail face |
| fb-s08 | the same at 0.8x durations | 1.0 at the end (policy stop at step 499), battery 6 mm in front of the rail face |
| fb-n2 | final demo (swing-lag compensation, pinch moves toward the bar's far end in the sweep) | 1.0, success at step 463; battery out to (0.274, 0.301), 5 cm in front of the rail face |
| fb-s08b | final demo at 0.8x durations | 1.0, success at step 374 |
| fb-s12 | final demo at 1.2x durations | 1.0, success at step 551; battery out to (0.280, 0.296) |
| fb-c1 | control: bar slid in 12 mm into the battery's line instead of 16 mm behind it; left arm reaches with closed fingers toward the battery | 1.0, success at step 463: the bar ran into the battery's near end, turned it 22 deg and pushed it 4 cm along its axis, and the sweep still rolled it out to (0.229, 0.309); the left arm's closed fingertips stopped on the floor 5 cm in front of the rail. A slide that only clips the battery's end is forgiven; the 12 cm sticky line is for a battery shoved straight back |
| fb-dream-1 | Dream-1 policy | 0.0, ran to the 1,800-step limit (37 policy requests, no error); the right hand hovered over the hanger and never pinched it |

In every run the chest's feet moved less than 1 mm.

## Why the hanger is needed

The battery's centre is 0.55 m from the left arm base and 0.63 m from the right (reach 0.36 m). With the palm in
front of the drawer fronts no fingertip gets within about 0.19 m of it, and the fingers cannot pass under a 27 mm
rail at all. Only a tool reaches it. In control run fb-c1 (above) the left arm's closed fingertips stopped at the
floor in front of the rail.

## Offline checker notes

The ladder's kinematic replay reports three `[knocks]`, all known replay limits: the sweep's contact with the
battery is the task itself (a carried tool pushing a free part); the slide contact is measured against the battery's
spawn pose, 15 mm from where it settles on the GPU before the hanger arrives; and the fingertip contact at "lift
clear" is the replay's boxy finger model against the hanger's proxy balls 25 mm off the bar (the GPU runs show no
contact). Warnings: the chest's mass (it is never gripped), the hanger's white colour (the real hanger is white),
the partly open pinch approach and the 9.5 mm tube's shallow grip.

## Licences

Only primitive shapes and our own kit modules (`yam_parts.py`, `yam_joints.py`, `yam_mesh.py`, copied unchanged).
No third-party meshes or textures.
