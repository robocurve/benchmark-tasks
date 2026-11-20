# Repot a small cactus one size up

A golden barrel cactus has outgrown its 9 cm nursery pot. The robot moves it into a 4 in. pot: it frees the bound root
ball by squeezing the thin pot wall with one hand, lifts the cactus by the foam collar round its body with the other,
and sets it upright and centred on the new pot's base layer at the same depth as before, then lets go.

## Why it matters

Helping people repot houseplants that have outgrown their pots is everyday household help. About two-thirds of US
adults keep at least one houseplant, and small cacti are among the commonest sold. A root-bound plant stops thriving,
yet repotting it is fiddly, and a cactus cannot be held by hand, so it is a chore many people put off, and one that
people with weak grip, arthritis or limited mobility often cannot do safely themselves. A robot that helps at home must
do it the way gardeners do: squeeze the flexible nursery pot to free the ball rather than pull the plant (Gardener's
Path), handle a cactus through a band or tongs and never by its body or roots (University of Arizona AZ1376; Chicago
Botanic Garden), and replant at the same depth. Backfilling the sides is the next step and is not part of this task.

## The real build

- Old pot: Teku VCG 9 (Pöppelmann), round black polypropylene, 9.0 cm across the rim, 6.8 cm tall, 0.29 L, thin wall,
  upright at (0.35, -0.06) m (x forward from the line between the arm bases, y to the robot's left), on a grey 3 mm
  rubber mat, 0.30 × 0.56 m, centred at (0.35, 0.03) m.
- Cactus: *Echinocactus grusonii*, bought already rooted in a 3.5 in. pot, 5.5-6.5 cm across the spine tips, 5-7 cm
  tall. Its root ball (the pot's inside shape: 8.0 cm top, 6.8 cm base, 5.5 cm tall, about 0.18 kg) is moved into the
  VCG 9 and pressed home, soil face 1.2 cm below the rim.
- Collar: a 30 × 250 mm strip of 4 mm EVA craft foam, wrapped once round the body and taped, 1.2-4.2 cm above the soil
  (about 6.8 cm across). It is fitted before the trial, as a person fits a band before lifting, and left on.
- New pot: HC Companies 4.00 in. Round Standard Thinwall (STG04000), black PP, 10.2 cm rim, 8.9 cm tall, at
  (0.35, 0.13) m, with dry cactus mix (Espoma Organic Cactus Mix or Black Gold Cactus Mix) firmed level 2.0 cm deep
  with a 7 cm disc.
- Gripper pads wear 1.5 mm neoprene covers against the spines.

## Calibration (the root ball's grip)

With a kitchen scale, once per plant and every 5 trials:
- Push-out: the pot stands on its rim on two wood blocks on the scale; a 12 mm dowel pushed up through a drain hole
  must peak at 1.2-2.0 kg (12-20 N). If not, the ball is pressed in again.
- Flex: the pot lies on its side on the scale; a 4 cm wide flat block pressed across its body 4.5 cm up until the wall
  has moved in 3 mm each side (ruler) reads X, nominally 2.0 kg (20 N). After that squeeze, the push-out must read
  under 0.3 kg.

## A real trial and its score

Instruction: "This cactus has outgrown its pot. Move it into the bigger pot: upright, in the middle, on the mix, with
the top of its root ball 1 to 2 cm below the rim, the same depth as before. Handle the cactus only by the foam collar
round it, and don't pull the root ball against the old pot's grip. Then let go and move clear."

Scored with a ruler across the new pot's rim at four points (depth = their mean, tilt = their largest difference,
off-centre = half the larger difference between opposite wall gaps), the video and the gripper log. "Home" means both
grippers within 3 cm of their start and every fingertip at least 3 cm from every object, for 1 s. "Freed" means the old
pot's wall pressed in 3 mm, 4.5 cm up, on two opposite sides, for 1 s.

| Score | When |
|---|---|
| 1.0 | home; the ball whole on the mix in the new pot; depth 1.0-2.0 cm; off-centre under 0.4 cm; tilt under 0.8 cm |
| 0.75 | home; the ball whole on the mix in the new pot; depth 0.5-2.5 cm; off-centre under 0.7 cm; tilt under 1.5 cm |
| 0.5 | home; the ball whole, out of the old pot, upright within 30° |
| 0.25 | freed; the ball whole and risen at least 1 cm (in the pot or out); not held |
| 0 | a gripper or arm touches the cactus outside the collar; a fingertip touches the soil face, or the ball outside the old pot; the ball rises 1 cm before it is freed; the old pot lifts 2 cm; the cactus falls 3 cm or tilts past 30°; the new pot moves 2 cm or tips past 15°; anything leaves the table; any other end state, including still held |

Reset: lift the cactus by its collar, slide the ball back into the old pot, press the soil face home on the scale to
3-4 kg for 5 s; top up and firm the base layer; check the pad covers. Retire a plant after 20 trials, a broken spine
cluster, or a ball 15 g lighter than at first.

## In simulation

- The cactus (with its collar) is one rigid body, bonded to its rigid root ball at 40 N or 0.8 N·m.
- The old pot's wall is four quarter panels, each hinged at the base on a spring calibrated so 20 N, 4.5 cm up, moves
  it in 3 mm (measured in simulation: 10 N gives 1.5 mm, 20 N gives 3.0 mm, 25 N gives 3.8 mm). The inward flex stops
  near 4 mm, where the real wall meets the ball.
- The roots' grip is a bond between ball and pot that lets go once two opposite panels have been pressed in at least
  3 mm, 4.5 cm up, for 1 s, the same check as the real trial. Unfreed, it holds to a 15 N pull; pulling the plant
  out that way is a fault.
- The sim ball is 7.5 cm across the top (real 8.0) so that the rigid ball leaves the wall room to flex, as real soil
  gives. The base layer is a rigid disc; no loose mix is modelled, so there is no spill measure, and a drop over 3 cm
  stands for a broken ball.
- Contacts are read from the simulator to apply the zero rules exactly as written above.

## Run a trial

Scripted demo (score 1.0):

```bash
TASK_DEMO=success inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
  -E control_hz=20 --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 2400
```

Any policy, here Dream-1, with the same command and `--policy dream -E cam_width=320 -E cam_height=176`.

The demo is an open-loop script of waypoints from the fixed start (`cactusdemo.py`): it reads no simulator state.
`TASK_DEMO=failure` runs the same plan without the squeeze, which pulls the plant against the pot's grip and scores 0.

Offline tests of the scoring rules (they import `yam_arena`, but need no simulator): `python -m pytest test_cactus_rules.py`.

Runtime: the demo finishes in 585 steps (29.2 s simulated); a full 2,400-step trial takes about 8 minutes of wall
time on one RTX 4090. The layout is fixed (`object_jitter=0`), so no seed is needed.

## Files

- `task.py`: the scene, `score()` and `success()`, the per-step contact readings.
- `cactus_rules.py`, `test_cactus_rules.py`: the scoring rules and their offline tests.
- `cactusassets.py`: the cactus, pots, collar and base layer.
- `cactusdemo.py`: the scripted demo.
- `bodykit.py`, `demokit.py`, `steptime.py`: helpers for multi-shape bodies, demos and step timing.

## Assets and licence

Every mesh is generated by the code in this folder from plain shapes; there are no third-party meshes, scans or
textures. The robot and the station come with the harness. Sizes come from the products named above. MIT.
