# Salt half a rocks glass's rim on its outer edge only

A Libbey 8 oz rocks glass stands upside down on a rubber bar mat beside a lime half (cut face up in a portion cup) and
a bread plate with 2 mm of coarse kosher salt. The robot half-salts the rim the way bar guides teach it, then stands
the glass upright on the mat's taped fill spot, ready to pour.

> Salt half the rim of the upside-down rocks glass for a margarita: wet the outside of its rim on the lime, roll the
> outer edge through the salt, keep salt out of the glass, hold it still over the plate, then stand it upright on the
> taped circle on the mat.

**Why it is hard.** Tilted 45 degrees, the 3 mm wall puts the inner edge 2.1 mm above the outer edge: rolled on the
plate floor through 2 mm of salt only the outer edge takes salt; tipped under 19.5 degrees or dunked upright, salt goes
inside (score 0). Then the glass must come rim up and be set down without the salted rim touching anything.

Sources: Cook's Country (wet the outer half-inch), https://www.americastestkitchen.com/cookscountry/how_tos/13644-how-to-rim-a-glass-with-salt-or-sugar;
MasterClass (hold at 45 degrees and spin), https://www.masterclass.com/articles/how-to-salt-a-rim; Diageo Bar Academy
(half the rim), https://www.diageobaracademy.com/en_zz/bar-trends-tips/tips/the-margarita.

## Run a trial

In the simulator's environment (yam-isaac-lab-arena), from this folder:

```bash
I="Salt half the rim of the upside-down rocks glass for a margarita: wet the outside of its rim on the lime, roll the outer edge through the salt, keep salt out of the glass, hold it still over the plate, then stand it upright on the taped circle on the mat."
for POLICY in dream yam_scripted; do
  inspect-robots run --embodiment yam_arena --policy $POLICY \
    -E task=salt_a_rocks_glass_rim_outer_edge_only.py -E object_jitter=0 -E control_hz=20 \
    --instruction "$I" --scorer task_score --max-steps 3600 --store-frames --log-dir logs/salt-$POLICY
done
```

Failure variants: `KIT_DEMO=none`, `slam`, `wrong_order` or `held` in the environment. Offline tests:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q` with `yam_arena` importable.

## Runtime and seeds

One fixed layout (`-E object_jitter=0`): the seed does not change the start. The demo reaches 1.0 in about 105 s of the
3,600-step (3 min) limit; Dream-1 runs to the limit unless it reaches 1.0 first.

## Runs

Isaac (PhysX), this task file: scripted demo 1.0 in 105 s (21 sectors salted, tilt held 46-48 degrees, inner edge never
under the salt, stood upright on the circle); Dream-1 0.0 (hovers round the glass, never lifts it).

## Scoring

The rim is counted in 36 sectors of 10 degrees.

| State | Simulator | Real bench |
|---|---|---|
| Wet sector | outer edge within 1 mm of the lime face, 22 mm of its centre, the face pressed 0.7 mm (1.0 N) | the juice |
| Salted | wet, then outer edge 0.5 mm or more under the salt surface inside the 120 mm disc | a band at least 2 mm wide |
| Salt inside (0, latched) | inner edge more than 1 mm under the salt surface | 3 or more grains inside |
| Touched (0, latched) | once wet, a rim point within 1 mm of the mat or the plate's raised edge | set on its rim, tipped over, dropped |
| Held over the plate | rim down (axis within 60 degrees of vertical), rim 30 mm above the raised edge, moves less than 2 mm for 3 s, after the last sector took salt | the video |
| Set down | upright (within 10 degrees), base within 2 mm of the mat, centre within 30 mm of the circle's, still (2 mm) for the last 3 s, grippers 30 mm away | video, photo |
| Dropped (caps 0.75) | once wet, its centre of mass falls 15 mm or more in one 50 ms step, no gripper within 10 mm | the video |

| Score | When |
|---|---|
| 0 | a latched zero, or never lifted with the rim 30 mm clear of the mat (doing nothing) |
| 0.1 / 0.25 / 0.5 | lifted 30 mm clear / 5 or more salted / 9 or more |
| 0.75 | 18 or more salted, but over 24, or not then held over the plate and set down (still held, tipped, off the circle, dropped) |
| 1.0 | 18 to 24 salted, held over the plate after the last, then set down and let go for 3 s, never dropped |

**Measuring it in a real trial.** The video shows the hold and the set-down (set square, tape circle, grippers clear,
still 3 s). The operator photographs the glass on the 36-sector card from 30 cm (salted sectors; grains inside against
black). The white salt shows on the clear rim; for the video, use salt tinted with food colouring or a dark plate.

## The real bench

| Object | Product | Size and material | Where (cm, x forward, y left) |
|---|---|---|---|
| Bar mat | Tablecraft 1218BK | 305 x 457 x 9.4 mm black rubber | centre 30, -14; taped |
| Plate | Corelle 6003887 bread and butter | 171 mm, 22 mm tall; floor flat to 6 cm, 5.6 mm up | centre 34, -23; taped |
| Salt | Morton Coarse Kosher Salt | 22 g, 120 mm disc levelled to 2 mm | on the plate |
| Cup and lime | Dart 200PC 2 oz cup; a fresh Persian lime half | face 40 mm above the mat | 36, -6; taped |
| Fill spot | white masking tape | 70 mm circle | centre 27, -4 |
| Glass | Libbey 15454 Winchester 8 oz rocks | 86 mm tall, 79 mm rim, 3 mm wall, 260 g | 24, -12, upside down |

## Simulation

The glass is one free rigid body (36 tapered wall slabs, a 17 mm base); mat, plate and cup are one 30 kg body (taped).
Friction, static equal to dynamic: glass 0.4, lime and mat 0.5. The lime face is a sprung slider (1 mm per newton,
0.3 N preload, an estimate). Salt and juice are a rule on poses; the salt is drawn with no collider and the rim rests on
the plate floor under it. Nothing is moved by code.

## The scripted demo

It reads the glass's pose and the lime face, as a camera could. The YAM can't point a grip upward, so the right hand
salts and the left stands the glass up.

1. Right: grip the body at its centre of mass, tilt 50 degrees (the guides say about 45), press the outer edge on the
   lime and dab round nine times by 20 degrees; over the salt, lower until the outer edge rests on the plate floor
   under the 2 mm of salt (the floor sets the depth; a 1 mm light press), then nine dabs back (lift 5 mm, turn 20
   degrees, land at 5 mm/s), re-aiming the rim's low point every step; hold it still over the plate.
2. Hand-over: the right turns it level, rim forward; the left grips it 36 mm from the rim; the right lets go.
3. Left: pitch the grip to level in three moves (the rim comes up), set the base on the fill spot, let go, go home.

| Variant (offline MuJoCo) | What it does | Score |
|---|---|---|
| `none` | both arms stay home | 0 |
| `slam` | no lime; dunked straight down into the salt | 0 (salt inside) |
| `wrong_order` | salt first, then the lime | 0.1 |
| `held` | the left hand never lets go | 0.75 |
| demo | as above | 1.0 |

## Safety

A tempered glass, a lime and dry salt; nothing sharp, hot or powered. Run behind the cell's guard with the e-stop in
reach.

## Licences

The task's code and text are released under the MIT licence (code) and CC BY 4.0 (text). No third-party assets.
