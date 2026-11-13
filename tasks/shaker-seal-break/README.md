# Open a cocktail shaker after shaking: hold the tins, strike the seam where the small tin leans away, lift it off

A closed tin-on-tin cocktail shaker stands on a bar mat after shaking: an 18 oz tin pushed mouth-down into a weighted 28 oz tin that holds the drink, sealed by the partial vacuum the cold drink leaves inside. A straight pull does not part the tins. The robot holds the big tin with one hand and strikes the big tin's rim with the other, at the seam, on the side where the small tin leans away and the gap between the tins is widest. The seal breaks; the robot lifts the small tin off, stands both tins on the mat, lets go and moves its hands clear.

**Where it comes from.** A tin-on-tin shaker seals itself on every shake, and every shaken drink ends with opening it. Difford's Guide teaches it as routine: "hold the larger can in one hand and break the seal between the two halves of the shaker" by "tapping the base can with the palm of your other hand at the point where it meets the upper can" (Difford's Guide, guide 1150 on diffordsguide.com). A Bar Above: "Hit your palm on the side of the shaker cups that has a little more space" (https://abarabove.com/how-to-open-a-stuck-cocktail-shaker/). America's Test Kitchen: "firmly tap the bottom cup just under where the two cups meet" (https://www.americastestkitchen.com/articles/7127-how-to-open-a-stuck-cocktail-shaker). Bartenders have the core task "Mix ingredients, such as liquor, soda, water, sugar, and bitters, to prepare cocktails and other drinks" (O*NET 35-3011.00, USDOL/ETA, CC BY 4.0; O*NET is a trademark of USDOL/ETA); there are 756,390 of them in the US (BLS OEWS, May 2025).

**What this task tests.** A placed blow on a tin held in the other hand. The blow must land at the seam, on the side where the small tin leans away and the gap is widest, and it must land: a punch pulled short of the tin does nothing. Carrying, rocking or pulling the shaker by the small tin does not open it. The big tin has to be held: in the `nohold` run, with nothing holding it, the small tin did not come off. Once the seal is open, a big tin tipped past 45 degrees spills the drink and scores 0.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "The drink is shaken. Open the shaker: hold the big tin, strike the big tin's rim at the seam on the side where the small tin leans away, lift the small tin off, set both tins upright on the mat, let go of them and move your hands clear."
```

A trial runs up to 2,400 steps (120 s of simulated time at 20 Hz). One fixed layout; no seed changes the scene.

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                                        # the full-score reference, filmed
python demokit.py task.py OUT --no-record --no-cameras --mode pulled,pull,tight,nohold,carry,rock # the failure checks
python check_score.py                                                                            # the blow gate, the holding test and the scoring table, no simulator
```

The demo is a scripted reference: it reads the tins' poses from the simulator, and a policy gets only the cameras. The left hand comes in level from the near side and closes its fingers either side of the big tin 9 cm above its base, at 40 N. The right hand, its fingers closed and pointing +y, comes level with the big tin's rim 5 cm out on the -y side, where the gap is widest, and strikes the rim at the seam with its fingertips (about 0.3 m/s at the rim), then draws back. The seal breaks. The right hand takes the small tin round its middle, lifts it 1 cm three times with a 6 degree twist each way, lifts it clear, carries it over the big tin's rim with 4 cm to spare, and stands it mouth-down on the mat beside the shaker. It lets go and rises clear; the left hand lets go of the big tin and draws back.

**Result** (scripted, GPU physics on an NVIDIA GeForce RTX 3090 Ti, 20 Hz, filmed at 640 x 360): 1.0 in 448 steps (22.4 s of simulated time), the blow landing at 0.32 m/s, 6 degrees from the open side; the film's first frame is the start state.

| Failure check | What it does | Score |
|---|---|---|
| `pulled` | the demo's blow at full speed, stopped 1 cm short of the rim | 0.0 in 502 steps: the finger points came no closer than 12.8 mm to the rim's wall; no blow landed, the seal held, and the pull then tipped the shaker over |
| `pull` | no blow: the right hand takes the small tin and lifts and twists it as in the demo | 0.0 in 432 steps: the seal held; lifted and twisted, the whole shaker tipped over in the holding hand |
| `tight` | the blow aimed 60 degrees round from the open side, then the pull | 0.0 in 502 steps: aimed there, the finger points came no closer than 6.2 mm to the wall, so no blow landed (the angle limit itself is checked in `check_score.py`); the seal held, and the pull tipped the shaker over as in `pull` |
| `nohold` | the demo with nothing holding the big tin | 0.25 in 382 steps: the blow opened the seal, but with nothing holding the big tin, the small tin did not come out of it |
| `carry` | no blow: the right hand grips the small tin 2 cm above the seam, lifts the shaker 1 cm and carries it 6 cm toward +y and back twice at 0.2 m/s, then lets go and lifts as in the demo | 0.0 in 454 steps: no blow (the fastest fingertip at the rim moved 0.09 m/s relative to the tin); the seal held |
| `rock` | no blow: the same grip, the shaker rocked 12 degrees each way three times, then the lift | 0.0 in 452 steps: no blow; the seal held, and the shaker tipped over (sealed, nothing spilled) |
| `check_score.py` | the blow gate on hand-built blows (the demo's blow: opens; 50 degrees round or on the tight side: holds; 0.10 m/s: holds; 1.2 m/s: a dent; 3 cm below the rim or 6 mm above it on the small tin: not a blow at the rim; a fingertip moving with the shaker: holds), the contact rule (fast and at the wall that step or the next: lands; a pulled punch stopped 8 mm out, the shaker jerked toward a fingertip hovering 8 mm off, a slow touch, or a rising swipe that passes over the rim: no blow; onto the rim's top edge: lands), the holding test (fingers either side of either tin: holding; a closed hand at the rim, or an open hand off the tin: not), and the scoring table on hand-built records | every case as expected |

## Scoring

| Score | When |
|---|---|
| 1.0 | the seal broken by a blow at the big tin's rim (a fingertip of a hand not holding a tin, in the rim's top 1.5 cm or up to 2 mm above it, from 1 mm inside its wall to 1.2 cm outside, moving toward the tin's axis at 0.15 m/s or more relative to the tin, within 45 degrees of the side where the gap is widest, and reaching the wall that step or the next: a point on the finger's middle within 5 mm of the wall's surface, or of the rim's top edge if above it, the finger's surface within about 1 mm), the small tin out of the big one, both tins upright within 10 degrees and standing on the mat (the big tin's base within 1 cm of it; the small tin mouth-down or mouth-up, within 1.2 cm), every fingertip of both hands 5 cm clear of each tin's side or 5 cm above its top, all held 1 s |
| 0.75 | the seal broken, the small tin out, the big tin upright within 10 degrees and standing on the mat (its base within 1 cm of it) with every fingertip of both hands 5 cm clear of it |
| 0.5 | the seal broken and the small tin out |
| 0.25 | the seal broken by such a blow (every tier needs it; on a real trial, a seal twisted, rocked or pried open scores 0) |
| 0 | at any moment: a blow of 1.0 m/s or more at the rim (a dent); once the seal is open, the big tin tipped past 45 degrees (the drink spilled); or a tin fallen off the table |

A hand is holding a tin when its two fingers are on either side of it, within 12 mm of its wall and more than 3 cm apart; its fingertips never count as a blow.

On a real trial: a top camera, both wrist cameras and a side camera level with the seam film the trial. The blow's speed and where it lands are read from the side camera's frames (at 120 frames per second or more). The seal counts as broken if, after the blow, the small tin lifts off by hand with no pull; for the 0.25 tier the judge checks this once the robot has stopped.

## Building it for a real trial

- **Tins:** a weighted 28 oz shaker tin and an 18 oz tin, stainless steel (a Koriko-style set; Koriko is named for identification only, and this task is not affiliated with its maker). Their sizes differ from the simulator's: see the table.
- **Seal:** before each trial, put about 150 ml of chilled water and ice in the big tin, seat the small tin with one firm press at its natural lean, and shake for 10 s, so the tins seal as they do at a bar. Check by hand that a straight pull does not part them.
- **Mat:** a black rubber bar mat, 30 x 42 cm, taped to the table. Turn the shaker on it so the side where the gap is widest faces -y.
- **Marks:** the judges' tape marks for the open side go on the table's underside or outside every camera's view; if they are in view, say so with the results.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Big tin's mass | 322 g, plus 150 g of drink | **Not measured.** A retailer's listing gives 322 g; weigh the tin |
| Small tin's mass | 150 g | **Not measured.** Weigh the tin |
| Tin sizes | big: 17.5 cm tall, 7.4 cm across the base, 9.2 cm across the mouth; small: 15 cm tall, 8.1 cm across the mouth | **Not measured.** A Koriko-style small tin is about 13 cm tall and the big tin's base about 6 cm across; measure both |
| Speed that opens the seal | 0.15 m/s toward the tin at the rim, within 45 degrees of the open side | **Not measured.** Strike the sealed shaker at the seam with a gloved hand, filmed at 120 frames per second or more, at rising speeds; note the slowest that opens it, and whether blows further round open it |
| Speed that dents the tin | 1.0 m/s | **Not measured.** Strike a spare tin's rim at rising speeds and look for a dent. The demo's blow is about 0.3 m/s; commanded faster, the arm's fingertips reached no more, so the dent limit is checked only in `check_score.py` |
| The small tin's lean | 2.5 degrees | Measure the gap at the seam on both sides with feeler gauges after shaking |

## What the simulator leaves out

There is no liquid: the drink is rigid mass in the big tin. The vacuum is not simulated as pressure: it is two latches inside the big tin's wall at opposite sides, reaching 1 mm over the small tin's rolled rim, locked until a blow at the seam on the open side opens them (a code coupling, stated above); opened, they draw 1.5 mm back into the wall (their outer faces 0.3 mm proud of it) and are hidden. A blow's force is not read, only the fingertip's speed toward the tin relative to it and its reaching the wall, and only from a hand that is not holding a tin. A real seal can also be twisted, rocked or pried open; here only a blow opens it, and on a real trial a seal opened any other way scores 0. The tapered walls are built from 16 flat staves each.

## Assets

Every shape in the scene (the tins, the latches, the mat) is built in code from lathed profiles, boxes and convex staves in `task.py`; no third-party meshes, textures or fonts are used. The YAM arms and the station come from `yam-isaac-lab-arena`.

## Licence

MIT
