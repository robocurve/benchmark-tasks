# Shut a full kebab box: hold the springy lettuce leaf tucked in while the other hand shuts the lid and clicks its tabs

A moulded bagasse 9 x 6 in takeaway clamshell stands open on the counter, filled with doner kebab. One lettuce leaf lies on the heap with its tip sticking out over the front rim, between the lid's two tabs. The leaf is springy: folded back in and let go, the tip flips back out over the rim, where the lid's front traps it, holds the lid a few millimetres proud and keeps the tabs from catching. So one hand lifts the tip, folds it back in and holds it there while the other swings the lid down; the tuck hand slips out as the lid's front wall comes over the tip, and the lid hand presses the lid's front until both tabs' barbs click under the base's flange.

**Ability tested:** two hands on one closure in order: a light hold of a springy, loose piece of food at the seam while the other hand closes the hinged lid over it and presses its two snap tabs home.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2000 \
  --instruction "Close the kebab box for takeaway. Tuck any salad hanging over the rim back inside, close the lid, and push its tabs into their slots so they lock."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                         # full score, with a video
python demokit.py task.py OUT --no-record --mode none,one_hand,tuck_release,push_release,hold_in,no_press
python check_score.py                                                             # the scoring table and the inside test, with no simulator
```

The demo steers on the box's pose and the lid's and leaf's joint angles, which a policy does not get. The right hand, fingers nearly level and stacked one above the other, comes up under the leaf's tip from outside the front flange, lifts it along its arc and pushes it over, so the tip lies folded back on the heap under the fingers. The left hand pushes the lid's outer face with the flat of its fingers and swings it down until it rests on the right hand's fingers, about 13 degrees open. The right hand slips out under the lid's front edge while the left hand follows it down, so the tip springing back meets the inside of the lid's front wall. The left hand pushes the lid shut, slides along the roof to the front between the two tabs and presses down with about 16 N; each barb rides over its notch's edge and clicks under the flange, and both hands leave.

## Scoring

1.0 at the end: both tabs' barbs caught under the base's flange (real trial: lift the box by its shut lid; it stays shut), every food piece inside the box's walls (real trial: phone photo from directly above, judged against the inner wall), the box flat (real trial: phone level app on the lid), both whole grippers (fingers, pads, palm and housing) 3 cm clear of the box (real trial: a 3 cm wooden block as gauge) and everything still for 2 s (real trial: phone timer). Otherwise the highest substep reached during the run: 0.7 the lid down (its front edge within 10 mm of the flange; real trial: steel ruler against the flange, read from the front) with every food piece inside, so the seam is clean; 0.3 the lid down with food in the seam or outside the box, which is where a latch over the leaf ends. 0 if the box tipped past 30 degrees (real trial: phone level app on the box) or left the counter.

"Inside" tests every vertex of every food piece and of both leaf parts against the base's inner walls (1 mm slack) and the shut lid's roof, so a leaf across the seam, a piece on the flange, on the lid or on the counter all fail it. "Caught" reads each barb's tip from its tab body's pose: past the notch's inner edge and under the flange's lower face, with the lid within 2 degrees of shut.

Dream-1 run (a model, not the scripted demo): 0, valid, one session, 2,000 steps (the step limit).

## Lazy routes (GPU, 320 x 176)

| Mode | What it does | Score |
|---|---|---|
| full | the demo | 1.0 |
| none | arms wait | 0.0 |
| one_hand | the left hand shuts the lid and presses with the leaf left over the rim: the leaf lies in the seam, the lid stays about 2 mm proud, the barbs cannot reach under the flange | 0.3 |
| tuck_release | the right hand folds the leaf in and leaves before the lid comes: the tip springs back over the rim into the seam | 0.3 |
| push_release | the right hand pushes the tip straight in at rim height and leaves: the full box gives the leaf no room to slide in, and the lid shuts over it | 0.3 |
| hold_in | as full, but the right hand stays on the tucked tip while the left shuts and presses: the lid stops on the fingers about 13 degrees open | 0.0 |
| no_press | as full without the press: the lid is down with all food in while held, then springs open | 0.7 |

## What is real and what is not

- The box is a free body; the lid and its two tabs are bodies on sprung hinges, and the barbs, notches and flange are colliders, so the latch holds or fails by contact. The hinge, tab and leaf spring values are estimates, tuned to the forces stated in task.json.
- The leaf is drawn as an irregular wavy-edged sheet; it is two rigid plates on a sprung hinge, standing in for a stiff leaf; the meat, lettuce, tomato, onion and chilli are rigid bodies with meshes of strips, slices and rings.
- Code does no part of the act: it only reads poses and joint angles to score.

## Sizes and sources

- Box: BioJX 9 x 6 in bagasse clamshell, 230 x 158 mm, 46/80 mm, 30 g, "reinforced Tab-lock system" (biojxcutlery.com/disposable-bamboo-utensils/bagasse-clamshell-containers/9x6-bagasse-clamshell.html); Vegware's 9 x 6 in large bagasse clamshell is 228 x 155 x 74 mm (vegware.com/uk-en/catalogue/takeaway-boxes/9-x-6in-large-bagasse-clamshell/). Tab and notch sizes are estimates from product photos. A pack of these boxes costs a few pence each.
- Food: about 350 g of doner kebab, heaped about 13 mm above the rim, as a takeaway box is filled.
- Press: the demo's press is about 16 N through the flat of the fingers, below the YAM's roughly 40 N grip and push figure; the leaf is lifted with fingertip forces of a few newtons.

## Real trial, reset and protection

- Reset (about 3 minutes): a fresh box with its lid open; weigh out about 350 g of cooked doner meat and salad from one shop and lay them to the photo of this layout, the leaf's tip over the front rim between the tabs. Bill of materials: the box (listing above, a few pence each), food from a kebab shop, a kitchen scale (about 10 pounds, 1 g resolution).
- Scoring rules in the real trial are the simulated ones: "caught" is the box lifted by its shut lid staying shut; "clean seam" is a photo along the front showing no food between lid and flange; "released" is both grippers 3 cm clear and still for 2 s. Tab bend, lid spring and leaf spring are estimates here and should be measured on the real parts.
- Protection: soft pad covers on the fingertips so the bagasse is not scored or crushed, and a tray under the box to catch food.

## Importance figures and sources

- Fast food and counter workers, about 3.7 million in the US, 3.1 million of them in restaurants: BLS Occupational Employment and Wage Statistics, "Fast Food and Counter Workers" (SOC 35-3023), https://www.bls.gov/oes/current/oes353023.htm
- Core task "Wrap menu items such as sandwiches, hot entrees, and desserts for serving or for takeout": O*NET-SOC 35-3023.00, "Fast Food and Counter Workers", https://www.onetonline.org/link/summary/35-3023.00
