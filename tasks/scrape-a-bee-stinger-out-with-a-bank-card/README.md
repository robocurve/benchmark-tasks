# Scrape a bee stinger out with a bank card's edge

**Instruction:** "Get the bee stinger out of the back of the hand with the bank card standing in the block, then lay
the card down on the table and let go. Only the card may touch the stinger."

A honeybee leaves its stinger in the skin with the venom sac attached. Public first-aid advice is to scrape it out
sideways with a straight edge, never to pinch it: MedlinePlus (Insect bites and stings,
https://medlineplus.gov/ency/article/000033.htm) says to scrape the back of a credit card or other straight-edged
object across the stinger and not to use tweezers, which may squeeze the venom sac; the NHS
(https://www.nhs.uk/conditions/insect-bites-and-stings/) says to scrape it sideways with a fingernail or the edge of a
bank card.

**Why it is hard.** The hand must:

1. lift a 0.76 mm card out of a 1 mm kerf by its top edge;
2. tilt it and bring its lower edge down onto soft skin (a pad that dents at 1 N per mm);
3. drag the edge sideways across the sting site, low enough to catch a 3 mm stub at its base (a 4 mm high pass misses
   it) and light enough not to press it in, so that it tips over and slides out of its track;
4. lay the card flat on the table and let go.

The natural shortcuts each fail: a pinch with the gripper is the tweezer move the sources forbid (a latched 0); the
card pressed down on the stub pushes it in (a latched 0); an edge passing over the bead leaves it in.

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=scrape_a_bee_stinger_out_with_a_bank_card.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the bee stinger out of the back of the hand with the bank card standing in the block, then lay the card down on the table and let go. Only the card may touch the stinger." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir logs/stinger-dream

# The scripted one-arm demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=scrape_a_bee_stinger_out_with_a_bank_card.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Get the bee stinger out of the back of the hand with the bank card standing in the block, then lay the card down on the table and let go. Only the card may touch the stinger." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir logs/stinger-demo
```

The run prints `task_score`, read at the end (latched zeros aside). Runtime per trial: at most 1,500 steps (75 s
simulated). The demo's plan lasts about 31 s (623 steps in the MuJoCo check). The layout is fixed
(`object_jitter=0`) and there are no seeds. Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`.

## Scoring (`grade()` in the task file)

**Drawn** is how far the stinger has slid out along its track (3 mm in at the start); **out** is drawn 3 mm;
**tipped** is its lean from upright.

| Score | State |
|---|---|
| 0 | anything else, including the untouched start, or a latched 0 |
| 0.25 | the card has tipped the stinger 30 degrees or more, at any moment |
| 0.5 | the stinger is out |
| 0.75 | also the card lies flat on the table (highest corner under 5 mm; not on the hand or either block), no gripper touching it |
| 1.0 | also both grippers 30 mm from the hand and card, the hand moved no more than 5 mm, everything still for 1 s |

Latched 0, at any time: a gripper finger touches the stinger (0.02 mm); the stinger pushed 2 mm further into its
track; the hand off its block (2 cm or 10 degrees).

Real trial: the side camera beyond the table's far edge, level with the back of the hand and looking back along it, sees the scrape side-on: the lean is read on its frames with an on-screen protractor, and "pushed in" is the stand-in's white paint ring (2 mm above the skin at the start) going into the skin. A 5x magnifier after the trial shows whether any of the stand-in is left in its track; the top video shows the hand against the block's edge and the card on the table.

Dream-1 refused the instruction at step 32 ("Instruction involves using a card to manipulate a bee stinger in a hand."), so it has no score.

**Safety.** Real trials are safe: no sharps reach the robot (the 0.5 mm needle is used only to make a track, by hand, before trials), nothing is hot, the card is a 5.5 g blank PVC card, and the scrape presses the silicone hand at about 0.5 N (forces stay under 1 N). The stinger is a nylon stand-in; no insect or venom is used.

## The sim (`scrape_a_bee_stinger_out_with_a_bank_card.py`)

- The hand is boxes on one 0.5 kg body lying loose on the block (strapped in the real setup); a 100 x 70 mm skin pad over its back is a 4 mm slider
  that dents at 1 N per mm.
- The stinger collides as a 1.5 mm square stick 3 mm out, on two lean hinges at the skin (0.003 N m/rad) and a slider
  along its track against 0.5 N of friction; once out it stays on its line, up to 8 mm out.
- The card is a free 85.6 x 54 x 0.76 mm box (5.5 g) standing in a 1 mm kerf.
- Nothing is moved, latched or held by code; the scorer reads poses and joint positions every step.

## The demo

The right arm pinches the card's top 23 mm from above, lifts it out, tilts it 20 degrees so its lower edge trails,
lowers the edge 15 mm to the left of the stinger until the skin dents, then drags it 60 mm to the right at 10 mm/s
while a servo holds the dent at 0.5 mm. The edge catches the stub's base, tips it and drags it out. The arm lifts,
stands the card's edge on the table, lets it fall flat, and goes home. The left arm stays home.

Cheat modes (`KIT_DEMO=<mode>`, checked in the tests): `none` 0, `pinch` (the gripper closes on the stinger) 0,
`press` (the card pushed down on the stub) 0, `high` (the edge 4 mm over the skin) 0, `held` (out, card kept) 0.5.

## Real-trial kit

Silicone mannequin hand (display prop, about US$65), pine block, two hook-and-loop straps; blank CR80 PVC cards; a pine
block with a 1 mm saw kerf; 0.5 mm black nylon monofilament, black UV resin and a UV torch (a stand-in takes about 3
minutes to make); a 0.5 mm needle; white paint for the ring; a 0 to 2 N push-pull gauge; a 5x
magnifier; a phone on a tripod as the side camera.
