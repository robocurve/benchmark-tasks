# Play whack-a-mole with the game's mallet: bop each mole the moment it lights, never one that is dark

**Instruction:** "Play one round of the whack-a-mole game with its mallet: pick the mallet up from the desk, then each time a mole lights up, bop that mole down with the mallet's head before its light goes out. Do not bop moles that are not lit, do not touch the moles with your fingers, and do not touch the game's switch or buttons: the round is started for you."

A light-up tabletop whack-a-mole game stands on the desk. Before the trial the person running it switches it on and
chooses one player and the easier level on the unit's controls (on the deck's far edge); the robot must not touch
them, and touching them scores 0. Its five moles (four
round a middle one) stand up out of the deck; one of the game's two plastic mallets lies on the desk to its right.
The robot picks up the mallet and plays one round: the unit lights one mole at a time, at random, for a short time,
and the robot bops each lit mole down with the mallet's head before the light goes out.

## Scoring

Score 0 if a mole was pressed with a gripper finger instead of the mallet, the robot (a finger or the mallet's head) touched the unit's on/off switch or buttons, the mallet fell off the desk, the game unit was shoved more than 30 mm or tipped more than 10 degrees, or no lit mole was hit. Otherwise the lower of two numbers. Hits (lit moles bopped down past their switch while lit by the head of the held mallet; a press by the handle, a loose mallet or the gripper puts the light out with no hit; the round lights 8): 8 = 1.0; 6-7 = 0.75; 4-5 = 0.5; 1-3 = 0.25. Wrong bops (a mole bopped while not lit): none = 1.0; 1-2 = 0.75; 3-4 = 0.5; 5 or more = 0. Doing nothing scores 0. Success: 1.0 (all 8, no wrong bop) held 1 s. In a real trial: count lit-mole hits (lights put out by the head of the mallet in the hand) and unlit bops on the trial video; the video shows any touch on the switch or buttons, and an end photo shows the unit still on, at the easier level, inside its tape outline.

| Outcome | Score |
|---|---|
| All 8 lit moles hit, no wrong bop | 1.0 |
| 6-7 hits and at most 2 wrong bops, or 8 hits with 1-2 wrong bops | 0.75 |
| 4-5 hits and at most 4 wrong bops, or more hits with 3-4 wrong bops | 0.5 |
| 1-3 hits and at most 4 wrong bops | 0.25 |
| No hit; or 5 or more wrong bops; or a mole pressed with a finger; or the switch or a button touched; or the mallet off the desk; or the unit shoved over 30 mm or tipped over 10 degrees | 0 |

The tiers are the lower of the hit number and the wrong-bop number, so they do not overlap and every outcome has one
score. The start scores 0 (no round played, no hit). Bopping every mole in turn without looking scores 0: it lands
far more wrong bops than hits. Pressing the lit moles with the fingers scores 0: it is not playing with the mallet.

**Real trial, read from the video.** Each lit mole's light goes out the moment it is bopped down; a person counts the
lights that went out under the head of the mallet held in the gripper (hits) and the bops on dark moles (wrong bops)
from the trial video (the unit's own count at the end cannot tell the head from the handle, so the video decides). A
touch on the switch or buttons is visible on the video; an
end photo shows the unit still on, still at the easier level, and inside its tape outline (ruler for the 30 mm).
What a viewer sees: a red light jumping from mole to mole and the mallet coming down on each lit mole, which goes
dark as it is hit.

## The product and its numbers

- A light-up tabletop whack-a-mole game, for example the Mattel WHAC-A-MOLE kids' arcade game (model GVD47), about
  US$20. A published review (https://ttpm.com/products/whac-a-mole-game/) describes "two mallets (one red and one
  green) and a light-up game unit with five moles"; the moles do not pop up: "they're lighting up and not lighting
  up". Solo play: "use one of the mallets to bop any mole that is lit"; "If you accidentally bop a mole that is not
  lit, you lose points from your total"; "Within each player mode, there are two levels of difficulty"; the goal is
  "to bop the most moles before time runs out", and at the end "the unit will tell you how many moles you got".
- Size and weight from retailer listings of that model: 10.5 x 10.5 x 4 in (267 x 267 x 102 mm), 1.3 lb (0.59 kg).

Estimates (not published; how to check in a minute): the layout (four moles round a middle one, 75 mm apart along each
axis) and the deck height (70 mm): ruler; the hard-hat dome (37 mm across, top 104.5 mm above the desk) and the press
travel (9 mm to the bottom, switch at 4 mm): ruler; the spring (1.2 N to start a mole moving, 2.3 N at the bottom):
kitchen scale pressed on a mole; the mallet (head 40 mm across and 65 mm long, handle 18 mm round, 145 mm overall,
35 g): ruler and kitchen scale; at the easier level a mole stays lit 1.6 s and the gaps between lightings are
0.6-1.4 s, 8 lightings a round: stopwatch on the trial video. Total cost of real objects: the game, about US$20, plus
three AA batteries.

## How the simulation works

The unit is one articulated body: a free base on rubber feet and five slide joints, each mole held against its top
stop by a preloaded spring. The game's electronics are a small script (`whack_kit.py`): after the start it waits a
random 0.6-1.4 s, lights a random mole (never the one just lit), and keeps it lit for 1.6 s or until it is hit; a
mole pushed 4 mm down registers a press, as the real switch does (a switch closing again within 0.25 s of
opening is the same press, so a mole bouncing under one blow is one bop). Dark means a wrong bop, whatever pressed it.
Lit means a hit only when the head of the held mallet pressed it; the light goes out at once either way, so a lit mole
pressed by anything else (the handle, a loose mallet, a finger or the gripper) is a lost lighting with no hit, and a
finger or gripper press scores the trial 0. Order and gaps are drawn from a new seed every episode, so nothing about the round can be learned
in advance. The light is a red lamp drawn over the mole's dome. Nothing moves a mole except contact.

What pressed a mole is judged from the poses at the control step its switch closes, in this order. A gripper point
(tool point, pad centres or fingertips) within 26 mm sideways of the dome's top and less than 12 mm above it: the
gripper (a finger press). Otherwise the mallet's head (its 40 x 65 mm drum and end caps) within 3 mm of the dome and
above it (within 60 degrees of straight up from the dome's centre): the head, counted as the held mallet's head only if
the mallet is held, else a loose mallet. Held means one gripper's jaws have stopped on the handle: the jaw gap is the
handle's 18 mm +/- 3 mm, both pad centres lie along the handle's length, the line between them crosses the handle within
45 degrees of square, and the handle's axis passes within 8 mm of their midpoint. Otherwise the handle within 3 mm of
the dome: the handle. The tests in `test_hit_attribution.py` press a lit mole each of these ways.

Scripted demo: the right hand picks the mallet up by the handle, pinched top-down 40 mm behind the head, and holds it
over the middle mole. Every 0.8 s it looks at the unit; when a mole has been lit for under 0.95 s it moves the head
over that mole (0.3 s), bops it down (0.2 s), holds 0.1 s and lifts; otherwise it waits. It aims
with the head's position relative to the hand, read at each look. The demo reads which mole is lit from the game
script; a policy reads it from the camera.

Demo result: 1.0 (demo-8a164e9e-a1, full length: all 8 lit moles hit with the head of the held mallet, no wrong bop, success at 20.3 s). In three earlier jittered probe episodes the same demo hit 8 of 8 each time (1.0, 0.75 with one wrong bop from a mole bouncing under one blow, now one press under the 0.25 s debounce, 1.0); a player that strikes only once a mole has been lit 1.25 s hit nothing and made 8 wrong bops (0).

Wrong way: bopping every mole in turn every 0.8 s without looking: 5 hits, 78 wrong bops, score 0 (5 or more wrong bops).

Scores: Dream-1 run: 0.0; demo: 1.0

## Fidelity and shortcuts

- The game's electronics are a script with estimated timings; each hit counts 1 (the real unit's bonus for the
  middle mole is left out). A hit still needs a real press past the switch, made with the head of the held mallet.
- Moles are rigid sliders on springs; the mallet is rigid (the real head is hollow and slightly soft).
- Built from simple shapes in the game's colours; no lettering or logos.
