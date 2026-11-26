# Play a tune on a xylophone with two mallets

A student xylophone (eight coloured bars C D E F G A B C of graduated length on a wooden frame with felt rails), two
mallets lying beside it, and a song card behind it showing seven coloured squares: C C G G A A G. The robot has to play
the tune, one mallet in each hand, and put the mallets back down.

Instruction: "Play the tune on the card, one mallet in each hand."

| File | What |
|---|---|
| `xylophone.py` | scene, `score()` and the scripted `demo()` |
| `mech.py` | helper (welds the mallet heads to their handles) |
| `test_xylophone.py` | offline scorer tests (`python test_xylophone.py`) |
| `task.json` | the task's details |

## Run

    -E task=xylophone/xylophone.py --scorer task_score -E control_hz=20 -E object_jitter=0

with the `yam_scripted` policy (plays `demo()`, about 25 s) or the Dream-1 model with the instruction above, run limit
1,200 steps (60 s at 20 Hz). Runtime per trial about 2-4 minutes. No seeds: the layout is fixed.

## Scoring

Strike = a mallet head arriving on a bar (lowest point within 6 mm of the bar's top, centre over the bar, moving down at
0.15 m/s or more just before contact, then lifting back up), counted once until the head rises 1 cm. Score = 0.6 x k / 7 for the first k
strikes matching C C G G A A G in order (the first wrong or extra strike ends the count), plus 0.4 when all seven were
played with no wrong or extra strike, each mallet played at least 2, and at the end both mallets lie back on their stands, both grippers at least 7 cm away, still for 1 s. Doing nothing, or a mallet off the table: 0.

## Real-world build

Pine board 24 x 40 x 1 cm, two felt strips 1.5 x 36 x 1 cm, eight bars 3.5 x 0.9 cm of the lengths in `task.json` (any
8-note diatonic student xylophone or glockenspiel with similar bars works; the lengths set which bar is which), two
mallets with 22 cm wooden handles and 2.5 cm wooden ball heads, and a 36 x 20 cm song card with seven 4 x 4 cm coloured
squares (and printed letters). All off-the-shelf; well under 50 USD.

## License

MIT.
