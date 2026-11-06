# Settle a blackjack: pay 3 to 2 beside the bet and rack the busted bet

A blackjack dealer settles a hand: the busted bet on the right goes into the chip rack, and the blackjack on the
left, a 2-chip bet that pays 3 to 2, is paid 3 chips from the rack. House rules put the payout as its own stack beside
the original bet, never on top of it ("capping"), so the surveillance camera can still see what was wagered. The
rack is in the right arm's reach and the left circle in the left arm's, so every payout chip crosses the midline on a
handover square.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pay_blackjack_beside_the_bet.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "You are the dealer. The right player busted: clear their red chip off the right green circle into the grey chip rack. The left player has blackjack on a 2-chip bet, which pays 3 to 2: pay them from the rack as one separate stack standing on the left green circle, at most 2 cm from their bet and never on top of it, and leave their bet where it is. The yellow square is a handover spot both arms reach; leave it empty." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/blackjack-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pay_blackjack_beside_the_bet.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "You are the dealer. The right player busted: clear their red chip off the right green circle into the grey chip rack. The left player has blackjack on a 2-chip bet, which pays 3 to 2: pay them from the rack as one separate stack standing on the left green circle, at most 2 cm from their bet and never on top of it, and leave their bet where it is. The yellow square is a handover spot both arms reach; leave it empty." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/blackjack-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/blackjack-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the payout, the collect and the untouched bet all
hold with every chip still for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_pay_blackjack_beside_the_bet.py` in an
environment with `yam_arena` installed.

## Scoring

The six chips are identical and are scored by where they stand, not by which chip is where. A chip stands flat when
its axis is within 10 degrees of vertical (either face up) and it is still. A chip's bottom is seated at a level when
it is from 4 mm below to 1 mm above that level: circle A's top, then 30, 60 and 90 mm above it. A stack is flat chips
seated level by level, one per level, each axis within 15 mm of the lowest chip's axis (each chip more than half on
the one below); it stands on circle A when its
lowest chip is seated on the circle with its axis within 75 mm of A's centre. Distances are horizontal, between chip
axes and the circles' and wells' marked centres. The bet is the stack standing on the chip seated on circle A nearest
A's centre, if that chip's axis is within 30 mm of A's centre; the bet's axis is the mean of its chips' axes.

| Credit | For |
|---|---|
| 0 overall | the bet broken or gone: that stack is missing or is not exactly 2 chips (a top chip whose axis is more than 15 mm from its base chip's axis breaks it) |
| 0 overall | the bet capped: a chip whose axis is within 38 mm of the bet's axis has its bottom 56 mm or more above circle A (the third level, 4 mm tolerance) and within 1 mm of the top of another chip there, whether it is still or moving |
| 0.60 | the payout: a stack of exactly 3 chips standing on circle A whose lowest chip's axis is 40 to 60 mm from the nearer bet chip's axis (a gap of 0 to 20 mm), and no chip other than the bet and that stack within 95 mm of A's centre. A stack of 1, 2, 4 or more chips gets 0 here |
| 0.25 | B collected with no chip loose: no chip within 95 mm of B's centre, at least one chip in a rack well, and every chip other than the bet either seated flat on the table in a rack well (axis within 10 mm of the well's centre) or in one single stack standing on circle A. Any well may be used, and B's chip may go into the payout, since the chips are identical; with the rack empty, B's chip cannot be in it, so this credit is lost |
| 0.15 | the bet untouched: both its chips' axes within 5 mm of A's centre, given only when at least one chip stands flat on circle A, seated on the circle, with its axis 40 to 60 mm from the nearer bet chip's axis |

Some outcomes: settled correctly, 1.0. Paid and untouched but B's chip left on circle B, 0.75. Even money (a 2-chip
payout, one bank chip left in the rack) with B racked and the bet untouched, 0.40. An overpay (4 chips in one stack,
which uses every non-bet chip and leaves the rack empty) with the bet untouched, 0.15. B racked and nothing else done,
0.25. Everything done but the bet shoved 20 mm, or its top chip nudged 9 mm on its base, 0.85; shoved 28 mm away from
the payout so their axes are 78 mm apart, 0.25 (the collect only). A payout chip left on the yellow square, still held
above the stack, still moving, dropped loose or set as a second stack, with the bet untouched, 0.15. All four non-bet
chips stacked on circle A with the rack empty, not beside the bet, 0. A chip capped on the bet (still or sliding), the
bet's top chip more than 15 mm off its base, the bet shoved more than 30 mm or taken away, or nothing done: 0.

On a real table, a ruler from the taped centres measures each stack's axis, a ruler beside a stack its levels (30,
60, 90 mm), and a photo from above shows that circle B, the square and the floor are clear.

## The scripted demo

Both arms grip each chip from above with the fingers closing front to back, 4 mm inside the chip's sides so the grip
centres it, and let go 5 mm above where it lands after a slow last 3 cm. The right arm moves the bank chip from the
well nearest the midline to the yellow square, takes B's chip to the empty outermost well, then feeds the other two
bank chips to the square one at a time, waiting above the rack with the last one until the left arm has stacked the
previous chip and lifted off the payout, so the two arms never work within 12 cm of each other. The left arm waits beside circle A, takes each chip off the square and stacks the payout 5 cm
right of A's centre (toward the midline), a 10 mm gap from the bet; its fingers sit in front of and behind the payout,
away from the bet. Both arms then park in front of their own bases.

## Runtime and seeds

- The demo's plan takes about 66 s of simulated time (the left arm's plan is the longer); the last chip lands at
  about 63 s, and with the 2.5 s success hold the trial ends at about 66 s, about 1,320 steps at 20 Hz, 55% of the
  2,400-step limit. On an L40S, once Isaac Sim has started,
  a scripted trial takes about 2 minutes of wall time and a full Dream-1 trial 4 to 5 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
