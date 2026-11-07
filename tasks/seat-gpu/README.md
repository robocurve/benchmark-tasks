# Seat a graphics card in a keyed x16 slot: connector, bracket gap and far guide engage at once, so it must go in square

A data-center install step at full size. A dual-slot graphics card (254 mm long, 111 mm tall, 350 g) stands in a foam
stand beside a fixed server riser board. The robot lifts it out, lines it up over the riser's x16 slot and presses it
straight down until it is seated. The source is a server vendor's GPU install procedure: "Align the GPU adapter with the
PCIe slot on the riser card. Then, carefully press the GPU adapter straight into the slot until it is securely seated"
([Lenovo ThinkSystem SR645 V3, Install a GPU adapter, step 3.2](https://pubs.lenovo.com/sr645-v3/install_a_gpu)).
Data-center technicians do this when they add or replace a GPU. A card left part way in is not detected or does not
work: Lenovo's troubleshooting for "PCIe adapter is not recognized or is not functioning" checks that "The device is
seated correctly" ([Lenovo ST50 optional-device problems](https://pubs.lenovo.com/st50/optional_device_problems)).

The hard part is that three features along the 254 mm card engage together, so the card has to come down square, not
only centred. The key notch in the gold edge connector has to meet the slot's key rib (0.4 mm of play each side), so
the card fits only with its bracket toward the bases. The 1 mm steel bracket at the near end goes into a 2.9 mm gap.
The far end goes into a support channel 0.5 mm wider than the card each side. The connector itself is a 1.6 mm board
in a 2.0 mm slot, 0.2 mm of play each side; over the connector's 85.3 mm that allows about 0.27 degrees of turn, while
1 degree of turn puts the far end 4.4 mm off. A card whose notch lands on the key rib rests 3.5 mm high, which is hard
to see from the camera above. In the sim, the scripted demo set down 1 mm off across the slot, or carried 0.5 degrees
off square, starts the card into the slot and does not seat it.

## The scripted demo
`demo()` uses the right arm. It grips the card's sides 30 mm below its top edge, lifts it out of the foam, carries it
over the riser and stops above the support channel. It lowers the card slowly in three stages (into the channel,
bracket into its gap, connector into the slot), pausing to settle before the first two, pushes it down into the slot
and lets go. Under `inspect-robots run` the card levels in the slot as the fingers open, and success fires at step 536
while the arm rises; the press that follows in the plan does not run. Started a little later, the same demo can seat
the card while still holding it, which the scorer counts the same.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=seat_gpu.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the graphics card in the green riser board: put its gold edge connector into the black slot and its steel bracket into the gap at the slot's near end, and press the card straight down until it is fully seated." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/seat-gpu-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=seat_gpu.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the graphics card in the green riser board: put its gold edge connector into the black slot and its steel bracket into the gap at the slot's near end, and press the card straight down until it is fully seated." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/seat-gpu-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/seat-gpu-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
Offsets are of the card from its seated place (the gold fingers on the slot floor, the board square over the slot).

| Condition | Number | Check on a real table |
|---|---|---|
| seated | the gold fingers' bottoms at both ends of the connector within 1 mm of the slot floor (so the card is pitched under about 0.7 degrees along its length); within 1.5 mm along the slot and 1.0 mm across; lean and turn under 2 degrees | the board edge down on the slot housing all along, no gold showing |
| started into the slot | over the slot (below), both ends of the connector under 8 mm above the floor, within 1.5 mm along and across | gold fingers inside the slot's mouth along their length |
| over the slot | turn under 3 degrees, tilt under 5, within 3 mm along and across, at most 40 mm above seated, and the card's bottom edge at the support channel below the channel's top | the far end inside the support channel, bracket toward the bases |

| Score | Condition |
|---|---|
| 1.0 | seated and still (under 1 cm/s and 0.5 rad/s) for 1 s (ends the trial) |
| 0.8 | seated at the last step, but not still |
| 0.5 | started into the slot but not seated |
| 0.2 | over the slot, far end in the support channel |
| 0 | anything else, including a card turned end for end |

The hooks don't see the grippers, so a card held seated counts the same as one let go.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream-1 gets 2,400 control steps (2 minutes at 20 Hz); the scripted demo reaches success at step 536 (26.8 s).
- `python -m pytest -q test_seat_gpu.py` checks the scoring and the demo's reach without the simulator.
- `make_meshes.py` writes the two USD files; they are committed, so nothing needs to be generated.

## Fidelity
The card and the riser are rigid bodies built from boxes and convex prisms at full size; the height, length and finger
sections follow published PCIe card dimensions (111.15 mm tall, 254 mm three-quarter length, x16 fingers 11.65 mm and
71.65 mm). The 35 mm thickness, the fingers' 8.5 mm height, the bracket and the riser's sizes are our estimates. The
connector has no contacts or springs, so the card goes in with no insertion force. The riser cage's latch, which the
source opens before and closes after this step, is left out: the seated card is held by gravity only. The slot's
lead-in mouth (5 mm) and the support channel's flare are wider than on real parts. The card is one rigid body with no
flex, and at 350 g it is likely lighter than a real dual-slot card. Every object has rubber-like friction (1.2 static,
1.0 dynamic), more than plastic and gold contacts have.
