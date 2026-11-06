# Pay a multicore off a free-turning drum to its cradle without leaving the drum running unheld

A cable drum (grey 5 cm hub with a 12 cm top flange, 0.7 kg in all, and a white bar on top that shows it turning)
stands on a vertical axle at the left of the table. A red 6 mm multicore is wound about 2.5 turns round it, its inner
end fixed to the hub; the tail and a 35 mm connector lie out toward the right. The robot must pay the cable off along
a yellow-taped run and rest the connector in a white cradle at the far right, keeping the drum under one turn per
second and **never letting go of the cable while the drum turns**.

The rules protect the coil. The drum has inertia and no brake: if the pull stops dead or the cable is let go, the
drum runs on for about a second and throws its turns loose. Paced payout keeps the coil tight (about 1 cm standoff in
the demo); stopping dead from a brisk pull loosens it past the 2 cm limit. The guidance comes from steel wire rope handling and a rope-drum patent, applied by analogy
to a multicore (no stage-cable source was found, and the robot has no brake, so pacing the pull replaces braking): "Mount reels or spools onto a shaft so that the reel will revolve when the rope is pulled off.
Take care to brake the spool to keep tension on the rope as it is spooled on or off the drum." (NSW Government,
Dogging and rigging guide, Handling wire ropes,
https://www.nsw.gov.au/employment/dogging-and-rigging/guide/part-1-general-rigging-principles/lifting-equipment/steel-wire-ropes/handling-wire-ropes).
On slack coils: "This causes damage to the rope and in certain cases, the rope may have to be cut off the drum and
discarded." (US patent 4895348, https://patents.google.com/patent/US4895348). The occupation is sound engineering
technicians, who "Set up, test, and adjust recording equipment for recording sessions and live performances." (O*NET
27-4014.00, https://www.onetonline.org/link/summary/27-4014.00).

The tested ability is steady, rate-limited payout from a free drum through one hand-over: the hand that lets go must
first slow the pull and wait for the drum to stop.

Both arms are needed (reach-forced, offline IK over 13 grip yaws): the connector starts at x 32.5, y +14.5 cm, and the
right arm's straight-down grip ends near y +9 cm at that depth, so it cannot take the connector or the cable behind
it. The cradle is at x 41.4, y -23.2 cm, and the left arm has no solution there or at x 40, y -10 cm. So the left arm
pays the cable out to the middle of the table, slows, waits for the drum to stop and sets the connector down; the
right arm takes it from there to the cradle.

## How this differs from similar tasks

Other tasks wind a cord onto spools or posts, unwind turns from a fixed post, let a spring reel retract a cord
under hand control, or pull a cable to a length mark from a channel. Here the store is a free-turning drum that
the pull itself drives: the ability tested is pacing that pull (a drum-speed limit) and never leaving the drum
turning unheld at the hand-over. Nothing pulls the cable back, and the cable is not wound or unwound round a fixed
part.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (cable simulation, pre-wound start, scripted
  demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \
    --instruction "Pay the red cable off the drum along the yellow-taped run and rest its connector in the white cradle, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 900
```

Closed loop on the live connector pose. The left arm pinches the connector across its body (wrist camera looking
along the run toward the cradle) and pays the cable out along the run to the middle of the table over 3.5 s
(peak drum speed about 140 deg/s; about 190 deg/s on the right arm's carry), lowers it, waits until the drum is still, lets go and leaves. The right arm takes
the connector there, carries it 15 mm above the table to the cradle over 3.5 s (wrist camera toward the cradle),
lowers it into the seat, waits for the drum to stop, lets go and rises. About 407 steps (20 s).

## Scoring

`score()` latches three violations each control step: drum faster than 360 deg/s (one turn per second); drum faster
than 20 deg/s while no closed gripper is within 3 cm of the cable or connector; a wound turn more than 2 cm off the
wound radius, with cable further along still on the hub. At the end every gripper must be at least 3 cm from the cable
and connector and the connector centre inside the cradle seat, lying on the table. Then 0.5: connector seated, but the
cable leaves the tape lines or the drum still turns. 1.0: connector seated, every cable point more than 6 cm from the
axle within 4 cm of the run centreline and not past the cradle's far wall, drum under 3 deg/s; `success()` ends the trial after 1 s of this.

Variants (env `DRUM_CHEAT`): `nothing`, `hold`, `yank`, `letgo`, `overrun`, `onearm` must score 0; `tier05` (seat the
connector, then drag the middle of the run out over the tape) scores 0.5.

Runtime: about 45 s of sim time per trial (900 steps at 20 Hz). Seeds: the layout is fixed with
`object_jitter=0`; the cable is not jittered (its wound start is written at reset and settled, deterministic).
