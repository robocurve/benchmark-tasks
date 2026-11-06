# Mic the three singers by the 3:1 rule

Three singers stand in a row 12 cm apart, and each gets a mic stand of their colour. Each stand must be within 6 cm
of its own singer, and every two stands at least 3 times as far apart as the larger of their two mic-to-singer
distances. This is the 3:1 rule sound technicians use against comb filtering. The required gap changes with where
the stands go. Setting each stand straight in front of its singer, 5 cm away, leaves neighbours 12 cm apart against
the 15 cm needed.

The scene is a wooden table, its top 1.10 m front to back and 1.30 m side to side, 74.75 cm above the floor. The two
arms stand on it facing forward, 19 cm in from its back edge, their bases 61 cm apart (30.5 cm either side of the
centre line); "in front of the arm bases" is measured forward from the line through the two bases, and left and right
are from the robot's view. The singer markers stand 28 cm in front of the arm bases, 12 cm apart. The stands start in
a case 20 cm in front of the arm bases and 33 cm left of the centre line, 2.5 cm further left than the left arm's
base; the right arm cannot reach it. The left arm can set the blue stand itself only on the near side of the blue
singer with its gripper low: the simulator's IK reaches 23 cm in front of the arm bases, 12 cm right of the centre
line, with the gripper 7 cm above the table, but not 15 cm up. With green behind its own singer, that layout scores
1.0 (a named test case). Handing the blue stand to the right arm by the orange pad on the centre line is another
route, and the one the demo takes. The full setup, every object's size, material and mass, is in `task.json`.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=mic_three_singers_three_to_one.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the red, green and blue mic stands out of the grey case and stand each one upright on the table within 6 cm of the singer marker of the same colour, with every two mic stands at least 3 times as far apart as the larger of their two distances to their own singers." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/mic-singers-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mic_three_singers_three_to_one.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the red, green and blue mic stands out of the grey case and stand each one upright on the table within 6 cm of the singer marker of the same colour, with every two mic stands at least 3 times as far apart as the larger of their two distances to their own singers." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/mic-singers-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/mic-singers-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all three stands are placed, every pair keeps the
3:1 rule and the stands have been still for 2.5 s. Otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_mic_three_singers_three_to_one.py` in an
environment with `yam_arena` installed.

## Scoring

All distances are between centres, seen from above. d is a stand's distance to the singer marker of its own colour.
A stand is stowed when its centre is over the inside of the case or over the orange pad, and carried when its bottom
is more than 8.2 cm above the table top (higher than the 8 cm top of a standing stand, so a stand stacked on a
standing one is not carried). Every other stand is a live mic, placed or not, standing, stacked, lying or held low.

| Credit | For |
|---|---|
| 0.1 | each placed mic: upright (axis at most 10 degrees from vertical), standing on the table (bottom within 3 mm of the table top), d at most 6 cm, and nearer its own marker than either other one |
| 0.3 | red-green, both placed |
| 0.3 | green-blue, both placed |
| 0.1 | red-blue, both placed |
| 0 | a stowed stand (centre over the case's inside or the pad) or a carried one (bottom more than 8.2 cm above the table top): it earns nothing and zeroes nothing |
| 0 overall | the 3:1 rule broken: any live stand's centre closer to a placed mic than 3 × that placed mic's d (for two placed mics: closer together than 3 × the larger of their two d) |
| 0 overall | any stand fallen off the table (bottom more than 3 mm below the table top) |
| 0 overall | any stand, including one still held, tipped more than 10 degrees with its lowest point less than 5 cm above the table top (knocked over, lying or leaning) |

A stand that is not placed earns nothing, but unless it is stowed or carried it is still a live mic and must keep the
3:1 rule with every placed mic: a stand dropped beside a placed mic, stacked on one, or set just past its own 6 cm in
the straight-in-front row zeroes the take, as it would if placed. Stands left in the case or on the pad, or lifted
with their bottom more than 8.2 cm up, score nothing and zero nothing, so a trial that times out mid-carry keeps the
credit already earned: the demo carries each stand with its bottom 8.5 cm up. A stand tilted in a gripper with its
lowest point 5 cm or more above the table is not knocked over; whether it must keep the 3:1 rule depends only on
the stowed and carried rules. Any layout that meets these conditions scores 1.0, not only the demo's. A stand
cannot be closer than 4.5 cm to its own singer (3 cm marker radius plus 1.5 cm stand radius), so every pair needs at
least 13.5 cm, more than the 12 cm between neighbouring singers.

On a real table, a photo from straight above with a ruler in frame gives every centre: measure each stand to the
three markers, then each pair of stands, and compare each pair's gap with 3 × the larger of its two d. A phone level
app checks a stand's tilt.

## The scripted demo

The left arm takes the red stand from the case and sets it 5 cm left of the red singer, then the green stand 5 cm in
front of the green singer, then sets the blue stand on the orange pad and parks above the empty case. The right arm
waits in front of its own base until then, picks the blue stand off the pad and sets it 5 cm right of the blue
singer, then parks back where it waited. Each stand is gripped from above 6.5 cm up from its bottom and let go 7 to 8 mm
above the table. Red-green and green-blue end √(5² + 17²) = 17.7 cm apart against 3 × 5 = 15 cm; red-blue 34 cm.

## Runtime and seeds

- The demo's plan takes 42.9 s of simulated time (the left arm 30.3 s, the right arm 42.9 s, ending with its park);
  replayed, the 2.5 s hold completes at 42.5 s, about 850 steps at 20 Hz, under half the 1,800-step limit. On an L40S, once Isaac Sim has
  started, a scripted trial takes about 1 minute of wall time and a full Dream-1 trial 3 to 4 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
