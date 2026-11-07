# Plug each dual-corded server into both power strips

Three servers each have two power cords, shown here as two plug heads of the same colour. One cord of each server goes
into the left power strip (PDU A, one feed) and the other into the right strip (PDU B, the other feed), so that either
feed can be switched off without taking a server down, and each strip carries three plugs. The left strip is within
reach of the left arm only and the right strip of the right arm only, so splitting every pair between the strips means
splitting it between the arms. The plugs wait on a tray between the arms where each arm's own side holds the wrong
mix (red, red, green on the left; green, blue, blue on the right): each arm has to reach across once.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=split_dual_power_across_pdus.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each pair of same-coloured plugs is one server's two power cords: plug one of each pair into the left power strip and the other into the right power strip, one plug per socket." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/split-power-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=split_dual_power_across_pdus.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each pair of same-coloured plugs is one server's two power cords: plug one of each pair into the left power strip and the other into the right power strip, one plug per socket." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/split-power-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/split-power-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends early at 1.0 once every strip holds one red, one green and one
blue plug, all six seated and still (under 1 cm/s and 0.5 rad/s), for 2.5 s; otherwise the partial credit below is
taken at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_split_dual_power_across_pdus.py` in an
environment with `yam_arena` installed.

## Scoring

A plug is **seated** when its centre is within 5 mm of a socket's centre front to back and side to side, its centre is
within 5 mm of the height it has standing on the socket floor (2.25 cm above the floor), and its long axis is within
10 degrees of vertical (either end up: the plug head is a plain box in one colour, so a plug stood on its other end
looks and fits the same). (The plug is 25 mm across in a 32 mm socket, so the 5 mm across holds whenever it is down in the socket.) The
scorer judges height by the plug's centre, not its bounding box, because the simulator pads a loose object's bounding
box by 5 mm.

| Credit | For |
|---|---|
| 0.3 | each colour (server) with both plugs seated, one in each strip |
| 0.1 | instead of 0.3: a colour with exactly one plug seated |
| 0.1 | all six sockets holding a plug (three per strip) |
| 0 overall | both plugs of any colour seated in the same strip |

Every outcome is covered: a plug that is not seated (on the tray, on the table or the floor, lying across a socket,
standing on another plug or held above a socket) earns nothing. Because of the zero rule, a layout with all six
sockets filled scores either 1.0 (every strip one red, one green, one blue) or 0 (some colour twice on one strip), so
the last 0.1 is only ever earned together with the three 0.3 terms. Taking each arm's own column alone (both reds on
PDU A, both blues on PDU B) scores 0.

On a real table, a ruler checks that each plug's top stands 2.0 cm above the strip when fully down (2.5 cm socket,
4.5 cm plug: at most 2.5 cm means its centre is within 5 mm of the seated height), and a photo from above shows which colour is in which strip.

## The scripted demo

The arms take turns at the tray, the right starting 6.65 s after the left (half of the 13.3 s cycle). Each grips a plug straight down across its
width with the fingers opened only 44 mm (so the open fingertips clear the other column), lifts it until its bottom is
10 cm above the table (the gripper point 14 cm), 4.5 cm clear of the standing plugs, carries it to a socket in its own strip and lets go 6 mm above the socket floor, then lifts clear and waits
2 s above the strip while the other arm is at the tray. The left arm plugs red (nearest socket), blue and green into
PDU A, taking the blue from the right column; the right arm plugs red, green and blue into PDU B, taking the red from
the left column. Both then park in front of their own bases so the camera sees the strips.

## Runtime and seeds

- The demo's plan takes about 47.1 s of simulated time; the last plug is let go at about 43.8 s, and with the 2.5 s
  hold the trial ends in success at about 46.2 s (about 923 steps at 20 Hz), well within the 1,800-step limit. A full Dream-1
  trial runs at most 1,800 steps (90 s of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
