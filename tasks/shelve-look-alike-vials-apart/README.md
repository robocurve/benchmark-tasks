# Shelve look-alike drug cartons apart, names facing the front

Four drug cartons go into a row of six shelf pockets, one per pocket, each standing upright with a printed wide face
toward the front of the shelf. Two of them, the red and the blue carton, are look-alike drugs and must not end up in
the same or neighbouring pockets. Both look-alikes start in front of the left arm, nearest pockets 1 to 3; shelving
them in the two nearest free pockets puts them side by side. Each carton also starts turned with its wide faces to
the sides, so it has to be turned a quarter turn on the way. The left arm reaches pockets 1 to 4 and the right arm
pockets 3 to 6, so the arms have to share the row.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shelve_look_alike_vials_apart.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the four drug cartons upright in the shelf pockets, one per pocket, with a wide face toward the black front lip. The red and blue cartons are look-alike drugs: never put them in the same or neighbouring pockets." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/shelve-vials-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shelve_look_alike_vials_apart.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the four drug cartons upright in the shelf pockets, one per pocket, with a wide face toward the black front lip. The red and blue cartons are look-alike drugs: never put them in the same or neighbouring pockets." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/shelve-vials-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/shelve-vials-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all four cartons are shelved facing the front,
with the red and blue cartons apart, for 3 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_shelve_look_alike_vials_apart.py` in an
environment with `yam_arena` installed.

## Scoring

The pockets are numbered 1 to 6 from the robot's left; each is 46 x 46 mm inside, and each owns a 52 mm band of the
rack, left to right, from the middle of one divider to the middle of the next, across the rack's full 58 mm depth
front to back.

| Credit | For |
|---|---|
| 0.15 | each carton shelved: its centre inside a pocket's 46 x 46 mm inside, tilted at most 10 degrees, its bottom within 5 mm of the pocket floor, and no other shelved carton in the same pocket |
| 0.10 | more for each shelved carton with a wide face pointing to the front lip within 20 degrees (either wide face; both carry the name) |
| 0 overall | the red and blue cartons' centres over the same or neighbouring pockets' bands, each of the two either standing on another carton (its bottom within 10 mm of that carton's top, their outlines overlapping seen from above), however high the stack, or with its bottom less than 70 mm above the pocket floor (in a pocket, on the lips or on a shelved carton), at any tilt |

All four shelved and facing the front, the look-alikes apart: 1.0. Only a carton in the air over the rack, its
bottom 70 mm or more above the pocket floor with no carton under it (as when carried in the gripper, clear of the
shelved cartons' 60 mm tops), does not count for the look-alike rule; it earns nothing itself until it is shelved.
A look-alike left on a stack of cartons next to its pair counts, however high it stands. A carton lying on its side, standing on the lips
or on another carton, left on the table or off it earns nothing. On a real table, a photo from above shows which
pocket each carton is in and, with a protractor laid along the front lip, how far each is turned; a phone level app
on a carton's side checks its tilt.

## The scripted demo

Each pick grips the carton from above across its 24 mm depth, 40 mm above its bottom, and turns the grip a quarter
turn while carrying it, so the carton arrives with its wide faces toward the front lip and the fingers closing front
to back, clear of the neighbouring pockets. The carry stops 3 cm short of the pocket, and the arm creeps the last
3 cm slowly and settles, so it arrives without overshoot (the carton has 3 mm to spare side to side). It lowers the
carton until it is 6 mm above the divider tops, settles again, slides it in slowly, holds still, lets go with the
carton's bottom 9 mm above the pocket floor, and the fingers stay open a moment before the arm lifts. (The code
sends the tool point 8 mm above where it would be with the carton on the floor. The carton hangs 1 mm lower in the
grip, because it starts 1 mm above the table and the grip closes before it settles. The replay measures 9 mm.) Held at
that height the carton is outside the 5 mm floor tolerance, so the success hold cannot start until it is let go.
The left arm shelves the red carton in pocket 1 and the blue carton in pocket 3, then parks over the red carton's
starting mark. At the same time the right arm shelves the yellow carton in pocket 6, then waits above the green
carton until the left arm has parked, shelves the green carton in pocket 5, lifts, and parks over the green
carton's starting mark 2.5 s after letting go. The 3 s success hold starts when the green carton lands, so both arms
are parked before the trial ends, and the last frame shows the filled row: red, empty, blue, empty, green, yellow.

## Runtime and seeds

- The demo's plan takes 46.2 s. The last carton is let go at 43.7 s and the right arm is parked at 46.2 s. With
  the 3 s success hold and a 0.6 s settling allowance, the trial ends by 47.3 s, about 946 steps at 20 Hz, 53% of
  the 1,800-step limit (90 seconds). The scripted policy stops at 49.2 s.
- Physics run of this demo on the official simulator (the run behind the demo video): the trial ended on success
  at step 927 (46.35 s at 20 Hz, 52% of the 1,800-step limit) with `task_score` 1.0.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
