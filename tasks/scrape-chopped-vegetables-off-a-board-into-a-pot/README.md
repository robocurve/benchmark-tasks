# Hold a cutting board up at a pot's rim with one hand while the other scrapes the chopped vegetables off it into the pot

Twelve diced vegetables lie on a wooden cutting board on the counter, next to a pot. One hand takes the board by the
grip bar across its end, lifts it and sets its other end on the pot's rim, reaching over the pot, and holds it there.
The other hand takes a bench scraper from its rack, sets its blade on the board behind the pieces and pushes them along
the board and off its end into the pot. The task is one task file,
[`scrape_chopped_vegetables_off_a_board_into_a_pot.py`](scrape_chopped_vegetables_off_a_board_into_a_pot.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

Why two hands: the wet pieces stick to the board, so tipping the board over the pot does not shed them; they have to be
pushed off its end. And the pot's rim is 6 cm above the top of the board lying on the counter: scraped where it lies,
the pieces go over the board's end onto the counter (none in the pot: 0). The board has to be held up with its end over
the pot while the other hand scrapes.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The board (with its grip bar), the pot, the scraper
and its rack are rigid bodies made of several shapes from `bodykit.py`; the pot and the rack are kinematic, fixed where
they are put.

- The vegetables are twelve rigid 1.2 cm cubes of 2 g. After every control step the task file looks at each piece:
  while it lies on the board's face (its centre over the board, within 1.1 cm of the face), it is pulled onto the face
  with three times its weight until the next step. With friction, that holds a piece on the board even with the board
  on edge or upside down, and lets go of it once it is pushed past the board's end.
- The scraper's blade is a 2 mm steel plate under a wooden bar; it stands in a slot in its rack.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with every piece still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=scrape_chopped_vegetables_off_a_board_into_a_pot.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the cutting board up by its grip bar with its end over the pot, and with the other hand scrape the chopped vegetables off the board into the pot with the bench scraper." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/scrape-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the grip bar, lifts the board, carries
it right and sets its end on the pot's rim, 4.5 cm over the pot, and holds it. The right hand comes straight down on the
scraper's bar, lifts it out of the rack, sets the blade on the board just right of the left hand and pushes it along the
board and past its end, sweeping the pieces into the pot (and goes round once more if any are left):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=scrape_chopped_vegetables_off_a_board_into_a_pot.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the cutting board up by its grip bar with its end over the pot, and with the other hand scrape the chopped vegetables off the board into the pot with the bench scraper." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/scrape-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/scrape-dream/*.json; do inspect-robots video "$log"; done
```

Set `SCRAPE_DEBUG=1` to print what the score is built from (where the board is and how tipped, how many pieces are held
on it, in the pot and spilt, where the scraper is on the board, and where the left gripper's TCP is on the grip bar)
twice a second.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (213 s on the clock, with Dream answering in 0.80 s on average), and about 1 minute for the scripted demo (322 steps, 16.1 s of simulated time, 66 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
