# Play Ode to Joy's opening on a touch-sensitive keyboard: a soft left-hand bass under a loud right-hand melody

Ode to Joy is the first two-hand piece in many beginner keyboard lessons. The Yamaha PSR-E373 has it as its lesson song 010, "Ode to Joy (Basic)", and
Yamaha's Song Book says of it: "In this version you play monophonic lines with both hands. No tricky techniques are required." The opening of that
arrangement is played here: the right hand E5 E5 F5 G5 | G5 F5 E5 D5 in quarter notes, the left hand a whole note C4 (middle C), then a whole note G3.

Nothing is read or chosen: the difficulty is in the presses.

- **Loud and soft at once.** A touch-sensitive keyboard sets each note's loudness from how fast its key goes down. The left hand presses its bass key
  slowly but all the way down (stopped short, a key does not sound) and holds it through the bar, while the right hand strikes its keys briskly.
- **Let up and strike again.** The repeated E and G are each let up and struck again: a key held through gives one note, not two.
- **Together, and in time.** Each bass note sounds with its melody note (within 80 ms), the melody at an even beat of about 80 per minute.
- **One key at a time.** A closed gripper's fingertips, 8 mm across, press a 23.5 mm white key without catching a neighbour or a black key.

**The arms.** In the demo the left arm plays the bass and the right arm the melody, as a pianist's hands do. The scoring asks for the result only: any
division of the work is allowed.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `mechkit.py` and `demokit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them.
- The key contacts are read after every physics step, by a wrapper the task file puts on the simulation's step (the harness steps physics 12 times per
  control step, and a brisk strike crosses both contacts within one control step).

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 1200 \
    --instruction "Play the opening of Ode to Joy, as in the keyboard's lesson, at about 80 beats per minute. Right hand, loud: E E F G, G F E D in quarter notes, letting each key up before the next, starting on the E a tenth above middle C. Left hand, soft: middle C with the first E, held through the first bar and let up before the G below it, then that G with the second bar's G, held through the second bar. Keep a steady beat, then let every key up and lift both hands clear of the keys."
```

The scene is the same for every seed.

## The demo

From this folder:

```bash
python demokit.py task.py OUT                                                     # the full-score run, with video
python demokit.py task.py OUT_checks --mode onespeed,melody,heldE,shallow,hold --fast   # the failure checks
python check_score.py                                                             # the scorer on hand-built key logs, with no simulator
```

The demo is a scripted reference, not a policy. It plays to its own clock at 80 per minute; it reads the keys' fixed places and its own fingertips, never the scorer.

1. Both hands close their fingers and come straight down over their first keys, the fingers closing across the key: the left just over C4, the right
   1.5 cm over E5.
2. On beat 1 the right hand strikes E5 (down in 0.1 s, up at once) while the left presses C4 slowly (0.45 s from touch to the bed) and holds it.
3. The right hand strikes E5, F5, G5 on the beat. After three beats the left lets C4 up, crosses over to G3 at the keys' height and presses it slowly, to
   sound with the right hand's G5 on beat 5; the right plays G5, F5, E5, D5.
4. Both hands let their keys up and lift 8 cm.

It scored 1.0 in GPU physics in 13.6 s of simulated time (272 steps): the melody's velocities 86 to 89 (its contacts about 21 ms apart), the bass
C4 43 and G3 41 (about 110 ms), each bass note 31 and 47 ms before its melody note, the beat 0.743 s (81 per minute), every gap within 6% of it. A CPU
run without cameras takes under a minute.

Failure checks (`--mode`, CPU physics):

| mode | what the script does | score |
|---|---|---|
| `onespeed` | plays every note, bass and melody, at the same brisk speed | 0.5 |
| `melody` | plays the melody alone, the left hand lifted away | 0.25 |
| `heldE` | holds the first E down through the second beat instead of striking it again | 0.0 (the next notes are wrong) |
| `shallow` | presses every key past its first contact but short of its second | 0.0 (nothing sounds) |
| `hold` | plays it all but leaves G3 down at the end | 0.0 |
| `none` | does nothing | 0.0 |

The demo film carries a velocity strip under the cameras (`velocity_strip.py`: each note's velocity, a bass note's from onset to release, a melody note's until the next melody note), since a film is silent. The
policy's camera images never carry it.

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold: `score()` is 1.0 only once the 1.0 state has
held 20 control steps (1 s) in a row (at most 0.95 until then), and `success()` is true from then on, so the two agree.

From the note log (key, onset, velocity, release). Each onset is the next expected note of the melody (E5 E5 F5 G5 G5 F5 E5 D5) or of the bass (C4,
then G3), or it is wrong: any other key, black keys included, or an extra press of an expected key.

- **0.25/8 for each melody note** sounded in order: 0.25 for all eight.
- With all eight, **0.125 for each bass note** that sounds within 80 ms of its melody note (C4 with note 1, G3 with note 5) and is held at least three
  beats (by the player's own mean beat), G3 only after C4 is let up: 0.5 with both.
- **0.75:** also both bass notes at least 25 velocity below the melody's mean (one press speed for every note earns 0.5 at most).
- **1.0:** also every gap between melody onsets within 10% of their mean, the mean between 60 and 100 per minute, each melody note let up before the
  next sounds; at the end every key up and every fingertip of both hands at least 5 cm above the white keys; all held 1 s.
- **Wrong notes:** each costs 0.1, up to two. A third scores 0, and so does any key still down.

Real trial: the keyboard's own MIDI output over its USB TO HOST port, logged on a computer (key, onset, velocity, release), scored by the same rules;
every key up at the end and the hands' clearance from a side camera with a mark 5 cm above the keys.

## Real trials: the build

- **Keyboard.** A Yamaha PSR-E373 (945 x 369 x 118 mm, 61 keys), Touch Response at 2 (Medium), its default, on the table with its front edge 25 cm in
  front of the arm bases and G4 on the arms' midline. Measure its keys' width, dip and top height and the press force at the front; set the model's
  numbers to them.
- **Logging.** A USB cable from its USB TO HOST port to a computer recording MIDI. The velocities the real keyboard reports follow its own curve, not the
  model's: check before trials that a person's slow full press and brisk strike, as in the demo, differ by at least 25 velocity at Touch Response 2; the margin
  stays 25.
- **Safety.** The keyboard on a non-slip mat; the e-stop in reach.

## The model behind it

The keyboard's case is a fixed body. All 61 keys move, in two articulations (C2 to E4 and F4 to C7, split where no black key sits between): each a link
on a hinge at its back, with a return spring set past its rest so that it rests on its stop with 0.5 N at its front and takes 0.8 N at the bed, and a
10 mm dip at its front (a black key's front, 95 mm from its hinge, dips the same). Two virtual contacts under each key close at 3 and 8 mm of dip at its
front. They are read after every physics step (240 Hz), each crossing time interpolated between steps: the note sounds when the second closes, with a
velocity from the time between the two (the key's speed between them, on a log scale: 0.01 m/s gives 1, 1 m/s gives 127), and ends when the key rises
past the first. No audio is simulated; the real keyboard sounds every note. Key sizes, the keybed's height and the spring are approximated.

## Attribution

Ode to Joy is by Ludwig van Beethoven (public domain). The arrangement played is the opening of Yamaha's lesson song "Ode to Joy (Basic)". Sources quoted:

- the keyboard's figures and settings: Yamaha PSR-E373 Owner's Manual (https://ca.yamaha.com/files/download/other_assets/3/1334923/psre373_en_om_c0.pdf);
- the arrangement: Yamaha Song Book for PSR-E373/PSR-EW310/YPT-370
  (https://usa.yamaha.com/files/download/other_assets/0/1354040/psr_e373_en_songbook_a1.pdf), p. 54.

Yamaha and PSR are trademarks of Yamaha Corporation, named for identification only.

## Assets

Every mesh is generated by `task.py` and `mechkit.py`. There are no textures and no third-party assets.

## Licence

MIT
