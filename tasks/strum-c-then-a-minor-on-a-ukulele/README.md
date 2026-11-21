# Strum C then A minor on a ukulele, all strings ringing

**Instruction:** "Play C on the ukulele: four even downstrums, then pluck its fretted string once to check it; then four
of A minor and check its G and A strings the same way, and keep holding the A minor."

A concert ukulele lies face up in a padded cradle across the table, neck to the robot's left. C (third fret of the A
string, the others open) and A minor (second fret of the G string, the others open) are the first two chords every
beginner learns. Each frets only an outer string, so one fingertip is enough: it must press the string just behind its
fret wire hard enough to reach the fret, while overhanging the fingerboard's edge so it does not touch the open string
10 mm beside it, and it must stay put while the other hand strums across all four strings. Teachers have players fret
a chord and pluck the strings one at a time to check none is muted; here each chord's fretted string is plucked alone
after its strums (and the open A string too in A minor, since the fretted G and the open A both sound A4).

Sources: Aquila 103U Super Nylgut concert set tensions G 4.37, C 3.91, E 4.41, A 4.72 kg (https://beckettsmusic.co.uk/products/103u);
nylon-string pluck displacements of 0 to 4 mm (Watson, "Transverse and longitudinal forces of a string on the guitar bridge",
https://huskiecommons.lib.niu.edu/allgraduate-thesesdissertations/6519/). Kala KA-C product page (https://kalabrand.com/products/ka-c): "Scale Length: 14.8125 inches", "Fingerboard
Width at Nut: 1.4375 inches", "Overall Instrument Length: 23.9375 inches", Aquila Super Nylgut strings. Chord shapes C =
0003 and A minor = 2000 (https://ukuleletricks.com/ukulele-chords/c/major, https://ukutabs.com/ukulele-chords-diagrams/c/).
Checking a chord string by string (https://support.yousician.com/hc/en-us/articles/115005290605-Playing-ukulele-chords).

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key configured, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=strum_c_then_a_minor_on_a_ukulele.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Play C on the ukulele: four even downstrums, then pluck its fretted string once to check it; then four of A minor and check its G and A strings the same way, and keep holding the A minor." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/ukulele-dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strum_c_then_a_minor_on_a_ukulele.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Play C on the ukulele: four even downstrums, then pluck its fretted string once to check it; then four of A minor and check its G and A strings the same way, and keep holding the A minor." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/ukulele-demo
```

- One fixed layout (`object_jitter=0`); the seed changes nothing. Runtime per trial: up to 2,400 steps (120 s of sim
  time at 20 Hz); the scripted demo is about 27 s (about 530 steps).
- Demo modes (`KIT_DEMO`): `demo`, `slam`, `wrong_order`, `no_check`, `stay_c`, `five`, `none` (the table below).

## Runs

Isaac Sim (`object_jitter=0`):

| Run | task_score | Notes |
|---|---|---|
| Scripted demo (`yam_scripted`) | 1.0 | 35 strikes (32 strum, 3 check), C held through its check ring, chord change 2.35 s, ends at 27 s |
| Dream-1 (`dream`) | 0.0 | 2,400 steps, no string struck |

Offline MuJoCo check of every demo mode:

| Mode | What it does | Expected at most | MuJoCo check |
|---|---|---|---|
| demo | C, check, A minor, two checks, hold | 1.0 | 1.0 |
| slam | flat pad across A and E | 0 | 0 |
| wrong_order | strums, nothing fretted | 0 | 0 |
| no_check | C strummed, no check pluck | 0.25 | 0.25 |
| stay_c | never changes to A minor | 0.5 | 0.5 |
| five | five C strums | 0.75 | 0.25 |
| none | arms stay home | 0 | 0 |

## Scoring (the scorer is `grade()` and `analyse()` in the task file)

| Score | Condition |
|---|---|
| 0 | the ukulele moved more than 5 mm in its cradle (permanent), or nothing below |
| 0.25 | four downstrums with C fretted and its open strings (G, C, E) ringing (real trial: all four blurred) |
| 0.5 | then the C check pluck of the A string, C kept fretted while it rings, the fretting tip within 2 mm of where it was at the first strum |
| 0.75 | then four A minor downstrums with C, E and A ringing (real trial: all four blurred), the G check pluck and the open A check pluck, A minor kept fretted while each rings, tip unmoved |
| 1.0 | exactly four strums per block within 25% of the block's mean interval, at most 2.5 s from the end of the C check's 0.5 s ring to the first A minor strum, and for the last 2 s A minor still fretted, the tip within 2 mm and the strumming gripper at least 20 mm from every string |

Rules (geometric on the grippers' collision hulls; no sound is simulated): fretting, strikes (a 1.5 mm sideways carry
against the string's 550-660 N/m spring, then the slip), ringing, downstrums and check plucks are defined in
`task.json` (scoring and fidelity) and in the task file's docstring.

Real trial, string by string, so no check relies on pitch (in A minor the fretted G and the open A both sound A4,
and the open C string's second harmonic sits at C5):
- **Each strum:** a phone on a stand films the strum zone from 30 cm above at 240 fps (any phone's slow-motion mode)
  beside a mm scale laid across the body, shutter no faster than 1/250 s so a vibrating string blurs (if the
  slow-motion mode picks a faster shutter, dim the room or use a manual-shutter camera app; before the trial, check
  that a plucked string reads as blurred). A ringing string shows a blurred band at least 2 mm across (the band's
  full width, string included, against the mm scale; a string at rest is under 1 mm), read anywhere along its 80 mm (the
  strumming hand may hide part of it), in every frame from 0.1 to 0.5 s after the strum; a muted or unstruck string
  stays a sharp line. All four strings must blur in each strum, each read on its own line: C = G, C, E and the
  fretted A; A minor = the fretted G, C, E and A. So the fretted G and the open A of A minor are seen separately.
- **Each fretted string** is also checked alone, by its check pluck after the strums: blurred the same way, and on the
  clip-on tuner within 20 cents (C: the A string at C5; A minor: the G string at A4, then the open A string at A4).
- **The fretting tip and the timing:** a close-up video of the fretting hand beside a mm scale (the 2 mm tip check),
  with the strum and pluck times read from the video timestamps.

## Cradle and safety

The cradle is two foam pads screwed to the table: a body rest 200 mm across and 150 mm along the strings, 12 mm thick,
under the lower bout, and a neck rest 60 mm wide under the neck from 100 to 160 mm from the nut, with a hook-and-loop
strap over the lower bout.

**Real trials are safe.** A nylon-strung ukulele on a padded cradle: nothing heated, powered, sharp or heavy; the
string forces are about 1-2 N. The person resetting retunes with the clip-on tuner and stays out of the arms' reach
while a trial runs.
