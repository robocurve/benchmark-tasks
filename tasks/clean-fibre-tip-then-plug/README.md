# Clean a fibre connector's end face, then plug it in

A loose yellow fibre patch cord lies straight across the table, its blue SC/UPC connector pointing right. On the
right stand an upright fibre cleaning card (a white lint-free pad on a grey holder) and a grey fibre adapter whose
socket faces left, both with their front faces in the same plane. Dirty end faces are the most common cause of
fibre link failures, so field practice is "clean, then connect": wipe the connector's end face across a clean pad
in one stroke right before mating it, and don't let the cleaned face touch anything on the way to the port. The
robot must wipe the blue connector's front end face along the pad in one continuous stroke of at least 4 cm, carry
it to the adapter without that face touching the table, the holder or the adapter's rim, push it at least 15 mm
into the socket and let go. Plugging it in without cleaning it first scores 0, however well it is seated: that is
the error the task targets. The left arm holds the cord up 25 cm behind the connector so the trailing cord does not
drag across the pad and socket or tug the connector round in the right gripper.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the cable simulation) and `cabledemo.py` (the scripted-demo runner) are in this folder. The cord is
  a free (unanchored) kit cable with a connector on each end. `task.py` adds a per-physics-step tracker of the
  blue connector's end face (no kit changes).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Before connecting the yellow fibre, clean its blue connector: wipe the connector's front end face across the white cleaning pad in one stroke of at least 4 cm, keep that end face from touching anything else, then push the connector fully into the grey adapter socket and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live connector pose. The right arm pinches the blue connector body 10 mm from its cord end and
sets it down 9 cm further forward, then picks it up again with the wrist turned 180 degrees (still pointing right):
the first grasp is the only one the wrist can reach where the connector starts, the second puts the right wrist
camera on the side the connector points to, so it watches the pad during the wipe and the adapter during insertion
(the connector is not wiped yet, so setting it down costs nothing). The left arm then lifts the cord 25 cm behind
the connector and holds it beside the work area. The right arm brings the end face 15 mm in front of the pad, then presses it flat on the pad (1 mm) and wipes
7.5 cm forward in one stroke, backs off 2 cm, lines the connector up with the socket (servoing on the measured end
face, since the arm sags a few mm under load) and pushes it 22 mm in. Both arms let go and move away.
task_score 1.0; success holds for 1 s and ends the episode at step 723 (~36 s). `demo/` holds the videos,
`setup.png` and `result.json`.

Shortcut variants, selected with the test-only env var `FIBRE_DEMO=<mode>`:

| `FIBRE_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 |
| `direct` | picks the connector up and seats it 21 mm deep without wiping, lets go | 0 |
| `drag` | valid 7.5 cm wipe, then rests the connector on the table and slides it 3 cm (end face drags on the table), then seats it and lets go | 0 |
| `hover` | the same 7.5 cm stroke but with the end face 4 mm off the pad, then seats it | 0 |
| `short` | end face on the pad, a 2.5 cm stroke only, then seats it | 0 |
| `hold` | everything right (wipe, seat 21 mm) but keeps holding the connector | 0 |

`test_scoring.py` (run with the yam_arena environment: `python test_scoring.py`) feeds the end-face tracker synthetic
connector paths: a valid wipe then insertion; inserting directly; a 3.5 cm stroke; a stroke 4 mm off the pad;
wiping with the connector's side flat on the pad; wiping with its edge (axis tilted 35°); a valid wipe followed by a
table drag; a valid wipe followed by a bump on the adapter rim; and a re-wipe after a table drag (clean again).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

The end face is the connector's front face (14 x 12 mm); the connector axis points from the last cord segment to
the connector centre. A tracker checks the end face after every physics step (240 Hz):

- **Valid wipe**: the end-face centre is within 1.5 mm of the pad surface (or pressed up to 3 mm into it), the whole
  face lies on the pad, the connector axis is within 20° of the pad normal (face flat on the pad, not its side or an
  edge), and the face-centre travels at least 4 cm (straight-line) in one continuous contact (gaps < 0.1 s allowed).
- **Touched**: any point of the end face within 1 mm of the table, the pad holder or the adapter (outside the
  socket opening) clears a valid wipe; the connector must be wiped again.
- **Dirty insertion** (latched): the end face more than 2 mm inside the socket while no valid wipe stands.

Score 0 if: a dirty insertion happened; no valid wipe stands at the end (none, or the face touched something
after it); a gripper TCP is within 4 cm of the blue connector's centre at the end (still holding it). Otherwise 1.0
if the end face is at least 15 mm inside the socket and lined up with it (centre within 6 mm of the socket axis,
axis within 26° of it); else 0.5 (wiped, clean, not seated).

Runtime per trial: 1,200 steps at 20 Hz (60 s of robot time); the demo ends at step 723. Seeds: the layout is
fixed (`object_jitter=0`); the cord is not jittered.
