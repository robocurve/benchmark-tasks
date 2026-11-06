# Bring a spring cord reel's work light home into its dock

A spring cord reel (red housing) lies on the left of the table. Its black 5 mm cord runs 34 cm straight to the right,
to a yellow work light (200 x 30 x 20 mm bar) parked against a small stop in front of the right arm. The reel always
pulls its cord in (about 1 N with the cord out, 0.5 N near home). The robot must bring the light home into a dock
(two low rails in front of the reel face) and **never let go of it while the reel can pull it**: a free light is
yanked across the table and slammed into the reel. A retractable cord reel owner's manual warns: "Do not allow the cord to
retract without restraining the recoil speed. Never let go of the cord when rewinding." (Owner's Manual & Safety
Instructions, models 72520/72522, https://images.homedepot-static.com/catalog/pdfImages/c7/c7f34c7f-cef5-4f7c-be8e-5bf360f815df.pdf). The tested ability is controlled retraction: yielding to a spring pull while
keeping hold, and passing the load from one hand to the other while it is under pull. The instruction states only
the goal; the never-let-go rule is tested and stated under Scoring.

Both arms are needed for any nonzero score (reach-forced). With a straight-down grip on the cord line, the right arm reaches the parked
light but no further left than about y -1 cm, where the light's cord end is still about 12.5 cm from the reel face (and even on a diagonal path it
cannot bring the cord end closer than about 4 cm when gripping 2.5 cm from the free end; gripping the very tip it
could just reach 2 cm, offline IK, but the light then lies 55 deg off the line, and without a hand-over that scores 0); the left
arm reaches the dock but not the parked light (its grasp points lie at y -7.5 cm and below, and the left arm reaches
no further right than about y +6 cm there). So the light has to change hands while the reel pulls: the left arm
closes on its cord end while the right arm still holds its free end (grasp points 15 cm apart), then the right arm
lets go.

## How this differs from similar board tasks

- "Release a loaded spring platform under control" (TK-ES7KZB): a vertical spring fixture; one arm supports the
  platform while a retaining bar is removed, and the platform's rise speed is capped (80 mm/s). Here nothing is
  unloaded or removed: a free-lying cord-tethered light is walked 34 cm along the table against a spring pull,
  the reach forces a hand-over between the two arms while under tension, and the check is "never unheld while the
  reel can pull" (a latched, near-binary rule) plus a seated dock, not a speed cap on a supported load.
- Spring-closed die (TK-W2SMC3), spring-shut breaker cover (TK-GAUQTC) and spring clamp (TK-27GV47) are about holding
  a spring part open or releasing it before another step; none carries a tethered object home along a line.
- Cord-linked devices (TK-J7Q44Q) and cord fishing (TK-9RHSRE) involve a passive cord; here the cord is actively
  retracted by the reel and the hand-over is the point.
- Residual overlap: both this task and TK-ES7KZB test controlled release of stored spring energy.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Let the cord reel wind its cord back in and put the yellow work light into the grey dock in front of the reel, then let go of the light."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live light pose. The right arm pinches the light 2.5 cm from its free end, lifts it over the
parking stop and walks it along the cord line toward the reel, the reel taking up the cord as it goes. At its reach
limit the left arm closes on the light 2.5 cm from its cord end (both wrist cameras look along the line toward the
reel), then the right arm opens and leaves. The left arm slides the light between the dock rails until its cord end
is about 3-6 mm from the reel face, lets go and rises; the reel draws it the last few millimetres. The recorded demo (`demo/`) reaches task_score 1.0 at step 389 (about 19 s).

Partial-credit variant `REEL_CHEAT=tier05`: the arms bring the light home as in the demo without a free moment. With
the cord wound in, the left arm re-grips the light at its middle, lifts it 1.5 cm, swings it about its cord end and
sets it down slanted, clear of the rails, then lets go. End state: 1.2 cm of cord out, light released and at rest
about 18 deg off the line with its centre 6 cm from it, not in the dock. Score exactly 0.5.

Shortcut variants (test only, `REEL_CHEAT=<name> python cabledemo.py task.py out`), all score exactly 0:

| variant | what it does | why 0 |
|---|---|---|
| `nothing` | waits | 34 cm of cord still out |
| `hold_still` | right arm lifts the light off the stop and holds it there | cord still out, light held |
| `letgo` | right arm lifts the light out and lets go | reel snaps it home unheld (free moment) |
| `onearm` | right arm carries it to its reach limit and lets go; the reel pulls it into the dock | free moment |
| `diagonal` | right arm alone takes its closest diagonal to the slot (cord end about 4 cm short) and lets go | over 2 cm of cord still out, or a free moment if the light snaps in; no hand-over either way |
| `drop` | both arms open during the hand-over | free moment |
| `hold` | whole task, but the left arm keeps holding the docked light | not released |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

Every tier is an end state, gated by one rule over the whole episode:

- **0** if at any control step while more than 2 cm of cord is out, the reel draws the cord in faster than 25 cm/s
  (the light got away, even if it slid through closed fingers), or the cord moves faster than 3 cm/s either way while no
  closed gripper holds the light (fingers closed and the gripper centre within 3 cm of the light's centre line). Latched over the whole
  episode (reset at each episode start). A guided light moves at arm speed (under 10 cm/s in the demo); a free one is
  pulled in at about 1 m/s.
- **0** if at the end any gripper centre is within 3 cm of the light or cord, or more than 2 cm of cord is still out
  of the reel.
- **0.5** at the end: both grippers held the light at the same moment during the run, cord wound in (at most 2 cm
  out), light released, but the light is not seated in the dock (it
  lies outside the rails, across a rail, jammed crooked against the reel face, or more than 10 deg off the line).
- **1.0** at the end: light in the dock between the rails, cord end within 1 cm of the reel face, axis within 10 deg
  of the dock line, released, cord at rest for 1 s.
- Real trial: judge holding and retraction from the video (cord speed at the slot from frame-to-frame marks on the
  cord); tape measure for the 1 cm and the dock.
- `progress()` reports retracted, handed_over, in_dock, seated, never_free, released; `distance()` is the cord end's
  gap to the reel face (3.0 after a free moment).

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes ~3–5 min wall time on one GPU including Isaac Sim start-up.
- Layout fixed with `object_jitter=0`; the cord is never jittered (it always starts straight from the reel slot).
