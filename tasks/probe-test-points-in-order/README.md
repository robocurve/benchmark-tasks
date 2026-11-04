# Probe three test points in order with a tethered probe

An electronics test bench. A yellow bench meter stands at the far left of the table; its thin red test lead (5 mm,
50 cm) leaves the meter's right side and lies straight along the far edge of the work area to a red probe, whose tip
rests in a black probe holder (an open-top slot) at the far right. In front of the lead is a fixed green circuit board
with three gold test pads marked 1, 2 and 3 in white digits. The pads lie on a diagonal with pad 1 in the middle
(pad 2 near-left, pad 3 far-right), so every simple spatial order (left to right, near to far, reading order,
nearest to the probe first) gives the wrong sequence: the robot has to read the numbers. Two small blue capacitors and
a black relay (a few grams each, loose) stand on the board between the pads and the lead, where a lead dragged over
the board sweeps them away. The robot must touch the probe tip on pads 1, 2 and 3 in that order, at least 1 s each,
keep the lead lifted off the board, then put the probe back in its holder and let go. When probing,
technicians keep the lead clear of the board so it does not knock parts out of place.

Instruction: `Probe the gold test pads in numbered order: touch the red probe's tip on pad 1, then 2, then 3, holding it still on each for at least one second. Keep the red lead off the board so no component is knocked over. Then put the probe back in its black holder and let go.`

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated lead) and `boxfont.py` (digits built from thin boxes) are in this folder next to
  `task.py`; `cabledemo.py` runs the scripted demo.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "Probe the gold test pads in numbered order: touch the red probe's tip on pad 1, then 2, then 3, holding it still on each for at least one second. Keep the red lead off the board so no component is knocked over. Then put the probe back in its black holder and let go."
```

Run one trial per process (or one per embodiment reset): the scorer records the trial history every control step
(pad visits, whether a component was knocked), and `task.py` clears it on every embodiment reset.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

The right arm pinches the probe by its back end, lifts it out of the slot and holds it up beyond the board's far-right
corner. The left arm pinches the lead 22 cm from the meter (where it still lies on the table) and lifts it. For each
pad, the left arm first moves to a high hold point beside the board's far-left part, so the lead hangs from it to the
probe with 2-3 cm of slack, clear of the components; then the right arm carries the probe over at 12 cm, measures
where the tip sits in its grip, and sets the tip down on the pad for 1.6 s (closed loop on the measured tip). After pad
3 the left arm lays the lead back down beyond the board and lets go, and the right arm puts the probe tip-first back
into the slot, lets go and moves clear. The probe arm grips with its wrist camera on the tip side, so the wrist view
shows the tip and the pads. It reaches `task_score` 1.0 in 914 steps. Videos are in `demo/`.

Shortcut variants (`PROBE_DEMO=...`), each scoring exactly 0:
- `nothing`: do nothing (probe stays in its holder, no visits).
- `nearest`: the full two-arm procedure, lead held up, probe stowed and released, but the pads are probed nearest to
  the probe first (3, 1, 2) - the same order as right-to-left or far-to-near sweeps. The first visit (pad 3) is out of
  order.
- `drag`: one-armed, correct order 1-2-3 and the probe stowed, but the lead is left to trail over the board; it sweeps
  the relay off its spot (moved 8 cm) on the way to pad 1.

## Scoring

History is recorded every control step (20 Hz) and latched until the next reset.

- **Visit:** the probe's tip end (the end face away from the lead) lies over a pad's 20 x 20 mm square with the
  tip's lower edge within 5 mm above the pad surface, continuously for at least 1 s (20 steps). One continuous contact
  is one visit.
- **Out of order (score 0):** a visit to any pad other than the next in the sequence 1, 2, 3 or the pad just visited
  (re-touching the current pad is harmless; returning to an earlier pad, or skipping ahead, is not). Brief touches
  under 1 s are ignored.
- **Knocked component (score 0):** any capacitor or the relay moves more than 1 cm horizontally from its start or
  tilts more than 30 degrees at any time.
- **Not stowed / not released (score 0):** at the end the probe tip is not at least 8 mm inside the holder slot
  (between its walls, below their tops), or a gripper's TCP is within 3.5 cm of the probe or any point of the lead.
- Otherwise **1/3 per pad visited in order**: 0.33 for pad 1, 0.67 for pads 1 then 2, 1.0 for 1, 2, 3.
- `success()` = score 1.0 with the lead at rest, held 1 s.

Real-world measurement: video of the trial (pad contacts and their duration, order) and tape marks around each
component's footprint (moved > 1 cm or tipped = 0); final check by eye with the grippers away.

## Runtime and seeds

Run limit 1,600 steps (80 s at 20 Hz); the demo needs 914. A trial takes several minutes wall-clock on one GPU
(first start of Isaac Sim adds 1-2 min). The layout is fixed (`object_jitter=0`; the cable is never jittered and the
components start on their marked spots), so every seed starts from the same state.
