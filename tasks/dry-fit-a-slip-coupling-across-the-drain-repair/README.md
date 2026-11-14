# Dry-fit a slip coupling across the open joint of a drain-pipe repair

A cracked length of a 1-1/2 in Schedule 40 PVC drain line has been cut out, and a replacement length put in the gap.
Its far joint is already dry-fitted with a repair coupling. The second repair coupling sits slid back onto the near
fixed end. The robot has to dry-fit that second coupling across the open joint, centred on the witness marks, and let
go. A repair coupling has no centre stop, so it slides along the pipe with 0.2 mm of play per side. The loose
replacement sags about 1.8 degrees in its dry far joint, so its free end hangs about 6.5 mm low. The coupling's
bevelled mouth cannot lift it: one hand has to hold that end up in line while the other slides the coupling across.

This is the dry-fit step that plumbers carry out on every joint before cementing. No primer, cement or seal is
claimed. Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py`
must sit next to `task.py`, which imports it. It is the parts kit: multi-colour parts built from boxes and cylinders,
with boxes turned about two axes (`roll`, `pitch`) for the round bores and bevels, and a per-part contact material.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Dry-fit the second repair coupling across the open joint, centred on the witness marks, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 700 --store-frames --log-dir logs/dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 700 --store-frames --log-dir logs/demo
```

- **Runtime per trial:** at most 35 s of simulated time (700 steps at 20 Hz), which is about 2 minutes of wall time
  on an L40S once Isaac Sim has started. The demo plan lasts 26.5 s.
- **Seeds:** submitted runs use `-E object_jitter=0`. The layout is fixed. The replacement and both couplings are
  the loose objects.

## Parts, as built

Positions are in metres in the world frame: x is forward from the line through the arm bases, y is to the robot's
left, and the table top is at z 0.7475. The drain line's axis runs along x at y 0, 90 mm above the table.

| Part | Simulation | Real item |
|---|---|---|
| Near fixed end | fixed, 48.3 mm cylinder, x 0.045 to 0.182 | the cut end of the installed line; deburred |
| Far fixed end | fixed, 48.3 mm cylinder, x 0.402 to 0.550 | the other cut end |
| Pipe clamps | fixed: a 6 mm foot plate, a 16 mm steel post and a saddle under each end, at x 0.072 and 0.515, with a strap round the pipe | steel pipe clamp on a post |
| Replacement | loose, 153 g, 48.3 mm cylinder 212 mm long (x 0.186 to 0.398, 4 mm short of each fixed end), with both cut ends bevelled on the outside 2 mm deep at 30 degrees | 1-1/2 in Sch 40 PVC (0.72 kg/m), cut 8 mm short and bevelled |
| Repair couplings (2) | loose, 80 g, 80 mm long, 24 flat staves: bore 48.7 mm across flats, 57.5 mm outside, both mouths bevelled 2 mm deep at 30 degrees, no stop | PVC repair (slip) coupling, no centre stop |
| Witness marks | 4 mm black bands, visual only, 38 mm from each pipe end (x 0.144, 0.224, 0.360 and 0.440) | marker lines at half the coupling's length less half the gap |

The pipe and couplings use dry PVC-on-PVC friction (static 0.40, dynamic 0.35). The fixed ends and clamps are one
rigid body each, as an installed, clamped drain line is. The replacement's far end starts centred in the far
coupling (x 0.360 to 0.440). It then sags in that coupling's clearance during the embodiment's settle: in every trace
it hung 6.4 to 6.7 mm low at its free end, at 1.7 to 1.9 degrees. The near coupling starts fully on the near fixed
end, from x 0.096 to 0.176, with its leading mouth 6 mm back from the cut.

## Scoring (exact)

`score()` and `success()` in `task.py`. A depth is the length of a pipe end inside a coupling. It is measured
along the coupling's own axis, from the mouth to the pipe's cut face.

- **No credit at all unless** every condition below holds:
  - **Let go:** every control step, a callback measures the convex hulls of both arms' gripper and fingertip links
    against every collision box of the replacement and both couplings. A cylinder counts as its box. The check
    passes only when all are more than 5 mm apart. Until the first measurement, the state is unknown, and unknown
    scores 0.
  - **Still:** the replacement and both couplings are each under 1 cm/s and 0.5 rad/s.
  - **In line:** both ends of the replacement are within 1.5 mm of the line's axis, and the replacement is within 1
    degree of the line's direction.
  - **Far joint still made:** the far coupling's middle is within 1.5 mm of the axis and within 1 degree of it.
    Both the far fixed end and the replacement are at least 30 mm inside the far coupling.
  - **Near coupling on the line:** its middle is within 1.5 mm of the axis and within 1 degree of it.
- **1.0:** the near fixed end and the replacement are each at least 32 mm inside the near coupling, and the two
  depths are within 6 mm of each other. The mouths are then within about 3 mm of the witness marks.
- **0.5:** each is at least 15 mm inside the coupling, but not centred as above. A coupling that bridges the joint
  with at least 15 mm on each side holds the replacement in line by itself after release, so this is a real,
  stable partial dry fit.
- **0.0:** anything else. That includes the coupling still on the fixed end, a coupling pushed on less than 15 mm,
  the replacement held up by a gripper, and the far joint pulled apart.

Full score counts once it has held for 1 s (the embodiment's success hold). Limits are compared with a 1e-6 margin,
so a value exactly at a limit passes. Nothing is judged over the trial history: every condition is a state that a
person can check with a ruler at the end. No condition can be met while either hand still holds a part.

## How each point of the earlier review is addressed

The idea was reviewed before, as a revisable task, with this feedback: "oversized dry-fit elbow and coupling allow a
sagging pipe to pivot and remain uncemented"; "specify the operation as a realistic dry-fit preparation or include
the actual sealing procedure; use real fitting clearances and a support arrangement that permits the stated
alignment".

- **Realistic dry-fit preparation:** the title, instruction and context call the task a dry fit. That is the step
  plumbers do before cementing, to check length, alignment and the coupling's position against witness marks. No
  sealed or cemented joint is claimed.
- **Real fitting clearances:** the couplings' bore is 48.7 mm across flats on the 48.3 mm pipe, which is 0.2 mm per
  side. That is a repair coupling's sliding fit. Repair couplings are made with no centre stop so that they slide
  fully onto the pipe. There is no oversized elbow or socket anywhere.
- **Support arrangement that permits the alignment:** the fixed ends are rigid and clamped at the right height on a
  common axis. The replacement is supported only by contact: in the far coupling at the start, and in both couplings
  at the end. With both couplings across, the clearances leave it within about 0.5 mm and 0.1 degree of the line in
  the demo. No joint, spring or friction setting holds anything in place.
- **The sag is real and is the reason for the second hand:** with 0.2 mm of play, one dry joint lets the free end
  hang about 6.5 mm low. Pushing the coupling without holding the replacement up leaves it jammed on the
  replacement's end (see the one-arm probe below).
- **Simulation, reset and demo updated:** all three are described in this README and in task.json.

## The demo, and what a policy can see

`demo()` is open-loop. It uses only the published layout and two settings calibrated once for the robot: how far
the left arm settles below and to the side of its target while it holds the pipe. It reads no object state.

1. The left hand comes down over the replacement, 8.4 cm beyond the open joint, and pinches it. It then lifts the
   free end into line and holds it there.
2. The right hand grips the near coupling 22 mm behind its middle, so the two hands stay about 10 cm apart. It slides
   the coupling 50 mm toward the far end of the line, across the joint, and lets go.
3. Both hands open and move away.

Run `trace-20261114T044537` on GPU: before the slide, the free end was held within 1.3 mm of the axis. The coupling
ended with depths of 35.3 mm (fixed end) and 38.0 mm (replacement). The replacement ended 0.5 mm below the axis at 0.0
degrees. The coupling's drag pushed the replacement 3 mm toward the far end. Score 1.0.

Tuning history: while the hold was being calibrated, the coupling jammed on the replacement's end in six traces.
The free end was 3 to 6 mm off the axis, or the contact friction was still the kit's grippy default (1.2). In
the seventh, the coupling went 3 mm past centre, with its depths 6 mm apart, and scored 0.5.

**One-arm probe:** this was the same task, with the left hand kept away and the right hand's plan unchanged (run `_dryfit-onearm/trace-20261114T050648`, a scratch copy that is not part of the submission). The free end stayed 6.3 mm low. The coupling's mouth stopped 0.5 mm onto the replacement's bevel and stayed there: score 0. A one-handed push does not work.

## Scores

Scripted demo: 1.0 (run `demo-20261114T054444`). Dream-1: 0.0 (run `dream-20261114T055241`, which hit the step limit).

## Fidelity and limits

- True size and true clearances. The bores are 24-sided, and the simulator's cylinders are faceted. That faceting is
  why the start sag (about 1.8 degrees) is larger than the roughly 0.6 degrees (about 2 mm at the free end) that 0.2 mm of play over 38 mm of engagement would give in a true round bore.
- The drain line is level here. A real drain slopes about 1/4 in per foot (1.2 degrees), which makes no difference
  to the fit.
- The pipe ends are solid colliders. The bore shows only as a dark disc at each cut face.
- PVC sockets on ordinary fittings are tapered for an interference fit. Repair couplings are not. Only repair
  couplings are modelled.
- The layout is fixed, as the board asks. A memorised replay of the demo cannot be ruled out.
