# Fix a hook tight to a hollow panel with its toggle bolt

A steel hook sits loose at a hole in a hollow wall panel, on a 3/16 in (10-24) spring-wing toggle bolt whose wings have
already opened in the closed cavity behind the board. About 2.8 turns of thread are left to take up. The toggle nut
cannot be reached: turned on its own it just spins with the screw. It only holds, by friction of its wings on the
back of the board, while the hook is pulled outward. So one arm has to keep pulling the hook out (and square to the
pencil line) while the other turns the screw with the ratcheting screwdriver from the holder.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. Two helper files
must sit next to `task.py`, which imports them:
- `parts.py`: the parts kit (multi-colour parts with joints), in a version extended for this task (see below).
- `tilt.py`: lets the scripted demo lean a gripper; it affects only the scripted demo, never Dream-1 or scoring.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fix the hook tight against the panel, square to the pencil line."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/demo
```

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 2 to 3 minutes of wall time on
  an L40S once Isaac Sim has started. The demo script lasts 55 s (it waits clear while the pull test runs); its trial ends on success at about 48 s.
- **Seeds:** submitted runs use `-E object_jitter=0`; the layout is fixed. The hook and the screwdriver are the loose
  objects; with jitter the hook's bolt would start off the hole's centre (it still sits in the hole's 3 mm play).

## The mechanism

- **Thread:** the toggle nut rides the bolt on a helical joint: a slider (`thread`) on the bolt, and the nut's
  hinge (`nut`) on that slider, tied by a PhysX mimic joint so the nut travels 1.06 mm along the bolt per turn
  relative to it (10-24, 24 threads per inch). The bolt turns in the plate on a hinge (`bolt`, 0.01 N m s/rad drag: its head bearing on
  the plate). The nut's hinge has 0.002 N m s/rad drag (thread friction), so a nut that nothing holds turns with
  the screw. Nothing switches between "spins" and "holds": the nut holds only when contact friction of its wings
  on the board's back face, which exists only while the hook is pulled outward, outweighs that drag.
- **Checked on the GPU** with a motor on the bolt instead of the screwdriver: 2 turns with the hook not pulled
  moved the nut 0.0 mm along the bolt (it turned with the screw); then 3 turns while an arm pulled the hook moved it
  3.80 mm (1.27 mm per turn, the lead of the 1/4-20 bolt in an earlier build) and drew the plate down flat on the face.
- **Screwdriver:** the shaft and tip are a link on a hinge in the handle. Its upper joint limit is the ratchet pawl:
  every control step an `on_step` callback lowers it to where the shaft has been (plus 0.02 rad play), so the shaft
  can turn anticlockwise in the handle (the back swing) but drives the screw clockwise.
- **Board:** the panel's contact material is slightly springy (2 x 10^4 N/m, 40 N s/m, PhysX compliant contact),
  as wallboard paper is, so a screw turned past tight builds a bounded clamp force instead of an impulse.
- **`parts.py` changes** from the stock kit: joints may hang from another joint's link (`parent=`); a slider may be
  tied to a hinge by a thread (`thread=(hinge, lead)`, written as `PhysxMimicJointAPI`); shapes may be turned about
  the part's vertical (`yaw=`, used for the octagonal hole and bore) or have no collider (`collide=False`, the pencil
  line and the thread slider's marker); a part may set its contact offset (0.5 mm for the hook, 0.3 mm for the
  screwdriver, 2 mm for the panel), turn link-to-link collision off, and use a compliant material.

## Scoring as implemented

`score()` and `success()` in `task.py`. Everything credited can be checked from outside the wall: the wings in the
closed cavity are not looked at; whether they grip is judged by pulling on the hook, as a fitter would.

- **Flush** (`_flush`): all four corners of the plate's underside within 0.5 mm of the face height (76.2 mm above
  the table).
- **Square** (`_yaw_off_deg`): the angle between the plate's long axis (bolt end to hook end) and the line's
  direction to the left: at most 3 degrees for 1.0, at most 10 for 0.5. A hook turned round fails.
- **Released and still:** an `on_step` callback measures every control step the distance from both arms' finger and
  gripper hulls (`yam_arena.analysis`) to every shape of the hook (all links); all must be more than 5 mm (the
  screwdriver may stay in a hand). Until it has measured, or if it cannot, the state is unknown and nothing scores
  (fail closed). The screwdriver's box must also be at least 5 mm from every shape of the hook; the screwdriver's own
  motion is not judged (held still in a gripper it was reported moving at 7 cm/s, so it may be held or put down).
  `score()` counts consecutive control steps with the hook at rest (under 1 cm/s and 0.5 rad/s) and its pose and
  thread unchanged (within 0.2 mm and 0.2 degrees).
- **Pull test** (`_pull_step`, `_apply_pull`): once the hook is flush, within 10 degrees of the line, released, clear
  of the screwdriver and has been still for 20 control steps (1 s), the scorer pulls it: an `on_step` callback applies
  a steady 5 N force straight out from the face (up, the panel lying face-up) at the foot of the hook's upright
  (on the plate's centre line, 80 mm from the bolt), for 10 control steps (0.5 s), then removes it for 10 more (0.5 s).
  Tight means no corner of the plate rose more than 0.5 mm above its height before the pull, at any step of the
  test, and the hook is back at rest within 0.2 mm, 0.2 degrees and 0.2 mm of thread travel of its pose before the pull.
  Fail closed: the callback records the force it set, and the test fails unless PULL_N was applied on at least 9 of
  the 10 pull steps and some corner rose at least 0.05 mm (a pull that did not act is not a test). A gripper within
  5 mm, the screwdriver within 5 mm or the toggle run off at any step of the test, or the hook moved after it,
  starts the test over (after a new 1 s at rest). A hook only resting on the face, with any play left on the thread,
  lifts under the pull until its wings meet the back of the board, and fails.
- **Credit:** 1.0 when the last test passed and the hook is still flush, square, released, at rest and where the
  test left it; 0.5 the same at 3 to 10 degrees; 0 otherwise, including a trial that ends before a test is finished.
  Success needs the same 1.0 state and must hold 1 s.
- **Whole trial:** if the nut's travel ever reaches the bolt's end (38.8 mm back from the start, less 0.5 mm, about
  36 turns; the nut's and bolt's hinges allow 250 rad each way so this can happen), the trial scores 0, until a scene
  reset: the hook back within 0.5 mm of its start, at rest, after a jump of over 20 mm in one control step.
- **Checked on the GPU:** two probe runs applied the same 5 N pull at the upright's foot. The hook as the demo leaves it
  (no play left on the thread) rose at most 0.27 mm, at the hook end, and came back within 0.005 mm. The same hook
  resting with its wings 0.5 mm off the back of the board rocked its hook end up 2.2 mm. A tight hook moves at all
  because the simulated board face is springy (2 x 10^4 N/m) and the thread's drag model keeps no clamp preload
  (the demo ends 0.003 mm past contact). 5 N, a light tug about three times the hook's weight, separates a tight hook
  (0.27 mm) from one with 0.5 mm play (2.2 mm); at 10 N a tight hook already moved 0.75 mm.
  The final scripted demo passed the test and scored 1.0 (the trial ended on success at 47.5 s).
- `near_misses.py` checks every tier and boundary (2.9/3.1/10/10.5 degrees; a lift of 0.5 mm and 0.05 mm more under the
  pull, straight up or with the hook end rocking up; 0.4/0.6 mm off the face; a 1 degree tilt), the reviewer's case
  (a hook resting flat with its wings 0.5 mm off the back of the board, whose hook end rocks up 2.2 mm under the pull, as measured on the GPU: 0), half
  the thread taken up, the toggle spun with no travel, the hook not back where it was after the test, the pull never applied, a pull
  under which no corner rose 0.05 mm, the
  screwdriver left in the head or 4 mm from the plate, and the toggle run off then refitted. Offline there is no
  robot and no physics, so it sets the arms as measured clear and gives the hook's pose under the pull. Checked
  separately offline: with the arms' state unknown (`None`) the tight-and-square state scores 0.

## Scripted demo

`demo()`, fixed waypoints from the published start layout only (no object state is read):

- Right arm: picks the screwdriver from its holder by the handle (pads 27 mm apart on 35 mm), carries it over the
  bolt, turns the hand a quarter turn (the cross tip looks the same), and lowers the tip into the recess, aimed
  0.6 mm in front of and 0.2 mm left of the head's centre, where the held tip was measured to settle short of the
  commanded point at this pose. It presses 4 mm past seated, then makes seven 190-degree clockwise strokes (wrist
  +145 to -45 degrees, the wrist's range here) with a ratchet back swing between them, eases off, lifts the driver
  out and holds it still over its holder while the pull test runs (it does not need putting down).
- Left arm: leans 45 degrees (finger tips toward the bolt; `tilt.py`) and pinches the hook's upright across its
  5 mm thickness, then pulls it 35 mm upward (commanded; the soft arm lifts the bolt end about 2.5 mm before the
  wings meet the back face) and holds it there square while the right arm turns, then eases off and lets go once
  the driver is out. Straight down, the two wrists 8 cm apart collide (forearm hulls overlap); leaning keeps them
  over 28 mm apart.

## Fidelity

True size, fixed layout. The panel lies face-up on the bench so a straight-down tool can turn the screw (the arm
turns tools only about the vertical); on a wall the same pull-and-turn is needed. The thread is an ideal helical joint
with velocity drag standing in for thread friction. The toggle starts already opened behind the board with about 2.8
turns of thread left, so the demo is practical. The recess has square-sided 2.6 mm slots (a real #2 Phillips recess
tapers to a cone). The board is rigid with a slightly springy surface, and the thread holds no clamp preload: 5 N, a light tug about three times the hook's weight, separates a tight hook (0.27 mm) from one with 0.5 mm play (2.2 mm). No damage to the board is modelled.

## Licences

All geometry is built from primitives in `task.py`; no third-party assets.
