# Put the door chain on: the dangling knob in through the slide track's keyhole and along the slot to its closed end

A section of an interior door stands shut in its frame on the table, with a chain door guard fitted the usual way: the
guard's brass slide track on the door's face near its latch edge, the chain holder on the frame's casing beside it. The
chain hangs from its holder with its brass knob dangling on the end. The instruction asks for the door chain to be put
on.

Many front doors, flats and hotel rooms have a chain door guard: with the chain on, the door opens only a few
centimetres. Putting it on at night is something most people have done or watched. A manufacturer's fitting sheet for a
chain door guard says: "Mount slide plate 1/4" from edge of door at
convenient height. Large hole at end of slot faces away from door jamb. Mount chain holder on door jamb so that knob on
chain will reach hole in slide plate." A typical guard, Prime-Line U 9907, has a solid brass slide of about 100 mm and a
6-inch, 8-link welded steel chain.

To put the chain on, the knob's head is pushed into the slide's large hole (the keyhole) and the knob is slid along the
slot to its closed end, next to the jamb. There it stays; when the door is opened, the chain pulls the knob against the
slot's end and the door stops.

## The hand skill

- The knob is small and hangs free: 10 mm across and 18 mm long, on 15 cm of light chain, 14.6 cm above the table,
  in front of the casing. It swings away when bumped and turns freely on its chain. The hand has to come in at its
  height and close on it without knocking it round.
- It is then carried up 15 cm to the track and its 11 mm head is pushed square into the 14 mm keyhole on the vertical
  door face: there are 1.5 mm of play each way, and the head goes in nowhere else (pressed against the slot anywhere
  else, it rides on the outside of the track).
- Once in, the knob is slid 8.3 cm along the slot (6.4 mm wide; the knob's neck is 4.8 mm) to the closed end, keeping
  the head inside the channel: pulled away from the door while still at the keyhole, it comes back out. The chain
  drags on it all the way.
- Then it is let go at the end. Let go early, it stays part way along the slot.

One hand does it, as for a person; the other arm is not needed. The task tests catching a small part on a swinging
chain, an insertion into a keyhole on a vertical face, and a guided slide in a narrow slot without losing the
part, all with the chain attached.

The task is the task file [`put_on_a_door_chain.py`](put_on_a_door_chain.py) for the `yam_arena` embodiment (scene,
scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the door, the frame, the guard, the chain and the knob are built in these files from boxes,
cylinders and capsules.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint. The door
with its frame and the guard's track (one fixed body of many shapes), the chain (eight bodies on ball joints) and the
knob (a body on a ball joint at the chain's end) are built with `bodykit.py`, which adds such bodies and joints:
importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives the task file
each body's pose.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, heights above the table top.

- Door: a white interior door section, 35 mm thick, 20 cm wide and 45 cm tall, standing on the table, its face toward
  the robot at x 28.0, its latch edge at y -3.0 (the door runs to y +17.0). Shut: fixed.
- Frame: the jamb (20 mm) beyond the door's edge with a 3 mm gap, and a wooden casing 70 x 12 mm, 12 mm proud of the
  door's face, from y -3.6 to -10.6. Fixed.
- Slide track (on the door): a brass channel along y, 100 mm long and 18 mm tall, standing 6 mm off the door's face,
  its middle line 30.0 cm up. Its closed end is at y -2.4 (6 mm from the door's edge), its far end at y +7.6. The front
  has lips 1.2 mm thick with a slot 6.4 mm wide along the middle and the keyhole, a 14 mm opening centred at y +6.4.
  Behind the lips is a cavity 3.8 mm deep and 14 mm tall in which the knob's head rides.
- Chain holder (on the casing's middle, y -7.1): a brass plate 22 x 45 mm; the chain hangs from its stud, 16 mm out
  from the casing, 31.0 cm up.
- Chain: 8 steel links at 19 mm pitch (152 mm), 3.2 mm wire, 2.5 g each.
- Knob (brass, 9 g): a body 10 mm across and 18 mm long, a neck 4.8 mm across and 3.5 mm long and a head 11 mm across
  and 2 mm thick on its front end, and a tongue on top of the body that the last link hooks through. At the start it
  hangs level under the chain, head toward the door (4.5 mm in front of the casing), its axis 14.6 cm up, its body's
  middle at x 24.9, y -7.1.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled:

- The track as its real shape: back plate, top and bottom walls, end walls, lips around the slot and the keyhole. The
  knob's head passes only through the keyhole; behind the lips it is trapped and can only slide along the slot.
- The chain: eight rigid links, each hung from the one above (the first from the holder's stud) by a ball joint that
  bends up to 80 degrees each way; the knob hangs from the last link by a ball joint (90 degrees). Light damping
  (0.1 linear, 0.3 angular). Real links interlock and bend a little more freely.
- The keyhole is a 14 mm square opening (real keyholes are round, about 14 mm across).
- Not modelled: opening the door against the chain (the door is fixed; the real trial tests it by a pull on the knob),
  screws, the knob's knurling.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the knob's pose and the points of each finger's collision hulls.

- Captured: the centre of the knob's head is behind the lips (inside the channel, within 0.5 mm of the lips' inner
  face or deeper), level with the slot and between the end walls. A captured head cannot be pulled straight out.
- 1.0 (success, held 1 s): captured, with the knob within 8 mm of where it stops at the slot's closed end (its neck
  against the end), let go (every finger point 1 cm or more from the knob) and still (moving less than 1 mm in 1 s).
- 0.5: captured at least 10 mm along the slot from the keyhole's centre (past the hole) but not at the end, or at the
  end and still held.
- 0: otherwise: never put in, left in the keyhole, or pulled back out.

A real trial is scored with a ruler (the gap between the knob's neck and the slot's closed end), the trial video (the
gripper let go) and a pull test afterwards: the knob pulled straight away from the door by hand stays in the track.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=put_on_a_door_chain.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the door chain on." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dchain-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The left gripper backs off from the
ready pose, turns level (pointing forward, fingers open across y), comes in at the hanging knob's height so that the
knob sits between its open fingers, and closes on the back part of the knob's body (a 3 mm commanded gap on the 10 mm
body). It lifts the knob, carries it up and along to 2 cm in front of the keyhole, pushes the head in through the
keyhole until it touches the back of the channel (the knob is captured at step 379), slides the knob along the slot to
the closed end (commanded 10 mm past it), lets go and backs away. The right arm stays at its ready pose. It reaches
success at step 518 (25.9 s); the run ends at step 538, after the 1 s hold.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=put_on_a_door_chain.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the door chain on." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/dchain-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/dchain-dream/*.json; do inspect-robots video "$log"; done
```

`DCHAIN_DEBUG=1` prints, every 5 steps (`DCHAIN_DEBUG_EVERY=1` for every step), the knob head's position and the knob's
axis, whether the head is captured, how far along the slot it is from the keyhole's centre, whether it is at the end,
how close the nearest finger is, whether it is let go and still, the score, the last events, the extreme points of the
left fingers, and where each gripper is and how far open.

Two wrong ways (`DCHAIN_WRONG`):

- `DCHAIN_WRONG=slot`: the same catch and carry, but the head is pushed at the slot 4 cm short of the keyhole. It
  cannot go in: it rides on the outside of the lips as the knob is slid to the end, and when let go the knob falls back
  on its chain. Score 0.
- `DCHAIN_WRONG=early`: the knob is put in through the keyhole but let go half way along the slot. It stays in the
  slot, 13 mm short of the end (the opening fingers dragged it a little further). Score 0.5.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: debug runs of the demo at
320 x 176 without video took about 5 minutes from start to finish; the scripted demo, rendering 1280 x 720 video,
reached success at step 518.
Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

- A chain door guard of the usual size (for example Prime-Line U 9907: brass slide about 100 mm, 6-inch 8-link chain),
  under 15 US dollars.
- A section of a 35 mm interior door 20 x 45 cm (an offcut of a solid-core door, or a 35 mm board painted white), a
  20 mm jamb strip and a 70 x 12 mm casing strip, screwed to a base board so the door section stands upright with its
  face 28 cm in front of the midpoint between the arm bases.
- The slide screwed on the door's face 6 mm from its latch edge, its middle 30 cm above the table, keyhole end away
  from the jamb; the holder screwed on the casing's middle at the same height, so the knob hangs about 15 cm above the
  table.
- Before each trial, take the knob out and let it hang still, head toward the door.
