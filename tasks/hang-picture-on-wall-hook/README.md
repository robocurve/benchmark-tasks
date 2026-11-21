# Hang a framed picture on a wall hook by the wire behind it, level, then let go

A framed picture (30 x 40 cm, 0.2 kg poster frame) stands on the table leaning back against a wall panel, just below the one
picture hook nailed into the wall. Braided picture wire runs between two D-rings on the back of the frame. The robot
is told to hang the picture on the hook by its wire so that it hangs straight, and to let go.

**Ability tested:** lifting and carrying a wide, flat object upright with two hands, then catching a wire that
cannot be seen. Once the frame is near the wall, the wire is hidden behind it (from the top camera and from both
wrist cameras), so the robot has to keep the frame upright and within about a centimetre of the wall, tip its top
slightly toward the wall so the slack wire falls back against it, and lower the frame until the wire drops into the
hook's throat and takes the weight, which it can only feel through the grippers or see as the frame stopping. Then it
has to let go without pulling the wire off the hook, and, if the frame hangs crooked (a wire caught off-centre stays
where friction holds it), push a lower corner until the top is level, and move clear.

**Why two arms:** the frame is 30 cm wide and the arms can only pinch its side rails. Held at one side alone, the
frame's weight acts 15 cm from the pinch, which a two-finger pinch on a 22 mm rail cannot resist: the frame swings
round the grip, and in the simulator a one-arm attempt drops it on the table. The middle of the top rail, the only
place one pinch could hold it level, is above where either arm can reach with its gripper pointing down.

## How this differs from existing tasks

- **BEHAVIOR-1K picture-hanging activities** (e.g. hanging pictures on a wall) place a rigid picture so that it
  attaches to the wall when it is near the right spot. Here nothing snaps on: a real braided wire, free to sag and
  swing, has to be caught on a real hook's 6 mm throat behind the frame where no camera sees it, the frame's weight
  has to be handed over to the wire without the wire sliding off, and the result is judged by whether it then hangs
  level on its own.
- **RLBench hang_frame_on_hanger** has one arm put a rigid frame onto a hanger peg, where the frame's own rigid
  geometry engages the hanger in view. Here the frame hangs by a flexible braided wire behind it that no camera sees
  near the wall, it needs both hands to stay upright, and the score asks for the wire seated in the hook's throat,
  the frame level within 1 cm and flush to the wall, and both grippers clear.
- **Other tasks that drape a cable over hooks** hand the robot the cable itself to lay over a visible hook. Here the
  robot never holds the wire: it holds a 30 x 40 cm frame with both hands and the wire follows the frame, out of sight.
- **Hanging a small planter by its ring on a wall bracket** lands a rigid ring the robot can see on a visible hook
  and scores the load clear of the table. Here the loop is a braided wire hidden behind a 30 x 40 cm frame, caught
  blind in a flat hook's throat, and the frame is then judged level and flush to the wall.
- **Pegging a hanging photo back onto a display wire** is about squeezing a clothes peg onto a taut line next to a
  light print. This task has no pegs: it is about a framed picture, a wall hook, the blind catch and levelling.

## Real-world setup (materials)

- **Wall:** white-painted 12 mm MDF sheet, 90 cm wide x 60 cm tall, standing on the table along y with its face 45 cm
  in front of the arm bases, held by two steel shelf brackets (L-brackets, 150 mm) screwed to its back and clamped
  to the table with two F-clamps. A pencil line drawn level across it 50 cm up, checked with a spirit level.
- **Hook:** one 30 lb steel nail-in picture hanger (OOK or Hillman, conventional single-nail type: plate 13 x 32 mm,
  lip 12 mm out from the wall), nail driven through it into the MDF at y 0 so the throat is 39 cm above the table.
- **Picture:** a black plastic poster frame, 30 x 40 cm outer size (portrait), moulding 25 mm wide x 22 mm deep,
  styrene glazing (no glass, so a drop cannot shatter), hardboard back with a printed picture inside, 0.2 kg in all
  (weigh it; tape washers inside the back if it is lighter).
- **Hanging hardware:** two D-ring hangers (single-hole, 25 mm) screwed to the back of the side rails 10 cm below the
  top, 2 cm in from each side edge; 1.5 mm braided stainless picture wire,
  27 cm between the rings, each end twisted back on itself through its ring. Hung, the wire's peak sits about 6 cm
  below the top of the frame.
- **Start:** the frame stands on its bottom edge on the table, centred at y 0, face to the robot, leaning back at 25
  degrees with its top against the panel just below the hook (bottom front edge at x 26, marked with tape), the wire
  hanging loose behind it. Both arms in the ready pose, grippers open.

## Real-trial scoring

Tools: the trial video, a ruler, the level pencil line on the panel. Judged 2 s after the robot stops.

- **0** if the frame rests on the table (lowest corner within 5 mm of it), a gripper is within 5 mm of it, or it does not hang from the hook by its wire
  alone (seen in the video and by looking behind the frame afterwards).
- **1.0** if it hangs from the hook by the wire, the two top corners differ in height by less than 1.0 cm (measured
  with the ruler from the pencil line; 1.0 cm over 30 cm is 2 degrees), the bottom rail is within 1 cm of the panel
  (ruler), and both grippers are at least 5 cm from the frame.
- **0.5** if it hangs from the hook by the wire but is crooked (corners 1.0 cm or more apart), stands off the panel at
  the bottom by more than 1 cm, or a gripper is between 5 mm and 5 cm from it. Example 0.5 end states: wire caught 2 cm off-centre so the frame hangs about 4 cm crooked; hung level with a gripper left 2 cm away.

## Fidelity

The frame is one rigid box with the real outer size and mass. The wire is a chain of 2 cm rigid segments with
bending springs, no stretch or twist; for stable contact it is 3 mm thick and 60 g (the real wire is 1.5 mm and a few
grams). Each end is a short 30 g stub, standing in for the end twisted back through the D-ring, held in its ring by a
stiff spring that gives about 3 mm under the hung load, the same at both ends. The hook is a fixed rigid shape (its
nail holds 30 lb, so it never pulls out) with friction 0.4, so a wire caught off-centre can leave the frame crooked,
as a braided wire does on a real hook. Panel and table are rigid.

**Simulated scorer, frame pose.** The scorer takes the frame's orientation from the embodiment's object state, a
quaternion in (x, y, z, w) order, and turns it into a rotation with a helper that expects that same order. Checked
offline on known poses, with the frame hung against the wall: the measured top-corner height difference is 0.00 cm
upright, 0.52 cm at 1 degree, 2.61 cm at 5 degrees and 7.76 cm at 15 degrees, matching 30 cm x sin(angle). Those score
1.0, 1.0, 0.5 and 0.5. An upright frame whose bottom rail stands 2 cm off the panel reads a 2.00 cm gap and scores
0.5.
The demo was rerun on the corrected scorer. It reaches 1.0 at step 434 of the 3,600-step limit, with the top corners
0.24 cm apart, both grippers 18 cm clear and the wire in the hook's throat carrying the frame. In simulation the score
is read on the final state. The arms are parked by 19.3 s and the run ends at 21.7 s, so the score is taken more
than 2 s after the robot stops, as in the real trial.
