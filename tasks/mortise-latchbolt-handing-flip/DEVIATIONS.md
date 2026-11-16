# DEVIATIONS

1. Layout: the spec placed the lock near the station centre; the reach screen (screen.py) measured a ~0.155 m TCP
   ceiling and no yaw -90 deg solution at (0.33, -0.02), so the lock sits at (0.32, -0.15) on the right arm's side
   (still near the image centre). Conservative option: keep real size, move the fixture.
2. Spec wording "stood on its front edge ... latchbolt up" is self-contradictory (the latchbolt is on the front
   edge); built as standing on its back edge, faceplate up, which is what "latchbolt up" requires.
3. Spring rate (0.4 N/mm) and key engagement (6.5 mm) are not published; estimated and labelled in fidelity/README.
   A first 0.8 N/mm value let the pads slip during the pull in MuJoCo; 0.4 N/mm is within the range of a light latch
   spring (estimate).
4. Guide friction lowered from 0.8 to 0.3 (lubricated brass in steel): at 0.8 the head wedged in its opening when the
   pads closed (8 corner contacts, ~4 N each, measured in MuJoCo).
5. (superseded by 9) Function check was a straight-down push on the head's top land; revision 2 replaces it with the
   real one, a strike on a closing jamb.
6. Handing screw (step 2 of the maker's procedure) is out of scope (it applies to locking functions).
7. Revision 1 (after a pre-submission review): the function check (push in >= 15 mm, springs back) and the pull are no
   longer scored -- they were run history a photo cannot verify; scoring is now end state only. The demo still does
   the push check (unscored). The hands-off term stays in the 1.0 tier although the official guide does not require
   it: the lock's end state after success-then-arm-replay (mj_check undo) is identical to success, so only the hands-off
   term keeps undo < 1.0, and the end-state photo needs the head unoccluded (official guide: scoring by
   "a tape measure, a photo"). Conservative choice: keep it as a pure end-state condition, drop the 2 s timer.
8. Revision 1: `media` removed from task.json (it was an empty string); the official upload tool writes the media ID at
   submission.
9. Revision 2 (after reviewer "too simple?" and a pre-submission video review: one arm, no visible consequence, head turned
   while still in its slot, overlay mismatch): added the strike side (jamb + strike on a linear guide, closed by the
   left arm) so wrong handing jams visibly and right handing latches; scoring now needs the jamb latched. Pull stop
   9 -> 13 mm and a 0.6 s wait before the turn, so the tail is 6.5 mm clear when the head turns (was 1.3 mm in the
   logged Isaac run: max pull 7.8 mm vs 6.5 mm engagement).
10. Latch spring 0.4 -> 0.2 N/mm (estimate either way): at 0.4 N/mm the closing push stalled in MuJoCo (the arm's
    pushing force at that pose is a few newtons). Bevel made one flat plane (pitched collider) from 2 mm above the
    faceplate (was a 3-step box stair from 6 mm, which a strike lip would snag on); the visual head follows.
11. The jamb slides instead of swinging (fidelity note): the guide covers the last 9 cm of a door's travel at the latch,
    where the arc deviates 4.5 mm from straight on a 0.9 m door. Its ~1 N return spring stands in for the door seal.
12. The left arm pushes the jamb's back end with closed fingers (no grasp): a knob pinch needed yaw 90 deg, which the
    left arm cannot reach at that height (IK error 13-43 mm); the end-face push at yaw 0 reaches with 0.1 mm.
    The push target is 41 mm past the stop; the arm's position error is its pushing force (hard stop takes it).
13. Hands-off box now covers the jamb over its whole travel (not only at its stop): mj_check undo rewinds the left arm
    to the open end of the jamb, which cannot pull a latched jamb open, so only the box keeps undo below 1.0. The
    right arm parks at (0.16, -0.32) to stay outside it.
14. Isaac-only fixes after two official 1280 runs scored 0 (MuJoCo had 1.0): (a) Isaac aimed the jamb's return spring
    at the default 0 (closed), so before the first step the jamb slid 48 mm and jammed on the unturned latch (STATE log
    jamb -0.042 m from step 0, turn 0 all run); the task now sets both spring targets on reset and every step, as it
    already did for the latch spring. (b) the left fingers came down on the jamb's back end; the descent now happens
    60 mm behind the end face with short settles. Third run: task_score 1. Observe-only STATE lines every 2 s stay in
    the log as run evidence.
