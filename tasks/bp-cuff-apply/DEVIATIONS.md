# bp-cuff-apply build deviations from BUILD-BRIEF.md / SPEC.md (2026-11-17)

1. Arm placement: axis 120 -> 100 mm above the table and the arm moved 80 mm toward the robots (cuff centre x 0.33 -> 0.25, crease
   x 0.43 -> 0.35). Why: a straight-down grip cannot reach over the arm top at x 0.33 / axis 120 mm (IK error 30-80 mm over a reach
   grid at x 0.25/0.33/0.40); at x 0.25 / axis 100 mm it reaches (< 6 mm). Underside still 45-51 mm clear for the cuff.
2. Upper-arm radius 51 -> 45 mm became 55 -> 49 mm (325 mm round the middle, within the cuff's 25-34 cm range; the mannequin
   circumference is unpublished either way). Why: with a 520 mm cuff on a 300 mm arm the free end must end ~230 deg past the start
   end (below the near side, out of reach); the ring stayed loose on that attempt (18-30 mm gaps).
3. Start state: the cuff lies across UNDER the arm (not beside it on the -y side), each 20 mm end piece sprung curled up so a
   down grip can pinch it. Why: a flat cuff on the table cannot be pinched, and passing the free end under the arm needs a
   3-handoff path that a first attempt flagged as unreachable; with the cuff under the arm its middle comes up round the arm when
   the ends are lifted.
4. Demo order: right holds the free end on the table while the left lays the start end over the top; then the right draws the
   free end round, lets it go, and presses the closure with closed fingers. Why: pulling the free end down the near side while
   pinched slipped the pinch (friction-limited) and locked a loose ring; letting go and pressing keeps it snug (max gap 5 mm).
5. Closure gate, first version: "pressed" = a gripper with closed fingers (each finger within 1.5 mm of closed) whose pad tip is
   within 4 mm of a loop segment that already lies on the hook face (brief: pad within 10 mm, or contact normal >= 2 N). Why: with
   10 mm the pinch holding the free end next to the loop engaged the closure mid-carry and locked a loose ring. Superseded by
   items 12-14: the 4 mm gate fired with no press at all.
6. Artery arrow on the outer turn (segment 21, 100 mm from the free end), so it shows after wrapping; the artery line is on top.
7. run_limit = demo plan duration x 1.3 rounded up to whole seconds (the check tool parses whole seconds). The original plan was a
   second and a half shorter; the revision lengthened it to 29.7 s, so the limit is 780 steps (item 12).
8. Friction: skin 0.6, cuff 0.7 (estimates). Lower skin friction (0.3) let the closed ring turn round the arm after release.
9. Text and staging pass folded in before the first submission: curled end pieces and the under-the-arm start disclosed as staging
   simplifications (task.json setup + fidelity, README Fidelity); 0.3 tier text "not closed or not snug"; the press rule stated in
   task.json; the engage gate factored into engages() (same logic) with tests (open fingers, fist 10 mm away, 20 mm overlap -> no;
   closed fist on lying overlap -> yes). Out-of-reach transition waypoints (30 mm at most, lift-off / park) noted in README.
10. Real-trial checks added (text only): README adds a 4 N spring-scale tug on the free end (closure stays closed); real staging of
    the curled end pieces stated (fold ~80 deg by hand, one strip of masking tape on the inner side) in README, task.json setup and
    reset_notes. No code, scene, demo or scorer change.
11. Instruction reworded: "Put the blood-pressure cuff on the patient's left upper arm, ready for a reading, then let go." ->
    "Wrap the blood-pressure cuff round the mannequin's left upper arm with its arrow on the marked line and its lower edge 2 to 4 cm
    above the elbow crease, fasten it, then let go." (the placement the 1.0 tier needs is named; the marked line is the dark artery
    line on the arm). Title: "patient's" -> "mannequin's". Why: the Dream-1 policy API refused the old wording (DA-4000 unsafe
    instruction, "attaching a cuff to a person"). The scene is a display-mannequin arm; the real-use context (nurses and medical
    assistants on patients) stays in task.json context. Scorer, scene and demo unchanged.
12. Rework after the first submission was not accepted. The official review's finding: the closure fired from a pad up to 4 mm away
    and stiffened every hinge from on_step. The closure gate is now CONTACT: pad tip within 2 mm (item 13; 1 mm in the first draft)
    of a loop segment that already lies on the hook face, sustained 0.3 s (6 control steps); a graze or a stand-off does not close it
    (replaces item 5's 4 mm rule). The fastening is the minimal hinge set spanning the overlap (hinges whose two segments both lie on
    the hook face, 21-25 on the ideal wrap), held at their angle by the stiff drive; every other hinge keeps its build-time stiffness
    and target from a passive table captured at import, so the released cuff holds shape by the closure and its own bending
    compliance, not a stiffened frame. A single fixed joint between the two faces is not expressible (both ends hang off one
    26-segment chain; a joint between them closes a kinematic loop that the PhysX articulation rejects). Two defects were fixed in
    the first draft of this rework on the way to a scoring demo, both measured in MuJoCo: (a) the bond used segment numbers as
    positions into the 25-joint array (there is no joint for the root segment), so the per-step hook raised IndexError at engage and
    was switched off -- the grippers then read as still on the cuff and 1.0 was unreachable; now mapped by joint name. (b) a 1 mm
    "take-up" on the bonded targets moved them 0.2 rad off the lock angle, which the release rule read as a 10 N peel and opened the
    bond 0.05 s after it closed; removed -- the bond holds the hinges where they engaged. Demo: the press waypoint moved from
    (y 0.037, z 0.1282) to (y 0.024, z 0.1260) and is held 2.5 s instead of 1.0 s. Why: the old pose commanded the pad 17 mm below
    where the arm's surface stops it, so it bounced and sat 1.3 mm off the loop face (over the 1 mm gate, contact only 2 isolated
    steps); the new pose rests the pad on the outer loop segments at 0.1 mm for 9 s. Plan 28.2 -> 29.7 s, so the run limit rose to
    780 steps. Checks on that draft: MuJoCo 4.0.1 simulator check PASS 1.0 (525 steps at that check); pytest 21 (17 + 4 negatives:
    no press, 1 mm stand-off, no overlap, bond = overlap hinges only). A real Isaac Sim 6.1 demo scored 0.3 at 654 steps: the same
    demo that scores 1.0 in MuJoCo did not close the closure in PhysX, so the press pose was still tuned to MuJoCo's settled cuff.
    Not fixed there; recorded so it is not mistaken for a pass.
13. Isaac Sim closure trace (per-step closure state logged from on_step in an instrumented copy): the press does happen in PhysX
    (right fist closed, cuff pushed to -1.9 mm lift) but the modelled pad tip (13 mm past the TCP) plateaus 1.0-1.4 mm from the
    loop-face box for 1.5 s and never under 1 mm: the fingertip collision mesh rests on the face there (cuff colliders: 0.5 mm
    contact offset, 0 rest offset), where MuJoCo's soft contact lets the same point sink to 0.1 mm. Pressing deeper only drives the
    cuff into the arm. So PRESS_CONTACT is 2 mm (= the measured contact resolution, not a stand-off: the old 4 mm rule fired with no
    press at all; the dwell stays 0.3 s). The stand-off negative test is 3 mm off the face. Second finding from the same trace: the
    "smooth it down" stroke after the press dragged the cuff round the arm (mark arc 3 -> 27 mm, over the 20 mm limit), so even an
    engaged cuff would have scored 0.6 in Isaac; the stroke is removed and the press held 4.3 s instead (plan length and the
    780-step limit unchanged). MuJoCo after the change: simulator check PASS 1.0 at 530 steps; pytest 21 passed. Real Isaac Sim
    6.1 demo after the change: 1.0 at 530 steps -- engaged at 23.5 s after 0.3 s of press at 1.7 mm, bond held through lift-off,
    full score from 26.0 s.
14. Second rework, after an independent review of the draft above (changes adopted). (a) The review asked for a contact-FORCE gate
    from the runtime's arm contact sensor (yam_arena.runtime.contact_forces over left_contacts/right_contacts). Checked in the
    running sim: the official runner builds the arms with contact_sensors=False (yam_arena/inspect_embodiment.py:271), so the scene
    has no such entity (scene keys in the Isaac trace: terrain, cuff, left_robot, right_robot, patient_arm, props, cameras, lamp,
    table, stand, housing) and no task code can read a contact force in the scoring path; a force threshold could not be measured
    there, so none is shipped. The gate therefore stays a proximity + dwell gate and is stated as that everywhere: a closed fist
    whose pad tip is within 2 mm of the lying loop face for 0.3 s. 2 mm is where PhysX rests a fingertip that is pushing the cuff
    (item 13: tip 1.0-1.4 mm off the face while the cuff is driven to -1.9 mm lift); it is not a stand-off (3.5 mm and 4.5 mm are
    tested as non-events) and the old rule fired with no press at all. (b) Release-after-closure: the release rule now runs only when
    no hand is on the closure (a hand on it is a press, not a peel), reads peel AND shear from the BONDED hinges' deflection only (an
    earlier Isaac trace released the bond 0.5 s after the fist left because shear read every joint and the passive hinges, bent by
    the press, carry 0.002 N m/rad, not the hold drive), uses the bonded set itself as the peel set, and needs the condition for 2
    control steps. (c) The joint stiffness and target arrays are written only on a state edge (engage, release, first step), not
    every step. (d) Tests: 23 (dwell power, close-then-peel opens / 2 mm lift holds / no release under a fist, both ways).
    (e) README line 16 says 3.5 mm and 4.5 mm. Checks after this rework: pytest 23; MuJoCo 4.0.1 simulator check PASS 1.0 at 530
    steps, every section as before. Cloud datapoint on an L40S (Isaac Sim 6.1.0.0 / Isaac Lab 17.0.2): scripted demo 1.0 at 530
    steps; Dream-1 0 at the 780-step limit with repeated server-side session expiries.
15. Third rework. The official text asks for "actual contact and a realistic press"; the 2 mm proximity + dwell gate above still
    closed for a pad inside 2 mm that pushes nothing. This version adds the press as a STATE quantity (no contact sensor exists in
    the runner, item 14): the lying loop segment under the fist must move >= PRESS_DEPTH = 0.15 mm toward the arm (radius about the
    arm axis) since the pad arrived on it, and hold that with the 2 mm prefilter for the 0.3 s dwell. Measured on both engines with
    the same demo: MuJoCo 4.0.1 press 0.30-0.47 mm sustained (the press pose and poses 2-18 mm higher all end on the cuff and
    press), a brush 25 mm higher peaks at 0.11 mm and closed under the previous gate but not under this one; Isaac Sim 6.1 press
    0.20-0.32 mm sustained for 1.5 s on the instrumented copy, 0.21-0.94 mm on the shipped code (the anchor is taken the step the
    pad arrives). 0.15 mm sits between the brush peak and the Isaac press floor; a hover reads 0 by construction. Shipped-code Isaac
    demo: 1.0, pressed from 23.2 s, engaged 23.5 s, two drive writes in the run, bond held after hand-off, full score 26.5 s.
    Tests: a pad at 1.5 mm with zero displacement never closes (the former stand-off test, now a negative at that distance); the
    same pad with the cuff pushed closes; plus the two regression tests the review asked for (release reads bonded hinges only:
    a passive hinge bent 0.6 rad with no hand on the closure does not open it; drives written on state edges only: 2 writes on the
    first step, 2 on engage, 2 on release, none between). Mannequin: the listing (Amazon B00IFJ9STC, SSWBasics male full-body) gives
    chest 36.5 in and waist 29.5 in and no arm circumference (not found - gap); README/task.json setup and reset say to measure the
    mid-arm circumference on arrival against the cuff's printed 25-34 cm and to change cuff size if outside. Dream-1 record validity:
    the gate script PASSES cloud Dream-1 records that carry "Dream: expired" lines (780 steps, 780 actions, numeric score, full
    episode, max_steps truncation) -- the expiry is the policy re-asking mid-episode, not a broken record.
16. Final text revision. Review finding on the item above: the press decision was written inline at the one call site, so dropping
    its depth term (pressed = bool(hands), the proximity-only semantics of the earlier gate) left every test green -- the wiring was
    untested. This version extracts press_gate(hands, depth) and calls it at that one place; no behaviour change (simulator check
    1.0 at 530 steps, sections unchanged). Test 26, test_press_gate_depth_term_is_load_bearing: the gate refuses no-hand,
    zero-displacement and below-threshold inputs and accepts at the threshold, and the test reads the call site's source so the
    depth-dropping mutant fails (verified: mutant = 1 failed / 25 passed, unmodified = 26 passed).
