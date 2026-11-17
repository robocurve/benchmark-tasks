# bp-cuff-apply build deviations from BUILD-BRIEF.md / SPEC.md (wB1:p1, 2026-11-17)

1. Arm placement: axis 120 -> 100 mm above the table and the arm moved 80 mm toward the robots (cuff centre x 0.33 -> 0.25, crease
   x 0.43 -> 0.35). Why: a straight-down grip cannot reach over the arm top at x 0.33 / axis 120 mm (official IK error 30-80 mm,
   reachgrid at x 0.25/0.33/0.40); at x 0.25 / axis 100 mm it reaches (< 6 mm). Underside still 45-51 mm clear for the cuff.
2. Upper-arm radius 51 -> 45 mm became 55 -> 49 mm (325 mm round the middle, within the cuff's 25-34 cm range; the mannequin
   circumference is unpublished either way). Why: with a 520 mm cuff on a 300 mm arm the free end must end ~230 deg past the start
   end (below the near side, out of reach); the probe ring stayed loose (18-30 mm gaps).
3. Start state: the cuff lies across UNDER the arm (not beside it on the -y side), each 20 mm end piece sprung curled up so a
   down grip can pinch it. Why: a flat cuff on the table cannot be pinched, and passing the free end under the arm needs the
   3-handoff path the stage-1 run flagged; with the cuff under the arm its middle comes up round the arm when the ends are lifted.
4. Demo order: right holds the free end on the table while the left lays the start end over the top; then the right draws the
   free end round, lets it go, and presses the closure with closed fingers. Why: pulling the free end down the near side while
   pinched slipped the pinch (friction-limited) and locked a loose ring; letting go and pressing keeps it snug (max gap 5 mm).
5. Closure gate: "pressed" = a gripper with closed fingers (each finger within 1.5 mm of closed) whose pad tip is within 4 mm of a
   loop segment that already lies on the hook face (brief: pad within 10 mm, or contact normal >= 2 N). Why: with 10 mm the
   pinch holding the free end next to the loop engaged the closure mid-carry and locked a loose ring.
6. Artery arrow on the outer turn (segment 21, 100 mm from the free end), so it shows after wrapping; the artery line is on top.
7. run_limit 740 steps (37 s) = demo plan 28.2 s x 1.3 = 734 steps rounded up to whole seconds (closeloop parses whole seconds).
8. Friction: skin 0.6, cuff 0.7 (estimates). Lower skin friction (0.3) let the closed ring turn round the arm after release.
9. Peer wAA FIX 1-5 (06:41Z) folded: curled end pieces and the under-the-arm start disclosed as staging simplifications
   (task.json setup + fidelity, README Fidelity); 0.3 text "not closed or not snug"; press = pad tip within 4 mm stated in
   task.json; engage gate factored into engages() (same logic) with tests (open fingers, fist 10 mm away, 20 mm overlap -> no;
   closed fist on lying overlap -> yes). Out-of-reach transition waypoints (up to 30 mm, lift-off / park) noted in README.
10. Prereview fix list items 2-3 (07:3xZ, text only): README real-trial checks add a 4 N spring-scale tug on the free end
    (closure stays closed); real staging of the curled end pieces stated (fold ~80 deg by hand, one strip of masking tape on the
    inner side) in README, task.json setup and reset_notes. No code, scene, demo or scorer change.
11. Instruction reworded (COO wAK 13:45Z GO, same fix class as #2196): "Put the blood-pressure cuff on the patient's left upper arm,
    ready for a reading, then let go." -> "Wrap the blood-pressure cuff round the mannequin's left upper arm with its arrow on the marked line
    and its lower edge 2 to 4 cm above the elbow crease, fasten it, then let go." (wAA text delta 13:4xZ: the placement the 1.0 tier
    needs is named; the marked line is the dark artery line on the arm). Title: "patient's" -> "mannequin's".
    Why: Dream API refused the old one (DA-4000 unsafe instruction, "attaching a cuff to a person", probe 13:39Z on abl-warm-32).
    The scene is a display-mannequin arm; real-use context (nurses/medical assistants on patients) stays in task.json context.
    Scorer, scene and demo unchanged; source of truth = build/mk_taskjson.py (also syncs the README quotes).
