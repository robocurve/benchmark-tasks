# keep-a-station-stocked-for-a-timed-arm build deviations (2026-11-21)

1. **The coworker overrides the right half of every action.** The task needs a second arm on a fixed clock that the
   policy cannot steer. `coworker.py` wraps `YamArenaEmbodiment.step` (patched in memory on import, never on disk) and
   replaces action entries 7..13 (right joints + grip) with the routine's joint targets at the embodiment's own clock
   (`self._t`, seconds since reset). The routine is rebuilt at every reset and reads nothing from the scene, so it moves
   the same way in every trial whatever the policy sends to the right arm. Offline checkers that do not import
   `yam_arena.inspect_embodiment` play `demo()`, whose right-arm plan is the same routine (`coworker_plan()`).

2. **Arm touch is a proxy, latched for the whole trial.** "The arms touched" is judged from body origins (link 4,
   link 5, gripper, both fingertips) within `TOUCH = 50 mm`, checked after every control step by a `labkit.on_step`
   hook, not from contact reports. A near miss can count; a brush between two link surfaces whose origins stay over
   50 mm apart is not seen. The flag is held in the task module (`_WATCH`), reset on every environment reset, and once
   set keeps the score at 0.0 for the rest of the trial.

3. **Block mass is an estimate.** The Uxcell listing gives the size (1.2 in = 30.5 mm) but no mass. 16 g comes from
   28.4 cm3 x ~0.55 g/cm3 (typical dry pine/birch density). A real trial should weigh one block and update `CUBE_M`.

4. **Tray and crate are square thin-walled boxes.** The weigh boat's walls slope outward and the organizer bin has
   rounded corners and a lid; both are modelled as straight 3 mm walls at the listed outside sizes (89 x 89 x 25 mm
   and 152 x 152 x 51 mm), with the bin used open. The weigh-boat inside is taken as 83 mm (outside minus two walls).

5. **Crate drop spots.** The coworker lets go over a different spot of a 3 x 3 grid at 40 mm pitch on each visit (8 of
   the 9 spots) so the blocks lie side by side on the crate floor; an earlier build that dropped all blocks in the
   middle stacked them above the rim and the stack toppled in the GPU simulator.

6. **No mat.** The blocks start directly on the station table (no separate mat part), because no listed mat was chosen.
