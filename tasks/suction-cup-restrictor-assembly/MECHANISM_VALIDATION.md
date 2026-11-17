# Mechanical capture and evidence

The task assembles two separate retaining interfaces through opposite faces of a
plate. The cup head captures the plate, and the core rib must pass and remain
behind the head's inner lips. Pressing until a nominal body origin is close is
insufficient: a tilted head can cross the plate, or the shaft can cross its bore.
The scorer checks these actual surfaces. No arm role or bracing action is scored.

The original-YAM full-score script holds the cup while inserting the core from
the other face. It reaches1.0 in1,060 actions/53s, then releases both parts. The
formal Dream run is0.0 after2,400 actions/120s. Complete original physics records
were replayed with the revised scorer; geometry, reset, instruction, cameras,
controls and action sequences are unchanged. This is not a new model trial.

## Independent component comparisons

A native-physics fixture starts with a cup already captured and the core shaft
partly in its bore, then applies the same slow physical-platen insertion with or
without a backing platen behind the cup. Only fixture platens are kinematic;
parts retain their original passive joints and contact geometry. All platens
withdraw before the final measurement. It has no robot, camera or policy score.

| Backing | Final cap gap | Inner-lip capture |
|---|---:|---|
| Present |0.105mm | All four lips clearly overlap the rib; minimum0.678mm. |
| Absent |0.562mm | One lip is effectively tangent to the rib; the others overlap. |

The unbacked case's smallest computed overlap is approximately-0.00075mm, at
numerical tangency. It is not a robust binary failure example and must not be
used to claim backing is always necessary. The comparison supports a dependence
between cup support and the final core fit for this particular motion. A deeper,
realigned or otherwise successful unbacked strategy remains allowed. Native
constraints pass in both fixtures. The platen loads are diagnostic, not measured
material strengths, YAM force requirements or hardware certification.

A separate earlier matched retention fixture retained the intact ribbed core
under its prescribed0.5N diagnostic pull; removing the physical rib permitted
release. This establishes the role of contact geometry, not a vacuum/flow/load
rating. No proximity-triggered latch, pose update or weld completes either task
interface. Manufacturing counterparts and material approximations are in
fabrication/README.md and PHYSICAL_SETUP.md.
