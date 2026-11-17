# Fixed custom vacuum-gripper mounting specimen

This custom plug-in cup and restrictor are not Schmalz replicas. They preserve
head compression through a mounting hole, elastic recovery and separate ribbed
core retention. Dimensions are design values. Assembly ends before sealing,
vacuum generation, airflow verification or robotic payload handling.

## Parts and material model

- Fixed aluminum-like plate:110mm OD,4mm thickness,18mm true through hole; one
  1mm-deep45deg entry chamfer opens to20mm. Hole centre(0.24,0,0.89)m,axis worldY.
  A30x18x110mm pillar and140x80x12mm base support it. The plate is a single static
  concave annulus mesh with an actual open bore, not overlapping solid blocks.
- Green18g rigid hollow bellows:40mm outer envelope,16.6mm neck OD/9.2mm ID,
  22mm OD lower shoulder. Four1g tapered fingers form a21mm maximum head and
  15.6mm entry-tip envelope. Capture faces at localY21.4 and26mm span the plate.
  Each finger has one convex collider, passive180N/m radial spring,0.8N*s/m
  damping and-3..+1.5mm travel. Finger coordinates are in assembly/geometry.py.
- White POM-like1.2g restrictor:8.8mm shaft OD,10.6mm retention rib,13mm cap,
  0.8mm through bore. The collision profile runs from localY-24 to+1.5mm; a
  total25.5mm axial extent.
  Taper, shaft, rib and cap profiles are specified in geometry.py.
- Fixed supports:22x24x132.5mm cup pedestal;8x2x146mm rear and4x2x148.1mm front
  core supports. Exact centres are in task.py and the setup screenshot.

All added surfaces use0.6 static/0.48 dynamic friction and zero restitution.
Bellows flexibility, continuous head deformation, sealing, flow and material
damage are omitted. Discrete springs model local compliance; their force curve
is not measured silicone/POM behavior. The continuous head's internal contacts
are filtered, but all plate/core/robot contacts remain physical. No pose updates,
goal-triggered latch, object weld or active force completes the installation.

## Reset and numerical settings

Cup origin(0.38,-0.09,0.90)m;core origin(0.38,0.08,0.90)m;identity rotations and
zero passive-joint displacement/velocity. Fixed supports, original station READY,
no jitter. Start a fresh process for each trial. Physical specimens must have
matching capture geometry and measured compliant behavior before load/seal claims;
replace damaged components instead of assuming simulation reset restores material.

Original calibrated YAM bodies, drive settings, limits and cameras are unchanged.
Actions20Hz; PhysX960Hz/PGS with articulation contact-last enabled; added assembly
128 position/1 velocity iteration,0.5mm friction correlation,0.2mm friction offset,
0.05mm contact offset. The cup/head is a freely moving native articulation with
four passive radial DOFs, not attached to the robot or world. Exact constants are
in assembly/native.py and runtime.py. Model and demo share them.

## Fabrication files

fabrication/ contains millimetre STL files, orthographic dimension drawings,
parts.json, and a step-by-step assembly and inspection guide. The rigid contact
interfaces are derived from the recorded scene. The manufacturing counterpart
adds the connections needed to make the simulated compliant body: connected
flexible roots or captive sliders/springs, as specified in the guide. These
construction details are distinct from the collision approximation; the files
are not claimed to be a measured OEM or material-identical reproduction.
