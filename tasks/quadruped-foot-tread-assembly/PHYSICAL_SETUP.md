# Fixed custom robot-foot subassembly

This is a custom benchtop robot-foot installation specimen, not a complete robot,
scaled quadruped, Spot replacement part, or walking-load qualification. The rigid
foot is already supported in an inverted service posture. No robot mass is being
made artificially liftable. All geometry is generated locally in assembly/geometry.py;
task.py specifies the staging stands. Dimensions are design values, not OEM measurements.

## Parts and material model

- Fixed aluminum-like foot envelope54x38x12mm: centre block50x32x12; four ears
  2x7x12 centred at local(+/-26,+/-15.5,-8.5)mm. Core datum world(0.24,0,0.8915)m,
  frame rotated90deg about worldZ. A35x26x121mm leg support and130x100x12mm base
  secure it to the table. Exact centres are in geometry.py.
- Blue elastomer-like cover: rigid64x48x5mm sole,25g, four3g wrap corners. Each
  corner has a3x14x13mm side and a14mm-wide extruded bevel/hook. Full vertices
  and26mm guide skirts are specified in geometry.py. Four passive Y hinges have
  stiffness0.001N*m/deg, damping0.00005N*m*s/deg and25deg opening travel.
- Two removable18g clamp bodies:4x48x16mm outer wall, stepped0.5mm inner lands,
  top bridge and10x40x2mm return lip. Each carries two2g steel-like sliding end
  clips with a tapered nose. Each clip has3mm outward travel,500N/m spring and
  0.1N*s/m damping. The complete nose vertices, clip arms and attachment frames
  are specified in geometry.py. These are spring quick clamps, not M3 screws.
- Fixed staging:30x30x75mm pad pedestal and four4x10x154.5mm bar-cradle posts.
  Locations are in task.py and match the original-camera setup image.

All added surface friction coefficients are0.6 static/0.48 dynamic, restitution0.
The model uses rigid bodies and discrete springs instead of continuous rubber or
metal bending. Elastic forces are not calibrated against manufactured specimens;
these parameters define the simulation approximation, not a measured material
grade. Continuous-part self-contact is omitted, while separate-piece/robot/core
contacts remain physical. The actual geometry and mechanical retention remain.

## Reset and controls

Cover origin(0.29,-0.18,0.825)m, orientation R. Bar origins(0.38,+/-0.095,0.91)m,
orientation R. All spring coordinates/velocities start at zero. Original station
READY joint state, fixed scene, no object jitter. Each trial starts in a fresh
process. For a physical specimen use equivalent hinges/springs or a manufactured
compliant part validated to give the same travel/retention; do not treat the
surrogate as a certified elastomer formulation. Replace damaged specimens.

Original dual YAM geometry, drive parameters, limits and three cameras are retained.
Actions20Hz; PhysX480Hz/TGS,192 position and8 velocity iterations for added parts.
The complete collision/contact settings are in assembly/native.py and runtime.py.
The exact same reset, task and scorer are used for demo and model.

## Fabrication files

fabrication/ contains millimetre STL files, orthographic dimension drawings,
parts.json, and a step-by-step assembly and inspection guide. The rigid contact
interfaces are derived from the recorded scene. The manufacturing counterpart
adds the connections needed to make the simulated compliant body: connected
flexible roots or captive sliders/springs, as specified in the guide. These
construction details are distinct from the collision approximation; the files
are not claimed to be a measured OEM or material-identical reproduction.
