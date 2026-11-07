# Related mechanisms and intended distinction

This task does not claim invention of ring-on-wire manipulation. Buzzwire robotic
benchmarks already exist, including the AIRA2024 challenge's task7 and the REEM-C
trajectory-optimization study. Our proposed distinction is an80 mm deep opaque
collar whose internal clearance is viewed through an opening that turns during
travel, using the robot's existing wrist cameras. It is a test of maintaining useful
observations during precision handling, not a required camera-motion sequence.

- AIRA task7: https://aira-challenge.com/2024AiraChallenge_Official_Rules_2023Nov.pdf
- REEM-C study: https://pmc.ncbi.nlm.nih.gov/articles/PMC9203844/

Closest RoboCurve comparisons, inspected2026-11-07:

| Task | Shared aspect | Difference claimed here |
|---|---|---|
|#841 casting ring over a wax crown|Opaque sleeve and protected hidden part; strong overlap|Single straight lowering after ring selection versus carrying an already-threaded deep collar around a curved path with changing inspection direction.|
|#69 rod deep into a box|Partially observed collision avoidance|One depth decision versus maintaining an internal clearance across a turning path.|
|#739 hidden locating dowel|Workpiece causes occlusion|Contact search is intended there; here protected contact leaves failure evidence.|
|#575 concealed service slider|Available wrist views and occluded precision; strong conceptual overlap|A fixed inspection opening/one-axis mark alignment versus a moving, turning occluding collar.|
|#802 sheath/guidewire|Threaded coaxial parts|Coupled sliding and wire retention; no contact-free opaque curved-guide transfer.|
|#778 messenger wire/ring|Ring on wire|Wire tension changes placement; contact with the wire is intended.|
|#819 camera commissioning|Inspection optics|Setting a separate camera's lens controls, not moving native wrist views during handling.|
|#539 split cable guide|Guide enclosure|Installing split halves and testing drawer travel, not moving a closed collar along a guide.|
|#369 collar puller|Collar manipulation|Threaded tool extraction/recovery rather than forbidden internal contact.|
|#120 concealed travel|Occluded fixture inspection|Measure three travel ranges and identify the largest; no ongoing turning opening.|
|#551 covered indicator memory|Occlusion|Remember a previously visible cue rather than maintain a current view.|

PR links use https://github.com/robocurve/benchmark-tasks/pull/NUMBER.
The RoboDojo published task catalog was also checked; no exact deep opaque
curved-guide task was identified in its listed entries:
https://robodojo-benchmark.com/doc/sim-tasks/

These are mechanism comparisons, not official novelty clearance or an exhaustive
source-code/video review of every benchmark. #841, #575 and conventional buzzwire
are material overlap risks. A successful known-layout script and an image gate do
not prove that ongoing camera movement is necessary for every valid strategy.
