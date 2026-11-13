# Asset sources and licensing

| Asset | Source | License / terms |
|---|---|---|
| YAM robot meshes, URDF and collision meshes | I2RT Robotics i2rt; supplied through the program YAM environment | MIT; see licenses/YAM_MIT.txt |
| Apple apple_01 | RoboLab assets/objects/objaverse/apple_01, loaded through Arena as apple_01_objaverse_robolab | CC BY-NC-SA 4.0; copyright NVIDIA Corporation; see licenses/ROBOLAB_THIRD_PARTY_NOTICES.md |
| Orange orange_02 | RoboLab assets/objects/fruits_veggies/orange_02, loaded through Arena as orange_02_fruits_veggies_robolab | CC BY-NC-SA 4.0; copyright NVIDIA Corporation; see licenses/ROBOLAB_THIRD_PARTY_NOTICES.md |
| Camera housing, lens panel, sliding shoe, channel, spring-tab representation, connector struts | Procedurally specified in this task | MIT for task-authored geometry; see LICENSE; no third-party mesh |
| Table, floor, camera stand | Primitive geometry from the program's YAM environment | No separately downloaded texture or object mesh; dependency license status below |

Upstream asset inventory and notices: https://github.com/NVlabs/RoboLab/blob/main/THIRD_PARTY_NOTICES.md . Assets remain external dependencies; this submission does not redistribute their USDs or textures. Do not assume Arena's code license applies to its assets.

YAM environment dependency: the program-provided YAM environment (source locator in dependencies.py) at commit bd62771e60320433893bd1d2df0fc79b5ddcd6d1. Its README explicitly says the wrapper repository has no license yet, while the bundled I2RT model is MIT. We do not assign it an invented license or vendor the wrapper. Isaac Lab-Arena code is Apache-2.0; Isaac Sim is subject to NVIDIA's own software terms. The official environment's dependency status is disclosed rather than represented as all-MIT.

CC BY-NC-SA 4.0 legal terms: https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode.en . The upstream notice assigns this license at dataset-folder level; individual license files were not present in the queried folders.

The task-specific MIT grant excludes external assets and environment dependencies. RoboCurve’s guide expressly permits third-party non-commercial assets when their license permits research use and redistribution; their notices must remain visible.
