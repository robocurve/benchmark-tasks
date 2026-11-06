# Asset provenance

The table and task objects are local authored USD assets. The fish appearance uses
the supplied generated raster texture `assets/textures/fish_skin_generated.png`;
its generation prompt is retained in `assets/textures/fish_texture_prompt.txt`.
Procedural fish-scale and bamboo textures are also included.

The YAM robot geometry derives from the provided YAM station. The bundled split
black/white link meshes originate in the Dream examples' YAM MuJoCo assets and
are attributed to I2RT Robotics. The inherited source note and mesh checksums are
in `assets/base/DREAM_MESH_SOURCE`. The I2RT MIT copyright and license are
preserved in [LICENSE.i2rt](assets/base/LICENSE.i2rt).

The source station's calibration is retained in `assets/base/station_layout.json`.
The live runtime uses the separately installed YAM/Arena embodiment and its
original cameras, lighting, action semantics and actuator setup. Isaac Sim,
Isaac Lab, Arena, Inspect, the Dream SDK, and the supplied Docker image are
external runtime prerequisites and are not redistributed in this folder.
