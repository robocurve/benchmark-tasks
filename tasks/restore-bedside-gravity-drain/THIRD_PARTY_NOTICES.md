# Asset credits and use

| Asset in this task or its runtime | Provenance and terms |
|---|---|
| Four trainer scenes, manifold, fixture, mannequin, colored sights and receiver | Task-authored USD primitives, curves, authored mesh vertices and materials. No patient data, commercial device scans, clinical illustrations or clinical device CAD are used. |
| Bedside order image | Task-rendered lettering in DejaVu Sans Bold. DejaVu changes are public domain. The underlying Bitstream Vera fonts carry the Bitstream Vera license. The PNG is distributed as a raster image. [Font project and license](https://dejavu-fonts.github.io/License.html) |
| `assets/link_bounds.json`, `assets/gripper_hulls.json` | Bounds of the YAM collision geometry and its gripper convex vertices. Derived from the I2RT MIT-licensed geometry. The source URDF checksum embedded in the bounds matches the runtime model. Full upstream MIT notice appears below. |
| YAM robot, converted USD, visual STL and collision STL | Installed with `yam-arena` 0.1.0. The wrapper records I2RT revision `120c3c81400171174604e503943f8d1ebc891058`. The historical Inspect 1.0.0 invocation records the URDF and USD entry file hashes. This audit matched both against YAM checkout `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`. The upstream robot code and assets have distinct licensing histories. The MIT notice below applies to the I2RT model. [YAM runtime snapshot](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam) · [I2RT source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) |
| Split Dream arm appearance | Runtime files `link2_black.stl`, `link2_white.stl`, `link3_black.stl`, `link3_white.stl`. The runtime identifies them as I2RT mesh derivatives from the dreamlabs 1.0.23 wheel. Verified individually against its four recorded SHA-256s. Geometry carries the same MIT notice. This attribution is specific to the four meshes. [Runtime provenance](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/dream/SOURCE) |
| Station, cameras and environment | Runtime-provided YAM station transforms, table, lighting and camera fixtures. The task overrides its camera mounts. The water trainer loads the seven files in `assets/`. It uses the station room with task-authored objects. The robot mesh, font binary, Dream model, SDK, Isaac assets and platform libraries are supplied separately by their respective runtimes. |
| Physical build files | Original water-trainer designs for this task. The permission grant below covers the new original CAD, drawings, and physical construction instructions. Manufacturer and product names identify compatible runtime or training hardware. |

The asset inventory lists relative paths and hashes for the URDF-referenced geometry, the converted USD package, the split appearance meshes and their source notices. Only the two derived JSON geometry files ship with the task. [Machine-readable YAM inventory](validation/yam_asset_inventory.json) · [Bundled task source hashes](validation/source_hashes.json)

## I2RT model notice

MIT License

Copyright (c) I2RT Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## New original trainer CAD, drawings and physical instructions

MIT License

Copyright (c) 2026 Water trainer contributors

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
