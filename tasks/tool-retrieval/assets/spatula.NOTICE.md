# Attribution for spatula.usdc

Applies to the blade and handle mesh geometry in `spatula.usdc`.

**Original asset:** NVIDIA `spatula_15`, RoboLab VoMP object collection;
Arena registry name `spatula_15_vomp_robolab`.

**Copyright:** © 2026 NVIDIA Corporation.

**License:** Creative Commons Attribution 4.0 International (CC BY 4.0).
Canonical license: https://creativecommons.org/licenses/by/4.0/
Full legal text: [CC-BY-4.0.txt](CC-BY-4.0.txt), copied from the
[official text](https://creativecommons.org/licenses/by/4.0/legalcode.txt)
with only redundant blank lines at the end removed.

## Source and license evidence

Verified against live primary sources on 2026-11-12:

- [Arena registry at the task's recorded runtime commit](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/isaaclab_arena/assets/object_library.py),
  class `Spatula15VompRobolab`.
- [NVIDIA Isaac 6.1 Arena source USD](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/vomp/spatula_15/spatula_15.usd),
  the download URL recorded by this task's `build_tool.py` and included USD.
  Its layer documentation traces the source to SimReady's
  `common_assets/props/spatula_15/spatula_15_inst.usd`.
- [RoboLab source USD](https://github.com/NVlabs/RoboLab/blob/9cce00cadd5dc575fd2b8ea3836f5bdcde574229/assets/objects/vomp/spatula_15/spatula_15.usd)
  at commit `9cce00cadd5dc575fd2b8ea3836f5bdcde574229`.
- [RoboLab's THIRD_PARTY_NOTICES.md at that commit](https://github.com/NVlabs/RoboLab/blob/9cce00cadd5dc575fd2b8ea3836f5bdcde574229/THIRD_PARTY_NOTICES.md#object-datasets-assetsobjects)
  explicitly gives this asset collection the following notice:

  | Folder | License | Source / Provenance |
  | --- | --- | --- |
  | `vomp` | CC BY 4.0 | © 2026 NVIDIA Corporation. |

RoboLab states that a folder's local license takes precedence. Its repository
tree at the cited commit contains no more specific license under
`assets/objects/vomp/`. The CC BY 4.0 basis here comes from this explicit
asset notice and the geometry comparison below, rather than from Arena's or
RoboLab's Apache-2.0 code license or the license of VoMP software or annotations.
It does not assert a blanket license for NVIDIA's asset catalog or simulator.

## Modifications for this task

`build_tool.py` retains the upstream mesh vertices and face topology, with
source transforms baked into the points, and produces a self-contained USD.
It replaces the source's textured materials with solid display colours,
omits the textures and MDL files, sets subdivision to `none` and double-sided
display, assigns an 80 g rigid-body mass, and replaces collision decomposition
with one convex hull per mesh. The convex hulls fill the small visual slots
for collision. It adds a rigid 35 × 24 × 18 mm grip sleeve centred at local
(-100, 0, 6) mm, with box collision and its own display colour.

These changes adapt a cooking utensil mesh for the retrieval task; they do
not represent an unmodified NVIDIA asset or imply NVIDIA endorsement.

## Redistribution conditions

CC BY 4.0 permits reproduction, sharing and adaptation, including commercial
use. When sharing the licensed geometry, including in modified form:

- Retain supplied creator and attribution information, copyright, the license
  notice and warranty-disclaimer notice, and a source URI where reasonably
  practicable. Identify modifications and retain previous modification notices.
- Identify CC BY 4.0 and include its text or a link. Distribute this notice and
  the accompanying license with copies of this task's `spatula.usdc` to carry
  that information together.
- Do not impose additional terms or effective technological measures that
  restrict recipients' exercise of the licensed rights. An adapter's license
  must allow compliance with CC BY 4.0. The license grants no endorsement,
  patent or trademark rights.

The licensed material is provided as-is and as-available, without warranties,
to the extent permitted by law. See sections 2, 3 and 5 of the full license
for the governing grant, conditions, disclaimer and limitation of liability.
This summary does not replace the license.

## Offline geometry verification

Both source USDs were downloaded for inspection, without running the
simulator or regenerating the task asset. Their file bytes differ, but the
`points`, `faceVertexCounts` and `faceVertexIndices` arrays match exactly for
each corresponding mesh, including the included `spatula.usdc`:

| Upstream mesh | Included mesh | Vertex count | Comparison |
| --- | --- | ---: | --- |
| `/spatula_15/Spatula_15/SpatulaBlade` | `/Spatula/blade` | 27,442 | All three arrays identical |
| `/spatula_15/Spatula_15/SpatulaHandle` | `/Spatula/handle` | 53,906 | All three arrays identical |

Applying each upstream mesh's local-to-world transform also produces exactly
the included points, as in `build_tool.py`. Thus the explicit RoboLab asset
license identifies the same blade and handle geometry redistributed here.

SHA-256 of inspected files:

| File | SHA-256 |
| --- | --- |
| NVIDIA Isaac 6.1 Arena source USD | `4e9a2446af80da2cc703d6c65b0712dd92c374ad9588b8427e4f4ea971bce328` |
| Pinned RoboLab source USD (Git LFS content) | `414996fae8ff1d5d5392c5854b6c977e0e72d31ea752ebac9a444d46727892b2` |
| Included `spatula.usdc` | `ab5df5cf212d3d099a6ef214e9dbcf1ed962f2b73d71d717558b3a8d21f523e0` |
