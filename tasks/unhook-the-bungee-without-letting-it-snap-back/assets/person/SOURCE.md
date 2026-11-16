# Person asset: source and licence

- Model: Microsoft Rocketbox Avatar Library, avatar `Construction_Male_01` (`Assets/Avatars/Professions/Construction_Male_01/Export/Construction_Male_01.fbx`
  and its `_color` textures: a work shirt and trousers), https://github.com/microsoft/Microsoft-Rocketbox (commit on
  `master`, downloaded 15 Nov 2026).
- Licence: MIT (`LICENSE-Rocketbox.md`, Copyright (c) 2020 Microsoft). The MIT licence allows modifying and
  redistributing the model, so the posed copy is shipped here.
- Changes: posed once in Blender 4.2 (standing across the bench from the robot, feet near x 102 cm, leaning 38
  degrees over it, his right palm flat on the crate's lid beside the cord, left arm hanging; see
  `colleague_proxies.json` "pose"), the skinning baked into a static mesh, the skeleton removed, colour textures
  downscaled to 1024 / 512 px JPEG, normal and specular maps dropped, invisible collision capsules (arms, torso, head,
  legs) added, the whole marked as one kinematic (fixed) rigid body.
- Why not NVIDIA's Isaac Sim people (`Isaac/People/Characters`): they are covered by the NVIDIA Isaac Sim Additional
  Software and Materials Licence, which does not allow modifying or redistributing them, and this task has to ship a
  posed static copy.
