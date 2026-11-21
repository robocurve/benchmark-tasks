# Person asset: source and licence

- Model: Microsoft Rocketbox Avatar Library, avatar `Male_Adult_01` (`Assets/Avatars/Adults/Male_Adult_01/Export/Male_Adult_01.fbx`
  and its `_color` textures), https://github.com/microsoft/Microsoft-Rocketbox (commit on `master`, downloaded
  16 Nov 2026). A man in a short-sleeved polo shirt, so the forearms are bare.
- Licence: MIT (`LICENSE-Rocketbox.md`, Copyright (c) 2020 Microsoft). The MIT licence allows modifying and
  redistributing the model, so the posed copy is shipped here.
- Changes: posed once in Blender 4.2 (seated at the end of the table, leaning forward, right arm hanging at his side,
  left hand on the table; see `colleague_proxies.json` "pose"), the skinning baked into a static mesh, the skeleton
  removed, colour textures downscaled to JPEG, normal and specular maps dropped, invisible collision shapes (capsules
  for arms, torso, head and legs; a box for each hand) added, the whole marked as one kinematic (fixed) rigid body.
- Why not NVIDIA's Isaac Sim people (`Isaac/People/Characters`): they are covered by the NVIDIA Isaac Sim Additional
  Software and Materials Licence, which does not allow modifying or redistributing them, and this task has to ship a
  posed static copy.
