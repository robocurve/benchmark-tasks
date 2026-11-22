# Asset provenance

The panel, paired lift keys, guide, base, mannequin, joints and materials are authored procedurally in fixture.py and build_asset.py. The public task includes their generated USD. It references the installed YAM station at runtime. The station observation bridge in scenekit.py is adapted from the benchmark repository's lock-before-loading task. The independent reproduction archive separately carries the YAM diagnostic meshes and their accompanying license under diagnostics/yam_meshes. Those meshes are outside the submitted task directory.
