"""Passive review camera and metric guides, separate from policy observations."""
import math
REVIEW_CAMERA = {
    'name': 'review_cam', 'width': 1600, 'height': 1280,
    'eye_m': (.72, -.35, 1.28), 'target_m': (.24, .14, .80),
    'focal_length_mm': 50., 'horizontal_aperture_mm': 20.955,
    'policy_input': False,
}

def add_review_camera(embodiment):
    """Extend a newly built embodiment, before ``yam_arena.scene.make_env``.

    The three calibrated policy cameras stay unchanged. The runner records
    ``base.scene['review_cam']`` separately, never exposing it to Dream.
    """
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg
    from isaaclab_arena.utils.cameras import ArenaCameraCfg
    from isaaclab_arena.utils.configclass import make_configclass
    from pxr import Gf
    from yam_arena.embodiment import RENDERER

    c = REVIEW_CAMERA
    rotation = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*c["eye_m"]), Gf.Vec3d(*c["target_m"]), Gf.Vec3d(0, 0, 1),
    ).GetInverse().ExtractRotationQuat()
    q = (*rotation.GetImaginary(), rotation.GetReal())
    config = CameraCfg(
        prim_path="{ENV_REGEX_NS}/BrickReviewCam",
        height=c["height"], width=c["width"], data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=c["focal_length_mm"],
            horizontal_aperture=c["horizontal_aperture_mm"],
            clipping_range=(0.01, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=c["eye_m"], rot=q, convention="opengl"),
        renderer_cfg=RENDERER,
    )
    # The default config is generated using the same helper. Rebuild its fields
    # so Arena's config traversal includes this sensor as an actual field.
    fields = [(name, CameraCfg, value) for name, value in vars(embodiment.camera_config).items()
              if isinstance(value, CameraCfg) and name != c["name"]]
    fields.append((c["name"], CameraCfg, config))
    embodiment.camera_config = make_configclass(
        "BrickReviewCameraCfg", fields, bases=(ArenaCameraCfg,),
    )()


def install_scene_visuals(base):
    """Add non-colliding metric guides and record their configuration."""
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    material_root = "/World/BrickReviewMaterials"
    UsdGeom.Scope.Define(stage, material_root)

    def material(name, srgb):
        path = f"{material_root}/{name}"
        mat = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        linear = tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in srgb)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    black = material("black", (0.025, 0.025, 0.025))
    white = material("ruler_white", (0.93, 0.93, 0.93))
    red = material("reference_red", (0.9, 0.08, 0.07))

    def mesh(path, vertices, mat):
        item = UsdGeom.Mesh.Define(stage, path)
        item.CreatePointsAttr([Gf.Vec3f(*p) for p in vertices])
        item.CreateFaceVertexCountsAttr([len(vertices)])
        item.CreateFaceVertexIndicesAttr(list(range(len(vertices))))
        item.CreateSubdivisionSchemeAttr("none")
        item.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(item.GetPrim()).Bind(mat)
        # Deliberately no CollisionAPI, RigidBodyAPI or mass/inertia attributes.
        return item

    # Render-only calibration graphics, outside the normal pickup/release area.
    # The tabletop scale has 5 mm ticks and 1 cm major ticks. The upright guide
    # shows 0–20 cm, with a red 3 cm reference. They are visual annotations, not
    # physical obstacles; the run metadata records this simplification.
    from yam_arena.task_file import TABLE_TOP
    top = TABLE_TOP["station"]
    ruler_path = "/World/BrickReviewRulers"
    UsdGeom.Xform.Define(stage, ruler_path)

    def horizontal_rect(name, x0, x1, y0, y1, mat):
        mesh(f"{ruler_path}/{name}", [(x0, y0, top + .00002), (x1, y0, top + .00002),
                                      (x1, y1, top + .00002), (x0, y1, top + .00002)], mat)

    horizontal_rect("TableScale", .14, .42, .275, .292, white)
    for i in range(57):
        x = .14 + .005 * i
        length = .012 if i % 2 == 0 else .007
        # Separate surfaces slightly to prevent depth flicker without collision.
        mesh(f"{ruler_path}/TableTick{i}", [(x-.00025,.276,top+.00004), (x+.00025,.276,top+.00004),
                                          (x+.00025,.276+length,top+.00004), (x-.00025,.276+length,top+.00004)], black)

    def vertical_rect(name, x0, x1, z0, z1, mat, y=.295):
        mesh(f"{ruler_path}/{name}", [(x0,y,z0), (x1,y,z0), (x1,y,z1), (x0,y,z1)], mat)

    vertical_rect("HeightScale", .102, .138, top, top+.207, white)
    for i in range(41):
        z = top + .005 * i
        width = .013 if i % 2 == 0 else .007
        vertical_rect(f"HeightTick{i}", .120, .120+width, z-.0003, z+.0003,
                      red if i == 4 else black, y=.2949)
    vertical_rect("TwentyMillimetres", .120, .157, top+.0195, top+.0205, red, y=.2948)

    # Plain seven-segment numbers give the height guide a readable scale without
    # requiring a font, texture, external file or renderer-specific text API.
    segments = {
        "a": ((0, 1), (1, 1)), "b": ((1, 1), (1, .5)),
        "c": ((1, .5), (1, 0)), "d": ((1, 0), (0, 0)),
        "e": ((0, 0), (0, .5)), "f": ((0, .5), (0, 1)),
        "g": ((0, .5), (1, .5)),
    }
    digits = {"0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg"}

    def glyph_stroke(name, start, end, x, z, mat):
        a = (x + start[0] * .0045, z + start[1] * .008)
        b = (x + end[0] * .0045, z + end[1] * .008)
        dx, dz = b[0] - a[0], b[1] - a[1]
        norm = math.hypot(dx, dz)
        ox, oz = -dz / norm * .0003, dx / norm * .0003
        mesh(f"{ruler_path}/{name}",
             [(a[0]+ox,.2947,a[1]+oz), (b[0]+ox,.2947,b[1]+oz),
              (b[0]-ox,.2947,b[1]-oz), (a[0]-ox,.2947,a[1]-oz)], mat)

    for label, height in (("0", .006), ("2", .02), ("10", .10), ("20", .20)):
        for j, char in enumerate(label):
            for segment in digits[char]:
                glyph_stroke(f"Label{label}_{j}_{segment}", *segments[segment],
                             .105 + .0065 * j, top + height - .004,
                             red if label == "2" else black)
    # Lower-case 'cm' just below the 20 cm label.
    for j, (start, end) in enumerate((((1,1),(0,1)), ((0,1),(0,0)), ((0,0),(1,0)))):
        glyph_stroke(f"UnitC{j}", start, end, .105, top+.181, black)
    for j, (start, end) in enumerate((((0,0),(0,1)), ((0,1),(.5,.5)),
                                     ((.5,.5),(1,1)), ((1,1),(1,0)))):
        glyph_stroke(f"UnitM{j}", start, end, .1115, top+.181, black)

    added = list(Usd.PrimRange(stage.GetPrimAtPath(ruler_path)))
    if any(p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI) for p in added):
        raise RuntimeError('review rulers unexpectedly acquired physics')
    from hand_geometry import geometry_provenance
    return {'version': 'brick-review-v1', 'review_camera': REVIEW_CAMERA,
            'references': {'physical': False, 'tick_spacing_m': .005, 'red_height_above_table_m': .02},
            'robot_collision_envelopes': geometry_provenance(include_all_links=True)}
