"""Build the toddler for this task: a girl of about 3 (Anny body model, 96 cm, about the median height at age 3) standing on the floor at the table's
side edge, facing into the table, both hands resting on the table top. Builds her pose (optionally searching it
again), then writes
assets/toddler.usda (vertex-coloured mesh with a face, convex-decomposition collider, a kinematic body), assets/toddler_points.npy
(points on her body for the distance rules) and assets/toddler_hands.npy (points on her hands), all in her own frame:
feet at z = 0, facing -y, y = 0 at the table's edge (her chest 4 cm behind it), x = 0 her midline.

Needs the anny, torch, trimesh and usd-core packages; uses the body-model helpers in bodymodel/ (make_child.py, build.py).
    python make_toddler.py [--search] [preview.png]
(--search repeats the pose search, about 10 minutes; without it the pose found before, POSE, is built.)
"""
import itertools
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "bodymodel") if (HERE / "bodymodel").is_dir() else str(HERE.parent / "people"))
from make_child import *  # noqa: E402,F401,F403  (anny model m, rot, F, torch, np)
import build as B  # noqa: E402

AGE, HEIGHT = 0.13, 0.5             # Anny's toddler phenotype: 96 cm tall (about the median for age 3)
TABLE = 0.76                        # the table top above the floor (the station's top 0.7475, floor -0.0125)
L = m.bone_labels
dom = np.array(B.dominant())
HAND = np.array([('wrist' in b) or ('finger' in b) or ('metacarpal' in b) for b in dom])
ARM = np.array([('lowerarm' in b) for b in dom])
SIDE_L = np.array([b.endswith('.L') for b in dom])
HEAD = dom == 'head'


def make(uz, uy, ux, ly, lz, lean, nod=-10):
    """Upper arms swung forward (uz) and raised (uy) about the shoulder, turned (ux); forearms bent (ly, lz); the
    spine leaning in (lean); the neck and head bent forward (nod, degrees each). The right arm mirrors the left."""
    mats = {'spine05': rot('x', lean), 'neck01': rot('x', nod), 'head': rot('x', nod),
            'upperarm01.L': rot('z', uz) @ rot('y', uy) @ rot('x', ux),
            'upperarm01.R': rot('z', -uz) @ rot('y', -uy) @ rot('x', ux),
            'lowerarm01.L': rot('y', ly) @ rot('z', lz), 'lowerarm01.R': rot('y', -ly) @ rot('z', -lz)}
    P = torch.eye(4)[None, None].repeat(1, m.bone_count, 1, 1)
    for b, R in mats.items():
        P[0, L.index(b)] = R
    ph = dict(age=AGE, gender=0.5, height=HEIGHT, weight=0.5, muscle=0.5, proportions=0.5)
    v = m(pose_parameters=P, phenotype_kwargs={k: torch.tensor([x]) for k, x in ph.items()})['vertices'][0].numpy()
    v = v - [v[:, 0].mean(), 0, v[:, 2].min()]
    chest = (v[:, 2] > TABLE - 0.12) & (v[:, 2] < TABLE - 0.02) & (np.abs(v[:, 0]) < 0.07) & ~HAND & ~ARM
    v[:, 1] -= v[chest, 1].min()        # y = 0 at the chest's front just below the table top
    return v.astype(np.float64)


def measure(v, gap):
    """``gap``: the chest stands this far back from the table edge (the table is y < -gap)."""
    over = v[:, 1] < -gap
    h = v[HAND]
    body = over & ~HAND
    return dict(hand_low=h[:, 2].min(), reach=-h[:, 1].min() - gap, hand_back=-h[:, 1].max() - gap,
                span=h[:, 0].max() - h[:, 0].min(),
                left_x=v[HAND & SIDE_L, 0].mean(), right_x=v[HAND & ~SIDE_L, 0].mean(),
                pen=max(0.0, TABLE - v[body, 2].min()) if body.any() else 0.0,
                hand_pen=max(0.0, TABLE - h[h[:, 1] < -gap, 2].min()) if (h[:, 1] < -gap).any() else 0.0,
                top=v[:, 2].max(), head_back=v[HEAD, 1].mean() + gap)


def cost_of(q):
    # hands resting on the top (lowest hand point 3 mm above it), wholly over the table, about 10-20 cm in;
    # no other part of her below the top over the table; hands about 30 cm apart (outer edges)
    # (left hand on her left, +x, the right on her right, each 8-14 cm from her midline: not crossed)
    side = sum(max(0.0, 0.08 - x) + max(0.0, x - 0.14) for x in (q['left_x'], -q['right_x']))
    return (abs(q['hand_low'] - TABLE - 0.003) * 20 + q['pen'] * 20 + max(0.0, 0.10 - q['reach']) * 4
            + max(0.0, q['reach'] - 0.20) * 4 + max(0.0, -q['hand_back']) * 3 + side * 10
            + max(0.0, q['head_back'] - 0.03) * 8)     # leaning in: her head over the edge, in the camera's view


def search():
    best = None
    grid = itertools.product(range(-100, -29, 14), range(-60, 11, 14), (-30, 0, 30), (-40, 0, 40), (-40, 0, 40), (0, 10), (-10, 5, 20))
    for args in grid:
        v = make(*args)
        for gap in (0.0, 0.02, 0.04):
            q = measure(v, gap)
            c = cost_of(q)
            if best is None or c < best[0]:
                best = (c, args + (gap,), q)
    for step in (7, 3):                  # refine around the best, one parameter at a time
        improved = True
        while improved:
            improved = False
            for i in range(7):
                for d in (-step, step):
                    a = list(best[1][:7]); a[i] += d
                    v = make(*a)
                    q = measure(v, best[1][7])
                    c = cost_of(q)
                    if c < best[0] - 1e-6:
                        best, improved = (c, tuple(a) + (best[1][7],), q), True
    return best




# The pose the search found (uz, uy, ux, ly, lz, lean, nod, gap), with the nod eased from 20 to 12 degrees so that she
# looks across the table rather than straight down at it.
POSE = (-30, -60, -26, -37, -40, 0, 12, 0.04)

if __name__ == '__main__':
    if '--search' in sys.argv:
        sys.argv.remove('--search')
        cost, args, q = search()
        print('search: pose uz uy ux ly lz lean nod gap', args, 'cost', round(cost, 4))
    args = POSE
    q = measure(make(*args[:7]), args[7])
    print('pose uz uy ux ly lz lean nod gap', args, {k: round(float(x) * 100, 1) for k, x in q.items()}, 'cost', round(cost_of(q), 4))
    gap = args[7]
    v = make(*args[:7])
    v[:, 1] += gap                       # y = 0 is now the table's edge; the chest stands `gap` behind it
    c = B.paint(AGE, HEIGHT)             # clothes, hair and a face (eyes, brows, lips), linear colours
    out = HERE / 'assets'
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
    st = Usd.Stage.CreateNew(str(out / 'toddler.usda'))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, '/Child')
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(st, '/Child/body')
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype('f4')))
    mesh.CreateFaceVertexCountsAttr([3] * len(F))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.astype('i4').ravel()))
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateExtentAttr([Gf.Vec3f(*v.min(0).tolist()), Gf.Vec3f(*v.max(0).tolist())])
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('displayColor', Sdf.ValueTypeNames.Color3fArray,
                                            UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray.FromNumpy(c.astype('f4')))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexDecomposition')
    st.GetRootLayer().Save()
    import trimesh
    tm = trimesh.Trimesh(v, F, process=False)
    pts, _ = trimesh.sample.sample_surface_even(tm, 4000, seed=0)
    hands = v[HAND]
    np.save(out / 'toddler_points.npy', np.concatenate([pts, hands[::2]]).astype('f4'))
    np.save(out / 'toddler_hands.npy', hands.astype('f4'))
    np.savez(out / 'toddler_mesh.npz', v=v.astype('f4'), f=F.astype('i4'))
    head = v[dom == 'head']
    nose = head[head[:, 1].argmin()]
    print('size cm', ((v.max(0) - v.min(0)) * 100).round(1), '| nose cm', (nose * 100).round(1),
          '| hands cm x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % tuple(np.ravel([[hands[:, i].min() * 100, hands[:, i].max() * 100] for i in range(3)])),
          '| points', len(pts) + len(hands[::2]))
    if len(sys.argv) > 1:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        cs = np.power(c, 1 / 2.2)
        fig = plt.figure(figsize=(15, 5))
        for i, (el, az, t) in enumerate([(5, 0, 'side'), (20, -90, 'front (from the table)'), (60, -120, 'above')]):
            ax = fig.add_subplot(1, 3, i + 1, projection='3d')
            tri = v[F]
            nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            nrm /= (np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-9)
            sh = (0.45 + 0.55 * np.clip(nrm @ np.array([0.3, -0.6, 0.7]), 0, 1))[:, None]
            ax.add_collection3d(Poly3DCollection(tri, facecolors=cs[F].mean(1) * sh, edgecolor='none'))
            tx, ty = np.meshgrid([-0.3, 0.3], [-0.35, 0.0])
            ax.plot_surface(tx, ty, np.full_like(tx, TABLE), color=(0.8, 0.65, 0.45), alpha=0.35)
            ax.set_xlim(-0.4, 0.4); ax.set_ylim(-0.5, 0.3); ax.set_zlim(0, 1.0)
            ax.view_init(el, az); ax.set_axis_off(); ax.set_title(t)
        plt.tight_layout(); plt.savefig(sys.argv[1], dpi=80)
