"""Build the nurse for this task: an adult woman (Anny body model, 166 cm tall standing) in scrubs and nitrile
gloves, seated on a clinic stool (seat 44.5 cm high) at the table's left-hand edge, facing across it, her knees under
the table top, her right forearm resting on the table with the hand open, palm up, and her left arm hanging at her
side. Writes, all in her own frame (feet at z = 0, facing -y, y = 0 at the table's edge, x = 0 her midline, her right
hand toward -x):

    assets/nurse.usda          vertex-coloured mesh with a face, convex-decomposition collider, a kinematic body
    assets/nurse_points.npy    points on her whole body (for the no-contact rules)
    assets/nurse_hand.npy      the vertices of her right (receiving) hand: wrist, palm and fingers
    assets/nurse_mesh.npz      the mesh (for the offline previews)
    assets/nurse_seat.npy      where the stool's seat centre goes under her

Needs the anny, torch, trimesh and usd-core packages; uses the body-model helpers in bodymodel/.
    python make_nurse.py [--search]
(--search repeats the pose search, about 5 minutes; without it the pose found before, POSE, is built.)
"""
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "bodymodel"))
from make_child import F, m, np, rot, torch  # noqa: E402
import build as B  # noqa: E402

PHENO = dict(age=0.667, gender=1.0, height=0.40, weight=0.5, muscle=0.5, proportions=0.5)   # a young adult woman, 166 cm
TABLE = 0.76                      # the table top above the floor
SLAB = 0.06                       # the table top's thickness (nothing under it but the floor)
SEAT = 0.445                      # the stool's seat top above the floor (the stool lowered 11.5 cm from its file's 56 cm)
GAP = 0.06                        # her front (at the table's height) sits this far behind the table's edge
L = m.bone_labels
DOM = np.array(B.dominant())
IS_HAND_R = np.array([b.endswith('.R') and (('wrist' in b) or ('finger' in b) or ('metacarpal' in b)) for b in DOM])
IS_HAND_L = np.array([b.endswith('.L') and (('wrist' in b) or ('finger' in b) or ('metacarpal' in b)) for b in DOM])
IS_FOREARM_R = np.array([b.startswith('lowerarm') and b.endswith('.R') for b in DOM])
IS_ARM_L = np.array([b.endswith('.L') and (('arm' in b) or ('wrist' in b) or ('finger' in b) or ('metacarpal' in b)) for b in DOM])
IS_HEAD = DOM == 'head'
IS_SEAT = np.array([b.startswith(('pelvis', 'upperleg')) for b in DOM])   # buttocks and thighs

# scrubs (ceil blue), light-blue nitrile gloves, white clogs, dark hair: sRGB
SCRUBS, GLOVE, CLOGS, HAIR = (0.33, 0.55, 0.62), (0.55, 0.68, 0.95), (0.95, 0.95, 0.94), (0.16, 0.11, 0.08)


def _aa(axis, deg):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    M = torch.eye(4)
    M[:3, :3] = torch.tensor(np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K, dtype=torch.float32)
    return M


def _run(mats):
    P = torch.eye(4)[None, None].repeat(1, m.bone_count, 1, 1)
    for b, R in mats.items():
        P[0, L.index(b)] = R
    o = m(pose_parameters=P, phenotype_kwargs={k: torch.tensor([x]) for k, x in PHENO.items()})
    return o['vertices'][0].numpy().astype(np.float64), o['bone_poses'][0].numpy().astype(np.float64)


def _joint(bp, b):
    return bp[L.index(b)][:3, 3]


REST_V, REST_BP = _run({})
_S, _E, _W = (_joint(REST_BP, b) for b in ('upperarm01.R', 'lowerarm01.R', 'wrist.R'))
UPPER = (_E - _S) / np.linalg.norm(_E - _S)            # the right upper arm's direction at rest
FORE = (_W - _E) / np.linalg.norm(_W - _E)             # the forearm's direction at rest
HINGE = np.cross(UPPER, FORE) / np.linalg.norm(np.cross(UPPER, FORE))   # the elbow's hinge axis at rest
REST_FLEX = math.degrees(math.acos(UPPER @ FORE))      # the rest pose's elbow bend (41 degrees)


INDEX_R = np.array([b.startswith('finger2-1') and b.endswith('.R') for b in DOM])
PINKY_R = np.array([b.startswith('finger5-1') and b.endswith('.R') for b in DOM])
MIDDLE_R = np.array([b.startswith('finger3') and b.endswith('.R') for b in DOM])
WRIST_R = DOM == 'wrist.R'


def palm_normal(v):
    """The right palm's outward normal, from the plane through the wrist and the index and little fingers' roots
    (for a right hand, (index - wrist) x (little - wrist) points out of the palm)."""
    w = v[WRIST_R].mean(0)
    n = np.cross(v[INDEX_R].mean(0) - w, v[PINKY_R].mean(0) - w)
    return n / np.linalg.norm(n)


_FINGERS = REST_V[MIDDLE_R].mean(0) - REST_V[WRIST_R].mean(0)
WRIST_FLEX = np.cross(palm_normal(REST_V), _FINGERS / np.linalg.norm(_FINGERS))   # the wrist's flexion axis at rest
WRIST_DEV = palm_normal(REST_V)                                                     # its sideways (deviation) axis


def make(p):
    """p = (lean, hip, knee, swing_x, swing_y, swing_z, upper_twist, flex, fore_twist, wrist_flex, wrist_dev, left_down,
    left_back, nod), degrees: the spine leaning in; both hips and knees bent (seated); the right upper arm swung about
    the world axes at the shoulder and turned about its own axis, the elbow bent to ``flex`` about its hinge, the
    forearm turned about its own axis (supination), the wrist bent and turned sideways; the left arm lowered to her
    side and swung back; the head bent forward."""
    lean, hip, knee, sx, sy, sz, tu, flex, tf, wr, wd, ld, lx, nod = p
    mats = {'spine05': rot('x', lean), 'neck01': rot('x', nod), 'head': rot('x', nod),
            'upperleg01.L': rot('x', -hip) @ rot('y', 4), 'upperleg01.R': rot('x', -hip) @ rot('y', -4),
            'lowerleg01.L': rot('x', knee), 'lowerleg01.R': rot('x', knee),
            'upperarm01.R': rot('z', sz) @ rot('y', sy) @ rot('x', sx) @ _aa(UPPER, tu),
            'lowerarm01.R': _aa(HINGE, flex - REST_FLEX),
            'lowerarm02.R': _aa(FORE, tf),
            'wrist.R': _aa(WRIST_FLEX, wr) @ _aa(WRIST_DEV, wd),
            'upperarm01.L': rot('x', lx) @ rot('y', ld), 'lowerarm01.L': rot('x', 10)}
    v, bp = _run(mats)
    v = v - np.array([v[:, 0].mean(), 0.0, v[:, 2].min()])        # feet on the floor
    torso = (v[:, 2] > TABLE - 0.10) & (v[:, 2] < TABLE + 0.15) & (np.abs(v[:, 0]) < 0.12) & ~np.array(
        [('arm' in b) or ('wrist' in b) or ('finger' in b) or ('metacarpal' in b) for b in DOM])
    v[:, 1] -= v[torso, 1].min() - GAP              # y = 0 at the table's edge: her front GAP behind it
    return v, palm_normal(v)


def measure(v, palm, lean=0.0):
    h = v[IS_HAND_R]
    over = v[:, 1] < 0.0
    in_top = over & (v[:, 2] > TABLE - SLAB - 0.01) & (v[:, 2] < TABLE)     # inside the 6 cm top (+1 cm margin)
    above = over & (v[:, 2] >= TABLE)
    fore = over & IS_FOREARM_R
    rear = v[IS_SEAT & np.array([b.startswith('pelvis') for b in DOM]), 1].min() - 0.12
    seat = v[IS_SEAT & (v[:, 1] > rear)]                    # what rests on the seat: buttocks and the thighs' rear
    lh = v[IS_HAND_L]
    return dict(lean=float(lean), palm_up=float(palm[2]), hand_low=float(h[:, 2].min()), reach=float(-h[:, 1].min()),
                hand_back=float(-h[:, 1].max()), hand_x=float(h[:, 0].mean()),
                pen=float(in_top.sum()),                                   # vertices inside the table top
                fore_low=float(v[fore, 2].min()) if fore.any() else 9.0,   # the forearm's underside over the table
                over_body=float((above & ~IS_HAND_R & ~IS_FOREARM_R & ~np.array([b.endswith('.R') and 'arm' in b for b in DOM])).sum()),
                seat=float(seat[:, 2].min()), thigh_top=float(v[IS_SEAT & over, 2].max()) if (IS_SEAT & over).any() else 0.0,
                left_in=float(max(0.0, 0.03 - lh[:, 1].min()) if lh[:, 2].max() > TABLE - SLAB - 0.01 else 0.0),
                left_out=float(lh[:, 0].mean()), top=float(v[:, 2].max()))


def cost_of(q):
    # palm up; the back of the hand resting on the top (its lowest point 3 mm above it) and the forearm resting on it
    # too; the whole hand over the table, the fingertips 45 to 50 cm in (she reaches 42.6 cm); the hand in front of her right shoulder
    # (x -0.10 to -0.26); her buttocks and thighs on the seat (lowest point at SEAT); nothing inside the table top;
    # only her right arm over the table; her left hand at her side; sitting nearly upright (a lean past 12 degrees
    # costs)
    return ((1 - q['palm_up']) * 8 + abs(q['hand_low'] - TABLE - 0.003) * 30 + abs(q['fore_low'] - TABLE - 0.004) * 10
            + max(0.0, 0.45 - q['reach']) * 6 + max(0.0, q['reach'] - 0.50) * 6 + max(0.0, 0.02 - q['hand_back']) * 10
            + max(0.0, q['hand_x'] + 0.10) * 5 + max(0.0, -0.26 - q['hand_x']) * 5
            + abs(q['seat'] - SEAT) * 30 + q['pen'] * 0.02 + q['over_body'] * 0.005 + max(0.0, q['lean'] - 12) * 0.02
            + max(0.0, 0.20 - q['left_out']) * 5 + q['left_in'] * 20)


BOUNDS = [(0, 15), (82, 100), (75, 105), (-90, 40), (-80, 30), (-50, 50), (-60, 60), (10, 120), (-20, 130), (-60, 50), (-25, 25), (30, 80), (-20, 40), (5, 25)]   # anatomical ranges from the rest pose (supination is +fore_twist)


def search(seed=0, restarts=32, iters=700):
    """Random restarts, each refined by a (1+1) evolution strategy with a shrinking step."""
    rng = np.random.default_rng(seed)
    lo, hi = np.array(BOUNDS, float).T
    best = None
    for r in range(restarts):
        p = lo + rng.random(len(lo)) * (hi - lo)
        p[-1], p[1], p[2] = 12.0, 90.0, 90.0
        c = cost_of(measure(*make(p), lean=p[0]))
        sigma = 0.25 * (hi - lo)
        for _ in range(iters):
            t = np.clip(p + rng.normal(size=len(p)) * sigma, lo, hi)
            ct = cost_of(measure(*make(t), lean=t[0]))
            if ct < c:
                p, c, sigma = t, ct, sigma * 1.2
            else:
                sigma = np.maximum(sigma * 0.96, 0.2)
        if best is None or c < best[0]:
            best = (c, [round(float(x), 1) for x in p], None)
        print('restart', r, 'cost', round(c, 4), 'best', round(best[0], 4), flush=True)
    return best


def paint():
    """Linear vertex colours (same vertex order in every pose): scrubs, gloves, clogs, hair and a face."""
    c = B.paint(PHENO['age'], PHENO['height'], linear=False)     # skin, face, hair; clothes recoloured below
    shirt, trousers, shoes = (np.all(np.isclose(c, x), 1) for x in (B.SHIRT, B.TROUSERS, B.SHOES))
    c[shirt | trousers] = SCRUBS
    c[shoes] = CLOGS
    c[np.all(np.isclose(c, B.HAIR), 1)] = HAIR
    c[IS_HAND_R | IS_HAND_L] = GLOVE
    return np.power(c, 2.2)


# The pose the search found (see make())
POSE = [4.0, 82.0, 95.1, -50.6, -4.1, 46.5, 12.2, 20.3, 99.7, -5.3, 4.2, 30.5, -0.5, 7.9]

if __name__ == '__main__':
    if '--search' in sys.argv or POSE is None:
        cost, POSE, q = search()
        print('search: pose', POSE, 'cost', round(cost, 4))
    v, palm = make(POSE)
    q = measure(v, palm, lean=POSE[0])
    print('pose', POSE, {k: round(x * 100, 1) if k not in ('palm_up', 'edge') else round(x, 3) for k, x in q.items()},
          'cost', round(cost_of(q), 4))
    c = paint()
    out = HERE / 'assets'
    rear = v[IS_SEAT & np.array([b.startswith('pelvis') for b in DOM]), 1].min() - 0.12
    seat = v[IS_SEAT & (v[:, 1] > rear) & (v[:, 2] < SEAT + 0.02)]
    stool_xy = np.array([seat[:, 0].mean(), seat[:, 1].mean()])            # the seat's centre under her buttocks
    np.savez(out / 'nurse_mesh.npz', v=v.astype('f4'), f=F.astype('i4'), c=c.astype('f4'), palm=palm)
    np.save(out / 'nurse_seat.npy', stool_xy.astype('f4'))
    print('stool seat centre (her frame)', stool_xy.round(3))
    if '--preview-only' in sys.argv:
        raise SystemExit
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
    st = Usd.Stage.CreateNew(str(out / 'nurse.usda'))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, '/Nurse')
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(st, '/Nurse/body')
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
    pts, _ = trimesh.sample.sample_surface(tm, 6000, seed=0)
    hand = v[IS_HAND_R]
    np.save(out / 'nurse_points.npy', np.concatenate([pts, hand[::2], v[IS_HAND_L][::4]]).astype('f4'))
    np.save(out / 'nurse_hand.npy', hand.astype('f4'))
    print('size cm', ((v.max(0) - v.min(0)) * 100).round(1),
          '| right hand cm x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % tuple(np.ravel([[hand[:, i].min() * 100, hand[:, i].max() * 100] for i in range(3)])),
          '| palm normal', palm.round(3), '| points', len(pts) + len(hand[::2]) + len(v[IS_HAND_L][::4]))
