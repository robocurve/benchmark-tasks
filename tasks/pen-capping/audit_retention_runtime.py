"""GPU runtime geometry/reset diagnostic. NOT an insertion demo or force test.

Run only when the parent schedules the verified live GPU. No task objects are
moved or trials launched by importing this module.
"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scheduled', action='store_true', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from inspect_robots.scene import Scene
    from pen_capping.embodiment import PenEmbodiment
    e = PenEmbodiment(task=str(Path(__file__).with_name('task.py')), device='cpu',
        top_cam='demo', object_jitter=0, control_hz=20, cam_width=640, cam_height=360)
    scene = Scene(id='liner-geometry-reset-audit', instruction='Fit each pen with a cap of the same color. Put the two red pens on the left and the two blue pens on the right.')
    try:
        e.reset(scene, seed=0)
        from pxr import UsdPhysics, UsdGeom, Gf, Sdf
        stage = e._base.sim.stage
        assets = {}
        for i in range(4):
            for kind, expected in [('pen', 4), ('cap', 49)]:
                name = f'{kind}_{i}'
                root_path = f'/World/envs/env_0/{name}'
                root = stage.GetPrimAtPath(root_path)
                children = [p for p in stage.Traverse() if str(p.GetPath()).startswith(root_path + '/')]
                colliders = [p for p in children if p.HasAPI(UsdPhysics.CollisionAPI)]
                assert len(colliders) == expected, (name, len(colliders))
                if kind == 'cap':
                    liner = [p for p in children if p.GetName().startswith(('LinerGrip', 'LinerLeadIn'))]
                    assert len(liner) == 128
                    assert all(not p.HasAPI(UsdPhysics.CollisionAPI) for p in liner)
                    assert all(UsdGeom.Imageable(p).ComputeVisibility() != 'invisible' for p in liner)
                assets[name] = dict(mass_kg=UsdPhysics.MassAPI(root).GetMassAttr().Get(),
                    colliders=[str(p.GetPath()) for p in colliders])
        assert not e._pairs and e.task.module.HISTORY.score() == 0
        # Synthetic joints at actual initial poses exercise cleanup only. Do not
        # call _retain: these pairs are uninserted and cannot pass its fit gate.
        poses, rots = e._read()
        from yam_arena.ik import matrix_to_quat_xyzw
        for i in range(4):
            pn, cn = f'pen_{i}', f'cap_{i}'
            relative = rots[pn].T @ (poses[cn][:3] - poses[pn][:3])
            q = matrix_to_quat_xyzw(rots[pn].T @ rots[cn])
            joint = UsdPhysics.FixedJoint.Define(stage, f'/World/envs/env_0/CapRetention{i}_{i}')
            joint.CreateBody0Rel().SetTargets([Sdf.Path(f'/World/envs/env_0/{pn}')])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(f'/World/envs/env_0/{cn}')])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, relative)))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3]))))
            joint.CreateLocalRot1Attr(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
            joint.CreateExcludeFromArticulationAttr(True)
            e._pairs[i] = i
        e._backing_contacts.record(['pen_0/Barrel','cap_0/LinerBacking00'],'found',e._t,[])
        assert e._backing_contacts.snapshot(e._t)
        e.reset(scene, seed=0)
        assert not e._backing_contacts.snapshot(e._t)
        assert not e._backing_contacts.drain()
        assert not e._joint_reference
        assert not e._pairs and not e._insertions and not e._contacts_seen
        assert e.task.module.HISTORY.score() == 0
        assert all(not stage.GetPrimAtPath(f'/World/envs/env_0/CapRetention{i}_{i}') for i in range(4))
        result = dict(environment_revision=e.info.environment_revision, assets=assets,
            reset_passed=True, solver_audit=e._solver_audit, scope='Runtime geometry inspection and synthetic reset only; no insertion, proof load or demo result.',
            physical_force_calibration=False, contact_only_retention=False)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print('LINER_RUNTIME_AUDIT', json.dumps(result), flush=True)
    finally:
        e.close()


if __name__ == '__main__':
    main()
