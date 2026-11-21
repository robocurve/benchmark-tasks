"""Update obsolete actuator keyword names in the pinned, isolated Arena checkout.

Arena eagerly imports these other robot configurations even for a YAM scene.
The rename preserves every numerical value and does not modify IsaacLab.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('arena_dir',type=Path)
    args = parser.parse_args()
    revision = subprocess.check_output(['git','-C',str(args.arena_dir),'rev-parse','HEAD'],text=True).strip()
    if revision != 'c7edf3a9d5d8276565208f72422f8f8a0806d6cf':
        parser.error('This compatibility patch is only for the documented pinned Arena revision')
    patched = {}
    for name in ['droid/droid.py','g1/g1.py','gr1t2/gr1t2.py']:
        path = args.arena_dir/'isaaclab_arena/embodiments'/name
        old = path.read_text()
        new = re.sub(r'(?<![\w])(effort_limit|velocity_limit)=',r'joint_\1=',old)
        if new != old:
            path.write_text(new)
        patched[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    path = args.arena_dir/'isaaclab_arena/environments/isaaclab_arena_manager_based_env_cfg.py'
    old = path.read_text()
    new = old.replace('from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg, NewtonMJWarpManager',
                      'from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg, MJWarpSolver as NewtonMJWarpManager')
    # The Newton preset is not selected by this PhysX task. Its obsolete tendon
    # compatibility hook is never called. PhysX settings remain unchanged.
    if new != old:
        path.write_text(new)
    patched[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    path = args.arena_dir/'isaaclab_arena/environments/isaaclab_arena_manager_based_env.py'
    old = path.read_text()
    marker = '        requested_env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device).sort().values'
    replacement = ('        if isinstance(env_ids, slice):\n'
                   '            env_ids = torch.arange(self.num_envs, device=self.device)[env_ids]\n' + marker)
    new = old if replacement in old else old.replace(marker, replacement)
    if new != old:
        path.write_text(new)
    patched[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    path = args.arena_dir/'isaaclab_arena/utils/cameras.py'
    old = path.read_text()
    new = old.replace('func=mdp.image,', 'func=getattr(mdp, "image", None) or {"rgb": mdp.image_rgb}[dt],')
    if new != old:
        path.write_text(new)
    patched[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    print(json.dumps({'arena_revision':revision,'patched_sha256':patched},indent=2))


if __name__=='__main__':
    main()
