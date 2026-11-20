"""Prepare an isolated compatibility overlay from an operator-installed adapter.

No third-party adapter source is redistributed with this task. Original
assets remain in the installed checkout with their own licences.
"""
import argparse,hashlib,json,shutil,subprocess
from pathlib import Path
ADAPTER_COMMIT='0059d02a1a91bc6834c1d0846022891df0c78730'

def needs_overlay(arena_root):
    """True only for Arena builds whose API differs from release/0.3.1 (Embodiment.registry_name, no GroundPlane)."""
    arena=Path(arena_root).resolve()/'isaaclab_arena'
    base=(arena/'embodiments'/'embodiment_base.py').read_text()
    library=(arena/'assets'/'object_library.py').read_text()
    return 'registry_name' in base or 'class GroundPlane' not in library

def prepare(adapter,root=None):
    adapter=Path(adapter).resolve();root=Path(root or Path(__file__).parent/'.runtime')
    revision=subprocess.check_output(['git','-C',str(adapter),'rev-parse','HEAD'],text=True).strip()
    if revision!=ADAPTER_COMMIT:raise RuntimeError(f'Expected adapter {ADAPTER_COMMIT}; found {revision}')
    source=adapter/'yam_arena';dest=root/'yam_arena';dest.mkdir(parents=True,exist_ok=True)
    for file in source.glob('*.py'):shutil.copy2(file,dest/file.name)
    file=dest/'embodiment.py';text=file.read_text();old='    name = "yam_bimanual"'
    if old not in text:raise RuntimeError('Pinned compatibility patch no longer applies')
    file.write_text(text.replace(old,'    registry_name = "yam_bimanual"',1))
    file=dest/'scene.py';text=file.read_text();old='        GroundPlane(),'
    if old not in text:raise RuntimeError('Pinned floor compatibility patch no longer applies')
    file.write_text(text.replace(old,'        _box("floor", (20.,20.,0.1), (0.,0.,-0.05), (0.2,0.2,0.2)),',1))
    link=dest/'assets'
    if not link.exists():link.symlink_to(source/'assets',target_is_directory=True)
    elif link.resolve()!=(source/'assets').resolve():raise RuntimeError('Overlay assets point to a different adapter')
    (root/'runtime_manifest.json').write_text(json.dumps({'adapter_commit':revision,'compatibility_patches':['Embodiment registry_name','rigid tabletop-scene floor'],'source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in dest.glob('*.py')}},indent=2))
    return root.resolve()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--adapter',type=Path,required=True);a=p.parse_args();print(prepare(a.adapter))
