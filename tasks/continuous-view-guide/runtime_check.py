"""Read-only official runtime checks; no simulator or hosted client is started."""
import importlib.metadata as metadata
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from urllib.parse import unquote, urlsplit

PINS = {
    'yam-arena': ('yam_arena', '0059d02a1a91bc6834c1d0846022891df0c78730'),
    'isaaclab-arena': ('isaaclab_arena', 'c7edf3a9d5d8276565208f72422f8f8a0806d6cf'),
    'inspect-robots-dream': ('inspect_robots_dream', 'b5e936a201cec66c54c297e522c920d98445e339'),
}
VERSIONS = {'inspect-robots': '0.60.0', 'inspect-robots-yam': '0.37.0', 'isaacsim': '6.1.0.0', 'torch': '2.11.0+cu128',
            'dreamlabs-client': '1.0.14', 'numpy': '2.3.1'}


def check():
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError('Use Python3.12 from the pinned official Arena runtime')
    found = {}
    for distribution, (module, expected) in PINS.items():
        direct = json.loads(metadata.distribution(distribution).read_text('direct_url.json') or '{}')
        actual = direct.get('vcs_info', {}).get('commit_id')
        if not actual:
            url = urlsplit(direct.get('url', ''))
            directory = (Path(unquote(url.path)) if url.scheme == 'file'
                         else Path(importlib.util.find_spec(module).origin).parent)
            actual = subprocess.check_output(['git', '-C', str(directory), 'rev-parse', 'HEAD'],
                                             text=True, stderr=subprocess.DEVNULL).strip()
        if actual != expected:
            raise RuntimeError(f'{distribution}: wrong source revision')
        found[distribution] = actual
    versions = {name: metadata.version(name) for name in VERSIONS}
    if versions != VERSIONS:
        raise RuntimeError('Runtime versions differ from runtime-lock.json; do not silently upgrade')
    return dict(passed=True, python=sys.version.split()[0], source_pins=found, packages=versions)


if __name__ == '__main__':
    print(json.dumps(check(), indent=2))
