#!/usr/bin/env python3
"""Verify packaged source, instruction and recorded clocks; no simulation or model call."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys

MODULE = 'rdb_toaster_tray'
SLUG = 'empty-a-toaster-crumb-tray'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(path.read_text())


def local(root, relative):
    require(isinstance(relative, str) and not Path(relative).is_absolute(),
            f'Expected relative package path: {relative!r}')
    path = (root / relative).resolve(strict=True)
    require(path.is_relative_to(root) and path.is_file(), f'Unsafe package path: {relative}')
    return path


def constants(package):
    """Read only literal constants and their explicit fixed_scene layout aliases."""
    layout = {}
    for node in ast.parse((package / 'layout.py').read_text()).body:
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    layout[target.id] = node.value.value
    tree = ast.parse((package / 'fixed_scene.py').read_text())
    aliases = {alias.asname or alias.name for node in tree.body
               if isinstance(node, ast.ImportFrom) and node.level == 1 and node.module is None
               for alias in node.names if alias.name == 'layout'}
    values = {}
    def value(node):
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id in aliases:
            return layout[node.attr]
        raise ValueError('Task constants must be literal or explicit layout aliases')
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            targets = target.elts if isinstance(target, (ast.Tuple, ast.List)) else [target]
            expressions = node.value.elts if isinstance(node.value, (ast.Tuple, ast.List)) else [node.value]
            if any(isinstance(t, ast.Name) and t.id in {'CONTROL_HZ', 'MAX_STEPS', 'INSTRUCTION'} for t in targets):
                require(len(targets) == len(expressions), 'Malformed fixed_scene constant assignment')
                for t, expression in zip(targets, expressions):
                    if isinstance(t, ast.Name) and t.id in {'CONTROL_HZ', 'MAX_STEPS', 'INSTRUCTION'}:
                        values[t.id] = value(expression)
    require(set(values) == {'CONTROL_HZ', 'MAX_STEPS', 'INSTRUCTION'}, 'Missing fixed_scene constants')
    return values


def verify(root, extra_results=()):
    root = Path(root).resolve(strict=True)
    validation, card = read(root / 'validation.json'), read(root / 'task.json')
    manifest = validation['source_sha256']
    recorded = validation['recorded_source_sha256']
    source_file = read(root / 'SOURCE-MANIFEST.json')
    require(source_file['source_sha256'] == manifest and source_file['recorded_source_sha256'] == recorded,
            'Package and validation manifests disagree')
    require(set(recorded) == set(manifest) and {n for n in manifest if recorded[n] != manifest[n]} == {'grading.py'},
            'Expected a scorer-only runtime revision')
    revision = read(root / validation['revision_evidence'])
    require(revision['scoring_source_sha256'] == manifest['grading.py'], 'Revision report uses a different scorer')
    require(revision['media_sha256'] == validation['media_sha256'], 'Revision media differs from recording')
    require(isinstance(manifest, dict) and bool(manifest), 'Empty source manifest')
    package = (root / MODULE).resolve(strict=True)
    require(package.is_relative_to(root) and package.is_dir(), 'Unsafe module directory')
    for name, expected in manifest.items():
        require(isinstance(name, str) and Path(name).name == name, f'Unsafe source name: {name!r}')
        path = local(package, name)
        require(isinstance(expected, str) and hashlib.sha256(path.read_bytes()).hexdigest() == expected,
                f'Source SHA-256 mismatch: {name}')
    actual = {p.name for p in package.iterdir() if p.is_file() and p.suffix in {'.py', '.usda', '.json'}}
    require(actual == set(manifest), 'Source manifest does not cover the complete runtime module')
    fixed = constants(package)
    hz, budget = fixed['CONTROL_HZ'], fixed['MAX_STEPS']
    require(isinstance(hz, int) and not isinstance(hz, bool) and hz == 20, 'Invalid source control clock')
    require(isinstance(budget, int) and not isinstance(budget, bool) and budget > 0, 'Invalid source budget')
    require(card['instruction'] == fixed['INSTRUCTION'], 'Task instruction differs from frozen fixed_scene')
    require(validation['control_hz'] == hz and validation['max_steps'] == budget,
            'Validation clock/budget differs from frozen source')
    result_paths = []
    for kind in ('scripted', 'dream'):
        run = validation['runs'][kind]
        require(run['step_limit'] == budget, f'{kind} budget differs from source')
        if 'control_hz' in run:
            require(run['control_hz'] == hz, f'{kind} clock differs from source')
        if 'source_sha256' in run:
            require(run['source_sha256'] == recorded, f'{kind} source manifest differs')
        result_paths.extend(run.get('trial_paths', []))
        if run.get('result_path'):
            result_paths.append(run['result_path'])
    require(bool(result_paths), 'No local recorded result paths in validation')
    checked, policies = [], set()
    for relative in dict.fromkeys([*result_paths, *extra_results]):
        path = local(root, relative)
        result = read(path)
        require(result['source_sha256'] == (recorded if relative in result_paths else manifest), f'Result source manifest differs: {relative}')
        require(result['control_hz'] == hz and result['step_limit'] == result['declared_step_limit'] == budget,
                f'Recorded clock/budget differs: {relative}')
        require(result.get('candidate_slug', SLUG) == SLUG and result.get('module', MODULE) == MODULE,
                f'Recorded task/module differs: {relative}')
        require(result['policy'] in {'scripted', 'dream'}, f'Unexpected recorded policy: {relative}')
        policies.add(result['policy'])
        checked.append(str(path.relative_to(root)))
    require(policies == {'scripted', 'dream'}, 'Both scripted and model result metadata are required')
    return dict(ok=True, candidate_slug=SLUG, module=MODULE, source_files=len(manifest),
                instruction_exact=True, control_hz=hz, max_steps=budget, recorded_results_checked=checked,
                scope='Source and recorded metadata consistency only; not native-input or model-provenance replay')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--result', action='append', default=[], help='Additional relative local result metadata')
    args = parser.parse_args()
    try:
        result = verify(args.root, args.result)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(json.dumps(dict(ok=False, error=str(exc))))
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
