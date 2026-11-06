"""Create a fresh private container from the exact image used for the evidence."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import uuid

ROOT = Path(__file__).resolve().parent
IMAGE = 'sha256:0e5a603c33c29eab2751be8977127a74a469f64dad70ca4bc3e50eacc3f504a8'


def output(command):
    return subprocess.check_output(command, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='clean-ice-review-' + uuid.uuid4().hex[:10])
    parser.add_argument('--runtime-dir', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=2)
    parser.add_argument('--dream', action='store_true', help='Enable bridge networking for Dream')
    parser.add_argument('--credentials-file', type=Path, help='Mount this private file read-only; never copy it')
    parser.add_argument('--dry-run', action='store_true', help='Print configuration without creating anything')
    args = parser.parse_args()
    runtime = args.runtime_dir.expanduser().resolve()
    if runtime == ROOT or ROOT in runtime.parents:
        parser.error('Keep runtime storage outside the task folder')
    if args.credentials_file and not args.dream:
        parser.error('--credentials-file is only used with --dream')
    credentials = args.credentials_file.expanduser().resolve() if args.credentials_file else None
    if args.dream and (credentials is None or not credentials.is_file() or credentials.stat().st_size == 0):
        parser.error('--dream requires an existing private --credentials-file')
    if credentials is not None and credentials.stat().st_mode & 0o077:
        parser.error('The credentials file must have private permissions (chmod 600)')
    gpu = output(['nvidia-smi', '-i', str(args.gpu), '--query-gpu=uuid', '--format=csv,noheader'])
    if len(gpu.splitlines()) != 1:
        raise SystemExit('Select exactly one physical GPU')
    free = int(output(['nvidia-smi', '-i', str(args.gpu), '--query-gpu=memory.free', '--format=csv,noheader,nounits']))
    if free < 16384:
        raise SystemExit('The selected GPU has less than 16 GiB free')
    assert output(['docker', 'image', 'inspect', IMAGE, '--format', '{{.Id}}']) == IMAGE
    existing = subprocess.run(['docker', 'inspect', args.name], capture_output=True)
    if existing.returncode == 0:
        raise SystemExit('That container name already exists; choose a fresh name')
    metadata = {'name': args.name, 'image': IMAGE, 'physical_gpu': args.gpu, 'gpu_uuid': gpu,
                'network': 'bridge' if args.dream else 'none', 'task_read_only': str(ROOT),
                'private_home': str(runtime/'home'), 'private_outputs': str(runtime/'outputs')}
    if args.dry_run:
        print(json.dumps(metadata, indent=2))
        return
    runtime.mkdir(parents=True, exist_ok=False, mode=0o700)
    for directory in ('home', 'outputs', 'home/.dreamlabs'):
        (runtime/directory).mkdir(parents=True, exist_ok=True, mode=0o700)
    command = ['docker', 'run', '-d', '--name', args.name, '--init', '--no-healthcheck',
               '--network', metadata['network'], '--gpus', 'device='+gpu,
               '--user', f'{os.getuid()}:{os.getgid()}', '--group-add', '1234',
               '--cpus', '8', '--memory', '24g', '--shm-size', '2g', '--workdir', '/workspace/task',
               '--label', 'task=clean-ice-fish', '--label', 'purpose=local-review',
               '--mount', f'type=bind,src={ROOT},dst=/workspace/task,readonly',
               '--mount', f'type=bind,src={runtime/"home"},dst=/home/reviewer',
               '--mount', f'type=bind,src={runtime/"outputs"},dst=/outputs']
    if credentials is not None:
        command += ['--mount', f'type=bind,src={credentials},dst=/home/reviewer/.dreamlabs/credentials,readonly']
    environment = {'HOME': '/home/reviewer', 'ACCEPT_EULA': 'Y', 'OMNI_KIT_ACCEPT_EULA': 'YES',
                   'PYTHONDONTWRITEBYTECODE': '1', 'PYTHONUNBUFFERED': '1', 'OMP_NUM_THREADS': '8',
                   'OPENBLAS_NUM_THREADS': '8', 'NVIDIA_DRIVER_CAPABILITIES': 'compute,utility,graphics',
                   'YAM_HOST_GPU_INDEX': str(args.gpu), 'OMNICLIENT_HUB_MODE': 'disabled'}
    for key, value in environment.items():
        command += ['--env', f'{key}={value}']
    command += ['--entrypoint', '/bin/bash', IMAGE, '-c', 'exec sleep infinity']
    subprocess.run(command, check=True)
    try:
        identity = f'''from pathlib import Path
uid,gid={os.getuid()},{os.getgid()}
for filename,label,entry in [('passwd','reviewer',f'reviewer:x:{{uid}}:{{gid}}::/home/reviewer:/bin/bash'),('group','reviewer',f'reviewer:x:{{gid}}:')]:
 p=Path('/etc')/filename
 number=uid if filename=='passwd' else gid
 if not any(line.split(':')[2]==str(number) for line in p.read_text().splitlines()):
  with p.open('a') as f:f.write(entry+'\\n')
'''
        subprocess.run(['docker', 'exec', '--user', '0:0', args.name, '/isaac-sim/python.sh', '-B', '-c', identity], check=True)
        assert output(['docker', 'exec', args.name, 'nvidia-smi', '--query-gpu=uuid', '--format=csv,noheader']) == gpu
        (runtime/'container.json').write_text(json.dumps(metadata, indent=2)+'\n')
        print(json.dumps(metadata, indent=2))
    except BaseException:
        # This name was newly created above; existing containers are never changed.
        subprocess.run(['docker', 'stop', '--timeout', '10', args.name], capture_output=True)
        raise


if __name__ == '__main__':
    main()
