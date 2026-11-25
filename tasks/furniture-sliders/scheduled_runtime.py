"""Task-local native startup adapter. Fail closed before any simulator import."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import sys

SCHEDULER = Path(os.environ.get('ROBOCURVE_SCHEDULER_DIR', Path.home()/'robocurve-gpu-scheduler')).expanduser()


def reservation():
    if os.environ.get('ROBOCURVE_TASK_ID') != 'furniture-sliders':
        raise RuntimeError('Launch through gpu-scheduler with --task furniture-sliders')
    sys.path.insert(0, str(SCHEDULER)) if str(SCHEDULER) not in sys.path else None
    from isaac_binding import assigned_gpu
    return assigned_gpu()


@contextmanager
def native_startup():
    """Inject required Kit arguments into native YAM without modifying installed files."""
    reservation()
    from isaac_binding import kit_args
    import isaaclab.app
    original = isaaclab.app.AppLauncher

    def scheduled_launcher(*args, **kwargs):
        if args or kwargs.get('device', 'cuda:0') != 'cuda:0':
            raise RuntimeError('Expected native keyword startup with masked cuda:0')
        kwargs.update(device='cuda:0', multi_gpu=False, kit_args=kit_args())
        return original(**kwargs)

    isaaclab.app.AppLauncher = scheduled_launcher
    try:
        yield
    finally:
        isaaclab.app.AppLauncher = original


def verify_runtime(emb, out):
    from isaac_binding import verify
    sim = emb._base.sim
    # This is the actual task rigid body view, not an allocated test tensor.
    bodies = sim.physics_sim_view.create_rigid_body_view('/World/envs/env_0/furniture/body')
    transforms = bodies.get_transforms()
    report = verify(sim, transforms.device)
    report['physics_tensor_source'] = '/World/envs/env_0/furniture/body:get_transforms'
    Path(out, 'device-verification.json').write_text(json.dumps(report, indent=2) + '\n')
    return report
