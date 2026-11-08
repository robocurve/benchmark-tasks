"""A separate assessor's tensile test; never used to assist the robot policy.

Two equal opposing world-space forces are applied at the end-link centres of
mass through the simulator's persistent wrench API. This corresponds to an
assessor loading the two ends with a specified weight. It is a numerical loading
fixture, not part of the robot action space, and never moves or anchors a body.
Recorded forces are read back from the force buffers after the simulator steps.
"""
from __future__ import annotations
import numpy as np


class EndpointLoad:
    def __init__(self, base, rope_name='cord', magnitude=.981):
        self.base = base
        self.asset = base.scene[rope_name]
        self.magnitude = float(magnitude)
        names = list(self.asset.body_names)
        self.n = len(names)
        self.indices = [names.index('seg_000'), names.index(f'seg_{self.n-1:03d}')]
        self.elapsed = 0.
        self.active = False

    def apply(self, start, end):
        import torch
        delta = np.asarray(end, float) - np.asarray(start, float)
        if not np.isfinite(delta).all() or np.linalg.norm(delta) < .05:
            raise ValueError('Proof load requires end separation of at least 5 cm')
        direction = delta / np.linalg.norm(delta)
        forces = torch.as_tensor(np.stack((-direction, direction)) * self.magnitude,
                                 dtype=torch.float32, device=self.base.device).unsqueeze(0)
        self.asset.set_external_force_and_torque(
            forces, torch.zeros_like(forces), body_ids=self.indices, is_global=True)
        self.active = True

    def record(self, dt):
        import warp as wp
        if not self.active:
            raise RuntimeError('Cannot record an unapplied proof load')
        self.elapsed += float(dt)
        composer = self.asset.permanent_wrench_composer
        # These are the actual persistent world-space buffers consumed by the
        # native physics backend, including any positional-force contribution.
        at_com = wp.to_torch(composer.global_force_at_com_w)[0].detach().cpu().numpy()
        positioned = wp.to_torch(composer.global_force_w)[0].detach().cpu().numpy()
        forces = at_com + positioned
        others = np.delete(forces, self.indices, axis=0)
        if not np.isfinite(forces).all() or np.max(np.abs(others), initial=0) > 1e-7:
            raise RuntimeError('Unexpected force on a non-endpoint rope link')
        endpoints = forces[self.indices]
        return dict(protocol='opposing_endpoint_load_v1', active=True,
                    source='simulator_applied_external_force',
                    endpoint_indices=[0, self.n - 1],
                    endpoint_forces_n=endpoints.tolist(),
                    applied_force_n=float(np.linalg.norm(endpoints, axis=1).min()),
                    elapsed_s=self.elapsed,
                    buffer_readback='permanent_wrench_composer world force buffers')

    def clear(self):
        self.asset.permanent_wrench_composer.reset()
        self.active = False
