"""Opt-in telemetry embodiment; inherits existing physics, task and scorer.

No simulator starts at import. Requires a unique telemetry_path for one episode.
"""

import hashlib
import json
from pathlib import Path

from pallet_lifter.embodiment import PalletEmbodiment
from pallet_lifter.telemetry import sample


class ValidationEmbodiment(PalletEmbodiment):
    def __init__(self, *, telemetry_path, **kwargs):
        super().__init__(**kwargs)
        self._telemetry_path = Path(telemetry_path).expanduser()
        self._telemetry = None
        self._pending_contacts = []

    def _write(self, record):
        # Convert nonfinite physics values to JSON null while retaining history.invalid.
        def clean(value):
            import math
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {k: clean(v) for k, v in value.items()}
            if isinstance(value, list):
                return [clean(v) for v in value]
            return value
        self._telemetry.write(json.dumps(clean(record), allow_nan=False) + '\n')
        self._telemetry.flush()

    def reset(self, scene, *, seed=None):
        if self._telemetry is not None:
            raise RuntimeError('Use a new validation process and telemetry path per episode')
        self._telemetry_path.parent.mkdir(parents=True, exist_ok=True)
        self._telemetry = self._telemetry_path.open('x')
        root = Path(__file__).resolve().parent.parent
        inputs = sorted(root.glob('pallet_lifter/*.py')) + sorted(root.glob('assets/*.usdc'))
        inputs += [root / 'task.py', root / 'pyproject.toml']
        self._write({
            'kind': 'metadata', 'schema_version': 1, 'seed': seed,
            'control_hz': self.info.control_hz, 'physics_device': self.device,
            'object_jitter': self.object_jitter,
            'controller_limitation': 'privileged simulator poses and FK',
            'sampling': 'one reset sample plus one sample after each control step',
            'source_sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in inputs},
        })
        obs = super().reset(scene, seed=seed)
        self._record(step=0, score=self.task.module.score(self._states()))
        return obs

    def _contacts(self, headers, data):
        super()._contacts(headers, data)
        from pxr import PhysicsSchemaTools
        for header in headers:
            if header.num_contact_data:
                actors = [str(PhysicsSchemaTools.intToSdfPath(actor))
                          for actor in (header.actor0, header.actor1)]
                if any('/pallet/' in a.lower() or a.lower().endswith('/pallet') for a in actors):
                    self._pending_contacts.append({
                        'actors': actors, 'contact_data_count': int(header.num_contact_data),
                    })

    def _record(self, *, step, score, terminated=False, termination_reason=None):
        self._write(sample(
            self._states(), self.task.module.HISTORY, self.task.module.TOP,
            step=step, time_s=self._t, contacts=self._pending_contacts, score=score,
            held_steps=self._held, terminated=terminated, termination_reason=termination_reason,
        ))
        self._pending_contacts.clear()

    def _task_step(self):
        result = super()._task_step()
        self._record(step=round(self._t * self.info.control_hz), score=result.info['score'],
                     terminated=result.terminated, termination_reason=result.termination_reason)
        return result

    def close(self):
        try:
            super().close()
        finally:
            if self._telemetry is not None:
                self._telemetry.close()


def make_embodiment(**kwargs):
    return ValidationEmbodiment(**kwargs)
