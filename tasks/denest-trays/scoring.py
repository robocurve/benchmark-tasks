"""One stateful scorer used by both the runtime and offline evaluation."""
import numpy as np


class TrialScorer:
    def __init__(self, state_metrics, table_z, hz, hold_seconds, finger_open):
        self.state_metrics = state_metrics
        self.table_z = table_z
        self.hold_frames = round(hz * hold_seconds)
        self.finger_open = finger_open
        self.last_step = 0
        self.held = 0
        self.dropped = False

    def update(self, *, step, objects, root_poses, robot):
        # Counting the same frame twice must never satisfy the dwell requirement.
        if step != self.last_step + 1:
            raise ValueError('Score every control step once, in order, starting at 1')
        if set(objects) != {'outer', 'inner'} or set(root_poses) != set(objects):
            raise ValueError('Both tray states and exact world root poses are required')
        clear = True
        for side in ('left', 'right'):
            fingers = np.asarray(robot[side]['fingers'], dtype=float)
            tcp = np.asarray(robot[side]['tcp'], dtype=float)
            if fingers.shape != (2,) or tcp.shape != (3,) or not np.isfinite(np.r_[fingers,tcp]).all():
                raise ValueError('Finite measured finger positions and TCPs are required')
            clear &= bool(np.mean(fingers / self.finger_open) > .8)
            for obj in objects.values():
                clear &= bool(np.linalg.norm(np.maximum(np.maximum(obj.low-tcp,tcp-obj.high),0)) > .05)
        dropped = self.dropped or any(obj.low[2] < self.table_z-.02 for obj in objects.values())
        metrics = self.state_metrics(objects, root_poses, clear=clear, dropped=dropped)
        instantaneous = metrics['score']
        held = self.held+1 if instantaneous == 1.0 else 0
        success = held >= self.hold_frames
        self.last_step, self.held, self.dropped = step, held, dropped
        return {**metrics, 'instantaneous_score': instantaneous,
                'score': instantaneous if success else min(instantaneous,.5),
                'success': success, 'held_frames': held, 'clear': clear, 'dropped': dropped}
