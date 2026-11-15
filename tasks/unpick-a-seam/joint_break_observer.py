"""Read native joint-break events without changing physics or scoring.

Uses Isaac6.1's installed omni.physx JointBreakDemo event-stream API. A reported
native joint break is distinct from small numerical endpoint error.
The current seam panel hinges do not have a calibrated finite fracture load.
Absence of an event therefore does not establish that fabric would remain intact.
"""


class JointBreakObserver:
    def __init__(self):
        from omni.physx import get_physx_interface
        from omni.physx.bindings._physx import SimulationEvent
        from pxr import PhysicsSchemaTools
        self.events = []
        self.phase = 'construction'
        self.control_step = None
        self._break_type = int(SimulationEvent.JOINT_BREAK)
        self._decode_path = PhysicsSchemaTools.decodeSdfPath
        self._stream = get_physx_interface().get_simulation_event_stream_v2()
        self._subscription = self._stream.create_subscription_to_pop(self._on_event)

    def context(self, phase, control_step=None):
        self._stream.pump()
        self.phase = str(phase)
        self.control_step = control_step

    def _on_event(self, event):
        if event.type != self._break_type:
            return
        path = str(self._decode_path(*event.payload['jointPath']))
        self.events.append(dict(joint_path=path, phase=self.phase,
                                control_step=self.control_step,
                                cloth_hinge='/cloth_' in path and '/hinge_' in path))

    def summary(self):
        # Dispatch queued notifications even if a substep guard interrupted
        # before the next rendered app update. This does not step physics.
        # https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/carb.events/carb.events.IEventStream.html
        self._stream.pump()
        return dict(observer_installed=True, event_count=len(self.events),
                    cloth_hinge_break_count=sum(e['cloth_hinge'] for e in self.events),
                    events=list(self.events),
                    interpretation='Native break notifications only; no joint edits, score or no-damage certificate.')

    def close(self):
        self._subscription = None
