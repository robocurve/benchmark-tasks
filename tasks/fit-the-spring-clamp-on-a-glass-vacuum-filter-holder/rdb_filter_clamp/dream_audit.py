"""Record Dream inputs and anchor each new chunk to the last executed target."""

import base64
from importlib.metadata import version
import json
from zipfile import ZipFile

from inspect_robots.logging.sink import NullSink
import numpy as np


SDK_VERSION = "1.0.14"
TRANSITION = "executed_target_blend_v2"


def blend_chunk(rows, target):
    """Join the SDK's filtered plan to the target held while inference ran."""
    chunk = np.array(rows, dtype=float, copy=True)
    for columns, ramp in ((slice(0, 6), 16), (slice(6, 7), 4)):
        count = min(len(chunk) - 1, ramp)
        phase = np.linspace(0, 1, count + 1)
        weight = (phase**3 * (10 + phase * (-15 + 6 * phase)))[:, None]
        chunk[:count + 1, columns] = (
            (1 - weight) * target[columns] + weight * chunk[:count + 1, columns]
        )
    return chunk


class DreamAudit(NullSink):
    critical = True

    def __init__(self, output):
        self.frames = ZipFile(output / "dream-input-rgb.zip", "x")
        self.events = (output / "dream-action-trace.jsonl").open("x")
        self.tick = 0
        self.last_target = None

    def event(self, kind, **fields):
        self.events.write(json.dumps({"kind": kind, "tick": self.tick, **fields},
                                     allow_nan=False, separators=(",", ":")) + "\n")

    def log_step(self, t, observation, action, result):
        if t != self.tick:
            raise RuntimeError("Dream audit lost synchronization with Inspect Robots")
        self.last_target = np.asarray(action.data, dtype=float).copy()
        self.event("executed", target=self.last_target.tolist(),
                   measured=np.asarray(result.observation.state["joint_pos"]).tolist())
        self.tick += 1

    def close(self):
        self.frames.close()
        self.events.close()


class BudgetedDreamClient:
    def __init__(self, native, limit, audit):
        self.native, self.limit, self.audit = native, limit, audit
        self.calls = 0

    def __getattr__(self, name):
        return getattr(self.native, name)

    def policy(self, config):
        if version("dreamlabs-client") != SDK_VERSION:
            raise RuntimeError(f"This Dream action audit requires dreamlabs-client=={SDK_VERSION}")
        self.native.policy(config)
        # The public SDK returns filtered actions. Tap the decoded response before filtering.
        policy = self.native._policy
        post = policy._post

        def record_response(observation):
            reply = post(observation)
            self.audit.event("server_reply", attempt=self.calls, rows=self._arm_values(reply))
            return reply

        policy._post = record_response

    def set_observation(self, frames, observation, *, timestamp=None):
        for camera in ("top", "left", "right"):
            jpeg = frames[f"image_window.{camera}"]
            self.audit.frames.writestr(f"{self.audit.tick:06d}/{camera}.jpg",
                                       base64.b64decode(jpeg, validate=True))
        self.audit.event("observation", time_s=timestamp,
                         measured=[observation[f"state.arm_{side}.{field}"]
                                   for side in ("l", "r") for field in ("joint_pos", "gripper_pos")])
        return self.native.set_observation(frames, observation, timestamp=timestamp)

    def get_action(self, *, commanded=None):
        if self.calls >= self.limit:
            raise RuntimeError("Dream action-request cap reached")
        self.calls += 1
        target = self.audit.last_target
        if target is None:
            raise RuntimeError("Dream requested actions before the first target was executed")
        anchor = {"q_l_chunk": target[:7], "q_r_chunk": target[7:]}
        self.audit.event("request", attempt=self.calls,
                         adapter_anchor=self._arm_values(commanded),
                         executed_anchor=self._arm_values(anchor))
        # Physics pauses and consumes the full previous chunk. Keep the SDK's row
        # filter, but join locally without searching the old, unguarded plan.
        self.native._policy._smoother.reset()
        reply = self.native.get_action()
        from inspect_robots_dream import _station

        _station.rows(reply)
        filtered = self._arm_values(reply)
        reply = {**reply, **{arm: blend_chunk(reply[arm], value) for arm, value in anchor.items()}}
        self.audit.event("reply", attempt=self.calls, sdk_rows=filtered, rows=self._arm_values(reply))
        return reply

    @staticmethod
    def _arm_values(values):
        if values is None:
            return None
        return {key: np.asarray(values[key]).tolist() for key in ("q_l_chunk", "q_r_chunk")}
