"""Record Dream inputs and anchor each new chunk to the last executed target."""

import base64
from importlib.metadata import version
import json
import sys
import time
from zipfile import ZipFile

from inspect_robots.logging.sink import NullSink
import numpy as np


SDK_VERSION = "1.0.14"
TRANSITION = "executed_target_blend_v2"
# DA-2000 refusals that only say Dream-1 is busy right now (dreamlabs-client 1.0.14, DreamClient.get_action).
CAPACITY_REASONS = frozenset({"pool_full", "queue_timeout", "max_serves", "pool_warming", "rate_limited"})
CAPACITY_PAUSE_S = 30.
CAPACITY_GIVE_UP_S = 2 * 3600.


def capacity_refusal(error):
    """True when ``error`` is a DA-2000 refusal saying only that Dream-1 is at capacity or starting."""
    import dream

    return (isinstance(error, dream.DreamServerError) and not isinstance(error, dream.DreamSessionExpired)
            and getattr(error, "code", None) == "DA-2000" and getattr(error, "reason", None) in CAPACITY_REASONS)


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


def request_budget_spent(client, policy, cap):
    """``"dream_request_cap"`` once the trial has made its last allowed action request and executed every row of
    that reply (the policy's queue is empty), else None. The embodiment then ends the trial at that step and the scorer
    scores it as it stands, instead of the next request failing the trial."""
    queue = getattr(getattr(policy, "_trial", None), "queue", None)
    return "dream_request_cap" if client.calls >= cap and not queue else None


class BudgetedDreamClient:
    def __init__(self, native, limit, audit, *, sleep=time.sleep):
        self.native, self.limit, self.audit = native, limit, audit
        self.calls = 0
        self.capacity_waits, self.capacity_wait_s = 0, 0.
        self._sleep = sleep

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
        if self.calls >= self.limit:      # never reached: run.py ends the trial when the last reply is used up
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
        reply = self._ask()
        from inspect_robots_dream import _station

        _station.rows(reply)
        filtered = self._arm_values(reply)
        reply = {**reply, **{arm: blend_chunk(reply[arm], value) for arm, value in anchor.items()}}
        self.audit.event("reply", attempt=self.calls, sdk_rows=filtered, rows=self._arm_values(reply))
        return reply

    def _ask(self):
        """The SDK's reply. While Dream-1 answers that it is at capacity, wait and ask again with the same
        observation, as the SDK documents. Physics stays paused for the whole wait, so the trial is the one it would
        have been without the wait. Any other refusal, and a wait past CAPACITY_GIVE_UP_S, still ends the trial."""
        waited = 0.
        while True:
            try:
                return self.native.get_action()
            except Exception as error:
                if not capacity_refusal(error) or waited >= CAPACITY_GIVE_UP_S:
                    raise
                pause = max(CAPACITY_PAUSE_S, float(getattr(error, "retry_after_s", None) or 0.))
                self.capacity_waits += 1
                self.capacity_wait_s += pause
                self.audit.event("capacity_wait", attempt=self.calls, reason=error.reason, pause_s=pause)
                print(f"Dream: {error.reason}; waiting {pause:.0f} s for a free seat", file=sys.stderr, flush=True)
                self._sleep(pause)
                waited += pause

    @staticmethod
    def _arm_values(values):
        if values is None:
            return None
        return {key: np.asarray(values[key]).tolist() for key in ("q_l_chunk", "q_r_chunk")}
