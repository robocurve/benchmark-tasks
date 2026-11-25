"""Observe stock Dream SDK 1.0.14 without changing inputs, replies or smoothing.

The only SDK hook replaces ``PolicyClient._post`` with a logging call-through.
It sees the selected frame window before transport and the decoded server action
before SDK filtering. No transport headers, client credentials or private client
configuration are inspected. All returned objects retain their original identity.
"""

from __future__ import annotations

import base64
from collections.abc import Mapping
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import time
from typing import Any
from zipfile import ZipFile

from inspect_robots.logging.sink import NullSink
import numpy as np


SDK_VERSION = "1.0.14"
_CAMERAS = {
    "image_window.top": "top",
    "image_window.left": "left",
    "image_window.right": "right",
}
_ARMS = ("q_l_chunk", "q_r_chunk")


def _json_value(value: Any) -> Any:
    """Snapshot arrays without changing them; represent invalid floats explicitly."""
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, np.generic):
        return _json_value(value.item())
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"__nonfinite_float__": repr(value)}
    if isinstance(value, Path):
        return str(value)
    return value


def _arm_rows(reply: Any) -> dict[str, Any] | None:
    if not isinstance(reply, Mapping):
        return None
    return {key: reply[key] for key in _ARMS if key in reply}


def _error_fields(error: BaseException) -> dict[str, Any]:
    fields = {"error_type": type(error).__name__, "message": str(error)}
    for name in ("code", "reason", "retryable", "retry_after_s"):
        if hasattr(error, name):
            fields[name] = getattr(error, name)
    return fields


class DreamAudit(NullSink):
    """Persist actual inputs and executed actions; never supply policy targets.

    Each camera tick stores its exact JPEG bytes as ``<tick>/<camera>.jpg``.
    A wire window names the SHA256 and an archived equivalent for every selected
    JPEG, oldest first. Identical JPEGs from multiple ticks are indistinguishable
    on the wire, so an equivalent archive path does not claim its source tick.
    """

    critical = True

    def __init__(self, output: Path):
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        self.frames = ZipFile(output / "dream-input-rgb.zip", "x")
        try:
            self.events = (output / "dream-action-trace.jsonl").open("x")
        except BaseException:
            self.frames.close()
            raise
        self.tick = 0
        self.last_target: np.ndarray | None = None
        self._last_observation_tick: int | None = None
        self._jpeg_index: dict[tuple[str, str], dict[str, Any]] = {}
        self._closed = False

    def event(self, kind: str, **fields: Any) -> None:
        record = {"kind": kind, "tick": self.tick, **_json_value(fields)}
        self.events.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
        self.events.flush()

    def record_observation(self, frames: Mapping[str, str], observation: Any,
                           timestamp: float | None) -> None:
        if self._last_observation_tick == self.tick:
            raise RuntimeError("Dream audit received two observations for one control tick")
        archived = {}
        for key, camera in _CAMERAS.items():
            jpeg = base64.b64decode(frames[key], validate=True)
            digest = hashlib.sha256(jpeg).hexdigest()
            path = f"{self.tick:06d}/{camera}.jpg"
            self.frames.writestr(path, jpeg)
            index = self._jpeg_index.setdefault((key, digest), {
                "equivalent_archive_path": path, "occurrences": 0,
            })
            index["occurrences"] += 1
            archived[key] = {"archive_path": path, "sha256": digest, "bytes": len(jpeg)}
        self.event("observation", time_s=timestamp, observation=observation, frames=archived)
        self._last_observation_tick = self.tick

    def wire_observation(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        metadata, windows = {}, {}
        for key, value in observation.items():
            if key not in _CAMERAS:
                metadata[key] = value
                continue
            entries = []
            for frame in value:
                digest = hashlib.sha256(base64.b64decode(frame, validate=True)).hexdigest()
                indexed = self._jpeg_index.get((key, digest))
                entries.append({"sha256": digest, **(indexed or {
                    "equivalent_archive_path": None, "occurrences": 0,
                })})
            windows[key] = entries
        return {"observation": metadata, "image_windows": windows}

    def log_step(self, t: int, observation: Any, action: Any, result: Any) -> None:
        if t != self.tick:
            raise RuntimeError("Dream audit lost synchronization with Inspect Robots")
        target = np.asarray(action.data).copy()
        self.event("executed", target=target, action_meta=action.meta,
                   input_measured=observation.state["joint_pos"],
                   measured=result.observation.state["joint_pos"],
                   input_time_s=observation.state_time,
                   result_time_s=result.observation.state_time)
        self.last_target = target
        self.tick += 1

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.frames.close()
        finally:
            self.events.close()


class AuditedDreamClient:
    """A bounded, transparent DreamClient proxy for a single sequential trial.

    ``calls`` counts attempts, including SDK/API errors. ``successful_replies``
    counts complete SDK returns. Neither count changes any SDK policy behavior.
    The private ``_post`` tap is intentionally version-locked to the reviewed SDK.
    """

    def __init__(self, native: Any, max_requests: int, audit: DreamAudit):
        if isinstance(max_requests, bool) or not isinstance(max_requests, int) or max_requests < 1:
            raise ValueError("max_requests must be a positive integer")
        self.native, self.max_requests, self.audit = native, max_requests, audit
        self.calls = 0
        self.successful_replies = 0
        self._tapped_policy = None
        self._post_tap = None
        self._original_post = None

    def __getattr__(self, name: str) -> Any:
        return getattr(self.native, name)

    def policy(self, config: Any) -> Any:
        if version("dreamlabs-client") != SDK_VERSION:
            raise RuntimeError(f"Dream response audit requires dreamlabs-client=={SDK_VERSION}")
        result = self.native.policy(config)
        policy = self.native._policy
        original_post = policy._post
        if policy is self._tapped_policy and original_post is self._post_tap:
            original_post = self._original_post

        def logged_post(observation: Any) -> Any:
            self.audit.event("wire_request", attempt=self.calls,
                             **self.audit.wire_observation(observation))
            try:
                reply = original_post(observation)
            except BaseException as error:
                self.audit.event("wire_error", attempt=self.calls, **_error_fields(error))
                raise
            # Persist before the SDK can filter an array in place.
            self.audit.event("server_reply", attempt=self.calls, reply=reply, rows=_arm_rows(reply))
            return reply

        policy._post = logged_post
        self._tapped_policy, self._post_tap = policy, logged_post
        self._original_post = original_post
        self.audit.event("policy", config=config, sdk_version=SDK_VERSION,
                         behavior="stock_sdk_passthrough")
        return result

    def set_observation(self, frames: Any, observation: Any, *, timestamp: Any = None) -> Any:
        self.audit.record_observation(frames, observation, timestamp)
        try:
            return self.native.set_observation(frames, observation, timestamp=timestamp)
        except BaseException as error:
            self.audit.event("observation_error", **_error_fields(error))
            raise

    def get_action(self, *, commanded: Any = None) -> Any:
        if self.calls >= self.max_requests:
            self.audit.event("request_cap", max_requests=self.max_requests)
            raise RuntimeError("Dream action-request cap reached")
        self.calls += 1
        started = time.monotonic()
        self.audit.event("request", attempt=self.calls, commanded=commanded)
        try:
            reply = self.native.get_action(commanded=commanded)
        except BaseException as error:
            self.audit.event("request_error", attempt=self.calls,
                             elapsed_s=time.monotonic() - started, **_error_fields(error))
            raise
        self.successful_replies += 1
        self.audit.event("reply", attempt=self.calls, reply=reply, sdk_rows=_arm_rows(reply),
                         elapsed_s=time.monotonic() - started,
                         last_timing=getattr(self.native, "last_timing", None),
                         session_id=getattr(self.native, "session_id", None))
        return reply
