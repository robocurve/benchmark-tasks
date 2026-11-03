"""Batch Dream observations over the existing localhost SSH bridge.

Every observation is copied locally and replayed, in order, immediately before
the next inference request. DreamSession.observe only updates its local frame
window; it does not call the inference gateway. Simulation timestamps, prompts,
states, preroll and action chunk cadence are unchanged. No ticks are discarded.

On the Mac, in the Dream environment:
    python dream_bridge.py --host 127.0.0.1 --port 8210

In the runner:
    DreamPolicy(session=BatchedBridgeSession("http://127.0.0.1:8210"))

Uses the installed inspect-robots-dream 0.1.0 BridgeSession transport and
BridgeServer HTTP handler/lock. Their protected transport attributes are the
only dependency on bridge internals. Credentials remain in DreamSession's
process; this module neither reads nor forwards them.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import uuid
from collections.abc import Mapping, Sequence
from contextlib import closing, suppress
from typing import Any

import numpy as np

from inspect_robots_dream.bridge import BridgeServer, BridgeSession
from inspect_robots_dream.session import Chunk, DreamSession, Episode, REQUEST_TIMEOUT_S, Session


def _vector(value: Any) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).reshape(-1)
    if result.shape != (14,) or not np.isfinite(result).all():
        raise ValueError("expected a finite 14-D bimanual YAM vector")
    return result.copy()


def _tick(frames: Mapping[str, str], state: Any, prompt: str, timestamp: Any) -> dict[str, Any]:
    if not isinstance(frames, Mapping) or not {"top_cam", "left_cam", "right_cam"} <= frames.keys():
        raise ValueError("each tick needs top_cam, left_cam and right_cam")
    if not all(isinstance(k, str) and isinstance(v, str) and v for k, v in frames.items()):
        raise ValueError("camera frames must be nonempty JPEG strings")
    if not isinstance(prompt, str):
        raise TypeError("prompt must be a string")
    if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
        raise ValueError("timestamp must be finite")
    return {"frames": dict(frames), "state": _vector(state).tolist(),
            "prompt": prompt, "timestamp": float(timestamp)}


class BatchedBridgeSession(BridgeSession):
    """The standard bridge session, with one observation batch per action call.

    Calls are sequential, as in DreamPolicy. A failed action request retains its
    exact payload for retry; observe/start cannot silently change that retry.
    """

    def __init__(self, url: str, *, post=None) -> None:
        super().__init__(url, post=post)
        self._ticks: list[dict[str, Any]] = []
        self._sequence = 0
        self._last_timestamp: float | None = None
        self._pending: dict[str, Any] | None = None

    def start(self, episode: Episode) -> None:
        super().start(episode)
        self._ticks.clear()
        self._sequence = 0
        self._last_timestamp = None
        self._pending = None

    def observe(self, frames, state, prompt, timestamp) -> None:
        if not self._episode:
            raise RuntimeError("start an episode before observing")
        if self._pending is not None:
            raise RuntimeError("retry the failed action request or start a new episode before observing")
        tick = _tick(frames, state, prompt, timestamp)
        if self._last_timestamp is not None and tick["timestamp"] < self._last_timestamp:
            raise ValueError("observation timestamps must not go backwards")
        self._ticks.append(tick)
        self._last_timestamp = tick["timestamp"]

    def act(self, commanded) -> Chunk:
        if not self._episode:
            raise RuntimeError("start an episode before acting")
        command = None if commanded is None else _vector(commanded).tolist()
        if self._pending is None:
            self._pending = {"episode": self._episode, "sequence": self._sequence,
                             "ticks": list(self._ticks), "commanded": command}
        elif command != self._pending["commanded"]:
            raise ValueError("an action retry must preserve its commanded joints")
        wait_s = self._timeout_s + REQUEST_TIMEOUT_S + 15.0
        reply = self._post(f"{self.url}/act_batch", self._pending, wait_s)
        chunk = Chunk.from_json(reply)
        self._ticks.clear()
        self._pending = None
        self._sequence += 1
        return chunk

    def close(self) -> None:
        self._ticks.clear()
        self._pending = None
        self._episode = ""
        super().close()


class BatchRoutes:
    """Episode-owned routes, invoked under BridgeServer's existing request lock.

    Retaining the last reply makes replay of a lost response idempotent. If
    inference fails after observations were applied, the same batch retries
    inference without adding those observations a second time.
    """

    def __init__(self, session: Session) -> None:
        self.session = session
        self._episode = ""
        self._next_sequence = 0
        self._last_timestamp: float | None = None
        self._pending_digest: str | None = None
        self._pending_applied = 0
        self._last_digest: str | None = None
        self._last_reply: dict[str, Any] | None = None

    @property
    def routes(self):
        return {"/check": self.check, "/start": self.start, "/act_batch": self.act_batch}

    def check(self, body):
        self.session.check(Episode(**body))
        return {}

    def start(self, body):
        self.session.start(Episode(**body))
        self._episode = uuid.uuid4().hex
        self._next_sequence = 0
        self._last_timestamp = None
        self._pending_digest = self._last_digest = None
        self._pending_applied = 0
        self._last_reply = None
        return {"episode": self._episode}

    def act_batch(self, body):
        if not self._episode or body.get("episode") != self._episode:
            raise RuntimeError("this episode is not the one the bridge is running: start it again")
        sequence = body.get("sequence")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise ValueError("batch sequence must be a nonnegative integer")
        if not isinstance(body.get("ticks"), list):
            raise TypeError("ticks must be an ordered list")
        # Validate the complete batch before applying any observations.
        ticks = [_tick(**tick) for tick in body["ticks"]]
        if any(a["timestamp"] > b["timestamp"] for a, b in zip(ticks, ticks[1:])):
            raise ValueError("batch timestamps must not go backwards")
        command = None if body["commanded"] is None else _vector(body["commanded"])
        normalized = {"ticks": ticks, "commanded": None if command is None else command.tolist()}
        digest = hashlib.sha256(json.dumps(normalized, sort_keys=True, allow_nan=False).encode()).hexdigest()
        if sequence == self._next_sequence - 1 and self._last_reply is not None:
            if digest != self._last_digest:
                raise ValueError("a completed batch cannot be retried with different observations or commands")
            return self._last_reply
        if sequence != self._next_sequence:
            raise ValueError("batch sequence is out of order")
        if self._pending_digest is not None and digest != self._pending_digest:
            raise ValueError("a pending batch cannot be changed during a retry")
        if self._pending_digest is None:
            if ticks and self._last_timestamp is not None and ticks[0]["timestamp"] < self._last_timestamp:
                raise ValueError("observation timestamps must not go backwards across batches")
            self._pending_digest = digest
            self._pending_applied = 0
        for tick in ticks[self._pending_applied:]:
            self.session.observe(tick["frames"], np.asarray(tick["state"], dtype=np.float64),
                                 tick["prompt"], tick["timestamp"])
            self._pending_applied += 1
            self._last_timestamp = tick["timestamp"]
        reply = self.session.act(command).to_json()
        self._last_digest, self._last_reply = digest, reply
        self._next_sequence += 1
        self._pending_digest = None
        self._pending_applied = 0
        return reply


class BatchedBridgeServer(BridgeServer):
    def __init__(self, session: Session, host="127.0.0.1", port=8210):
        super().__init__(session, host, port)
        # Replace all routes together so ownership has exactly one source.
        self.routes = BatchRoutes(session).routes


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", choices=["127.0.0.1"])
    parser.add_argument("--port", type=int, default=8210)
    args = parser.parse_args(argv)
    with closing(DreamSession()) as session:
        with BatchedBridgeServer(session, args.host, args.port) as server, suppress(KeyboardInterrupt):
            print(f"Batched Dream bridge listening on http://{args.host}:{server.server_port}", flush=True)
            server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
