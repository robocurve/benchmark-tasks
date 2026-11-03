"""No-key tests: full ordered replay, episode ownership and retry semantics."""

import json
import threading
import unittest
from dataclasses import asdict

import numpy as np

from dream_bridge import BatchRoutes, BatchedBridgeServer, BatchedBridgeSession
from inspect_robots_dream.session import Chunk, Episode


FRAMES = {name: "jpeg-" + name for name in ("top_cam", "left_cam", "right_cam")}


class FakeSession:
    def __init__(self):
        self.calls = []
        self.fail_next_act = False

    def check(self, episode):
        self.calls.append(("check", episode))

    def start(self, episode):
        self.calls.append(("start", episode))

    def observe(self, frames, state, prompt, timestamp):
        self.calls.append(("observe", dict(frames), state.tolist(), prompt, timestamp))

    def act(self, commanded):
        self.calls.append(("act", None if commanded is None else commanded.tolist()))
        if self.fail_next_act:
            self.fail_next_act = False
            raise RuntimeError("temporary inference error")
        return Chunk(np.arange(48 * 14, dtype=float).reshape(48, 14), .4, 1, 23.0, "fake-session")


class FakeTransport:
    def __init__(self, server):
        self.server = server
        self.requests = []
        self.lose_reply = False

    def __call__(self, url, body, timeout):
        # Actual JSON round trips catch numpy scalars/arrays and mutable aliases.
        body = json.loads(json.dumps(body, allow_nan=False))
        self.requests.append((url, body, timeout))
        reply = self.server.routes["/" + url.rsplit("/", 1)[1]](body)
        if self.lose_reply and url.endswith("/act_batch"):
            self.lose_reply = False
            raise ConnectionError("reply lost after server completed")
        return json.loads(json.dumps(reply, allow_nan=False))


class BatchedBridgeTests(unittest.TestCase):
    def setUp(self):
        self.session = FakeSession()
        self.server = BatchRoutes(self.session)
        self.transport = FakeTransport(self.server)
        self.client = BatchedBridgeSession("http://127.0.0.1:8210", post=self.transport)
        self.client.start(Episode())

    def observe(self, t, value=0):
        self.client.observe(FRAMES, np.full(14, value), "roll the die", t)

    def test_preroll_and_48_tick_batches_preserve_all_observations_and_commands(self):
        for i in range(33):
            self.observe(i / 20, i)
        self.assertEqual(len(self.transport.requests), 1)  # start only
        chunk = self.client.act(None)
        self.assertEqual(chunk.rows.shape, (48, 14))
        self.assertEqual(chunk.session_id, "fake-session")
        for i in range(33, 81):
            self.observe(i / 20, i)
        commanded = np.arange(14, dtype=float)
        self.client.act(commanded)
        observations = [c for c in self.session.calls if c[0] == "observe"]
        self.assertEqual([c[4] for c in observations], [i / 20 for i in range(81)])
        self.assertEqual([c[2] for c in observations], [[i] * 14 for i in range(81)])
        self.assertEqual([c[3] for c in observations], ["roll the die"] * 81)
        self.assertEqual(self.session.calls[34], ("act", None))
        self.assertEqual(self.session.calls[-1], ("act", commanded.tolist()))
        self.assertEqual(len(self.transport.requests), 3)  # start + two batches

    def test_observations_are_snapshots_not_mutable_references(self):
        state, frames = np.ones(14), dict(FRAMES)
        self.client.observe(frames, state, "original", 0.)
        state[:] = 9
        frames["top_cam"] = "changed"
        self.client.act(None)
        self.assertEqual(self.session.calls[1][1], FRAMES)
        self.assertEqual(self.session.calls[1][2], [1.] * 14)

    def test_reset_discards_pending_ticks_and_allows_time_to_restart(self):
        self.observe(20.)
        self.client.start(Episode())
        self.observe(0., 7)
        self.client.act(None)
        observations = [c for c in self.session.calls if c[0] == "observe"]
        self.assertEqual(len(observations), 1)
        self.assertEqual(observations[0][4], 0.)

    def test_new_episode_rejects_an_old_owner_before_replaying(self):
        self.observe(0.)
        self.server.start(asdict(Episode()))
        before = list(self.session.calls)
        with self.assertRaisesRegex(RuntimeError, "not the one"):
            self.client.act(None)
        self.assertEqual(self.session.calls, before)

    def test_lost_reply_retry_does_not_repeat_observations_or_inference(self):
        self.observe(0.)
        self.transport.lose_reply = True
        with self.assertRaises(ConnectionError):
            self.client.act(np.zeros(14))
        before = list(self.session.calls)
        with self.assertRaises(ValueError):
            self.client.act(np.ones(14))
        chunk = self.client.act(np.zeros(14))
        self.assertEqual(chunk.rows.shape, (48, 14))
        self.assertEqual(self.session.calls, before)
        self.observe(.05)
        self.client.act(np.ones(14))

    def test_inference_error_retry_does_not_replay_ticks(self):
        self.observe(0.)
        self.session.fail_next_act = True
        with self.assertRaises(RuntimeError):
            self.client.act(None)
        with self.assertRaises(RuntimeError):
            self.observe(.05)
        self.client.act(None)
        self.assertEqual(sum(c[0] == "observe" for c in self.session.calls), 1)
        self.assertEqual(sum(c[0] == "act" for c in self.session.calls), 2)

    def test_invalid_batch_is_rejected_before_any_tick_is_replayed(self):
        self.observe(.1)
        self.observe(.2)
        # Capture a legitimate request, then test a reversed sequence in a new episode.
        self.client.act(None)
        body = self.transport.requests[-1][1]
        body["episode"] = self.server.start(asdict(Episode()))["episode"]
        body["ticks"].reverse()
        before = list(self.session.calls)
        with self.assertRaises(ValueError):
            self.server.act_batch(body)
        self.assertEqual(self.session.calls, before)

    def test_backward_time_and_nonfinite_state_do_not_enter_queue(self):
        self.observe(.1)
        with self.assertRaises(ValueError):
            self.observe(.05)
        with self.assertRaises(ValueError):
            self.client.observe(FRAMES, np.full(14, np.nan), "bad", .2)
        self.client.act(None)
        self.assertEqual(sum(c[0] == "observe" for c in self.session.calls), 1)

    def test_real_http_handler_transport_and_chunk_serialization(self):
        with BatchedBridgeServer(self.session, "127.0.0.1", 0) as server:
            thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
            thread.start()
            try:
                client = BatchedBridgeSession(f"http://127.0.0.1:{server.server_port}")
                client.check(Episode())
                client.start(Episode())
                client.observe(FRAMES, np.arange(14), "HTTP test", 0.)
                result = client.act(np.arange(14))
                np.testing.assert_array_equal(result.rows, np.arange(48 * 14).reshape(48, 14))
                self.assertEqual(result.infer_ms, 23.)
                self.assertEqual(self.session.calls[-1], ("act", np.arange(14).tolist()))
            finally:
                server.shutdown()
                thread.join(timeout=2)

    def test_real_dream_frame_window_matches_unbatched_delivery(self):
        from dream._window import Window

        class WindowSession(FakeSession):
            def __init__(self):
                super().__init__()
                self.window = Window(9, .2)
                self.snapshots = []

            def start(self, episode):
                super().start(episode)
                self.window.clear()

            def observe(self, frames, state, prompt, timestamp):
                self.window.add(frames, {"state": state.tolist(), "prompt": prompt}, timestamp)

            def act(self, commanded):
                self.snapshots.append(self.window.build_observation())
                return super().act(commanded)

        direct, batched = WindowSession(), WindowSession()
        client = BatchedBridgeSession("http://local", post=FakeTransport(BatchRoutes(batched)))
        direct.start(Episode())
        client.start(Episode())
        for i in range(81):
            frames = {name: f"frame-{i}-{name}" for name in FRAMES}
            state = np.full(14, i)
            prompt = "first instruction" if i < 44 else "latest instruction"
            direct.observe(frames, state, prompt, i / 20)
            client.observe(frames, state, prompt, i / 20)
            if i in (32, 80):
                direct.act(None)
                client.act(None)
        self.assertEqual(direct.snapshots, batched.snapshots)
        self.assertEqual(len(batched.snapshots[-1]["top_cam"]), 9)


if __name__ == "__main__":
    unittest.main()
