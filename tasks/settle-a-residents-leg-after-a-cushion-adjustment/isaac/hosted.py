"""Account for the Dream camera warm-up and the provider queue on the Inspect action clock."""

import math

MODEL = "dreamlabs/dream-1-beta"


def require(ok, message):
    if not ok:
        raise ValueError(f"Dream evidence: {message}")


def count(value):
    return type(value) is int and value >= 0


def coverage(doc):
    (sample,) = doc["samples"]
    transcripts = sample.get("policy_transcripts", [])
    require(len(transcripts) == 1, "expected one policy transcript")
    transcript = transcripts[0]
    steps = doc["stats"]["total_steps"]
    require(
        count(steps) and steps == 280, f"expected 280 executed steps, found {steps}"
    )
    require(
        transcript.get("model") == doc["eval"].get("policy_checkpoint") == MODEL,
        "checkpoint differs from requested Dream-1",
    )
    require(transcript.get("dry_run") is False, "expected a live model session")
    require(
        transcript.get("control_hz")
        == doc["eval"]["embodiment_info"]["control_hz"]
        == 20,
        "expected the 20 Hz YAM station",
    )
    # This runner leaves Dream's observation settings at their defaults. The window
    # spans 8 intervals between 9 frames at 5 Hz, or 32 control steps at 20 Hz.
    preroll = transcript.get("preroll_steps")
    require(
        count(preroll) and preroll == math.ceil((9 - 1) * 20 / 5),
        f"unexpected observation warm-up {preroll!r}",
    )
    require(
        sample.get("termination_reasons") == ["max_steps"]
        or (
            sample.get("termination_reasons") == ["success"]
            and sample.get("reduced", {}).get("task_score") == 1
        ),
        "episode did not end at the configured action limit",
    )
    cap = transcript.get("rows_per_request")
    require(count(cap) and 1 <= cap <= 48, f"invalid rows_per_request {cap!r}")
    requests = transcript.get("requests")
    require(isinstance(requests, list) and requests, "missing hosted requests")
    events = sample.get("events", [])
    require(
        len(events) == 1 and isinstance(events[0], list),
        "missing Inspect event timeline",
    )
    step_events = [e for e in events[0] if e["kind"] == "step"]
    inference = [e for e in events[0] if e["kind"] == "inference"]
    timeline = list(range(steps))
    require(
        [e["t"] for e in step_events] == timeline,
        "missing, repeated or reordered station step",
    )
    require(
        [e["t"] for e in inference] == timeline
        and all(e["data"].get("chunk_len") == 1 for e in inference),
        "inference timeline does not cover one action per station step",
    )
    calls = [e for e in inference if e["data"].get("latency_s") is not None]
    require(
        [e["t"] for e in calls] == [r.get("step") for r in requests],
        "provider requests differ from timed Inspect inference events",
    )
    cursor = preroll
    returned = used = discarded = tail = 0
    sessions = set()
    for i, (r, event) in enumerate(zip(requests, calls, strict=True), 1):
        at = r.get("step")
        n = r.get("rows")
        require(
            count(at) and at == cursor and at < steps,
            f"request {i} starts at {at}, expected {cursor} before episode end",
        )
        require(
            count(n) and 1 <= n <= 48, f"request {i} returned invalid row count {n!r}"
        )
        sid = r.get("session_id")
        require(
            isinstance(sid, str) and bool(sid.strip()),
            f"request {i} lacks a hosted session ID",
        )
        dt = r.get("rtt_s")
        require(
            isinstance(dt, (float, int))
            and math.isfinite(dt)
            and dt >= 0
            and dt == event["data"]["latency_s"],
            f"request {i} has inconsistent round-trip timing",
        )
        sessions.add(sid)
        queued = min(n, cap)
        consumed = min(queued, steps - cursor)
        cursor += consumed
        returned += n
        used += consumed
        discarded += n - queued
        tail += queued - consumed
    require(cursor == steps, f"model rows end at step {cursor}; expected {steps}")
    return {
        "control_steps": steps,
        "preroll_steps": preroll,
        "provider_requests": len(requests),
        "hosted_sessions": len(sessions),
        "provider_rows_returned": returned,
        "model_rows_used": used,
        "rows_discarded_by_request_cap": discarded,
        "unused_final_rows": tail,
        "first_request_step": requests[0]["step"],
        "last_request_step": requests[-1]["step"],
    }
