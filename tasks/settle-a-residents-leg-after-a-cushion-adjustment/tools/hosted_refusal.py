"""Qualify a recorded pre-action Dream safety refusal without contacting the model."""

import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from isaac.hosted import MODEL
from isaac.readout import Readout
from isaac.render import main as render, source_frames


def check(ok, message):
    if not ok:
        raise ValueError("Dream refusal evidence: " + message)


def refusal_event(doc):
    samples = doc.get("samples") or []
    if len(samples) != 1 or samples[0].get("status") != "error":
        return None
    timeline = samples[0].get("events") or []
    if len(timeline) != 1 or not timeline[0]:
        return None
    event = timeline[0][-1]
    data = event.get("data") or {}
    message = data.get("message", "")
    if (
        event.get("kind") == "error"
        and data.get("type") == "PolicyError"
        and message.startswith(
            "DreamAPI refused the inference request: DA-4000 BAD_REQUEST"
        )
        and "unsafe instruction" in message.lower()
        and "refused" in message.lower()
    ):
        return event
    return None


def inspect_evidence(doc, prompt):
    event = refusal_event(doc)
    check(event is not None, "Missing explicit provider safety refusal")
    sample = doc["samples"][0]
    spec = doc["eval"]
    (transcript,) = sample["policy_transcripts"]
    check(
        spec["embodiment"] == "yam_arena"
        and spec["policy"] == "dream"
        and spec["policy_checkpoint"] == transcript["model"] == MODEL
        and spec["seed"] == 0
        and spec["max_steps"] == 280
        and sample["instruction"] == prompt
        and transcript["control_hz"] == spec["embodiment_info"]["control_hz"] == 20
        and transcript["dry_run"] is False,
        "Wrong task, policy, or episode configuration",
    )
    check(
        sample.get("reduced") == {}
        and doc["results"].get("metrics") == {}
        and doc["results"].get("errored_trials") == 1,
        "Inspect score or error count conflicts with refusal",
    )
    events = sample["events"][0]
    steps = doc["stats"]["total_steps"]
    check(
        steps == transcript["preroll_steps"] == event["t"] == 32
        and transcript["requests"] == [],
        "Provider failure was outside the pre-action warm-up",
    )
    expected = list(range(steps))
    check(
        [e["t"] for e in events if e["kind"] == "step"] == expected,
        "Incomplete warm-up",
    )
    inference = [e for e in events if e["kind"] == "inference"]
    check(
        [e["t"] for e in inference] == expected
        and all(
            e["data"].get("latency_s") is None and e["data"].get("chunk_len") == 1
            for e in inference
        ),
        "Recorded inference activity before refusal",
    )
    error = event["data"]["message"]
    check(error in sample["error"], "Sample and event errors differ")
    return dict(
        status="refused_before_model_action",
        score=None,
        model=MODEL,
        warmup_steps=steps,
        model_actions=0,
        exact_provider_error=error,
        inspect_sample_error=sample["error"],
    )


def record_attempt(record, fixture, prompt, sha):
    folder = record.parent.parent
    doc = json.loads(record.read_text())
    outcome = inspect_evidence(doc, prompt)
    trace = folder / "trace.jsonl.gz"
    with gzip.open(trace, "rt") as stream:
        header = json.loads(next(stream))
        initial = json.loads(next(stream))["initial"]
        rows = list(map(json.loads, stream))
    check(
        header["settings"] == {}
        and header["fixture_sha256"] == sha(fixture)
        and abs(header["dt"] - 1 / 240) < 1e-7,
        "Trace fixture or physics differs",
    )
    resets = [e for e in doc["samples"][0]["events"][0] if e["kind"] == "reset"]
    check(
        len(resets) == 1 and header["seed"] == resets[0]["data"]["seed"],
        "Trace uses a different reset",
    )
    check(
        [r["tick"] for r in rows] == list(range(outcome["warmup_steps"])),
        "Incomplete trace",
    )
    read = Readout({})
    read.sample(initial)
    for row in rows:
        witness, _ = read.sample(row["raw"])
        check(witness == row["witness"], "Raw frame and measured state disagree")
    commands = np.asarray([r["action"] for r in rows], dtype="<f8")
    check(
        commands.shape == (32, 14)
        and np.isfinite(commands).all()
        and np.all(commands == commands[0]),
        "Warm-up targets were not a constant hold",
    )
    return dict(
        **outcome,
        inspect_file=record.name,
        inspect_sha256=sha(record),
        trace_sha256=sha(trace),
        actions_sha256=hashlib.sha256(commands.tobytes()).hexdigest(),
        fixture_sha256=header["fixture_sha256"],
    )


def saved_media(folder, steps, sha):
    frames_dir = folder / "inspect" / "frames"
    media = folder / "media"
    model = media / "model.mp4"
    if any(frames_dir.rglob("*.npy")):
        views, ids, load, _ = source_frames(folder / "inspect", None)
        check(
            1 <= len(ids) <= steps + 1,
            "Warm-up camera count differs from the trace",
        )
        for names in views.values():
            for i in ids:
                rgb = load(names[i])
                check(
                    rgb.dtype == np.uint8 and rgb.shape == (352, 640, 3),
                    "Invalid native warm-up image",
                )
        render(
            [
                "--run",
                str(folder / "inspect"),
                "--output",
                str(media),
                "--mode",
                "model",
                "--fps",
                "20",
            ]
        )
        return dict(kind="warmup_only", frames=len(ids), media_sha256=sha(model))
    if model.is_file():
        # Some exports retain only the encoded recording. Verify its stream and duration.
        import subprocess

        probe = subprocess.check_output(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=codec_name,width,height,nb_frames,avg_frame_rate",
                "-of",
                "json",
                str(model),
            ],
            text=True,
        )
        (stream,) = json.loads(probe)["streams"]
        from fractions import Fraction

        count = int(stream["nb_frames"])
        check(
            stream["codec_name"] == "h264"
            and (stream["width"], stream["height"]) == (640, 352)
            and Fraction(stream["avg_frame_rate"]) == 20
            and 1 <= count <= steps + 1,
            "Existing recording does not match this run",
        )
        return dict(kind="retained_warmup_clip", frames=count, media_sha256=sha(model))
    other_clips = list(media.glob("*.mp4"))
    check(
        not other_clips,
        "Found other recordings. Restore the original frame arrays or model.mp4 for review.",
    )
    doc = json.loads(next((folder / "inspect").glob("adhoc_*.json")).read_text())
    declared = doc["stats"].get("frames_dir")
    if declared:
        expected = frames_dir / Path(declared).name
        check(
            expected.is_dir(),
            "Inspect identifies a saved frame directory. Restore it before exporting media.",
        )
    return dict(kind="no_recording_available", frames=0)


def from_saved(folder, fixture, prompt, sha):
    """Return None for other failures so the ordinary qualification path handles them."""
    current = list((folder / "inspect").glob("adhoc_*.json"))
    if len(current) != 1:
        return None
    try:
        doc = json.loads(current[0].read_text())
    except (OSError, ValueError):
        return None
    if not refusal_event(doc):
        return None
    receipt = record_attempt(current[0], fixture, prompt, sha)
    earlier = []
    for path in sorted(folder.glob("attempt-*/inspect/adhoc_*.json")):
        prior = json.loads(path.read_text())
        if refusal_event(prior):
            previous = record_attempt(path, fixture, prompt, sha)
            previous["directory"] = path.parent.parent.name
            earlier.append(previous)
    receipt["previous_refusals"] = earlier
    receipt["recording"] = saved_media(folder, receipt["warmup_steps"], sha)
    receipt["qualification_code_sha256"] = sha(Path(__file__))
    return receipt
