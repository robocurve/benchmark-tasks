"""Run and replay full Inspect station episodes; resume from completed native records."""

import argparse
import gzip
import hashlib
import json
import os
import re
import subprocess
import sys
import zipfile
from pathlib import Path
import numpy as np
from tools.suite import CASES, ROOT, signature
from task.scoring import score
from isaac.readout import Readout
from tools.hosted_refusal import from_saved as saved_refusal
from isaac.hosted import coverage
from isaac.render import source_frames, main as render


FOLLOWUP = "fixed_50mm_followup"
FOLLOWUP_OPTIONS = {"resident_y": 0.050}
FOLLOWUP_FIXTURE = "runtime/calibration/fixture-5b49a9119bf1.usda"
PREVIOUS_QUALIFIER = "a513082609c7eab6a5f50250f5bf8ca7c799d1316dda921806fd212e80139fd8"
FIRST_RESUME_QUALIFIER = (
    "cb94db407c3c40f46a7f53574d34f3cd2583388ea51f9b3a89e8247fda52a434"
)
COMPARE = ("fixed_partial", "fixed_late_lift", "fixed_later_lift", "fixed_heel_on_foam")


def sha(p):
    h = hashlib.sha256()
    with p.open("rb") as f:
        for buf in iter(lambda: f.read(1048576), b""):
            h.update(buf)
    return h.hexdigest()


def save(p, d):
    t = p.with_name(p.name + ".tmp")
    t.write_text(json.dumps(d, indent=2, allow_nan=False) + "\n")
    t.replace(p)


def check(ok, why):
    if not ok:
        raise ValueError(why)


def sources():
    return dict(
        signature(),
        **{
            x.relative_to(ROOT).as_posix(): sha(x)
            for x in sorted(
                [
                    *(ROOT / "isaac").glob("*.py"),
                    *(ROOT / "isaac/assets").glob("*.usda"),
                    ROOT / FOLLOWUP_FIXTURE,
                ]
            )
        },
    )


def compatible(recorded, current):
    """Only the reporting code and a new, independent fixture may differ on v2 tags."""
    legacy = dict(current)
    legacy.pop(FOLLOWUP_FIXTURE)
    legacy["isaac/qualify.py"] = PREVIOUS_QUALIFIER
    previous = dict(current)
    previous["isaac/qualify.py"] = FIRST_RESUME_QUALIFIER
    last = dict(current)
    last["isaac/qualify.py"] = (
        "5677530c1f52d00328869f725cd18f43c55f4ec81869a1c7c6ab1cb2f78492b1"
    )
    return recorded in (current, previous, legacy, last)


def comparisons(results):
    return {
        name: results[name]["actions_sha256"]
        == results["fixed_nominal"]["actions_sha256"]
        for name in COMPARE
    }


def misses(results):
    return [
        name
        for name, (_, _, target) in CASES.items()
        if (
            results[name]["score"] not in (0, 0.5, 1)
            if name == "fixed_late_lift"
            else results[name]["score"] != target
        )
    ]


def inspect_record(folder):
    (path,) = (folder / "inspect").glob("adhoc_*.json")
    return path


def printable_error(message):
    text = str(message)
    # Error strings can echo signed URLs or HTTP credentials.
    text = re.sub(r"(?i)https?://[^\s<>]+", "<endpoint>", text)
    text = re.sub(r"(?i)(?:Bearer|Basic)\s+[^\s,;]+", "<credential>", text)
    text = re.sub(
        r"""(?i)((?:api[_-]?key|access[_-]?token|password|token|secret)\s*["']?\s*[:=]\s*)["']?[^\s,;}]+""",
        r"\1<redacted>",
        text,
    )
    return " ".join(text.split())[:800]


def inspect_failure(doc):
    """The summary can say success when its only trial raised a policy exception."""
    samples = doc.get("samples", [])
    if (
        not isinstance(samples, list)
        or len(samples) != 1
        or not isinstance(samples[0], dict)
    ):
        return "Expected one Inspect sample. " + printable_error(
            doc.get("error") or doc.get("status")
        )
    sample = samples[0]
    if (
        doc.get("status") == sample.get("status") == "success"
        and not sample.get("error")
        and not doc.get("error")
    ):
        return None
    events = [
        e
        for trial in sample.get("events", [])
        if isinstance(trial, list)
        for e in trial
        if isinstance(e, dict) and e.get("kind") == "error"
    ]
    details = []
    if events:
        e = events[-1]
        data = e.get("data") or {}
        details.append(
            f"step {e.get('t')}, {data.get('type', 'error')}: {data.get('message', '')}"
        )
    details += [
        sample.get("error"),
        doc.get("error"),
        *(sample.get("termination_details") or []),
    ]
    summary = " | ".join(dict.fromkeys(printable_error(msg) for msg in details if msg))
    return (
        f"Inspect run={doc.get('status')}, sample={sample.get('status')}, "
        f"steps={(doc.get('stats') or {}).get('total_steps', '?')}. {summary}"
    ).strip()


def attempt_status(folder):
    logs = list((folder / "inspect").glob("adhoc_*.json"))
    if len(logs) != 1:
        return f"Found {len(logs)} Inspect records. See {folder / 'console.log'}"
    try:
        doc = json.loads(logs[0].read_text())
        if not isinstance(doc, dict):
            raise ValueError("invalid Inspect record")
    except (OSError, ValueError) as exc:
        return "Unreadable Inspect record: " + printable_error(exc)
    return inspect_failure(doc)


def read_commands(folder):
    (path,) = (folder / "inspect").glob("actions/*/*.jsonl")
    entries = [
        a for a in map(json.loads, path.read_text().splitlines()) if "action" in a
    ]
    cmd = np.asarray([r["action"] for r in entries], dtype="<f8")
    check(
        cmd.shape == (280, 14)
        and np.isfinite(cmd).all()
        and [r["t"] for r in entries] == list(range(280)),
        "Incomplete action record",
    )
    return cmd


def read_certified_controls(dest, record, original, current):
    results = record["trials"]
    check(set(results) == set(CASES), "Incomplete saved control checkpoint")
    for name, result in results.items():
        f = dest / name
        check(
            result["mode"] == name
            and result["settings"] == CASES[name][1]
            and compatible(result["source"], current)
            and result == json.loads((f / "receipt.json").read_text()),
            "Saved receipt differs for " + name,
        )
        check(
            sha(f / "trace.jsonl.gz") == result["trace_sha256"]
            and sha(inspect_record(f)) == result["inspect_sha256"],
            "Saved native record differs for " + name,
        )
        for filename, digest in result["media"].items():
            check(
                Path(filename).name == filename
                and sha(f / "media" / filename) == digest,
                "Saved media differs for " + name,
            )
        check(
            hashlib.sha256(read_commands(f).tobytes()).hexdigest()
            == result["actions_sha256"],
            "Saved action sequence differs for " + name,
        )
    check(
        record["missed"] == misses(results)
        and record["identical_commands"] == comparisons(results),
        "Checkpoint disagrees with saved outcomes or actions",
    )
    return dict(results)


def load_window(folder, result):
    from statistics import median

    trace = folder / "trace.jsonl.gz"
    check(sha(trace) == result["trace_sha256"], "Changed partial-relief trace")
    with gzip.open(trace, "rt") as stream:
        next(stream)
        next(stream)
        rows = [json.loads(line) for line in stream]
    check(
        score(rows) == (result["score"], result["reason"]),
        "Partial-relief replay mismatch",
    )
    baseline = median(r["witness"]["heel_channel_n"] for r in rows[:5])
    loads = [r["witness"]["heel_channel_n"] for r in rows[-41:]]
    return dict(
        baseline_n=baseline,
        half_baseline_n=baseline / 2,
        final_heel_n=loads[-1],
        terminal_min_n=min(loads),
        terminal_max_n=max(loads),
        frames_over_half_baseline=sum(v > baseline / 2 for v in loads),
    )


def certify(folder, name, opt, fixture, sig):
    docpath = inspect_record(folder)
    doc = json.loads(docpath.read_text())
    draft = json.loads((ROOT / "task/task.template.json").read_text())
    failure = inspect_failure(doc)
    check(failure is None, f"{name}: {failure}. Record: {docpath}")
    ex = doc["eval"]
    (sample,) = doc["samples"]

    check(
        ex["embodiment"] == "yam_arena"
        and ex["seed"] == 0
        and ex["max_steps"] == doc["stats"]["total_steps"] == 280
        and ex["policy"] == ("dream" if name == "dream" else "heel_scripted")
        and sample["instruction"] == draft["instruction"],
        "Incorrect Inspect run",
    )
    read = Readout(opt)
    rows = []
    trace = folder / "trace.jsonl.gz"
    with gzip.open(trace, "rt") as f:
        hdr = json.loads(next(f))
        check(
            hdr["settings"] == opt
            and hdr["fixture_sha256"] == sha(fixture)
            and abs(hdr["dt"] - 1 / 240) < 1e-7,
            "Trace header mismatch",
        )
        read.sample(json.loads(next(f))["initial"])
        for i, line in enumerate(f):
            row = json.loads(line)
            w, _ = read.sample(row["raw"])
            check(
                w == row["witness"] and i == row["tick"],
                f"Native reading mismatch at {i}",
            )
            rows.append(row)
    val, why = score(rows)
    check(
        val
        == sample["reduced"]["task_score"]
        == doc["results"]["metrics"]["task_score"],
        "Inspect/replay score mismatch",
    )
    a = read_commands(folder)
    check(
        np.allclose(a, [r["action"] for r in rows], atol=1e-6, rtol=0),
        "Different guarded and executed actions",
    )
    views, ids, load, _ = source_frames(folder / "inspect", None)
    check(len(ids) in (280, 281), "Incomplete camera sequence")
    frame_hash = hashlib.sha256()
    for key in sorted(views):
        for i in ids:
            x = load(views[key][i])
            check(
                x.dtype == np.uint8 and x.shape == (352, 640, 3),
                "Incorrect camera frame",
            )
            if key == "top_cam":
                check(np.ptp(x[::8, ::8]) > 25, "Blank overview")
            frame_hash.update(x.tobytes())
    media = folder / "media"
    if (
        not (media / "review_three_cameras.mp4").is_file()
        or not (folder / "receipt.json").is_file()
    ):
        render(
            [
                "--run",
                str(folder / "inspect"),
                "--output",
                str(media),
                "--mode",
                "model" if name == "dream" else "demo",
                "--fps",
                "20",
            ]
        )
    overview = None
    if name in ("reference", "fixed_nominal", "fixed_heel_on_foam"):
        from isaac.framing import check_view

        overview = check_view(
            load(views["top_cam"][ids[-1]]), rows[-1]["witness"], media
        )
    receipt = dict(
        overview=overview,
        mode=name,
        source=sig,
        settings=opt,
        score=val,
        reason=why,
        heel_n=rows[-1]["witness"]["heel_channel_n"],
        gap_mm=rows[-1]["witness"]["heel_clear_mm"],
        actions_sha256=hashlib.sha256(a.tobytes()).hexdigest(),
        trace_sha256=sha(trace),
        inspect_sha256=sha(docpath),
        fixture_sha256=sha(fixture),
        frame_count=len(ids),
        rgb_sha256=frame_hash.hexdigest(),
        media={f.name: sha(f) for f in media.iterdir() if f.is_file()},
    )
    if name == "harmless_bed_touch":
        check(
            any(r["witness"]["bed_touch"] for r in rows[210:240]),
            "Bed-touch control missed contact",
        )
    if name == "park_closed":
        check(
            np.max(np.abs(a[-41:, [6, 13]][:, 1])) <= 0.02,
            "Working jaws did not close while parked",
        )
    if name == "dream":
        receipt["hosted"] = coverage(doc)
    if (folder / "receipt.json").exists():
        check(
            json.loads((folder / "receipt.json").read_text()) == receipt,
            "Native receipt differs on replay",
        )
    else:
        save(folder / "receipt.json", receipt)
    return receipt


def review_archive(dest, results):
    """Keep the carry-back bundle small while preserving the full frame arrays on HPC01."""
    include = [
        dest / name
        for name in (
            "qualification.json",
            "controls.json",
            "sources.json",
            "task.json",
            "continuation.json",
        )
    ]
    for mode in results:
        folder = dest / mode
        refused = results[mode].get("status") == "refused_before_model_action"
        receipt_name = "refusal.json" if refused else "receipt.json"
        include += [folder / "trace.jsonl.gz", folder / receipt_name]
        if refused:
            include.append(folder / "inspect" / results[mode]["inspect_file"])
            for item in results[mode]["previous_refusals"]:
                prev = folder / item["directory"]
                include += [
                    prev / "inspect" / item["inspect_file"],
                    prev / "trace.jsonl.gz",
                ]
        if mode in {
            "reference",
            "fixed_nominal",
            "fixed_partial",
            FOLLOWUP,
            "fixed_heel_on_foam",
            "dream",
        }:
            include += sorted((folder / "media").glob("*"))
    bundle = dest / "qualification-review.zip"
    temp = dest / "qualification-review.inprogress.zip"
    try:
        with zipfile.ZipFile(
            temp, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=1
        ) as z:
            for path in include:
                z.write(path, path.relative_to(dest))
            z.write(ROOT / "isaac/qualify.py", "qualification-code/qualify.py")
            z.write(
                ROOT / "tools/hosted_refusal.py", "qualification-code/hosted_refusal.py"
            )
            if FOLLOWUP in results:
                z.write(ROOT / FOLLOWUP_FIXTURE, FOLLOWUP_FIXTURE)
        temp.replace(bundle)
    finally:
        temp.unlink(missing_ok=True)
    print(f"Station qualification and review media: {bundle}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "phase", choices=("framing", "native", "dream", "qualify", "finalize", "all")
    )
    parser.add_argument("tag", type=str)
    a = parser.parse_args()
    check(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,60}", a.tag), "Invalid run tag")
    dest = ROOT / "results/station" / a.tag
    offline = a.phase in ("finalize", "qualify")
    if a.phase == "finalize":
        check(
            (dest / "controls.json").is_file() and (dest / "sources.json").is_file(),
            "Finalization needs the existing station checkpoint",
        )
    dest.mkdir(exist_ok=True, parents=True)
    sig = sources()
    manifest = dest / "sources.json"
    if manifest.exists():
        original = json.loads(manifest.read_text())
        check(
            compatible(original, sig),
            "Frozen task source differs. Inspect sources.json.",
        )
    else:
        original = sig
        save(manifest, sig)
    results = {}

    def run(name):
        mode, opt, _ = CASES.get(name, ("dream", {}, None))
        if name == FOLLOWUP:
            mode, opt = "clock", FOLLOWUP_OPTIONS
        folder = dest / name
        if offline:
            check(folder.is_dir(), "Missing saved recording for " + name)
        folder.mkdir(exist_ok=True)
        geometry = {k: v for k, v in opt.items() if k != "delay"}
        slug = (
            "fixture-"
            + hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()[
                :12
            ]
        )
        fixture = ROOT / "isaac/assets" / (slug + ".usda")
        if name == FOLLOWUP:
            fixture = ROOT / FOLLOWUP_FIXTURE
        check(
            fixture.is_file()
            and sha(fixture) == sig[fixture.relative_to(ROOT).as_posix()],
            "Missing frozen station fixture",
        )

        def refusal():
            if name != "dream":
                return None
            prompt = json.loads((ROOT / "task/task.template.json").read_text())[
                "instruction"
            ]
            receipt = saved_refusal(folder, fixture, prompt, sha)
            if receipt is not None:
                save(folder / "refusal.json", receipt)
                print(
                    f"Dream-1 refused at step {receipt['warmup_steps']}. No model actions or numeric score.",
                    flush=True,
                )
            return receipt

        previous_failure = attempt_status(folder)
        if name == "dream" and previous_failure:
            refused = refusal()
            if refused is not None:
                return refused
        completed = previous_failure is None
        if not completed:
            check(not offline, f"{name}: {previous_failure}")
            if (folder / "trace.jsonl.gz").exists() or (folder / "inspect").exists():
                import time

                saved = folder / ("attempt-" + str(time.time_ns()))
                saved.mkdir()
                print(
                    f"Preserving {name} in {saved.name}. {previous_failure}", flush=True
                )
                for f in (
                    "trace.jsonl.gz",
                    "inspect",
                    "console.log",
                    "media",
                    "receipt.json",
                ):
                    if (folder / f).exists():
                        (folder / f).rename(saved / f)
            env = dict(
                os.environ,
                HEEL_USD=str(fixture),
                HEEL_TRACE=str(folder / "trace.jsonl.gz"),
                HEEL_MODE=mode,
                HEEL_CONDITION=json.dumps(opt),
                PYTHONDONTWRITEBYTECODE="1",
            )
            policy = "dream" if name == "dream" else "heel_scripted"
            argv = [
                sys.executable,
                "-m",
                "isaac.cli",
                "run",
                "--embodiment",
                "yam_arena",
                "--policy",
                policy,
                "-E",
                "task=isaac/entry.py",
                "-E",
                "object_jitter=0",
                "-E",
                "control_hz=20",
                "-E",
                "cam_width=640",
                "-E",
                "cam_height=352",
                "--instruction",
                json.loads((ROOT / "task/task.template.json").read_text())[
                    "instruction"
                ],
                "--scorer",
                "task_score",
                "--max-steps",
                "280",
                "--seed",
                "0",
                "--no-prompt",
                "--store-frames",
                "--log-dir",
                str(folder / "inspect"),
            ]
            if name == "dream":
                argv += ["-P", "model=dreamlabs/dream-1-beta"]
                url = env.get("DREAM_BRIDGE_URL")
                if url:
                    argv += ["-P", "server_url=" + url]
            print(f"Running {name}. Log: {folder / 'console.log'}", flush=True)
            with (folder / "console.log").open("w") as f:
                ret = subprocess.run(
                    argv, env=env, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT
                )
            failure = attempt_status(folder)
            if ret.returncode != 0 or failure:
                refused = refusal()
                if refused is not None:
                    return refused
                detail = failure or "Inspect process stopped after writing its result"
                raise ValueError(
                    f"{name} exit {ret.returncode}. {detail}. Log: {folder / 'console.log'}"
                )
        recorded = sig
        if completed and (folder / "receipt.json").exists():
            recorded = json.loads((folder / "receipt.json").read_text())["source"]
            check(compatible(recorded, sig), "Recording source differs for " + name)
        result = certify(folder, name, opt, fixture, recorded)
        print(f"{name:24} {result['score']}  {result['reason']}", flush=True)
        return result

    if a.phase == "framing":
        for n in ("fixed_nominal", "fixed_heel_on_foam"):
            run(n)
        print(
            "Inspect each media/demo.mp4 in the primary overview and media/review_three_cameras.mp4 for wrist context."
        )
        return
    if a.phase == "dream":
        run("dream")
        return
    ctl = dest / "controls.json"
    if ctl.is_file() and a.phase != "qualify":
        results = read_certified_controls(
            dest, json.loads(ctl.read_text()), original, sig
        )
        print(
            f"Resuming {a.tag}. Reused {len(results)} certified native controls.",
            flush=True,
        )
    else:
        for name in CASES:
            results[name] = run(name)
        record = dict(
            missed=misses(results),
            identical_commands=comparisons(results),
            trials=results,
        )
        if ctl.is_file():
            check(
                record == json.loads(ctl.read_text()),
                "Replayed controls differ from checkpoint",
            )
        else:
            save(ctl, record)
    missed, matched = misses(results), comparisons(results)
    partial_missed = (
        missed == ["fixed_partial"]
        and results["fixed_partial"]["score"] == 0
        and results["fixed_partial"]["reason"] == "no_sustained_relief"
    )
    check(
        (not missed or partial_missed) and all(matched.values()),
        "Control qualification stopped. Inspect controls.json: " + str(missed),
    )
    evidence = {
        "fixed_partial": load_window(dest / "fixed_partial", results["fixed_partial"])
    }
    confirmed = (
        results["fixed_partial"]["score"] == 0.5
        and evidence["fixed_partial"]["terminal_min_n"] > 0
    )
    if partial_missed:
        results[FOLLOWUP] = run(FOLLOWUP)
        evidence[FOLLOWUP] = load_window(dest / FOLLOWUP, results[FOLLOWUP])
        matched[FOLLOWUP] = (
            results[FOLLOWUP]["actions_sha256"]
            == results["fixed_nominal"]["actions_sha256"]
        )
        confirmed = (
            results[FOLLOWUP]["score"] == 0.5
            and results[FOLLOWUP]["reason"] == "relief_incomplete"
            and evidence[FOLLOWUP]["terminal_min_n"] > 0
            and matched[FOLLOWUP]
        )

    save(
        dest / "continuation.json",
        dict(
            original_sources=original,
            continuation_sources=sig,
            original_expectation_misses=missed,
            original_80mm_target=0.5,
            original_80mm_score=results["fixed_partial"]["score"],
            residual_load=evidence,
            residual_partial_reproduced=confirmed,
            identical_commands=matched,
        ),
    )
    if a.phase in ("all", "qualify", "finalize"):
        results["dream"] = run("dream")
        meta = json.loads((ROOT / "task/task.template.json").read_text())
        extra = (
            f", 50 mm follow-up {results[FOLLOWUP]['score']:g}"
            if FOLLOWUP in results
            else ""
        )
        dream = results["dream"]
        refused = dream.get("status") == "refused_before_model_action"
        if refused:
            dream_score = "Dream-1 refused before any model action; unscored."
            if dream["recording"]["kind"] == "no_recording_available":
                meta["no_model_video_reason"] = dream["exact_provider_error"]
            else:
                dream_score += " Model clip shows camera warm-up only."
        else:
            dream_score = f"Hosted Dream-1 {dream['score']:g}."
        meta["scores"] = (
            f"Seed 0, 280 actions at 20 Hz. Reference {results['reference']['score']:g}, "
            f"heel on foam {results['fixed_heel_on_foam']['score']:g}, 80 mm offset {results['fixed_partial']['score']:g}"
            f"{extra}, linger {results['still_at_cushion']['score']:g}, parked closed {results['park_closed']['score']:g}, "
            f"{dream_score}"
        )
        check(
            all(
                len(meta[k]) <= v
                for k, v in (("scores", 300), ("scoring", 300), ("fidelity", 500))
            ),
            "Metadata length",
        )
        save(dest / "task.json", meta)
        save(
            dest / "qualification.json",
            dict(
                station_replay_passed=True,
                hosted_outcome=dream.get("status", "scored"),
                hosted_rollout_completed=not refused,
                board_media=dict(
                    demo="reference/media/demo.mp4",
                    screenshot="reference/media/setup.png",
                    model_video=None
                    if "no_model_video_reason" in meta
                    else "dream/media/model.mp4",
                    model_video_content="camera warm-up only"
                    if refused and "no_model_video_reason" not in meta
                    else ("none" if refused else "model rollout"),
                ),
                original_matrix_met=not missed,
                original_expectation_misses=missed,
                residual_partial_reproduced=confirmed,
                qualified_with_followup=bool(partial_missed and confirmed),
                identical_commands=matched,
                trials=results,
                media_review="Check heel visibility in overview on both outcomes",
                physical_validation="unmeasured",
            ),
        )
        review_archive(dest, results)
    check(
        confirmed,
        "Residual partial relief was not reproduced. See continuation.json for the measured result.",
    )


if __name__ == "__main__":
    main()
