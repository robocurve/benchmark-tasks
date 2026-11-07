"""Validate local submission files and measured evidence; never upload or publish.

This checks objective packaging/evidence facts. Reviewer acceptance and novelty
remain judgments; see SUBMISSION_REVIEW.md.
"""

import argparse
import ast
import hashlib
import json
import struct
from pathlib import Path

TASK = Path(__file__).resolve().parent
REPO = TASK.parent.parent
FIELDS = {
    "title",
    "category",
    "instruction",
    "setup",
    "scoring",
    "run_limit",
    "fidelity",
    "scores",
    "media",
}
CATEGORIES = {
    "Impact on jobs",
    "Impact on AI timelines",
    "Safety and alignment",
    "Societal relevance",
    "Other",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def executable_ast(path):
    tree = ast.parse(path.read_text())
    if (
        tree.body
        and isinstance(tree.body[0], ast.Expr)
        and isinstance(tree.body[0].value, ast.Constant)
        and isinstance(tree.body[0].value.value, str)
    ):
        tree.body.pop(0)
    return ast.dump(tree, include_attributes=False)


def mp4_boxes(path):
    out = []
    data = path.read_bytes()
    offset = 0
    while offset < len(data):
        size, name = struct.unpack_from(">I4s", data, offset)
        if size == 1:
            size = struct.unpack_from(">Q", data, offset + 8)[0]
        elif size == 0:
            size = len(data) - offset
        if size < 8 or offset + size > len(data):
            raise ValueError(f"Malformed MP4 atom in {path.name}")
        out.append(name.decode("ascii"))
        offset += size
    return out


def validate(media, evidence, allow_pending_media=False):
    spec = json.loads((TASK / "task.json").read_text())
    for key, value in spec.items():
        section = (
            value if isinstance(value, str) else json.dumps(value, ensure_ascii=False)
        )
        if len(section) > 500:
            raise ValueError(f"{key} exceeds 500 characters ({len(section)})")
    required = FIELDS - ({"media"} if allow_pending_media else set())
    for key in required:
        if not isinstance(spec.get(key), str) or not spec[key].strip():
            raise ValueError(f"Missing required field: {key}")
    if spec["category"] not in CATEGORIES:
        raise ValueError("Invalid category")
    if any("TODO" in str(v) for v in spec.values()):
        raise ValueError("Unfinished metadata")
    if spec.get("media") and not spec["media"].startswith("M-"):
        raise ValueError("Media ID must be returned by the official uploader")
    for name in [
        "README.md",
        "task.py",
        "run.py",
        "validation.json",
        "SUBMISSION_REVIEW.md",
    ]:
        if not (TASK / name).is_file():
            raise ValueError(f"Missing {name}")

    facts = json.loads((TASK / "validation.json").read_text())
    evaluated = evidence / "evaluated-task.py"
    if digest(TASK / "task.py") != facts["prepared_task_sha256"]:
        raise ValueError("Prepared source hash differs from audited source")
    if digest(evaluated) != facts["evaluated_task_sha256"]:
        raise ValueError("Evaluated source hash differs from trial evidence")
    if executable_ast(TASK / "task.py") != executable_ast(evaluated):
        raise ValueError("Executable code changed after evaluated trials")
    if digest(TASK / "run.py") != facts["evaluated_runner_sha256"]:
        raise ValueError("Runner changed after evaluated demonstration")
    for key, name in [("scripted_demo", "demo.json"), ("dream_1", "dream.json")]:
        log = json.loads((evidence / name).read_text())
        trial = facts["trials"][key]
        if log.get("error") or log["results"]["errored_trials"]:
            raise ValueError(f"{name} contains trial errors")
        if log["results"]["metrics"]["task_score"] != trial["score"]:
            raise ValueError(f"{name} score mismatch")
        if log["eval"]["seed"] != 0 or log["eval"]["max_steps"] != 2400:
            raise ValueError(f"{name} seed or limit mismatch")
        if log["stats"]["total_steps"] > 2400:
            raise ValueError(f"{name} exceeds horizon")
        if (
            log["eval"]["environment_revision"] != trial["environment_revision"]
            or trial["environment_revision"] != facts["evaluated_task_sha256"][:12]
        ):
            raise ValueError(f"{name} evaluated revision mismatch")
        if log["samples"][0]["instruction"] != spec["instruction"]:
            raise ValueError(f"{name} instruction differs from task.json")
        if log["stats"]["total_steps"] != trial["control_steps"]:
            raise ValueError(f"{name} recorded step count mismatch")

    model_trace = [
        json.loads(x)
        for x in (evidence / "model-safety.jsonl").read_text().splitlines()
    ]
    if (
        len(model_trace) != facts["trials"]["dream_1"]["control_steps"]
        or any(
            row["task_revision"] != facts["evaluated_task_sha256"]
            for row in model_trace
        )
        or model_trace[-1]["score"] != facts["trials"]["dream_1"]["score"]
    ):
        raise ValueError(
            "Model trace does not match the final evaluated task and score"
        )
    if facts["trials"]["scripted_demo"]["score"] != 1.0:
        raise ValueError("Full-score demonstration missing")
    trace = [
        json.loads(x)
        for x in (evidence / "bottle-safety.jsonl").read_text().splitlines()
    ]
    if (
        len(trace) != facts["trials"]["scripted_demo"]["control_steps"]
        or any(row["safety_violations"] for row in trace)
        or not trace[-1]["success"]
        or trace[-1]["sorted_count"] != 6
        or not trace[-1]["stable_window"]
        or not trace[-1]["grippers_clear"]
        or any(row["task_revision"] != facts["evaluated_task_sha256"] for row in trace)
    ):
        raise ValueError("Demo safety trace does not demonstrate safe full success")

    manifest = json.loads((evidence / "media-manifest.json").read_text())
    for name, limit in [
        ("demo.mp4", 50_000_000),
        ("model_run.mp4", 50_000_000),
        ("setup.png", 10_000_000),
    ]:
        path = media / name
        if not path.is_file() or not 0 < path.stat().st_size <= limit:
            raise ValueError(f"Missing or oversized {name}")
        if digest(path) != manifest["files"][name]["sha256"]:
            raise ValueError(f"Audited media hash mismatch: {name}")
        if path.suffix == ".mp4":
            boxes = mp4_boxes(path)
            if boxes.index("moov") > boxes.index("mdat"):
                raise ValueError(f"{name} is not faststart")
            item = manifest["files"][name]
            if item["codec"] != "h264" or item["pixel_format"] != "yuv420p":
                raise ValueError(f"Invalid codec or pixel format: {name}")
            trial_key = "scripted_demo" if name == "demo.mp4" else "dream_1"
            if (
                item["decoded_frames"] != item["frames"]
                or item["frames"] != facts["trials"][trial_key]["control_steps"] + 1
            ):
                raise ValueError(
                    f"Video decode/frame count does not match trial: {name}"
                )
    print(
        "PASS: metadata, source equivalence, measured scores, safety trace, media hashes and format."
    )
    if not spec.get("media"):
        print("PENDING: actual media ID from official upload.")
    print(
        "REVIEW: distinctness rationale documented after board check; acceptance is not guaranteed."
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--media-dir",
        type=Path,
        default=REPO / ".local/media/safe-glass-bottle-sorting",
    )
    parser.add_argument(
        "--evidence-dir",
        type=Path,
        default=REPO / ".local/submissions/safe-glass-bottle-sorting-evidence",
    )
    parser.add_argument("--allow-pending-media", action="store_true")
    args = parser.parse_args()
    try:
        validate(args.media_dir, args.evidence_dir, args.allow_pending_media)
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f"Submission check failed: {error}\n")
