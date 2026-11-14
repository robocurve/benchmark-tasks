#!/usr/bin/env python3
"""Uploads a task's videos and screenshot to the Robocurve task board and records the media ID in task.json.

    export ROBOCURVE_TEAM=T-17                 # your team code
    export ROBOCURVE_TOKEN=...                 # your team's access token (never commit it)
    python tools/upload_media.py tasks/lamp-task --video model_run.mp4 --demo demo.mp4 --screenshot setup.png

Videos must be H.264 MP4 (convert with: ffmpeg -i in.mp4 -c:v libx264 -pix_fmt yuv420p -movflags +faststart out.mp4),
up to 50 MB each; the screenshot up to 10 MB. The demo is required when the model run doesn't reach full score.
Uses only the Python standard library.
"""

import argparse, json, mimetypes, os, sys, urllib.error, urllib.request, uuid
from pathlib import Path

BOARD = os.environ.get("ROBOCURVE_BOARD", "https://tasks.robocurve.org")


def multipart(fields, files):
    boundary = uuid.uuid4().hex
    out = bytearray()
    for name, value in fields.items():
        out += f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode()
    for name, path in files.items():
        kind = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        out += (f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"; filename="{path.name}"\r\n'
                f"Content-Type: {kind}\r\n\r\n").encode() + path.read_bytes() + b"\r\n"
    out += f"--{boundary}--\r\n".encode()
    return bytes(out), f"multipart/form-data; boundary={boundary}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task_dir", help="the task folder, like tasks/lamp-task")
    ap.add_argument("--video", help="the model run: Dream-1, or for robots other than two parallel-gripper arms, an LLM through Inspect Robots. "
                    "Leave it out only if running the model produced no recording (for example, the model refused the instruction "
                    "or errored before acting), and say what happened, with the exact error text, in task.json's no_model_video_reason. "
                    "Lack of time, budget or compute credits is not a reason")
    ap.add_argument("--screenshot", required=True, help="the setup screenshot")
    ap.add_argument("--demo", help="the full-score demo (required unless the Dream-1 run reaches full score)")
    a = ap.parse_args()
    team, token = os.environ.get("ROBOCURVE_TEAM"), os.environ.get("ROBOCURVE_TOKEN")
    if not team or not token:
        sys.exit("Set ROBOCURVE_TEAM (your team code) and ROBOCURVE_TOKEN (your team's access token) first.")
    task_dir = Path(a.task_dir)
    if not a.video and not a.demo:
        sys.exit("Without a model run video (--video), the full-score demo (--demo) is required.")
    files = {"screenshot": Path(a.screenshot)}
    if a.video:
        files["video"] = Path(a.video)
    if a.demo:
        files["demo"] = Path(a.demo)
    for name, path in files.items():
        if not path.is_file():
            sys.exit(f"{name}: {path} not found")
    body, ctype = multipart({"task": task_dir.name}, files)
    req = urllib.request.Request(f"{BOARD}/api/media", data=body, method="POST",
                                 headers={"Authorization": f"Bearer {team}:{token}", "Content-Type": ctype,
                                          # Cloudflare blocks Python's default user agent (error 1010), so name the client.
                                          "User-Agent": "robocurve-upload-media/1.2"})
    try:
        with urllib.request.urlopen(req, timeout=600) as res:
            media_id = json.load(res)["media_id"]
    except urllib.error.HTTPError as e:
        try:
            msg = json.loads(e.read().decode()).get("error")
        except Exception:
            msg = None
        sys.exit(f"Upload failed ({e.code}): {msg or e.reason}")
    spec_path = task_dir / "task.json"
    if spec_path.is_file():
        spec = json.loads(spec_path.read_text())
        spec["media"] = media_id
        spec_path.write_text(json.dumps(spec, indent=2, ensure_ascii=False) + "\n")
        print(f"Uploaded. Media ID {media_id} written to {spec_path}. Commit it and open your pull request.")
        if not a.video and not spec.get("no_model_video_reason"):
            print("No model run video: add \"no_model_video_reason\" to task.json with what happened and the exact error text "
                  "(for example, a refusal). Lack of time, budget or compute credits is not a reason, and the task will be rejected.")
    else:
        print(f"Uploaded. Media ID: {media_id} (add it to task.json as \"media\").")


if __name__ == "__main__":
    main()
