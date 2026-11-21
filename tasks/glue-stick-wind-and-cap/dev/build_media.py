"""Build the H.264 demo video (top camera, left wrist camera, right wrist camera side by side) from a run's stored
frames, and optionally the setup image (the top camera's first frame).
usage: build_media.py <run folder> <out.mp4> [setup.png]"""
import glob, struct, subprocess, sys, zlib
import numpy as np

run, out = sys.argv[1], sys.argv[2]


def files(cam):
    return sorted(glob.glob(f"{run}/frames/*/*{cam}_*.npy"), key=lambda f: int(f.rsplit("_", 1)[1][:-4]))


cams = [files(c) for c in ("top_cam", "left_cam", "right_cam")]
n = min(len(c) for c in cams)
load = lambda f: np.load(f)[..., :3].astype(np.uint8)
first = load(cams[0][0])
h, w = first.shape[:2]
p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{3 * w}x{h}",
                      "-r", "20", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                      "-movflags", "+faststart", out], stdin=subprocess.PIPE)
last = None
for i in range(n):
    last = np.hstack([load(c[i]) for c in cams])
    p.stdin.write(last.tobytes())
for _ in range(40):  # hold the last frame for 2 s
    p.stdin.write(last.tobytes())
p.stdin.close()
p.wait()
print(out, n, "frames", f"{3 * w}x{h}", "exit", p.returncode)
if len(sys.argv) > 3:
    img = first
    raw = b"".join(b"\x00" + row.tobytes() for row in img)
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d))
    open(sys.argv[3], "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", img.shape[1], img.shape[0], 8, 2, 0, 0, 0))
                                   + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
    print(sys.argv[3], img.shape)
if len(sys.argv) > 4:  # a still of all three cameras at a given frame index, for checking
    k = min(int(sys.argv[4]), n - 1)
    img = np.hstack([load(c[k]) for c in cams])
    raw = b"".join(b"\x00" + row.tobytes() for row in img)
    open(sys.argv[3].replace(".png", f"-all{k}.png"), "wb").write(
        b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", img.shape[1], img.shape[0], 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
