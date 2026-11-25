"""bandscan.py FRAMES_DIR EVERY [OUT.png]: hard black/bright banding per camera every EVERY steps (stored .npy frames):
the share of pixels that are near-black (<15) with a bright pixel (>140) 2-4 px above or below, i.e. thin black bands on
lit surfaces, as the ray-traced shadow artifact draws them. Also writes a contact sheet of the sampled frames."""
import glob, os, sys
import numpy as np
from PIL import Image
root = sys.argv[1]
D = (glob.glob(os.path.join(root, "*/")) or [root + "/"])[0]
every = int(sys.argv[2]); out = sys.argv[3] if len(sys.argv) > 3 else None
cams = ["a_top_cam", "a_left_cam", "a_right_cam", "b_top_cam", "b_left_cam", "b_right_cam", "overview_cam"]
steps = sorted(int(f.rsplit("_", 1)[1][:-4]) for f in glob.glob(D + "*a_top_cam_*.npy"))
def band(a):
    g = a[..., :3].astype(np.float32).mean(axis=2)
    dark = g < 15
    bright = np.zeros_like(dark)
    for k in (2, 3, 4):
        bright[k:] |= g[:-k] > 140
        bright[:-k] |= g[k:] > 140
    return float((dark & bright).mean())
rows, sheet = [], []
print("step  " + " ".join(f"{c[:8]:>9s}" for c in cams))
for s in list(range(steps[0], steps[-1] + 1, every)) + ([steps[-1]] if steps[-1] % every else []):
    vals, ims = [], []
    for c in cams:
        f = D + f"scene-0-e0_{c}_{s:06d}.npy"
        if not os.path.exists(f):
            vals.append(float("nan")); continue
        a = np.load(f)
        vals.append(band(a))
        if c in ("a_top_cam", "a_right_cam", "b_left_cam", "overview_cam"):
            ims.append(Image.fromarray(a[..., :3].astype(np.uint8)).resize((240, 135)))
    rows.append((s, vals)); sheet.append(ims)
    print(f"{s:5d} " + " ".join(f"{v:9.4f}" for v in vals))
worst = max(max(v for v in vals if v == v) for _, vals in rows)
print("max banding", round(worst, 4))
if out:
    W = 240 * 4; H = 135 * len(sheet)
    img = Image.new("RGB", (W, H))
    for i, ims in enumerate(sheet):
        for j, im in enumerate(ims):
            img.paste(im, (j * 240, i * 135))
    img.save(out)
