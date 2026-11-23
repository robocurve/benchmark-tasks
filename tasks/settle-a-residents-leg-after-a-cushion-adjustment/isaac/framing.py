"""Keep terminal evidence from the native overview, including a calibrated 1:1 crop."""

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
from yam_arena import station
from isaac.camera import configure
from isaac.readout import ORIGIN


def check_view(image, w, out):
    cam = configure(station.rig()).top_cam
    world = np.asarray(w["heel_xyz_mm"]) / 1000 + ORIGIN
    rot = Rotation.from_quat(cam.rot_xyzw).inv()
    h, width = image.shape[:2]
    lens = cam.lens.resized(width, h)

    def project(pt):
        x, y, z = rot.apply(pt - np.asarray(cam.pos))
        if z >= 0:
            raise ValueError("Heel behind overview camera")
        return np.array([lens.cx + lens.fx * x / -z, lens.cy - lens.fy * y / -z])

    center = project(world)
    scale = np.linalg.norm(project(world + [0, 0, 0.010]) - center)
    if scale < 3 or not (54 < center[0] < width - 54 and 52 < center[1] < h - 52):
        raise ValueError(
            f"Heel too small or cropped in overview: {center}, {scale:.2f} px per 10 mm"
        )
    x, y = map(round, center)
    patch = image[y - 50 : y + 50, x - 52 : x + 52]
    q = np.percentile(patch, [5, 95])
    if q[1] - q[0] < 15:
        raise ValueError("Heel region lacks contrast")
    pic = Image.fromarray(image)
    view = pic.copy()
    pen = ImageDraw.Draw(view)
    pen.rectangle((x - 52, y - 50, x + 52, y + 50), outline="#ffda50", width=2)
    sheet = Image.new("RGB", (width + 208, h), "#162632")
    sheet.paste(view, (0, 0))
    sheet.paste(Image.fromarray(patch).resize((208, 200)), (width, 40))
    ImageDraw.Draw(sheet).text(
        (width + 8, 14), "Overview crop | final frame", fill="white"
    )
    sheet.save(out / "overview_heel.png")
    return dict(
        pixel_center=center.tolist(),
        pixels_per_10mm=float(scale),
        local_p05_p95=q.tolist(),
    )
