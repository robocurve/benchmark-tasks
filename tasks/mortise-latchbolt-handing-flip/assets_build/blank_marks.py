"""Blank asset-library label/engraving texels (boxes measured on the texture atlases) by interpolating from each box border."""
import sys, numpy as np
from PIL import Image
P = sys.argv[1]
JOBS = {
 "hammer/Textures/t_tool_hammer_a01_alb.png": [(313,198,362,242),(8,474,62,490)],
 "hammer/Textures/t_tool_hammer_a01_nor.png": [(313,198,362,242),(8,474,62,490),(12,306,74,344)],
 "hammer/Textures/t_tool_hammer_a01_orm.png": [(313,198,362,242),(8,474,62,490),(12,306,74,344)],
 "pliers/Textures/t_tool_plier_a01_alb.png": [(314,218,351,306),(144,292,182,408)],
 "pliers/Textures/t_tool_plier_a01_nor.png": [(314,218,351,306),(144,292,182,408)],
 "pliers/Textures/t_tool_plier_a01_orm.png": [(314,218,351,306),(144,292,182,408)],
}
for f, boxes in JOBS.items():
    im = Image.open(f"{P}/{f}"); mode = im.mode; a = np.array(im.convert("RGB")).astype(float)
    for x0, y0, x1, y1 in boxes:
        h, w = y1 - y0, x1 - x0
        ty = np.linspace(0, 1, h)[:, None, None]; tx = np.linspace(0, 1, w)[None, :, None]
        top, bot = a[y0 - 1, x0:x1][None], a[y1, x0:x1][None]; left, right = a[y0:y1, x0 - 1][:, None], a[y0:y1, x1][:, None]
        a[y0:y1, x0:x1] = 0.5 * (top * (1 - ty) + bot * ty) + 0.5 * (left * (1 - tx) + right * tx)
    out = Image.fromarray(a.round().clip(0, 255).astype("uint8"), "RGB")
    (out.convert(mode) if mode != "RGB" else out).save(f"{P}/{f}", optimize=True); print(f, len(boxes))
