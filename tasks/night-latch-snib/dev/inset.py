"""Add a zoomed inset of the latch (from the same top-camera frame) to the setup image.
usage: inset.py <setup.png> <out.png>"""
import sys
from PIL import Image, ImageDraw

img = Image.open(sys.argv[1]).convert("RGB")
k = img.width / 640  # the box is given for the 640x360 top view
box = tuple(int(v * k) for v in (290, 112, 356, 158))  # the case with its knob and snib, and the bolt at the door's edge
z = 3
crop = img.crop(box).resize(((box[2] - box[0]) * z, (box[3] - box[1]) * z), Image.LANCZOS)
x0, y0 = (img.width - crop.width) // 2 - int(8 * k), img.height - crop.height - int(8 * k)
img.paste(crop, (x0, y0))
d = ImageDraw.Draw(img)
d.rectangle((x0 - 1, y0 - 1, x0 + crop.width, y0 + crop.height), outline=(40, 40, 40), width=max(2, int(2 * k)))
d.rectangle(box, outline=(40, 40, 40), width=max(1, int(k)))
img.save(sys.argv[2])
print(img.size, crop.size)
