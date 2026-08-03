"""녹화 프레임 → GIF (포트폴리오용)."""
import glob
import os
import sys

from PIL import Image

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/gazebo-so101-capstone/assets/demo_yolo_pick.gif')
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 3

frames = sorted(glob.glob('/tmp/frames/f_*.png'))[::STEP]
imgs = []
for f in frames:
    im = Image.open(f).convert('P', palette=Image.ADAPTIVE, colors=128)
    im = im.resize((480, 360))
    imgs.append(im)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
imgs[0].save(OUT, save_all=True, append_images=imgs[1:], duration=140, loop=0, optimize=True)
print(f'GIF: {OUT} ({os.path.getsize(OUT) // 1024}KB, {len(imgs)}프레임)')
