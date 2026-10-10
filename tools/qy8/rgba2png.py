#!/usr/bin/env python3
# rgba2png.py FILE.rgba W H OUT.png [--alpha]  — GL bottom-up RGBA to PNG
import sys, zlib, struct
src, w, h, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
keep_alpha = '--alpha' in sys.argv
d = open(src, 'rb').read()
rows = []
for y in range(h - 1, -1, -1):
    r = d[y * w * 4:(y + 1) * w * 4]
    if not keep_alpha:
        r = b''.join(r[i:i + 3] for i in range(0, len(r), 4))
    rows.append(b'\0' + r)
def chunk(t, b):
    return struct.pack('>I', len(b)) + t + b + struct.pack('>I', zlib.crc32(t + b) & 0xffffffff)
png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 6 if keep_alpha else 2, 0, 0, 0))
png += chunk(b'IDAT', zlib.compress(b''.join(rows))) + chunk(b'IEND', b'')
open(out, 'wb').write(png)
