#!/usr/bin/env python3
# exports.py IMAGE MODULE...  — print each ROM module's exports as ordinal, VA, name
import os, sys, struct
sys.path.insert(0, os.environ.get('NANDX', os.path.expanduser('~/nandx')))
from ximg import u32, u16, cstr, romhdr, toc, extract
import io, contextlib, tempfile

d = open(sys.argv[1], 'rb').read()
hdr, delta = romhdr(d)
mods, _ = toc(d, hdr, delta)
tmp = tempfile.mkdtemp()
for want in sys.argv[2:]:
    for n, s, e32o, o32o in mods:
        if not n or n.lower() != want.lower():
            continue
        e32 = e32o + delta
        vbase = u32(d, e32 + 8)
        with contextlib.redirect_stdout(io.StringIO()):
            extract(d, delta, n, e32o, o32o, tmp)
        img = open(f'{tmp}/{n}.flat.bin', 'rb').read()
        def rva(x): return x - vbase if x >= vbase else x
        # find the EXP unit: first (rva,size) pair whose directory names this module
        for uo in range(0x18, 0x40, 4):
            er = u32(d, e32 + uo)
            if not er or rva(er) + 40 > len(img):
                continue
            nm = cstr(img, rva(u32(img, rva(er) + 12)))
            # the directory may name the module without its extension
            if nm and nm.lower().split('.dll')[0] == n.lower().split('.dll')[0]:
                break
        else:
            print(n, 'no export dir'); continue
        e = rva(er)
        base, nfun, nnam = u32(img, e+16), u32(img, e+20), u32(img, e+24)
        af, an, ao = rva(u32(img, e+28)), rva(u32(img, e+32)), rva(u32(img, e+36))
        names = {}
        for k in range(nnam):
            names[u16(img, ao + 2*k)] = cstr(img, rva(u32(img, an + 4*k)))
        print(f'# {n} vbase {vbase:#x} exports {nfun} named {nnam} ordinal base {base}')
        for k in range(nfun):
            a = u32(img, af + 4*k)
            if a:
                print(f'{base+k:4d} {vbase + rva(a):#010x} {names.get(k, "")}')
