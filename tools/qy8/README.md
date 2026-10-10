# QY8 helper tools

Local tooling for the Clarion QY8 (Nissan Leaf ZE1) emulator. Boot it with `../../qy8-shell.sh`.

| tool | what it does |
|---|---|
| `exports.py IMAGE MODULE...` | lists a ROM module's exports as `ordinal address name`. Needs a Windows CE ROM extractor that provides `ximg.py` (not included); point `$NANDX` at its directory. |
| `qmp.py SOCK 'hmp cmd'...` | runs monitor commands over a QMP socket, e.g. `screendump out.png -f png` |
| `shell.py SOCK SECS cmd...` | waits for boot output, then types debug-shell commands and prints the replies |
| `tap.py SOCK X Y` | taps screen pixel X,Y (800x480) through QMP input events; the touch panel model turns it into a touch |
| `rgba2png.py FILE W H OUT` | turns a frame the GL plugin dumped (`out=DIR`) into a PNG |

## Rebuilding the GL symbol table

`contrib/plugins/qy8gl.syms` holds addresses for nav image `G218ENNI.120` and its NK1. Another
firmware version needs a new table:

```
python3 exports.py G218ENNI.flash.img libGLESv2.dll libEGL.dll  > syms
python3 exports.py NK1.bin coredll.dll | grep -E 'CreateDIBSection|CreateBitmap' >> syms
python3 exports.py NK1.bin gdisub.dll | grep DDWaitForBltDone >> syms
```

`NK1.bin` is the NAND dump from offset `0x1c0000` on. The `eglCreateImageKHR` and `REL` extension addresses come from the name-to-function table in
`libIMGEGL.dll`; `glEGLImageTargetTexture2DOES` is caught at runtime from `eglGetProcAddress`.
The four fragment-shader addresses at the top of the renderer in `qy8gl.c` are the
`glShaderBinary` sources inside `auirtdll.dll`.
