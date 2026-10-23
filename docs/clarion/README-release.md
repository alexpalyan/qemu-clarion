# Clarion head-unit emulator (binary package)

Layout:

- `qemu-clarion`: launcher; reads the unit model from the PROD block of the
  NOR image and starts the matching engine.
- `libexec/qemu-system-arm`: engine for QY8 units.
- `libexec/qemu-system-sh4`: engine for the QY7221NL.
- `lib/`: debug-print plugin and bundled shared libraries.
- `qmp.py`: drives a running unit over QMP (tap, screenshot, monitor
  commands); needs Python 3.

Usage:

    ./qemu-clarion NOR [SD]
    ./qemu-clarion --dbgprint guest.log NOR [SD]
    ./qemu-clarion --help

`--dbgprint FILE` records Windows CE debug output while the guest runs.

Without a window, add a QMP socket and use `qmp.py`:

    ./qemu-clarion --headless NOR SD -qmp unix:/tmp/hu.sock,server=on,wait=off
    ./qmp.py /tmp/hu.sock shot screen.png
    ./qmp.py /tmp/hu.sock tap 268 445

On macOS, remove the quarantine attribute first (the binaries are only
ad-hoc signed):

    xattr -dr com.apple.quarantine .

Source and documentation: https://github.com/alexpalyan/qemu-clarion
