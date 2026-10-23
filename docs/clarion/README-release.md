# Clarion head-unit emulator (binary package)

Layout:

- `qemu-clarion`: launcher; reads the unit model from the PROD block of the
  NOR image and starts the matching engine.
- `libexec/qemu-system-arm`: engine for QY8 units.
- `libexec/qemu-system-sh4`: engine for the QY7221NL.
- `libexec/qemu-img`: creates session overlays for `--read-only` runs.
- `lib/`: debug-print plugin and bundled shared libraries.

Usage:

    ./qemu-clarion NOR [SD]
    ./qemu-clarion --dbgprint guest.log NOR [SD]
    ./qemu-clarion --help

`--dbgprint FILE` records Windows CE debug output while the guest runs.

The launcher supervises QEMU and restarts it after the QY7 micom power-cycle
request. `--read-only` creates temporary qcow2 overlays for NOR and SD; those
overlays persist across process restarts and are removed when the launcher
session ends. Without the launcher, `qemu-system-sh4` exits with the shared
power-cycle status code and does not restart itself.

On macOS, remove the quarantine attribute first (the binaries are only
ad-hoc signed):

    xattr -dr com.apple.quarantine .

Source and documentation: https://github.com/alexpalyan/qemu
