# QY8 head unit emulator

The `clarion-qy8` machine (see `README-clarion-qy8.md`) emulates the Clarion QY8 head unit of the
Nissan Leaf. This page covers what it takes to run both units on it: the 2014-2017 unit as a second
board, a USB host, the ZE0's touch controller, and a software renderer for the head unit's UI, so
both units boot to a screen you can tap.

No firmware ships with it. You need a NAND dump and a map card image from your own head unit.

## What works

| | ZE1 / 2018+ (QY8602NB) | ZE0 / 2014-2017 (QY8202NA) |
|---|---|---|
| Boots NK1 and the nav application | yes | yes, with `BOARD=ze0` |
| Boot animation and splash | yes | yes |
| UI screens (telematics consent, menus) | yes | yes |
| Touch, by clicking in the window | yes | yes |
| Debug shell on the console | yes | yes |
| USB host, devices enumerate | yes | yes |
| Map | no | no |
| Camera video, audio, CAN vehicle data | no | no |

The map stays blank: `Navi.exe` draws it through `XGLDLL.dll` on the GPU, and that path isn't
emulated. The map screen still comes up, with its buttons around a cyan area where the map would
be.

## What you need

- A 64 MiB NAND dump of your unit. [leafsdtools](https://github.com/developerfromjokela/leafsdtools)
  writes one (`nand_<id>_<serial>.bin`).
- An image of your unit's map SD card, e.g. `dd if=/dev/sdX of=card.img bs=8M`.

The launcher works on copies, so neither file is ever modified. Keep both to yourself: they hold your
unit's serial number and configuration.

## Build

macOS (Homebrew) or Linux, needs the usual QEMU build dependencies (glib, pixman, ninja, Python 3):

```
mkdir build && cd build
../configure --target-list=arm-softmmu --disable-docs
ninja qemu-system-arm contrib/plugins/libqy8gl.dylib   # libqy8gl.so on Linux
```

## Run

```
./qy8-shell.sh NAND CARD                 # ZE1 unit
BOARD=ze0 ./qy8-shell.sh NAND CARD       # ZE0 unit
```

A window opens and the debug shell runs in the terminal. The consent screen appears after about two
minutes; click to tap. Ctrl-A then X quits.

| variable | effect |
|---|---|
| `BOARD=ze0` | 2014-2017 board and its symbol table |
| `HEADLESS=1` | no window |
| `NOGL=1` | turn off the UI renderer (screen goes black after the splash) |
| `IMMO=1` | keep the stored immobiliser setting instead of reading it as disabled |
| `KEEP_VEUP=1` | keep the update-requested flag that leafsdtools dumps carry; boots the update kernel |

Debug shell commands take `<app id> <command>`. Useful ones: `00 ti` (unit info), `00 showid`
(registered apps), `00 cpu` (busiest threads), `26 help` (UI renderer), `09 help` (touch panel),
`01 keplogs func` (key and touch dispatcher logging).

## What changed, in short

- The leafsdtools dump has VEUP set to "update requested", which boots the update kernel. The
  launcher clears it in its copy.
- ZE0 board: USB host (EHCI with an OHCI companion) at 0xffe70000. Without it the ZE0's USB driver
  hangs while holding the driver loader lock and nothing after it loads.
- Touch: the ZE0 gets a TMA616 on the same I2C4 bus and GPIO lines as the ZE1's TMA460. Mouse
  clicks become touches.
- SD host: CSD/CID responses were shifted by a byte, DMA reads of the data port came back as zeros,
  and multi-block reads never sent CMD12.
- Board controller inputs: reverse, parking brake and illumination read as out of reverse, brake on,
  lights off. Left at zero they read as in reverse and the ZE0 started in the rear camera view. Type
  `qom-set /machine reverse on` in the monitor (Ctrl-A C) to shift into reverse.
- SGX heartbeat: the host driver's lockup watchdog reset the GPU hundreds of times per boot.
- `contrib/plugins/qy8gl.c`: hooks the UI's OpenGL ES calls at their fixed ROM addresses
  (`qy8gl.syms` for nav image G218ENNI, `qy8gl-g214.syms` for G214ELNI), draws its textured quads
  in software and hands frames to the display model. It also prints the OS debug messages.
- The GL plugin answers `DDWaitForBltDone` at once. Navi's map engine waited a second per call for
  blits that never finish, which kept the screen on "Please wait..." for about ten minutes after
  the consent screen.
- `tcg/aarch64`: wrong FEAT_CSSC min/max opcodes crashed QEMU on Apple M4/M5 hosts.

`tools/qy8/` has the helpers: export-table reader, QMP and debug-shell scripts, scripted taps.

## Status

Experimental, for poking at the firmware. Code here is not suitable for submission to upstream
QEMU; see `AGENTS.md`.
