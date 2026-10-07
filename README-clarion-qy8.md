# Clarion QY8XXX in QEMU: building and running

The `clarion_qy8` branch adds the `clarion-qy8` machine: an emulated board
of the Clarion QY8XXX head unit (Renesas R-Car).

## 2DG blitter

The `clarion-2dg` model is mapped at `0xffe80000` and provides the register
readback, synthetic blit completion interrupt, and command-list diagnostics
needed by the guest driver. It does not execute the command list or draw
pixels. Completion follows a synthetic 1 ms virtual-clock delay; the
`+0x0c` bit 0 interrupt-enable interpretation is also synthetic. The model
is enabled by default and can be disabled with
`-M clarion-qy8,g2d=off` (or `QY8_G2D=off` with `tools/qy8_run.sh`).

Set `g2d-log=/path/to/file.jsonl` to append one JSON record per start,
including the virtual-clock timestamp, list address, and first 256 words.
This log is disabled by default.

No guest image is included in this repository. The machine boots from a
raw 64 MiB flash image that you supply.

Tested on macOS (Apple Silicon). A Linux build should work the same way,
but is not covered here.

## 1. Dependencies

* Xcode Command Line Tools;
* Homebrew: `brew install ninja pkg-config glib pixman`;
* Python 3 (`configure` creates its own venv with meson).

## 2. Building

Two build directories are used. Run the optimized one by default; the
debug build makes the guest noticeably slower.

```sh
# optimized build - use this one to run
mkdir -p build-release && cd build-release
../configure --target-list=arm-softmmu --disable-werror --enable-plugins --disable-docs
ninja qemu-system-arm
cd ..

# debug build - only when you need QEMU assertions and symbols
mkdir -p build && cd build
../configure --target-list=arm-softmmu --disable-werror --enable-debug
ninja qemu-system-arm
```

After a code change, `ninja qemu-system-arm` in the relevant directory is
enough.

## 3. Supplying the flash image and running

The machine has no kernel loader. It takes one raw image of the board's
NOR flash (chip select 0, exactly 64 MiB, no OOB data), maps it at
physical address 0 and starts the CPU from the reset vector. The image is
not parsed in any way. There are two ways to pass it, and exactly one of
them must be given:

| Option | Flash model | Guest writes |
|---|---|---|
| `-drive if=pflash,format=raw,file=IMAGE` | a real NOR chip (`cfi.pflash02`): the guest can read the ID, erase sectors and program words | go into `IMAGE` and survive a restart |
| `-bios IMAGE` | read-only ROM | are not possible; `IMAGE` is never modified |

With `-drive`, work on a copy, because the guest does write to flash:

```sh
cp my-image.bin flash-rw.bin

build-release/qemu-system-arm -M clarion-qy8 \
    -drive if=pflash,format=raw,file=flash-rw.bin -nographic
```

The guest console (SCIF3) is on the terminal. Leave `-nographic` with
`Ctrl-A X`; the QEMU monitor is `Ctrl-A C`.

With the 800x480 display window (macOS):

```sh
build-release/qemu-system-arm -M clarion-qy8,du-dotclk=33333333 \
    -drive if=pflash,format=raw,file=flash-rw.bin \
    -display cocoa,show-cursor=on -serial mon:stdio
```

`show-cursor=on` keeps the host cursor visible: the window hides it when
an absolute pointer device is present.

## 4. Machine properties

Set with `-M clarion-qy8,name=value`; a second `-machine name=value` on
the same command line is merged with the first.

| Property | Default | Purpose |
|---|---|---|
| `dipsw` | `5` | value of the board DIP switches, 0..7 |
| `board` | `auto` | board peripherals by unit model: `auto`, `qy8652nb`, `qy8202na` (see below) |
| `micom` | `on` | built-in companion MCU on SCIF4; `off` lets you attach your own responder via `-serial` |
| `dispmicom` | `on` | built-in display panel MCU on SCIF1; `off` works the same way |
| `gps` | `on` | built-in u-blox GNSS receiver on SCIF2; `off` leaves SCIF2 on `-serial` index 3 |
| `gps-lat` | `50.4501` | fix latitude in decimal degrees; writable at runtime with `qom-set` |
| `gps-lon` | `30.5234` | fix longitude in decimal degrees; writable at runtime with `qom-set` |
| `gps-speed` | `0` | speed in knots; a nonzero value advances the position |
| `gps-course` | `0` | course in degrees |
| `du-dotclk` | `0` | display dot clock in Hz; `0` means no frame tick. Use `33333333` for the window |
| `du-spi` | `31` | GIC line of the display frame interrupt |
| `i2c-empty` | `off` | I2C0..I2C2 as empty buses: an immediate NACK instead of a bus timeout |
| `i2c4` | `on` | Bounded I2C4 controller model at `0xffc73000` (the touchscreen bus) |
| `i2c4-recorder` | `off` | transaction recorder on I2C4, address `0x24` |
| `tma460` | `on` | TMA460 touchscreen controller model on I2C4 (requires `i2c4=on`); on the `qy8202na` board a TMA616 at `0x67` instead |
| `tma460-profile` | `on` | Synthetic TMA460 register profile with pointer-to-touch input |

### Board selection

Boards are named after the unit model, because the car generation does
not tell the unit apart: the 2014-2017 Leaf (ZE0) was fitted with units
of more than one family. With `board=auto` the machine reads the model
from the flash image, from the block that starts with `PROD` (8 ASCII
characters at `+0x40` of the block), and prints one line at start-up:

```
clarion-qy8: unit model QY8202NA (PROD block at 0x40000), board qy8202na (auto)
```

| Unit model | `board` | Touch controller | Notes |
|---|---|---|---|
| `QY8652NB` | `qy8652nb` | TMA460 at `0x24` | found in the 2018+ Leaf (ZE1) |
| `QY8202NA` | `qy8202na` | TMA616 at `0x67` | found in the 2014-2017 Leaf (ZE0) |

`ze1` and `ze0` are deprecated aliases of `qy8652nb` and `qy8202na`.

With `auto`, an image without a `PROD` block or with an unknown model is
an error; name the board explicitly to run it anyway. An explicit board
that does not match the model in the image only gives a warning. A
QY7-series unit (SH-4) is rejected: this machine does not model it.

A typical configuration with touch:

```
-M clarion-qy8,du-dotclk=33333333,i2c-empty=on
```

The touchscreen bus, controller, and synthetic profile are enabled by default.
Disable them explicitly with `i2c4=off,tma460=off,tma460-profile=off` when a
test needs the original no-touch behavior. `i2c-empty` remains opt-in.

### GNSS receiver

The built-in `clarion-ublox` device is connected to SCIF2 and enabled by
default. It answers UBX configuration requests with ACK-ACK, answers MON-VER,
and publishes the configured NAV messages and NMEA RMC, VTG, GGA, GSA, GSV,
and GLL sentences once per guest-clock second. GLL is deliberately last: the
unit's locator hands a buffered NMEA group to its parser when it receives a
sentence whose identifier ends in `GLL`.

Use `gps=off` to leave SCIF2 available through `-serial 3`. The position
properties accept decimal strings; for example, a QMP `qom-set` of `gps-lat`
or `gps-lon` changes the next receiver update. The clock comes from QEMU's
guest RTC, so `-rtc base=...` makes runs reproducible.

## 5. Serial ports and SD cards

**`-serial` order.** The first one is SCIF3 (the console), then SCIF0,
SCIF1, SCIF2, SCIF4, SCIF5, HSCIF0. With `micom=on` and `dispmicom=on`,
SCIF4 and SCIF1 are taken by the built-in models.

**SD cards.** Both slots always exist; without an image a slot is empty.

```sh
-drive if=sd,index=0,format=raw,file=sd.img    # front slot; index=1 is the second one
```

## 6. Pointer input

With `tma460-profile=on`, pointer events are reported only after the
controller exits bootloader and enters working mode. Button transitions are
preserved until the previous report is read; intermediate movement can be
coalesced. Raw report coordinates use the target profile offsets `X + 14`
and `Y + 9`. Y is reported bottom to top (`479 - Y`), because `kepdrv.dll`
mirrors it before it posts the touch to the window.

| Pointer event | TMA460 event ID | Queue state |
|---|---:|---|
| Left button down | 1 | Pressed |
| Movement while held | 2 | Pressed |
| Left button up | 3 | Released |

The same input path can be scripted over QMP (`-qmp
unix:path,server=on,wait=off`), with coordinates on the `0..0x7fff` scale.
The handler is bound to the display, so the events need `"device": "qy8-du"`;
without it QEMU answers "Input handler not found":

```json
{"execute": "qmp_capabilities"}
{"execute": "input-send-event", "arguments": {"device": "qy8-du", "events": [
  {"type": "abs", "data": {"axis": "x", "value": 16383}},
  {"type": "abs", "data": {"axis": "y", "value": 16383}},
  {"type": "btn", "data": {"button": "left", "down": true}}]}}
```

followed by the same event with `"down": false`.

## 7. Diagnostics

**Environment variables** (the main ones):

| Variable | Effect |
|---|---|
| `QY8_SCIF3_TXI=0` | disable the SCIF3 transmit interrupt |
| `QY8_CAN=off` | remove the CAN controller model |
| `QY8_SGX=off` | remove the GPU model |
| `QY8_SGX_EXEC=1`, `QY8_SGX_NULLRENDER=all` | the mode MIRROR needs, see section 8 |
| `QY8_UNIMP_PC=1` | add the guest code address to the unimplemented-peripheral log |

**The usual QEMU tools:**

* `-d unimp -D file` logs accesses to unimplemented devices and the model
  traces;
* `-s -S` waits for gdb/lldb on port 1234. Use hardware breakpoints for
  code that executes from flash;
* `-monitor tcp:127.0.0.1:PORT,server,nowait` gives a monitor, including
  `screendump file.ppm`.

## 8. MIRROR: GPU-rendered content in the window

The GPU model (`hw/display/clarion_sgx.c`) accepts commands but does not
rasterize. MIRROR works around that: a TCG plugin intercepts the guest's
EGL/GLES calls, replays them on the host GPU through ANGLE and puts the
finished frame into the guest display buffer.

MIRROR has three parts, and at the moment they are built separately from
QEMU. Only the first one can be built from this branch alone. The plugin
is tied to the current GPU approach and is expected to change; a shader
translator is planned for this branch so that pre-translated shaders are
no longer needed:

* **ANGLE** - the host EGL/GLES implementation;
* **the renderer** (`libqy8r`) - a thin layer over ANGLE.
  `subprojects/qy8r/` holds a basic version; the plugin needs an extended
  one (texture upload and copy) that is not in this branch yet;
* **the plugin** (`qy8_m138_plugin`) - not in this branch yet. It also
  needs translated guest shaders and their lookup table, which are
  specific to the guest image.

Build steps on macOS:

1. **ANGLE** (Metal backend) from the official sources; `args.gn`:

   ```
   is_debug = false
   is_component_build = false
   symbol_level = 0
   angle_enable_metal = true
   angle_enable_vulkan = false
   angle_enable_gl = false
   angle_enable_swiftshader = false
   angle_build_tests = false
   use_custom_libcxx = false
   treat_warnings_as_errors = false
   ```

   You need `libEGL.dylib` and `libGLESv2.dylib`.

2. **The renderer:**

   ```sh
   clang -dynamiclib -fPIC -O2 -D__APPLE__ -I<dir with qy8r.h> \
       -I<angle>/include qy8r.c -L<angle>/out/Release -lEGL -lGLESv2 \
       -Wl,-rpath,<angle>/out/Release -o libqy8r.dylib
   ```

3. **The plugin** (QEMU must be configured with `--enable-plugins`; the
   plugin headers come from this branch):

   ```sh
   clang -dynamiclib -fPIC -O2 -D__APPLE__ -undefined dynamic_lookup \
       -I include/plugins -I<dir with the shader tables> \
       $(pkg-config --cflags glib-2.0) qy8_m138_plugin.c \
       -o qy8_m138_plugin.dylib $(pkg-config --libs glib-2.0) -ldl
   ```

Running:

```sh
QY8_SGX_EXEC=1 QY8_SGX_NULLRENDER=all \
build-release/qemu-system-arm \
    -M clarion-qy8,du-dotclk=33333333,i2c-empty=on \
    -drive if=pflash,format=raw,file=flash-rw.bin \
    -display cocoa,show-cursor=on -serial mon:stdio \
    -plugin file=qy8_m138_plugin.dylib,out=mirror.calls.jsonl,qy8r=libqy8r.dylib,artifacts=<shaders>,lookup=<table.tsv>,lookup_sha256=<sha256 of the table>,mirror=1,present=1
```

`QY8_SGX_NULLRENDER=all` is a diagnostic mode: the model reports each
render as complete without drawing anything; the pixels come from the
plugin only. Current limitation: only the first few frames are presented;
the guest is not redrawn after that.

## 9. Limitations

* The GPU model does not rasterize; GPU-rendered content appears in the
  window only through MIRROR (section 8).
* Not modeled: the CAN bus beyond the controller itself, USB, I2C devices
  other than the TMA460.
