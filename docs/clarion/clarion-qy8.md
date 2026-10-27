# The `clarion-qy8` machine: reference

Details of the `clarion-qy8` machine (QY86xx and QY82xx units): running
the emulator directly, machine properties, serial ports, pointer input,
diagnostics and the MIRROR renderer. For building and the everyday way to
run a unit, see the [README](../../README.rst); the `qemu-clarion` launcher
described there builds the command lines shown here for you.

## 1. Supplying the flash image and running

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

## 2. Machine properties

Set with `-M clarion-qy8,name=value`; a second `-machine name=value` on
the same command line is merged with the first.

| Property | Default | Purpose |
|---|---|---|
| `dipsw` | `5` | value of the board DIP switches, 0..7 |
| `board` | `auto` | board peripherals by firmware build: `auto`, `g214`, `g218` (see below) |
| `micom` | `on` | built-in companion MCU on SCIF4; `off` lets you attach your own responder via `-serial` |
| `dispmicom` | `on` | built-in display panel MCU on SCIF1; `off` works the same way |
| `gps` | `on` | built-in u-blox GNSS receiver on SCIF2; `off` leaves SCIF2 on `-serial` index 3 |
| `g2d` | `on` | minimal 2DG blitter model: completes blits for the guest driver, draws nothing; `off` leaves the block unmodelled |
| `g2d-log` | unset | file that receives one JSON line per 2DG start (list address, first 256 words) |
| `can-log` | unset | append CAN frames and mode/filter changes using guest virtual milliseconds |
| `can-replay` | unset | file of periodic `period_ms ID#DATA` receive frames |
| `can-rx` | write-only | inject one `ID#DATA` frame at runtime through QMP |
| `rx-seq` | write-only | schedule `DELAY_MS ID#DATA` frames on the virtual clock |
| `shcore` | `on` | `on` models synthetic CPUCOM startup and initialization-end; `init` keeps only initialization-end; `off` disables SH modeling |
| `gps-lat` | `50.4501` | fix latitude in decimal degrees; writable at runtime with `qom-set` |
| `gps-lon` | `30.5234` | fix longitude in decimal degrees; writable at runtime with `qom-set` |
| `gps-speed` | `0` | speed in knots; a nonzero value advances the position |
| `gps-course` | `0` | course in degrees |
| `du-dotclk` | `33333333` | display dot clock in Hz; `0` disables frame ticks. An assumed reference value, not a measured board clock |
| `du-spi` | `31` | GIC line of the display frame interrupt |
| `i2c-empty` | `off` | I2C0..I2C2 as empty buses: an immediate NACK instead of a bus timeout |
| `i2c4` | `on` | Bounded I2C4 controller model at `0xffc73000` (the touchscreen bus) |
| `i2c4-recorder` | `off` | transaction recorder on I2C4, address `0x24` |
| `tma460` | `on` | TMA460 touchscreen controller model on I2C4 (requires `i2c4=on`); on the `g214` board a TMA616 at `0x67` instead |
| `tma460-profile` | `on` | Synthetic TMA460 register profile with pointer-to-touch input |

### Board selection

A firmware build is made for one board, so the board is chosen from the
build named in the NK2 image header (12 ASCII characters at NOR
`0xAC0016`), not from the unit model. With `board=auto` the machine prints
one line at start-up:

```
clarion-qy8: unit model QY8252NF, firmware G214ELNI.112, board g214 (auto), TMA616 touch controller
```

| Firmware build | `board` | Touch controller | Units seen |
|---|---|---|---|
| `G214…` | `g214` | TMA616 at `0x67` | QY8202NA, QY8202NC, QY8252NF (2014-2017 Leaf), QY8202PB (e-NV200) |
| `G218…` | `g218` | TMA460 at `0x24` | QY8652NB, QY8602NB, QY8600NE (2018+ Leaf, ZE1) |

With `auto`, an image whose NK2 header names no known build is an error;
name the board explicitly to run it anyway (for example a damaged or erased
NK2 image). The unit model from the `PROD` block is only reported. A
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

### 2DG blitter

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

### SH core host interface

The `clarion-shcore` model is enabled by default and mapped at `0xfe700000`
with a `0x3c`-byte register window. GIC SPI 54 is demultiplexed through INTC2 status word
`0xfe78208c`, bit `0x10000`. A guest write with bit 0 set to HPB `+0x04`
arms or rearms a synthetic 100 ms virtual-clock delay. When it expires, the
model sets `+0x1c` bit 2 and raises the level-triggered interrupt. A guest
write with bit `0x100` set to `+0x1c` acknowledges the event and lowers the
line. Reads of `+0x08` always return zero and reads of `+0x30` always return
bit 0 clear. `shcore=init` preserves this initialization-end-only behavior.

With `shcore=on`, a 0x2000-byte shared CPUCOM window is also mapped at
`0xfe790000`. The model completes guest Tx doorbells and returns synthetic
responses for the registered local Bluetooth task: configuration, start,
stop, security, local-name, and SSP IO-capability confirmations, followed by
an application-init indication. Reply status, Bluetooth address, version,
and service fields are synthetic values inferred from the guest receiver.
No packets were captured from a real SH core or Bluetooth module. Other tasks
and functions are logged without a response; profile traffic is not modeled.
Disable the model with `-M clarion-qy8,shcore=off` (or `QY8_SHCORE=off` with
`tools/qy8_run.sh`).

### CAN controller

The Renesas controller is mapped at `0xfffd1000`. `can-log` appends one
line per observed TX/RX frame and mode or filter change. Frame lines use
`guest_ms=... direction=... mailbox=... id=... format=... dlc=... data=...`;
the timestamp uses the QEMU virtual guest clock. Receive injection matches
the ID against an armed mailbox; receive masks are not modeled. For example,
inject a standard frame after the guest has configured mailbox 0 for ID
`0x683`:

```json
{"execute":"qom-set","arguments":{"path":"/machine","property":"can-rx","value":"683#400d81"}}
```

At launch, set `can-log` and `can-replay` as machine properties:

```sh
-M clarion-qy8,can-log=/tmp/qy8-can.log,can-replay=/tmp/can-replay.txt
```

Replay files contain one periodic frame per line. Blank lines and lines
beginning with `#` are ignored:

```text
# period in ms, then can-utils style ID#DATA
100 683#400d81
```

`can-rx` is write-only; set it with QMP after machine initialization. A frame
with no configured receive mailbox is still recorded in `can-log` with
`mailbox=-1` and is not delivered to the guest.

`rx-seq` schedules a one-shot sequence through the same receive path. Its
semicolon-separated items are `DELAY_MS ID#DATA`; each delay is relative to
the preceding frame, and the first delay starts when `rx-seq` is set. For
example:

```json
{"execute":"qom-set","arguments":{"path":"/machine","property":"rx-seq","value":"12 6ac#21414141414141;12 6ac#22414141414141"}}
```

Only one sequence can be active at a time. A new sequence requested while one
is pending returns an error. Every frame is delivered by a `QEMU_CLOCK_VIRTUAL`
timer and recorded in `can-log`.

## 3. Serial ports and SD cards

**`-serial` order.** The first one is SCIF3 (the console), then SCIF0,
SCIF1, SCIF2, SCIF4, SCIF5, HSCIF0. With `micom=on` and `dispmicom=on`,
SCIF4 and SCIF1 are taken by the built-in models.

**SD cards.** Both slots always exist; without an image a slot is empty.

```sh
-drive if=sd,index=0,format=raw,file=sd.img    # front slot; index=1 is the second one
```

**SDHI clock gating (SCLKEN).** On real hardware, SDHI commands and bus operations require an active SD clock (`SD_CLK_CTRL.SCLKEN = 1`, bit 8). The bootloader enables `SCLKEN` only for the duration of each command/transfer and clears it (`0x0001`) immediately afterwards. In the unclocked mode (`input-clock-hz` unset, used by `clarion-qy8`), any SD command issued while `SCLKEN = 0` is deferred until the guest writes `SD_CLK_CTRL` with `SCLKEN = 1`. A deferred command start triggers the trace event `renesas_sdhi_deferred_start`.

## 4. Pointer input

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

### Panel keys and knobs

The display panel MCU (`dispmicom`) sends the bezel keys as command `0x34`
(matrix code, bit 7 = pressed) and the two knobs as command `0x35`
(sign-magnitude detents). Host keys map to the same functions on QY8 and QY7;
the codes come from the `kepdrv.dll` `InputLcdKey` tables, and `panel=row|jpn`
picks the column (`row` for EUR and USA units, the default).

| Host key | Function | QY8 `row` | QY8 `jpn` |
|---|---|---:|---:|
| `M` (or `F4`) | MAP | `0x00` | `0x32` |
| `N` (or `F1`) | MENU | `0x13` | `0x01` |
| `Esc` (or `F2`) | BACK | `0x30` | `0x03` |
| `D` (or `F3`) | ☀/☾: day/night; hold: display off | `0x20` | `0x31` |
| `C` (or `F5`) | CAMERA | `0x01` | `0x13` |
| `A` (or `F6`) | AUDIO | `0x03` | `0x30` |
| `[` / `]` (or `F7` / `F8`) | SEEK back / forward | `0x31` / `0x32` | `0x00` / `0x20` |
| `Enter` (or `F9`) | TUNE press | `0x21` | `0x10` |
| Up / Down, PageUp / PageDown, mouse wheel | TUNE knob | `0x35` | `0x35` |
| `=` / `-` | VOL knob | `0x35` | `0x35` |
| `O` | POWER (VOL knob push) | bare `0x37` frame | bare `0x37` frame |
| `B` | brightness (QY7 only) | — | — |

Keys can also be injected over QMP: `qom-set` on `/machine/panel-micom` with
`key="CODE[,HOLD_MS]"` (hex matrix code, default hold 150 ms) or a raw frame
body with `frame="35 01 00"`. A zoom knob and the remaining bezel keys have
no established codes yet.

## 5. Diagnostics

**Environment variables** (the main ones):

| Variable | Effect |
|---|---|
| `QY8_SCIF3_TXI=0` | disable the SCIF3 transmit interrupt |
| `QY8_CAN=off` | remove the CAN controller model |
| `QY8_SGX=off` | remove the GPU model |
| `QY8_SGX_EXEC=1`, `QY8_SGX_NULLRENDER=all` | the mode MIRROR needs, see the MIRROR section |
| `QY8_UNIMP_PC=1` | add the guest code address to the unimplemented-peripheral log |

**The usual QEMU tools:**

* `-d unimp -D file` logs accesses to unimplemented devices and the model
  traces;
* `-s -S` waits for gdb/lldb on port 1234. Use hardware breakpoints for
  code that executes from flash;
* `-monitor tcp:127.0.0.1:PORT,server,nowait` gives a monitor, including
  `screendump file.ppm`.

## 6. MIRROR: GPU-rendered content in the window

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

## 7. Limitations

* The GPU model does not rasterize; GPU-rendered content appears in the
  window only through MIRROR (see the MIRROR section).
* Not modeled: the CAN bus beyond the controller itself, USB, I2C devices
  other than the TMA460.

## Guest debug output

The `--dbgprint FILE` launcher option enables the bundled
`clarion-dbgprint` plugin. It prints calls to `NKDbgPrintfW`,
`NKvDbgPrintfW`, and `OutputDebugStringW` as `DBG <return-address> <text>`.
ARM arguments are read from R0-R3 and then the stack. Wide `%s`, narrow
`%S`, integer, pointer, and character conversions are supported. The plugin
scans the NOR image for kernel XIP exports when it starts.

```sh
build-release/qemu-clarion --read-only --dbgprint guest-debug.log NOR
```

The launcher finds the plugin in the release package or build tree. XIP
export addresses are fixed virtual addresses and do not require runtime
relocation. For manual use, `-plugin PATH,image=NOR,out=FILE` performs the
same scan; `syms=FILE` remains available as a fallback for explicit address
lists.

## Diagnostics

Normal runs keep model diagnostics out of stderr. QEMU startup errors, real
model errors, the launcher messages, and the unit/board identification remain
visible.

| Diagnostic | Enable it with |
|---|---|
| SGX kick decode and page-table detail | `QY8_SGX_KICKS=N` (number of initial kicks to report; default `0`) |
| SGX address/range, graph, shader, and renderer probes | Existing `QY8_SGX_*` options; for SGX address dumps also set `QY8_SGX_KICKS` above zero |
| Renderer GL call log and frame dumps | `QY8_RENDER_LOG=PATH` and `QY8_RENDER_DUMP_DIR=DIR` |
| Guest accesses and model events logged with `LOG_GUEST_ERROR` | `-d guest_errors` |
| Display-micom protocol stream | `-trace 'clarion_dispmicom_*'` |

Examples:

```sh
# Display-micom protocol
build-release/qemu-clarion --read-only --headless NOR \
    -trace 'clarion_dispmicom_*'

# SGX report for the first two kicks
QY8_SGX_KICKS=2 build-release/qemu-clarion --read-only --headless NOR

# Other guest error events (can be verbose)
build-release/qemu-clarion --read-only --headless NOR -d guest_errors
```
