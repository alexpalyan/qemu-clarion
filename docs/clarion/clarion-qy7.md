# The `clarion-qy7` machine

`clarion-qy7` models the QY7221NL SH-4A unit. The `qemu-clarion` launcher
selects the machine from the NOR image and can keep both the NOR and SD card
unchanged with `--read-only`:

```sh
build-release/qemu-clarion --read-only --headless qy7-nor.bin map-card.img \
    -qmp unix:/tmp/qy7.sock,server=on,wait=off
```

Omit `--headless` to use the display window. The QY7 guest takes longer to
reach its display than the QY8 guest. The machine connects its display
microcontroller to SCIF3; panel input is represented as guest protocol frames.

## Front-panel keys

QEMU key-down and key-up events become command `0x34` frames. The high bit in
the data byte marks key-down; the low seven bits are the panel key code. The
frame uses the display-micom `DLE STX … DLE ETX` framing and BCC, and the model
waits for the guest's `DLE ACK` before completing the key event.

| Host key | Panel control | Panel code | Evidence |
|---|---|---:|---|
| `D` | DAY/NIGHT | `0x01` | Active KEPDRV tables map `01 -> 01 -> 0x22`; on the map the guest reports `SK_HWK_DAY_NIGHT_CLICK`. |
| `B` | Brightness/day-night control (inferred) | `0x21` | Active tables map `21 -> 02 -> 0xD9`; an early UI run showed `Lysstyrke` for this code under the previous host alias `S`. |

Host keys follow one table for QY7 and QY8 (see the QY8 reference, “Panel
keys and knobs”): the same key presses the same function. On QY7 only the
codes above and the MCAN routes below are proven; other keys of the table
(`Esc`, `Enter`, `C`, `A`, `[`/`]`, F1–F9, knobs) send nothing here.

The active QY7 `kepdrv.dll` table observed after boot has two first-stage
entries, `01 -> 01` and `21 -> 02`, and two second-stage entries,
`01 -> 0x22` and `02 -> 0xD9`. Both codes reach `AuiKeyEvent`; the first
translation is VK `0x22`, while the second is `0xD9`. The displayed brightness
overlay associated code `0x21` with brightness/day-night control in an early
UI run. The final replay stayed in a blank post-consent transition, so those
visible effects were not reproduced. The physical button label remains an
inference from the panel photo.

MENU, MAP/VOICE, STATUS and Zero Emission also arrive over the vehicle CAN
bus as MCAN `PUSH_SW` frames; see
[Verified MCAN PUSH_SW routes](#verified-mcan-push_sw-routes). Codes for
TUNE/SEEK/CH, FM-AM, XM, CD-AUX, VOL+/VOL−, CAMERA, and OPEN/TILT have
not been established. Do not infer codes for them from their labels.

Send a key through QMP in headless mode:

```json
{"execute":"send-key","arguments":{"keys":[{"type":"qcode","data":"m"}],"hold-time":100}}
```

The same host keys can be pressed in the QEMU display window.

## Raw panel-frame diagnostic

The QY7 display-micom exposes a persistent write-only `key-raw` QOM property
for investigating panel commands whose meaning is unknown. Its value is the
frame body as hexadecimal bytes, not a complete framed packet. The model adds
DLE/STX, DLE escaping, DLE/ETX, and BCC, sends the bytes through SCIF3, and
waits for DLE ACK (retrying up to three times on timeout or NAK).

For example, inject the tested one-byte `0x34` body and a four-byte `0x36`
body:

```json
{"execute":"qom-set","arguments":{"path":"/machine/qy7-dispmicom","property":"key-raw","value":"34 81"}}
{"execute":"qom-set","arguments":{"path":"/machine/qy7-dispmicom","property":"key-raw","value":"36 86 60 01 00"}}
{"execute":"qom-set","arguments":{"path":"/machine/qy7-dispmicom","property":"key-raw","value":"36 83 3B FF 01"}}
{"execute":"qom-set","arguments":{"path":"/machine/qy7-dispmicom","property":"key-raw","value":"36 86 6A 01 00"}}
```

Spaces, commas, and colons may separate bytes. One to 64 bytes are accepted.
The diagnostic does not validate command semantics, so use it with a
read-only guest run. It is enabled only on `clarion-qy7`; QY8 does not enable
the key input or this property.

## CAN receive, logging, and replay

The QY7 machine exposes the same CAN properties as QY8. `can-rx` is a
write-only QOM property whose value is a can-utils style hexadecimal
`ID#DATA` frame. QEMU offers the frame to configured RCAN receive mailboxes;
if none accepts it, the machine preserves the established SCPU `0x65` receive
path. This models an injected frame, not a CAN responder, and does not by
itself confirm that a guest driver accepted it.

For example, inject the `PUSH_SW`-shaped frame used by the T253 check:

```json
{"execute":"qom-set","arguments":{"path":"/machine","property":"can-rx","value":"280#400d81"}}
{"execute":"qom-set","arguments":{"path":"/machine","property":"can-rx","value":"280#400d01"}}
```

Set `can-log` and `can-replay` as machine properties at launch:

```sh
build-release/qemu-clarion --read-only --headless \\
    -M clarion-qy7,can-log=/tmp/qy7-can.log,can-replay=/tmp/can-replay.txt \\
    qy7-nor.bin map-card.img
```

`can-log` appends TX/RX frames and mode/filter changes. Frame records have the
same fields on QY7 and QY8: guest virtual milliseconds, direction, mailbox,
ID, format, DLC, and data. `can-replay` reads one periodic frame per line;
blank lines and lines beginning with `#` are ignored:

```text
# period in ms, then ID#DATA
100 280#400d81
```

The RCAN mailbox mechanism and interrupt behavior are unchanged. An injected
frame with no configured mailbox is logged with `mailbox=-1` before using the
existing SCPU receive path.

Dynamic QY7 replay confirmed that each tested subtype `36 86 60 xx yy`,
`36 83 3B FF xx`, and `36 86 6A xx yy` reaches the `lcddrv` key queue,
signals `EVENT_LCD_KEY`, and enters KEPDRV's `InputLcdKey` handler. KEPDRV
routes these to distinct state structures, but their physical meanings
(volume, tune/seek, or channel) are not established. A repeated key-down while
held is suppressed by the model; key-up still emits its release frame.

In the tested headless state, KEPDRV's `AuiKeyEvent` took its `PostMessageW`
path; its `keybd_event` call site was not reached. Statically, a nonzero
readiness flag at `ctx+0x64` selects `PostMessageW`; otherwise
`IsAPIReady(SH_WMGR)` is checked, and when that is false `IsWindowReady`
selects `keybd_event` for a ready window or `PostMessageW` otherwise. Display-micom
command `0x37` sends `key_code = 0xF0` with `KeyType = 1` in `kepdrv.dll`. On Nissan Leaf
(`Maker == 2`), the active key table `tbl_158` contains only MAP (`0x01`) and
Brightness (`0x02`), rejecting `0xF0` as `VK_KEY_NONE`. POWER on QY7 is routed through
MCAN as `AUDIO_ONOFF` (`20 0d`) instead.

The `PostMessageW` branch targets the HWND stored in KEPDRV's context and
posts `WM_KEYDOWN`/`WM_KEYUP` with the translated VK. The window's application
identity was not recovered from the available runtime state.

## Verified MCAN routes

These rows record guest-visible mappings from read-only QY7 captures. They do
not claim physical names for rows without a visible result. QY7 host keyboard
aliases send the five proven controls through MCAN (`PUSH_SW` for panel navigation keys,
and `AUDIO_ONOFF` function `0x20` for `POWER`); `D` and `B` use the display-micom frames above.
`M` is MAP on both series: on QY7 it sends the MCAN MAP/VOICE key (`P` is an alias), because
panel code `0x01` is day/night, not MAP.

| Panel control | Hard key | VKey | Verified guest result | Route |
|---|---:|---:|---|---|
| MAP/VOICE | `0x0C` | `0x0D` | Returns from menu/language screens to map | MCAN `PUSH_SW` |
| MENU | `0x44` | `0x1C` | Opens `MENY` from the map | MCAN `PUSH_SW` |
| STATUS | `0x31` | `0x1A` | Opens `Status / Lyd av / Ingen rute` | MCAN `PUSH_SW` |
| ZERO EMISSION | `0x4C` | `0x1F` | Opens the Zero Emission menu from the map | MCAN `PUSH_SW` |
| POWER | `0xF0` | `0x05` | Toggles media on/off (`AudioSourceSelect`, audio bar overlay) | MCAN `AUDIO_ONOFF` (`20 0d`) |

| Host key | Hard key | Panel control |
|---|---:|---|
| `N` | `0x44` | MENU |
| `M` (or `P`) | `0x0C` | MAP/VOICE |
| `T` | `0x31` | STATUS |
| `Z` | `0x4C` | ZERO EMISSION |
| `O` | `0xF0` | POWER |

Service menu: on the map, `M M M O O M` (MAP three times, POWER twice, MAP)
opens the system diagnostic menu (self diagnosis, confirmation/adjustment),
the documented way into Software Update on these units.

The MCAN keyboard input property is enabled only on QY7. Its first host key
starts the observed F-frame refresh and sends the observed `E0`/type-2 setup
before the press/release pair. Later pairs advance the three-bit message number
so both frames pass the session duplicate filter.

Other tested codes remain unnamed unless a visible result matched a control
on the panel photo. `0x12` and `0x47` remove a route overlay in the tested
state, but that does not identify a physical control. The Clarif30 evidence
records all tested VKeys and screenshots.

## Guest debug output

The `--dbgprint FILE` launcher option enables the bundled
`clarion-dbgprint` plugin. It prints calls to `NKDbgPrintfW`,
`NKvDbgPrintfW`, and `OutputDebugStringW` as `DBG <return-address> <text>`.
It reads SH-4 arguments from R4-R7 and the stack after the 16-byte home area.
The SH-4 GDB register XML exposes the active banked registers to the plugin
register API. The plugin scans every kernel XIP region in NOR and SD itself;
the SD card can contain multiple copies with different export addresses.

```sh
build-release/qemu-clarion --read-only --dbgprint guest-debug.log NOR SD
```

The launcher finds the plugin in the release package or build tree. XIP
exports have fixed virtual addresses, so no runtime module-load adjustment is
needed. For manual use, `-plugin PATH,image=NOR[,image=SD],out=FILE` performs
the same scan; `syms=FILE` remains available as a fallback for explicit
address lists.

## QY7 power-off and process restart

The micom frame `11 0d 00 01 00 00 40 20` is ordinary boot traffic and is
ACKed without a power-cycle action. The updater's YES path instead proceeds
through the kernel `OFFP` route. On X110, `nk.exe` writes zero to the board
power-hold register `0xB9100000` at guest PC `0x88047A24`; boot IPL first
asserts the same register with a write of one at `0xA0002488`. On X112, the
corresponding OFFP write is at `0x88047FB4`. This register transition models
the hardware releasing its power hold.

After the release, the QY7 machine requests shutdown with the shared
`CLARION_POWER_CYCLE_EXIT_CODE`. The `qemu-clarion` launcher treats that code
as a power cycle and starts a new child QEMU with the same arguments. A direct
`qemu-system-sh4` invocation exits with the power-cycle code because it has no
supervisor. With `--read-only`, NOR and SD qcow2 overlays stay in place across
the child restart and are removed when the launcher session exits; the NOR
overlay therefore retains guest flash writes such as clearing BMF.

## Diagnostics

QY7 model protocol traffic is quiet by default. Enable display-micom frames,
keys, retries, and acknowledgements with:

```sh
build-release/qemu-clarion --read-only --headless NOR SD \
    -trace 'clarion_dispmicom_*'
```

Other guest-driven model diagnostics using `LOG_GUEST_ERROR` are available
with `-d guest_errors`; this category can be verbose. Launcher messages, unit identification, and real
errors remain visible without these flags.
