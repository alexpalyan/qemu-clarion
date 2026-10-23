# The `clarion-qy7` machine

`clarion-qy7` models the QY7221NL SH-4A unit. The `qemu-clarion` launcher
selects the machine from the NOR image and, by default, keeps both the NOR
and SD card unchanged (`snapshot=on`; add `--writable` to write them in
place):

```sh
build-release/qemu-clarion --headless qy7-nor.bin map-card.img \
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
| `M` | MAP/VOICE | `0x01` | Active KEPDRV tables map `01 -> 01 -> 0x22`; frame reaches `AuiKeyEvent`. An early UI run showed the map after M. |
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
TUNE/SEEK/CH, FM-AM, XM, CD-AUX, VOL+/VOL−, CAMERA, OPEN/TILT and power have
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

The QY7 machine also exposes a write-only `can-raw` QOM property for
diagnosing CAN receive frames. Its value is an 11-bit hexadecimal CAN ID, a
colon, and one to eight hexadecimal data bytes. QEMU first offers the frame
to configured RCAN receive mailboxes; if no mailbox accepts the ID, it wraps
the bytes in the sub-CPU `0x65` receive message and queues it on the UART path.
This does not model a CAN responder or confirm that a guest driver accepted
the frame.

For example, send a `PUSH_SW`-shaped body to the SCPU-only ID `0x0280`:

```json
{"execute":"qom-set","arguments":{"path":"/machine","property":"can-raw","value":"0280:40 0d 81"}}
{"execute":"qom-set","arguments":{"path":"/machine","property":"can-raw","value":"0280:40 0d 01"}}
```

Spaces or commas may separate data bytes. Use it with a read-only guest run;
the machine accepts the property only after its QY7 sub-CPU is initialized.

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
selects `keybd_event` for a ready window or `PostMessageW` otherwise. Tested
`0x37` caused a black display in the observed state; its semantic meaning is
unknown.

The `PostMessageW` branch targets the HWND stored in KEPDRV's context and
posts `WM_KEYDOWN`/`WM_KEYUP` with the translated VK. The window's application
identity was not recovered from the available runtime state.

## Verified MCAN PUSH_SW routes

These rows record guest-visible mappings from read-only QY7 captures. They do
not claim physical names for rows without a visible result. QY7 host keyboard
aliases send only the four proven controls through MCAN `PUSH_SW`; `M` and `B`
retain their existing display-micom behavior.

| Panel control | Hard key | VKey | Verified guest result | Route |
|---|---:|---:|---|---|
| MAP/VOICE | `0x0C` | `0x0D` | Returns from menu/language screens to map | MCAN `PUSH_SW` |
| MENU | `0x44` | `0x1C` | Opens `MENY` from the map | MCAN `PUSH_SW` |
| STATUS | `0x31` | `0x1A` | Opens `Status / Lyd av / Ingen rute` | MCAN `PUSH_SW` |
| ZERO EMISSION | `0x4C` | `0x1F` | Opens the Zero Emission menu from the map | MCAN `PUSH_SW` |

| Host key | Hard key | Panel control |
|---|---:|---|
| `N` | `0x44` | MENU |
| `P` | `0x0C` | MAP/VOICE |
| `T` | `0x31` | STATUS |
| `Z` | `0x4C` | ZERO EMISSION |

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
build-release/qemu-clarion --dbgprint guest-debug.log NOR SD
```

The launcher finds the plugin in the release package or build tree. XIP
exports have fixed virtual addresses, so no runtime module-load adjustment is
needed. For manual use, `-plugin PATH,image=NOR[,image=SD],out=FILE` performs
the same scan; `syms=FILE` remains available as a fallback for explicit
address lists.
