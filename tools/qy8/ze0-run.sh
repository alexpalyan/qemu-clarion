#!/bin/bash
# Boot the ZE0 unit headless for a while and summarise how far it got.
# Usage: ze0-run.sh SECS OUTDIR [extra qemu args...]
# Env: ZE0_NAND, ZE0_CARD (required), QEMU (binary), BOARD (ze0), DIPSW (1),
#      GL=FILE.syms to software-render the AUI with the qy8gl bridge
set -u
secs=$1; out=$2; shift 2
here=$(cd "$(dirname "$0")/../.." && pwd)
qemu=${QEMU:-$here/build/qemu-system-arm}
nand=${ZE0_NAND:?set ZE0_NAND to the NAND dump}
card=${ZE0_CARD:?set ZE0_CARD to the map card image}
for f in "$nand" "$card"; do
    [ -f "$f" ] || { echo "no such file: $f" >&2; exit 1; }
done
mkdir -p "$out"
rm -f "$out"/q.sock "$out"/*.log "$out"/screen.png

# copies only: the guest writes flash and card. Clear VEUP's update request.
cp "$nand" "$out/nand.bin"
printf '\x00\x00' | dd of="$out/nand.bin" bs=1 seek=$((0x100010)) conv=notrunc 2>/dev/null
if [ ! -f "$out/card.img" ]; then
    # clone instead of copying 16 GB where the filesystem allows it
    if [ "$(uname)" = Darwin ]; then cp -c "$card" "$out/card.img"; else cp --reflink=auto "$card" "$out/card.img"; fi
    truncate -s 16G "$out/card.img"
fi

cat "$here/contrib/plugins/qy8dbg.syms" > "$out/syms"
if [ -n "${GL:-}" ]; then
    cat "$GL" >> "$out/syms"
    export QY8_GL_FRAME="$out/glframe.bin"
    rm -f "$QY8_GL_FRAME"
fi
"$qemu" -M clarion-qy8,board=${BOARD:-ze0},dipsw=${DIPSW:-1},du-dotclk=33333333 \
    -drive if=pflash,format=raw,file="$out/nand.bin" \
    -drive if=sd,index=0,format=raw,file="$out/card.img" \
    -display none -serial file:"$out/console.log" \
    -qmp unix:"$out/q.sock",server,nowait \
    -plugin "$here/build/contrib/plugins/libqy8gl.$([ "$(uname)" = Darwin ] && echo dylib || echo so),syms=$out/syms,log=$out/dbg.log" \
    -d guest_errors,unimp -D "$out/guest.log" "$@" > "$out/qemu.log" 2>&1 &
pid=$!
echo "booting ${BOARD:-ze0} headless for ${secs}s, logs in $out ..."
sleep "$secs"
python3 "$here/tools/qy8/qmp.py" "$out/q.sock" "info registers" > "$out/regs.txt" 2>/dev/null
python3 "$here/tools/qy8/qmp.py" "$out/q.sock" "screendump $out/screen.png -f png" quit >/dev/null 2>&1
sleep 2; kill $pid 2>/dev/null; wait $pid 2>/dev/null

pc=$(grep -o 'R15=[0-9a-f]*' "$out/regs.txt" | cut -d= -f2)
echo "pc=$pc $([ "$pc" = 88037e28 ] && echo '(halted: failsafe reboot request)')"
echo "debug lines: $(grep -c '^DBG' "$out/dbg.log")"
echo "-- drivers started but never Ready:"
grep -o 'ActivateDevice([^)]*) \(Start\|Ready\)' "$out/dbg.log" | sort | uniq | \
    awk '{k=$1; s[k]=s[k]" "$2} END {for (k in s) if (s[k] !~ /Ready/) print "   " k}'
echo "-- last debug lines:"
grep '^DBG' "$out/dbg.log" | tail -8 | cut -c14-
