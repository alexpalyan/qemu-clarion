#!/bin/bash
# Boot a Clarion QY8 head unit and attach its debug shell to this terminal.
# Usage: ./qy8-shell.sh NAND CARD [DIPSW]
#   NAND   64 MiB NAND dump of the unit (leafsdtools "nand_*.bin")
#   CARD   image of the unit's map SD card
#   DIPSW  boot mode, default 1 (NORM(EVA), debug shell on the console)
# QY8_NAND and QY8_CARD can stand in for the first two arguments.
# BOARD=ze0 for the 2014-2017 unit (QY8202NA); the default is ze1 (QY8602NB).
# HEADLESS=1 runs without a window, NOGL=1 leaves the screen to the SGX
# model, IMMO=1 keeps the unit's stored immobiliser setting.
set -e
here=$(cd "$(dirname "$0")" && pwd)
nand=${1:-$QY8_NAND}
card=${2:-$QY8_CARD}
mode=${3:-1}
board=${BOARD:-ze1}

if [ -z "$nand" ] || [ -z "$card" ]; then
    sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
fi
for f in "$nand" "$card"; do
    [ -f "$f" ] || { echo "no such file: $f" >&2; exit 1; }
done
case $board in
    ze0) glsyms=$here/contrib/plugins/qy8gl-g214.syms ;;
    ze1) glsyms=$here/contrib/plugins/qy8gl.syms ;;
    *)   echo "BOARD must be ze0 or ze1" >&2; exit 1 ;;
esac
case $(uname) in
    Darwin) plugin=libqy8gl.dylib; display=cocoa ;;
    *)      plugin=libqy8gl.so;    display=gtk ;;
esac
[ -n "$HEADLESS" ] && display=none

work=$(mktemp -d "${TMPDIR:-/tmp}/qy8.XXXXXX")

# the guest writes to both, so run on copies
cp "$nand" "$work/nand.bin"
# leafsdtools dumps carry VEUP 'Z' (update requested), which boots the update kernel instead of NK1
if [ -z "$KEEP_VEUP" ]; then
    printf '\x00\x00' | dd of="$work/nand.bin" bs=1 seek=$((0x100010)) conv=notrunc 2>/dev/null
fi
# clone instead of copying 16 GB where the filesystem allows it
if [ "$(uname)" = Darwin ]; then cp -c "$card" "$work/card.img"; else cp --reflink=auto "$card" "$work/card.img"; fi
truncate -s 16G "$work/card.img"

# software-render the AUI's GL calls; the immobiliser check reads as
# disabled (config 0x0f = 0, the value leafsdtools writes)
gl=()
if [ -z "$NOGL" ]; then
    export QY8_GL_FRAME="$work/glframe.bin"
    gl=(-plugin "$here/build/contrib/plugins/$plugin,syms=$glsyms,log=$work/gl.log")
    [ -z "$IMMO" ] && gl[1]="${gl[1]},cnf=0x0f:0"
fi

echo "work dir: $work   (Ctrl-A X quits)"
exec "$here/build/qemu-system-arm" -M clarion-qy8,board="$board",dipsw="$mode",du-dotclk=33333333 \
    -drive if=pflash,format=raw,file="$work/nand.bin" \
    -drive if=sd,index=0,format=raw,file="$work/card.img" \
    "${gl[@]}" -display "$display" -serial mon:stdio 2>"$work/qemu.log"
