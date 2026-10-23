#!/bin/bash
# Ported subset of qy8boot.sh for the packaged app: no python, no QMP screendump.
# Lives at Contents/Resources/boot.sh; runs the qemu-clarion launcher next
# to itself, which in turn picks and execs the right qemu-system-* engine.
#
#   boot.sh --identify CARD NAND
#   boot.sh CARD NAND [--model ze0|me0|ze1] [--board NAME] [--keep] [--verbose]
#                      [--headless] [--workdir DIR] [--timeout SECS] [--extra "FLAGS"]
#
# Prints a small line protocol the config UI reads: UNIT:, BOARD:, WARN:, WORK:,
# PID:, then READY or FAIL: reason (with LOG: lines of qemu.log tail on failure).
#
# The engine's own model table (clarion_boards[]) covers every unit we
# care about, so it is left to read the PROD block itself; dipsw/du-dotclk/
# gps and the bezel panel table are ours, not the engine's business, so
# those still go in as our own -M and -global, which it leaves alone once
# we supply -M ourselves.
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
ENGINE="$HERE/qemu-clarion"

hexat() { dd if="$1" bs=1 skip="$2" count="$3" 2>/dev/null | xxd -p -c 999 | tr -d '\n'; }

cstr() {
  local hex=$1 out="" b
  while [ ${#hex} -ge 2 ]; do
    b=${hex:0:2}
    [ "$b" = "00" ] && break
    [ "$b" = "ff" ] && break
    out="$out$(printf "\\x$b")"
    hex=${hex:2}
  done
  printf '%s' "$out"
}

NAND_OK=0 NAND_CODE= NAND_QY= NAND_NAV= CARD_NAV=

identify_nand() {
  local nand=$1 prod nk2 app
  prod=$(hexat "$nand" $((0x40000)) 128)
  [ "${prod:0:8}" = "50524f44" ] || return
  NAND_OK=1
  NAND_CODE=${prod:64:8}
  NAND_QY=$(cstr "${prod:128:16}")
  nk2=$(hexat "$nand" $((0xac0000)) 96)
  app=$(cstr "${nk2:44:52}")
  [[ "$app" =~ ^G2[0-9]{2}[A-Z]{4}\.[0-9]{3}$ ]] && NAND_NAV=$app
}

identify_card() {
  local card=$1 nk2 app
  nk2=$(hexat "$card" $((0x04d00000)) 96)
  app=$(cstr "${nk2:44:52}")
  [[ "$app" =~ ^G2[0-9]{2}[A-Z]{4}\.[0-9]{3}$ ]] && CARD_NAV=$app
}

family_label() {
  case "$NAND_QY" in
    QY8202NA) echo "ZE0" ;;
    QY8202PB) echo "ME0" ;;
    QY8602NB|QY8652NB) echo "ZE1" ;;
    QY8600NE) echo "ZE1 US" ;;
    *) case "$NAND_CODE" in 18*) echo "ZE1" ;; *) echo "ZE0" ;; esac ;;
  esac
}

pick_board() {
  case "$NAND_QY" in
    QY8202NA) BOARD=qy8202na ;;
    QY8202PB) BOARD=qy8202pb ;;
    QY8602NB) BOARD=qy8602nb ;;
    QY8652NB) BOARD=qy8652nb ;;
    QY8600NE) BOARD=qy8600ne ;;
    *)
      case "$NAND_CODE" in
        18*) BOARD=qy8602nb ;;
        *) case "$NAND_QY" in *PB) BOARD=qy8202pb ;; *) BOARD=qy8202na ;; esac ;;
      esac
      ;;
  esac
  PANEL=row
  [ "${NAND_CODE:2:1}" = "1" ] && PANEL=jpn
}

if [ "${1:-}" = "--identify" ]; then
  CARD=$2; NAND=$3
  [ -f "$NAND" ] && identify_nand "$NAND"
  [ -f "$CARD" ] && identify_card "$CARD"
  if [ "$NAND_OK" = 1 ]; then
    pick_board
    KEYS=ROW; [ "$PANEL" = jpn ] && KEYS=JPN
    echo "UNIT: $NAND_QY . $(family_label) . $KEYS keys"
  else
    BOARD=qy8202na
    echo "UNIT: unrecognized NAND (no PROD block) - will try board $BOARD"
  fi
  echo "BOARD: $BOARD"
  if [ -n "${NAND_NAV:-}" ] && [ -n "${CARD_NAV:-}" ] && [ "$NAND_NAV" != "$CARD_NAV" ]; then
    echo "WARN: card app $CARD_NAV vs NAND app $NAND_NAV - expect a version error"
  fi
  exit 0
fi

CARD=$1; NAND=$2; shift 2 || true
MODEL= BOARD_FORCED= KEEP= VERBOSE= HEADLESS=
WORK="$HOME/Library/Logs/QY8 Emulator/work"
TIMEOUT=240 EXTRA=

while [ $# -gt 0 ]; do
  case "$1" in
    --model) MODEL=$2; shift 2 ;;
    --board) BOARD_FORCED=$2; shift 2 ;;
    --keep) KEEP=1; shift ;;
    --verbose) VERBOSE=1; shift ;;
    --headless) HEADLESS=1; shift ;;
    --workdir) WORK=$2; shift 2 ;;
    --timeout) TIMEOUT=$2; shift 2 ;;
    --extra) EXTRA=$2; shift 2 ;;
    *) shift ;;
  esac
done

# "Extra flags" carries ~/.config/qy8boot/units-style presets, written for the
# terminal qy8boot.sh's own short flags - not literal qemu arguments. Parse
# the ones that mean something here; anything left over really is a qemu flag.
QEMU_EXTRA=()
if [ -n "$EXTRA" ]; then
  read -ra EXTRA_ARR <<< "$EXTRA"
  j=0
  while [ $j -lt ${#EXTRA_ARR[@]} ]; do
    tok=${EXTRA_ARR[$j]}
    case "$tok" in
      -m) MODEL=${EXTRA_ARR[$((j + 1))]}; j=$((j + 2)) ;;
      -b) BOARD_FORCED=${EXTRA_ARR[$((j + 1))]}; j=$((j + 2)) ;;
      -k) KEEP=1; j=$((j + 1)) ;;
      -v) VERBOSE=1; j=$((j + 1)) ;;
      -t) TIMEOUT=${EXTRA_ARR[$((j + 1))]}; j=$((j + 2)) ;;
      -w) WORK=${EXTRA_ARR[$((j + 1))]}; j=$((j + 2)) ;;
      -s | -n) j=$((j + 2)) ;;      # socket and NAND are ours to pick, not a preset's
      -g | -u | -p) j=$((j + 1)) ;; # window/update/screenshot modes: no-op here
      *) QEMU_EXTRA+=("$tok"); j=$((j + 1)) ;;
    esac
  done
fi

[ -f "$CARD" ] || { echo "FAIL: no such card: $CARD"; exit 1; }
[ -f "$NAND" ] || { echo "FAIL: no such NAND: $NAND"; exit 1; }
[ -x "$ENGINE" ] || { echo "FAIL: qemu-clarion missing from the app bundle"; exit 1; }

identify_nand "$NAND"
identify_card "$CARD"
pick_board
[ -z "$BOARD_FORCED" ] || BOARD=$BOARD_FORCED
case "$MODEL" in
  ze0) BOARD=${BOARD_FORCED:-qy8202na} ;;
  me0) BOARD=${BOARD_FORCED:-qy8202pb} ;;
  ze1) BOARD=${BOARD_FORCED:-qy8602nb} ;;
esac

KEYS=ROW; [ "$PANEL" = jpn ] && KEYS=JPN
if [ "$NAND_OK" = 1 ]; then
  echo "UNIT: $NAND_QY . $(family_label) . $KEYS keys"
else
  echo "UNIT: unrecognized NAND (no PROD block) - trying board $BOARD"
fi
echo "BOARD: $BOARD"
if [ -n "${NAND_NAV:-}" ] && [ -n "${CARD_NAV:-}" ] && [ "$NAND_NAV" != "$CARD_NAV" ]; then
  echo "WARN: card app $CARD_NAV vs NAND app $NAND_NAV - expect a version error"
fi

mkdir -p "$WORK"
rm -f "$WORK/console.log" "$WORK/qemu.log" "$WORK/qemu.pid"
cp "$NAND" "$WORK/nand.bin"
chmod u+w "$WORK/nand.bin"
printf '\x00\x00' | dd of="$WORK/nand.bin" bs=1 seek=$((0x100010)) conv=notrunc status=none 2>/dev/null

# --keep wants its writes to stick in the clone, so the engine's own
# default (snapshot=on, an overlay discarded on exit) has to be turned off
# for this run; nand.bin is a scratch copy either way, so making it
# writable too costs nothing. Without --keep, SDFILE is the real CARD and
# stays under the engine's default read-only overlay.
WRITABLE=
if [ -n "$KEEP" ]; then
  cp -c "$CARD" "$WORK/card.img" 2>/dev/null || cp "$CARD" "$WORK/card.img"
  chmod u+w "$WORK/card.img"
  SZ=$(stat -f %z "$WORK/card.img")
  P2=1; while [ "$P2" -lt "$SZ" ]; do P2=$((P2 * 2)); done
  [ "$SZ" -gt $((1 << 31)) ] || truncate -s "$P2" "$WORK/card.img"
  SDFILE="$WORK/card.img"; WRITABLE=--writable
else
  SDFILE=$CARD
fi

DISP="cocoa,show-cursor=on"
[ -z "$HEADLESS" ] || DISP="none"
SOCK="$WORK/qy8.qmp"
rm -f "$SOCK"

# NOR and SD are positional; everything from -M on is passed straight
# through qemu-clarion to qemu-system-arm, same as it was given directly.
QARGS=($WRITABLE --log "$WORK/qemu.log" "$WORK/nand.bin" "$SDFILE"
       -M "clarion-qy8,board=$BOARD,dipsw=1,du-dotclk=33333333,gps=off"
       -global "clarion-dispmicom.panel=$PANEL"
       -display "$DISP"
       -serial "file:$WORK/console.log" -serial null -serial null -serial null
       -qmp "unix:$SOCK,server=on,wait=off")
[ ${#QEMU_EXTRA[@]} -eq 0 ] || QARGS+=("${QEMU_EXTRA[@]}")

TMPDIR="$WORK" "$ENGINE" "${QARGS[@]}" &
PID=$!
echo "$PID" > "$WORK/qemu.pid"
echo "WORK: $WORK"
echo "PID: $PID"

tail_log() { tail -n 15 "$WORK/qemu.log" 2>/dev/null | sed 's/^/LOG: /'; }

# The config UI tails console.log/qemu.log itself once it has WORK: - piping
# them onto this same stdout used to share one fd with boot.sh's own protocol
# lines, and two writers racing on one fd garbled lines (a stray "READY" could
# land stuck to the end of an unrelated qemu.log line and go unrecognized).

qmp_screendump() {
  printf '{"execute":"qmp_capabilities"}\n{"execute":"human-monitor-command","arguments":{"command-line":"screendump %s -f png"}}\n' "$1" \
    | nc -U -w 3 "$SOCK" >/dev/null 2>&1
}

STATE=
for ((i = 0; i < TIMEOUT; i += 5)); do
  kill -0 "$PID" 2>/dev/null || { STATE=exited; break; }
  L="$WORK/console.log"
  if grep -q 'Version Error\|DISCREFUSE' "$L" 2>/dev/null; then STATE=version; break
  elif grep -qi 'launch failed\|NorFlash Load failed' "$L" 2>/dev/null; then STATE=launch; break
  elif grep -q 'Start Verify Check' "$L" 2>/dev/null; then STATE=ready; break
  fi
  sleep 5
done

case "$STATE" in
  version) sleep 5; kill "$PID" 2>/dev/null
    echo "FAIL: the card and the NAND are from different units (version error)"; tail_log; exit 1 ;;
  launch) kill "$PID" 2>/dev/null; echo "FAIL: the NAND failed to launch"; tail_log; exit 1 ;;
  exited) echo "FAIL: qemu exited before the HMI came up"; tail_log; exit 1 ;;
  '') kill "$PID" 2>/dev/null; echo "FAIL: no response after ${TIMEOUT}s"; tail_log; exit 1 ;;
esac

# "Start Verify Check" means the HMI task is up, not that anything is drawn
# yet; confirm with a real screendump the same way qy8boot.sh does. A consent
# dialog is drawn content too and counts as ready, same as any other screen.
REMAIN=$((TIMEOUT - i)); [ "$REMAIN" -gt 30 ] || REMAIN=30
SHOTOK=
for ((j = 0; j < REMAIN; j += 5)); do
  kill -0 "$PID" 2>/dev/null || { echo "FAIL: qemu exited while finishing boot"; tail_log; exit 1; }
  qmp_screendump "$WORK/boot.png"
  SZ=$(stat -f %z "$WORK/boot.png" 2>/dev/null || echo 0)
  [ "$SZ" -gt 3000 ] && { SHOTOK=1; break; }
  sleep 5
done
[ -n "$SHOTOK" ] || { echo "FAIL: screen still blank after ${TIMEOUT}s"; tail_log; exit 1; }
echo "READY"

# stays alive for as long as qemu runs, so the log pane keeps streaming and
# the UI learns the real exit code - whether qemu quit itself or Stop killed it
wait "$PID"
echo "EXITED: $?"
