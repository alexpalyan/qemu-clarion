#!/bin/bash
# Contents/MacOS/launcher - the app's CFBundleExecutable.
# Normally hands off to the compiled config window. QY8_CARD/QY8_NAND (hidden,
# for our own testing) skip the window and boot straight from those paths.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
RES="$HERE/../Resources"
LOGDIR="$HOME/Library/Logs/QY8 Emulator"
mkdir -p "$LOGDIR"

if [ -n "${QY8_CARD:-}" ] && [ -n "${QY8_NAND:-}" ]; then
  HEADFLAG=""
  [ -z "${QY8_HEADLESS:-}" ] || HEADFLAG="--headless"
  if [ -n "${QY8_EXTRA:-}" ]; then
    exec "$RES/boot.sh" "$QY8_CARD" "$QY8_NAND" \
      --timeout "${QY8_TIMEOUT:-240}" --workdir "${QY8_WORKDIR:-$LOGDIR/headless-work}" \
      $HEADFLAG --extra "$QY8_EXTRA" > "$LOGDIR/headless.log" 2>&1
  fi
  exec "$RES/boot.sh" "$QY8_CARD" "$QY8_NAND" \
    --timeout "${QY8_TIMEOUT:-240}" --workdir "${QY8_WORKDIR:-$LOGDIR/headless-work}" \
    $HEADFLAG > "$LOGDIR/headless.log" 2>&1
fi

export QY8_RES="$RES"
export QY8_LOG_DIR="$LOGDIR"
exec "$HERE/QY8ConfigUI"
