#!/bin/bash
# Build QY8 Emulator.app: a double-click wrapper around the alex qemu-clarion
# build, with its own card image and NAND dump, and no firmware shipped.
#
#   contrib/clarion/macos-gui/package-app.sh [OUTDIR]
#
# OUTDIR defaults to /Volumes/ABM/work/app. Writes "QY8 Emulator.app" and
# "QY8-Emulator.zip" there and prints both sizes. QEMU_BUILD_DIR overrides
# where qemu-system-arm and qemu-clarion are read from (default: REPO/build),
# for a release build whose configured tree isn't named "build".
#
# Steps: copy qemu-clarion and qemu-system-arm plus their combined Homebrew
# dylib closure, rewrite every load path to @executable_path/lib or
# @loader_path, ad-hoc sign everything, compile the config window, lay out
# the bundle, zip it. Both binaries end up next to each other in Resources/,
# so @executable_path/lib resolves the same way for either one once it is
# the running process's own main executable (boot.sh execs qemu-clarion,
# which execs qemu-system-arm in turn - each becomes "the" executable for
# dyld in its own moment, never the launcher script).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
BUILD_DIR=${QEMU_BUILD_DIR:-$REPO/build}
QEMU_BIN="$BUILD_DIR/qemu-system-arm"
CLARION_BIN="$BUILD_DIR/qemu-clarion"
OUT=${1:-/Volumes/ABM/work/app}
APP="$OUT/QY8 Emulator.app"
BUNDLE_ID=local.qy8.emulator

[ -x "$QEMU_BIN" ] || { echo "no qemu binary at $QEMU_BIN (build it first)" >&2; exit 1; }
[ -x "$CLARION_BIN" ] || { echo "no qemu-clarion at $CLARION_BIN (build it first)" >&2; exit 1; }
command -v swiftc >/dev/null || { echo "swiftc not found (Command Line Tools needed to build, not to run)" >&2; exit 1; }

echo "== laying out $APP"
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources/lib"

# ---- 1. qemu-clarion + qemu-system-arm, and their combined dylib closure ----
cp "$QEMU_BIN" "$APP/Contents/Resources/qemu-system-arm"
cp "$CLARION_BIN" "$APP/Contents/Resources/qemu-clarion"

python3 - "$APP/Contents/Resources/lib" \
  "$APP/Contents/Resources/qemu-system-arm" "$APP/Contents/Resources/qemu-clarion" <<'PYEOF'
import os, re, shutil, subprocess, sys

libdir, main_bins = sys.argv[1], sys.argv[2:]

def otool_deps(path):
    out = subprocess.run(['otool', '-L', path], capture_output=True, text=True, check=True).stdout
    deps = []
    for line in out.splitlines()[1:]:
        m = re.match(r'\s*(\S+)', line)
        if m:
            deps.append(m.group(1))
    return deps

def is_system(p):
    return (p.startswith('/usr/lib/') or p.startswith('/System/')
            or p.startswith('@'))

# Walk the closure of non-system deps starting from both main binaries.
# realpath so the Cellar path and the opt/ symlink collapse to one file.
to_copy = {}   # realpath -> basename used in lib/
queue = list(main_bins)
seen_bins = set()
while queue:
    p = queue.pop()
    if p in seen_bins:
        continue
    seen_bins.add(p)
    for d in otool_deps(p):
        if is_system(d) or d == p:
            continue
        real = os.path.realpath(d)
        base = os.path.basename(real)
        if real not in to_copy:
            to_copy[real] = base
            queue.append(real)

for real, base in to_copy.items():
    dest = os.path.join(libdir, base)
    shutil.copy2(real, dest)
    os.chmod(dest, 0o755)
print(f"copied {len(to_copy)} dylibs into {libdir}")

# sdl2-compat dlopens the real SDL3 from "@loader_path/libSDL3.dylib" at first
# use; otool -L never shows that, so it has to be added by hand.
sdl3 = '/opt/homebrew/opt/sdl3/lib/libSDL3.dylib'
sdl2_compat_present = any('SDL2' in b for b in to_copy.values())
if sdl2_compat_present and os.path.exists(sdl3):
    real_sdl3 = os.path.realpath(sdl3)
    base_sdl3 = os.path.basename(real_sdl3)
    dest = os.path.join(libdir, base_sdl3)
    shutil.copy2(real_sdl3, dest)
    os.chmod(dest, 0o755)
    link = os.path.join(libdir, 'libSDL3.dylib')
    if not os.path.exists(link):
        os.symlink(base_sdl3, link)
    to_copy[real_sdl3] = base_sdl3
    print(f"added {base_sdl3} (+ libSDL3.dylib symlink) for sdl2-compat's dlopen")

def rewrite(path, is_main):
    cmds = []
    for d in otool_deps(path):
        if is_system(d) or d == path:
            continue
        real = os.path.realpath(d)
        if real not in to_copy:
            continue
        base = to_copy[real]
        newpath = f'@executable_path/lib/{base}' if is_main else f'@loader_path/{base}'
        if d != newpath:
            cmds.append(['-change', d, newpath])
    if not is_main:
        subprocess.run(['install_name_tool', '-id', f'@loader_path/{os.path.basename(path)}', path], check=True)
    for c in cmds:
        subprocess.run(['install_name_tool'] + c + [path], check=True)

for main_bin in main_bins:
    rewrite(main_bin, True)
for real, base in to_copy.items():
    rewrite(os.path.join(libdir, base), False)
print("rewrote load commands")
PYEOF

# ---- 2. ad-hoc sign every dylib and both binaries ----------------------------
for f in "$APP/Contents/Resources/lib/"*.dylib; do
  codesign --force -s - "$f"
done
codesign --force -s - "$APP/Contents/Resources/qemu-system-arm"
codesign --force -s - "$APP/Contents/Resources/qemu-clarion"

# ---- 3. compile the config window, macOS 14 minimum --------------------------
swiftc -O -target arm64-apple-macosx14.0 -o "$APP/Contents/MacOS/QY8ConfigUI" \
  "$HERE/ConfigUI.swift" "$HERE/PresetStore.swift" "$HERE/main.swift" -framework AppKit
codesign --force -s - "$APP/Contents/MacOS/QY8ConfigUI"

# ---- 4. launcher, boot.sh, quickstart ----------------------------------------
cp "$HERE/launcher.sh" "$APP/Contents/MacOS/launcher"
chmod +x "$APP/Contents/MacOS/launcher"
cp "$HERE/boot.sh" "$APP/Contents/Resources/boot.sh"
chmod +x "$APP/Contents/Resources/boot.sh"
cp "$HERE/QUICKSTART.txt" "$APP/Contents/Resources/QUICKSTART.txt"

# ---- 4b. app icon, built from icon.png (already macOS-shaped) ----------------
ICONWORK=$(mktemp -d)
ICONPNG="$HERE/icon.png"
ICONSET="$ICONWORK/AppIcon.iconset"
mkdir -p "$ICONSET"
sips -z 16 16   "$ICONPNG" --out "$ICONSET/icon_16x16.png" >/dev/null
sips -z 32 32   "$ICONPNG" --out "$ICONSET/icon_16x16@2x.png" >/dev/null
sips -z 32 32   "$ICONPNG" --out "$ICONSET/icon_32x32.png" >/dev/null
sips -z 64 64   "$ICONPNG" --out "$ICONSET/icon_32x32@2x.png" >/dev/null
sips -z 128 128 "$ICONPNG" --out "$ICONSET/icon_128x128.png" >/dev/null
sips -z 256 256 "$ICONPNG" --out "$ICONSET/icon_128x128@2x.png" >/dev/null
sips -z 256 256 "$ICONPNG" --out "$ICONSET/icon_256x256.png" >/dev/null
sips -z 512 512 "$ICONPNG" --out "$ICONSET/icon_256x256@2x.png" >/dev/null
sips -z 512 512 "$ICONPNG" --out "$ICONSET/icon_512x512.png" >/dev/null
cp "$ICONPNG" "$ICONSET/icon_512x512@2x.png"
iconutil -c icns "$ICONSET" -o "$APP/Contents/Resources/AppIcon.icns"
cp "$APP/Contents/Resources/AppIcon.icns" "$HERE/AppIcon.icns"
rm -rf "$ICONWORK"

# ---- 5. Info.plist -------------------------------------------------------------
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key><string>launcher</string>
  <key>CFBundleIconFile</key><string>AppIcon.icns</string>
  <key>CFBundleIdentifier</key><string>$BUNDLE_ID</string>
  <key>CFBundleName</key><string>QY8 Emulator</string>
  <key>CFBundleDisplayName</key><string>QY8 Emulator</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>CFBundleVersion</key><string>1</string>
  <key>LSMinimumSystemVersion</key><string>14.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>LSApplicationCategoryType</key><string>public.app-category.utilities</string>
</dict>
</plist>
PLIST

# ---- 6. sign the bundle, verify ------------------------------------------------
codesign --force -s - --deep "$APP"

echo "== verifying no Homebrew paths remain"
if otool -L "$APP/Contents/Resources/qemu-system-arm" "$APP/Contents/Resources/qemu-clarion" \
         "$APP/Contents/Resources/lib/"*.dylib \
    | grep -i '/opt/homebrew'; then
  echo "FAIL: Homebrew paths still present" >&2
  exit 1
fi
codesign --verify --deep --strict "$APP"
echo "signing and paths OK"

# ---- 7. zip --------------------------------------------------------------------
ZIP="$OUT/QY8-Emulator.zip"
rm -f "$ZIP"
( cd "$OUT" && ditto -c -k --keepParent "QY8 Emulator.app" "$(basename "$ZIP")" )

echo "== sizes"
du -sh "$APP"
du -sh "$ZIP"
