#!/bin/bash
# Assemble a relocatable Clarion release archive from a built tree.
#
# Usage: scripts/clarion/package.sh [-b BUILD_DIR] [-o OUT_DIR] TAG
#
# The build tree must be configured with
#   --target-list=arm-softmmu,sh4-softmmu --enable-plugins
# and have qemu-system-arm, qemu-system-sh4 and qemu-clarion built.
#
# Result: OUT_DIR/qemu-clarion-TAG-OS-ARCH.tar.gz containing
#   qemu-clarion            launcher
#   qmp.py                  QMP helper (tap, screenshot, monitor)
#   libexec/qemu-system-*   emulation engines
#   lib/                    debug-print plugin and bundled shared libraries
#   README.md COPYING
# The clarion machines load no data files at run time, so there is no share/.

# Written for bash 3.2 (macOS /bin/bash): no associative arrays.
set -eo pipefail

src=$(cd "$(dirname "$0")/../.." && pwd)
build=$src/build
out=$PWD

while getopts "b:o:" opt; do
    case $opt in
    b) build=$(cd "$OPTARG" && pwd) ;;
    o) mkdir -p "$OPTARG"; out=$(cd "$OPTARG" && pwd) ;;
    *) exit 2 ;;
    esac
done
shift $((OPTIND - 1))
tag=${1:?usage: package.sh [-b BUILD_DIR] [-o OUT_DIR] TAG}

os=$(uname -s | tr '[:upper:]' '[:lower:]')
arch=$(uname -m)
[ "$arch" = aarch64 ] && arch=arm64
name=qemu-clarion-$tag-$os-$arch
stage=$(mktemp -d "${TMPDIR:-/tmp}/qemu-clarion.XXXXXX")
trap 'rm -rf "$stage"' EXIT
root=$stage/$name

mkdir -p "$root/libexec" "$root/lib"
cp "$build/qemu-clarion" "$src/scripts/clarion/qmp.py" "$root/"
cp "$build/qemu-system-arm" "$build/qemu-system-sh4" "$root/libexec/"
case $os in
darwin) plugin_ext=dylib ;;
linux) plugin_ext=so ;;
esac
cp "$build/contrib/plugins/libclarion-dbgprint.$plugin_ext" \
    "$root/lib/"
cp "$src/COPYING" "$root/"
cp "$src/docs/clarion/README-release.md" "$root/README.md"

binaries=("$root/qemu-clarion" "$root"/libexec/qemu-system-* \
          "$root/lib/libclarion-dbgprint.$plugin_ext")

bundle_macos() {
    local queue=("${binaries[@]}") file dep real base
    local -a libs=()
    local seen=" "

    # Collect the transitive closure of non-system dylibs.
    while [ ${#queue[@]} -gt 0 ]; do
        file=${queue[0]}
        queue=("${queue[@]:1}")
        while read -r dep; do
            case $dep in
            /usr/lib/* | /System/*) continue ;;
            @*) echo "unsupported reference $dep in $file" >&2; exit 1 ;;
            esac
            base=$(basename "$dep")
            case $seen in *" $base "*) continue ;; esac
            seen="$seen$base "
            real=$(python3 -c 'import os,sys;print(os.path.realpath(sys.argv[1]))' "$dep")
            cp "$real" "$root/lib/$base"
            chmod u+w "$root/lib/$base"
            libs+=("$root/lib/$base")
            queue+=("$root/lib/$base")
        done < <(otool -L "$file" | tail -n +2 | awk '{print $1}')
    done

    # Remember the entitlements the build applied before touching anything.
    for file in "${binaries[@]}"; do
        codesign -d --entitlements - --xml "$file" \
            > "$stage/$(basename "$file").ents" 2> /dev/null || true
    done

    strip -x "${binaries[@]}" "${libs[@]}" 2> /dev/null || true

    local prefix
    for file in "${binaries[@]}" "${libs[@]}"; do
        case $file in
        */libexec/*) prefix=@loader_path/../lib ;;
        */lib/*) prefix=@loader_path ;;
        *) prefix=@loader_path/lib ;;
        esac
        case $file in
        */lib/*) install_name_tool -id "@loader_path/$(basename "$file")" "$file" ;;
        esac
        while read -r dep; do
            case $dep in
            /usr/lib/* | /System/* | @*) continue ;;
            esac
            install_name_tool -change "$dep" "$prefix/$(basename "$dep")" "$file"
        done < <(otool -L "$file" | tail -n +2 | awk '{print $1}')
    done

    for file in "${libs[@]}"; do
        codesign --force -s - "$file"
    done
    for file in "${binaries[@]}"; do
        local e=$stage/$(basename "$file").ents
        if [ -s "$e" ]; then
            codesign --force -s - --entitlements "$e" "$file"
        else
            codesign --force -s - "$file"
        fi
    done
}

bundle_linux() {
    local file dep base
    local -a libs=()
    local seen=" "
    # Never ship the C runtime: it must match the host kernel and loader.
    local skip='^(linux-vdso|ld-linux.*|libc|libm|libpthread|libdl|librt|libutil|libresolv|libnss_.*|libanl)\.so'

    for file in "${binaries[@]}"; do
        while read -r dep; do
            base=$(basename "$dep")
            [[ $base =~ $skip ]] && continue
            case $seen in *" $base "*) continue ;; esac
            seen="$seen$base "
            cp -L "$dep" "$root/lib/$base"
            chmod u+w "$root/lib/$base"
            libs+=("$root/lib/$base")
        done < <(ldd "$file" | awk '/=> \// {print $3}')
    done
    # Dependencies of the bundled libraries themselves.
    local again=1
    while [ $again = 1 ]; do
        again=0
        for file in "${libs[@]}"; do
            while read -r dep; do
                base=$(basename "$dep")
                [[ $base =~ $skip ]] && continue
                case $seen in *" $base "*) continue ;; esac
                seen="$seen$base "
                cp -L "$dep" "$root/lib/$base"
                chmod u+w "$root/lib/$base"
                libs+=("$root/lib/$base")
                again=1
            done < <(ldd "$file" | awk '/=> \// {print $3}')
        done
    done

    strip --strip-unneeded "${binaries[@]}" "${libs[@]}" 2> /dev/null || true
    patchelf --set-rpath '$ORIGIN/lib' "$root/qemu-clarion"
    for file in "$root"/libexec/*; do
        patchelf --set-rpath '$ORIGIN/../lib' "$file"
    done
    for file in "${libs[@]}"; do
        patchelf --set-rpath '$ORIGIN' "$file"
    done
}

case $os in
darwin) bundle_macos ;;
linux) bundle_linux ;;
*) echo "unsupported OS: $os" >&2; exit 1 ;;
esac

tar -C "$stage" -czf "$out/$name.tar.gz" "$name"
echo "$out/$name.tar.gz"
