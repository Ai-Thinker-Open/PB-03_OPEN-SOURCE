#!/usr/bin/env sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
source_sdk="$repo_root/Ai_PB-03F_OPEN-SOURCE"
temp_root=$(mktemp -d "${TMPDIR:-/tmp}/pb03-gcc-build.XXXXXX")

cleanup() {
    rm -rf -- "$temp_root"
}
trap cleanup EXIT INT TERM

command -v arm-none-eabi-gcc >/dev/null 2>&1 || {
    printf '%s\n' "arm-none-eabi-gcc was not found in PATH." >&2
    exit 1
}

cp -a "$source_sdk" "$temp_root/sdk"
project="$temp_root/sdk/example/ble_peripheral/simpleBlePeripheral/gcc"
log_file="$temp_root/build.log"

make -C "$project" -j2 2>&1 | tee "$log_file"

elf="$project/output/sbp.elf"
bin="$project/output/sbp.bin"
ihex="$project/output/sbp.ihex"
test -f "$elf"
test -f "$bin"
test -f "$ihex"

warnings=$(grep -c "warning:" "$log_file" || true)
errors=$(grep -c "error:" "$log_file" || true)
test "$errors" -eq 0

printf '\n%s\n' "Firmware build: PASS"
printf 'Warnings: %s\nErrors: %s\n' "$warnings" "$errors"
arm-none-eabi-size "$elf"
arm-none-eabi-readelf -h "$elf" | grep -E "Class:|Machine:|Entry point"
sha256sum "$elf" "$bin" "$ihex"
