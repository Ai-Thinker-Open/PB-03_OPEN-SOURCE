#!/usr/bin/env sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
tinycrypt_dir="$script_dir/../Ai_PB-03F_OPEN-SOURCE/components/libraries/tinycrypt-0.2.8"
test_log=$(mktemp "${TMPDIR:-/tmp}/pb03-tinycrypt-test.XXXXXX")

cleanup() {
    make -C "$tinycrypt_dir" clean >/dev/null
    rm -f -- "$tinycrypt_dir/tests/pseudo-random-data.bin"
    rm -f -- "$test_log"
}
trap cleanup EXIT INT TERM

make -C "$tinycrypt_dir" clean >/dev/null
make -C "$tinycrypt_dir" -j2

tests="
test_aes
test_cbc_mode
test_ccm_mode
test_cmac_mode
test_ctr_mode
test_ctr_prng
test_ecc_dh
test_ecc_dsa
test_hmac
test_hmac_prng
test_sha256
"

for test_name in $tests; do
    printf '%s\n' "RUN: $test_name"
    if (cd "$tinycrypt_dir/tests" && "./$test_name") >"$test_log" 2>&1; then
        printf '%s\n' "PASS: $test_name"
    else
        cat "$test_log" >&2
        printf '%s\n' "FAIL: $test_name" >&2
        exit 1
    fi
done

printf '%s\n' "TinyCrypt validation: PASS (11/11 tests)"
