#!/bin/sh
# Build and run the host tests with the PC's gcc.
#
#   sh tests/host/run.sh
#
# Each test_*.c is compiled together with the driver source it tests.
# Exit status is non-zero if any check fails.
set -e
cd "$(dirname "$0")/../.."
OUT=$(mktemp -d)
fail=0

run() {
    name=$1; shift
    gcc -std=c11 -Wall -Wextra -O1 -Itests/host "$@" -o "$OUT/$name" -lm
    "$OUT/$name" || fail=1
}

run vn200 tests/host/test_vn200.c Devices/vn200/vn200.c -IDevices/vn200
run bar30 tests/host/test_bar30.c Devices/bar30/ms5837_math.c -IDevices/bar30
run dvl tests/host/test_dvl.c Devices/dvl/dvl.c -IDevices/dvl -Itests/host/stubs
run adc_depth tests/host/test_adc_depth.c -IDevices/adc_depth

rm -rf "$OUT"
exit $fail
