#!/usr/bin/env bash
# Host-side regression tests for the wind-angle transform.
#
# src/sin_cos_angle_transform.h is pure math on top of a thin SensESP base, so
# it can be compiled and exercised on a laptop. test/host/stubs/ supplies just
# enough of Arduino / ArduinoJson / SensESP for the real header to build; the
# header itself is used unmodified, straight out of src/.
#
# Usage: tools/run_host_tests.sh
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
c++ -std=c++17 -Wall -Wextra -Wno-unused-parameter \
    -I "$root/test/host/stubs" -I "$root/src" \
    -o "$out/test_angle" "$root/test/host/test_angle.cpp"
"$out/test_angle"
