#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

mkdir -p tests/build

clang++ -std=c++17 -Wall -Wextra -Werror \
  -Itests/stubs \
  -I. \
  tests/cpp_mbus_frames_test.cpp \
  components/e450_cyprus_meter/e450_cyprus_meter.cpp \
  -o tests/build/cpp_mbus_frames_test

tests/build/cpp_mbus_frames_test
