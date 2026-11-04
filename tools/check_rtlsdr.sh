#!/bin/sh
# Runs the radio path against tools/fake_rtlsdr.c, built with the real rtl-sdr.h
# (from librtlsdr-dev).
set -e
out="$(mktemp -d)"
cc -Wall -Wextra -Werror -shared -fPIC -o "$out/libfake_rtlsdr.so" tools/fake_rtlsdr.c -lpthread
LIBRTLSDR_PATH="$out/libfake_rtlsdr.so" cargo test --release --test pipeline -- --ignored --nocapture
rm -rf "$out"
