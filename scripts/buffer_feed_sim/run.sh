#!/bin/bash
# Build and run the host simulation of the buffer_feed MCU code.
# Tests both stepper code paths (optimized edge path and the full path).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$HERE/../../src
B=$(mktemp -d)
trap 'rm -rf "$B"' EXIT
# Stub headers must be next to the sources so that they win over src/*.h
cp -r "$HERE"/stubs/* "$B"/
cp "$SRC"/stepper.c "$SRC"/buffer_feed.c "$HERE"/sim.c "$B"/
cd "$B"
# Mach-O (macOS) section names need a "segment,section" form
OS=()
[ "$(uname)" = Darwin ] && OS=('-D__section(S)=__attribute__((section("__DATA,__ctr")))')
for V in edge full; do
    D=""; [ $V = full ] && D="-DNO_EDGE_OPT"
    CC="gcc -O1 -g -Wall -Wno-unused-function -Wno-misleading-indentation $D -I. -I$SRC"
    CC="$CC ${OS[*]+"${OS[@]}"}"
    $CC -c stepper.c -o st_$V.o
    $CC -c buffer_feed.c -o bf_$V.o -Dstepper_inject_step=sim_inject_step \
        -Dstepper_set_idle_dir=sim_set_idle_dir
    $CC -c sim.c -o sim_$V.o
    gcc -o sim_$V st_$V.o bf_$V.o sim_$V.o -lm
    echo "######## stepper path: $V"
    ./sim_$V
done
