#!/bin/bash
# Run the error handling tests (heater_faults, bme280, non-critical mcu)
# against fake mcus on a pty - no hardware or cross compiler needed.
#
#   scripts/error_handling_sim/run.sh [test ...]     (default: all tests)
#
# The klippy code is copied to $ERRSIM_BUILD (default /tmp/klipper_errsim)
# and its C helper is built there (with macOS shims if needed).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
export ERRSIM_BUILD=${ERRSIM_BUILD:-/tmp/klipper_errsim}
B=$ERRSIM_BUILD
mkdir -p "$B"
if [ ! -x "$B/venv/bin/python" ]; then
    python3 -m venv "$B/venv"
    "$B/venv/bin/pip" install -q pyserial cffi greenlet jinja2 markupsafe
fi
rsync -a --delete --exclude '*.so' --exclude '*.dSYM' --exclude __pycache__ \
    "$HERE/../../klippy/" "$B/klippy/"
if [ "$(uname)" = Darwin ]; then
    sed -i '' "s|^GCC_CMD = \"gcc\"|GCC_CMD = \"gcc -I$HERE/shim -Wno-unknown-attributes -Wno-ignored-optimization-argument\"|" \
        "$B/klippy/chelper/__init__.py"
fi
(cd "$B/klippy" && "$B/venv/bin/python" -c "import chelper; chelper.get_ffi()" \
    > "$B/chelper_build.log" 2>&1) || { cat "$B/chelper_build.log"; exit 1; }
TESTS=${*:-t_bme t_heater t_print t_station t_station2}
RES=0
for t in $TESTS; do
    out=$("$B/venv/bin/python" -u "$HERE/$t.py" 2>&1) || true
    echo "$out" | grep "^FAIL" || true
    r=$(echo "$out" | grep "^RESULT" || echo "RESULT: CRASHED")
    echo "$t: $r"
    case "$r" in *OK) ;; *) RES=1; echo "$out" | tail -20;; esac
done
exit $RES
