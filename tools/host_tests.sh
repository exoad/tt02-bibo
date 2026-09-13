#!/bin/sh
# The firmware/lib host suites, firmware/tests/test_*.cxx, for CI. From the
# repository root:
#
#   sh tools/host_tests.sh run   <cxx> [flag...]   build each suite, then run it
#   sh tools/host_tests.sh build <cxx> [flag...]   build only, for CodeQL to watch
#   sh tools/host_tests.sh tidy  [flag...]         clang-tidy each suite instead
#
# Exits 1 if any suite fails to build, pass or parse - read from exit codes,
# never from what a suite prints. Only chassis reaches into hal.hxx, so only it
# gets BIBO_FAKE_HAL, which swaps the SDK half for tests/fakes/hal.hxx.
set -u

usage="usage: tools/host_tests.sh run|build <cxx> [flag...]  |  tidy [flag...]"
mode=${1:-}
[ $# -gt 0 ] && shift
case $mode in
    run|build)
        if [ $# -eq 0 ]; then echo "$usage" >&2; exit 2; fi
        cxx=$1
        shift
        ;;
    tidy) ;;
    *) echo "$usage" >&2; exit 2 ;;
esac

mkdir -p out
fail=0
for src in firmware/tests/test_*.cxx; do
    name=$(basename "$src" .cxx)
    defs=""
    if [ "$name" = test_chassis ]; then defs="-DBIBO_FAKE_HAL"; fi

    # tidy's report is stdout, for the caller to count; its "N warnings
    # generated" chatter is stderr, and is dropped.
    if [ "$mode" = tidy ]; then
        if ! clang-tidy --quiet "$src" -- -std=c++20 "$@" $defs -I firmware/lib 2>/dev/null; then
            echo "::error title=clang-tidy failed::$name did not parse" >&2
            fail=1
        fi
        continue
    fi

    if ! "$cxx" -std=c++20 "$@" $defs -I firmware/lib -o "out/$name" "$src"; then
        echo "::error title=build failed::$name did not compile"
        fail=1
        continue
    fi
    if [ "$mode" = build ]; then
        echo "  [ ok ] $name compiled"
        continue
    fi

    if "out/$name"; then
        echo "  [ ok ] $name"
    else
        echo "::error title=suite failed::$name exited non-zero"
        fail=1
    fi
done
exit $fail
