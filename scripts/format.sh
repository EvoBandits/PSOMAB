#!/bin/sh -e

set -x

find tests src -iname '*.cpp' -o -iname '*.h' | xargs clang-format -i
