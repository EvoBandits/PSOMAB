#!/bin/sh -e

set -x

find tests memo -iname '*.cpp' -o -iname '*.h' | xargs clang-format -i