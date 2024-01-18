#!/usr/bin/env bash

set -e
set -x

find tests/ memo/ -name '*.cpp' -o -name '*.h' | xargs clang-tidy --checks=.clang-tidy -p=./cmake-build-debug/
