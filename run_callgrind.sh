#!/usr/bin/env bash

set -e
rm -f callgrind.out.*
bazel build -c dbg --strip=never --copt="-g" //...
#valgrind --tool=callgrind ./bazel-bin/run_for_profiling
valgrind --tool=callgrind ./bazel-bin/test_fast_tree
rm -f test_fast_tree
kcachegrind callgrind.out.*
