#!/usr/bin/env bash

git ls-files "*.cpp" "*.hpp" | xargs -n 1 -P 16 clang-tidy --quiet -extra-arg="-std=c++17" -extra-arg="-I." -extra-arg="-Ibazel-rrtstar_se2/external/bb3d" -extra-arg="-isystem/usr/include/freetype2" -extra-arg="-isystem $(clang -print-resource-dir)/include"

#clang-tidy \
#    -extra-arg="-std=c++17" \
#    -extra-arg="-I." \
#    -extra-arg="-Ibazel-rrtstar_se2/external/bb3d" \
#    -extra-arg="-isystem/usr/include/freetype2" \
#    -extra-arg="-isystem $(clang -print-resource-dir)/include" \
#    -fix \
#    $(git ls-files "*.cpp" "*.hpp")
