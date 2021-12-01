#!/usr/bin/env bash

git ls-files "*.cpp" "*.hpp" | xargs -n 1 -P 16 clang-format --Werror --style=file -i 
