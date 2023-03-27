#!/bin/bash

# Copyright (c) 2022 Skyward Experimental Rocketry
# Authors: Damiano Amatruda
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

found_ccache=false
found_ninja=false

# Move to the root of the kernel repository
cd -- "$(dirname "$0")/../.."

# Check tools
command -v ccache > /dev/null 2>&1 && found_ccache=true
command -v ninja  > /dev/null 2>&1 && found_ninja=true

# Tell the user
printf "Found Ccache: "; [ "$found_ccache" = true ] && echo "yes" || echo "no, continuing without ccache"
printf "Found Ninja: ";  [ "$found_ninja" = true ]  && echo "yes" || echo "no, falling back to make"

# Prepare defines
declare -a defs
[ "$found_ccache" = true ] && defs+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
[ "$found_ninja" = true ]  && defs=-GNinja || gen=-G"Unix Makefiles"

# Build
cmake \
    -Bbuild \
    -DCMAKE_TOOLCHAIN_FILE=_tools/toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    "${defs[@]}"
    .
cmake --build build --target miosix-testsuite
