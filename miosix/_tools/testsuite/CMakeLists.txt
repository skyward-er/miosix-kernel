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

add_custom_target(miosix-testsuite)

foreach(OPT_BOARD ${BOARDS})
    set(KERNEL_TESTSUITE miosix-testsuite-${OPT_BOARD})
    add_executable(${KERNEL_TESTSUITE} ${KPATH}/_tools/testsuite/testsuite.cpp)
    target_link_libraries(${KERNEL_TESTSUITE} PRIVATE Miosix::Miosix::${OPT_BOARD})
    add_custom_command(
        TARGET ${KERNEL_TESTSUITE} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${KERNEL_TESTSUITE} ${KERNEL_TESTSUITE}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${KERNEL_TESTSUITE} ${KERNEL_TESTSUITE}.bin
        BYPRODUCTS ${KERNEL_TESTSUITE}.hex ${KERNEL_TESTSUITE}.bin
    )
    add_dependencies(miosix-testsuite ${KERNEL_TESTSUITE})
endforeach()
