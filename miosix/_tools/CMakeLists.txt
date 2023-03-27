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

# Use only the board where the testsuite has a chance to compile
set(BOARDS
    stm32f103ze_stm3210e-eval
    stm32f103ve_mp3v2
    stm32f103ve_strive_mini
    stm32f103ze_redbull_v2
    stm32f407vg_stm32f4discovery
    stm32f207zg_ethboard_v2
    stm32f205rg_sony-newman
    stm32f407vg_bitsboard
    stm32f429zi_stm32f4discovery
    stm32f429zi_oledboard2
    efm32gg332f1024_wandstem
    stm32f411re_nucleo
    stm32f429zi_skyward_anakin
    stm32f100rc_solertegiard
    stm32f205rc_skyward_stormtrooper
    stm32f401vc_stm32f4discovery
    stm32f469ni_stm32f469i-disco
    stm32f429zi_skyward_homeone
    stm32f401re_nucleo
    stm32f746zg_nucleo
    stm32h753xi_eval
    stm32f407vg_thermal_test_chip
    stm32f205_generic
    stm32f411ce_blackpill
    stm32f429zi_hre_test_stand
)

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
