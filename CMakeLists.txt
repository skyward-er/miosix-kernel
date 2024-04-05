# Copyright (C) 2023 by Skyward
#
# This program is free software; you can redistribute it and/or 
# it under the terms of the GNU General Public License as published 
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# As a special exception, if other files instantiate templates or use
# macros or inline functions from this file, or you compile this file
# and link it with other works to produce a work based on this file,
# this file does not by itself cause the resulting work to be covered
# by the GNU General Public License. However the source code for this
# file must still be made available in accordance with the GNU 
# Public License. This exception does not invalidate any other 
# why a work based on this file might be covered by the GNU General
# Public License.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>

cmake_minimum_required(VERSION 3.25)

project(MiosixExamples C CXX ASM)

# Add the Miosix kernel
add_subdirectory(miosix)

# Makes available the mbs_target function
include(miosix/cmake/mbs.cmake)

#-----------------------------------------------------------------------------#
#                                  Examples                                   #
#-----------------------------------------------------------------------------#

add_executable(asm miosix/_examples/asm/main.s)
mbs_target(asm stm32f407vg_stm32f4discovery)

add_executable(atsam4l_lcd miosix/_examples/atsam4l_lcd/main.cpp)
mbs_target(atsam4l_lcd atsam4lc2aa_generic)

add_executable(blinking_led miosix/_examples/blinking_led/simple.cpp)
mbs_target(blinking_led stm32f429zi_stm32f4discovery)

add_executable(datalogger
    miosix/_examples/datalogger/main.cpp
    miosix/_examples/datalogger/Logger.cpp
    miosix/_examples/datalogger/tscpp/buffer.cpp
)
target_include_directories(datalogger PUBLIC miosix/_examples/datalogger/)
mbs_target(datalogger stm32f407vg_stm32f4discovery)

add_executable(hd44780 miosix/_examples/hd44780/hd44780.cpp)
mbs_target(hd44780 stm32f407vg_stm32f4discovery)

add_executable(ir_decoder miosix/_examples/ir_decoder/ir_decoder.cpp)
mbs_target(ir_decoder stm32f407vg_stm32f4discovery)

add_executable(led_display miosix/_examples/led_display/main.cpp)
mbs_target(led_display stm32f100rb_stm32vldiscovery)

add_executable(sad_trombone
    miosix/_examples/sad_trombone/main.cpp
    miosix/_examples/sad_trombone/player.cpp
    miosix/_examples/sad_trombone/adpcm.c
)
mbs_target(sad_trombone stm32f407vg_stm32f4discovery)

add_executable(servo-prompt miosix/_examples/servo/prompt.cpp)
mbs_target(servo-prompt stm32f407vg_stm32f4discovery)

add_executable(servo-sweep miosix/_examples/servo/sweep.cpp)
mbs_target(servo-sweep stm32f407vg_stm32f4discovery)

add_executable(streamwriter miosix/_examples/streamwriter/streamwriter.cpp)
mbs_target(streamwriter stm32f407vg_stm32f4discovery)

add_executable(termios miosix/_examples/termios/main.cpp)
mbs_target(termios stm32f407vg_stm32f4discovery)

add_executable(thread_native miosix/_examples/thread_native/native_thread_example.cpp)
mbs_target(thread_native stm32f407vg_stm32f4discovery)

add_executable(thread_pthread miosix/_examples/thread_pthread/pthread_example.cpp)
mbs_target(thread_pthread stm32f407vg_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                                   Others                                    #
#-----------------------------------------------------------------------------#

add_executable(delay_test miosix/_tools/delay_test/delay_test.cpp)
mbs_target(delay_test stm32f407vg_stm32f4discovery)

add_executable(feedforward_profiling miosix/_tools/feedforward_profiling/test.cpp)
mbs_target(feedforward_profiling stm32f407vg_stm32f4discovery)

add_executable(fs_backend miosix/_tools/fs_backend/backend_benchmark.cpp)
mbs_target(fs_backend stm32f407vg_stm32f4discovery)

add_executable(fs_misc_testcode miosix/_tools/fs_misc_testcode/dirlist.cpp)
mbs_target(fs_misc_testcode stm32f407vg_stm32f4discovery)

# To build this entrypoint remember to define WITH_PROCESSES in miosix_settings.h
# add_executable(processes miosix/_tools/processes/main_processes.cpp)
# target_include_directories(processes PUBLIC miosix/_tools/processes/)
# add_custom_command(
#   TARGET processes
#   COMMAND make
#   WORKING_DIRECTORY miosix/_tools/processes/process_template
# )
# mbs_target(processes stm32f407vg_stm32f4discovery)

add_executable(ram_test
    miosix/_tools/ram_test/main.cpp
    miosix/_tools/ram_test/sha1.cpp
)
mbs_target(ram_test stm32f407vg_stm32f4discovery)

# If you want to test the MPU remember to run build.sh in miosix/_tools/testsuite
# add_executable(testsuite miosix/_tools/testsuite/testsuite.cpp)
# mbs_target(testsuite stm32f407vg_stm32f4discovery)

# Build the testsuite for all supported boards to test compilation
set(TESTSUITE_BOARDS
    lpc2138_miosix_board
    # stm32f072rb_stm32f0discovery
    efm32gg332f1024_wandstem
    # stm32f100c8_microboard
    # stm32f100c8_vaisala_rs41
    # stm32f100cb_tempsensor
    # stm32f100cx_generic
    # stm32f100rb_stm32vldiscovery
    stm32f100rc_solertegiard
    # stm32f103c8_breakout
    # stm32f103cb_als_mainboard_rev2
    # stm32f103cx_generic
    stm32f103ve_mp3v2
    stm32f103ve_strive_mini
    stm32f103ze_redbull_v2
    stm32f103ze_stm3210e-eval
    stm32f205_generic
    stm32f205rc_skyward_stormtrooper
    stm32f205rg_sony-newman
    stm32f207ig_stm3220g-eval
    stm32f207ze_als_camboard
    stm32f207zg_EthBoardV2
    stm32f207zg_nucleo
    # stm32l151c8_als_mainboard
    # atsam4lcc
    stm32f303vc_stm32f3discovery
    stm32f401re_nucleo
    stm32f401vc_stm32f4discovery
    stm32f407vg_bitsboard
    stm32f407vg_stm32f4discovery
    stm32f407vg_thermal_test_chip
    stm32f411ce_blackpill
    stm32f411re_nucleo
    stm32f429zi_oledboard2
    stm32f429zi_skyward_anakin
    stm32f429zi_skyward_homeone
    stm32f429zi_stm32f4discovery
    stm32f469ni_stm32f469i-disco
    stm32l4r9zi_sensortile
    stm32l476rg_nucleo
    stm32f746zg_nucleo
    stm32f767zi_nucleo
    # stm32f769ni_discovery
    stm32h753xi_eval
)
foreach(OPT_BOARD ${TESTSUITE_BOARDS})
    add_executable(testsuite-${OPT_BOARD} miosix/_tools/testsuite/testsuite.cpp)
    mbs_target(testsuite-${OPT_BOARD} ${OPT_BOARD})
endforeach()
