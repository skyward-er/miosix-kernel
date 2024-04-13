# Copyright (C) 2024 by Skyward
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

set(BOARD_NAME efm32gg332f1024_wandstem)
set(ARCH_NAME cortexM3_efm32gg)

# Base directories with header files for this board
set(ARCH_PATH ${KPATH}/arch/${ARCH_NAME}/common)
set(BOARD_PATH ${KPATH}/arch/${ARCH_NAME}/${BOARD_NAME})
set(BOARD_CONFIG_PATH ${KPATH}/config/arch/${ARCH_NAME}/${BOARD_NAME})

# Boot file and linker script
set(BOOT_FILE ${BOARD_PATH}/core/stage_1_boot.cpp)
set(LINKER_SCRIPT ${BOARD_PATH}/efm32_1M+128k_rom_usbbootloader.ld)

# Clock frequency
set(CLOCK_FREQ -DEFM32_HFXO_FREQ=48000000 -DEFM32_LFXO_FREQ=32768)

# C++ Exception/rtti support disable flags.
# To save code size if not using C++ exceptions (nor some STL code which
# implicitly uses it) uncomment this option.
# -D__NO_EXCEPTIONS is used by Miosix to know if exceptions are used.
# set(OPT_EXCEPT -fno-exceptions -fno-rtti -D__NO_EXCEPTIONS)

# Specify a custom flash command
# This is the program that is invoked when the program-<target_name> target is
# built. Use <binary> or <hex> as placeolders, they will be replaced by the
# build systems with the binary or hex file path repectively.
# If a command is not specified, the build system will fallback to st-flash
set(PROGRAM_CMDLINE echo "make program not supported.")

# Basic flags
set(FLAGS_BASE -mcpu=cortex-m3 -mthumb)

# Flags for ASM and linker
set(AFLAGS ${FLAGS_BASE})
set(LFLAGS ${FLAGS_BASE} -Wl,--gc-sections,-Map,main.map -Wl,-T${LINKER_SCRIPT} ${OPT_EXCEPT} ${OPT_OPTIMIZATION} -nostdlib)

# Flags for C/C++
string(TOUPPER ${ARCH_NAME} ARCH_NAME_UPPER)
set(CFLAGS
    -D_BOARD_WANDSTEM -D_MIOSIX_BOARDNAME="${BOARD_NAME}"
    -D_DEFAULT_SOURCE=1 -ffunction-sections -Wall -Werror=return-type
    -D_ARCH_${ARCH_NAME_UPPER} -DEFM32GG332F1024
    ${CLOCK_FREQ} ${FLAGS_BASE} -c
)
set(CXXFLAGS ${CFLAGS} -std=c++14 ${OPT_EXCEPT})

# Select architecture specific files
set(ARCH_SRC
    ${ARCH_PATH}/drivers/adc.cpp
    ${ARCH_PATH}/interfaces-impl/delays.cpp
    ${ARCH_PATH}/interfaces-impl/gpio_impl.cpp
    ${ARCH_PATH}/interfaces-impl/portability.cpp
    ${BOARD_PATH}/interfaces-impl/bsp.cpp
    ${BOARD_PATH}/interfaces-impl/flopsync_vht.cpp
    ${BOARD_PATH}/interfaces-impl/gpio_timer.cpp
    ${BOARD_PATH}/interfaces-impl/gpioirq.cpp
    ${BOARD_PATH}/interfaces-impl/hrtb.cpp
    ${BOARD_PATH}/interfaces-impl/os_timer.cpp
    ${BOARD_PATH}/interfaces-impl/power_manager.cpp
    ${BOARD_PATH}/interfaces-impl/rtc.cpp
    ${BOARD_PATH}/interfaces-impl/spi.cpp
    ${BOARD_PATH}/interfaces-impl/timer_interface.cpp
    ${BOARD_PATH}/interfaces-impl/transceiver_timer.cpp
    ${BOARD_PATH}/interfaces-impl/transceiver.cpp
    ${BOARD_PATH}/interfaces-impl/vht.cpp
    ${BOARD_PATH}/interfaces-impl/virtual_clock.cpp
    ${KPATH}/arch/common/CMSIS/Device/SiliconLabs/EFM32GG/Source/system_efm32gg.c
    ${KPATH}/arch/common/core/interrupts_cortexMx.cpp
    ${KPATH}/arch/common/drivers/serial_efm32.cpp
)
