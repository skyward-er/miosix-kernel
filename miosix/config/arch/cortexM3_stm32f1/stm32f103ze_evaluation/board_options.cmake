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

set(BOARD_NAME stm32f103ze_evaluation)
set(ARCH_NAME cortexM3_stm32f1)

# Base directories with header files for this board
set(ARCH_PATH ${KPATH}/arch/${ARCH_NAME}/common)
set(BOARD_PATH ${KPATH}/arch/${ARCH_NAME}/${BOARD_NAME})
set(BOARD_CONFIG_PATH ${KPATH}/config/arch/${ARCH_NAME}/${BOARD_NAME})

# Boot file and linker script
set(BOOT_FILE ${BOARD_PATH}/core/stage_1_boot.cpp)

# Linker script type, there are three options
# 1) Code in FLASH, stack + heap in internal RAM (file *_rom.ld)
#    the most common choice, available for all microcontrollers
# 2) Code in FLASH stack in internal RAM heap in external RAM (file
#    *_xram.ld) useful for hardware like STM3210E-EVAL when big heap is
#    needed. The microcontroller must have an external memory interface.
# 3) Code + stack + heap in external RAM, (file *_all_in_xram.ld)
#    useful for debugging code in hardware like STM3210E-EVAL. Code runs
#    *very* slow compared to FLASH. Works only with a booloader that
#    forwards interrrupts @ 0x68000000 (see miosix/_tools/bootloaders for
#    one).
#    The microcontroller must have an external memory interface.
# Warning! enable external ram if you use a linker script that requires it
# (see the XRAM flag below)
# set(LINKER_SCRIPT ${BOARD_PATH}/stm32_512k+64k_rom.ld)
# set(LINKER_SCRIPT ${BOARD_PATH}/stm32_512k+64k_xram.ld)
set(LINKER_SCRIPT ${BOARD_PATH}/stm32_512k+64k_all_in_xram.ld)

# Enable/disable initialization of external RAM at boot. Three options:
# __ENABLE_XRAM : If you want the heap in xram (with an appropriate linker
# script selected above)
# __ENABLE_XRAM and __CODE_IN_XRAM : Debug mode with code + stack + heap
# in xram (with an appropriate linker script selected above)
# none selected : don't use xram (with an appropriate linker script
# selected above)
# set(XRAM -D__ENABLE_XRAM)
set(XRAM -D__ENABLE_XRAM -D__CODE_IN_XRAM)

# Clock frequency
# Not defining anything results in HSI being used
# set(CLOCK_FREQ -DSYSCLK_FREQ_24MHz=24000000)
# set(CLOCK_FREQ -DSYSCLK_FREQ_36MHz=36000000)
# set(CLOCK_FREQ -DSYSCLK_FREQ_48MHz=48000000)
# set(CLOCK_FREQ -DSYSCLK_FREQ_56MHz=56000000)
set(CLOCK_FREQ -DSYSCLK_FREQ_72MHz=72000000)

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
if(${LINKER_SCRIPT} STREQUAL ${BOARD_PATH}/stm32_512k+64k_all_in_xram.ld)
    set(PROGRAM_CMDLINE ${KPATH}/_tools/bootloaders/stm32/pc_loader/pc_loader /dev/ttyUSB0 <binary>)
else()
    set(PROGRAM_CMDLINE stm32flash -w <hex> -v /dev/ttyUSB0)
endif()

# Basic flags
set(FLAGS_BASE -mcpu=cortex-m3 -mthumb)

# Flags for ASM and linker
set(AFLAGS ${FLAGS_BASE})
set(LFLAGS ${FLAGS_BASE} -Wl,--gc-sections,-Map,main.map -Wl,-T${LINKER_SCRIPT} ${OPT_EXCEPT} ${OPT_OPTIMIZATION} -nostdlib)

# Flags for C/C++
string(TOUPPER ${BOARD_NAME} BOARD_NAME_UPPER)
string(TOUPPER ${ARCH_NAME} ARCH_NAME_UPPER)
set(CFLAGS
    -D_BOARD_${BOARD_NAME_UPPER} -D_MIOSIX_BOARDNAME="${BOARD_NAME}"
    -D_DEFAULT_SOURCE=1 -ffunction-sections -Wall -Werror=return-type
    -D_ARCH_${ARCH_NAME_UPPER} -DSTM32F10X_HD
    ${CLOCK_FREQ} ${XRAM} ${FLAGS_BASE} -c
)
set(CXXFLAGS ${CFLAGS} -std=c++14 ${OPT_EXCEPT})

# Select architecture specific files
set(ARCH_SRC
    ${ARCH_PATH}/interfaces-impl/delays.cpp
    ${ARCH_PATH}/interfaces-impl/gpio_impl.cpp
    ${ARCH_PATH}/interfaces-impl/portability.cpp
    ${BOARD_PATH}/interfaces-impl/bsp.cpp
    ${KPATH}/arch/common/CMSIS/Device/ST/STM32F10x/Source/Templates/system_stm32f10x.c
    ${KPATH}/arch/common/core/interrupts_cortexMx.cpp
    ${KPATH}/arch/common/core/stm32f1_l1_os_timer.cpp
    ${KPATH}/arch/common/drivers/dcc.cpp
    ${KPATH}/arch/common/drivers/serial_stm32.cpp
    ${KPATH}/arch/common/drivers/sd_stm32f1.cpp
)
