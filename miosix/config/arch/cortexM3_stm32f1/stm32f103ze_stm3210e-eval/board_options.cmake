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

set(BOARD_NAME stm32f103ze_stm3210e-eval)
set(ARCH_NAME cortexM3_stm32f1)

# Base directories with header files for this board
set(ARCH_PATH arch/${ARCH_NAME}/common)
set(BOARD_PATH arch/${ARCH_NAME}/${BOARD_NAME})
set(BOARD_CONFIG_PATH config/${BOARD_PATH})

# Optimization flags:
# -O0 do no optimization, the default if no optimization level is specified
# -O or -O1 optimize minimally
# -O2 optimize more
# -O3 optimize even more
# -Ofast optimize very aggressively to the point of breaking the standard
# -Og Optimize debugging experience, enables optimizations that do not
# interfere with debugging
# -Os Optimize for size with -O2 optimizations that do not increase code size
set(OPT_OPTIMIZATION -O2)

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
# This is the program that is invoked when the flash flag (-f or --flash) is
# used with the Miosix Build System. Use $binary or $hex as placeolders, they
# will be replaced by the build systems with the binary or hex file repectively.
# If a command is not specified, the build system will use st-flash if found
if(${LINKER_SCRIPT} STREQUAL ${BOARD_INC}/stm32_512k+64k_all_in_xram.ld)
    set(PROGRAM_CMDLINE "_tools/bootloaders/stm32/pc_loader/pc_loader /dev/ttyUSB0 $binary")
else()
    set(PROGRAM_CMDLINE "stm32flash -w $hex -v /dev/ttyUSB0")
endif()

# Basic flags
set(FLAGS_BASE -mcpu=cortex-m3 -mthumb)

# Flags for ASM and linker
set(AFLAGS_BASE ${FLAGS_BASE})
set(LFLAGS_BASE ${FLAGS_BASE} -Wl,--gc-sections,-Map,main.map ${OPT_EXCEPT} ${OPT_OPTIMIZATION} -nostdlib)

# Flags for C/C++
set(CFLAGS_BASE
    -D_BOARD_STM3210E_EVAL "-D_MIOSIX_BOARDNAME=\"${BOARD_NAME}\""
    -D_DEFAULT_SOURCE=1 -ffunction-sections -Wall -Werror=return-type -g
    -D_ARCH_CORTEXM3_STM32F1 -DSTM32F10X_HD
    ${CLOCK_FREQ} ${XRAM} ${FLAGS_BASE} ${OPT_OPTIMIZATION} -c
)
set(CXXFLAGS_BASE ${CFLAGS_BASE} ${OPT_EXCEPT})

# Select architecture specific files
set(ARCH_SRC
    ${ARCH_PATH}/interfaces-impl/delays.cpp
    ${ARCH_PATH}/interfaces-impl/gpio_impl.cpp
    ${ARCH_PATH}/interfaces-impl/portability.cpp
    ${BOARD_PATH}/interfaces-impl/bsp.cpp
    arch/common/CMSIS/Device/ST/STM32F10x/Source/Templates/system_stm32f10x.c
    arch/common/core/interrupts_cortexMx.cpp
    arch/common/core/stm32f1_l1_os_timer.cpp
    arch/common/drivers/dcc.cpp
    arch/common/drivers/serial_stm32.cpp
    arch/common/drivers/sd_stm32f1.cpp
)
