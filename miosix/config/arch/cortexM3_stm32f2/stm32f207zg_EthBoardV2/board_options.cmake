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

set(BOARD_NAME stm32f207zg_EthBoardV2)
set(ARCH_NAME cortexM3_stm32f2)

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

# Linker script type, there are two options
# 1) Code in FLASH, stack + heap in internal RAM (file *_rom.ld)
#    the most common choice, available for all microcontrollers
# 2) Code in external RAM, stack + heap in internal RAM
#    (file *_code_in_xram.ld) useful for debugging. Code runs
#    *very* slow compared to FLASH. Works only with a booloader that
#    forwards interrrupts @ 0x60000000 (see miosix/_tools/bootloaders for
#    one). You must -D__CODE_IN_XRAM below.
# set(LINKER_SCRIPT ${BOARD_PATH}/stm32_1m+128k_rom.ld)
set(LINKER_SCRIPT ${BOARD_PATH}/stm32_1m+128k_code_in_xram.ld)

# XRAM is always enabled on this board, even if the _rom linker script
# does not make explicit use of it.
# Uncommenting __CODE_IN_XRAM (with an appropriate linker script selected
# above) allows to run code from external RAM, useful for debugging
# set(XRAM -D__ENABLE_XRAM)
set(XRAM -D__ENABLE_XRAM -D__CODE_IN_XRAM)

# Clock frequency
set(CLOCK_FREQ -DHSE_VALUE=25000000 -DSYSCLK_FREQ_120MHz=120000000)

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
if(${LINKER_SCRIPT} STREQUAL ${BOARD_PATH}/stm32_1m+128k_code_in_xram.ld)
    set(PROGRAM_CMDLINE "_tools/bootloaders/stm32/pc_loader/pc_loader /dev/ttyUSB1 $binary")
else()
    set(PROGRAM_CMDLINE "stm32flash -w $hex -v /dev/ttyUSB1")
endif()

# Basic flags
set(FLAGS_BASE -mcpu=cortex-m3 -mthumb)

# Flags for ASM and linker
set(AFLAGS_BASE ${FLAGS_BASE})
set(LFLAGS_BASE ${FLAGS_BASE} -Wl,--gc-sections,-Map,main.map ${OPT_EXCEPT} ${OPT_OPTIMIZATION} -nostdlib)

# Flags for C/C++
set(CFLAGS_BASE
    -D_BOARD_ETHBOARDV2 "-D_MIOSIX_BOARDNAME=\"${BOARD_NAME}\""
    -D_DEFAULT_SOURCE=1 -ffunction-sections -Wall -Werror=return-type -g
    -D_ARCH_CORTEXM3_STM32F2
    ${CLOCK_FREQ} ${XRAM} ${FLAGS_BASE} ${OPT_OPTIMIZATION} -c
)
set(CXXFLAGS_BASE ${CFLAGS_BASE} ${OPT_EXCEPT})

# Select architecture specific files
set(ARCH_SRC
    ${ARCH_PATH}/interfaces-impl/gpio_impl.cpp
    ${ARCH_PATH}/interfaces-impl/portability.cpp
    ${BOARD_PATH}/interfaces-impl/bsp.cpp
    ${BOARD_PATH}/interfaces-impl/delays.cpp
    arch/common/CMSIS/Device/ST/STM32F2xx/Source/Templates/system_stm32f2xx.c
    arch/common/core/interrupts_cortexMx.cpp
    arch/common/core/mpu_cortexMx.cpp
    arch/common/core/stm32f2_f4_l4_f7_h7_os_timer.cpp
    arch/common/drivers/dcc.cpp
    arch/common/drivers/serial_stm32.cpp
    arch/common/drivers/sd_stm32f2_f4_f7.cpp
)
