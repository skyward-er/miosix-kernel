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

set(BOARD_NAME stm32f746zg_nucleo)
set(ARCH_NAME cortexM7_stm32f7)

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
set(LINKER_SCRIPT ${BOARD_PATH}/stm32_1m+256k_rom.ld)

# Select clock frequency (HSE_VALUE is the xtal on board, fixed)
set(CLOCK_FREQ -DHSE_VALUE=8000000 -DSYSCLK_FREQ_216MHz=216000000)

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
# set(PROGRAM_CMDLINE "here your custom flash command")

# Basic flags
set(FLAGS_BASE -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)

# Flags for ASM and linker
set(AFLAGS_BASE ${FLAGS_BASE})
set(LFLAGS_BASE ${FLAGS_BASE} -Wl,--gc-sections,-Map,main.map ${OPT_EXCEPT} ${OPT_OPTIMIZATION} -nostdlib)

# Flags for C/C++
set(CFLAGS_BASE
    -D_BOARD_STM32F746ZG_NUCLEO "-D_MIOSIX_BOARDNAME=\"${BOARD_NAME}\""
    -D_DEFAULT_SOURCE=1 -ffunction-sections -Wall -Werror=return-type -g
    -D_ARCH_CORTEXM7_STM32F7
    ${CLOCK_FREQ} ${XRAM} ${SRAM_BOOT} ${SD} ${FLAGS_BASE} ${OPT_OPTIMIZATION} -c
)
set(CXXFLAGS_BASE ${CFLAGS_BASE} ${OPT_EXCEPT})

# Select architecture specific files
set(ARCH_SRC
    ${ARCH_PATH}/interfaces-impl/delays.cpp
    ${ARCH_PATH}/interfaces-impl/gpio_impl.cpp
    ${ARCH_PATH}/interfaces-impl/portability.cpp
    ${BOARD_PATH}/interfaces-impl/bsp.cpp
    arch/common/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
    arch/common/core/cache_cortexMx.cpp
    arch/common/core/interrupts_cortexMx.cpp
    arch/common/core/mpu_cortexMx.cpp
    arch/common/core/stm32f2_f4_l4_f7_h7_os_timer.cpp
    arch/common/drivers/dcc.cpp
    arch/common/drivers/sd_stm32f2_f4_f7.cpp
    arch/common/drivers/serial_stm32.cpp
    arch/common/drivers/stm32_hardware_rng.cpp
)
