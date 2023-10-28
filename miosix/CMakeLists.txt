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
project(Miosix C CXX ASM)

# Get the kernel folder
set(KPATH ${CMAKE_CURRENT_LIST_DIR})

# Include board list
include(${KPATH}/cmake/boards.cmake)

# Function to configure a target to be built with kernel sources
function(configure_target_for_kernel TARGET)
    # Include kernel directories
    target_include_directories(${TARGET} PUBLIC
        ${KPATH}
        ${KPATH}/config/${BOARD_INC}
        ${KPATH}/arch/common
        ${KPATH}/${ARCH_INC}
        ${KPATH}/${BOARD_INC}
    )

    # Define COMPILING_MIOSIX
    target_compile_definitions(${TARGET} PRIVATE $<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:COMPILING_MIOSIX>)

    # Require cpp 14target_compile_features
    target_compile_features(${TARGET} PUBLIC cxx_std_14)

    # Configure compiler flags
    set(DFLAGS -MMD -MP)
    target_compile_options(${TARGET} PUBLIC
        $<$<COMPILE_LANGUAGE:ASM>:${AFLAGS_BASE}>
        $<$<COMPILE_LANGUAGE:C>:${DFLAGS} ${CFLAGS_BASE}>
        $<$<COMPILE_LANGUAGE:CXX>:${DFLAGS} ${CXXFLAGS_BASE}>
    )
endfunction()

# Create a library for each board
foreach(OPT_BOARD ${BOARDS})
    # Use block to create a scope and avoid variable leaking between loops
    block()
        # Use options.cmake to get board specific variables
        include(${KPATH}/config/options.cmake)

        # Create a library for the boot file
        set(BOOT_LIB boot-${OPT_BOARD})
        add_library(${BOOT_LIB} OBJECT ${BOOT_FILE})
        configure_target_for_kernel(${BOOT_LIB})

        # Create a library for the rest of the kernel
        set(KERNEL_LIB kernel-${OPT_BOARD})
        add_library(${KERNEL_LIB} STATIC
            kernel/kernel.cpp
            kernel/sync.cpp
            kernel/error.cpp
            kernel/pthread.cpp
            kernel/stage_2_boot.cpp
            kernel/elf_program.cpp
            kernel/process.cpp
            kernel/process_pool.cpp
            kernel/timeconversion.cpp
            kernel/intrusive.cpp 
            kernel/SystemMap.cpp
            kernel/cpu_time_counter.cpp   
            kernel/scheduler/priority/priority_scheduler.cpp
            kernel/scheduler/control/control_scheduler.cpp
            kernel/scheduler/edf/edf_scheduler.cpp
            filesystem/file_access.cpp
            filesystem/file.cpp
            filesystem/stringpart.cpp
            filesystem/console/console_device.cpp
            filesystem/mountpointfs/mountpointfs.cpp
            filesystem/devfs/devfs.cpp
            filesystem/fat32/fat32.cpp
            filesystem/fat32/ff.cpp
            filesystem/fat32/diskio.cpp
            filesystem/fat32/wtoupper.cpp
            filesystem/fat32/ccsbcs.cpp
            stdlib_integration/libc_integration.cpp
            stdlib_integration/libstdcpp_integration.cpp
            e20/e20.cpp
            e20/unmember.cpp
            util/util.cpp
            util/unicode.cpp
            util/version.cpp
            util/crc16.cpp
            util/lcd44780.cpp
            ${ARCH_SRC}
        )
        configure_target_for_kernel(${KERNEL_LIB})
        add_custom_command(
            TARGET ${KERNEL_LIB} PRE_LINK
            COMMAND perl ${KPATH}/_tools/kernel_global_objects.pl $<TARGET_OBJECTS:${KERNEL_LIB}>
            VERBATIM
            COMMAND_EXPAND_LISTS
        )

        # Configure linker file and options
        set_property(TARGET ${KERNEL_LIB} PROPERTY INTERFACE_LINK_DEPENDS ${LINKER_SCRIPT})
        target_link_options(${KERNEL_LIB} INTERFACE ${LFLAGS_BASE})

        # Configure flash command
        set_property(TARGET ${KERNEL_LIB} PROPERTY PROGRAM_CMDLINE ${PROGRAM_CMDLINE})

        # Create nice aliases for the libraries
        add_library(Miosix::Boot::${OPT_BOARD} ALIAS ${BOOT_LIB})
        add_library(Miosix::Kernel::${OPT_BOARD} ALIAS ${KERNEL_LIB})
    endblock()
endforeach()
