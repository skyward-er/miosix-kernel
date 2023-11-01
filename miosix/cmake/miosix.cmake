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

# Load in KPATH the project path
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH KPATH)

# Include board list
include(${KPATH}/cmake/boards.cmake)

# Miosix source files
set(MIOSIX_SRC
    ${KPATH}/kernel/kernel.cpp
    ${KPATH}/kernel/sync.cpp
    ${KPATH}/kernel/error.cpp
    ${KPATH}/kernel/pthread.cpp
    ${KPATH}/kernel/stage_2_boot.cpp
    ${KPATH}/kernel/elf_program.cpp
    ${KPATH}/kernel/process.cpp
    ${KPATH}/kernel/process_pool.cpp
    ${KPATH}/kernel/timeconversion.cpp
    ${KPATH}/kernel/intrusive.cpp 
    ${KPATH}/kernel/SystemMap.cpp
    ${KPATH}/kernel/cpu_time_counter.cpp   
    ${KPATH}/kernel/scheduler/priority/priority_scheduler.cpp
    ${KPATH}/kernel/scheduler/control/control_scheduler.cpp
    ${KPATH}/kernel/scheduler/edf/edf_scheduler.cpp
    ${KPATH}/filesystem/file_access.cpp
    ${KPATH}/filesystem/file.cpp
    ${KPATH}/filesystem/stringpart.cpp
    ${KPATH}/filesystem/console/console_device.cpp
    ${KPATH}/filesystem/mountpointfs/mountpointfs.cpp
    ${KPATH}/filesystem/devfs/devfs.cpp
    ${KPATH}/filesystem/fat32/fat32.cpp
    ${KPATH}/filesystem/fat32/ff.cpp
    ${KPATH}/filesystem/fat32/diskio.cpp
    ${KPATH}/filesystem/fat32/wtoupper.cpp
    ${KPATH}/filesystem/fat32/ccsbcs.cpp
    ${KPATH}/stdlib_integration/libc_integration.cpp
    ${KPATH}/stdlib_integration/libstdcpp_integration.cpp
    ${KPATH}/e20/e20.cpp
    ${KPATH}/e20/unmember.cpp
    ${KPATH}/util/util.cpp
    ${KPATH}/util/unicode.cpp
    ${KPATH}/util/version.cpp
    ${KPATH}/util/crc16.cpp
    ${KPATH}/util/lcd44780.cpp
)

# Function to configure a target to be built for the kernel
function(configure_target_for_kernel TARGET)
    # Include kernel directories
    target_include_directories(${TARGET} PUBLIC
        ${KPATH}
        ${KPATH}/arch/common
        ${ARCH_PATH}
        ${BOARD_PATH}
        ${BOARD_CONFIG_PATH}
    )

    # Set include path where to find config/miosix_settings.h
    if(DEFINED BOARD_MIOSIX_SETTINGS_PATH)
        target_include_directories(${TARGET} PUBLIC ${BOARD_MIOSIX_SETTINGS_PATH})
    else()
        target_include_directories(${TARGET} PUBLIC ${KPATH}/default)
    endif()

    # Define COMPILING_MIOSIX (only for C and C++)
    target_compile_definitions(${TARGET} PRIVATE $<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:COMPILING_MIOSIX>)

    # Require cpp14 target_compile_features (this will add the -std=c++14 flag)
    target_compile_features(${TARGET} PUBLIC cxx_std_14)

    # Configure compiler flags
    target_compile_options(${TARGET} PUBLIC
        $<$<COMPILE_LANGUAGE:ASM>:${AFLAGS_BASE}>
        $<$<COMPILE_LANGUAGE:C>:${DFLAGS} ${CFLAGS_BASE}>
        $<$<COMPILE_LANGUAGE:CXX>:${DFLAGS} ${CXXFLAGS_BASE}>
    )
endfunction()

# Creates the Miosix::Boot::${BOARD_NAME} and Miosix::Kernel::${BOARD_NAME} libraries
function(add_miosix_libraries BOARD_OPTIONS_FILE)
    # Get board options
    include(${BOARD_OPTIONS_FILE})

    # Create a library for the boot file
    set(BOOT_LIB boot-${BOARD_NAME})
    add_library(${BOOT_LIB} OBJECT EXCLUDE_FROM_ALL ${BOOT_FILE})
    configure_target_for_kernel(${BOOT_LIB})

    # Create a library for the rest of the kernel
    set(KERNEL_LIB kernel-${BOARD_NAME})
    add_library(${KERNEL_LIB} STATIC EXCLUDE_FROM_ALL ${MIOSIX_SRC} ${ARCH_SRC})
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
    add_library(Miosix::Boot::${BOARD_NAME} ALIAS ${BOOT_LIB})
    add_library(Miosix::Kernel::${BOARD_NAME} ALIAS ${KERNEL_LIB})
endfunction()

# Create Miosix libraries for each board
foreach(BOARD_OPTIONS_FILE ${MIOSIX_BOARDS_OPTIONS_FILES})
    add_miosix_libraries(${BOARD_OPTIONS_FILE})
endforeach()
