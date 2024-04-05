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

project(Miosix
    VERSION 2.7
    DESCRIPTION "Miosix is an OS kernel designed to run on 32bit microcontrollers."
    HOMEPAGE_URL "https://miosix.org/"
    LANGUAGES C CXX
)

# Load in KPATH the project path
set(KPATH ${CMAKE_CURRENT_LIST_DIR})

# Include board list and kernel sources
include(${KPATH}/cmake/boards.cmake)
include(${KPATH}/cmake/kernel_sources.cmake)

# Configures a target to be built for the kernel
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
    add_library(${KERNEL_LIB} STATIC EXCLUDE_FROM_ALL ${KERNEL_SRC} ${ARCH_SRC})
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

# Create libraries for each board
foreach(BOARD_OPTIONS_FILE ${MIOSIX_BOARDS_OPTIONS_FILES})
    add_miosix_libraries(${BOARD_OPTIONS_FILE})
endforeach()
