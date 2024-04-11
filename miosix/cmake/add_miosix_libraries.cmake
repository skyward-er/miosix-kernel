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

# Parameters:
# - The path to the bps board_options.cmake file
# Creates three libraries:
# - Miosix::board-${BOARD_NAME}
# - Miosix::boot-${BOARD_NAME}
# - Miosix::kernel-${BOARD_NAME}
function(add_miosix_libraries BOARD_OPTIONS_FILE)
    # Get board options and kernel sources
    # The files are included here to scope all the variables
    include(${BOARD_OPTIONS_FILE})
    include(${KPATH}/cmake/kernel_sources.cmake)

    ## Board-specific interface library

    # This defined all the include directories and compile options needed to compile kernel code 

    set(BOARD_LIB board-${BOARD_NAME})
    add_library(${BOARD_LIB} INTERFACE EXCLUDE_FROM_ALL)
    add_library(Miosix::${BOARD_LIB} ALIAS ${BOARD_LIB})

    # Include kernel directories
    target_include_directories(${BOARD_LIB} INTERFACE
        $<BUILD_INTERFACE:${KPATH}>  
        $<INSTALL_INTERFACE:Miosix>
        $<BUILD_INTERFACE:${KPATH}/arch/common>  
        $<INSTALL_INTERFACE:Miosix/arch/common>
        $<BUILD_INTERFACE:${KPATH}/${ARCH_PATH}>  
        $<INSTALL_INTERFACE:Miosix/${ARCH_PATH}>
        $<BUILD_INTERFACE:${KPATH}/${BOARD_PATH}>  
        $<INSTALL_INTERFACE:Miosix/${BOARD_PATH}>
        $<BUILD_INTERFACE:${KPATH}/${BOARD_CONFIG_PATH}>  
        $<INSTALL_INTERFACE:Miosix/${BOARD_CONFIG_PATH}>
    )

    # The user can set a custom path for miosix_settings.h by setting the variable CUSTOM_BOARD_MIOSIX_SETTINGS_PATH
    if(DEFINED CUSTOM_BOARD_MIOSIX_SETTINGS_PATH)
        target_include_directories(${BOARD_LIB} INTERFACE ${CUSTOM_BOARD_MIOSIX_SETTINGS_PATH})
    else()
        # By default miosix/default/config/miosix_settings.h is used
        target_include_directories(${BOARD_LIB} INTERFACE
            $<BUILD_INTERFACE:${KPATH}/default>  
            $<INSTALL_INTERFACE:Miosix/default>
        )
    endif()

    # Add COMPILING_MIOSIX define (only for C and C++)
    target_compile_definitions(${BOARD_LIB} INTERFACE $<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:COMPILING_MIOSIX>)

    # Require cpp14 target_compile_features (this will add the -std=c++14 flag)
    target_compile_features(${BOARD_LIB} INTERFACE cxx_std_14)

    # Configure compiler flags
    target_compile_options(${BOARD_LIB} INTERFACE
        $<$<COMPILE_LANGUAGE:ASM>:${AFLAGS_BASE}>
        $<$<COMPILE_LANGUAGE:C>:${DFLAGS} ${CFLAGS_BASE}>
        $<$<COMPILE_LANGUAGE:CXX>:${DFLAGS} ${CXXFLAGS_BASE}>
    )

    # Configure linker file and options
    set_property(TARGET ${BOARD_LIB} PROPERTY INTERFACE_LINK_DEPENDS
        $<BUILD_INTERFACE:${KPATH}/${LINKER_SCRIPT}>  
        $<INSTALL_INTERFACE:Miosix/${LINKER_SCRIPT}>
    )
    target_link_options(${BOARD_LIB} INTERFACE ${LFLAGS_BASE}
        $<BUILD_INTERFACE:-Wl,-T${KPATH}/${LINKER_SCRIPT}>
        $<INSTALL_INTERFACE:-Wl,-TMiosix/${LINKER_SCRIPT}>
    )

    # Configure flash command
    set_property(TARGET ${BOARD_LIB} PROPERTY PROGRAM_CMDLINE ${PROGRAM_CMDLINE})

    ## Boot library

    set(BOOT_LIB boot-${BOARD_NAME})
    add_library(${BOOT_LIB} OBJECT ${KPATH}/${BOOT_FILE})
    add_library(Miosix::${BOOT_LIB} ALIAS ${BOOT_LIB})
    target_link_libraries(${BOOT_LIB} PUBLIC ${BOARD_LIB})

    # Kernel library

    set(KERNEL_LIB kernel-${BOARD_NAME})
    add_library(${KERNEL_LIB} STATIC ${KERNEL_SRC} ${ARCH_SRC})
    add_library(Miosix::${KERNEL_LIB} ALIAS ${KERNEL_LIB})
    target_link_libraries(${KERNEL_LIB} PUBLIC ${BOARD_LIB})
    add_custom_command(
        TARGET ${KERNEL_LIB} PRE_LINK
        COMMAND perl ${KPATH}/_tools/kernel_global_objects.pl $<TARGET_OBJECTS:${KERNEL_LIB}>
        VERBATIM
        COMMAND_EXPAND_LISTS
    )
endfunction()