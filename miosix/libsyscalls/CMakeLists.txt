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

cmake_minimum_required(VERSION 3.16)

project(LibSyscalls C CXX ASM)

add_library(syscalls STATIC crt0.s crt1.cpp memoryprofiling.cpp)

# Use include directories from miosix
get_target_property(MIOSIX_INCLUDE_DIRS miosix INCLUDE_DIRECTORIES)
target_include_directories(syscalls PUBLIC ${MIOSIX_INCLUDE_DIRS})

# Set ASM/C/C++ flags
set(AFLAGS ${CPU})
set(CFLAGS
    ${CPU} -fpie -msingle-pic-base -ffunction-sections -Wall
    -Werror=return-type -D_DEFAULT_SOURCE=1 -c
)
set(CXXFLAGS -std=c++14 ${OPT_EXCEPT} ${CFLAGS})
target_compile_options(syscalls PUBLIC
    $<$<COMPILE_LANGUAGE:ASM>:${AFLAGS}>
    $<$<COMPILE_LANGUAGE:C>:${CFLAGS}>
    $<$<COMPILE_LANGUAGE:CXX>:${CXXFLAGS}>
)

# Configure linker file and options
set(LFLAGS_BASE
    ${CPU} -fpie -msingle-pic-base -nostdlib -Wl,--gc-sections
    -Wl,-Map,process.map,-T${KPATH}/libsyscalls/process.ld # TODO: Check process.map
    -Wl,-n,-pie,--spare-dynamic-tags,3,--target2=mx-data-rel
)
set_property(TARGET syscalls PROPERTY LINK_DEPENDS ${KPATH}/libsyscalls/process.ld)
target_link_options(syscalls PUBLIC ${LFLAGS_BASE})
