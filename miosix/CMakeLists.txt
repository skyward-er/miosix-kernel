# Copyright (c) 2021 Skyward Experimental Rocketry
# Authors: Damiano Amatruda, Michele Scuttari
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

cmake_minimum_required(VERSION 3.16)
project(Miosix C CXX ASM)

set(KPATH ${CMAKE_CURRENT_LIST_DIR})

include(${KPATH}/config/boards.cmake)

foreach(OPT_BOARD ${BOARDS})
    set(DFLAGS -MMD -MP)
    include(${KPATH}/config/options.cmake)

    set(MIOSIX_LIBRARY miosix-${OPT_BOARD})
    add_library(${MIOSIX_LIBRARY} STATIC
        kernel/kernel.cpp
        kernel/sync.cpp
        kernel/error.cpp
        kernel/pthread.cpp
        kernel/stage_2_boot.cpp
        kernel/elf_program.cpp
        kernel/process.cpp
        kernel/process_pool.cpp
        kernel/timeconversion.cpp
        kernel/SystemMap.cpp
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
    add_library(Miosix::Miosix-${OPT_BOARD} ALIAS ${MIOSIX_LIBRARY})
    target_sources(${MIOSIX_LIBRARY} INTERFACE ${BOOT_FILE})
    target_include_directories(${MIOSIX_LIBRARY} PUBLIC
        ${KPATH}/config
        ${KPATH}/config/${BOARD_INC}
        ${KPATH}
        ${KPATH}/arch/common
        ${KPATH}/${ARCH_INC}
        ${KPATH}/${BOARD_INC}
    )
    target_compile_definitions(${MIOSIX_LIBRARY} PUBLIC COMPILING_MIOSIX $<$<CONFIG:Debug>:DEBUG>)
    target_compile_features(${MIOSIX_LIBRARY} PUBLIC cxx_std_14)
    target_compile_options(${MIOSIX_LIBRARY} PUBLIC ${DFLAGS} $<$<COMPILE_LANGUAGE:C>:${CFLAGS_BASE}> $<$<COMPILE_LANGUAGE:CXX>:${CXXFLAGS_BASE}> $<$<COMPILE_LANGUAGE:ASM>:${AFLAGS_BASE}>)
    set_property(TARGET ${MIOSIX_LIBRARY} PROPERTY INTERFACE_LINK_DEPENDS ${LINKER_SCRIPT})
    target_link_options(${MIOSIX_LIBRARY} INTERFACE ${LFLAGS_BASE})
    target_link_libraries(${MIOSIX_LIBRARY} INTERFACE stdc++ c m gcc atomic)
    add_custom_command(
        TARGET ${MIOSIX_LIBRARY} POST_BUILD
        COMMAND perl ${KPATH}/_tools/kernel_global_objects.pl $<TARGET_OBJECTS:${MIOSIX_LIBRARY}>
        VERBATIM
        COMMAND_EXPAND_LISTS
    )
endforeach()
