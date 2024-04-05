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

# Create a custom target to list all the boards
string(REPLACE ";" "\\n" BOARDS_STR "${MIOSIX_BOARDS}")
add_custom_target(
    help-boards
    COMMAND printf ${BOARDS_STR}
    COMMENT "All boards available:"
    VERBATIM
)

# Function to link the Miosix libraries to the target
function(mbs_target TARGET OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()

    # Link libraries
    target_link_libraries(${TARGET} PRIVATE
        $<TARGET_OBJECTS:Miosix::Boot::${OPT_BOARD}>
        $<LINK_GROUP:RESCAN,Miosix::Kernel::${OPT_BOARD},stdc++,c,m,gcc,atomic>
    )

    # Linker script and linking options are eredited from the kernel library

    # Add a post build command to create the hex file to flash on the board
    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET} ${TARGET}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${TARGET}.bin
        BYPRODUCTS ${TARGET}.hex ${TARGET}.bin
        VERBATIM
    )

    # Save custom flash command to file
    get_target_property(PROGRAM_CMDLINE Miosix::Kernel::${OPT_BOARD} PROGRAM_CMDLINE)
    if(NOT PROGRAM_CMDLINE STREQUAL "PROGRAM_CMDLINE-NOTFOUND")
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/flash-${TARGET}.txt" ${PROGRAM_CMDLINE})
    endif()
endfunction()
