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

set(MIOSIX_BOARDS
    lpc2138_miosix_board
    stm32f072rb_stm32f0discovery
    efm32gg332f1024_wandstem
    stm32f100c8_microboard
    stm32f100c8_vaisala_rs41
    stm32f100cb_tempsensor
    stm32f100cx_generic
    stm32f100rb_stm32vldiscovery
    stm32f100rc_solertegiard
    stm32f103c8_breakout
    stm32f103cb_als_mainboard_rev2
    stm32f103cx_generic
    stm32f103ve_mp3v2
    stm32f103ve_strive_mini
    stm32f103ze_redbull_v2
    stm32f103ze_stm3210e-eval
    stm32f205_generic
    stm32f205rc_skyward_stormtrooper
    stm32f205rg_sony-newman
    stm32f207ig_stm3220g-eval
    stm32f207ze_als_camboard
    stm32f207zg_EthBoardV2
    stm32f207zg_nucleo
    stm32l151c8_als_mainboard
    atsam4lc2aa_generic
    stm32f303vc_stm32f3discovery
    stm32f401re_nucleo
    stm32f401vc_stm32f4discovery
    stm32f407vg_bitsboard
    stm32f407vg_stm32f4discovery
    stm32f407vg_thermal_test_chip
    stm32f411ce_blackpill
    stm32f411re_nucleo
    stm32f429zi_oledboard2
    stm32f429zi_skyward_anakin
    stm32f429zi_skyward_homeone
    stm32f429zi_stm32f4discovery
    stm32f469ni_stm32f469i-disco
    stm32l4r9zi_sensortile
    stm32l476rg_nucleo
    stm32f746zg_nucleo
    stm32f767zi_nucleo
    stm32f769ni_discovery
    stm32h753xi_eval
)

function(get_board_architecture BOARD)
    if(${BOARD} STREQUAL lpc2138_miosix_board)
        set(ARCH arm7_lpc2000 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f072rb_stm32f0discovery)
        set(ARCH cortexM0_stm32f0 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL efm32gg332f1024_wandstem)
        set(ARCH cortexM3_efm32gg PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f100c8_microboard)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f100c8_vaisala_rs41)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f100cb_tempsensor)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f100cx_generic)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f100rb_stm32vldiscovery)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f100rc_solertegiard)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f103c8_breakout)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f103cb_als_mainboard_rev2)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f103cx_generic)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f103ve_mp3v2)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f103ve_strive_mini)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f103ze_redbull_v2)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f103ze_stm3210e-eval)
        set(ARCH cortexM3_stm32f1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f205_generic)
        set(ARCH cortexM3_stm32f2 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f205rc_skyward_ciuti)
        set(ARCH cortexM3_stm32f2 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f205rc_skyward_stormtrooper)
        set(ARCH cortexM3_stm32f2 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f205rg_sony-newman)
        set(ARCH cortexM3_stm32f2 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f207ig_stm3220g-eval)
        set(ARCH cortexM3_stm32f2 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f207ze_als_camboard)
        set(ARCH cortexM3_stm32f2 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f207zg_EthBoardV2)
        set(ARCH cortexM3_stm32f2 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f207zg_nucleo)
        set(ARCH cortexM3_stm32f2 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32l151c8_als_mainboard)
        set(ARCH cortexM3_stm32l1 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL atsam4lc2aa_generic)
        set(ARCH cortexM4_atsam4l PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f303vc_stm32f3discovery)
        set(ARCH cortexM4_stm32f3 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f401re_nucleo)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f401vc_stm32f4discovery)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f407vg_bitsboard)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f407vg_stm32f4discovery)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f407vg_thermal_test_chip)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f411ce_blackpill)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f411re_nucleo)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_oledboard2)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_skyward_anakin)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_skyward_death_stack_v1)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_skyward_death_stack_v2)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_skyward_death_stack_v3)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_skyward_groundstation_nokia)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_skyward_homeone)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_skyward_pyxis_auxiliary)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f429zi_stm32f4discovery)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f469ni_stm32f469i-disco)
        set(ARCH cortexM4_stm32f4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32l4r9zi_sensortile)
        set(ARCH cortexM4_stm32l4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32l476rg_nucleo)
        set(ARCH cortexM4_stm32l4 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f746zg_nucleo)
        set(ARCH cortexM7_stm32f7 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f756zg_nucleo)
        set(ARCH cortexM7_stm32f7 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f767zi_compute_unit)
        set(ARCH cortexM7_stm32f7 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f767zi_skyward_death_stack_v4)
        set(ARCH cortexM7_stm32f7 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f767zi_nucleo)
        set(ARCH cortexM7_stm32f7 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32f769ni_discovery)
        set(ARCH cortexM7_stm32f7 PARENT_SCOPE)
    elseif(${BOARD} STREQUAL stm32h753xi_eval)
        set(ARCH cortexM7_stm32h7 PARENT_SCOPE)
    else()
        message(FATAL_ERROR "${BOARD} board not found in miosix/cmake/boards.cmake")
    endif()
endfunction()
