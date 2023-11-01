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

set(MIOSIX_BOARDS_OPTIONS_FILES
    ${KPATH}/config/arch/arm7_lpc2000/lpc2138_miosix_board/board_options.cmake
    ${KPATH}/config/arch/cortexM0_stm32f0/stm32f072rb_stm32f0discovery/board_options.cmake
    ${KPATH}/config/arch/cortexM3_efm32gg/efm32gg332f1024_wandstem/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f100c8_microboard/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f100c8_vaisala_rs41/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f100cb_tempsensor/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f100cx_generic/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f100rb_stm32vldiscovery/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f100rc_solertegiard/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f103c8_breakout/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f103cb_als_mainboard_rev2/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f103cx_generic/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f103ve_mp3v2/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f103ve_strive_mini/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f103ze_redbull_v2/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f1/stm32f103ze_stm3210e-eval/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f2/stm32f205_generic/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f2/stm32f205rc_skyward_stormtrooper/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f2/stm32f205rg_sony-newman/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f2/stm32f207ig_stm3220g-eval/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f2/stm32f207ze_als_camboard/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f2/stm32f207zg_EthBoardV2/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32f2/stm32f207zg_nucleo/board_options.cmake
    ${KPATH}/config/arch/cortexM3_stm32l1/stm32l151c8_als_mainboard/board_options.cmake
    ${KPATH}/config/arch/cortexM4_atsam4l/atsam4lc2aa_generic/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f3/stm32f303vc_stm32f3discovery/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f401re_nucleo/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f401vc_stm32f4discovery/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f407vg_bitsboard/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f407vg_stm32f4discovery/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f407vg_thermal_test_chip/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f411ce_blackpill/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f411re_nucleo/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f429zi_oledboard2/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f429zi_skyward_anakin/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f429zi_skyward_homeone/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f429zi_stm32f4discovery/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32f4/stm32f469ni_stm32f469i-disco/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32l4/stm32l4r9zi_sensortile/board_options.cmake
    ${KPATH}/config/arch/cortexM4_stm32l4/stm32l476rg_nucleo/board_options.cmake
    ${KPATH}/config/arch/cortexM7_stm32f7/stm32f746zg_nucleo/board_options.cmake
    ${KPATH}/config/arch/cortexM7_stm32f7/stm32f767zi_nucleo/board_options.cmake
    ${KPATH}/config/arch/cortexM7_stm32f7/stm32f769ni_discovery/board_options.cmake
    ${KPATH}/config/arch/cortexM7_stm32h7/stm32h753xi_eval/board_options.cmake
)
