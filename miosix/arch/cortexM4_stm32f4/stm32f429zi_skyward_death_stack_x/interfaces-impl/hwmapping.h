/***************************************************************************
 *   Copyright (C) 2018 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#ifndef HWMAPPING_H
#define HWMAPPING_H

#include "interfaces/gpio.h"

namespace miosix
{

namespace interfaces
{

namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

namespace spi2
{
using sck  = Gpio<GPIOB_BASE, 13>;
using miso = Gpio<GPIOB_BASE, 14>;
using mosi = Gpio<GPIOB_BASE, 15>;
}  // namespace spi2

namespace i2c
{
using scl = Gpio<GPIOB_BASE, 8>;
using sda = Gpio<GPIOB_BASE, 9>;
}  // namespace i2c

namespace uart2
{  // GPS UART
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace uart2

namespace uart1
{  // USB UART
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace uart1

namespace can
{  // CAN1
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can

}  // namespace interfaces

namespace sensors
{

namespace ads1118
{
using cs = Gpio<GPIOB_BASE, 1>;
}  // namespace ads1118

namespace bmx160
{
using cs   = Gpio<GPIOA_BASE, 8>;
using intr = Gpio<GPIOE_BASE, 5>;
}  // namespace bmx160

/*namespace lsm9ds1
{
using cs_a_g   = Gpio<GPIOC_BASE, 1>;
using cs_m     = Gpio<GPIOC_BASE, 3>;
using intr_a_g = Gpio<GPIOB_BASE, 12>;
}  // namespace lsm9ds1 */

namespace lis3mdl
{
using cs = Gpio<GPIOG_BASE, 6>;
}  // namespace lis3mdl

namespace ms5803
{
using cs = Gpio<GPIOD_BASE, 7>;
}  // namespace ms5803

}  // namespace sensors

namespace inputs
{
using vbat         = Gpio<GPIOF_BASE, 7>;
using lp_dtch      = Gpio<GPIOE_BASE, 4>;   // launchpad detach
using nc_dtch      = Gpio<GPIOC_BASE, 14>;  // nosecone detach
using expulsion_in = Gpio<GPIOB_BASE, 7>;   // expulsion sensor
}  // namespace inputs

namespace actuators
{

namespace nosecone
{
using nc_servo_pwm = Gpio<GPIOD_BASE, 12>;  // Pwm expulsion servo, TIM4_CH1
// using th_cut_pwm   = Gpio<GPIOE_BASE, 6>;   // Pwm thermal cutters, TIM9_CH2
using th_cut_input = Gpio<GPIOE_BASE, 6>;  // Input thermal cutters

namespace thCut1
{
using ena   = Gpio<GPIOG_BASE, 2>;
using csens = Gpio<GPIOF_BASE, 6>;  // ADC3 CH4
}  // namespace thCut1

namespace thCut2
{
using ena   = Gpio<GPIOD_BASE, 11>;
using csens = Gpio<GPIOF_BASE, 8>;  // ADC3 CH6
}  // namespace thCut2
}  // namespace nosecone

namespace airbrakes
{
using airbrakes_servo_pwm =
    Gpio<GPIOC_BASE, 7>;  // Pwm airbrakes actuation servo, TIM8_CH2
}  // namespace airbrakes

}  // namespace actuators

namespace misc
{
using aux1        = Gpio<GPIOE_BASE, 2>;
using aux2        = Gpio<GPIOE_BASE, 3>;
using aux_pd_pu   = Gpio<GPIOC_BASE, 5>;
using aux_spi1_cs = Gpio<GPIOG_BASE, 7>;
}  // namespace misc

namespace leds
{
using led_red1  = Gpio<GPIOG_BASE, 7>;
using led_red2  = Gpio<GPIOG_BASE, 10>;
using led_blue1 = Gpio<GPIOG_BASE, 14>;

/* NOTE:
 * These are conencted to the enable pin of the thermal
 * cutters and the cs of the lis3mdl magnetometer
 */
using led_blue2  = Gpio<GPIOG_BASE, 2>;
using led_green1 = Gpio<GPIOG_BASE, 6>;
using led_green2 = Gpio<GPIOD_BASE, 11>;
}  // namespace leds

namespace xbee
{
using cs    = Gpio<GPIOF_BASE, 9>;
using attn  = Gpio<GPIOF_BASE, 10>;
using reset = Gpio<GPIOC_BASE, 13>;
}  // namespace xbee

}  // namespace miosix

#endif  // HWMAPPING_H
