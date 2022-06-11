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

// Remember to modify pins of leds
namespace miosix
{

namespace interfaces
{

// ADS131M04
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// MS5803 - GPS
namespace spi2
{
using sck  = Gpio<GPIOB_BASE, 13>;
using miso = Gpio<GPIOB_BASE, 14>;
using mosi = Gpio<GPIOB_BASE, 15>;
}  // namespace spi2

namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOB_BASE, 5>;
}  // namespace spi3

// BMX160 - MPU9250
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

// SX127x
namespace spi5
{
using sck  = Gpio<GPIOF_BASE, 7>;
using miso = Gpio<GPIOF_BASE, 8>;
using mosi = Gpio<GPIOF_BASE, 9>;
}  // namespace spi5

// CC3135
namespace spi6
{
using sck  = Gpio<GPIOG_BASE, 13>;
using miso = Gpio<GPIOG_BASE, 12>;
using mosi = Gpio<GPIOG_BASE, 14>;
}  // namespace spi6

// Debug
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// VN100
namespace usart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart2

// HIL
namespace usart3
{
using tx = Gpio<GPIOB_BASE, 10>;
using rx = Gpio<GPIOB_BASE, 11>;
}  // namespace usart3

// RunCam
namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

namespace uart5
{
using tx = Gpio<GPIOC_BASE, 12>;
using rx = Gpio<GPIOD_BASE, 2>;
}  // namespace uart5

namespace usart6
{
using tx = Gpio<GPIOC_BASE, 6>;
using rx = Gpio<GPIOC_BASE, 7>;
}  // namespace usart6

namespace uart7
{
using tx = Gpio<GPIOE_BASE, 8>;
using rx = Gpio<GPIOE_BASE, 7>;
}  // namespace uart7

namespace uart8
{
using tx = Gpio<GPIOE_BASE, 1>;
using rx = Gpio<GPIOE_BASE, 0>;
}  // namespace uart8

namespace can1
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can1

namespace can2
{
using rx = Gpio<GPIOB_BASE, 5>;
using tx = Gpio<GPIOB_BASE, 6>;
}  // namespace can2

}  // namespace interfaces

namespace sensors
{

// Removed: lsm9ds1, lis3mdl

namespace ads131m04
{
using cs1  = Gpio<GPIOA_BASE, 4>;
using cs2  = Gpio<GPIOA_BASE, 8>;  // TIM1_CH1
using sck2 = Gpio<GPIOB_BASE, 1>;  // TIM3_CH4
}  // namespace ads131m04

namespace bmx160
{
using cs   = Gpio<GPIOE_BASE, 4>;
using intr = Gpio<GPIOE_BASE, 3>;
}  // namespace bmx160

namespace mpu9250
{
using cs = Gpio<GPIOD_BASE, 7>;
}  // namespace mpu9250

namespace cc3135
{
using cs   = Gpio<GPIOG_BASE, 11>;
using intr = Gpio<GPIOG_BASE, 10>;
}  // namespace cc3135

namespace sx127x
{
using cs   = Gpio<GPIOF_BASE, 6>;
using dio0 = Gpio<GPIOF_BASE, 10>;
}  // namespace sx127x

namespace gps
{
using cs = Gpio<GPIOB_BASE, 12>;
}  // namespace gps

namespace ms5803
{
using cs = Gpio<GPIOD_BASE, 11>;
}  // namespace ms5803

namespace mlx91221_1
{
using vout = Gpio<GPIOC_BASE, 1>;  // ADC
}  // namespace mlx91221_1

namespace mlx91221_2
{
using vout = Gpio<GPIOC_BASE, 2>;  // ADC
}  // namespace mlx91221_2

}  // namespace sensors

namespace inputs
{
using vbat            = Gpio<GPIOF_BASE, 7>;
using launchpad       = Gpio<GPIOD_BASE, 5>;  // launchpad detach
using nosecone_detach = Gpio<GPIOD_BASE, 4>;  // nosecone detach
using expulsion       = Gpio<GPIOD_BASE, 3>;  // expulsion sensor
}  // namespace inputs

namespace actuators
{

namespace nosecone
{
using servo = Gpio<GPIOB_BASE, 7>;  // Pwm expulsion servo, TIM4_CH2

namespace thermal_cutter_1
{
using enable      = Gpio<GPIOG_BASE, 2>;
using cutter_sens = Gpio<GPIOC_BASE, 14>;
}  // namespace thermal_cutter_1

namespace thermal_cutter_1_backup
{
using enable      = Gpio<GPIOG_BASE, 3>;
using cutter_sens = Gpio<GPIOC_BASE, 14>;
}  // namespace thermal_cutter_1_backup

// NOT PRESENT
namespace thermal_cutter_2
{
using enable      = Gpio<GPIOG_BASE, 3>;
using cutter_sens = Gpio<GPIOC_BASE, 14>;
}  // namespace thermal_cutter_2

}  // namespace nosecone

namespace airbrakes
{
using servo  = Gpio<GPIOB_BASE, 8>;  // Airbrakes PWM, TIM10_CH1
using sensor = Gpio<GPIOC_BASE, 3>;  // Airbrakes ADC
}  // namespace airbrakes

}  // namespace actuators

namespace aux
{
using sense_aux_1 = Gpio<GPIOG_BASE, 6>;
using sense_aux_2 = Gpio<GPIOG_BASE, 7>;
using servo       = Gpio<GPIOB_BASE, 9>;  // TIM11_CH1
}  // namespace aux

namespace leds
{
using led_green1 = Gpio<GPIOC_BASE, 5>;
using led_red    = Gpio<GPIOC_BASE, 4>;
using led_blue   = Gpio<GPIOD_BASE, 13>;
using led_green2 = Gpio<GPIOD_BASE, 12>;
}  // namespace leds

namespace buzzer
{
using drive = Gpio<GPIOC_BASE, 6>;  // PWM TIM8_CH1
}  // namespace buzzer

namespace black_box
{
using sdio_clk = Gpio<GPIOC_BASE, 12>;
using sdio_cmd = Gpio<GPIOD_BASE, 2>;
using sdio_d0  = Gpio<GPIOC_BASE, 8>;
using sdio_d1  = Gpio<GPIOC_BASE, 9>;
using sdio_d2  = Gpio<GPIOC_BASE, 10>;
using sdio_d3  = Gpio<GPIOC_BASE, 11>;
}  // namespace black_box

}  // namespace miosix

#endif  // HWMAPPING_H
