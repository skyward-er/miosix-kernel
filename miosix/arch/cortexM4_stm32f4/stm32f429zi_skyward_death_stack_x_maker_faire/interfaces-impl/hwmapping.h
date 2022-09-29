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

using RANDOM_PIN = miosix::Gpio<GPIOJ_BASE, 0>;

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

namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

// SX127x
namespace spi5
{
using sck  = RANDOM_PIN;
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

// USB UART
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// GPS UART
namespace usart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart2

namespace usart3
{
using tx = Gpio<GPIOB_BASE, 10>;
using rx = Gpio<GPIOB_BASE, 11>;
}  // namespace usart3

namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

// CAN1
namespace can1
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can1

}  // namespace interfaces

namespace sensors
{

namespace ads131m04
{
using cs1  = RANDOM_PIN;
using cs2  = RANDOM_PIN;  // TIM1_CH1
using sck2 = RANDOM_PIN;  // TIM3_CH4
}  // namespace ads131m04

namespace bmx160
{
using cs   = Gpio<GPIOA_BASE, 8>;
using intr = Gpio<GPIOE_BASE, 5>;
}  // namespace bmx160

namespace ms5803
{
using cs = Gpio<GPIOD_BASE, 7>;
}  // namespace ms5803

namespace mpu9250
{
using cs = RANDOM_PIN;
}  // namespace mpu9250

namespace cc3135
{
using cs   = RANDOM_PIN;
using intr = RANDOM_PIN;
}  // namespace cc3135

namespace sx127x
{
using cs   = RANDOM_PIN;
using dio0 = RANDOM_PIN;
}  // namespace sx127x

namespace gps
{
using cs = RANDOM_PIN;
}  // namespace gps

namespace mlx91221_1
{
using vout = RANDOM_PIN;  // ADC
}  // namespace mlx91221_1

namespace mlx91221_2
{
using vout = RANDOM_PIN;  // ADC
}  // namespace mlx91221_2

using launchpad_detach = RANDOM_PIN;  // launchpad detach

}  // namespace sensors

namespace inputs
{
using vbat = Gpio<GPIOF_BASE, 7>;
}  // namespace inputs

namespace expulsion
{
using servo           = Gpio<GPIOB_BASE, 7>;  // Pwm expulsion servo, TIM4_CH2
using sense           = RANDOM_PIN;           // Expulsion sensor
using nosecone_detach = RANDOM_PIN;           // Nosecone detach
}  // namespace expulsion

namespace cutter
{
using enable        = Gpio<GPIOG_BASE, 2>;
using enable_backup = Gpio<GPIOD_BASE, 11>;
using sense         = Gpio<GPIOF_BASE, 6>;
}  // namespace cutter

namespace airbrakes
{
using servo  = Gpio<GPIOC_BASE, 7>;  // Airbrakes PWM, TIM8_CH2
using sensor = RANDOM_PIN;           // Airbrakes ADC
}  // namespace airbrakes

namespace leds
{
using red       = Gpio<GPIOG_BASE, 7>;
using green     = Gpio<GPIOE_BASE, 3>;
using blue      = Gpio<GPIOG_BASE, 14>;
using led_ring  = Gpio<GPIOE_BASE, 3>;
using led_blue1 = Gpio<GPIOG_BASE, 14>;
}  // namespace leds

namespace buzzer
{
using drive = RANDOM_PIN;  // PWM TIM8_CH1
}  // namespace buzzer

namespace aux
{
using servo   = RANDOM_PIN;  // TIM11_CH1
using sense_1 = RANDOM_PIN;
using sense_2 = RANDOM_PIN;
}  // namespace aux

}  // namespace miosix

#endif  // HWMAPPING_H
