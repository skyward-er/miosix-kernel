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

#pragma once

#include "interfaces/gpio.h"

namespace miosix
{

namespace interfaces
{

namespace spi1
{
using cs   = Gpio<GPIOA_BASE, 4>;
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

namespace spi4
{
using cs   = Gpio<GPIOE_BASE, 4>;
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

namespace spi5
{
using sck  = Gpio<GPIOF_BASE, 7>;
using miso = Gpio<GPIOF_BASE, 8>;
using mosi = Gpio<GPIOF_BASE, 9>;
}  // namespace spi5

namespace uart1
{  // USB UART
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace uart1

namespace uart3
{  // GPS UART
using tx = Gpio<GPIOB_BASE, 10>;
using rx = Gpio<GPIOB_BASE, 11>;
}  // namespace uart3

namespace timers
{
using tim1ch4  = Gpio<GPIOA_BASE, 11>;
using tim3ch4  = Gpio<GPIOB_BASE, 1>;
using tim4ch2  = Gpio<GPIOB_BASE, 7>;
using tim8ch2  = Gpio<GPIOC_BASE, 7>;
using tim9ch1  = Gpio<GPIOA_BASE, 2>;
using tim9ch2  = Gpio<GPIOA_BASE, 3>;
using tim10ch1 = Gpio<GPIOB_BASE, 8>;
using tim11ch1 = Gpio<GPIOB_BASE, 9>;
}  // namespace timers

}  // namespace interfaces

namespace sensors
{

namespace ads131m04_1
{
using cs        = interfaces::spi1::cs;
using sck       = interfaces::spi1::sck;
using miso      = interfaces::spi1::miso;
using mosi      = interfaces::spi1::mosi;
using cs_timer  = interfaces::timers::tim1ch4;
using sck_timer = interfaces::timers::tim3ch4;
}  // namespace ads131m04_1

namespace ads131m04_2
{
using cs        = interfaces::spi4::cs;
using sck       = interfaces::spi4::sck;
using miso      = interfaces::spi4::miso;
using mosi      = interfaces::spi4::mosi;
using cs_timer  = interfaces::timers::tim4ch2;
using sck_timer = interfaces::timers::tim8ch2;
}  // namespace ads131m04_2

namespace imu
{
using cs   = Gpio<GPIOF_BASE, 6>;
using sck  = interfaces::spi5::sck;
using miso = interfaces::spi5::miso;
using mosi = interfaces::spi5::mosi;
using intr = Gpio<GPIOF_BASE, 10>;
}  // namespace imu

namespace thermocouples
{
using cs1  = Gpio<GPIOC_BASE, 13>;
using cs2  = Gpio<GPIOC_BASE, 14>;
using cs3  = Gpio<GPIOC_BASE, 15>;
using cs4  = Gpio<GPIOD_BASE, 11>;
using cs5  = Gpio<GPIOD_BASE, 12>;
using cs6  = Gpio<GPIOD_BASE, 13>;
using sck  = interfaces::spi5::sck;
using miso = interfaces::spi5::miso;
using mosi = interfaces::spi5::mosi;
}  // namespace thermocouples

}  // namespace sensors

namespace actuators
{

namespace leds
{
using led1 = Gpio<GPIOD_BASE, 3>;
using led2 = Gpio<GPIOD_BASE, 4>;
using led3 = Gpio<GPIOD_BASE, 5>;
}  // namespace leds

namespace servos
{
using servo1 = interfaces::timers::tim10ch1;
using servo2 = interfaces::timers::tim11ch1;
using servo3 = interfaces::timers::tim9ch1;
using servo4 = interfaces::timers::tim9ch2;
}  // namespace servos

}  // namespace actuators

}  // namespace miosix
