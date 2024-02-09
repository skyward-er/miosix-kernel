/***************************************************************************
 *   Copyright (C) 2016 by Terraneo Federico                               *
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

#define MIOSIX_RADIO_DIO0_IRQ EXTI1_IRQHandlerImpl
#define MIOSIX_RADIO_DIO1_IRQ EXTI12_IRQHandlerImpl
#define MIOSIX_RADIO_DIO3_IRQ EXTI13_IRQHandlerImpl
#define MIOSIX_RADIO_SPI SPI1

namespace miosix
{

namespace interfaces
{
// Miosix radio SPI
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1
}  // namespace interfaces

namespace btns
{
using ignition = Gpio<GPIOB_BASE, 4>;
using filling  = Gpio<GPIOE_BASE, 6>;
using venting  = Gpio<GPIOE_BASE, 4>;
using release  = Gpio<GPIOG_BASE, 9>;
using detach   = Gpio<GPIOD_BASE, 7>;
using tars     = Gpio<GPIOD_BASE, 5>;
using arm      = Gpio<GPIOE_BASE, 2>;
}  // namespace btns

namespace radio
{
namespace spi
{
using namespace interfaces::spi1;
}

using cs   = Gpio<GPIOF_BASE, 6>;
using dio0 = Gpio<GPIOB_BASE, 1>;
using dio1 = Gpio<GPIOD_BASE, 12>;
using dio3 = Gpio<GPIOD_BASE, 13>;
using txEn = Gpio<GPIOG_BASE, 2>;
using rxEn = Gpio<GPIOG_BASE, 3>;
using nrst = Gpio<GPIOB_BASE, 0>;
}  // namespace radio

namespace ui
{
using buzzer   = Gpio<GPIOB_BASE, 7>;
using armedLed = Gpio<GPIOC_BASE, 13>;
using redLed   = Gpio<GPIOG_BASE, 14>;
}  // namespace ui

}  // namespace miosix

#endif  // HWMAPPING_H
