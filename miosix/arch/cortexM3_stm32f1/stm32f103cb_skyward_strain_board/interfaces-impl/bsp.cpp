/***************************************************************************
 *   Copyright (C) 2015, 2016, 2017, 2018 by Terraneo Federico             *
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

/***********************************************************************
 * bsp.cpp Part of the Miosix Embedded OS.
 * Board support package, this file initializes hardware.
 ************************************************************************/

#include <utility>
#include <sys/ioctl.h>
#include "interfaces/bsp.h"
#include "interfaces/delays.h"
#include "interfaces/arch_registers.h"
#include "config/miosix_settings.h"
#include "filesystem/file_access.h"
#include "filesystem/console/console_device.h"
#include "drivers/serial.h"
#include "board_settings.h"
#include "hwmapping.h"

using namespace std;

namespace miosix {

//
// Initialization
//

void IRQbspInit()
{
    //Enable all gpios, as well as AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN |
                    RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    //Enable SPI1 + CAN
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC_SYNC();

    using namespace interfaces;
    spi1::sck::mode(Mode::ALTERNATE);
    spi1::miso::mode(Mode::INPUT);
    spi1::mosi::mode(Mode::ALTERNATE);

    can::tx::mode(Mode::ALTERNATE);
    can::rx::mode(Mode::INPUT);

    using namespace sensors;
    ads1118::cs::mode(Mode::OUTPUT);
    ads1118::cs::high();

    using namespace leds;
    led1::mode(Mode::OUTPUT);
    led1::low();
    led2::mode(Mode::OUTPUT);
    led2::low();
    led3::mode(Mode::OUTPUT);
    led3::low();

    for (uint8_t i = 0; i < 5; i++)
    {
        ledOn();
        delayMs(100);
        ledOff();
        delayMs(100);
    }

    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
        new STM32Serial(defaultSerial,defaultSerialSpeed,
        defaultSerialFlowctrl ? STM32Serial::RTSCTS : STM32Serial::NOFLOWCTRL)));
}

void bspInit2()
{
}

//
// Shutdown and reboot
//

void shutdown()
{
    reboot(); //This board has no shutdown support, so we reboot on shutdown
}

void reboot()
{
    ioctl(STDOUT_FILENO,IOCTL_SYNC,0);

    disableInterrupts();
    miosix_private::IRQsystemReboot();
}

} //namespace miosix