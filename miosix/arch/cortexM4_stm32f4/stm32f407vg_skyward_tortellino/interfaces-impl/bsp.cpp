/***************************************************************************
 *   Copyright (C) 2012, 2013, 2014 by Terraneo Federico                   *
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

#include <cstdlib>
#include <inttypes.h>
#include <sys/ioctl.h>
#include "interfaces/bsp.h"
#include "kernel/kernel.h"
#include "kernel/sync.h"
#include "interfaces/delays.h"
#include "interfaces/portability.h"
#include "interfaces/arch_registers.h"
#include "config/miosix_settings.h"
#include "kernel/logging.h"
#include "filesystem/file_access.h"
#include "filesystem/console/console_device.h"
#include "drivers/serial.h"
#include "drivers/sd_stm32f2_f4.h"
#include "board_settings.h"
#include "hwmapping.h"

namespace miosix {

typedef Gpio<GPIOD_BASE,4>  cs43l22reset;

//
// Initialization
//

void IRQbspInit()
{
    // Enable all gpios
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN;
    RCC_SYNC();

    GPIOA->OSPEEDR=0xaaaaaaaa; //Default to 50MHz speed for all GPIOS
    GPIOB->OSPEEDR=0xaaaaaaaa;
    GPIOC->OSPEEDR=0xaaaaaaaa;
    GPIOD->OSPEEDR=0xaaaaaaaa;
    GPIOE->OSPEEDR=0xaaaaaaaa;
    GPIOH->OSPEEDR=0xaaaaaaaa;

    // On stm32f4discovery some of the SDIO pins conflict with the
    // audio output chip, so keep it permanently reset to avoid issues
    cs43l22reset::mode(Mode::OUTPUT);
    cs43l22reset::low();

    // Peripherals initialization
    using namespace interfaces;
    spi1::sck::mode(Mode::ALTERNATE);
    spi1::sck::alternateFunction(5);
    spi1::miso::mode(Mode::ALTERNATE);
    spi1::miso::alternateFunction(5);
    spi1::mosi::mode(Mode::ALTERNATE);
    spi1::mosi::alternateFunction(5);

    spi2::sck::mode(Mode::ALTERNATE);
    spi2::sck::alternateFunction(5);
    spi2::miso::mode(Mode::ALTERNATE);
    spi2::miso::alternateFunction(5);
    spi2::mosi::mode(Mode::ALTERNATE);
    spi2::mosi::alternateFunction(5);

    using namespace sensors;
    ms5803::cs::mode(Mode::OUTPUT);
    ms5803::cs::high();

    lsm9ds1::cs_ag::mode(Mode::OUTPUT);
    lsm9ds1::cs_ag::high();
    lsm9ds1::cs_mag::mode(Mode::OUTPUT);
    lsm9ds1::cs_mag::high();
    lsm9ds1::int_ag::mode(Mode::INPUT);
    lsm9ds1::drdy_mag::mode(Mode::INPUT);

    slitta::mode(Mode::INPUT);
    finecorsa::mode(Mode::INPUT);

    nosecone_dtch::mode(Mode::INPUT);
    launchpad_dtch::mode(Mode::INPUT);

    using namespace actuators;
    hbridge::is::mode(Mode::INPUT);
    hbridge::in::mode(Mode::OUTPUT);
    hbridge::in::low();
    hbridge::inh::mode(Mode::OUTPUT);
    hbridge::inh::low();

    servo::in::mode(Mode::OUTPUT);
    servo::in::low();

    using namespace misc;
    leds::green::mode(Mode::OUTPUT);
    leds::green::low();
    leds::orange::mode(Mode::OUTPUT);
    leds::orange::low();
    leds::red::mode(Mode::OUTPUT);
    leds::red::low();
    leds::blue::mode(Mode::OUTPUT);
    leds::blue::low();

    xbee::cs::mode(Mode::OUTPUT);
    xbee::cs::high();
    xbee::attn::mode(Mode::INPUT_PULL_UP);
    xbee::reset::mode(Mode::OPEN_DRAIN);
    xbee::reset::high();

    // Led blink
    _led::mode(Mode::OUTPUT);
    ledOn();
    delayMs(100);
    ledOff();

    // Open default serial on USART3 (PB10, PB11)
    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
        new STM32Serial(3, 9600,
        defaultSerialFlowctrl ? STM32Serial::RTSCTS : STM32Serial::NOFLOWCTRL)));
}

void bspInit2()
{
    #ifdef WITH_FILESYSTEM
    // Open GPS serial on USART2 (PA2, PA3)
    intrusive_ref_ptr<DevFs> devFs = basicFilesystemSetup(SDIODriver::instance());
    devFs->addDevice("gps", intrusive_ref_ptr<Device>(new STM32Serial(2, 115200)));
    #endif //WITH_FILESYSTEM
}

//
// Shutdown and reboot
//

/**
 * For safety reasons, we never want the board to shutdown.
 * When requested to shutdown, we reboot instead.
 */
void shutdown()
{
    reboot();
}

void reboot()
{
    ioctl(STDOUT_FILENO,IOCTL_SYNC,0);

    #ifdef WITH_FILESYSTEM
    FilesystemManager::instance().umountAll();
    #endif //WITH_FILESYSTEM

    disableInterrupts();
    miosix_private::IRQsystemReboot();
}

} //namespace miosix
