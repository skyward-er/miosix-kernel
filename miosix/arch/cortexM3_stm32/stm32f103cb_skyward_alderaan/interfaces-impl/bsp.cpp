/***************************************************************************
 *   Copyright (C) 2011, 2012, 2013, 2014 by Terraneo Federico             *
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
#include "drivers/dcc.h"
#include "board_settings.h"
#include "hwmapping.h"

namespace miosix {

/**
 * \brief Initialize SPI1
 */
void initSPI1()
{
    using namespace interfaces;

    spi1::cs::mode(Mode::OUTPUT); 
    spi1::cs::high();
    spi1::sck::mode(Mode::ALTERNATE);
    spi1::miso::mode(Mode::ALTERNATE);
    spi1::mosi::mode(Mode::ALTERNATE);

    // APB2 clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI1->CR1=SPI_CR1_SSM  //No HW cs
        | SPI_CR1_SSI
        | SPI_CR1_SPE  //SPI enabled
        | SPI_CR1_BR_2 //SPI clock 24MHz / 32 = 750kHz
        | SPI_CR1_MSTR;//Master mode
}

/**
 * \brief Activate the CAN1 peripheral.
 * NOTE: no initialization is made here: timings and register settings
 * should be taken care of by a dedicated CAN bus driver.
 */
void initCAN1()
{
    using namespace interfaces;
    can1::rx::mode(Mode::ALTERNATE);
    can1::tx::mode(Mode::ALTERNATE);

    // APB1 clock
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
}

/**
 * \brief Initialize hardware timer TIM2
 * TIMING: 
 * TIM frequency = CLOCK_FREQ / ((PSC +1) * (ARR+1))
 * - CLOCK_FREQ = 24MHz     (defined in miosix/config/Makefile.inc)
 * - TIM2->PSC = 0xFFFF
 * - TIM2->ARR = 0x0E4E
 * - hence, TIM2 period is ~10.002sec
 */
void initTIM2()
{
    using namespace interfaces;
    // APB1 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Clear the update event flag
    TIM2->SR   &= ~TIM_SR_UIF;
    // Clear the counter value
    TIM2->CNT  = 0;
    // Prescaler and Reload
    TIM2->PSC  = 0xFFFFU;
    TIM2->ARR  = 0x0E4EU;
}

/**
 * \brief Initialize the independent watchdog
 * TIMING: 
 * IDWG Timeout ~= (4*(2^PR)*RLR) / LSI_freq
 * - LSI_freq   ~= 45kHz  (nominal between 30 and 60, see datasheet)
 * - IWDG->PR    = 5
 * - IWDG->RLR   = 0x7
 * - hence, IWDG period is ~19.911ms (between 14.933ms and 29.866ms)
 */
void initIWDG()
{
    RCC->CSR |= RCC_CSR_LSION; //Enable LSI Clock
    while((RCC->CSR & RCC_CSR_LSIRDY)==0);

    IWDG->KR = 0xCCCC; //Enable IWDG
    IWDG->KR = 0x5555; //Enable register access
    IWDG->PR |= IWDG_PR_PR_0
               | IWDG_PR_PR_2; //Set prescaler to 5
    IWDG->RLR = 0x7; //Set reload value

    while(IWDG->SR); //Check if flags are reset

    IWDG->KR = 0xAAAA; //Refresh the counter
}

//
// Initialization
//
void IRQbspInit()
{
    //Enable all gpios
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN |
                    RCC_APB2ENR_AFIOEN;
    
    // Initialize pins
    // Abort pin logic : 0 -> ABORT (burn fuse)
    //                   1 -> NOT ABORT
    using namespace actuators;
    abortPin::mode(Mode::OUTPUT);
    abortPin::high();

    // Ignition pin logic : 0 -> NO IGNITION
    //                      1 -> IGNITION 
    ignitionPin::mode(Mode::OUTPUT);
    ignitionPin::low();

    // Spare Pin initialization
    // Spare pin logic : 0 -> AVR BUSY
    //                   1 -> AVR READY
    using namespace interfaces;
    sparePin::mode(Mode::INPUT);

    // Initialize peripherals
    initIWDG();
    initSPI1();
    initCAN1();
    initTIM2();

    // UART1 initialization
    uart1::tx::mode(Mode::OUTPUT);
    uart1::rx::mode(Mode::INPUT);

    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
    #ifndef STDOUT_REDIRECTED_TO_DCC
        new STM32Serial(defaultSerial,defaultSerialSpeed,
        defaultSerialFlowctrl ? STM32Serial::RTSCTS : STM32Serial::NOFLOWCTRL)));
    #else //STDOUT_REDIRECTED_TO_DCC
        new ARMDCC();
    #endif //STDOUT_REDIRECTED_TO_DCC
}

void bspInit2()
{
//     #ifdef WITH_FILESYSTEM
//     basicFilesystemSetup();
//     #endif //WITH_FILESYSTEM
}

//
// Shutdown and reboot
//

/**
This function disables filesystem (if enabled), serial port (if enabled) and
puts the processor in deep sleep mode.<br>
Wakeup occurs when PA.0 goes high, but instead of sleep(), a new boot happens.
<br>This function does not return.<br>
WARNING: close all files before using this function, since it unmounts the
filesystem.<br>
When in shutdown mode, power consumption of the miosix board is reduced to ~
5uA??, however, true power consumption depends on what is connected to the GPIO
pins. The user is responsible to put the devices connected to the GPIO pin in the
minimal power consumption mode before calling shutdown(). Please note that to
minimize power consumption all unused GPIO must not be left floating.
*/
void shutdown()
{
    ioctl(STDOUT_FILENO,IOCTL_SYNC,0);

    #ifdef WITH_FILESYSTEM
    FilesystemManager::instance().umountAll();
    #endif //WITH_FILESYSTEM

    disableInterrupts();

    /*
    Removed because low power mode causes issues with SWD programming
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; //Fuckin' clock gating...  
    RCC_SYNC();
    PWR->CR |= PWR_CR_PDDS; //Select standby mode
    PWR->CR |= PWR_CR_CWUF;
    PWR->CSR |= PWR_CSR_EWUP; //Enable PA.0 as wakeup source
    
    SCB->SCR |= SCB_SCR_SLEEPDEEP;
    __WFE();
    NVIC_SystemReset();
    */
    for(;;) ;
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
