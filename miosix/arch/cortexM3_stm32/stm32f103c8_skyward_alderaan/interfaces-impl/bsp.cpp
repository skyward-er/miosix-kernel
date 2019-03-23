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
 * Initilize SPI
 */
void initSPI1()
{
    using namespace interfaces;
    // SPI1 intialization
    spi1::cs::mode(Mode::OUTPUT); 
    spi1::cs::high();
    spi1::sck::mode(Mode::ALTERNATE);
    spi1::miso::mode(Mode::ALTERNATE);
    spi1::mosi::mode(Mode::ALTERNATE);

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC_SYNC();

    SPI1->CR1=SPI_CR1_SSM  //No HW cs
        | SPI_CR1_SSI
        | SPI_CR1_SPE  //SPI enabled
        | SPI_CR1_BR_2 //SPI clock 24MHz / 32 = 750kHz
        | SPI_CR1_MSTR;//Master mode
}

/**
 * Initialize Canbus
 */
void initCAN1()
{
    using namespace interfaces;
    // CAN1 initialization
    can1::rx::mode(Mode::ALTERNATE);
    can1::tx::mode(Mode::ALTERNATE);

    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    NVIC_SetPriority(CAN1_RX1_IRQn, 1);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);
}

/**
 * Initialize hardware timer
 */
void initTIM2()
{
    using namespace interfaces;
    // TIM2 initialization
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Clear the update event flag */
    TIM2->SR   = 0;
    /* Clear the counter value */
    TIM2->CNT  = 0;
    /* Prescaler and Reload set to maximum = overflow every 59.6523235555 sec*/
    TIM2->PSC  = 0xFFFF;
    TIM2->ARR  = 0xFFFF;  
    /* Enable Counter */
    TIM2->CR1  |= TIM_CR1_CEN;  

    /* Configure Interupt */
    TIM2->DIER |= TIM_DIER_UIE; 
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);
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
    
    // Actuator Pins Initialization
    // Abort pin logic : 0 -> ABORT (burn fuse)
    //                   1 -> NOT ABORT
    // Ignition pin logic : 0 -> NO IGNITION
    //                      1 -> IGNITION 
    using namespace actuators;
    abortPin::mode(Mode::OUTPUT);
    abortPin::high();

    ignitionPin::mode(Mode::OUTPUT);
    ignitionPin::low();

    using namespace interfaces;
    // Spare Pin initialization
    // Spare pin logic : 0 -> AVR BUSY
    //                   1 -> AVR READY
    sparePin::mode(Mode::INPUT);
    sparePin::high();

    initSPI1();
    initCAN1();
    initTIM2();

    //#ifdef DEBUG
        // UART1 initialization
        uart1::tx::mode(Mode::OUTPUT);
        uart1::rx::mode(Mode::INPUT);

        _led::mode(Mode::OUTPUT_2MHz);
        ledOn();
        delayMs(100);
        ledOff();

        DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
        #ifndef STDOUT_REDIRECTED_TO_DCC
            new STM32Serial(defaultSerial,defaultSerialSpeed,
            defaultSerialFlowctrl ? STM32Serial::RTSCTS : STM32Serial::NOFLOWCTRL)));
        #else //STDOUT_REDIRECTED_TO_DCC
            new ARMDCC();
        #endif //STDOUT_REDIRECTED_TO_DCC
    //#endif
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
