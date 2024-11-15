/***************************************************************************
 *   Copyright (C) 2012-2021 by Terraneo Federico                          *
 *   Copyright (C) 2024 by Daniele Cattaneo                                *
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

/**
 * \internal
 * Versioning for board_settings.h for out of git tree projects
 */
#define BOARD_SETTINGS_VERSION 300

namespace miosix {

/**
 * \addtogroup Settings
 * \{
 */

/// Size of stack for main().
/// The C standard library is stack-heavy (iprintf requires 1KB) but the
/// STM32F407VG only has 192KB of RAM so there is room for a big 4K stack.
const unsigned int MAIN_STACK_SIZE=4*1024;

/// Serial port
/// Serial ports 1 to 6 are available
const unsigned int defaultSerial=3;
const unsigned int defaultSerialSpeed=19200;
const bool defaultSerialFlowctrl=false;
const bool defaultSerialDma=true;
// Default serial 1 pins (uncomment when using serial 1)
// Note: on this board, pins PA9-12 are in use by the user USB port, and PB6 is
// connected to the Cirrus audio chip
//using defaultSerialTxPin = Gpio<GPIOB_BASE,6>;
//using defaultSerialRxPin = Gpio<GPIOB_BASE,7>;
//using defaultSerialRtsPin = Gpio<GPIOA_BASE,12>;
//using defaultSerialCtsPin = Gpio<GPIOA_BASE,11>;
// Default serial 2 pins (uncomment when using serial 2)
//using defaultSerialTxPin = Gpio<GPIOA_BASE,2>;
//using defaultSerialRxPin = Gpio<GPIOA_BASE,3>;
//using defaultSerialRtsPin = Gpio<GPIOA_BASE,1>;
//using defaultSerialCtsPin = Gpio<GPIOA_BASE,0>;
// Default serial 3 pins (uncomment when using serial 3)
using defaultSerialTxPin = Gpio<GPIOB_BASE,10>;
using defaultSerialRxPin = Gpio<GPIOB_BASE,11>;
using defaultSerialRtsPin = Gpio<GPIOB_BASE,14>;
using defaultSerialCtsPin = Gpio<GPIOB_BASE,13>;

// Aux serial port
// Uncomment AUX_SERIAL to enable. The device will appear as /dev/auxtty.
//#define AUX_SERIAL "auxtty"
const unsigned int auxSerial=2;
const unsigned int auxSerialSpeed=9600;
const bool auxSerialFlowctrl=false;
//Disable DMA for serial 2 because it conflicts with I2S driver in examples
const bool auxSerialDma=false;
// Default aux serial 1 pins (uncomment when using serial 1)
// Note: on this board, pins PA9-12 are in use by the user USB port, and PB6 is
// connected to the Cirrus audio chip
//using auxSerialTxPin = Gpio<GPIOB_BASE,6>;
//using auxSerialRxPin = Gpio<GPIOB_BASE,7>;
//using auxSerialRtsPin = Gpio<GPIOA_BASE,12>;
//using auxSerialCtsPin = Gpio<GPIOA_BASE,11>;
// Default aux serial 2 pins (uncomment when using serial 2)
using auxSerialTxPin = Gpio<GPIOA_BASE,2>;
using auxSerialRxPin = Gpio<GPIOA_BASE,3>;
using auxSerialRtsPin = Gpio<GPIOA_BASE,1>;
using auxSerialCtsPin = Gpio<GPIOA_BASE,0>;
// Default aux serial 3 pins (uncomment when using serial 3)
//using auxSerialTxPin = Gpio<GPIOB_BASE,10>;
//using auxSerialRxPin = Gpio<GPIOB_BASE,11>;
//using auxSerialRtsPin = Gpio<GPIOB_BASE,14>;
//using auxSerialCtsPin = Gpio<GPIOB_BASE,13>;

//SD card driver
static const unsigned char sdVoltage=30; //Board powered @ 3.0V
#define SD_ONE_BIT_DATABUS //Can't use 4 bit databus due to pin conflicts

/**
 * \}
 */

} //namespace miosix
