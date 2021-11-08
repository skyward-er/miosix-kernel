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

namespace miosix {

namespace interfaces {
    namespace spi1 {
    using sck   = Gpio<GPIOA_BASE, 5>;
    using miso  = Gpio<GPIOA_BASE, 6>;
    using mosi  = Gpio<GPIOA_BASE, 7>;
    }

    namespace spi2 {
    using sck   = Gpio<GPIOB_BASE, 13>;
    using miso  = Gpio<GPIOB_BASE, 14>;
    using mosi  = Gpio<GPIOB_BASE, 15>;
    }
}

namespace sensors {
    // External ADC
    namespace ads1118 {
    using cd    = Gpio<GPIOE_BASE, 6>;
    }

    // Digital Pressure
    namespace ms5803 {
    using cs    = Gpio<GPIOE_BASE, 13>;
    }

    // IMU magneto
    namespace lsm9ds1 {
    using cs_ag     = Gpio<GPIOE_BASE, 7>;  // chip select acc
    using cs_mag    = Gpio<GPIOE_BASE, 9>;  // chip select magn
    using int_ag    = Gpio<GPIOC_BASE, 13>; // interrupt accelerometro
    using drdy_mag  = Gpio<GPIOC_BASE, 15>; // data ready magnetometro
    }

    // Dpl mecahnism sensors
    using finecorsa = Gpio<GPIOC_BASE, 1>;
    using slitta    = Gpio<GPIOC_BASE, 3>;

    // Detach pins
    using nosecone_dtch  = Gpio<GPIOE_BASE, 10>;
    using launchpad_dtch = Gpio<GPIOE_BASE, 12>;
}

namespace actuators {
    // Backup expulsion system
    namespace hbridge {
    using is    = Gpio<GPIOC_BASE, 0>; // current sensor
    using in    = Gpio<GPIOB_BASE, 5>; // input
    using inh   = Gpio<GPIOB_BASE, 7>; // inhibit
    }

    // Servomotor for primary expulsion system
    namespace servo {
    using in = Gpio<GPIOC_BASE, 6>;
    }
}

namespace misc {
    //using button = Gpio<GPIOA_BASE, 0>;
    using aux    = Gpio<GPIOC_BASE, 5>;

    namespace leds {
    using green  = Gpio<GPIOD_BASE, 12>;
    using orange = Gpio<GPIOD_BASE, 13>;
    using red    = Gpio<GPIOD_BASE, 14>;
    using blue   = Gpio<GPIOD_BASE, 15>;
    }
}

namespace xbee {
    using attn   = Gpio<GPIOE_BASE, 2>;
    using cs     = Gpio<GPIOE_BASE, 4>;
    using reset  = Gpio<GPIOE_BASE, 5>;
}

} //namespace miosix

#endif //HWMAPPING_H
