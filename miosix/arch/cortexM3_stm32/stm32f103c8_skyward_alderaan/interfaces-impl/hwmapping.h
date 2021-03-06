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
#define	HWMAPPING_H

#include "interfaces/gpio.h"

namespace miosix {

namespace interfaces 
{

	namespace spi1 {
	using sck   = Gpio<GPIOA_BASE, 5>;
	using miso  = Gpio<GPIOA_BASE, 6>;
	using mosi  = Gpio<GPIOA_BASE, 7>;
	} //namespace spi1

	namespace spi2 {
	using sck   = Gpio<GPIOB_BASE, 13>;
	using miso  = Gpio<GPIOB_BASE, 14>;
	using mosi  = Gpio<GPIOB_BASE, 15>;
	} //namespace spi2 [non usato]

	namespace i2c {
	using scl   = Gpio<GPIOB_BASE, 8>;
	using sda   = Gpio<GPIOB_BASE, 9>;
	} //namespace i2c [non usato]

	namespace uart1 {
	using tx    = Gpio<GPIOA_BASE, 9>;
	using rx    = Gpio<GPIOA_BASE, 10>;
	} //namespace uart1

	namespace can {
	using rx    = Gpio<GPIOB_BASE, 8>;
	using tx    = Gpio<GPIOB_BASE, 9>;
	} // namespace can
} //namespace interfaces


namespace actuators 
{

	using abort1   	= Gpio<GPIOA_BASE, 1>;
	using ignition1 = Gpio<GPIOA_BASE, 2>;
	using spare 	= Gpio<GPIOA_BASE, 3>;

} //namespace actuators

} //namespace miosix
#endif //HWMAPPING_H
