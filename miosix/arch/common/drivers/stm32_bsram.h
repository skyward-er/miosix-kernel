/***************************************************************************
 *   Copyright (C) 2024 by Niccolò Betto                                   *
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

namespace miosix
{

/**
 * Enumeration of possible causes of an STM32 reset.
 */
enum class ResetReason
{
    UNKNOWN = 0,
    LOW_POWER,
    WINDOW_WATCHDOG,
    INDEPENDENT_WATCHDOG,
    SOFTWARE,
    POWER_ON,
    PIN,
};

/**
 * Returns the cause of the last reset of the microcontroller.
 */
ResetReason lastResetReason();

/**
 * Driver for the STM32F2/F4/F7 backup SRAM.
 */
namespace BSRAM
{

/**
 * Initialize the backup SRAM and populate the last reset reason variable.
 */
void init();

/**
 * Disable writing to the backup SRAM.
 */
void disableWrite();

/**
 * Enable writing to the backup SRAM.
 */
void enableWrite();

/**
 * @brief This class is a RAII lock for disabling write protection on backup
 * SRAM. This call avoids the error of not re-enabling write protection since it
 * is done automatically.
 */
class EnableWriteLock
{
public:
    EnableWriteLock() { enableWrite(); }

    ~EnableWriteLock() { disableWrite(); }

    ///< Delete copy/move constructors/operators.
    EnableWriteLock(const EnableWriteLock& l)             = delete;
    EnableWriteLock(const EnableWriteLock&& l)            = delete;
    EnableWriteLock& operator=(const EnableWriteLock& l)  = delete;
    EnableWriteLock& operator=(const EnableWriteLock&& l) = delete;
};

}  // namespace BSRAM
}  // namespace miosix
