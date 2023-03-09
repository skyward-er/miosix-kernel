/***************************************************************************
 *   Copyright (C) 2017 by Matteo Michele Piazzolla                        *
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

#include "board_settings.h"

#define PRESERVE __attribute__((section(".preserve")))

namespace miosix
{

/**
 * Possible causes for an STM32 reset
 */
enum class ResetReason
{
    RST_LOW_PWR         = 0,  // Low power
    RST_WINDOW_WDG      = 1,  // Reset from the windows watchdog
    RST_INDEPENDENT_WDG = 2,  // Reset from the independent watchdog
    RST_SW              = 3,  // Sofware reset
    RST_POWER_ON        = 4,  // System power on
    RST_PIN             = 5,  // Reset pin
    RST_UNKNOWN         = 6,  // Unknown
};

/**
 * Driver for the backup SRAM.
 *
 * @warning Tested only on stm32f2, stm32f4 and stm32f7 microcontrollers.
 */
class BackupDomain
{
public:
    /**
     * @return An instance of this class (singleton).
     */
    static BackupDomain& instance();

    /**
     * Enables the backup domain clock and write access.
     */
    void enable();

    /**
     * Disable the backup domain clock and write access.
     */
    void disable();

    /**
     * Enable the backup SRAM.
     */
    void enableBackupSRAM();

    /**
     * Disable the backup SRAM.
     */
    void disableBackupSRAM();

    /**
     * Return the cause of the last reset of the microcontroller
     */
    ResetReason lastResetReason() { return lastReset; }

private:
    ResetReason lastReset;

    BackupDomain(const BackupDomain&)            = delete;
    BackupDomain& operator=(const BackupDomain&) = delete;

    BackupDomain();
    void readResetRegister();
    void clearResetFlag();
};

}  // namespace miosix
