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

#include "stm32_backup_domain.h"

#include <string.h>

#include "board_settings.h"
#include "miosix.h"

#if defined(_ARCH_CORTEXM3_STM32F2) || defined(_ARCH_CORTEXM4_STM32F4)
#define PWR_CR1 PWR->CR
#define PWR_CR1_DBP PWR_CR_DBP
#define PWR_CSR1 PWR->CSR
#define PWR_CSR1_BRE PWR_CSR_BRE
#define PWR_CSR1_BRR PWR_CSR_BRR
#define RCC_CSR_IWDGRSTF RCC_CSR_WDGRSTF
#define RCC_CSR_PINRSTF RCC_CSR_PADRSTF
#elif defined(_ARCH_CORTEXM7_STM32F7)
#define PWR_CSR1 PWR->CSR1
#define PWR_CR1 PWR->CR1
#endif

namespace miosix
{

BackupDomain &BackupDomain::instance()
{
    static BackupDomain singleton;
    return singleton;
}

void BackupDomain::enable()
{
    // Enable PWR clock
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Enable access to the backup domain
    PWR_CR1 |= PWR_CR1_DBP;
}

void BackupDomain::disable()
{
    // Disable PWR clock
    RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;

    // Disable access to the backup domain
    PWR_CR1 &= ~PWR_CR1_DBP;
}

void BackupDomain::enableBackupSRAM()
{
    // Enable backup SRAM Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;

    // Enable Backup regulator
    PWR_CSR1 |= PWR_CSR1_BRE;
}

void BackupDomain::disableBackupSRAM()
{
    // Disable backup SRAM Clock
    RCC->AHB1ENR &= ~RCC_AHB1ENR_BKPSRAMEN;

    // Disable Backup regulator
    PWR_CSR1 &= ~PWR_CSR1_BRE;
}

BackupDomain::BackupDomain()
{
    // Retrive last reset reason and clear the pending flag
    readResetRegister();
}

void BackupDomain::clearResetFlag() { RCC->CSR |= RCC_CSR_RMVF; }

void BackupDomain::readResetRegister()
{
    uint32_t resetReg = RCC->CSR;

    clearResetFlag();

    if (resetReg & RCC_CSR_LPWRRSTF)
    {
        lastReset = ResetReason::RST_LOW_PWR;
    }
    else if (resetReg & RCC_CSR_WWDGRSTF)
    {
        lastReset = ResetReason::RST_WINDOW_WDG;
    }
    else if (resetReg & RCC_CSR_IWDGRSTF)
    {
        lastReset = ResetReason::RST_INDEPENDENT_WDG;
    }
    else if (resetReg & RCC_CSR_SFTRSTF)
    {
        lastReset = ResetReason::RST_SW;
    }
    else if (resetReg & RCC_CSR_PORRSTF)
    {
        lastReset = ResetReason::RST_POWER_ON;
    }
    else if (resetReg & RCC_CSR_PINRSTF)
    {
        lastReset = ResetReason::RST_PIN;
    }
    else
    {
        lastReset = ResetReason::RST_UNKNOWN;
    }
}

}  // namespace miosix
