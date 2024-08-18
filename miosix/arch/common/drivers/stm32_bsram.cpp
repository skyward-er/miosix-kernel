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

#include "stm32_bsram.h"

#include <miosix.h>

#include <cstring>

namespace miosix
{

static ResetReason lastReset = ResetReason::UNKNOWN;

static ResetReason readResetRegister()
{
    uint32_t resetReg = RCC->CSR;

    // Clear the reset flag
    RCC->CSR |= RCC_CSR_RMVF;

    if (resetReg & RCC_CSR_LPWRRSTF)
    {
        return ResetReason::LOW_POWER;
    }
    else if (resetReg & RCC_CSR_WWDGRSTF)
    {
        return ResetReason::WINDOW_WATCHDOG;
    }
#ifdef _ARCH_CORTEXM7_STM32F7
    else if (resetReg & RCC_CSR_IWDGRSTF)
#else
    else if (resetReg & RCC_CSR_WDGRSTF)
#endif
    {
        return ResetReason::INDEPENDENT_WATCHDOG;
    }
    else if (resetReg & RCC_CSR_SFTRSTF)
    {
        return ResetReason::SOFTWARE;
    }
    else if (resetReg & RCC_CSR_PORRSTF)
    {
        return ResetReason::POWER_ON;
    }
#ifdef _ARCH_CORTEXM7_STM32F7
    else if (resetReg & RCC_CSR_PINRSTF)
#else
    else if (resetReg & RCC_CSR_PADRSTF)
#endif
    {
        return ResetReason::PIN;
    }
    else
    {
        return ResetReason::UNKNOWN;
    }
}

ResetReason lastResetReason() { return lastReset; }

namespace BSRAM
{

void init()
{
    // Enable PWR clock
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Enable write on BSRAM since we need it to write on the PWR peripheral
    enableWrite();

    // Enable backup SRAM Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;

    // Enable the backup regulator and wait for it to be ready
#ifdef _ARCH_CORTEXM7_STM32F7
    PWR->CSR1 |= PWR_CSR1_BRE;
    while (!(PWR->CSR1 & (PWR_CSR1_BRR)))
        ;
#else
    PWR->CSR |= PWR_CSR_BRE;
    while (!(PWR->CSR & (PWR_CSR_BRR)))
        ;
#endif

    // Retrieve last reset reason
    lastReset = readResetRegister();

    // Disable write on BSRAM since we don't need to write on the peripheral for
    // now. It will be necessary to re-enable the write on the backup SRAM
    // around a PRESERVED variable to effectively change its value.
    disableWrite();
}

void disableWrite()
{
    // Ensure all memory instructions complete before disabling write
    __DSB();

    // Enable backup domain write protection
#ifdef _ARCH_CORTEXM7_STM32F7
    PWR->CR1 &= ~PWR_CR1_DBP;
#else
    PWR->CR &= ~PWR_CR_DBP;
#endif

    // Ensure write is disabled when exiting this function
    __DSB();
}

void enableWrite()
{
    // Disable backup domain write protection
#ifdef _ARCH_CORTEXM7_STM32F7
    PWR->CR1 |= PWR_CR1_DBP;
#else
    PWR->CR |= PWR_CR_DBP;
#endif

    // Ensure writes to the control registers complete before returning
    __DSB();
}

}  // namespace BSRAM
}  // namespace miosix
