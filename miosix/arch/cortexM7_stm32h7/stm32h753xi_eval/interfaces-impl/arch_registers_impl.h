#pragma once

//stm32h7xx.h defines a few macros like __ICACHE_PRESENT, __DCACHE_PRESENT and
//includes core_cm7.h. Do not include core_cm7.h before.
#define STM32H753xx
#include "CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h"

#if (__ICACHE_PRESENT != 1) || (__DCACHE_PRESENT != 1)
#error "Wrong include order"
#endif

#include "CMSIS/Device/ST/STM32H7xx/Include/system_stm32h7xx.h"

//RCC_SYNC is a Miosix-defined primitive for synchronizing the CPU with the RCC
//after modifying peripheral clocks/resets. It should be defined to a no-op on
//architectures without bus access reordering, and to a __DSB() on all other
//ARM architectures. The DMB is required for example in stm32f42x
//microcontrollers. Note that reordering does not necessarily happen at the
//CPU level alone, the bus matrices and peripherals themselves may also reorder
//accesses as a side-effect of how they work.
#define RCC_SYNC() __DSB()
