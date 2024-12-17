#pragma once

//stm32f7xx.h defines a few macros like __ICACHE_PRESENT, __DCACHE_PRESENT and
//includes core_cm7.h. Do not include core_cm7.h before.
#define STM32F746xx
#include "CMSIS/Device/ST/STM32F7xx/Include/stm32f7xx.h"

#if (__ICACHE_PRESENT != 1) || (__DCACHE_PRESENT != 1)
#error "Wrong include order"
#endif

#include "CMSIS/Device/ST/STM32F7xx/Include/system_stm32f7xx.h"

//RCC_SYNC is a Miosix-defined primitive for synchronizing the CPU with the RCC
//after modifying peripheral clocks/resets. It should be defined to a no-op on
//architectures without bus access reordering, and to a __DSB() on all other
//ARM architectures. The DMB is required for example in stm32f42x
//microcontrollers. Note that reordering does not necessarily happen at the
//CPU level alone, the bus matrices and peripherals themselves may also reorder
//accesses as a side-effect of how they work.
#define RCC_SYNC() __DSB() //TODO: can this dsb be removed?

//Peripheral interrupt start from 0 and the last one is 97, so there are 98
#define MIOSIX_NUM_PERIPHERAL_IRQ 98
