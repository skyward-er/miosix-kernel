#pragma once

//Always include stm32f3xx.h before core_cm4.h, there's some nasty dependency
#define STM32F303xC
#include "CMSIS/Device/ST/STM32F3xx/Include/stm32f3xx.h"
#include "CMSIS/Include/core_cm4.h"
#include "CMSIS/Device/ST/STM32F3xx/Include/system_stm32f3xx.h"

//RCC_SYNC is a Miosix-defined primitive for synchronizing the CPU with the RCC
//after modifying peripheral clocks/resets. It should be defined to a no-op on
//architectures without bus access reordering, and to a __DSB() on all other
//ARM architectures. The DMB is required for example in stm32f42x
//microcontrollers. Note that reordering does not necessarily happen at the
//CPU level alone, the bus matrices and peripherals themselves may also reorder
//accesses as a side-effect of how they work.
#define RCC_SYNC()

//Peripheral interrupt start from 0 and the last one is 81, so there are 82
#define MIOSIX_NUM_PERIPHERAL_IRQ 82
