
#ifndef ARCH_REGISTERS_IMPL_H
#define	ARCH_REGISTERS_IMPL_H

//Always include stm32f4xx.h before core_cm4.h, there's some nasty dependency
#define STM32F407xx
#include "CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
#include "CMSIS/Include/core_cm4.h"
#include "CMSIS/Device/ST/STM32F4xx/Include/system_stm32f4xx.h"

#define RCC_SYNC() //Workaround for a bug in stm32f42x

// Peripheral interrupt start from 0 and the last one is 81, so there are 82
#define MIOSIX_NUM_PERIPHERAL_IRQ 82

#endif	//ARCH_REGISTERS_IMPL_H
