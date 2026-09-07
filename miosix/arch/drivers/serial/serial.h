//Serial code is common for all the cortex M cores, so it has been put here

#ifdef _ARCH_ARM7_LPC2000
#include "lpc2000_serial.h"
#elif defined(_ARCH_CORTEXM3_STM32L1) || defined(_ARCH_CORTEXM3_STM32F1) \
   || defined(_ARCH_CORTEXM4_STM32F4) || defined(_ARCH_CORTEXM3_STM32F2) 
#include "stm32_serial_common.h" 
#include "stm32_f1_f2_f4_serial.h"
#elif defined(_ARCH_CORTEXM7_STM32H7) || defined(_ARCH_CORTEXM4_STM32F3) \
   || defined(_ARCH_CORTEXM4_STM32L4) || defined(_ARCH_CORTEXM0_STM32F0) \
   || defined(_ARCH_CORTEXM7_STM32F7) 
#include "stm32_serial_common.h" 
#include "stm32f7_serial.h"
#elif defined(_ARCH_CORTEXM3_EFM32GG)
#include "efm32_serial.h"
#elif defined(_ARCH_CORTEXM4_ATSAM4L)
#include "atsam4l_serial.h"
#else
#error "Unknown arch"
#endif

