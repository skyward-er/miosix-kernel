#ifndef HWMAPPING_H
#define	HWMAPPING_H

#include "interfaces/arch_registers.h"
#include "interfaces/gpio.h"

namespace miosix {

namespace interfaces {

namespace spi1 {
using sck = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}

namespace usart3 {
using tx = Gpio<GPIOB_BASE, 10>;
using rx = Gpio<GPIOB_BASE, 11>;
}

namespace can {
using tx = Gpio<GPIOA_BASE, 12>;
using rx = Gpio<GPIOA_BASE, 11>;
}

}

namespace leds
{
    using led1 = Gpio<GPIOB_BASE, 12>;
    using led2 = Gpio<GPIOB_BASE, 13>;
    using led3 = Gpio<GPIOB_BASE, 14>;
}

namespace sensors {

namespace ads1118 {
using cs = Gpio<GPIOA_BASE, 4>;
}

}

}

#endif