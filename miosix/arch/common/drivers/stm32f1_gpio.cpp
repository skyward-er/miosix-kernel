
#include "stm32f1_gpio.h"

namespace miosix {

void GpioPin::mode(Mode m)
{
    unsigned char n=getNumber();
    GPIO_TypeDef *d=getPortDevice();
    if(n<8)
    {
        d->CRL &= ~(0xf<<(n*4));
        d->CRL |= toUint(m)<<(n*4);
    } else {
        d->CRH &= ~(0xf<<((n-8)*4));
        d->CRH |= toUint(m)<<((n-8)*4);
    }
}

} //namespace miosix
