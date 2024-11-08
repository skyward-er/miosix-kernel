/***************************************************************************
 *   Copyright (C) 2010-2018 by Terraneo Federico                          *
 *   Copyright (C) 2024 by Daniele Cattaneo                                *
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

#include <cstring>
#include <errno.h>
#include <termios.h>
#include "stm32f4_serial.h"
#include "kernel/sync.h"
#include "kernel/scheduler/scheduler.h"
#include "filesystem/ioctl.h"
#include "cache/cortexMx_cache.h"
#include "interfaces/gpio.h"

using namespace std;
using namespace miosix;

//Work around ST renaming register fields for some STM32L4
#if defined(USART_CR1_RXNEIE_RXFNEIE) && !defined(USART_CR1_RXNEIE)
#define USART_CR1_RXNEIE    USART_CR1_RXNEIE_RXFNEIE
#endif
#if defined(USART_ISR_RXNE_RXFNE) && !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE      USART_ISR_RXNE_RXFNE
#endif
#if defined(USART_ISR_TXE_TXFNF) && !defined(USART_ISR_TXE)
#define USART_ISR_TXE       USART_ISR_TXE_TXFNF
#endif

namespace miosix {

//
// class STM32Serial
//

// A note on the baudrate/500: the buffer is selected so as to withstand
// 20ms of full data rate. In the 8N1 format one char is made of 10 bits.
// So (baudrate/10)*0.02=baudrate/500
STM32Serial::STM32Serial(int id, int baudrate, GpioPin tx, GpioPin rx)
    : Device(Device::TTY), rxQueue(rxQueueMin+baudrate/500),
      flowControl(false), portId(id)
{
    commonInit(id,baudrate,tx,rx,tx,rx); //The last two args will be ignored
}

STM32Serial::STM32Serial(int id, int baudrate, GpioPin tx, GpioPin rx,
    miosix::GpioPin rts, miosix::GpioPin cts)
    : Device(Device::TTY), rxQueue(rxQueueMin+baudrate/500),
      flowControl(true), portId(id)
{
    commonInit(id,baudrate,tx,rx,rts,cts);
}

void STM32Serial::commonInit(int id, int baudrate, GpioPin tx, GpioPin rx,
                             GpioPin rts, GpioPin cts)
{
    InterruptDisableLock dLock;
    unsigned int freq=SystemCoreClock;
    switch(id)
    {
        case 1:
            port=USART1;
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
            RCC_SYNC();
            IRQregisterIrq(USART1_IRQn,&STM32Serial::IRQhandleInterrupt,this);
            NVIC_SetPriority(USART1_IRQn,15);//Lowest priority for serial
            NVIC_EnableIRQ(USART1_IRQn);
            if(RCC->CFGR & RCC_CFGR_PPRE2_2) freq/=1<<(((RCC->CFGR>>RCC_CFGR_PPRE2_Pos) & 0x3)+1);
            break;
        case 2:
            port=USART2;
            RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
            RCC_SYNC();
            IRQregisterIrq(USART2_IRQn,&STM32Serial::IRQhandleInterrupt,this);
            NVIC_SetPriority(USART2_IRQn,15);//Lowest priority for serial
            NVIC_EnableIRQ(USART2_IRQn);
            if(RCC->CFGR & RCC_CFGR_PPRE1_2) freq/=1<<(((RCC->CFGR>>RCC_CFGR_PPRE1_Pos) & 0x3)+1);
            break;
        case 3:
            port=USART3;
            RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
            RCC_SYNC();
            IRQregisterIrq(USART3_IRQn,&STM32Serial::IRQhandleInterrupt,this);
            NVIC_SetPriority(USART3_IRQn,15);//Lowest priority for serial
            NVIC_EnableIRQ(USART3_IRQn);
            if(RCC->CFGR & RCC_CFGR_PPRE1_2) freq/=1<<(((RCC->CFGR>>RCC_CFGR_PPRE1_Pos) & 0x3)+1);
            break;
    }
    const int altFunc = 7;
    tx.mode(Mode::ALTERNATE);
    tx.alternateFunction(altFunc);
    rx.mode(Mode::ALTERNATE);
    rx.alternateFunction(altFunc);
    if(flowControl)
    {
        rts.mode(Mode::ALTERNATE);
        rts.alternateFunction(altFunc);
        cts.mode(Mode::ALTERNATE);
        rts.alternateFunction(altFunc);
    }
    const unsigned int quot=2*freq/baudrate; //2*freq for round to nearest
    port->BRR=quot/2 + (quot & 1);           //Round to nearest
    if(flowControl==false) port->CR3 |= USART_CR3_ONEBIT;
    else port->CR3 |= USART_CR3_ONEBIT | USART_CR3_RTSE | USART_CR3_CTSE;
    //Enabled, 8 data bit, no parity, interrupt on character rx
    port->CR1 = USART_CR1_UE     //Enable port
              | USART_CR1_RXNEIE //Interrupt on data received
              | USART_CR1_IDLEIE //Interrupt on idle line
              | USART_CR1_TE     //Transmission enbled
              | USART_CR1_RE;    //Reception enabled
}

ssize_t STM32Serial::readBlock(void *buffer, size_t size, off_t where)
{
    Lock<FastMutex> l(rxMutex);
    char *buf=reinterpret_cast<char*>(buffer);
    size_t result=0;
    FastInterruptDisableLock dLock;
    DeepSleepLock dpLock;
    for(;;)
    {
        //Try to get data from the queue
        for(;result<size;result++)
        {
            if(rxQueue.tryGet(buf[result])==false) break;
            //This is here just not to keep IRQ disabled for the whole loop
            FastInterruptEnableLock eLock(dLock);
        }
        if(idle && result>0) break;
        if(result==size) break;
        //Wait for data in the queue
        do {
            rxWaiting=Thread::IRQgetCurrentThread();
            Thread::IRQenableIrqAndWait(dLock);
        } while(rxWaiting);
    }
    return result;
}

ssize_t STM32Serial::writeBlock(const void *buffer, size_t size, off_t where)
{
    Lock<FastMutex> l(txMutex);
    DeepSleepLock dpLock;
    const char *buf=reinterpret_cast<const char*>(buffer);
    for(size_t i=0;i<size;i++)
    {
        while((port->SR & USART_SR_TXE)==0) ;
        port->DR=*buf++;
    }
    return size;
}

void STM32Serial::IRQwrite(const char *str)
{
    // We can reach here also with only kernel paused, so make sure
    // interrupts are disabled. This is important for the DMA case
    bool interrupts=areInterruptsEnabled();
    if(interrupts) fastDisableInterrupts();
    while(*str)
    {
        while((port->SR & USART_SR_TXE)==0) ;
        port->DR=*str++;
    }
    waitSerialTxFifoEmpty();
    if(interrupts) fastEnableInterrupts();
}

int STM32Serial::ioctl(int cmd, void* arg)
{
    if(reinterpret_cast<unsigned>(arg) & 0b11) return -EFAULT; //Unaligned
    termios *t=reinterpret_cast<termios*>(arg);
    switch(cmd)
    {
        case IOCTL_SYNC:
            waitSerialTxFifoEmpty();
            return 0;
        case IOCTL_TCGETATTR:
            t->c_iflag=IGNBRK | IGNPAR;
            t->c_oflag=0;
            t->c_cflag=CS8 | (flowControl ? CRTSCTS : 0);
            t->c_lflag=0;
            return 0;
        case IOCTL_TCSETATTR_NOW:
        case IOCTL_TCSETATTR_DRAIN:
        case IOCTL_TCSETATTR_FLUSH:
            //Changing things at runtime unsupported, so do nothing, but don't
            //return error as console_device.h implements some attribute changes
            return 0;
        default:
            return -ENOTTY; //Means the operation does not apply to this descriptor
    }
}

void STM32Serial::IRQhandleInterrupt()
{
    unsigned int status=port->SR;
    char c;
    if(status & USART_SR_RXNE)
    {
        //Always read data, since this clears interrupt flags
        c=port->DR;
        //If no error put data in buffer
        if((status & USART_SR_FE)==0)
            if(rxQueue.tryPut(c)==false) /*fifo overflow*/;
        idle=false;
    }
    if(status & USART_SR_IDLE)
    {
        c=port->DR; //clears interrupt flags
        idle=true;
    }
    if((status & USART_SR_IDLE) || rxQueue.size()>=rxQueueMin)
    {
        //Enough data in buffer or idle line, awake thread
        if(rxWaiting)
        {
            rxWaiting->IRQwakeup();
            if(rxWaiting->IRQgetPriority()>
                Thread::IRQgetCurrentThread()->IRQgetPriority())
                    IRQinvokeScheduler();
            rxWaiting=0;
        }
    }
}

STM32Serial::~STM32Serial()
{
    waitSerialTxFifoEmpty();
    {
        InterruptDisableLock dLock;
        port->CR1=0;
        int id=getId();
        switch(id)
        {
            case 1:
                NVIC_DisableIRQ(USART1_IRQn);
                NVIC_ClearPendingIRQ(USART1_IRQn);
                IRQunregisterIrq(USART1_IRQn);
                RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
                break;
            case 2:
                NVIC_DisableIRQ(USART2_IRQn);
                NVIC_ClearPendingIRQ(USART2_IRQn);
                IRQunregisterIrq(USART2_IRQn);
                RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
                break;
            case 3:
                NVIC_DisableIRQ(USART3_IRQn);
                NVIC_ClearPendingIRQ(USART3_IRQn);
                IRQunregisterIrq(USART3_IRQn);
                RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
                break;
        }
    }
}

} //namespace miosix
