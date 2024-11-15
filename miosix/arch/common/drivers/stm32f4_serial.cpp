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

/*
 * Auxiliary class that encapsulates all parts of code that differ between
 * between each instance of this peripheral.
 * 
 * Try not to use the attributes of this class directly even if they are public,
 * in the current implementation they are not the best use of ROM space and
 * by using getter/setters we make it easier for ourselves to compress this
 * data structure a bit in the future.
 * TODO: Remove redundant information in the attributes
 */
class STM32SerialHW
{
public:
    enum Bus { APB1, APB2, AHB1, AHB2 };
    enum DmaIntRegShift
    {
        Stream0= 0, Stream1= 0+6, Stream2=16, Stream3=16+6,
        Stream4=32, Stream5=32+6, Stream6=48, Stream7=48+6
    };
    
    inline USART_TypeDef *get() const { return port; }
    inline IRQn_Type getIRQn() const { return irq; }
    inline unsigned char getAltFunc() const { return altFunc; }
    inline unsigned int IRQgetClock() const
    {
        unsigned int freq=SystemCoreClock;
        switch(bus)
        {
            case APB1:
                if(RCC->CFGR & RCC_CFGR_PPRE1_2)
                    freq/=1<<(((RCC->CFGR>>RCC_CFGR_PPRE1_Pos) & 0x3)+1);
                break;
            case APB2:
                if(RCC->CFGR & RCC_CFGR_PPRE2_2)
                    freq/=1<<(((RCC->CFGR>>RCC_CFGR_PPRE2_Pos) & 0x3)+1);
                break;
            default:
                break;
        }
        return freq;
    }
    inline void IRQenable() const { IRQgenericEn(bus, clkEnMask); }
    inline void IRQdisable() const { IRQgenericDis(bus, clkEnMask); }

    inline DMA_TypeDef *getDma() const { return dma; }
    inline void IRQdmaEnable() const { IRQgenericEn(dmaBus, dmaClkEnMask); }
    inline void IRQdmaDisable() const { IRQgenericDis(dmaBus, dmaClkEnMask); }

    inline DMA_Stream_TypeDef *getDmaTx() const { return dmaTx; }
    inline IRQn_Type getDmaTxIRQn() const { return dmaTxIrq; }
    inline unsigned long getDmaTxChannel() const { return dmaTxChannel; }
    inline unsigned long getDmaTxISR() const { return getDmaISR(dmaTxIRShift); }
    inline void setDmaTxIFCR(unsigned long v) const { return setDmaIFCR(dmaTxIRShift, v); }

    inline DMA_Stream_TypeDef *getDmaRx() const { return dmaRx; }
    inline IRQn_Type getDmaRxIRQn() const { return dmaRxIrq; }
    inline unsigned long getDmaRxChannel() const { return dmaRxChannel; }
    inline unsigned long getDmaRxISR() const { return getDmaISR(dmaRxIRShift); }
    inline void setDmaRxIFCR(unsigned long v) const { return setDmaIFCR(dmaRxIRShift, v); }

    USART_TypeDef *port;        ///< USART port
    IRQn_Type irq;              ///< USART IRQ number
    unsigned char altFunc;      ///< Alternate function to set for GPIOs
    STM32SerialHW::Bus bus;     ///< Bus where the port is (APB1 or 2)
    unsigned long clkEnMask;    ///< USART clock enable

    DMA_TypeDef *dma;           ///< Pointer to the DMA peripheral (DMA1/2)
    STM32SerialHW::Bus dmaBus;  ///< Bus where the DMA port is (AHB1 or 2)
    unsigned long dmaClkEnMask; ///< DMA clock enable bit

    DMA_Stream_TypeDef *dmaTx;  ///< Pointer to DMA TX stream
    IRQn_Type dmaTxIrq;         ///< DMA TX stream IRQ number
    unsigned char dmaTxIRShift; ///< Value from DMAIntRegShift for the stream
    unsigned char dmaTxChannel; ///< DMA TX stream channel

    DMA_Stream_TypeDef *dmaRx;  ///< Pointer to DMA RX stream
    IRQn_Type dmaRxIrq;         ///< DMA RX stream IRQ number
    unsigned char dmaRxIRShift; ///< Value from DMAIntRegShift for the stream
    unsigned char dmaRxChannel; ///< DMA TX stream channel

private:
    static inline void IRQgenericEn(STM32SerialHW::Bus bus, unsigned long mask)
    {
        switch(bus)
        {
            case APB1: RCC->APB1ENR|=mask; break;
            case APB2: RCC->APB2ENR|=mask; break;
            case AHB1: RCC->AHB1ENR|=mask; break;
            case AHB2: RCC->AHB2ENR|=mask; break;
        }
        RCC_SYNC();
    }
    static inline void IRQgenericDis(STM32SerialHW::Bus bus, unsigned long mask)
    {
        switch(bus)
        {
            case APB1: RCC->APB1ENR&=~mask; break;
            case APB2: RCC->APB2ENR&=~mask; break;
            case AHB1: RCC->AHB1ENR&=~mask; break;
            case AHB2: RCC->AHB2ENR&=~mask; break;
        }
        RCC_SYNC();
    }
    inline unsigned long getDmaISR(unsigned char pos) const
    {
        if(pos<32) return (dma->LISR>>pos) & 0b111111;
        return (dma->HISR>>(pos-32)) & 0b111111;
    }
    inline void setDmaIFCR(unsigned char pos, unsigned long value) const
    {
        value=(value&0b111111) << (pos%32);
        if(pos<32) dma->LIFCR=value;
        else dma->HIFCR=value;
    }
};

constexpr int numPorts = 6;
static const STM32SerialHW ports[numPorts] = {
    { USART1, USART1_IRQn, 7, STM32SerialHW::APB2, RCC_APB2ENR_USART1EN,
      DMA2, STM32SerialHW::AHB1, RCC_AHB1ENR_DMA2EN,
      DMA2_Stream7, DMA2_Stream7_IRQn, STM32SerialHW::Stream7, 4,
      DMA2_Stream5, DMA2_Stream5_IRQn, STM32SerialHW::Stream5, 4 },
    { USART2, USART2_IRQn, 7, STM32SerialHW::APB1, RCC_APB1ENR_USART2EN,
      DMA1, STM32SerialHW::AHB1, RCC_AHB1ENR_DMA1EN,
      DMA1_Stream6, DMA1_Stream6_IRQn, STM32SerialHW::Stream6, 4,
      DMA1_Stream5, DMA1_Stream5_IRQn, STM32SerialHW::Stream5, 4 },
    { USART3, USART3_IRQn, 7, STM32SerialHW::APB1, RCC_APB1ENR_USART3EN,
      DMA1, STM32SerialHW::AHB1, RCC_AHB1ENR_DMA1EN,
      DMA1_Stream3, DMA1_Stream3_IRQn, STM32SerialHW::Stream3, 4,
      DMA1_Stream1, DMA1_Stream1_IRQn, STM32SerialHW::Stream1, 4 },
    { UART4 , UART4_IRQn , 8, STM32SerialHW::APB1, RCC_APB1ENR_UART4EN,
      DMA1, STM32SerialHW::AHB1, RCC_AHB1ENR_DMA1EN,
      DMA1_Stream4, DMA1_Stream4_IRQn, STM32SerialHW::Stream4, 4,
      DMA1_Stream2, DMA1_Stream2_IRQn, STM32SerialHW::Stream2, 4 },
    { UART5 , UART5_IRQn , 8, STM32SerialHW::APB1, RCC_APB1ENR_UART5EN,
      DMA1, STM32SerialHW::AHB1, RCC_AHB1ENR_DMA1EN,
      DMA1_Stream7, DMA1_Stream7_IRQn, STM32SerialHW::Stream7, 4,
      DMA1_Stream0, DMA1_Stream0_IRQn, STM32SerialHW::Stream0, 4 },
    { USART6, USART6_IRQn, 8, STM32SerialHW::APB2, RCC_APB2ENR_USART6EN,
      DMA2, STM32SerialHW::AHB1, RCC_AHB1ENR_DMA2EN,
      DMA2_Stream6, DMA2_Stream6_IRQn, STM32SerialHW::Stream6, 5,
      DMA2_Stream1, DMA2_Stream1_IRQn, STM32SerialHW::Stream1, 5 },
};

//
// class STM32SerialBase
//

// A note on the baudrate/500: the buffer is selected so as to withstand
// 20ms of full data rate. In the 8N1 format one char is made of 10 bits.
// So (baudrate/10)*0.02=baudrate/500
STM32SerialBase::STM32SerialBase(int id, int baudrate, bool flowControl) : 
    flowControl(flowControl), portId(id), rxQueue(rxQueueMin+baudrate/500)
{
    if(id<1 || id>numPorts) errorHandler(UNEXPECTED);
    port=&ports[id-1];
}

void STM32SerialBase::commonInit(int id, int baudrate, GpioPin tx, GpioPin rx,
                    GpioPin rts, GpioPin cts)
{
    tx.mode(Mode::ALTERNATE);
    tx.alternateFunction(port->getAltFunc());
    rx.mode(Mode::ALTERNATE);
    rx.alternateFunction(port->getAltFunc());
    if(flowControl)
    {
        rts.mode(Mode::ALTERNATE);
        rts.alternateFunction(port->getAltFunc());
        cts.mode(Mode::ALTERNATE);
        rts.alternateFunction(port->getAltFunc());
    }
    unsigned int freq=port->IRQgetClock();
    unsigned int quot=2*freq/baudrate; //2*freq for round to nearest
    port->get()->BRR=quot/2 + (quot & 1);           //Round to nearest
    if(flowControl==false) port->get()->CR3 |= USART_CR3_ONEBIT;
    else port->get()->CR3 |= USART_CR3_ONEBIT | USART_CR3_RTSE | USART_CR3_CTSE;
}

void STM32SerialBase::IRQwrite(const char *str)
{
    while(*str)
    {
        while((port->get()->SR & USART_SR_TXE)==0) ;
        port->get()->DR=*str++;
    }
    waitSerialTxFifoEmpty();
}

inline void STM32SerialBase::waitSerialTxFifoEmpty()
{
    while((port->get()->SR & USART_SR_TC)==0) ;
}

ssize_t STM32SerialBase::readFromRxQueue(void *buffer, size_t size)
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

void STM32SerialBase::rxWakeup()
{
    if(rxWaiting)
    {
        rxWaiting->IRQwakeup();
        if(rxWaiting->IRQgetPriority()>
            Thread::IRQgetCurrentThread()->IRQgetPriority())
                IRQinvokeScheduler();
        rxWaiting=nullptr;
    }
}

int STM32SerialBase::ioctl(int cmd, void* arg)
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

//
// class STM32Serial
//

STM32Serial::STM32Serial(int id, int baudrate, GpioPin tx, GpioPin rx)
    : STM32SerialBase(id,baudrate,false), Device(Device::TTY)
{
    commonInit(id,baudrate,tx,rx,tx,rx); //The last two args will be ignored
}

STM32Serial::STM32Serial(int id, int baudrate, GpioPin tx, GpioPin rx,
                         GpioPin rts, GpioPin cts)
    : STM32SerialBase(id,baudrate,true), Device(Device::TTY)
{
    commonInit(id,baudrate,tx,rx,rts,cts);
}

void STM32Serial::commonInit(int id, int baudrate, GpioPin tx, GpioPin rx,
                             GpioPin rts, GpioPin cts)
{
    InterruptDisableLock dLock;
    port->IRQenable();
    IRQregisterIrq(port->getIRQn(),&STM32Serial::IRQhandleInterrupt,this);
    NVIC_SetPriority(port->getIRQn(),15);//Lowest priority for serial
    NVIC_EnableIRQ(port->getIRQn());
    STM32SerialBase::commonInit(id,baudrate,tx,rx,rts,cts);
    //Enabled, 8 data bit, no parity, interrupt on character rx
    port->get()->CR1 = USART_CR1_UE     //Enable port
                     | USART_CR1_RXNEIE //Interrupt on data received
                     | USART_CR1_IDLEIE //Interrupt on idle line
                     | USART_CR1_TE     //Transmission enbled
                     | USART_CR1_RE;    //Reception enabled
}

ssize_t STM32Serial::readBlock(void *buffer, size_t size, off_t where)
{
    return STM32SerialBase::readFromRxQueue(buffer, size);
}

ssize_t STM32Serial::writeBlock(const void *buffer, size_t size, off_t where)
{
    Lock<FastMutex> l(txMutex);
    DeepSleepLock dpLock;
    const char *buf=reinterpret_cast<const char*>(buffer);
    for(size_t i=0;i<size;i++)
    {
        while((port->get()->SR & USART_SR_TXE)==0) ;
        port->get()->DR=*buf++;
    }
    return size;
}

void STM32Serial::IRQhandleInterrupt()
{
    unsigned int status=port->get()->SR;
    char c;
    rxUpdateIdle(status);
    if(status & USART_SR_RXNE)
    {
        //Always read data, since this clears interrupt flags
        c=port->get()->DR;
        //If no error put data in buffer
        if((status & USART_SR_FE)==0)
            if(rxQueuePut(c)==false) /*fifo overflow*/;
    }
    if(status & USART_SR_IDLE)
    {
        c=port->get()->DR; //clears interrupt flags
    }
    if((status & USART_SR_IDLE) || rxQueue.size()>=rxQueueMin)
    {
        //Enough data in buffer or idle line, awake thread
        rxWakeup();
    }
}

void STM32Serial::IRQwrite(const char *str)
{
    // We can reach here also with only kernel paused, so make sure
    // interrupts are disabled
    bool interrupts=areInterruptsEnabled();
    if(interrupts) fastDisableInterrupts();
    STM32SerialBase::IRQwrite(str);
    if(interrupts) fastEnableInterrupts();
}

STM32Serial::~STM32Serial()
{
    waitSerialTxFifoEmpty();
    {
        InterruptDisableLock dLock;
        port->get()->CR1=0;
        NVIC_DisableIRQ(port->getIRQn());
        NVIC_ClearPendingIRQ(port->getIRQn());
        IRQunregisterIrq(port->getIRQn());
        port->IRQdisable();
    }
}

//
// class STM32DMASerial
//

STM32DMASerial::STM32DMASerial(int id, int baudrate, GpioPin tx, GpioPin rx)
    : STM32SerialBase(id,baudrate,false), Device(Device::TTY)
{
    commonInit(id,baudrate,tx,rx,tx,rx); //The last two args will be ignored
}

STM32DMASerial::STM32DMASerial(int id, int baudrate, GpioPin tx, GpioPin rx,
            GpioPin rts, GpioPin cts)
    : STM32SerialBase(id,baudrate,true), Device(Device::TTY)
{
    commonInit(id,baudrate,tx,rx,rts,cts);
}

void STM32DMASerial::commonInit(int id, int baudrate, GpioPin tx, GpioPin rx,
                    GpioPin rts, GpioPin cts)
{
    //Check if DMA is supported for this port
    if(!port->getDma()) errorHandler(UNEXPECTED);
    InterruptDisableLock dLock;

    port->IRQdmaEnable();
    IRQregisterIrq(port->getDmaTxIRQn(),&STM32DMASerial::IRQhandleDmaTxInterrupt,this);
    NVIC_SetPriority(port->getDmaTxIRQn(),15);
    NVIC_EnableIRQ(port->getDmaTxIRQn());
    //Higher priority to ensure IRQhandleDmaRxInterrupt() is called before
    //IRQhandleInterrupt(), so that idle is set correctly
    IRQregisterIrq(port->getDmaRxIRQn(),&STM32DMASerial::IRQhandleDmaRxInterrupt,this);
    NVIC_SetPriority(port->getDmaRxIRQn(),14);
    NVIC_EnableIRQ(port->getDmaRxIRQn());

    port->IRQenable();
    IRQregisterIrq(port->getIRQn(),&STM32DMASerial::IRQhandleInterrupt,this);
    NVIC_SetPriority(port->getIRQn(),15);//Lowest priority for serial
    NVIC_EnableIRQ(port->getIRQn());

    STM32SerialBase::commonInit(id,baudrate,tx,rx,rts,cts);

    port->get()->CR3 = USART_CR3_DMAT | USART_CR3_DMAR; //Enable USART DMA
    port->get()->CR1 = USART_CR1_UE     //Enable port
                       | USART_CR1_IDLEIE //Interrupt on idle line
                       | USART_CR1_TE     //Transmission enbled
                       | USART_CR1_RE;    //Reception enabled
    IRQstartDmaRead();
}

ssize_t STM32DMASerial::writeBlock(const void *buffer, size_t size, off_t where)
{
    Lock<FastMutex> l(txMutex);
    DeepSleepLock dpLock;
    const char *buf=reinterpret_cast<const char*>(buffer);
    size_t remaining=size;
    //If the client-provided buffer is not in CCM, we can use it directly
    //as DMA source memory (zero copy).
    if(isInCCMarea(buf)==false) 
    {
        //We don't use zero copy for the last txBufferSize bytes because in this
        //way we can return from this function a bit earlier.
        //If we returned while the DMA was still reading from the client
        //buffer, and the buffer is immediately rewritten, shit happens
        while(remaining>txBufferSize)
        {
            //DMA is limited to 64K
            size_t transferSize=min<size_t>(remaining-txBufferSize,65535);
            waitDmaWriteEnd();
            startDmaWrite(buf,transferSize);
            buf+=transferSize;
            remaining-=transferSize;
        }
    }
    //DMA out all remaining data through txBuffer
    while(remaining>0)
    {
        size_t transferSize=min(remaining,static_cast<size_t>(txBufferSize));
        waitDmaWriteEnd();
        //Copy to txBuffer only after DMA xfer completed, as the previous
        //xfer may be using the same buffer
        memcpy(txBuffer,buf,transferSize);
        startDmaWrite(txBuffer,transferSize);
        buf+=transferSize;
        remaining-=transferSize;
    }
    #ifdef WITH_DEEP_SLEEP
    //The serial driver by default can return even though the last part of
    //the data is still being transmitted by the DMA. When using deep sleep
    //however the DMA operation needs to be fully enclosed by a deep sleep
    //lock to prevent the scheduler from stopping peripheral clocks.
    waitDmaWriteEnd();
    waitSerialTxFifoEmpty(); //TODO: optimize by doing it only when entering deep sleep
    #endif //WITH_DEEP_SLEEP
    return size;
}

void STM32DMASerial::startDmaWrite(const char *buffer, size_t size)
{
    markBufferBeforeDmaWrite(buffer,size);
    //Quirk: DMA messes up the TC bit, and causes waitSerialTxFifoEmpty() to
    //return prematurely, causing characters to be missed when rebooting
    //immediately a write. You can just clear the bit manually, but doing that
    //is dangerous, as if you clear the bit but for any reason the serial
    //write doesn't start (think an invalid buffer, or another thread crashing),
    //then TC will never be set and waitSerialTxFifoEmpty() deadlocks!
    //The only way to clear it safely is to first read SR and then write to
    //DR (thus the bit is cleared at the same time a transmission is started,
    //and the race condition is eliminated). This is the purpose of this
    //instruction, it reads SR. When we start the DMA, the DMA controller
    //writes to DR and completes the TC clear sequence.
    DeepSleepLock dpLock;
    while((port->get()->SR & USART_SR_TXE)==0) ;
    
    dmaTxInProgress=true;
    port->getDmaTx()->PAR=reinterpret_cast<unsigned int>(&port->get()->DR);
    port->getDmaTx()->M0AR=reinterpret_cast<unsigned int>(buffer);
    port->getDmaTx()->NDTR=size;
    //Quirk: not enabling DMA_SxFCR_FEIE because the USART seems to
    //generate a spurious fifo error. The code was tested and the
    //transfer completes successfully even in the presence of this fifo
    //error
    port->getDmaTx()->FCR=DMA_SxFCR_DMDIS;//Enable fifo
    port->getDmaTx()->CR =
                (port->getDmaTxChannel() << DMA_SxCR_CHSEL_Pos) //Select channel
              | DMA_SxCR_MINC    //Increment RAM pointer
              | DMA_SxCR_DIR_0   //Memory to peripheral
              | DMA_SxCR_TCIE    //Interrupt on completion
              | DMA_SxCR_TEIE    //Interrupt on transfer error
              | DMA_SxCR_DMEIE   //Interrupt on direct mode error
              | DMA_SxCR_EN;     //Start the DMA
}

void STM32DMASerial::IRQhandleDmaTxInterrupt()
{
    port->setDmaTxIFCR(
          DMA_LIFCR_CTCIF0
        | DMA_LIFCR_CTEIF0
        | DMA_LIFCR_CDMEIF0
        | DMA_LIFCR_CFEIF0);
    dmaTxInProgress=false;
    if(txWaiting==nullptr) return;
    txWaiting->IRQwakeup();
    if(txWaiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
        IRQinvokeScheduler();
    txWaiting=nullptr;
}

void STM32DMASerial::waitDmaWriteEnd()
{
    FastInterruptDisableLock dLock;
    // If a previous DMA xfer is in progress, wait
    if(dmaTxInProgress)
    {
        txWaiting=Thread::IRQgetCurrentThread();
        do {
            Thread::IRQwait();
            {
                FastInterruptEnableLock eLock(dLock);
                Thread::yield();
            }
        } while(txWaiting);
    }
}

ssize_t STM32DMASerial::readBlock(void *buffer, size_t size, off_t where)
{
    return STM32SerialBase::readFromRxQueue(buffer, size);
}

void STM32DMASerial::IRQstartDmaRead()
{
    port->getDmaRx()->PAR=reinterpret_cast<unsigned int>(&port->get()->DR);
    port->getDmaRx()->M0AR=reinterpret_cast<unsigned int>(rxBuffer);
    port->getDmaRx()->NDTR=rxQueueMin;
    port->getDmaRx()->CR = 
                (port->getDmaRxChannel() << DMA_SxCR_CHSEL_Pos) //Select channel
              | DMA_SxCR_MINC    //Increment RAM pointer
              | 0                //Peripheral to memory
              | DMA_SxCR_HTIE    //Interrupt on half transfer
              | DMA_SxCR_TEIE    //Interrupt on transfer error
              | DMA_SxCR_DMEIE   //Interrupt on direct mode error
              | DMA_SxCR_EN;     //Start the DMA
}

int STM32DMASerial::IRQstopDmaRead()
{
    //Stop DMA and wait for it to actually stop
    port->getDmaRx()->CR &= ~DMA_SxCR_EN;
    while(port->getDmaRx()->CR & DMA_SxCR_EN) ;
    port->setDmaRxIFCR(
          DMA_LIFCR_CTCIF0
        | DMA_LIFCR_CHTIF0
        | DMA_LIFCR_CTEIF0
        | DMA_LIFCR_CDMEIF0
        | DMA_LIFCR_CFEIF0);
    return rxQueueMin - port->getDmaRx()->NDTR;
}

void STM32DMASerial::IRQflushDmaReadBuffer()
{
    int elem=IRQstopDmaRead();
    markBufferAfterDmaRead(rxBuffer,rxQueueMin);
    for(int i=0;i<elem;i++)
        if(rxQueue.tryPut(rxBuffer[i])==false) /*fifo overflow*/;
    IRQstartDmaRead();
}

void STM32DMASerial::IRQhandleDmaRxInterrupt()
{
    IRQflushDmaReadBuffer();
    rxUpdateIdle(0);
    rxWakeup();
}

void STM32DMASerial::IRQhandleInterrupt()
{
    unsigned int status=port->get()->SR;
    rxUpdateIdle(status);
    if(status & USART_SR_IDLE)
    {
        (unsigned long)port->get()->DR; //clears interrupt flags
        IRQflushDmaReadBuffer();
    }
    if((status & USART_SR_IDLE) || rxQueue.size()>=rxQueueMin)
    {
        //Enough data in buffer or idle line, awake thread
        rxWakeup();
    }
}

void STM32DMASerial::IRQwrite(const char *str)
{
    //We can reach here also with only kernel paused, so make sure
    //interrupts are disabled
    bool interrupts=areInterruptsEnabled();
    if(interrupts) fastDisableInterrupts();
    //Wait until DMA xfer ends. EN bit is cleared by hardware on transfer end
    while(port->getDmaTx()->CR & DMA_SxCR_EN) ;
    STM32SerialBase::IRQwrite(str);
    if(interrupts) fastEnableInterrupts();
}

STM32DMASerial::~STM32DMASerial()
{
    waitSerialTxFifoEmpty();
    {
        InterruptDisableLock dLock;
        port->get()->CR1=0;
        IRQstopDmaRead();
        NVIC_DisableIRQ(port->getDmaTxIRQn());
        NVIC_ClearPendingIRQ(port->getDmaTxIRQn());
        IRQunregisterIrq(port->getDmaTxIRQn());
        NVIC_DisableIRQ(port->getDmaRxIRQn());
        NVIC_ClearPendingIRQ(port->getDmaRxIRQn());
        IRQunregisterIrq(port->getDmaRxIRQn());
        NVIC_DisableIRQ(port->getIRQn());
        NVIC_ClearPendingIRQ(port->getIRQn());
        IRQunregisterIrq(port->getIRQn());
        port->IRQdisable();
    }
}

} //namespace miosix
