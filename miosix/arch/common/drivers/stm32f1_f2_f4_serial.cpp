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
#include "stm32f1_f2_f4_serial.h"
#include "kernel/sync.h"
#include "kernel/scheduler/scheduler.h"
#include "filesystem/ioctl.h"
#include "cache/cortexMx_cache.h"
#include "interfaces/gpio.h"

using namespace std;

/*
 * This serial driver supports multiple revisions of the hardware; the following
 * defines select the hardware variant and additional quirks that need to be
 * taken into account in the code.
 */

#if defined(_ARCH_CORTEXM3_STM32F1)
#define BUS_HAS_AHB
#define DMA_STM32F1
#define ALTFUNC_STM32F1
#elif defined(_ARCH_CORTEXM3_STM32L1)
#define BUS_HAS_AHB
#define DMA_STM32F1
#define ALTFUNC_STM32F2
#else
#define BUS_HAS_AHB12
#define DMA_STM32F2
#define ALTFUNC_STM32F2
#endif

namespace miosix {

/*
 * Helper class for handling enabling/disabling peripherals on a STM32 bus
 */

class STM32Bus
{
public:
    enum ID {
        APB1, APB2,
        #if defined(BUS_HAS_AHB)
            AHB,
        #elif defined(BUS_HAS_AHB12)
            AHB1, AHB2
        #endif // BUS_HAS_x
    };

    static inline void IRQen(STM32Bus::ID bus, unsigned long mask)
    {
        switch(bus)
        {
            case STM32Bus::APB1: RCC->APB1ENR|=mask; break;
            case STM32Bus::APB2: RCC->APB2ENR|=mask; break;
            #if defined(BUS_HAS_AHB)
                case STM32Bus::AHB: RCC->AHBENR|=mask; break;
            #elif defined(BUS_HAS_AHB12)
                case STM32Bus::AHB1: RCC->AHB1ENR|=mask; break;
                case STM32Bus::AHB2: RCC->AHB2ENR|=mask; break;
            #endif // BUS_HAS_x
        }
        RCC_SYNC();
    }
    static inline void IRQdis(STM32Bus::ID bus, unsigned long mask)
    {
        switch(bus)
        {
            case STM32Bus::APB1: RCC->APB1ENR&=~mask; break;
            case STM32Bus::APB2: RCC->APB2ENR&=~mask; break;
            #if defined(BUS_HAS_AHB)
                case STM32Bus::AHB: RCC->AHBENR|=mask; break;
            #elif defined(BUS_HAS_AHB12)
                case STM32Bus::AHB1: RCC->AHB1ENR&=~mask; break;
                case STM32Bus::AHB2: RCC->AHB2ENR&=~mask; break;
            #endif // BUS_HAS_x
        }
        RCC_SYNC();
    }
};

/*
 * Auxiliary class that encapsulates all parts of code that differ between
 * between each instance of the DMA interface
 * 
 * Try not to use the attributes of this class directly even if they are public,
 * in the current implementation they are not the best use of ROM space and
 * by using getter/setters we make it easier for ourselves to compress this
 * data structure a bit in the future.
 * TODO: Remove redundant information in the attributes
 */

#if defined(DMA_STM32F1)

class STM32SerialDMAHW
{
public:
    enum IntRegShift
    {
        Channel1=0*4, Channel2=1*4, Channel3=2*4,
        Channel4=3*4, Channel5=4*4, Channel6=5*4, Channel7=6*4
    };

    inline DMA_TypeDef *get() const { return DMA1; }
    inline void IRQenable() const { STM32Bus::IRQen(STM32Bus::AHB, RCC_AHBENR_DMA1EN); }
    inline void IRQdisable() const { STM32Bus::IRQdis(STM32Bus::AHB, RCC_AHBENR_DMA1EN); }

    inline IRQn_Type getTxIRQn() const { return txIrq; }
    inline unsigned long getTxISR() const { return getISR(txIRShift); }
    inline void setTxIFCR(unsigned long v) const { return setIFCR(txIRShift,v); }

    inline IRQn_Type getRxIRQn() const { return rxIrq; }
    inline unsigned long getRxISR() const { return getISR(rxIRShift); }
    inline void setRxIFCR(unsigned long v) const { return setIFCR(rxIRShift,v); }

    inline void startDmaWrite(volatile uint32_t *dr, const char *buffer, size_t size) const
    {
        tx->CPAR=reinterpret_cast<unsigned int>(dr);
        tx->CMAR=reinterpret_cast<unsigned int>(buffer);
        tx->CNDTR=size;
        tx->CCR = DMA_CCR_MINC    //Increment RAM pointer
                | DMA_CCR_DIR     //Memory to peripheral
                | DMA_CCR_TCIE    //Interrupt on completion
                | DMA_CCR_TEIE    //Interrupt on transfer error
                | DMA_CCR_EN;     //Start the DMA
    }

    inline void IRQhandleDmaTxInterrupt() const
    {
        setTxIFCR(DMA_IFCR_CGIF1);
        tx->CCR=0; //Disable DMA
    }

    inline void IRQwaitDmaWriteStop() const
    {
        while((tx->CCR & DMA_CCR_EN) && !(getTxISR() & (DMA_ISR_TCIF1|DMA_ISR_TEIF1))) ;
    }

    inline void IRQstartDmaRead(volatile uint32_t *dr, const char *buffer, unsigned int size) const
    {
        rx->CPAR=reinterpret_cast<unsigned int>(dr);
        rx->CMAR=reinterpret_cast<unsigned int>(buffer);
        rx->CNDTR=size;
        rx->CCR = DMA_CCR_MINC    //Increment RAM pointer
                | 0               //Peripheral to memory
                | DMA_CCR_TEIE    //Interrupt on transfer error
                | DMA_CCR_TCIE    //Interrupt on transfer complete
                | DMA_CCR_EN;     //Start the DMA
    }

    inline int IRQstopDmaRead() const
    {
        rx->CCR=0;
        setRxIFCR(DMA_IFCR_CGIF1);
        return rx->CNDTR;
    }

    DMA_Channel_TypeDef *tx;    ///< Pointer to DMA TX channel
    IRQn_Type txIrq;            ///< DMA TX stream IRQ number
    unsigned char txIRShift;    ///< Value from DMAIntRegShift for the stream

    DMA_Channel_TypeDef *rx;    ///< Pointer to DMA RX channel
    IRQn_Type rxIrq;            ///< DMA RX stream IRQ number
    unsigned char rxIRShift;    ///< Value from DMAIntRegShift for the stream

private:
    inline unsigned long getISR(unsigned char pos) const
    {
        return (get()->ISR>>pos) & 0b1111;
    }
    inline void setIFCR(unsigned char pos, unsigned long value) const
    {
        get()->IFCR=(value&0b1111) << pos;
    }
};

#elif defined(DMA_STM32F2)

class STM32SerialDMAHW
{
public:
    enum IntRegShift
    {
        Stream0= 0, Stream1= 0+6, Stream2=16, Stream3=16+6,
        Stream4=32, Stream5=32+6, Stream6=48, Stream7=48+6
    };

    inline DMA_TypeDef *get() const { return dma; }
    inline void IRQenable() const { STM32Bus::IRQen(bus, clkEnMask); }
    inline void IRQdisable() const { STM32Bus::IRQdis(bus, clkEnMask); }

    inline IRQn_Type getTxIRQn() const { return txIrq; }
    inline unsigned long getTxISR() const { return getISR(txIRShift); }
    inline void setTxIFCR(unsigned long v) const { return setIFCR(txIRShift,v); }

    inline IRQn_Type getRxIRQn() const { return rxIrq; }
    inline unsigned long getRxISR() const { return getISR(rxIRShift); }
    inline void setRxIFCR(unsigned long v) const { return setIFCR(rxIRShift,v); }

    inline void startDmaWrite(volatile uint32_t *dr, const char *buffer, size_t size) const
    {
        tx->PAR=reinterpret_cast<unsigned int>(dr);
        tx->M0AR=reinterpret_cast<unsigned int>(buffer);
        tx->NDTR=size;
        //Quirk: not enabling DMA_SxFCR_FEIE because at the beginning of a transfer
        //there is always at least one spurious fifo error due to the fact that the
        //FIFO doesn't begin to fill up until after the DMA request is triggered.
        //  This is just a FIFO underrun error and as such it is resolved
        //automatically by the DMA internal logic and does not stop the transfer,
        //just like for FIFO overruns.
        //  On the other hand, a FIFO error caused by register misconfiguration
        //would prevent the transfer from even starting, but the conditions for
        //FIFO misconfiguration are known in advance and we don't fall in any of
        //those cases.
        //  In other words, underrun, overrun and misconfiguration are the only FIFO
        //error conditions; misconfiguration is impossible, and we don't need to do
        //anything for overruns and underruns, so there is literally no reason to
        //enable FIFO error interrupts in the first place.
        tx->FCR=DMA_SxFCR_DMDIS;//Enable fifo
        tx->CR = (txChannel << DMA_SxCR_CHSEL_Pos) //Select channel
               | DMA_SxCR_MINC    //Increment RAM pointer
               | DMA_SxCR_DIR_0   //Memory to peripheral
               | DMA_SxCR_TCIE    //Interrupt on completion
               | DMA_SxCR_TEIE    //Interrupt on transfer error
               | DMA_SxCR_DMEIE   //Interrupt on direct mode error
               | DMA_SxCR_EN;     //Start the DMA
    }

    inline void IRQhandleDmaTxInterrupt() const
    {
        setTxIFCR(DMA_LIFCR_CTCIF0
                | DMA_LIFCR_CTEIF0
                | DMA_LIFCR_CDMEIF0
                | DMA_LIFCR_CFEIF0);
    }

    inline void IRQwaitDmaWriteStop() const
    {
        while(tx->CR & DMA_SxCR_EN) ;
    }

    inline void IRQstartDmaRead(volatile uint32_t *dr, const char *buffer, unsigned int size) const
    {
        rx->PAR=reinterpret_cast<unsigned int>(dr);
        rx->M0AR=reinterpret_cast<unsigned int>(buffer);
        rx->NDTR=size;
        rx->CR = (rxChannel << DMA_SxCR_CHSEL_Pos) //Select channel
               | DMA_SxCR_MINC    //Increment RAM pointer
               | 0                //Peripheral to memory
               | DMA_SxCR_HTIE    //Interrupt on half transfer
               | DMA_SxCR_TEIE    //Interrupt on transfer error
               | DMA_SxCR_DMEIE   //Interrupt on direct mode error
               | DMA_SxCR_EN;     //Start the DMA
    }

    inline int IRQstopDmaRead() const
    {
        //Stop DMA and wait for it to actually stop
        rx->CR &= ~DMA_SxCR_EN;
        while(rx->CR & DMA_SxCR_EN) ;
        setRxIFCR(DMA_LIFCR_CTCIF0
            | DMA_LIFCR_CHTIF0
            | DMA_LIFCR_CTEIF0
            | DMA_LIFCR_CDMEIF0
            | DMA_LIFCR_CFEIF0);
        return rx->NDTR;
    }

    DMA_TypeDef *dma;           ///< Pointer to the DMA peripheral (DMA1/2)
    STM32Bus::ID bus;           ///< Bus where the DMA port is (AHB1 or 2)
    unsigned long clkEnMask;    ///< DMA clock enable bit

    DMA_Stream_TypeDef *tx;     ///< Pointer to DMA TX stream
    IRQn_Type txIrq;            ///< DMA TX stream IRQ number
    unsigned char txIRShift;    ///< Value from DMAIntRegShift for the stream
    unsigned char txChannel;    ///< DMA TX stream channel

    DMA_Stream_TypeDef *rx;     ///< Pointer to DMA RX stream
    IRQn_Type rxIrq;            ///< DMA RX stream IRQ number
    unsigned char rxIRShift;    ///< Value from DMAIntRegShift for the stream
    unsigned char rxChannel;    ///< DMA TX stream channel

private:
    inline unsigned long getISR(unsigned char pos) const
    {
        if(pos<32) return (dma->LISR>>pos) & 0b111111;
        return (dma->HISR>>(pos-32)) & 0b111111;
    }
    inline void setIFCR(unsigned char pos, unsigned long value) const
    {
        value=(value&0b111111) << (pos%32);
        if(pos<32) dma->LIFCR=value;
        else dma->HIFCR=value;
    }
};

#endif // DMA_STM32Fx

/*
 * Auxiliary class that encapsulates all parts of code that differ between
 * between each instance of the USART peripheral.
 * 
 * Try not to use the attributes of this class directly even if they are public.
 */
class STM32SerialHW
{
public:
    inline USART_TypeDef *get() const { return port; }
    inline IRQn_Type getIRQn() const { return irq; }
    #ifdef ALTFUNC_STM32F2
        inline unsigned char getAltFunc() const { return altFunc; }
    #endif
    inline unsigned int IRQgetClock() const
    {
        unsigned int freq=SystemCoreClock;
        switch(bus)
        {
            case STM32Bus::APB1:
                if(RCC->CFGR & RCC_CFGR_PPRE1_2)
                    freq/=1<<(((RCC->CFGR>>RCC_CFGR_PPRE1_Pos) & 0x3)+1);
                break;
            case STM32Bus::APB2:
                if(RCC->CFGR & RCC_CFGR_PPRE2_2)
                    freq/=1<<(((RCC->CFGR>>RCC_CFGR_PPRE2_Pos) & 0x3)+1);
                break;
            default:
                break;
        }
        return freq;
    }
    inline void IRQenable() const { STM32Bus::IRQen(bus, clkEnMask); }
    inline void IRQdisable() const { STM32Bus::IRQdis(bus, clkEnMask); }
    inline const STM32SerialDMAHW& getDma() const { return dma; } 

    USART_TypeDef *port;        ///< USART port
    IRQn_Type irq;              ///< USART IRQ number
    #ifdef ALTFUNC_STM32F2
        unsigned char altFunc;  ///< Alternate function to set for GPIOs
    #endif
    STM32Bus::ID bus;           ///< Bus where the port is (APB1 or 2)
    unsigned long clkEnMask;    ///< USART clock enable

    STM32SerialDMAHW dma;
};

/*
 * Table of hardware configurations
 */

#if defined(STM32F100xB) || defined(STM32F103xB)
constexpr int maxPorts = 3;
static const STM32SerialHW ports[maxPorts] = {
    { USART1, USART1_IRQn, STM32Bus::APB2, RCC_APB2ENR_USART1EN,
      { DMA1_Channel4, DMA1_Channel4_IRQn, STM32SerialDMAHW::Channel4,
        DMA1_Channel5, DMA1_Channel5_IRQn, STM32SerialDMAHW::Channel5 } },
    { USART2, USART2_IRQn, STM32Bus::APB1, RCC_APB1ENR_USART2EN,
      { DMA1_Channel7, DMA1_Channel7_IRQn, STM32SerialDMAHW::Channel7,
        DMA1_Channel6, DMA1_Channel6_IRQn, STM32SerialDMAHW::Channel6 } },
    { USART3, USART3_IRQn, STM32Bus::APB1, RCC_APB1ENR_USART3EN,
      { DMA1_Channel2, DMA1_Channel2_IRQn, STM32SerialDMAHW::Channel2,
        DMA1_Channel3, DMA1_Channel3_IRQn, STM32SerialDMAHW::Channel3 } },
};
#elif defined(STM32F103xE)
constexpr int maxPorts = 5;
static const STM32SerialHW ports[maxPorts] = {
    { USART1, USART1_IRQn, STM32Bus::APB2, RCC_APB2ENR_USART1EN,
      { DMA1_Channel4, DMA1_Channel4_IRQn, STM32SerialDMAHW::Channel4,
        DMA1_Channel5, DMA1_Channel5_IRQn, STM32SerialDMAHW::Channel5 } },
    { USART2, USART2_IRQn, STM32Bus::APB1, RCC_APB1ENR_USART2EN,
      { DMA1_Channel7, DMA1_Channel7_IRQn, STM32SerialDMAHW::Channel7,
        DMA1_Channel6, DMA1_Channel6_IRQn, STM32SerialDMAHW::Channel6 } },
    { USART3, USART3_IRQn, STM32Bus::APB1, RCC_APB1ENR_USART3EN,
      { DMA1_Channel2, DMA1_Channel2_IRQn, STM32SerialDMAHW::Channel2,
        DMA1_Channel3, DMA1_Channel3_IRQn, STM32SerialDMAHW::Channel3 } },
    { UART4, UART4_IRQn, STM32Bus::APB1, RCC_APB1ENR_UART4EN, { 0 } },
    { UART5, UART5_IRQn, STM32Bus::APB1, RCC_APB1ENR_UART5EN, { 0 } },
};
#elif defined(STM32L152xE)
constexpr int maxPorts = 5;
static const STM32SerialHW ports[maxPorts] = {
    { USART1, USART1_IRQn, 7, STM32Bus::APB2, RCC_APB2ENR_USART1EN,
      { DMA1_Channel4, DMA1_Channel4_IRQn, STM32SerialDMAHW::Channel4,
        DMA1_Channel5, DMA1_Channel5_IRQn, STM32SerialDMAHW::Channel5 } },
    { USART2, USART2_IRQn, 7, STM32Bus::APB1, RCC_APB1ENR_USART2EN,
      { DMA1_Channel7, DMA1_Channel7_IRQn, STM32SerialDMAHW::Channel7,
        DMA1_Channel6, DMA1_Channel6_IRQn, STM32SerialDMAHW::Channel6 } },
    { USART3, USART3_IRQn, 7, STM32Bus::APB1, RCC_APB1ENR_USART3EN,
      { DMA1_Channel2, DMA1_Channel2_IRQn, STM32SerialDMAHW::Channel2,
        DMA1_Channel3, DMA1_Channel3_IRQn, STM32SerialDMAHW::Channel3 } },
    { UART4 , UART4_IRQn , 8, STM32Bus::APB1, RCC_APB1ENR_UART4EN, { 0 } },
    { UART5 , UART5_IRQn , 8, STM32Bus::APB1, RCC_APB1ENR_UART5EN, { 0 } },
};
#elif defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F411xE)
constexpr int maxPorts = 6;
static const STM32SerialHW ports[maxPorts] = {
    { USART1, USART1_IRQn, 7, STM32Bus::APB2, RCC_APB2ENR_USART1EN,
      { DMA2, STM32Bus::AHB1, RCC_AHB1ENR_DMA2EN,
        DMA2_Stream7, DMA2_Stream7_IRQn, STM32SerialDMAHW::Stream7, 4,
        DMA2_Stream5, DMA2_Stream5_IRQn, STM32SerialDMAHW::Stream5, 4 } },
    { USART2, USART2_IRQn, 7, STM32Bus::APB1, RCC_APB1ENR_USART2EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream6, DMA1_Stream6_IRQn, STM32SerialDMAHW::Stream6, 4,
        DMA1_Stream5, DMA1_Stream5_IRQn, STM32SerialDMAHW::Stream5, 4 } },
    { 0 },
    { 0 },
    { 0 },
    { USART6, USART6_IRQn, 8, STM32Bus::APB2, RCC_APB2ENR_USART6EN,
      { DMA2, STM32Bus::AHB1, RCC_AHB1ENR_DMA2EN,
        DMA2_Stream6, DMA2_Stream6_IRQn, STM32SerialDMAHW::Stream6, 5,
        DMA2_Stream1, DMA2_Stream1_IRQn, STM32SerialDMAHW::Stream1, 5 } },
};
#elif defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) \
   || defined(STM32F417xx) || defined(STM32F205xx) || defined(STM32F207xx)
constexpr int maxPorts = 6;
static const STM32SerialHW ports[maxPorts] = {
    { USART1, USART1_IRQn, 7, STM32Bus::APB2, RCC_APB2ENR_USART1EN,
      { DMA2, STM32Bus::AHB1, RCC_AHB1ENR_DMA2EN,
        DMA2_Stream7, DMA2_Stream7_IRQn, STM32SerialDMAHW::Stream7, 4,
        DMA2_Stream5, DMA2_Stream5_IRQn, STM32SerialDMAHW::Stream5, 4 } },
    { USART2, USART2_IRQn, 7, STM32Bus::APB1, RCC_APB1ENR_USART2EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream6, DMA1_Stream6_IRQn, STM32SerialDMAHW::Stream6, 4,
        DMA1_Stream5, DMA1_Stream5_IRQn, STM32SerialDMAHW::Stream5, 4 } },
    { USART3, USART3_IRQn, 7, STM32Bus::APB1, RCC_APB1ENR_USART3EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream3, DMA1_Stream3_IRQn, STM32SerialDMAHW::Stream3, 4,
        DMA1_Stream1, DMA1_Stream1_IRQn, STM32SerialDMAHW::Stream1, 4 } },
    { UART4 , UART4_IRQn , 8, STM32Bus::APB1, RCC_APB1ENR_UART4EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream4, DMA1_Stream4_IRQn, STM32SerialDMAHW::Stream4, 4,
        DMA1_Stream2, DMA1_Stream2_IRQn, STM32SerialDMAHW::Stream2, 4 } },
    { UART5 , UART5_IRQn , 8, STM32Bus::APB1, RCC_APB1ENR_UART5EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream7, DMA1_Stream7_IRQn, STM32SerialDMAHW::Stream7, 4,
        DMA1_Stream0, DMA1_Stream0_IRQn, STM32SerialDMAHW::Stream0, 4 } },
    { USART6, USART6_IRQn, 8, STM32Bus::APB2, RCC_APB2ENR_USART6EN,
      { DMA2, STM32Bus::AHB1, RCC_AHB1ENR_DMA2EN,
        DMA2_Stream6, DMA2_Stream6_IRQn, STM32SerialDMAHW::Stream6, 5,
        DMA2_Stream1, DMA2_Stream1_IRQn, STM32SerialDMAHW::Stream1, 5 } },
};
#elif defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F469xx) \
   || defined(STM32F479xx)
constexpr int maxPorts = 8;
static const STM32SerialHW ports[maxPorts] = {
    { USART1, USART1_IRQn, 7, STM32Bus::APB2, RCC_APB2ENR_USART1EN,
      { DMA2, STM32Bus::AHB1, RCC_AHB1ENR_DMA2EN,
        DMA2_Stream7, DMA2_Stream7_IRQn, STM32SerialDMAHW::Stream7, 4,
        DMA2_Stream5, DMA2_Stream5_IRQn, STM32SerialDMAHW::Stream5, 4 } },
    { USART2, USART2_IRQn, 7, STM32Bus::APB1, RCC_APB1ENR_USART2EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream6, DMA1_Stream6_IRQn, STM32SerialDMAHW::Stream6, 4,
        DMA1_Stream5, DMA1_Stream5_IRQn, STM32SerialDMAHW::Stream5, 4 } },
    { USART3, USART3_IRQn, 7, STM32Bus::APB1, RCC_APB1ENR_USART3EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream3, DMA1_Stream3_IRQn, STM32SerialDMAHW::Stream3, 4,
        DMA1_Stream1, DMA1_Stream1_IRQn, STM32SerialDMAHW::Stream1, 4 } },
    { UART4 , UART4_IRQn , 8, STM32Bus::APB1, RCC_APB1ENR_UART4EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream4, DMA1_Stream4_IRQn, STM32SerialDMAHW::Stream4, 4,
        DMA1_Stream2, DMA1_Stream2_IRQn, STM32SerialDMAHW::Stream2, 4 } },
    { UART5 , UART5_IRQn , 8, STM32Bus::APB1, RCC_APB1ENR_UART5EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream7, DMA1_Stream7_IRQn, STM32SerialDMAHW::Stream7, 4,
        DMA1_Stream0, DMA1_Stream0_IRQn, STM32SerialDMAHW::Stream0, 4 } },
    { USART6, USART6_IRQn, 8, STM32Bus::APB2, RCC_APB2ENR_USART6EN,
      { DMA2, STM32Bus::AHB1, RCC_AHB1ENR_DMA2EN,
        DMA2_Stream6, DMA2_Stream6_IRQn, STM32SerialDMAHW::Stream6, 5,
        DMA2_Stream1, DMA2_Stream1_IRQn, STM32SerialDMAHW::Stream1, 5 } },
    { UART7 , UART7_IRQn , 8, STM32Bus::APB1, RCC_APB1ENR_UART7EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream1, DMA1_Stream1_IRQn, STM32SerialDMAHW::Stream1, 5,
        DMA1_Stream3, DMA1_Stream3_IRQn, STM32SerialDMAHW::Stream3, 5 } },
    { UART8 , UART8_IRQn , 8, STM32Bus::APB1, RCC_APB1ENR_UART8EN,
      { DMA1, STM32Bus::AHB1, RCC_AHB1ENR_DMA1EN,
        DMA1_Stream0, DMA1_Stream0_IRQn, STM32SerialDMAHW::Stream0, 5,
        DMA1_Stream6, DMA1_Stream6_IRQn, STM32SerialDMAHW::Stream6, 5 } },
};
#else
#error Unsupported STM32 chip for this serial driver
#endif

//
// class STM32SerialBase
//

// A note on the baudrate/500: the buffer is selected so as to withstand
// 20ms of full data rate. In the 8N1 format one char is made of 10 bits.
// So (baudrate/10)*0.02=baudrate/500
STM32SerialBase::STM32SerialBase(int id, int baudrate, bool flowControl) : 
    flowControl(flowControl), portId(id), rxQueue(rxQueueMin+baudrate/500)
{
    if(id<1 || id>maxPorts) errorHandler(UNEXPECTED);
    port=&ports[id-1];
    if(port->get()==nullptr) errorHandler(UNEXPECTED);
}

void STM32SerialBase::commonInit(int id, int baudrate, GpioPin tx, GpioPin rx,
                    GpioPin rts, GpioPin cts)
{
    #if defined(ALTFUNC_STM32F1)
        //Quirk: stm32f1 rx pin has to be in input mode, while stm32f2 and up
        //want it in ALTERNATE mode. Go figure...
        tx.mode(Mode::ALTERNATE);
        rx.mode(Mode::INPUT);
        if(flowControl)
        {
            rts.mode(Mode::ALTERNATE);
            cts.mode(Mode::INPUT);
        }
    #elif defined(ALTFUNC_STM32F2)
        //First we set the AF then the mode to avoid glitches
        tx.alternateFunction(port->getAltFunc());
        tx.mode(Mode::ALTERNATE);
        rx.alternateFunction(port->getAltFunc());
        rx.mode(Mode::ALTERNATE);
        if(flowControl)
        {
            rts.alternateFunction(port->getAltFunc());
            rts.mode(Mode::ALTERNATE);
            cts.alternateFunction(port->getAltFunc());
            cts.mode(Mode::ALTERNATE);
        }
    #endif
    unsigned int freq=port->IRQgetClock();
    unsigned int quot=2*freq/baudrate;      //2*freq for round to nearest
    port->get()->BRR=quot/2 + (quot & 1);   //Round to nearest
    // Some STM32 (i.e. F103xB) have broken one bit sampling mode,
    // so ST "helpfully" removed the register field from the headers.
    #ifdef USART_CR3_ONEBIT
        unsigned long onebit=USART_CR3_ONEBIT;
    #else
        unsigned long onebit=0;
    #endif
    if(flowControl==false) port->get()->CR3 |= onebit;
    else port->get()->CR3 |= onebit | USART_CR3_RTSE | USART_CR3_CTSE;
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
    auto dma=port->getDma();
    if(!dma.get()) errorHandler(UNEXPECTED);
    InterruptDisableLock dLock;

    dma.IRQenable();
    IRQregisterIrq(dma.getTxIRQn(),&STM32DMASerial::IRQhandleDmaTxInterrupt,this);
    NVIC_SetPriority(dma.getTxIRQn(),15);
    NVIC_EnableIRQ(dma.getTxIRQn());
    //Higher priority to ensure IRQhandleDmaRxInterrupt() is called before
    //IRQhandleInterrupt(), so that idle is set correctly
    IRQregisterIrq(dma.getRxIRQn(),&STM32DMASerial::IRQhandleDmaRxInterrupt,this);
    NVIC_SetPriority(dma.getRxIRQn(),14);
    NVIC_EnableIRQ(dma.getRxIRQn());

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
    DeepSleepLock dpLock;
    //The TC (Transfer Complete) bit in the Status Register (SR) of the serial
    //port can be reset by writing to it directly, or by first reading it
    //out and then writing to the Data Register (DR).
    //  The waitSerialTxFifoEmpty() function relies on the status of TC to
    //accurately reflect whether the serial port is pushing out bytes or not,
    //so it is extremely important for TC to *only* be reset at the beginning of
    //a transmission.
    //  Since the DMA peripheral only writes to DR and never reads from SR,
    //we must read from SR manually first to ensure TC is cleared as soon as
    //the DMA writes to it -- and not earlier!
    //  The alternative is to zero out TC by hand, but if we do that TC becomes
    //unreliable as a flag. Consider the case in which we are clearing out TC
    //in this routine before configuring the DMA. If the DMA fails to start due
    //to an error, or the CPU is reset before the DMA transfer is started, the
    //USART won't be pushing out bytes but TC will still be zero. As a result,
    //waitSerialTxFifoEmpty() will loop forever waiting for the end of a
    //transfer that is not happening.
    while((port->get()->SR & USART_SR_TXE)==0) ;
    
    dmaTxInProgress=true;
    port->getDma().startDmaWrite(&port->get()->DR,buffer,size);
}

void STM32DMASerial::IRQhandleDmaTxInterrupt()
{
    port->getDma().IRQhandleDmaTxInterrupt();
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
    port->getDma().IRQstartDmaRead(&port->get()->DR,rxBuffer,rxQueueMin);
}

int STM32DMASerial::IRQstopDmaRead()
{
    return rxQueueMin - port->getDma().IRQstopDmaRead();
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
    port->getDma().IRQwaitDmaWriteStop();
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
        auto dma=port->getDma();
        NVIC_DisableIRQ(dma.getTxIRQn());
        NVIC_ClearPendingIRQ(dma.getTxIRQn());
        IRQunregisterIrq(dma.getTxIRQn());
        NVIC_DisableIRQ(dma.getRxIRQn());
        NVIC_ClearPendingIRQ(dma.getRxIRQn());
        IRQunregisterIrq(dma.getRxIRQn());
        NVIC_DisableIRQ(port->getIRQn());
        NVIC_ClearPendingIRQ(port->getIRQn());
        IRQunregisterIrq(port->getIRQn());
        port->IRQdisable();
    }
}

} //namespace miosix
