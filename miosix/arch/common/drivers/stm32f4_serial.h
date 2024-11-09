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

#pragma once

#include "filesystem/console/console_device.h"
#include "kernel/sync.h"
#include "kernel/queue.h"
#include "interfaces/gpio.h"
#include "board_settings.h"

namespace miosix {

class STM32SerialHW;

/**
 * Serial port class for stm32 microcontrollers.
 * 
 * Classes of this type are reference counted, must be allocated on the heap
 * and managed through intrusive_ref_ptr<FileBase>
 */
class STM32Serial : public Device
{
public:
    /**
     * Constructor, initializes the serial port using remapped pins and disables
     * flow control.
     * 
     * NOTE: for stm32f2, f4, f7 and h7 you have to set the correct alternate
     * function to the pins in order to connect then to the USART peripheral
     * before passing them to this class.
     * 
     * Calls errorHandler(UNEXPECTED) if id is not in the correct range, or when
     * attempting to construct multiple objects with the same id. That is,
     * it is possible to instantiate only one instance of this class for each
     * hardware USART.
     * \param id a number 1 to 3 to select which USART
     * \param baudrate serial port baudrate
     * \param tx tx pin
     * \param rx rx pin
     */
    STM32Serial(int id, int baudrate, GpioPin tx, GpioPin rx);
    
    /**
     * Constructor, initializes the serial port using remapped pins and enables
     * flow control.
     * 
     * NOTE: for stm32f2, f4, f7 and h7 you have to set the correct alternate
     * function to the pins in order to connect then to the USART peripheral
     * before passing them to this class.
     * 
     * Calls errorHandler(UNEXPECTED) if id is not in the correct range, or when
     * attempting to construct multiple objects with the same id. That is,
     * it is possible to instantiate only one instance of this class for each
     * hardware USART.
     * \param id a number 1 to 3 to select which USART
     * \param tx tx pin
     * \param rx rx pin
     * \param rts rts pin
     * \param cts cts pin
     */
    STM32Serial(int id, int baudrate, GpioPin tx, GpioPin rx,
                GpioPin rts, GpioPin cts);
    
    /**
     * Read a block of data
     * \param buffer buffer where read data will be stored
     * \param size buffer size
     * \param where where to read from
     * \return number of bytes read or a negative number on failure. Note that
     * it is normal for this function to return less character than the amount
     * asked
     */
    ssize_t readBlock(void *buffer, size_t size, off_t where);
    
    /**
     * Write a block of data
     * \param buffer buffer where take data to write
     * \param size buffer size
     * \param where where to write to
     * \return number of bytes written or a negative number on failure
     */
    ssize_t writeBlock(const void *buffer, size_t size, off_t where);
    
    /**
     * Write a string.
     * An extension to the Device interface that adds a new member function,
     * which is used by the kernel on console devices to write debug information
     * before the kernel is started or in case of serious errors, right before
     * rebooting.
     * Can ONLY be called when the kernel is not yet started, paused or within
     * an interrupt. This default implementation ignores writes.
     * \param str the string to write. The string must be NUL terminated.
     */
    void IRQwrite(const char *str);
    
    /**
     * Performs device-specific operations
     * \param cmd specifies the operation to perform
     * \param arg optional argument that some operation require
     * \return the exact return value depends on CMD, -1 is returned on error
     */
    int ioctl(int cmd, void *arg);
    
    /**
     * \internal the serial port interrupts call this member function.
     * Never call this from user code.
     */
    void IRQhandleInterrupt();
    
    /**
     * \return port id, 1 for USART1, 2 for USART2, ... 
     */
    int getId() const { return portId; }
    
    /**
     * Destructor
     */
    ~STM32Serial();
    
private:
    /**
     * Code common for all constructors
     */
    void commonInit(int id, int baudrate, miosix::GpioPin tx, miosix::GpioPin rx,
                    miosix::GpioPin rts, miosix::GpioPin cts);
    
    /**
     * Wait until all characters have been written to the serial port.
     * Needs to be callable from interrupts disabled (it is used in IRQwrite)
     */
    void waitSerialTxFifoEmpty()
    {
        while((port->SR & USART_SR_TC)==0) ;
    }

    FastMutex txMutex;                ///< Mutex locked during transmission
    FastMutex rxMutex;                ///< Mutex locked during reception
    
    DynUnsyncQueue<char> rxQueue;     ///< Receiving queue
    static const unsigned int rxQueueMin=16; ///< Minimum queue size
    Thread *rxWaiting=0;              ///< Thread waiting for rx, or 0

    USART_TypeDef *port;              ///< Pointer to USART peripheral
    const STM32SerialHW *portHw;      ///< Pointer to USART port object
    bool idle=true;                   ///< Receiver idle
    const bool flowControl;           ///< True if flow control GPIOs enabled
    const unsigned char portId;       ///< 1 for USART1, 2 for USART2, ...
};

} //namespace miosix
