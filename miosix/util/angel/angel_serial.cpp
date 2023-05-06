/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "angel_serial.h"

#include <termios.h>

#include "angel.h"
#include "filesystem/ioctl.h"

using namespace miosix;

miosix::AngelSerial::AngelSerial() : Device(DeviceType::TTY), handle(angel::sys_open_stdout()) {}

ssize_t miosix::AngelSerial::readBlock(void *buffer, size_t size, off_t where) {
    return size - angel::sys_read(handle, buffer, size);
}

ssize_t miosix::AngelSerial::writeBlock(const void *buffer, size_t size, off_t where) {
    return size - angel::sys_write(handle, buffer, size);
}

int miosix::AngelSerial::ioctl(int cmd, void *arg) {
    if(reinterpret_cast<unsigned>(arg) & 0b11) 
        return -EFAULT; // Unaligned
    
    termios *t = reinterpret_cast<termios*>(arg);

    switch(cmd)
    {
        case IOCTL_SYNC:
            return 0;
        case IOCTL_TCGETATTR:
            t->c_iflag = IGNBRK | IGNPAR;
            t->c_oflag = 0;
            t->c_cflag = CS8;
            t->c_lflag = 0;
            return 0;
        case IOCTL_TCSETATTR_NOW:
        case IOCTL_TCSETATTR_DRAIN:
        case IOCTL_TCSETATTR_FLUSH:
            // Changing things at runtime unsupported, so do nothing, but don't
            // return error as console_device.h implements some attribute changes
            return 0;
        default:
            return -ENOTTY; // Means the operation does not apply to this descriptor
    }
}
