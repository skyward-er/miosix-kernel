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

#pragma once

namespace miosix {
    namespace angel {
        enum Mode {
            MODE_R = 0,
            MODE_RB = 1,
            MODE_RP = 2,
            MODE_RPB = 3,
            MODE_W = 4,
            MODE_WB = 5,
            MODE_WP = 6,
            MODE_WPB = 7,
            MODE_A = 8,
            MODE_AB = 9,
            MODE_AP = 10,
            MODE_APB = 11
        };

        int angelswi(int num2, int param2);

        int sys_clock();
        int sys_close(int handle);
        int sys_errno();
        int sys_flen(int handle);
        int sys_istty(int handle);
        int sys_open(const char *filename, Mode mode);
        int sys_read(int handle, void *buf, int len);
        int sys_write(int handle, const void *buf, int len);
        int sys_seek(int handle, int pos);
        int sys_remove(const char *filename);
        int sys_rename(const char *old_filename, const char *new_filename);
    
        int sys_open_stdout();
        int sys_open_stderr();
    }
}