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

        enum OpNum {
            SYS_OPEN = 0x01,
            SYS_CLOSE = 0x02,
            SYS_WRITEC = 0x03,
            SYS_WRITE0 = 0x04,
            SYS_WRITE = 0x05,
            SYS_READ = 0x06,
            SYS_READC = 0x07,
            SYS_ISERROR = 0x08,
            SYS_ISTTY = 0x09,
            SYS_SEEK = 0x0a,
            SYS_FLEN = 0x0c,
            SYS_TMPNAM = 0x0d,
            SYS_REMOVE = 0x0e,
            SYS_RENAME = 0x0f,
            SYS_CLOCK = 0x10,
            SYS_TIME = 0x11,
            SYS_SYSTEM = 0x12,
            SYS_ERRNO = 0x13,
            SYS_GET_CMDLINE = 0x15,
            SYS_HEAPINFO = 0x16,
            SYS_EXIT = 0x18,
            SYS_EXIT_EXTENDED = 0x20,
            SYS_ELAPSED = 0x30,
            SYS_TICKFREQ = 0x31,
        };

        struct Heapinfo {
            void *heap_base;
            void *heap_limit;
            void *stack_base;
            void *stack_limit;
        };

        int angelswi(int num2, int param2);

        int sys_open(const char *filename, Mode mode);
        int sys_close(int handle);
        void sys_writec(char inc);
        void sys_write0(const char *str);
        int sys_write(int handle, const void *buf, int len);
        int sys_read(int handle, void *buf, int len);
        int sys_readc();
        int sys_iserror(int code);
        int sys_istty(int handle);
        int sys_seek(int handle, int pos);
        int sys_flen(int handle);
        int sys_tmpnam(char *buf, int id, int len);
        int sys_remove(const char *filename);
        int sys_rename(const char *old_filename, const char *new_filename);
        int sys_clock();
        int sys_time();
        int sys_system(const char *cmd);
        int sys_errno();
        int sys_get_cmdline(char *buf, int *len);
        void sys_heapinfo(Heapinfo *heapinfo);
        void sys_exit(int reason);
        void sys_exit_extended(int reason, int code);
        int sys_elapsed(long long *ticks);
        int sys_tickfreq();

        int sys_open_stdout();
        int sys_open_stderr();
    }
}