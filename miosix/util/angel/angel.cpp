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

#include "angel.h"

#include <cstring>

using namespace miosix::angel;

__attribute__((target("thumb")))
int miosix::angel::angelswi(int num2, int param2) {
    register int num asm("r0") = num2;
    register int param asm("r1") = param2;
    register int ret asm("r0");
    
    asm(
        "bkpt 0xAB"
        : "=r" (ret)
        : "r" (num), "r" (param)
        : "memory"
    );

    return ret;
}

int miosix::angel::sys_clock() {
    return angelswi(0x10, 0);
}

int miosix::angel::sys_close(int handle) {
    struct {
        int handle;
    } params;

    params.handle = handle;
    return angelswi(0x02, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_errno() {
    return angelswi(0x13, 0);
}

int miosix::angel::sys_flen(int handle) {
    struct {
        int handle;
    } params;

    params.handle = handle;
    return angelswi(0x0c, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_istty(int handle) {
    struct {
        int handle;
    } params;

    params.handle = handle;
    return angelswi(0x09, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_open(const char *filename, Mode mode) {
    struct {
        const char *filename;
        int mode;
        int filename_len;
    } params;

    params.filename = filename;
    params.mode = mode;
    params.filename_len = strlen(filename);

    return angelswi(0x01, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_read(int handle, void *buf, int len) {
    struct {
        int handle;
        void *buf;
        int len;
    } params;

    params.handle = handle;
    params.buf = buf;
    params.len = len;

    return angelswi(0x06, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_write(int handle, const void *buf, int len) {
    struct {
        int handle;
        const void *buf;
        int len;
    } params;

    params.handle = handle;
    params.buf = buf;
    params.len = len;

    return angelswi(0x05, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_seek(int handle, int pos) {
    struct {
        int handle;
        int pos;
    } params;

    params.handle = handle;
    params.pos = pos;

    return angelswi(0x0a, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_remove(const char *filename) {
    struct {
        const char *filename;
        int filename_len;
    } params;

    params.filename = filename;
    params.filename_len = strlen(filename);

    return angelswi(0x0e, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_rename(const char *old_filename, const char *new_filename) {
    struct {
        const char *old_filename;
        int old_filename_len;
        const char *new_filename;
        int new_filename_len;
    } params;

    params.old_filename = old_filename;
    params.old_filename_len = strlen(old_filename);
    params.new_filename = new_filename;
    params.new_filename_len = strlen(new_filename);

    return angelswi(0x0f, reinterpret_cast<int>(&params));
}

int miosix::angel::sys_open_stdout() {
    return sys_open(":tt", MODE_W);
}

int miosix::angel::sys_open_stderr() {
    return sys_open(":tt", MODE_A);
}