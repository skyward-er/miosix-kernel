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

namespace miosix {

__attribute__((target("thumb")))
int angel::angelswi(int num2, int param2) {
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

int angel::sys_open(const char *filename, Mode mode) {
    struct {
        const char *filename;
        int mode;
        int filename_len;
    } params;

    params.filename = filename;
    params.mode = mode;
    params.filename_len = strlen(filename);
    return angelswi(SYS_OPEN, reinterpret_cast<int>(&params));
}

int angel::sys_close(int handle) {
    struct {
        int handle;
    } params;

    params.handle = handle;
    return angelswi(SYS_CLOSE, reinterpret_cast<int>(&params));
}

void angel::sys_writec(char c) {
    angelswi(SYS_WRITEC, reinterpret_cast<int>(&c));
}

void angel::sys_write0(const char *str) {
    angelswi(SYS_WRITE0, reinterpret_cast<int>(str));
}

int angel::sys_write(int handle, const void *buf, int len) {
    struct {
        int handle;
        const void *buf;
        int len;
    } params;

    params.handle = handle;
    params.buf = buf;
    params.len = len;

    return angelswi(SYS_WRITE, reinterpret_cast<int>(&params));
}

int angel::sys_read(int handle, void *buf, int len) {
    struct {
        int handle;
        void *buf;
        int len;
    } params;

    params.handle = handle;
    params.buf = buf;
    params.len = len;

    return angelswi(SYS_READ, reinterpret_cast<int>(&params));
}

int angel::sys_readc() {
    return angelswi(SYS_READC, 0);
}

int angel::sys_iserror(int code) {
    struct {
        int code;
    } params;

    params.code = code;

    return angelswi(SYS_ISERROR, reinterpret_cast<int>(&params));
}

int angel::sys_istty(int handle) {
    struct {
        int handle;
    } params;

    params.handle = handle;

    return angelswi(SYS_ISTTY, reinterpret_cast<int>(&params));
}

int angel::sys_seek(int handle, int pos) {
    struct {
        int handle;
        int pos;
    } params;

    params.handle = handle;
    params.pos = pos;

    return angelswi(SYS_SEEK, reinterpret_cast<int>(&params));
}

int angel::sys_flen(int handle) {
    struct {
        int handle;
    } params;

    params.handle = handle;
    return angelswi(SYS_FLEN, reinterpret_cast<int>(&params));
}

int angel::sys_tmpnam(char *buf, int id, int len) {
    struct {
        char *buf;
        int id;
        int len;
    } params;

    params.buf = buf;
    params.id = id;
    params.len = len;
    return angelswi(SYS_TMPNAM, reinterpret_cast<int>(&params));
}

int angel::sys_remove(const char *filename) {
    struct {
        const char *filename;
        int filename_len;
    } params;

    params.filename = filename;
    params.filename_len = strlen(filename);
    return angelswi(SYS_REMOVE, reinterpret_cast<int>(&params));
}

int angel::sys_rename(const char *old_filename, const char *new_filename) {
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
    return angelswi(SYS_RENAME, reinterpret_cast<int>(&params));
}

int angel::sys_clock() {
    return angelswi(SYS_CLOCK, 0);
}

int angel::sys_time() {
    return angelswi(SYS_TIME, 0);
}

int angel::sys_system(const char *cmd) {
    struct {
        const char *cmd;
        int cmd_len;
    } params;

    params.cmd = cmd;
    params.cmd_len = strlen(cmd);
    return angelswi(SYS_SYSTEM, reinterpret_cast<int>(&params));
}

int angel::sys_errno() {
    return angelswi(SYS_ERRNO, 0);
}

int angel::sys_get_cmdline(char *buf, int *len) {
    struct {
        char *buf;
        int len;
    } params;

    params.buf = buf;
    params.len = *len;

    int ret = angelswi(SYS_GET_CMDLINE, reinterpret_cast<int>(&params));
    *len = params.len;

    return ret;
}

void angel::sys_heapinfo(Heapinfo *heapinfo) {
    angelswi(SYS_HEAPINFO, reinterpret_cast<int>(heapinfo));
}

void angel::sys_exit(int reason) {
    angelswi(SYS_EXIT, reason);
}

void angel::sys_exit_extended(int reason, int code) {
    struct {
        int reason;
        int code;
    } params;

    params.reason = reason;
    params.code = code;
    angelswi(SYS_EXIT_EXTENDED, reinterpret_cast<int>(&params));
}

int angel::sys_elapsed(long long *ticks) {
    struct {
        int low;
        int high;
    } params;

    int ret = angelswi(SYS_ELAPSED, reinterpret_cast<int>(&params));
    *ticks = (long long)params.low | ((long long) params.high << 32);

    return ret;
}

int angel::sys_tickfreq() {
    return angelswi(SYS_TICKFREQ, 0);
}

int angel::sys_open_stdout() {
    return sys_open(":tt", MODE_W);
}

int angel::sys_open_stderr() {
    return sys_open(":tt", MODE_A);
}

}