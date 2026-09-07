/*
 * Linker script for stm32f767ZI
 * For compiling Miosix as a unikernel.
 * Allocates all RAM for the kernel and kernelspace application, no process pool
 */

MEMORY
{
    flash(rx)   : ORIGIN = 0x08000000, LENGTH = 2M
    ram1(wx)    : ORIGIN = 0x20000000, LENGTH = 128K
    ram2(wx)    : ORIGIN = 0x20020000, LENGTH = 384K
    sgm(rw)     : ORIGIN = 0x40024000, LENGTH = 4K /* safeguard memory (.preserve) */

}

INCLUDE ldscripts/miosix-flash-3ram-separate-heap-sgm.ld


