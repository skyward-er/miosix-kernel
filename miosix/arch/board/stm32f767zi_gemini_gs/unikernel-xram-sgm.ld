/*
 * Linker script for stm32f767ZI
 * For compiling Miosix as a unikernel.
 * Allocates all RAM for the kernel and kernelspace application, no process pool
 */


MEMORY
{
    flash(rx)   : ORIGIN = 0x08000000, LENGTH = 2M
    ram1(wx)    : ORIGIN = 0x20020000, LENGTH = 384K
    ram2(wx)    : ORIGIN = 0xd0000000, LENGTH = 16M  /* XRAM */
    sgm(rw)     : ORIGIN = 0x40024000, LENGTH = 4K /* safeguard memory (.preserve) */
}

/* Mark ram2 as being an external RAM */
_xram_start = ORIGIN(ram2);
_xram_size  = LENGTH(ram2);

INCLUDE ldscripts/miosix-flash-3ram-separate-heap-sgm.ld
