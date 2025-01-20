## `extern const`

In processes compiled with GCC 4.7.3:

- stuff in `.data` are accessed from the GOT, which is OK
- strings in `.rodata` are accessed PC-relative, which is OK
- consts in `.rodata` are accessed from the GOT, which is WRONG! (more on that later when pointer are involved, though)

Here's an example:

```cpp
extern const int aRodata;
extern int aData;
const char str[]="Hello world\n";

int get1() { return aRodata; }
int get2() { return aData; }
const char *get3() { return str; }
```

Compiled with `arm-miosix-eabi-gcc -mcpu=cortex-m3 -mthumb -mfix-cortex-m3-ldrd -fpie -msingle-pic-base -O2 -S processes.c`:

```asm
get1:
	ldr	r3, .L3
	ldr	r3, [r9, r3]
	ldr	r0, [r3]
	bx	lr

get2:
	ldr	r3, .L6
	ldr	r3, [r9, r3]
	ldr	r0, [r3]
	bx	lr


get3:
	ldr	r0, .L9
.LPIC0:
	add	r0, pc
	bx	lr
```

`get1()` is the wrong one, of course.

What usually masks the issue is constant folding. Unless it's an extern const the value gets folded and the problem does not arise.

Solution: The previous patch relied on a GCC function, `decl_readonly_section()` to check whether the global variable to be loaded is const or not, but that function missed a few cases, so a dedicated function, the `miosix_processes_ref_demux()` function was added in `arm.c`. This new function handles the corner cases better, and also allows to fix some of the next issues.
