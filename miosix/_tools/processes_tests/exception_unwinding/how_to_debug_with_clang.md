Compile and flash with:

```shell
cmake -DCMAKE_BUILD_TYPE=Debug -Bbuild -DCMAKE_TOOLCHAIN_FILE=../../../cmake/Toolchains/clang.cmake
cmake --build build -j 16 --target main_program
```

Run OpenOCD:

```shell
openocd -f /home/alberton/Documents/CTO/miosix-kernel/miosix/arch/cortexM4_stm32f4/stm32f429zi_stm32f4discovery/stm32f4discovery.cfg
```

Run `gdb`:

```shell
gdb build/main.elf
(gdb) target remote :3333
```

`monitor reset halt` to reset the board.
