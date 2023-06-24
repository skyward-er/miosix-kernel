Step response test for control scheduler
========================================

Required tools:
Miosix compiler set up, a jtag adapter, a board running Miosix,
openocd, scilab, CMake, boost libraries

1) Select Miosix OPT_OPTIMIZATION := -O0 by building in debug mode
to have precise breakpoints

2) Build target by running `./mbs -d -b feedforward`

3) Build jtag_profiler and copy main.elf, jtag_profiler and
gdb_init.script to same folder

4) Start openocd

5) Run
```
./jtag_profiler > ff_on.txt
With Miosix compiled with feedforward on and
./jtag_profiler > ff_off.txt
With Miosix compiled with feedforward off
```

6) run ./plot.sh to see results
