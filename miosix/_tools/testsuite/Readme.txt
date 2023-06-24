To run the testsuite:
1) Define WITH_PROCESSES in miosix_settings.h and select a supported board
2) Run `./build.sh` in this directory
3) Run `./mbs -b testsuite`

You can skip points 1 and 2 if you will not test the MPU

Run './cleanup.sh` in this directory to clean compilation files.

For more information, see the Readme.txt files in the subdirectories.
