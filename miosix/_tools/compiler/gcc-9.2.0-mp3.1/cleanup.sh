#!/bin/bash

# After running install-script.sh, this script will clean up temporary files.
# It will not remove the files created by the download.sh script, so that
# running install-script.sh is possible without re-downloading them.

rm -rf binutils-2.32 gcc-9.2.0 gdb-8.3 newlib-3.1.0 newlib-obj \
	gmp-6.1.2 mpfr-4.0.2 mpc-1.1.0 make-4.0 ncurses-5.9 makeself-2.1.5 \
	lib quickfix lpc21isp.c

rm -rf objdir/ log/
if [[ $? -ne 0 ]]; then
    sudo rm -rf objdir/ log/
fi
