
# Miosix for Skyward

This is a fork of the original fedetft/miosix-kernel repo made for Skyward Experimental Rocketry. Every Skyward-related stuff, like new boards or experimental features should be pushed here.

**IMPORTANT:** when you modify this repo, remember to update the submodule in the repo that includes it.

You can find information on how to configure and use the kernel
at the following url: http://miosix.org


### Main Differences

A part from additional boards, there are some important differences with the original kernel version:

1. `miosix/config/Makefile.inc`: The version of the makefile must be `1.07S`, to distinguish this kernel from the official one

```
ifneq ($(MAKEFILE_VERSION),1.07S)
    $(info You are using an incompatible makefile. Make sure it matches \
      the one distributed with the current version of the kernel)
    $(error Error)
endif
```

2. `miosix/config/miosix`: The usual `error` is commented out and `JTAG` enabled by default

Note that these changes must be respected when pulling from the original repo


### Pulling from Original Repo

If you wish to pull from the original kernel repo you should:

```
git pull https://github.com/fedetft/miosix-kernel.git testing
```

Then merge (keeping the 1.07S check in the Makefiles), commit and push.

Note that the testing branch is were normally the latest updates
are made for the Skyward guys.
