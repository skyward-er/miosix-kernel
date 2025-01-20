# When compiling with `clang`

`__cxa_throw` gets called with:

```
__cxxabiv1::__cxa_throw (obj=0xd0002348, tinfo=0x801fa84 <typeinfo for int>, dest=0x0)
```

These 3 parameters are (from [The Secret Life of C++: Day 3: Exceptions](https://www.mit.edu/~sipb-iap/2004/inside-c/day3/exceptions.html)):

- An exception object
- A typeinfo for that object
- A pointer to the destructor to call when we are done with that object.

They are correct because:

- The exception object address is returned from `__cxa_allocate_exception`, and by reading that address from memory the correct value (1) is found
- The exception object is an `int`
- There is no constructor

`__cxa_throw` is defined in `miosix/_tools/compiler/gcc-9.2.0-mp3.2/gcc-9.2.0/libstdc++-v3/libsupc++/eh_throw.cc:75`.

`__cxa_throw` calls `_Unwind_RaiseException` from `gcc-9.2.0/libgcc/config/arm/libunwind.S:356` but this is just a wrapper for `__gnu_Unwind_RaiseException` in `gcc-9.2.0/libgcc/unwind-arm-common.inc:429`.

```
__gnu_Unwind_RaiseException (ucbp=0xd00022f0, entry_vrs=0xd0001a74)
```

Where the parameters are:

- `ucbp` is the pointer to the unwind control block, which contains information about the exception being raised. It is used to manage the state of the exception.
- `entry_vrs` is the pointer to a structure that contains the virtual register set for pahse 2 of the unsinding process. This holds the state of the registers at the point where the exception was raised and is used to restore the state when transferring control to a catch block or cleanup code.

In `__gnu_Unwind_RaiseException` the personality routing gets called through `UCB_PR_ADDR`.
