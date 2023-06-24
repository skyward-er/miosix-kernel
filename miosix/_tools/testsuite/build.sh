#!/bin/bash

cd mpu_testsuite
./build.sh
cd ..

cd syscall_testsuite
./build.sh testsuite
cd ..
