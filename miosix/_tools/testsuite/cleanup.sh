#!/bin/bash

rm mpu_testsuite/includes.h

cd mpu_testsuite
for i in *;
do
	if test -d $i; then
		rm "$i.h"
		cd $i && make clean && cd ..
	fi;
done;
cd ..

cd syscall_testsuite
for i in *;
do
	if test -d $i; then
		rm "$i.h"
		cd $i && make clean && cd ..
	fi;
done;
cd ..