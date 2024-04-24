
cmake_minimum_required(VERSION 3.1)
project(FeedForwardProfiler)

## Targets
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(SRCS jtag_profiler.cpp)
add_executable(jtag_profiler ${SRCS})
