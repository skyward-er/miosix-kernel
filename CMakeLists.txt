cmake_minimum_required(VERSION 3.16)
project(MiosixProject C CXX ASM)

set(KPATH miosix CACHE PATH "Path to kernel directory")
set(OPT_BOARD "" CACHE STRING "Target board")
if(NOT OPT_BOARD)
    message(FATAL_ERROR "No board selected")
endif()

add_subdirectory(${KPATH} EXCLUDE_FROM_ALL)

add_executable(main main.cpp)
target_link_libraries(main miosix::miosix-${OPT_BOARD})
add_custom_command(
    TARGET main POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex main main.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary main main.bin
    BYPRODUCTS main.hex main.bin
)
