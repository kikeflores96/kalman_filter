# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/kike/Documents/Droneproject/RaspberrypicoCpp/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
project(kalman_filter C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(kalman_filter kalman_filter.cpp)

pico_set_program_name(kalman_filter "kalman_filter")
pico_set_program_version(kalman_filter "0.1")

pico_enable_stdio_uart(kalman_filter 1)
pico_enable_stdio_usb(kalman_filter 0)


# Add the standard library to the build
target_link_libraries(kalman_filter pico_stdlib)


# Add any user requested libraries
target_link_libraries(kalman_filter
        hardware_spi
        hardware_i2c
        imu
        attitude
        )
#target_link_libraries(kalman_filter  imu)

pico_add_extra_outputs(kalman_filter)

add_subdirectory(lib)



target_link_libraries (kalman_filter Eigen3::Eigen)



