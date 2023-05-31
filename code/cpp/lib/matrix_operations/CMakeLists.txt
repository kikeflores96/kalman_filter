
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
# Adds a library target called nrf24_driver for the NRF24L01 driver
# STATIC libraries are archives of object files for use when linking other targets
# INTERFACE libraries do not compile sources and don't produce a library artifact on disk
add_library(matrix_operations INTERFACE)

# PRIVATE means these sources should only be added to nrf24_driver
# PUBLIC wuould mean these sources should be added to nrf24_driver and to any 
# target that links to nrf24_driver library. 
target_sources(matrix_operations
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}/matrix_operations.cpp
)


# Link nrf24_driver against pico-sdk;
# pico_stdlib, hardware_spi & hardware_gpio libraries
target_link_libraries(matrix_operations 
    INTERFACE
    pico_stdlib
)