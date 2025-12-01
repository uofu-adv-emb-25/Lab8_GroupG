# This is the main binary. List your C files here.
add_executable(CANTransmit
    CANTransmit.c
    )

pico_set_program_name(CANTransmit "test")
pico_set_program_version(CANTransmit "0.1")

# Modify the below lines to enable/disable output over UART/USB.
# These are mutually exclusive, only set one.
pico_enable_stdio_uart(CANTransmit 0)
pico_enable_stdio_usb(CANTransmit 1)

# Add the libraries to the build
target_link_libraries(CANTransmit PRIVATE
  pico_stdlib
  pico_multicore
  pico_async_context_freertos
  can2040
  FreeRTOS-Kernel-Heap4
)

# The CYW43 is the Wifi/Bluetooth module. If the board is set to pico_w, this
# variable will be set and the wireless libraries added.
if(PICO_CYW43_SUPPORTED)
    # For led support on pico_w
    target_link_libraries(CANTransmit PRIVATE
        pico_cyw43_arch_none
    )
endif()

# Another pico macro which generates the uf2 file in addition to the elf
pico_add_extra_outputs(CANTransmit)

# ---------------------------------------------- 

# This is the main binary. List your C files here.
add_executable(CANReceive
    CANReceive.c
    )

pico_set_program_name(CANReceive "test")
pico_set_program_version(CANReceive "0.1")

# Modify the below lines to enable/disable output over UART/USB.
# These are mutually exclusive, only set one.
pico_enable_stdio_uart(CANReceive 0)
pico_enable_stdio_usb(CANReceive 1)

# Add the libraries to the build
target_link_libraries(CANReceive PRIVATE
  pico_stdlib
  pico_multicore
  pico_async_context_freertos
  can2040
  FreeRTOS-Kernel-Heap4
)

# The CYW43 is the Wifi/Bluetooth module. If the board is set to pico_w, this
# variable will be set and the wireless libraries added.
if(PICO_CYW43_SUPPORTED)
    # For led support on pico_w
    target_link_libraries(CANReceive PRIVATE
        pico_cyw43_arch_none
    )
endif()

# Another pico macro which generates the uf2 file in addition to the elf
pico_add_extra_outputs(CANReceive)

# ---------------------------------------------- 

# This is the combined code.
add_executable(CANCombine
    CANCombine.c
    )

pico_set_program_name(CANCombine "test")
pico_set_program_version(CANCombine "0.1")

# Modify the below lines to enable/disable output over UART/USB.
# These are mutually exclusive, only set one.
pico_enable_stdio_uart(CANCombine 0)
pico_enable_stdio_usb(CANCombine 1)

# Add the libraries to the build
target_link_libraries(CANCombine PRIVATE
  pico_stdlib
  pico_multicore
  pico_async_context_freertos
  can2040
  FreeRTOS-Kernel-Heap4
)

# The CYW43 is the Wifi/Bluetooth module. If the board is set to pico_w, this
# variable will be set and the wireless libraries added.
if(PICO_CYW43_SUPPORTED)
    # For led support on pico_w
    target_link_libraries(CANCombine PRIVATE
        pico_cyw43_arch_none
    )
endif()

# Another pico macro which generates the uf2 file in addition to the elf
pico_add_extra_outputs(CANCombine)
