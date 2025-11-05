set(CAN_PATH "../can2040")
include("../can2040/can2040.cmake")

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

find_program(OPENOCD openocd)
find_program(PICOTOOL picotool)

add_custom_target(flash ${PICOTOOL} load -f $<TARGET_FILE:CANTransmit> DEPENDS CANTransmit)

add_custom_target(openocd_flash ${OPENOCD} -s ${OPENOCD_PATH} -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c 'adapter speed 5000; program "$<TARGET_FILE:CANTransmit>" verify reset exit' DEPENDS CANTransmit)

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

find_program(OPENOCD openocd)
find_program(PICOTOOL picotool)

add_custom_target(flash ${PICOTOOL} load -f $<TARGET_FILE:CANReceive> DEPENDS CANReceive)

add_custom_target(openocd_flash ${OPENOCD} -s ${OPENOCD_PATH} -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c 'adapter speed 5000; program "$<TARGET_FILE:CANReceive>" verify reset exit' DEPENDS CANReceive)
