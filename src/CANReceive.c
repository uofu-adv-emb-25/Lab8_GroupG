/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"

#include <../can2040/src/can2040.h>
#include <queue.h>

int count = 0;
static struct can2040 cbus;
QueueHandle_t msgs;

#define RECEIVE_TASK_PRIORITY      ( tskIDLE_PRIORITY + 1UL )
#define RECEIVE_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify == CAN2040_NOTIFY_RX) {
        xQueueSendToBack(msgs, msg, portMAX_DELAY); 
    }
}

static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 16, gpio_tx = 17;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, PICO_DEFAULT_IRQ_PRIORITY - 1);
    irq_set_enabled(PIO0_IRQ_0, 1);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void receive_task(__unused void *params) {
    struct can2040_msg msg;

    printf("Receiving\n");
    while (true) {
        xQueueReceive(msgs, &msg, portMAX_DELAY);
        printf("MSG data: %d\n", msg.data32[0]);
    }
}

int main( void )
{
    stdio_init_all();
    const char *rtos_name;
    rtos_name = "FreeRTOS";

    sleep_ms(5000);

    msgs = xQueueCreate(100, sizeof(struct can2040_msg));
    canbus_setup();

    printf("Receive Setup Canbus\n");

    TaskHandle_t task;
    xTaskCreate(receive_task, "ReceiveThread",
                RECEIVE_TASK_STACK_SIZE, NULL, RECEIVE_TASK_PRIORITY, &task);

    vTaskStartScheduler();
    return 0;
}
