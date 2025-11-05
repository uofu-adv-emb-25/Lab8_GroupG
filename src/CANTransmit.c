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

#include <can2040.h>
#include <queue.h>
#include "fifo.h"

int count = 0;
static struct can2040 cbus;
QueueHandle_t msgs;

#define TRANSMIT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2UL )
#define TRANSMIT_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define DELAY_MS 1000

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify == CAN2040_NOTIFY_RX) {
        xQueueSendToBack(msgs, *msg, portMAX_DELAY); 
    } else if (notify == CAN2040_NOTIFY_TX) {

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
    uint32_t gpio_rx = 4, gpio_tx = 5;

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

void transmit_task(__unused void *params) {
    struct can2040_msg msg = {0, 0, {1, 2}};

    while (true) {
        if (can2040_check_transmit(cbus))
            can2040_transmit(&cbus, &msg);
        sleep_ms(DELAY_MS);
    }
}

int main( void )
{
    stdio_init_all();
    const char *rtos_name;
    rtos_name = "FreeRTOS";

    msgs = xQueueCreate(100, sizeof(struct can2040_msg));
    canbus_setup();
    
    TaskHandle_t task;
    xTaskCreate(transmit_task, "TransmitThread",
                TRANSMIT_TASK_STACK_SIZE, NULL, TRANSMIT_TASK_PRIORITY, &task);

    vTaskStartScheduler();
    return 0;
}
