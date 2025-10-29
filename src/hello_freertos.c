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

int count = 0;
bool on = false;
static struct can2040 cbus;

#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 1UL )
#define TRANSMIT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2UL )
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define TRANSMIT_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

void transmit_task(__unused void *params) {
    struct can2040_msg msg;

    hard_assert(cyw43_arch_init() == PICO_OK);
    while (true) {
        if (can2040_check_transmit(cbus))
            can2040_transmit(cbus, &msg);
    
    }
}


int main( void )
{
    stdio_init_all();
    const char *rtos_name;
    rtos_name = "FreeRTOS";

    
    
    TaskHandle_t task;
    xTaskCreate(transmit_task, "TransmitThread",
                TRANSMIT_TASK_STACK_SIZE, NULL, TRANSMIT_TASK_PRIORITY, &task);

    vTaskStartScheduler();
    return 0;
}
