// MIT License

// Copyright (c) 2020 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm_err.h"
#include "stm_log.h"

#include "stepmotor.h"

/* 
 * This example shows how to control step motor driver .
 *      - Pin direction: PD13.
 *      - Pin pulse: PD12.
 */

#define TASK_SIZE   1024
#define TASK_PRIOR  5

#define DIR_GPIO_PORT           GPIO_PORT_D
#define DIR_GPIO_NUM            GPIO_NUM_13
#define PULSE_TIMER_NUM         TIMER_NUM_4
#define PULSE_TIMER_PINS_PACK   TIMER_PINS_PACK_2
#define PULSE_TIMER_CHANNEL     TIMER_CHNL_1

/* Handle structure */
stepmotor_handle_t stepmotor_handle;

static const char *TAG = "APP_MAIN";

static void example_task(void* arg)
{
    /* Configure step motor */
    stepmotor_config_t stepmotor_cfg;
    stepmotor_cfg.dir_gpio_port = DIR_GPIO_PORT;
    stepmotor_cfg.dir_gpio_num = DIR_GPIO_NUM;
    stepmotor_cfg.pulse_timer_num = PULSE_TIMER_NUM;
    stepmotor_cfg.pulse_timer_pins_pack = PULSE_TIMER_PINS_PACK;
    stepmotor_cfg.pulse_timer_chnl = PULSE_TIMER_CHANNEL;
    stepmotor_handle = stepmotor_config(&stepmotor_cfg);

    /* Set step driver parameter */
    stepmotor_set_pwm_duty(stepmotor_handle, 50);
    stepmotor_set_pwm_freq(stepmotor_handle, 800);
    stepmotor_set_dir(stepmotor_handle, 0);

    /* Start motor */
    stepmotor_start(stepmotor_handle);

    while(1)
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        stepmotor_stop(stepmotor_handle);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        stepmotor_start(stepmotor_handle);
    }
}

int main(void)
{
    /* Set log output level */
    stm_log_level_set("*", STM_LOG_NONE);
    stm_log_level_set("APP_MAIN", STM_LOG_INFO);

    /* Create task */
    xTaskCreate(example_task, "example_task", TASK_SIZE, NULL, TASK_PRIOR, NULL);

    /* Start RTOS scheduler */
    vTaskStartScheduler();
}