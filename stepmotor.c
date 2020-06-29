#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "stepmotor.h"

#define STEPMOTOR_INIT_ERR_STR                  "step motor init error"
#define STEPMOTOR_SET_DIR_ERR_STR               "step motor set direction error"
#define STEPMOTOR_TOGGLE_DIR_ERR_STR            "step motor toggle direction error"
#define STEPMOTOR_SET_PWM_FREQ_ERR_STR          "step motor set pwm frequency error"
#define STEPMOTOR_SET_PWM_DUTYCYCLE_ERR_STR     "step motor set pwm duty cycle error"
#define STEPMOTOR_START_ERR_STR                 "step motor start error"
#define STEPMOTOR_STOP_ERR_STR                  "step motor stop error"

#define mutex_lock(x)                           while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)                         xSemaphoreGive(x)
#define mutex_create()                          xSemaphoreCreateMutex()
#define mutex_destroy(x)                        vQueueDelete(x)

static const char* STEPMOTOR_TAG = "STEP MOTOR";
#define STEPMOTOR_CHECK(a, str, ret)  if(!(a)) {                                            \
        STM_LOGE(STEPMOTOR_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);     \
        return (ret);                                                                       \
        }

typedef struct stepmotor {
    gpio_port_t         dir_gpio_port;          /*!< Pin dir GPIO Port */
    gpio_num_t          dir_gpio_num;           /*!< Pin dir GPIO Num */
    timer_num_t         pulse_timer_num;        /*!< Pin pulse Timer Num */
    timer_pins_pack_t   pulse_timer_pins_pack;  /*!< Pin pulse Timer Pins Pack */
    timer_chnl_t        pulse_timer_chnl;       /*!< Pin pulse Timer Channel */
    bool                dir;                    /*!< Direction */
    uint32_t            freq_hz;                /*!< PWM frequency in Hz */
    uint8_t             duty;                   /*!< PWM duty cycle in % */
    SemaphoreHandle_t   lock;                   /*!< Step motor mutex */
} stepmotor_t;

stepmotor_handle_t stepmotor_config(stepmotor_config_t *config)
{
    /* Check input conditions */
    STEPMOTOR_CHECK(config, STEPMOTOR_INIT_ERR_STR, NULL);

    /* Allocate memory for handle structure */
    stepmotor_handle_t handle = calloc(1, sizeof(stepmotor_t));
    STEPMOTOR_CHECK(handle, STEPMOTOR_INIT_ERR_STR, NULL);

    /* Configure pin direction */
    gpio_cfg_t dir_cfg;
    dir_cfg.gpio_port = config->dir_gpio_port;
    dir_cfg.gpio_num = config->dir_gpio_num;
    dir_cfg.mode = GPIO_OUTPUT_PP;
    dir_cfg.reg_pull_mode = GPIO_REG_PULL_NONE;
    STEPMOTOR_CHECK(!gpio_config(&dir_cfg), STEPMOTOR_INIT_ERR_STR, NULL);
    STEPMOTOR_CHECK(!gpio_set_level(config->dir_gpio_port, config->dir_gpio_num, 0), STEPMOTOR_INIT_ERR_STR, NULL);

    /* Configure pin pulse */
    pwm_cfg_t pulse_cfg;
    pulse_cfg.timer_num = config->pulse_timer_num;
    pulse_cfg.timer_pins_pack = config->pulse_timer_pins_pack;
    pulse_cfg.timer_chnl = config->pulse_timer_chnl;
    STEPMOTOR_CHECK(!pwm_config(&pulse_cfg), STEPMOTOR_INIT_ERR_STR, NULL);
    STEPMOTOR_CHECK(!pwm_set_params(config->pulse_timer_num, config->pulse_timer_chnl, 0, 50), STEPMOTOR_INIT_ERR_STR, NULL);

    /* Update handle structure */
    handle->dir_gpio_port = config->dir_gpio_port;
    handle->dir_gpio_num = config->dir_gpio_num;
    handle->pulse_timer_num = config->pulse_timer_num;
    handle->pulse_timer_pins_pack = config->pulse_timer_pins_pack;
    handle->pulse_timer_chnl = config->pulse_timer_chnl;
    handle->dir = 0;
    handle->freq_hz = 0;
    handle->duty = 50;
    handle->lock = mutex_create();

    return handle;
}

stm_err_t stepmotor_set_dir(stepmotor_handle_t handle, bool dir)
{
    mutex_lock(handle->lock);

    int ret;
    ret = gpio_set_level(handle->dir_gpio_port, handle->dir_gpio_num, dir);
    if (ret) {
        STM_LOGE(STEPMOTOR_TAG, STEPMOTOR_SET_DIR_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    handle->dir = dir;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_toggle_dir(stepmotor_handle_t handle)
{
    mutex_lock(handle->lock);

    int ret;
    ret = gpio_toggle_level(handle->dir_gpio_port, handle->dir_gpio_num);
    if (ret) {
        STM_LOGE(STEPMOTOR_TAG, STEPMOTOR_TOGGLE_DIR_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    handle->dir = !handle->dir;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_set_pwm_freq(stepmotor_handle_t handle, uint32_t freq_hz)
{
    mutex_lock(handle->lock);
    int ret;

    ret = pwm_set_frequency(handle->pulse_timer_num, handle->pulse_timer_chnl, freq_hz);
    if (ret) {
        STM_LOGE(STEPMOTOR_TAG, STEPMOTOR_SET_PWM_FREQ_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    ret = pwm_set_duty(handle->pulse_timer_num, handle->pulse_timer_chnl, handle->duty);
    if (ret) {
        STM_LOGE(STEPMOTOR_TAG, STEPMOTOR_SET_PWM_FREQ_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    handle->freq_hz = freq_hz;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_set_pwm_duty(stepmotor_handle_t handle, uint8_t duty)
{
    mutex_lock(handle->lock);

    int ret;
    ret = pwm_set_duty(handle->pulse_timer_num, handle->pulse_timer_chnl, duty);
    if (ret) {
        STM_LOGE(STEPMOTOR_TAG, STEPMOTOR_SET_PWM_DUTYCYCLE_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    handle->duty = duty;
    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_start(stepmotor_handle_t handle)
{
    mutex_lock(handle->lock);

    int ret;
    ret = pwm_start(handle->pulse_timer_num, handle->pulse_timer_chnl);
    if (ret) {
        STM_LOGE(STEPMOTOR_TAG, STEPMOTOR_START_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    mutex_unlock(handle->lock);
    return STM_OK;
}

stm_err_t stepmotor_stop(stepmotor_handle_t handle)
{
    mutex_lock(handle->lock);

    int ret;
    ret = pwm_stop(handle->pulse_timer_num, handle->pulse_timer_chnl);
    if (ret) {
        STM_LOGE(STEPMOTOR_TAG, STEPMOTOR_STOP_ERR_STR);
        mutex_unlock(handle->lock);
        return STM_FAIL;
    }

    mutex_unlock(handle->lock);
    return STM_OK;
}