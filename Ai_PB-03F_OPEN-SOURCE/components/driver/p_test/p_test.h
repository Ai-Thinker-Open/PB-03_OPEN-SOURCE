#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "adc.h"

#include "log.h"
#include "mcu.h"
#include "i2c.h"
#include "pwm.h"
#include "timer.h"
#ifndef TEST_ADC
    #define TEST_ADC 1
#endif
#ifndef TEST_I2C
    #define TEST_I2C 1
#endif

#ifndef TEST_PWM
    #define TEST_PWM 1
#endif

#ifndef TEST_SPI
    #define  TEST_SPI 1
#endif

#ifndef TEST_TIMER
    #define TEST_TIMER 1
#endif

#ifndef TEST_WATCHDOG
    #define TEST_WATCHDOG 1
#endif

#ifndef TEST_GPIO
    #define TEST_GPIO 1
#endif



#define I2C_TXRX_LEN 7

#if (TEST_I2C == 1)

typedef struct
{
    gpio_pin_e sda_pin;
    gpio_pin_e scl_pin;
    I2C_CLOCK_e i2c_clock;
    i2c_dev_t i2c_mod;
    void* i2c01;
    uint16_t slave_addr;
    int8_t len;
    uint8_t tx_data[I2C_TXRX_LEN];
    uint8_t rx_data[I2C_TXRX_LEN];
    uint16_t reg_addr;
} i2c_cfg;
#endif

#if (TEST_PWM == 1)
typedef struct
{
    gpio_pin_e pwm_pin;
    PWM_CLK_DIV_e pwm_div;
    PWMN_e ch;
    uint16_t top;
    uint16_t cmp;
} pwm_cfg;
#endif

#if (TEST_TIMER == 1)
typedef struct
{
    User_Timer_e channel ;
    uint32_t us;
} timer_cfg;
#endif

#if (TEST_WATCHDOG == 1)

typedef struct
{
    uint8_t cycle;
} watchdog_cfg;

#endif


#if (TEST_GPIO == 1)
typedef struct
{
    gpio_pin_e pin;
} gpio_cfg;
#endif

uint16_t p_test_(uint8_t mod);
