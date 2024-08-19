#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "log.h"
#include "mcu.h"
#include "p_test.h"



#if (TEST_ADC == 1)
#include "adc.h"
#define MAX_SAMPLE_POINT    64
uint16_t adc_debug[6][MAX_SAMPLE_POINT];
static uint8_t channel_done_flag = 0;


#define ADC_CHANEL_SELE ADC_CH3P_P20
adc_Cfg_t adc_cfg =
{


    .channel = ADC_BIT(ADC_CHANEL_SELE),

    .is_continue_mode = FALSE,
    .is_differential_mode = 0x00,

    .is_high_resolution = 0x00,

};
static void p_adc_evt(adc_Evt_t* pev)
{
    float value = 0;
    int i = 0;
    bool is_high_resolution = FALSE;
    bool is_differential_mode = FALSE;
    uint8_t ch = 0;

    if((pev->type != HAL_ADC_EVT_DATA) || (pev->ch < 2))
        return;

    osal_memcpy(adc_debug[pev->ch-2],pev->data,2*(pev->size));
    channel_done_flag |= BIT(pev->ch);

    if(channel_done_flag == adc_cfg.channel)
    {
        for(i=2; i<8; i++)
        {
            if(channel_done_flag & BIT(i))
            {
                is_high_resolution = (adc_cfg.is_high_resolution & BIT(i))?TRUE:FALSE;
                is_differential_mode = (adc_cfg.is_differential_mode & BIT(i))?TRUE:FALSE;
                value = hal_adc_value_cal((adc_CH_t)i,adc_debug[i-2], pev->size, is_high_resolution,is_differential_mode);

                switch(i)
                {
                case ADC_CH1N_P11:
                    ch=11;
                    break;

                case ADC_CH1P_P23:
                    ch=23;
                    break;

                case ADC_CH2N_P24:
                    ch=24;
                    break;

                case ADC_CH2P_P14:
                    ch=14;
                    break;

                case ADC_CH3N_P15:
                    ch=15;
                    break;

                case ADC_CH3P_P20:
                    ch=20;
                    break;

                default:
                    break;
                }

                if(ch!=0)
                {
                    LOG("P%d %d mv ",ch,(int)(value*1000));
                }
                else
                {
                    LOG("invalid channel\n");
                }
            }
        }

        LOG(" mode:%d \n",adc_cfg.is_continue_mode);
        channel_done_flag = 0;
    }
}



void p_adc_init(void)
{
    hal_adc_init();
    hal_adc_config_channel(adc_cfg, p_adc_evt);
    hal_adc_start(POLLING_MODE);
//  p_adcMeasureTask();
}
void p_adc_read(void)
{
    hal_adc_value_read((adc_CH_t)ADC_CHANEL_SELE);
}

#endif

#if (TEST_I2C == 1)
#include "i2c.h"
//----------------i2c---------------------
i2c_cfg p_i2c_cfg =
{
    .sda_pin = P33,
    .scl_pin = P34,
    .i2c_clock = I2C_CLOCK_400K,
    .i2c_mod = I2C_1,
    .i2c01 = AP_I2C1,
    .slave_addr = 0x50,
    .len = I2C_TXRX_LEN,
    .tx_data = {0x50,0x51,0x52,0x53,0x54,0x55,0x56},
    .rx_data = {0},
    .reg_addr = 0x10,
};

void p_i2c_init(void)
{
    i2c_cfg p = p_i2c_cfg;
    hal_gpio_pin_init(p.sda_pin,IE);
    hal_gpio_pin_init(p.scl_pin,IE);
    hal_gpio_pull_set(p.sda_pin,STRONG_PULL_UP);
    hal_gpio_pull_set(p.scl_pin,STRONG_PULL_UP);
    hal_i2c_pin_init(p.i2c_mod, p.sda_pin, p.scl_pin);
    void* master_pi2cc = hal_i2c_init(p.i2c_mod,p.i2c_clock);

    if(master_pi2cc==NULL)
    {
        LOG("I2C master init fail\n");
    }
    else
    {
        LOG("I2C master init OK\n");
    }
}

void p_i2c_write(void)
{
    i2c_cfg p = p_i2c_cfg;
    int ret = 0;
    hal_i2c_addr_update(p.i2c01,p.slave_addr);           //????24c02?υτ???--§Υ
    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_send(p.i2c01, (uint8*)&(p.reg_addr), 1);            //????24c02?υτ???--§Υ
    hal_i2c_send(p.i2c01, p.tx_data, p.len);
    HAL_EXIT_CRITICAL_SECTION();
    ret = hal_i2c_wait_tx_completed(p.i2c01);

    if(ret != 0)
    {
        LOG("i2c_write_error\n");
    }

    ret = 0;
}
void p_i2c_read(void)
{
    i2c_cfg p = p_i2c_cfg;
    int ret = 0;
    ret = hal_i2c_read(p.i2c01,p.slave_addr,p.reg_addr,p.rx_data,p.len);

    if(ret != 0)
    {
        LOG("i2c_read_error\n");
    }
    else
    {
        while(p.len > 0)
        {
            LOG("data[%d] = 0x%x ",p.len-1,p.rx_data[p.len-1]);
            p.len --;
        }

        LOG("\n");
    }
}
#endif

#if (TEST_PWM == 1)
#include "pwm.h"


pwm_cfg p_pwm_cfg =
{
    .pwm_pin = P16,
    .top = 10,
    .cmp = 5,
    .pwm_div = PWM_CLK_NO_DIV,
    .ch = PWM_CH0,
};
uint16_t p_pwm_output(void)
{
    pwm_cfg p = p_pwm_cfg;
    LOG("pin = %d, ch = %d, freq = %d,top = %d , cmp = %d\n",p.pwm_pin,p.ch,p.pwm_div,p.top,p.cmp);
    hal_pwm_init(p.ch,p.pwm_div,PWM_CNT_UP,PWM_POLARITY_RISING);
    hal_pwm_set_count_val(p.ch,p.cmp,p.top);
    hal_pwm_open_channel(p.ch,p.pwm_pin);
    hal_pwm_start();
//  success_cmd(2);
    return 0;
}
#endif

#if (TEST_SPI == 1)
#include "spi.h"
#include "spiflash.h"
#include "dma.h"
void p_spi_init(void)
{
    hal_dma_init();
    spiflash_init();
}

void p_spi_read_id(void)
{
    uint32_t id = spiflash_read_identification();
    LOG("flash_id = 0x%x\n",id);
}

#endif

#if (TEST_TIMER == 1)
#include "timer.h"

timer_cfg p_timer_cfg =
{
    .channel = AP_TIMER_ID_5,
    .us = 500000,
};
void timer_event(uint8_t evt)
{
    switch(evt)
    {
    case HAL_EVT_TIMER_5:
        LOG("timer5 evt happen\n");
        break;

    case HAL_EVT_TIMER_6:
        LOG("timer6 evt happen\n");
        break;

    case HAL_EVT_WAKEUP:
        LOG("wakeup\n");
        LOG("timer will disable when sleep,so if you want it work please init it when wakeup");
        break;

    case HAL_EVT_SLEEP:
        LOG("timer_sleep\n");
        break;

    default:
        LOG("timer err ");
        break;
    }
}

void p_timer_init(void)
{
    hal_timer_init(timer_event);
    hal_timer_set(p_timer_cfg.channel,500000);
}

void p_timer_mask(void)
{
    hal_timer_mask_int(p_timer_cfg.channel,1);
}

#endif

#if (TEST_WATCHDOG == 1)
#include "watchdog.h"

watchdog_cfg p_watchdog_cfg =
{
    .cycle = 1,
};
//cycle             = 0,1,2,3 ,4 ,5 ,6  ,7
//maxfeed_time      = 2,4,8,16,32,64,128,256

void p_watchdog_init(void)
{
    watchdog_config(p_watchdog_cfg.cycle);
    volatile int cancel_error = 1;

    while(cancel_error)
    {
        LOG("no feed wdt\n");
    }
}
void feed_dog(void)
{
    AP_WDT_FEED;
}

#endif

#if (TEST_GPIO == 1)

gpio_cfg p_gpio_cfg =
{
    .pin = P17,
};

static void gpio_event(gpio_pin_e pin,IO_Wakeup_Pol_e type)
{
    uint8 i;

    for(i = 0; i < 20; i++)
    {
        if(pin == p_gpio_cfg.pin)
        {
            LOG("%dth plority %d wakeup or int happen\n",pin,type);
            break;
        }
    }
}

void p_gpio_init(void)
{
    hal_gpio_init();
    hal_gpioretention_register(p_gpio_cfg.pin);
    hal_gpio_write(p_gpio_cfg.pin,1);
    hal_gpioin_register(p_gpio_cfg.pin, gpio_event, gpio_event);
}

#endif

uint16_t p_test_(uint8_t mod)
{
    LOG("\r\n+OK\r\n");

//    hal_system_soft_reset();
    switch(mod)
    {
    case 0:
        #if (TEST_ADC == 1)
        ;
        static int adc_flag = 0;

        if(adc_flag == 0)
        {
            p_adc_init();
            adc_flag = 1;
        }

        p_adc_read();
        LOG("adc\n");
        #endif
        break;

    case 1:
        #if (TEST_I2C == 1)
        p_i2c_init();
        p_i2c_write();
        WaitMs(50);
        p_i2c_read();
        LOG("i2c\n");
        #endif
        break;

    case 2:
        #if (TEST_PWM == 1)
        p_pwm_output();
        LOG("pwm\n");
        #endif
        break;

    case 3:
        #if (TEST_SPI == 1)
        p_spi_init();
        p_spi_read_id();
        LOG("spi\n");
        #endif
        break;

    case 4:
        #if (TEST_WATCHDOG == 1)
        p_watchdog_init();
        LOG("watchdog\n");
        #endif
        break;

    case 5:
        #if (TEST_TIMER == 1)
        p_timer_init();
        LOG("timer\n");
        #endif
        break;

    case 6:
        #if (TEST_GPIO == 1)
        p_gpio_init();
        LOG("gpio\n");
        #endif
        break;
    }

    return 0;
}
