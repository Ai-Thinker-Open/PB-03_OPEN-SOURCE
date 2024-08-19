/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/**************************************************************************************************
    Filename:       bsp_btn_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include <string.h>
#include "gpio.h"
#include "bsp_gpio.h"
#include "log.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "bsp_button_task.h"
#include "bsp_btn_demo.h"
#include "clock.h"

static uint8 Demo_TaskID;

#if ((BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_KSCAN) || (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN))
#define BTN_LONG_PRESS_START_TICK_COUNT         (100)
#define BTN_LONG_PRESS_KEEP_TICK_COUNT          (100)

static void keyboard_bsp_btn_callback(uint8 evt);
static void app_powerkey_press_report(uint8 key_index);
static void app_powerkey_release_report(uint8 key_index);
static void app_powerkey_longpress_report(void);

static bsp_app bsp_app_CBs =
{
    keyboard_bsp_btn_callback,
    app_powerkey_press_report,
    app_powerkey_release_report,
    app_powerkey_longpress_report,
};
static bsp_button_Cfg_t user_button_cfg =
{
    .bsp_evt_cb = &bsp_app_CBs,
    #ifdef BSP_BTN_LONG_PRESS_ENABLE
    .bsp_long_press_keep_cnt = BTN_LONG_PRESS_KEEP_TICK_COUNT,
    .bsp_long_press_start_cnt = BTN_LONG_PRESS_START_TICK_COUNT,
    #endif
    #if (BSP_COMBINE_BTN_NUM > 0)
    // ! bsp combine button config
    .usr_combine_btn_array = {
        (BIT(9) | BIT(10)),
        (BIT(10) | BIT(11) | BIT(12)),
    },
    .combine_btn_num = BSP_COMBINE_BTN_NUM,
    #endif
    // ! config keyboard col gpio
    .col_pin = {
        KSCAN_COL_0_GPIO,
        KSCAN_COL_1_GPIO,
        KSCAN_COL_2_GPIO,
        KSCAN_COL_3_GPIO,
    },
    // ! config keyboard row gpio
    .row_pin = {
        KSCAN_ROW_0_GPIO,
        KSCAN_ROW_1_GPIO,
        KSCAN_ROW_2_GPIO,
        KSCAN_ROW_3_GPIO,
    },
};
#endif

#if ((BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_GPIO) || (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN))

static bool bsp_btn_timer_flag = FALSE;
BTN_COMBINE_T usr_combine_btn_array[BSP_COMBINE_BTN_NUM]=
{
    (BIT(9) | BIT(10)),
    (BIT(10) | BIT(11) | BIT(12)),
};
uint8_t Bsp_Btn_Get_Index(gpio_pin_e pin)
{
    uint8_t i;

    if(hal_gpio_btn_get_index(pin,&i) == PPlus_SUCCESS)
    {
        return (i);
    }

    return 0xFF;
}

static void Bsp_Btn_Check(uint8_t ucKeyCode)
{
    #if ((BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_GPIO) || (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN))
    hal_gpio_btn_cb(ucKeyCode);
    #endif

    if (bsp_btn_timer_flag == TRUE)
    {
        if (bsp_KeyEmpty() == TRUE)
        {
            osal_stop_timerEx(Demo_TaskID, BSP_BTN_EVT_SYSTICK);
            bsp_btn_timer_flag = FALSE;
        }
    }
}

void gpio_btn_pin_event_handler(gpio_pin_e pin,IO_Wakeup_Pol_e type)
{
    if (((GPIO_SINGLE_BTN_IDLE_LEVEL == 0) && (POL_RISING == type)) ||
            ((GPIO_SINGLE_BTN_IDLE_LEVEL == 1) && (POL_RISING != type)))
    {
        bsp_btn_timer_flag = TRUE;
        osal_start_reload_timer(Demo_TaskID, BSP_BTN_EVT_SYSTICK, BTN_SYS_TICK);
        bsp_set_key_value_by_index(Bsp_Btn_Get_Index(pin), 1);
    }
    else
    {
        bsp_set_key_value_by_index(Bsp_Btn_Get_Index(pin), 0);
    }
}

static void gpio_bsp_btn_callback(uint8_t evt)
{
    LOG("gpio evt:0x%x  ", evt);

    switch (BSP_BTN_TYPE(evt))
    {
    case BSP_BTN_PD_TYPE:
        LOG("press down ");
        break;

    case BSP_BTN_UP_TYPE:
        LOG("press up ");
        break;

    case BSP_BTN_LPS_TYPE:
        LOG("long press start ");
        break;

    case BSP_BTN_LPK_TYPE:
        LOG("long press keep ");
        break;

    default:
        LOG("unexpected ");
        break;
    }

    LOG("value:%d\n", BSP_BTN_INDEX(evt));
}

Gpio_Btn_Info gpio_btn_info =
{
    {P14, P15, P26},
    gpio_bsp_btn_callback,
};

#endif

#if ((BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_KSCAN) || (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN))
static void keyboard_bsp_btn_callback(uint8_t evt)
{
    uint8 key_index = BSP_BTN_INDEX(evt);

    // !start power off event
    // start_enter_power_off_mode();
    switch (BSP_BTN_TYPE(evt))
    {
    case BSP_BTN_PD_TYPE:
        LOG("[key]:%02d pre\n", key_index);
        break;

    case BSP_BTN_UP_TYPE:
        LOG("[key]:%02d rel\n", key_index);
        break;

    case BSP_BTN_LPS_TYPE:
        LOG("[key]:%02d longpress\n", key_index);
        break;

    case BSP_BTN_LPK_TYPE:
        LOG("long press keep\r\n");
        break;

    default:
        LOG("unexpected\n");
        break;
    }
}

/*********************************************************************
    @fn      app_powerkey_press_report

    @brief   power offkey press report api

    @param   key_index---key data

    @return  none
*/
static void app_powerkey_press_report(uint8 key_index)
{
    LOG("[key]:%02d pre---tick=%d\n", key_index, hal_systick());
}

/*********************************************************************
    @fn      app_powerkey_release_report

    @brief   power offkey release report api

    @param   key_index---key data

    @return  none
*/
static void app_powerkey_release_report(uint8 key_index)
{
    LOG("[key]:%02d rel\n", key_index);
}

/*********************************************************************
    @fn      app_powerkey_release_report

    @brief   power offkey release report api

    @param   key_index---key data

    @return  none
*/
static void app_powerkey_longpress_report(void)
{
    LOG("poweroff key long tickc=%d\n", hal_systick());
}
#endif

void Demo_Init(uint8 task_id)
{
    Demo_TaskID = task_id;
    #if ((BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_KSCAN) || (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN))
    btp_button_init(&user_button_cfg);
    #endif
    #if ((BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_GPIO) || (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN))

    if (PPlus_SUCCESS != hal_gpio_btn_init(&gpio_btn_info))
    {
        LOG("hal_gpio_btn_init error:%d\n", __LINE__);
    }

    for (int i = 0; i < BSP_TOTAL_BTN_NUM; i++)
    {
        usr_sum_btn_array[i].KeyConfig = (BSP_BTN_PD_CFG | BSP_BTN_UP_CFG | BSP_BTN_LPS_CFG | BSP_BTN_LPK_CFG);
    }

    #if (BSP_COMBINE_BTN_NUM > 0)

    if (PPlus_SUCCESS != bsp_InitBtn(usr_sum_btn_array, BSP_TOTAL_BTN_NUM, BSP_SINGLE_BTN_NUM, usr_combine_btn_array))
    #else
    if (PPlus_SUCCESS != bsp_InitBtn(usr_sum_btn_array, BSP_TOTAL_BTN_NUM, 0, NULL))
    #endif
    {
        LOG("bsp button init error\n");
    }

    #endif
}

uint16 Demo_ProcessEvent(uint8 task_id, uint16 events)
{
    if (Demo_TaskID != task_id)
    {
        return 0;
    }

    #if ((BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_GPIO) || (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN))
    uint8_t ucKeyCode;

    if (events & BSP_BTN_EVT_SYSTICK)
    {
        ucKeyCode = bsp_KeyPro();

        if (ucKeyCode != BTN_NONE)
        {
            Bsp_Btn_Check(ucKeyCode);
        }

        return (events ^ BSP_BTN_EVT_SYSTICK);
    }

    #endif
    return 0;
}
