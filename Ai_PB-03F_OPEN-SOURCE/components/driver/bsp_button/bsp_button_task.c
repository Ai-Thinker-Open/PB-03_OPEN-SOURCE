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
    Filename:       bsp_button_task.c
    Revised:        $Date $
    Revision:       $Revision $
**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include <string.h>
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "pwrmgr.h"
#include "bsp_button_task.h"
uint8 Bsp_Btn_TaskID;

KEYBOARD_MODE_t keybaord_extend_mode = NO_EXTEND;

// ! keyboard register core strure
static keyboard_cfg app_keyboard_cfg;

static bsp_button_Cfg_t app_button_cfg;

static void PowerOff_key_change_event(uint8 col_nums, uint8 row, uint8 col, uint8 key_state);
static void PowerOff_key_polling_event(void);
static void PowerOff_key_restore_event(void);
static void normal_key_change_event(uint8 col_nums, uint8 row, uint8 col, uint8 key_state);
static void normal_key_polling_event(void);
static void normal_key_retore_event(void);
static void poweroff_Key_switch_keybaord_checking(void);
static void keyboard_init(void);
static void keyboard_poweroff_init(void);

// ! keybaord driver register callback for start mode
static poweroff_key_report AppPoweroffKeyCBs =
{
    PowerOff_key_change_event,  /* key press or release report by this api when key status change for poweroff key*/
    PowerOff_key_polling_event, /* key scan report by this api when keybaord polling for poweroff key*/
    PowerOff_key_restore_event, /* keyboard critical report by this api for poweroff key*/
};

// ! keybaord driver register callback for working mode
static normal_key_report AppNormalKeyCBs =
{
    normal_key_change_event,  /* key press or release report by this api when key status change */
    normal_key_polling_event, /* key scan report by this api when keybaord polling*/
    normal_key_retore_event,  /* keyboard idle*/
};

/*********************************************************************
    @fn      PowerOff_key_change_event

    @brief   keybaord soft scan key status change report for poweroffkey

    @param   col_nums row col key_state

    @return  void
*/
static void PowerOff_key_change_event(uint8 col_nums, uint8 row, uint8 col, uint8 key_state)
{
    if (key_state == 1)
    {
        bsp_set_key_value_by_row_col(col_nums, row, col, TRUE);
    }
    else
    {
        if ((row == READ_MATRIX_KEY % MATRIX_KEYBOARD_ROW) && (col == READ_MATRIX_KEY / MATRIX_KEYBOARD_ROW) && (key_checking_flag == 1))
        {
            key_checking_flag = 0;
            poweroff_key_release = 1;
        }

        bsp_set_key_value_by_row_col(col_nums, row, col, FALSE);
    }
}

/*********************************************************************
    @fn      PowerOff_key_change_event

    @brief   void

    @param   void

    @return  void
*/
static void PowerOff_key_polling_event(void)
{
    // ! ir repeated handle
    static bool send_flag = 0;

    if (get_key_checking_Flag() == 1)
    {
        if (time_exceed(poweroff_keydown_timestamp, POWER_REPEATED_RELATIVE_TIME))
        {
            if (send_flag == 0)
            {
                send_flag = 1;

                if (bsp_btn_cb && bsp_btn_cb->poweroff_Key_lon_cb)
                {
                    bsp_btn_cb->poweroff_Key_lon_cb();
                }
            }
        }
    }

    // ! key press or combine key press
    keybaord_middle_button_data_handle();
}

/*********************************************************************
    @fn      PowerOff_key_restore_event

    @brief   void

    @param   void

    @return  void
*/
static void PowerOff_key_restore_event(void)
{
    key_checking_flag = 0;
    poweroff_key_Release_StartPhaseReport();
    keyboard_init();
}

/*********************************************************************
    @fn      normal_key_change_event

    @brief   update keyboard data to middle key layout

    @param   col_nums--col number  row--row value col--col value  key_state--press or release

    @return  void
*/
static void normal_key_change_event(uint8 col_nums, uint8 row, uint8 col, uint8 key_state)
{
    if (key_state == 1)
    {
        // ! key press
        bsp_set_key_value_by_row_col(col_nums, row, col, TRUE);
    }
    else
    {
        // ! key release
        bsp_set_key_value_by_row_col(col_nums, row, col, FALSE);
    }
}

/*********************************************************************
    @fn      normal_key_polling_event

    @brief   void

    @param   void

    @return  void
*/
static void normal_key_polling_event(void)
{
    // ! bsp button data handle
    keybaord_middle_button_data_handle();
}

/*********************************************************************
    @fn      normal_key_retore_event

    @brief   void

    @param   void

    @return  void
*/
static void normal_key_retore_event(void)
{
    // ! nothing to do
}

/*********************************************************************
    @fn      short_press_key_report

    @brief   void

    @param   void

    @return  void
*/
static void poweroff_Key_switch_keybaord_checking(void)
{
    uint8 ret = power_Off_init_phase_key_checking(app_keyboard_cfg.keyboard_param.rows, app_keyboard_cfg.keyboard_param.cols);

    // ! poweroffkey aleady report release
    if (ret == 0)
    {
        keyboard_init();
    }
    else
    {
        // ! poweroffkey long press and switch soft scan this poweroff key status
        if (READ_MATRIX_KEY <= MATRIX_KEY_MAX && g_system_reset_cause == WAKE_SYSTEM_CAUSE)
        {
            keyboard_poweroff_init();
            osal_start_timerEx(Bsp_Btn_TaskID, BSP_SOFT_POLLING_EVT, 1);
        }
        else
        {
            LOG("no check key\r\n");
            keyboard_init();
        }
    }
}

/*********************************************************************
    @fn      keyboard_init

    @brief   void

    @param   void

    @return  void
*/
static void keyboard_init(void)
{
    app_keyboard_cfg.keyboard_param.rows = (GPIO_Pin_e*)app_button_cfg.row_pin;
    app_keyboard_cfg.keyboard_param.cols = (GPIO_Pin_e*)app_button_cfg.col_pin;
    app_keyboard_cfg.keyboard_param.keyboard_mode = NO_EXTEND;
    app_keyboard_cfg.keyboard_param.keyboard_osal_event = BSP_SOFT_POLLING_EVT;
    app_keyboard_cfg.keyboard_param.keyboard_osal_taskId = Bsp_Btn_TaskID;
    app_keyboard_cfg.keyboard_param.combine_param.keyboard_combine_num = app_button_cfg.combine_btn_num;
    app_keyboard_cfg.keyboard_param.combine_param.keyboard_combine_btn_array = app_button_cfg.usr_combine_btn_array;
    app_keyboard_cfg.keyboard_param.polling_interval = 10;
    app_keyboard_cfg.keyboard_param.poweroff_polling_interval = 0;
    app_keyboard_cfg.keyboard_param.keyboard_pwrmgr_mod = MOD_USR0;
    #ifdef BSP_BTN_LONG_PRESS_ENABLE
    app_keyboard_cfg.keyboard_param.long_press_keep_count = app_button_cfg.bsp_long_press_keep_cnt;
    app_keyboard_cfg.keyboard_param.long_press_start_count = app_button_cfg.bsp_long_press_start_cnt;
    #endif
    app_keyboard_cfg.PowerOffKeyReportCB = NULL;
    app_keyboard_cfg.NormalKeyReportCB = &AppNormalKeyCBs;
    bsp_button_param_init(&app_keyboard_cfg);
}

/*********************************************************************
    @fn      keyboard_poweroff_init

    @brief   void

    @param   void

    @return  void
*/
static void keyboard_poweroff_init(void)
{
    app_keyboard_cfg.keyboard_param.rows = (GPIO_Pin_e*)app_button_cfg.row_pin;
    app_keyboard_cfg.keyboard_param.cols = (GPIO_Pin_e*)app_button_cfg.col_pin;
    app_keyboard_cfg.keyboard_param.keyboard_mode = keybaord_extend_mode;
    app_keyboard_cfg.keyboard_param.keyboard_osal_event = BSP_SOFT_POLLING_EVT;
    app_keyboard_cfg.keyboard_param.keyboard_osal_taskId = Bsp_Btn_TaskID;
    app_keyboard_cfg.keyboard_param.combine_param.keyboard_combine_num = app_button_cfg.combine_btn_num;
    app_keyboard_cfg.keyboard_param.combine_param.keyboard_combine_btn_array = app_button_cfg.usr_combine_btn_array;
    app_keyboard_cfg.keyboard_param.polling_interval = 0;
    app_keyboard_cfg.keyboard_param.poweroff_polling_interval = 4;
    app_keyboard_cfg.keyboard_param.keyboard_pwrmgr_mod = MOD_USR4;
    app_keyboard_cfg.PowerOffKeyReportCB = &AppPoweroffKeyCBs;
    app_keyboard_cfg.NormalKeyReportCB = NULL;
    keyboard_register(&app_keyboard_cfg);
    LOG("%s\r\n", __FUNCTION__);
}

/*********************************************************************
    @fn      get_key_checking_Flag

    @brief   get keyboard wheather idle

    @param   void

    @return  1---working  0 -- idle
*/
void system_run_keyboard_checking(void)
{
    // ! POWEROFF KEY PRESS REPORT
    poweroff_key_Press_StartPhaseReport();
    // ! POWEROFF KEY RELEASE CHECKING
    power_Off_init_phase_key_checking(app_keyboard_cfg.keyboard_param.rows, app_keyboard_cfg.keyboard_param.cols);
}

/*********************************************************************
    @fn      btp_button_init

    @brief   kscan or soft scan init config & bsp button config

    @param   void

    @return  void
*/
void btp_button_init(bsp_button_Cfg_t* button_cfg)
{
    osal_memcpy(&app_button_cfg,button_cfg,sizeof(bsp_button_Cfg_t));
    key_app_register(app_button_cfg.bsp_evt_cb);

    // ! bsp button hal init config
    if (g_system_reset_cause == WAKE_SYSTEM_CAUSE)
    {
        osal_set_event(Bsp_Btn_TaskID, BSP_POWER_ON_EVT);
    }
    else
    {
        keyboard_init();
    }

    osal_start_timerEx(Bsp_Btn_TaskID, BSP_SOFT_POLLING_EVT, BSP_SOFT_SCAN_POLLING_TIME);
}

/*********************************************************************
    @fn      kscan_enter_power_off_gpio_option

    @brief   kscan or soft scan poweroff gpio config

    @param   void

    @return  void
*/
void kscan_enter_power_off_gpio_option(void)
{
    keyboard_deep_sleep_handler();
}

void Bsp_Btn_Init(uint8 task_id)
{
    Bsp_Btn_TaskID = task_id;
}

uint16 Bsp_Btn_ProcessEvent(uint8 task_id, uint16 events)
{
    if (Bsp_Btn_TaskID != task_id)
    {
        return 0;
    }

    if (events & BSP_SOFT_POLLING_EVT) // ! soft scan polling events
    {
        keyboard_io_read();
        return (events ^ BSP_SOFT_POLLING_EVT);
    }

    if (events & BSP_POWER_ON_EVT) // ! start key or poweroff handle
    {
        poweroff_Key_switch_keybaord_checking();
        return (events ^ BSP_POWER_ON_EVT);
    }

    return 0;
}
