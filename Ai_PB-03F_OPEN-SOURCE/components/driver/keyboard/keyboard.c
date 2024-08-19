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

#include "keyboard.h"
#include "bsp_button.h"
#include "pwrmgr.h"
#include "OSAL_Timers.h"
#include "OSAL.h"

static keyboard_cfg* KeyBoardCfg = NULL;
static uint8 matrix_pin_state[MATRIX_KEYBOARD_ROW] = {0x00};

static void keyboard_matrix_pin_init(uint8 choice_mode);

/*********************************************************************
    @fn      keyboard_to_sleep_gpio_config

    @brief   GPIO CONFIG BEFORE KEYBAORD SLEEP

    @param   void

    @return  void
*/
static void keyboard_to_sleep_gpio_config(void)
{
    keyboard_param_t* key_para = &KeyBoardCfg->keyboard_param;

    for (uint8 n = 0; n < MATRIX_KEYBOARD_ROW; n++)
    {
        hal_gpio_pin_init(key_para->rows[n], IE);
        hal_gpio_pull_set(key_para->rows[n], WEAK_PULL_UP);
    }
}

/*********************************************************************
    @fn      compare_data_arr_is_null

    @brief   COMPARE DATA

    @param   data & data len

    @return  true-data all is zero  false--data have other value
*/
uint8_t compare_data_arr_is_null(uint8_t* data, uint8_t data_len)
{
    if (data == NULL)
    {
        return true;
    }

    for (uint8 i = 0; i < data_len; i++)
    {
        if (data[i] != 0)
        {
            return false;
        }
    }

    return true;
}

/*********************************************************************
    @fn      keyboard_sleep_handler

    @brief   SYSTEM SLEEP CALLBACK FOR KEYBOARD

    @param   void

    @return  void
*/
__ATTR_SECTION_SRAM__ void keyboard_sleep_handler(void)
{
    IO_Wakeup_Pol_e pol;
    keyboard_param_t* key_para = &KeyBoardCfg->keyboard_param;

    if (key_para->rows == NULL || key_para->cols == NULL)
    {
        return;
    }

    for (uint8 i = 0; i < MATRIX_KEYBOARD_COL; i++)
    {
        GPIO_Pin_e col_pin = key_para->cols[i];
        hal_gpio_pin_init(col_pin, OEN);
        hal_gpio_fast_write(col_pin, 0);
        hal_gpio_pull_set(col_pin, PULL_DOWN);
    }

    for (uint8 i = 0; i < MATRIX_KEYBOARD_ROW; i++)
    {
        hal_gpio_pull_set(key_para->rows[i], WEAK_PULL_UP);
        GPIO_Pin_e row_pin = key_para->rows[i];
        hal_gpio_pin_init(row_pin, IE);
        pol = hal_gpio_read(row_pin) ? NEGEDGE : POSEDGE;
        hal_gpio_wakeup_set(row_pin, pol);
        matrix_pin_state[i] = pol;
    }
}

/*********************************************************************
    @fn      keyboard_sleep_handler

    @brief   SYSTEM WAKEUP CALLBACK FOR KEYBOARD

    @param   void

    @return  void
*/
__ATTR_SECTION_SRAM__ static void keyboard_wakeup_handler(void)
{
    IO_Wakeup_Pol_e pol;
    keyboard_param_t* key_para = &KeyBoardCfg->keyboard_param;

    if (key_para->rows == NULL || key_para->cols == NULL)
    {
        return;
    }

    for (uint8 i = 0; i < MATRIX_KEYBOARD_COL; i++)
    {
        GPIO_Pin_e col_pin = key_para->cols[i];
        hal_gpio_pin_init(col_pin, OEN);
        hal_gpio_fast_write(col_pin, 0);
    }

    for (uint8 i = 0; i < MATRIX_KEYBOARD_ROW; i++)
    {
        GPIO_Pin_e row_pin = key_para->rows[i];
        hal_gpio_pin_init(row_pin, IE);
        hal_gpio_pull_set(row_pin, WEAK_PULL_UP);
    }

    for (uint8 i = 0; i < MATRIX_KEYBOARD_ROW; i++)
    {
        GPIO_Pin_e row_pin = key_para->rows[i];
        hal_gpio_pin_init(row_pin, IE);
        pol = hal_gpio_read(row_pin) ? POSEDGE : NEGEDGE;

        if (pol == matrix_pin_state[i])
        {
            break;
        }
        else if (pol == 0 && matrix_pin_state[i] == 1)
        {
            break;
        }
        else if (i == MATRIX_KEYBOARD_ROW - 1)
        {
            return;
        }
    }

    osal_start_timerEx(key_para->keyboard_osal_taskId, key_para->keyboard_osal_event, 1);
}

/*********************************************************************
    @fn      keyboard_matrix_pin_init

    @brief   row or col gpio input or output config for keyboard

    @param   choice_mode

    @return  void
*/
static void keyboard_matrix_pin_init(uint8 choice_mode)
{
    uint8_t i = 0;
    keyboard_param_t* key_para = &KeyBoardCfg->keyboard_param;

    if (key_para->rows == NULL || key_para->cols == NULL)
    {
        return;
    }

    switch (choice_mode)
    {
    case 0:
        for (i = 0; i < MATRIX_KEYBOARD_COL; i++)
        {
            hal_gpio_pull_set(key_para->cols[i], WEAK_PULL_UP);
            hal_gpio_pin_init(key_para->cols[i], OEN);
            hal_gpio_fast_write(key_para->cols[i], 1);
        }

        for (i = 0; i < MATRIX_KEYBOARD_ROW; i++)
        {
            hal_gpio_pull_set(key_para->rows[i], WEAK_PULL_UP);
            hal_gpio_pin_init(key_para->rows[i], IE);
        }

        break;

    case 1:
        for (i = 0; i < MATRIX_KEYBOARD_COL; i++)
        {
            hal_gpio_pull_set(key_para->cols[i], WEAK_PULL_UP);
            hal_gpio_pin_init(key_para->cols[i], IE);
        }

        for (i = 0; i < MATRIX_KEYBOARD_ROW; i++)
        {
            hal_gpio_pull_set(key_para->rows[i], WEAK_PULL_UP);
            hal_gpio_pin_init(key_para->rows[i], OEN);
            hal_gpio_fast_write(key_para->rows[i], 1);
        }

        break;
    }
}
/*********************************************************************
    @fn      keyboard_scan_col

    @brief   read row gpio data by col out high or low level

    @param   scan_col_data--for save read row data

    @return  void
*/
void keybaord_pin_config(uint8 col)
{
    keyboard_param_t* key_para = &KeyBoardCfg->keyboard_param;

    for (uint8 i = 0; i < MATRIX_KEYBOARD_COL; i++)
    {
        if (i != col)
        {
            hal_gpio_pull_set(key_para->cols[i], WEAK_PULL_UP);
            hal_gpio_pin_init(key_para->cols[i], IE);
        }
    }
}

/*********************************************************************
    @fn      keyboard_scan_col

    @brief   read row gpio data by col out high or low level

    @param   scan_col_data--for save read row data

    @return  void
*/
void keyboard_scan_col(uint8* scan_col_data)
{
    uint8 row_state_arr[MATRIX_KEYBOARD_COL] = {0x00};
    GPIO_Pin_e* scan_col = KeyBoardCfg->keyboard_param.cols;
    GPIO_Pin_e* read_row = KeyBoardCfg->keyboard_param.rows;

    if (scan_col == NULL || read_row == NULL)
    {
        return;
    }

    keyboard_matrix_pin_init(0);

    for (uint8 column = 0; column < MATRIX_KEYBOARD_COL; column++)
    {
        hal_gpio_write(scan_col[column], 0);
        keybaord_pin_config(column);

        for (uint8 i = 0; i < MATRIX_KEYBOARD_ROW; i++)
        {
            uint32 r = hal_gpio_read((GPIO_Pin_e)read_row[i]);
            row_state_arr[column] |= ((uint8_t)(((!r) ? 1 : 0) << i));
        }

        if (row_state_arr[column] != 0)
        {
            for (uint8 row = 0; row < MATRIX_KEYBOARD_ROW; row++)
            {
                if (row_state_arr[column] & (1U << row))
                {
                    scan_col_data[column * MATRIX_KEYBOARD_ROW + row] = column * MATRIX_KEYBOARD_ROW + row + 1;
                }
                else
                {
                    scan_col_data[column * MATRIX_KEYBOARD_ROW + row] = 0;
                }
            }
        }
        else
        {
            for (uint8 row = 0; row < MATRIX_KEYBOARD_ROW; row++)
            {
                scan_col_data[column * MATRIX_KEYBOARD_ROW + row] = 0;
            }
        }

        hal_gpio_write(scan_col[column], 1);
        osal_memset(row_state_arr, 0x00, sizeof(row_state_arr));
    }
}

/*********************************************************************
    @fn      keyboard_scan_row

    @brief   read col gpio data by row out high or low level

    @param   scan_row_data--for save read col data

    @return  void
*/
void keyboard_scan_row(uint8* scan_row_data)
{
    uint8 column_state_arr[MATRIX_KEYBOARD_ROW] = {0x00};
    GPIO_Pin_e* scan_row = KeyBoardCfg->keyboard_param.rows;
    GPIO_Pin_e* read_col = KeyBoardCfg->keyboard_param.cols;

    if (scan_row == NULL || read_col == NULL)
    {
        return;
    }

    keyboard_matrix_pin_init(1);

    for (uint8 row = 0; row < MATRIX_KEYBOARD_ROW; row++)
    {
        hal_gpio_write(scan_row[row], 0);

        for (uint8 i = 0; i < MATRIX_KEYBOARD_COL; i++)
        {
            uint32 r = hal_gpio_read(read_col[i]);
            column_state_arr[row] |= (((!r) ? 1 : 0) << i);
        }

        if (column_state_arr[row] != 0)
        {
            for (uint8 column = 0; column < MATRIX_KEYBOARD_COL; column++)
            {
                if (column_state_arr[row] & (1U << column))
                {
                    scan_row_data[column * MATRIX_KEYBOARD_ROW + row] = column * MATRIX_KEYBOARD_ROW + row + 1;
                }
                else
                {
                    scan_row_data[column * MATRIX_KEYBOARD_ROW + row] = 0;
                }
            }
        }
        else
        {
            for (uint8 column = 0; column < MATRIX_KEYBOARD_COL; column++)
            {
                scan_row_data[column * MATRIX_KEYBOARD_ROW + row] = 0;
            }
        }

        hal_gpio_write(scan_row[row], 1);
        osal_memset(column_state_arr, 0x00, sizeof(column_state_arr));
    }
}

/*********************************************************************
    @fn      keybaord_data_handle

    @brief   row data & col data analyse handle

    @param   new_row_data new_col_data debonce_data previous_data

    @return  void
*/
void keybaord_data_handle(uint8* new_row_data, uint8* new_col_data, uint8* debonce_data, uint8* previous_data)
{
    uint8 push_key_state = 0;
    uint8 push_row = 0, push_col = 0;
    normal_key_report* nor_keycb = KeyBoardCfg->NormalKeyReportCB;
    poweroff_key_report* pow_keycb = KeyBoardCfg->PowerOffKeyReportCB;

    if (nor_keycb == NULL && pow_keycb == NULL)
    {
        return;
    }

    for (uint8 i = 0; i < MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW; i++)
    {
        if (new_row_data[i] == 0 && new_col_data[i] != 0)
        {
            new_row_data[i] = new_col_data[i];
        }

        if (new_row_data[i] != 0)
        {
            if (debonce_data[i] < MATRIX_DEBONCE_COUNT)
            {
                debonce_data[i]++;

                if (debonce_data[i] == MATRIX_DEBONCE_COUNT && previous_data[i] != 0)
                {
                    push_key_state = !!new_row_data[i];
                    push_row = i % MATRIX_KEYBOARD_ROW;
                    push_col = i / MATRIX_KEYBOARD_ROW;
                    KEYBOARD_LOG("[%d %d %d]\n", push_row, push_col, push_key_state);

                    // ! REPORT KEY PRESS FROM POWEROFF MODE
                    if (pow_keycb && pow_keycb->poweroff_keybaord_change_report)
                    {
                        pow_keycb->poweroff_keybaord_change_report(MATRIX_KEYBOARD_COL, push_row, push_col, push_key_state);
                    }

                    // ! REPORT KEY PRESS FROM WORK MODE
                    if (nor_keycb && nor_keycb->keyboard_change_cb)
                    {
                        nor_keycb->keyboard_change_cb(MATRIX_KEYBOARD_COL, push_row, push_col, push_key_state);
                    }
                }
            }
        }
        else
        {
            if (debonce_data[i] == MATRIX_DEBONCE_COUNT && new_row_data[i] == 0)
            {
                // push_key_state = !!current_matrix_data[i];
                push_key_state = !!new_row_data[i];
                push_row = i % MATRIX_KEYBOARD_ROW;
                push_col = i / MATRIX_KEYBOARD_ROW;
                KEYBOARD_LOG("[%d %d %d]\n", push_row, push_col, push_key_state);

                // ! REPORT KEY PRESS FROM POWEROFF MODE
                if (pow_keycb && pow_keycb->poweroff_keybaord_change_report)
                {
                    pow_keycb->poweroff_keybaord_change_report(MATRIX_KEYBOARD_COL, push_row, push_col, push_key_state);
                }

                // ! REPORT KEY PRESS FROM WORK MODE
                if (nor_keycb && nor_keycb->keyboard_change_cb)
                {
                    nor_keycb->keyboard_change_cb(MATRIX_KEYBOARD_COL, push_row, push_col, push_key_state);
                }
            }

            debonce_data[i] = 0;
        }

        previous_data[i] = new_row_data[i];
        new_row_data[i] = 0;
    }

    if (pow_keycb && pow_keycb->poweroff_keybaord_polling_report)
    {
        pow_keycb->poweroff_keybaord_polling_report();
    }

    if (nor_keycb && nor_keycb->keyboard_polling_cb)
    {
        nor_keycb->keyboard_polling_cb();
    }
}
/*********************************************************************
    @fn      keyboard_next_scaning

    @brief   next scaning restart or next scan to stop scan

    @param   previous_data previous_data_len debonce_data

    @return  void
*/
void keyboard_next_scaning(uint8* previous_data, uint8 previous_data_len, uint8* debonce_data)
{
    poweroff_key_report* pw_key_report = KeyBoardCfg->PowerOffKeyReportCB;
    normal_key_report* nor_key_report = KeyBoardCfg->NormalKeyReportCB;
    keyboard_param_t* key_parm = &KeyBoardCfg->keyboard_param;
    keyboard_to_sleep_gpio_config();

    if (true == compare_data_arr_is_null(previous_data, previous_data_len))
    {
        osal_memset(previous_data, 0x00, previous_data_len);
        osal_memset(debonce_data, 0x00, previous_data_len);

        if (hal_pwrmgr_get_module_lock_status() == true)
        {
            osal_clear_event(key_parm->keyboard_osal_taskId, key_parm->keyboard_osal_event);

            if (pw_key_report && pw_key_report->poweroff_keybaord_restore_report)
            {
                pw_key_report->poweroff_keybaord_restore_report();
            }

            if (nor_key_report && nor_key_report->keybaord_restore_cb)
            {
                nor_key_report->keybaord_restore_cb();
            }
        }
        else
        {
            if (pw_key_report)
            {
                osal_start_timerEx(key_parm->keyboard_osal_taskId, key_parm->keyboard_osal_event, key_parm->poweroff_polling_interval);
            }
            else
            {
                osal_start_timerEx(key_parm->keyboard_osal_taskId, key_parm->keyboard_osal_event, key_parm->polling_interval);
            }
        }
    }
    else
    {
        if (pw_key_report)
        {
            osal_start_timerEx(key_parm->keyboard_osal_taskId, key_parm->keyboard_osal_event, key_parm->poweroff_polling_interval);
        }
        else
        {
            osal_start_timerEx(key_parm->keyboard_osal_taskId, key_parm->keyboard_osal_event, key_parm->polling_interval);
        }
    }
}

/*********************************************************************
    @fn      keyboard_io_read

    @brief   polling use this function can do keyboard scaning

    @param   void

    @return  void
*/
void keyboard_io_read(void)
{
    static uint8 previous_matrix_data[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW];
    static uint8 current_matrix_data_column[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW];
    static uint8 current_matrix_data_row[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW];
    static uint8 current_data_checking[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW];
    keyboard_scan_col(current_matrix_data_column);
    // keyboard_scan_row( current_matrix_data_row );
    keybaord_data_handle(current_matrix_data_row, current_matrix_data_column, current_data_checking, previous_matrix_data);
    keyboard_next_scaning(previous_matrix_data, MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW, current_data_checking);
}

/*********************************************************************
    @fn      keyboard_register

    @brief   register keyboard relative param

    @param   cfg---param

    @return  void
*/
void keyboard_register(keyboard_cfg* cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    KeyBoardCfg = cfg;
    keyboard_to_sleep_gpio_config();
    hal_pwrmgr_register(KeyBoardCfg->keyboard_param.keyboard_pwrmgr_mod, keyboard_sleep_handler, keyboard_wakeup_handler);
}

/*********************************************************************
    @fn      keyboard_read_once

    @brief   for start phase to read key status

    @param   void

    @return  1-key press 0--no key press
*/
uint8 keyboard_read_once(GPIO_Pin_e* matrix_row_to_pin_map, GPIO_Pin_e* matrix_col_to_pin_map)
{
    uint32 matrix_io_status = 0x00;
    uint8 key_index[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW] = {0x00};
    uint8 key_state[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW] = {0x00};
    uint8 key_change = 0;
    uint8 push_key_state = 0XFF;
    uint8 preious_key_state[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW] = {0x00};

    for (uint8 a = 0; a < MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW; a++)
    {
        key_index[a] = a + 1;
    }

    /* for  key extend to gpio and no connect gnd*/
    // ! row io init
    for (uint8 n = 0; n < MATRIX_KEYBOARD_ROW; n++)
    {
        hal_gpio_pin_init(matrix_row_to_pin_map[n], IE);
        hal_gpio_pull_set(matrix_row_to_pin_map[n], WEAK_PULL_UP);
    }

    // ! col io init
    for (uint8 n = 0; n < sizeof(matrix_col_to_pin_map); n++)
    {
        hal_gpio_write(matrix_col_to_pin_map[n], 1);
    }

    for (uint8 col = 0; col < sizeof(matrix_col_to_pin_map); col++)
    {
        // ! col io init
        hal_gpio_write(matrix_col_to_pin_map[col], 0);
        keybaord_pin_config(col);

        // ! read row io status when key no extend to gnd
        for (uint8 m = 0; m < sizeof(matrix_row_to_pin_map); m++)
        {
            if ((matrix_io_status & (1 << m)) == 0)
            {
                matrix_io_status |= (((!hal_gpio_read(matrix_row_to_pin_map[m])) ? 1 : 0) << (col * MATRIX_KEYBOARD_ROW + m));
            }
        }

        hal_gpio_write(matrix_col_to_pin_map[col], 1);
    }

    for (uint8 j = 0; j < MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW; j++)
    {
        if ((matrix_io_status & BIT(j)) != 0)
        {
            key_state[j] = key_index[j];
        }
        else
        {
            key_state[j] = 0;
        }
    }

    // !key data analyse
    for (uint8 l = 0; l < MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW; l++)
    {
        key_change = preious_key_state[l] ^ key_state[l];

        if (key_change != 0)
        {
            push_key_state = !!key_state[l];

            if (push_key_state == 1)
            {
                // row * cols_num + col
                //              uint8 kscan_index = 0;
                //              kscan_index = l%MATRIX_KEYBOARD_ROW * MATRIX_KEYBOARD_COL + l/MATRIX_KEYBOARD_ROW + 1;
                return 1;
            }

            preious_key_state[l] = key_state[l];
        }
    }

    return 0;
}

/*********************************************************************
    @fn      keyboard_deep_sleep_handler

    @brief   keyboard gpio config for enter poweroff

    @param   void

    @return  void
*/
void keyboard_deep_sleep_handler(void)
{
    keyboard_param_t* key_para = &KeyBoardCfg->keyboard_param;

    for (uint8 i = 0; i < MATRIX_KEYBOARD_COL; i++)
    {
        GPIO_Pin_e col_pin = key_para->cols[i];
        hal_gpio_pin_init(col_pin, OEN);
        hal_gpio_pull_set(col_pin, PULL_DOWN);
    }

    for (uint8 i = 0; i < MATRIX_KEYBOARD_ROW; i++)
    {
        GPIO_Pin_e row_pin = key_para->rows[i];
        hal_gpio_pin_init(row_pin, IE);
        hal_gpio_pull_set(row_pin, WEAK_PULL_UP);
        hal_gpio_wakeup_set(row_pin, POL_FALLING);
    }
}
