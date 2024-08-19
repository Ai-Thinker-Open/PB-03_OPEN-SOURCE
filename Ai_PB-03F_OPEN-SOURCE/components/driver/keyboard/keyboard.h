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

#ifndef __KEYBOARD_TASK_H__
#define __KEYBOARD_TASK_H__

#include "bcomdef.h"

// #define KEYBOARD_DEBUG

#ifdef KEYBOARD_DEBUG
    #define KEYBOARD_LOG(...)    LOG(__VA_ARGS__)
#else
    #define KEYBOARD_LOG(...)
#endif

typedef enum
{
    NO_EXTEND  = 0X00,
    EXTEND_GND = 0X01,
} KEYBOARD_MODE_t;

typedef void (*keyboard_event_t)( uint8 col_nums, uint8 row, uint8 col, uint8 key_state );
typedef void (*keyboard_polling_event_t)( void );
typedef void (*keybaord_restore_event_t)( void );
typedef void (*poweroff_keybaord_event_t)( uint8 col_nums, uint8 row, uint8 col, uint8 key_state );
typedef void (*poweroff_keyboard_polling_event_t)( void );
typedef void (*poweroff_keybaord_restore_event_t)( void );

typedef struct poweroff_key_t
{

    poweroff_keybaord_event_t           poweroff_keybaord_change_report;
    poweroff_keyboard_polling_event_t   poweroff_keybaord_polling_report;
    poweroff_keybaord_restore_event_t   poweroff_keybaord_restore_report;
} poweroff_key_report;

typedef struct _key_t
{
    keyboard_event_t             keyboard_change_cb;
    keyboard_polling_event_t     keyboard_polling_cb;
    keybaord_restore_event_t     keybaord_restore_cb;
} normal_key_report;

typedef struct keyboard
{
    uint8_t keyboard_combine_num;
    uint32_t* keyboard_combine_btn_array;
} COMBINE_Btn_cfg_t;

typedef struct _keybaord_param_t
{
    uint8 polling_interval;
    uint8 poweroff_polling_interval;
    uint8 keyboard_osal_taskId;
    COMBINE_Btn_cfg_t combine_param;
    uint16 long_press_start_count;
    uint16 long_press_keep_count;
    uint16 keyboard_osal_event;
    GPIO_Pin_e* rows;
    GPIO_Pin_e* cols;
    KEYBOARD_MODE_t keyboard_mode;
    MODULE_e keyboard_pwrmgr_mod;
} keyboard_param_t;

typedef struct keyboard_cfg_t
{
    normal_key_report* NormalKeyReportCB;
    poweroff_key_report* PowerOffKeyReportCB;
    keyboard_param_t keyboard_param;
} keyboard_cfg;


#define MATRIX_DEBONCE_COUNT              2


void keyboard_register( keyboard_cfg* cfg );

void keyboard_io_read( void );

void keyboard_deep_sleep_handler( void );

uint8 keyboard_read_once( GPIO_Pin_e* matrix_row_to_pin_map, GPIO_Pin_e* matrix_col_to_pin_map );
#endif
