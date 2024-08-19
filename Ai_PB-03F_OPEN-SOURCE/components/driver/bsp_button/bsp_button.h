#ifndef __BSP_BUTTON_H__
#define __BSP_BUTTON_H__

#include "types.h"
#include "keyboard.h"
#include "bsp_gpio.h"

#define BSP_BTN_LONG_PRESS_ENABLE

#define BTN_SYS_TICK                            10  //unit:ms
#define BTN_FILTER_TICK_COUNT                   5   //(BTN_SYS_TICK*BTN_FILTER_TICK_COUNT)            ms

#define MATRIX_KEYBOARD_ROW                  (4) /*only gpio for row */
#define MATRIX_KEYBOARD_COL                  (4) /*only gpio for col */
#define BTN_NUMBER                              32  //valid:[0,0x2F].reserved[0x30,0x3F]
#define BTN_NONE                                0xFF

#if (BTN_NUMBER > 48)
    #error "error bsp button config,please check"
#endif

#define BSP_BTN_JUST_GPIO                                   (0x01)
#define BSP_BTN_JUST_KSCAN                                  (0x02)
#define BSP_BTN_GPIO_AND_KSCAN                              (0x03)
#define BSP_BTN_HARDWARE_CONFIG                             BSP_BTN_JUST_KSCAN

#if (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_GPIO)

    #define BSP_SINGLE_BTN_NUM                                  (GPIO_SINGLE_BTN_NUM)
    #define BSP_COMBINE_BTN_NUM                                 (2)
    #define BSP_TOTAL_BTN_NUM                                   (BSP_SINGLE_BTN_NUM + BSP_COMBINE_BTN_NUM)

#elif  (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_KSCAN)

    #define BSP_SINGLE_BTN_NUM                   (MATRIX_KEYBOARD_ROW * MATRIX_KEYBOARD_COL)
    #define BSP_COMBINE_BTN_NUM                  (2)
    #define BSP_TOTAL_BTN_NUM                    (BSP_SINGLE_BTN_NUM + BSP_COMBINE_BTN_NUM)

#elif  (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN)

    #define BSP_KSCAN_SINGLE_BTN_NUM                            (MATRIX_KEYBOARD_ROW * MATRIX_KEYBOARD_COL)
    #define BSP_GPIO_SINGLE_BTN_NUM                             (GPIO_SINGLE_BTN_NUM)

    #define BSP_KSCAN_COMBINE_BTN_NUM                           (1)
    #define BSP_GPIO_COMBINE_BTN_NUM                            (1)

    #define BSP_SINGLE_BTN_NUM                                  (BSP_KSCAN_SINGLE_BTN_NUM + BSP_GPIO_SINGLE_BTN_NUM)
    #define BSP_COMBINE_BTN_NUM                                 (BSP_KSCAN_COMBINE_BTN_NUM + BSP_GPIO_COMBINE_BTN_NUM)
    #define BSP_TOTAL_BTN_NUM                                   (BSP_SINGLE_BTN_NUM + BSP_COMBINE_BTN_NUM)

#else

    #error "error bsp button config,please check"

#endif

#define MATRIX_KEY_ADDRESS                   (0x4000f0cc)
#define READ_MATRIX_KEY                      (uint16)(read_reg(MATRIX_KEY_ADDRESS) & 0xff)
#define MATRIX_KEY_MAX                        BSP_SINGLE_BTN_NUM

#define BSP_IR_LEARN_KEY_ID                  (13) /* key id for enable ir learn */
#define BSP_COMBINE_KEY_ID_0                 (BSP_SINGLE_BTN_NUM + 0) /* key id for enable keyid */
#define BSP_COMBINE_KEY_ID_1                 (BSP_SINGLE_BTN_NUM + 1) /* key id for enable singlecarrier */
#define BSP_COMBINE_KEY_ID_2                 (BSP_SINGLE_BTN_NUM + 2) /* key id for enable product mode */

#define WAKE_SYSTEM_CAUSE                    (2)

/*
    bit0:press down
    bit1:press up
    bit2:long press start
    bit3:long press keep
    bit4:combine or not
*/
#define BSP_BTN_PD_CFG                         0x01
#define BSP_BTN_UP_CFG                         0x02
#define BSP_BTN_LPS_CFG                        0x04
#define BSP_BTN_LPK_CFG                        0x08
#define BSP_BTN_CM_CFG                         0x10

#define BSP_BTN_PD_TYPE                        (0U<<6)
#define BSP_BTN_UP_TYPE                        (1U<<6)
#define BSP_BTN_LPS_TYPE                       (2U<<6)
#define BSP_BTN_LPK_TYPE                       (3U<<6)

#define BSP_BTN_PD_BASE                       BSP_BTN_PD_TYPE
#define BSP_BTN_UP_BASE                       BSP_BTN_UP_TYPE
#define BSP_BTN_LPS_BASE                      BSP_BTN_LPS_TYPE
#define BSP_BTN_LPK_BASE                      BSP_BTN_LPK_TYPE

#define BSP_BTN_TYPE(key)                    (key & 0xC0)
#define BSP_BTN_INDEX(key)                   (key & 0x3F)

typedef uint32_t BTN_COMBINE_T;

typedef struct
{
    uint8_t KeyConfig;
    uint8_t State;
    uint8_t Count;
    uint8_t FilterTime;

    #ifdef BSP_BTN_LONG_PRESS_ENABLE
    uint16_t LongCount;
    uint16_t LongTime;
    uint16_t RepeatSpeed;
    uint16_t RepeatCount;
    #endif
} BTN_T;


#define KEY_FIFO_SIZE   20
typedef struct
{
    uint8_t Buf[KEY_FIFO_SIZE];
    uint8_t Read;
    uint8_t Write;
} KEY_FIFO_T;

typedef void (*bsp_btn_callback_t)(uint8_t evt);
typedef void (*bsp_Poweroff_Key_Press_callback)(uint8 powerkey_index);
typedef void (*bsp_Poweroff_Key_Relea_callback)(uint8 powerkey_index);
typedef void (*bsp_Poweroff_Key_Longp_callback)(void);

typedef struct bsp_app_t
{
    bsp_btn_callback_t bsp_btn_change_cb;
    bsp_Poweroff_Key_Press_callback poweroff_Key_pre_cb;
    bsp_Poweroff_Key_Relea_callback poweroff_Key_rel_cb;
    bsp_Poweroff_Key_Longp_callback poweroff_Key_lon_cb;
} bsp_app;

extern uint8_t key_checking_flag;
extern uint8_t poweroff_key_release;
extern bsp_app* bsp_btn_cb;
extern uint32_t poweroff_keydown_timestamp;
extern BTN_T usr_sum_btn_array[BSP_TOTAL_BTN_NUM];

bool bsp_InitBtn(BTN_T* sum_btn_array,uint8_t sum_btn_num,uint8_t combine_btn_start,BTN_COMBINE_T* combine_btn_array);
bool bsp_set_key_value_by_row_col(uint8_t cols_num,uint8_t row,uint8_t col,bool value);
bool bsp_KeyEmpty(void);
int8_t pragram_start_checking(GPIO_Pin_e* rows, GPIO_Pin_e* cols);
uint8_t power_Off_init_phase_key_checking(GPIO_Pin_e* rows, GPIO_Pin_e* cols);
uint8_t get_key_checking_Flag(void);
uint8_t bsp_KeyPro(void);
uint8_t bsp_GetKey(void);
void bsp_button_param_init(keyboard_cfg* cfg);
void bsp_set_key_value_by_index(uint8_t index,bool value);
void key_app_register(bsp_app* bsp_app_cb);
void keybaord_middle_button_data_handle(void);
int time_exceed(unsigned int ref, unsigned int span);
void poweroff_key_Press_StartPhaseReport(void);
void poweroff_key_Release_StartPhaseReport(void);

#endif

