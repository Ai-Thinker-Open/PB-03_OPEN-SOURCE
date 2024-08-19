#include "error.h"
#include "bsp_button.h"
#include "log.h"
#include "pwrmgr.h"

#define KEY_BUF_INDEX(index)    (index / 32)
#define KEY_BUF_MOD(index)      (index % 32)
#define KEY_COMBINE_LEN         (sizeof(BTN_COMBINE_T) << 3)

uint8_t key_checking_flag = 1;
uint8_t poweroff_key_release = 0;
uint32_t poweroff_keydown_timestamp = 0;

static BTN_T*                   g_btn_ptr = NULL;
static uint8_t                  g_btn_num;

static uint32_t*                g_combing_ptr = NULL;
static uint8_t                  g_combing_start;

// ! app key register callback
bsp_app* bsp_btn_cb = NULL;
static keyboard_cfg* bsp_KeyBoardCfg = NULL;
// ! bsp button register arr
BTN_T usr_sum_btn_array[BSP_TOTAL_BTN_NUM];
static KEY_FIFO_T s_Key;

static uint32_t g_btn_value[(BTN_NUMBER - 1) / 32 + 1];

static uint8_t bsp_btn_map(uint32_t index)
{
    switch(index)
    {
    case BIT(0):
        return 0;

    case BIT(1):
        return 1;

    case BIT(2):
        return 2;

    case BIT(3):
        return 3;

    case BIT(4):
        return 4;

    case BIT(5):
        return 5;

    case BIT(6):
        return 6;

    case BIT(7):
        return 7;

    case BIT(8):
        return 8;

    case BIT(9):
        return 9;

    case BIT(10):
        return 10;

    case BIT(11):
        return 11;

    case BIT(12):
        return 12;

    case BIT(13):
        return 13;

    case BIT(14):
        return 14;

    case BIT(15):
        return 15;

    case BIT(16):
        return 16;

    case BIT(17):
        return 17;

    case BIT(18):
        return 18;

    case BIT(19):
        return 19;

    case BIT(20):
        return 20;

    case BIT(21):
        return 21;

    case BIT(22):
        return 22;

    case BIT(23):
        return 23;

    case BIT(24):
        return 24;

    case BIT(25):
        return 25;

    case BIT(26):
        return 26;

    case BIT(27):
        return 27;

    case BIT(28):
        return 28;

    case BIT(29):
        return 29;

    case BIT(30):
        return 30;

    case BIT(31):
        return 31;
    }

    return 0xFF;
}

static bool bsp_is_key_press(uint8_t index)
{
    uint32_t ComKeyFlag = (g_btn_ptr + index)->KeyConfig & BSP_BTN_CM_CFG;
    uint32_t combine_temp;

    if(ComKeyFlag == 0)
    {
        if(g_btn_value[KEY_BUF_INDEX(index)] & BIT(KEY_BUF_MOD(index)))
        {
            return TRUE;
        }
        else
        {
            return FALSE;
        }
    }
    else
    {
        combine_temp = g_combing_ptr[(index - g_combing_start)];

        for(int i = 0; i < KEY_COMBINE_LEN; i++)
        {
            if(combine_temp & BIT(i))
            {
                index = bsp_btn_map(BIT(i));

                if((g_btn_value[KEY_BUF_INDEX(index)] & BIT(KEY_BUF_MOD(index))) == 0x00)
                {
                    return FALSE;
                }
            }
        }

        return TRUE;
    }
}

bool bsp_set_key_value_by_row_col(uint8_t cols_num,uint8_t row,uint8_t col,bool value)
{
    uint8_t m0;
    uint32_t temp,combine_temp;
    uint32_t index = row * cols_num + col;

    if (KEY_BUF_INDEX(index) >= ((BTN_NUMBER - 1) / 32 + 1))
    {
        LOG("----------->(%d) %d %d\n",__LINE__,index,value);
        return FALSE;
    }

    if(value)
    {
        g_btn_value[KEY_BUF_INDEX(index)] |= BIT(KEY_BUF_MOD(index));
    }
    else
    {
        g_btn_value[KEY_BUF_INDEX(index)] &= ~BIT(KEY_BUF_MOD(index));
    }

    if((g_btn_ptr + index)->KeyConfig & BSP_BTN_CM_CFG)
    {
        combine_temp = g_combing_ptr[(index - g_combing_start)];

        for(int i=0; i<KEY_COMBINE_LEN; i++)
        {
            temp = (combine_temp & BIT(i));

            if(temp)
            {
                m0 = bsp_btn_map(temp);

                if((g_btn_value[KEY_BUF_INDEX(m0)] & BIT(KEY_BUF_MOD(m0))) == 0)
                {
                    return FALSE;
                }
            }
        }
    }

    return TRUE;
}

void bsp_set_key_value_by_index(uint8_t index,bool value)
{
    //LOG("----------->(%d) %d %d\n",__LINE__,index,value);
    if(value)
    {
        g_btn_value[KEY_BUF_INDEX(index)] |= BIT(KEY_BUF_MOD(index));
    }
    else
    {
        g_btn_value[KEY_BUF_INDEX(index)] &= ~BIT(KEY_BUF_MOD(index));
    }
}

bool bsp_InitBtn(BTN_T* sum_btn_array,uint8_t sum_btn_num,uint8_t combine_btn_start,BTN_COMBINE_T* combine_btn_array)
{
    int i;
    s_Key.Read = 0;
    s_Key.Write = 0;

    if( (sum_btn_array == NULL) || (sum_btn_num == 0) || (sum_btn_num > BTN_NUMBER))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    g_btn_ptr = sum_btn_array;
    g_btn_num = sum_btn_num;

    for(i=0; i<sum_btn_num; i++)
    {
        g_btn_ptr[i].State = 0;
        g_btn_ptr[i].Count = 0;
        g_btn_ptr[i].FilterTime = BTN_FILTER_TICK_COUNT;
        #ifdef BSP_BTN_LONG_PRESS_ENABLE
        g_btn_ptr[i].LongCount = 0;

        if((g_btn_ptr[i].KeyConfig & BSP_BTN_LPS_CFG) || (g_btn_ptr[i].KeyConfig & BSP_BTN_LPK_CFG))
        {
            g_btn_ptr[i].LongTime = bsp_KeyBoardCfg->keyboard_param.long_press_start_count;
            g_btn_ptr[i].RepeatSpeed = bsp_KeyBoardCfg->keyboard_param.long_press_keep_count;
            g_btn_ptr[i].RepeatCount = 0;
        }
        else
        {
            g_btn_ptr[i].LongTime = 0;
            g_btn_ptr[i].RepeatSpeed = 0;
            g_btn_ptr[i].RepeatCount = 0;
        }

        #endif
    }

    for(i=0; i<sizeof(g_btn_value)/sizeof(g_btn_value[0]); i++)
    {
        g_btn_value[i] = 0;
    }

    if((combine_btn_array != NULL) && combine_btn_start <= (sum_btn_num - 1))
    {
        g_combing_start = combine_btn_start;
        g_combing_ptr = combine_btn_array;

        for(int i = combine_btn_start; i < sum_btn_num; i++)
        {
            g_btn_ptr[i].KeyConfig |= BSP_BTN_CM_CFG;
        }
    }

    return PPlus_SUCCESS;
}

static void bsp_PutKey(uint8_t _KeyCode)
{
    s_Key.Buf[s_Key.Write] = _KeyCode;

    if (++s_Key.Write  >= KEY_FIFO_SIZE)
    {
        s_Key.Write = 0;
    }
}

uint8_t bsp_GetKey(void)
{
    uint8_t ret;

    if (s_Key.Read == s_Key.Write)
    {
        return BTN_NONE;
    }
    else
    {
        ret = s_Key.Buf[s_Key.Read];

        if (++s_Key.Read >= KEY_FIFO_SIZE)
        {
            s_Key.Read = 0;
        }

        return ret;
    }
}

static void bsp_DetectBtn(uint8_t index)
{
    BTN_T* _pBtn = (g_btn_ptr + index);

    if (bsp_is_key_press(index))
    {
        if (_pBtn->State == 0)
        {
            _pBtn->State = 1;

            if (_pBtn->KeyConfig & BSP_BTN_PD_CFG)
            {
                bsp_PutKey(BSP_BTN_PD_BASE + index);
            }
        }

        #ifdef BSP_BTN_LONG_PRESS_ENABLE

        if (_pBtn->LongTime > 0)
        {
            if (_pBtn->LongCount < _pBtn->LongTime)
            {
                if (++_pBtn->LongCount == _pBtn->LongTime)
                {
                    if (_pBtn->KeyConfig & BSP_BTN_LPS_CFG)
                    {
                        bsp_PutKey(BSP_BTN_LPS_BASE + index);
                    }
                }
            }
            else
            {
                if (_pBtn->RepeatSpeed > 0)
                {
                    if (++_pBtn->RepeatCount >= _pBtn->RepeatSpeed)
                    {
                        _pBtn->RepeatCount = 0;

                        if (_pBtn->KeyConfig & BSP_BTN_LPK_CFG)
                        {
                            bsp_PutKey(BSP_BTN_LPK_BASE + index);
                        }
                    }
                }
            }
        }

        #endif
    }
    else
    {
        if (_pBtn->State == 1)
        {
            _pBtn->State = 0;

            if (_pBtn->KeyConfig & BSP_BTN_UP_CFG)
            {
                bsp_PutKey(BSP_BTN_UP_BASE + index);
            }
        }

        #ifdef BSP_BTN_LONG_PRESS_ENABLE
        _pBtn->LongCount = 0;
        _pBtn->RepeatCount = 0;
        #endif
    }
}

uint8_t bsp_KeyPro(void)
{
    uint8_t ucKeyCode;

    for(int i = 0; i < g_btn_num; i++)
    {
        bsp_DetectBtn(i);
    }

    ucKeyCode = bsp_GetKey();
    return ucKeyCode;
}

bool bsp_KeyEmpty(void)
{
    for(int i = 0; i < g_btn_num; i++)
    {
        BTN_T* _pBtn = (g_btn_ptr + i);
        #ifdef BSP_BTN_LONG_PRESS_ENABLE

        if ((_pBtn->State == 1) || (_pBtn->Count != 0) || (_pBtn->LongCount != 0) ||(_pBtn->RepeatCount != 0))
        #else
        if ((_pBtn->State == 1) || (_pBtn->Count != 0) )
        #endif
        {
            return FALSE;
        }
    }

    if (s_Key.Read == s_Key.Write)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void bsp_button_param_init(keyboard_cfg* cfg)
{
    bsp_KeyBoardCfg = cfg;
    keyboard_register(bsp_KeyBoardCfg);

    // !config user key support status
    for (int i = 0; i < BSP_TOTAL_BTN_NUM; i++)
    {
        if (i == BSP_COMBINE_KEY_ID_0 || i == BSP_COMBINE_KEY_ID_1 || i == BSP_COMBINE_KEY_ID_2 || i == BSP_IR_LEARN_KEY_ID)
        {
            usr_sum_btn_array[i].KeyConfig = (BSP_BTN_PD_CFG | BSP_BTN_UP_CFG | BSP_BTN_LPS_CFG | BSP_BTN_LPK_CFG);
        }
        else
        {
            usr_sum_btn_array[i].KeyConfig = (BSP_BTN_PD_CFG | BSP_BTN_UP_CFG);
        }
    }

    #if (BSP_COMBINE_BTN_NUM > 0)

    // ! combine button config size checking
    if (BSP_COMBINE_BTN_NUM != bsp_KeyBoardCfg->keyboard_param.combine_param.keyboard_combine_num)
    {
        LOG("combine button config error\n");
        return;
    }

    // ! combine button config checking
    for (int i = 0; i < BSP_COMBINE_BTN_NUM; i++)
    {
        if (bsp_KeyBoardCfg->keyboard_param.combine_param.keyboard_combine_btn_array[i] == 0x00)
        {
            LOG("combine button data init error\n");
            return;
        }
    }

    if (PPlus_SUCCESS != bsp_InitBtn(usr_sum_btn_array, BSP_TOTAL_BTN_NUM, BSP_SINGLE_BTN_NUM, bsp_KeyBoardCfg->keyboard_param.combine_param.keyboard_combine_btn_array))
    #else
    if (PPlus_SUCCESS != bsp_InitBtn(usr_sum_btn_array, BSP_TOTAL_BTN_NUM, 0, NULL))
    #endif
    {
        LOG("bsp button init error\n");
    }
}

/*********************************************************************
    @fn      time_exceed

    @brief   void

    @param   void

    @return  void
*/
int time_exceed(unsigned int ref, unsigned int span)
{
    unsigned int now = hal_systick();
    return now >= ref ? (now - ref) > span : (0xffffffff - ref + now) > span;
}

/*********************************************************************
    @fn      key_app_register

    @brief   register key status change callback api

    @param   void

    @return  void
*/
void key_app_register(bsp_app* bsp_app_cb)
{
    if (bsp_app_cb == NULL)
    {
        return;
    }

    bsp_btn_cb = bsp_app_cb;
}

/*********************************************************************
    @fn      poweroff_key_Press_StartPhaseReport

    @brief   poweroff key press report  in start phase

    @param   void

    @return  void
*/
void poweroff_key_Press_StartPhaseReport(void)
{
    // ! bsp button param init
    bsp_button_param_init(bsp_KeyBoardCfg);

    if (READ_MATRIX_KEY <= MATRIX_KEY_MAX && g_system_reset_cause == WAKE_SYSTEM_CAUSE)
    {
        // ! get poweroff key press tick value
        poweroff_keydown_timestamp = hal_systick();
        // ! update row col data into bspbutton  row = READ_MATRIX_KEY%MATRIX_KEYBOARD_ROW | col = READ_MATRIX_KEY/MATRIX_KEYBOARD_ROW
        bsp_set_key_value_by_row_col(MATRIX_KEYBOARD_COL, READ_MATRIX_KEY % MATRIX_KEYBOARD_ROW, READ_MATRIX_KEY / MATRIX_KEYBOARD_ROW, TRUE);
        uint8 poweroffkey_press_index = BSP_BTN_INDEX(bsp_KeyPro());
        //        LOG("[key]:%02d pre\n", poweroffkey_press_index);

        if (bsp_btn_cb && bsp_btn_cb->poweroff_Key_pre_cb)
        {
            bsp_btn_cb->poweroff_Key_pre_cb(poweroffkey_press_index);
        }
    }
}

/*********************************************************************
    @fn      poweroff_key_Release_StartPhaseReport

    @brief   poweroff key release report  in start phase

    @param   void

    @return  void
*/
void poweroff_key_Release_StartPhaseReport(void)
{
    if (poweroff_key_release == 0)
    {
        if (READ_MATRIX_KEY <= MATRIX_KEY_MAX && g_system_reset_cause == WAKE_SYSTEM_CAUSE)
        {
            poweroff_key_release = 1;
            bsp_set_key_value_by_row_col(MATRIX_KEYBOARD_COL, READ_MATRIX_KEY % MATRIX_KEYBOARD_ROW, READ_MATRIX_KEY / MATRIX_KEYBOARD_ROW, FALSE);
            uint8 poweroff_key_release_index = BSP_BTN_INDEX(bsp_KeyPro());
            //          LOG("[key]:%02d rel\n", poweroff_key_release_index);

            if (bsp_btn_cb && bsp_btn_cb->poweroff_Key_rel_cb)
            {
                bsp_btn_cb->poweroff_Key_rel_cb(poweroff_key_release_index);
            }
        }
    }
}

/*********************************************************************
    @fn      pragram_start_checking

    @brief   checking keyboard wheather press or release

    @param   void

    @return  0 -- key release  1-- key press
*/
int8_t pragram_start_checking(GPIO_Pin_e* rows, GPIO_Pin_e* cols)
{
    if (g_system_reset_cause == WAKE_SYSTEM_CAUSE)
    {
        if (READ_MATRIX_KEY <= MATRIX_KEY_MAX)
        {
            if (keyboard_read_once(rows, cols) != 0)
            {
                return 1;
            }
            else
            {
                key_checking_flag = 0;
                return 0;
            }
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }
}

/*********************************************************************
    @fn      power_Off_init_phase_key_checking

    @brief   keyboard release checking  and  keyboard report  in start phase & polling call this function in start phase

    @param   void

    @return  void
*/
// ! key checking on poweon init process
uint8_t power_Off_init_phase_key_checking(GPIO_Pin_e* rows, GPIO_Pin_e* cols)
{
    // ! to report
    if (get_key_checking_Flag() == 0)
    {
        poweroff_key_Release_StartPhaseReport();
        return 0;
    }
    else
    {
        // ! again checking
        if (pragram_start_checking(rows, cols) == 0)
        {
            poweroff_key_Release_StartPhaseReport();
        }

        return 1;
    }
}

/*********************************************************************
    @fn      get_key_checking_Flag

    @brief   get keyboard wheather idle

    @param   void

    @return  1---working  0 -- idle
*/
uint8_t get_key_checking_Flag(void)
{
    return key_checking_flag;
}

/*********************************************************************
    @fn      keybaord_middle_button_data_handle

    @brief   bsp button handle

    @param   void

    @return  void
*/
void keybaord_middle_button_data_handle(void)
{
    uint8 KeyCode;
    KeyCode = bsp_KeyPro();

    if (KeyCode != BTN_NONE)
    {
        if (bsp_btn_cb && bsp_btn_cb->bsp_btn_change_cb)
        {
            bsp_btn_cb->bsp_btn_change_cb(KeyCode);
        }
    }
}
