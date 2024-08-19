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
    Filename:       adc_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "adc.h"
#include "adc_compare_demo.h"
#include "log.h"
#include "voice_demo.h"
#include "Voice_Queue.h"
#include "adc_config.h"

#if (APP_RUN_MODE == ADC_RUNMODE_COMPARE)

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
#define MAX_SAMPLE_POINT    64
static uint16_t adc_debug[6][MAX_SAMPLE_POINT];
static uint8_t channel_done_flag = 0;



const static uint16_t threshold_low_compare = 3548;
const static uint16_t threshold_high_compare = 3548;

/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

static uint8 adcDemo_Compare_TaskID;   // Task ID for internal task/event processing
/*
    channel:
    is_differential_mode:
    is_high_resolution:
    [bit7~bit2]=[p20,p15~p11],ignore[bit1,bit0]
    when measure adc(not battery),we'd better use high_resolution.
    when measure battery,we'd better use no high_resolution and keep the gpio alone.

    differential_mode is rarely used,
    if use please config channel as one of [ADC_CH3DIFF,ADC_CH2DIFF,ADC_CH1DIFF],
    and is_high_resolution as one of [0x80,0x20,0x08],
    then the pair of [P20~P15,P14~P13,P12~P11] will work.
    other adc channel cannot work.
*/
static adc_Cfg_t adc_cfg =
{

    .channel = ADC_BIT(ADC_CH2DIFF),
    .is_continue_mode = FALSE,
    .is_differential_mode = ADC_BIT(ADC_CH2DIFF),
    .is_high_resolution = 0x00,

};


/*********************************************************************
    LOCAL FUNCTIONS
*/
static void adc_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void adc_Compare_MeasureTask( void );




/*********************************************************************
    PROFILE CALLBACKS
*/

/*********************************************************************
    PUBLIC FUNCTIONS
*/

static void adc_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    return ;
}

void adc_Compare_Init( uint8 task_id )
{
    adcDemo_Compare_TaskID = task_id;
    adc_Compare_MeasureTask();
}


uint16 adc_Compare_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
    //LOG("adc_ProcessEvent: 0x%x\n",events);

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( adcDemo_Compare_TaskID )) != NULL )
        {
            adc_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & adcMeasureTask_Compare_EVT )
    {
        adc_Compare_MeasureTask();
        return (events ^ adcMeasureTask_Compare_EVT);
    }

    return 0;
}


static void adc_compare_evt(adc_Evt_t* pev)
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

        if(adc_cfg.is_continue_mode == FALSE)
        {
            osal_start_timerEx(adcDemo_Compare_TaskID, adcMeasureTask_Compare_EVT,1000);
        }
    }
}

static void adc_Compare_MeasureTask( void )
{
    int ret;
    LOG("adcMeasureTask COMPARE\n");
    ret = hal_adc_config_channel(adc_cfg, adc_compare_evt);

    if(ret)
    {
        LOG("ret = %d\n",ret);
        return;
    }

    // static uint8_t threshold_flag = 0;
    // ret =   hal_adc_comppare_reset(ADC_CH2DIFF);
    // //  LOG("_______________ret = %08x ",ret);
    // if(threshold_flag == 0)//check low threshold
    // {
    //     hal_adc_compare_enable(ADC_CH2DIFF,0,threshold_low_compare,(threshold_low_compare-5));//adc<threshold_low_compare int
    //     threshold_flag = 1;
    // }
    // else
    // {
    //     if(adc_get_high_threshold_flag() == TRUE)//check high threshold
    //     {
    //         hal_adc_compare_enable(ADC_CH2DIFF,1,(threshold_high_compare+5),threshold_high_compare);//adc>threshold_high_compare int
    //         threshold_flag = 0;
    //     }
    //     else//check low threshold
    //     {
    //         hal_adc_compare_enable(ADC_CH2DIFF,0,threshold_low_compare,(threshold_low_compare-5));//adc<threshold_low_compare int
    //         threshold_flag = 0;
    //     }
    // }
    hal_adc_start(CCOMPARE_MODE);
}

#endif
