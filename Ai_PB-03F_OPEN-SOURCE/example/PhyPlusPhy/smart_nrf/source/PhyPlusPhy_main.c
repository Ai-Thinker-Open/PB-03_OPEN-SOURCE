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
    Filename:       PhyPlusPhy_main.c
    Revised:
    Revision:

    Description:    This file contains the phyplus phy sample application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "rom_sym_def.h"

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "log.h"
#include "timer.h"
#include "phy_plus_phy.h"


#define SNRF_START_TX_EVT           0x0001
#define SNRF_START_RX_EVT           0x0002
#define SNRF_UPDATE_TX_DATA_EVT     0x0004
#define SNRF_SEND_DATA_EVT          0x0008
#define SNRF_STOP_RX_EVT            0x0020
#define SNRF_SEARCH_HIGH_IO_EVT     0x0040

#define SNRF_PREPARED_ACKPDU_NUM    8

uint8_t Smart_nRF_data_process(phy_comm_evt_t* pdata);

uint8_t Smart_nRF_generate_ackpdu(phy_comm_evt_t* packbuf);

// phy_comm_evt_t s_prepared_ackpdu[SNRF_PREPARED_ACKPDU_NUM];
// uint8_t test_prepared_ackpdu[32] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};
#if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE)
    static uint8_t key_pin;
    extern uint16_t s_rf_netid;
#endif
/**************************************************************************************************
    @fn          main

    @brief       Start of application.

    @param       none

    @return      none
 **************************************************************************************************
*/
int app_main(void)
{
    /* Initialize the operating system */
    osal_init_system();
    osal_pwrmgr_device(PWRMGR_BATTERY);
    /* Start OSAL */
    osal_start_system(); // No Return from here
    return 0;
}
uint8_t Smart_nRF_TaskID;

void Smart_nRF_Init(uint8 task_id)
{
    Smart_nRF_TaskID = task_id;
    // #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_DISABLE)
    phy_cbfunc_regist(PHY_DATA_CB,Smart_nRF_data_process);
    // #endif
    #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
    //phy_cbfunc_regist(PHY_OPCODE_CB,Smart_nRF_generate_ackpdu);
    // phy_comm_evt_t a =
    // {
    //  .type = 1,
    //  .len = 31,
    //  .data = test_prepared_ackpdu,
    //  .rssi = 0,
    // };
    // s_prepared_ackpdu[0] = a;
    #endif
    #if(DEF_PHYPLUS_TRX_SUPPORT == PHYPLUS_CONFIG_TRX_ALL)
    {
        if(gpio_read(P0) == 1)
        {
            LOG_DEBUG("SMART_nRF: Start Tx\n");
            osal_set_event(Smart_nRF_TaskID, SNRF_START_TX_EVT);
        }
        else
        {
            LOG_DEBUG("SMART_nRF: Start Rx\n");
            osal_set_event(Smart_nRF_TaskID, SNRF_START_RX_EVT);
        }

        osal_start_timerEx(Smart_nRF_TaskID,SNRF_SEARCH_HIGH_IO_EVT,1000);
        #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE)
        LOG_DEBUG("s_rf_netid = %x\n",s_rf_netid);
        #endif
    }
#elif(DEF_PHYPLUS_TRX_SUPPORT == PHYPLUS_CONFIG_TX)
    LOG_DEBUG("SMART_nRF: Start Tx\n");
    osal_set_event(Smart_nRF_TaskID,SNRF_START_TX_EVT);
    #else
    osal_set_event(Smart_nRF_TaskID,SNRF_START_RX_EVT);
    LOG_DEBUG("SMART_nRF: Start Rx\n");
    #endif
}
#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)
    static uint16_t advCnt=1;
#endif

uint16 Smart_nRF_ProcessEvent(uint8 task_id, uint16 events)
{
    uint8_t ret = PPlus_ERR_NULL;
    VOID task_id;
    #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)

    if(events & SNRF_START_TX_EVT)
    {
// advert data for iBeacon
        static uint8 advertData[] =
        {
            0x02,   // length of this data
            0x01,//GAP_ADTYPE_FLAGS,
            0x06,//DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
            // complete name
            0x09,   // length of this data
            0x09,//GAP_ADTYPE_LOCAL_NAME_COMPLETE,
            0x53,0x6D,0x61,0x72,0x54,0x6E,0x52,0x46,//SmarTnRF
            0x07, // length of this data including the data type byte
            0xff,//GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
            0x04, // Company ID - Fixed
            0x05, // Company ID - Fixed
            0x02, // Data Type - Fixed
            0x02, // Data Length - Fixed
            0x00, // cnt
            0x00, // cnt
        };
        uint8_t dlen = sizeof(advertData);
        advertData[dlen-2]=advCnt>>8;
        advertData[dlen-1]=advCnt&0xff;

        if(advCnt&0x01)
        {
            // phy_adv_opcode_update(advCnt % 9);
            ret=phy_rf_start_tx(advertData,dlen, 0, 0);
        }
        else
        {
            ret=phy_rf_stop_tx();
            ret=PPlus_SUCCESS;
        }

        if(ret==PPlus_SUCCESS)
            advCnt++;

        LOG_DEBUG("%d %d %d\n",ret,advCnt,phy_rf_get_current_status());
        osal_start_timerEx(Smart_nRF_TaskID,SNRF_START_TX_EVT,1000);
        return(events ^ SNRF_START_TX_EVT);
    }

    #endif
    #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)

    if(events & SNRF_START_RX_EVT)
    {
        ret=phy_rf_start_rx(10000);

        if(ret == PPlus_SUCCESS)
        {
            LOG_DEBUG("start rx\n");
        }

        // osal_start_timerEx(Smart_nRF_TaskID,SNRF_STOP_RX_EVT,5);
        return(events ^ SNRF_START_RX_EVT);
    }

    if(events & SNRF_STOP_RX_EVT)
    {
        ret = phy_rf_stop_rx();

        if(ret == PPlus_SUCCESS)
        {
            LOG_DEBUG("stop rx\n");
        }

        osal_start_timerEx(Smart_nRF_TaskID,SNRF_START_RX_EVT,195);
        return ( events ^ SNRF_STOP_RX_EVT);
    }

    #endif

    if ( events & SNRF_SEND_DATA_EVT)
    {
        #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE && DEF_PHYPLUS_TRX_SUPPORT == PHYPLUS_CONFIG_TRX_ALL)
        uint8_t pdata[32];
        osal_memset(pdata,key_pin,32);
        uint16_t target_netid=0;

        switch (key_pin)
        {
        case P7:
            target_netid = (s_rf_netid & 0xFF00) | 0x0001;
            break;

        case P11:
            target_netid = (s_rf_netid & 0xFF00) | 0x0002;
            break;

        case P14:
            target_netid = (s_rf_netid & 0xFF00) | 0x0003;
            break;

        case P20:
            target_netid = (s_rf_netid & 0xFF00) | 0x0004;
            break;

        default:
            break;
        }

        key_pin = NULL;

        if(target_netid == s_rf_netid)
        {
            target_netid = (s_rf_netid & 0xFF00) | 0x0000;
        }

        phy_set_tx_maxtime(210000);
        LOG_DEBUG("target net id: %x\n",target_netid);
        phy_rf_stop_tx();
        osal_stop_timerEx(Smart_nRF_TaskID,SNRF_START_TX_EVT);
        phy_rf_stop_rx();
        osal_stop_timerEx(Smart_nRF_TaskID,SNRF_START_RX_EVT);
        osal_stop_timerEx(Smart_nRF_TaskID,SNRF_STOP_RX_EVT);
        ret = phy_rf_start_tx( pdata, 27, 0, target_netid);
        #endif
        // if(ret == PPlus_SUCCESS)
        // {
        //     LOG_DEBUG("start tx\n");
        // }
        return ( events ^ SNRF_SEND_DATA_EVT);
    }

    if(events & SNRF_SEARCH_HIGH_IO_EVT)
    {
        #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE && DEF_PHYPLUS_TRX_SUPPORT == PHYPLUS_CONFIG_TRX_ALL)

        if(gpio_read(P7) == 1)
        {
            key_pin = P7;
        }
        else if(gpio_read(P11) == 1)
        {
            key_pin = P11;
        }
        else if(gpio_read(P14) == 1)
        {
            key_pin = P14;
        }
        else if(gpio_read(P20) == 1)
        {
            key_pin = P20;
        }
        else
        {
            key_pin = NULL;
        }

        if(key_pin != NULL)
        {
            osal_set_event(Smart_nRF_TaskID,SNRF_SEND_DATA_EVT);
        }

        #endif
        osal_start_timerEx(Smart_nRF_TaskID,SNRF_SEARCH_HIGH_IO_EVT,1000);
        return(events ^ SNRF_SEARCH_HIGH_IO_EVT);
    }

    return 0;
}

uint8_t Smart_nRF_data_process(phy_comm_evt_t* pdata)
{
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT==1)
    LOG_DEBUG("OPCODE=%x  datalen=%d\n",pdata->type,pdata->len);
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    uint8_t status = phy_rf_get_current_status();

    if(status == PHYPLUS_RFPHY_RX_ONLY)
    {
        LOG_DEBUG("It's nrf CB ack:");
        my_dump_byte(pdata->data,pdata->len);
    }
    else
    {
        LOG_DEBUG("It's nrf CB data:");
        my_dump_byte(pdata->data,pdata->len);
    }

    #else

    if(pdata->type == PHYPLUS_STX_DONE_TYPE)
    {
        LOG_DEBUG("STX Done Reporting\n");
        #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE)
        osal_set_event(Smart_nRF_TaskID,SNRF_START_RX_EVT);
        #endif
    }
    else if(PHYPLUS_GET_ACK_BIT(pdata->type))
    {
        LOG_DEBUG("It's rf CB ack:");
        my_dump_byte(pdata->data,pdata->len);
        #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE)
        osal_set_event(Smart_nRF_TaskID,SNRF_START_RX_EVT);
        #endif
    }
    else if(PHYPLUS_GET_NEEDACK_BIT(pdata->type))
    {
        LOG_DEBUG("It's rf CB data:");
        my_dump_byte(pdata->data,pdata->len);
    }
    else if(pdata->len != NULL)
    {
        LOG_DEBUG("It's rf broadcast data:");
        my_dump_byte(pdata->data,pdata->len);
    }
    else                         //TX FAIL
    {
        #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE)
        osal_set_event(Smart_nRF_TaskID,SNRF_START_RX_EVT);
        #endif
    }

    #endif
    #else
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    LOG_DEBUG("It's noack nrf data:");
    my_dump_byte(pdata->data,pdata->len);
    #else
    LOG_DEBUG("It's noack rf data:");
    my_dump_byte(pdata->data,pdata->len);
    #endif
    #endif
    return PPlus_SUCCESS;
}

#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
    // uint8_t Smart_nRF_generate_ackpdu(phy_comm_evt_t *packbuf)
    // {
    //     uint8_t opcode = PHYPLUS_GET_OPCODE(packbuf->type);
    //     if((opcode > SNRF_PREPARED_ACKPDU_NUM) || (s_prepared_ackpdu[opcode-1].len > PHYPLUS_ACK_DATA_MAX_NUM))
    //     {
    //         return PPlus_ERR_INVALID_PARAM;
    //     }
    //     else if(s_prepared_ackpdu[opcode-1].type == NULL)
    //     {
    //         return PPlus_ERR_NULL;
    //     }
    //     else
    //     {
    //         packbuf->len = s_prepared_ackpdu[opcode-1].len;
    //         osal_memcpy(packbuf->data, s_prepared_ackpdu[opcode-1].data, s_prepared_ackpdu[opcode-1].len);
    //     }
    //  return PPlus_SUCCESS;
    // }
#endif

/*********************************************************************
*********************************************************************/
