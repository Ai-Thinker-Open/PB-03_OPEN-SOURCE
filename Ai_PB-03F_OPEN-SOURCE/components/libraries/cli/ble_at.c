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
    Filename:       ble_at.c
    Revised:
    Revision:

    Description:    This file contains the at ble sample application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "ble_at.h"
#include "at_ble_sbm_cmd.h"
#include "pwrmgr.h"
#include "rf_phy_driver.h"


// Task ID for internal task/event processing
uint8 bleAT_TaskID;


void bleAT_Init( uint8 task_id )
{
    bleAT_TaskID = task_id;
    osal_set_event( bleAT_TaskID, AT_START_DEVICE_EVT );
}

static void ProcessAtUartData(uart_Evt_t* evt)
{
    switch(evt->type)
    {
    case  UART_EVT_TYPE_RX_DATA:
    case  UART_EVT_TYPE_RX_DATA_TO:
    {
        uint16_t c_len = cmdlen + evt->len;

        if(c_len < sizeof(cmdstr))
        {
            osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
            cmdlen += evt->len;
            osal_set_event( bleAT_TaskID, AT_PROCESS_UART_RX_CMD_EVT );
        }
        else
        {
            AT_LOG("c_len error %d  %d  %d \n",c_len,cmdlen,evt->len);
            LOG_DUMP_BYTE(cmdstr,c_len);
            cmdlen = 0;
        }
    }
    break;

    default:
        break;
    }
}

void ble_at_uart_init(void)
{
    hal_uart_deinit(UART0);
    uart_Cfg_t cfg =
    {
        .tx_pin = AT_UART_TX_PIN,
        .rx_pin = AT_UART_RX_PIN,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = ProcessAtUartData,
    };
    hal_uart_init(cfg,UART0);//uart init
    hal_pwrmgr_register(MOD_USR0, at_sleep_handle,  at_wakeup_handle);
}

void connHdlMapRoleInit(void)
{
    for(uint8_t i = 0 ; i < MAX_CONNECTION_NUM; i++)
    {
        at_parameters.conn_param[i].con_role_state = Idle_Role;
    }
}

uint16_t bleAT_ProcessEvent( uint8 task_id, uint16_t events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( bleAT_TaskID )) != NULL )
        {
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & AT_START_DEVICE_EVT )
    {
        ///ble at uart config & at cmd list init
        ble_at_uart_init();
        CLI_init((CLI_COMMAND*)ble_at_cmd_list,(sizeof (ble_at_cmd_list)/sizeof(CLI_COMMAND)));
        connHdlMapRoleInit();
        ///set sleep handler
        // hal_pwrmgr_register(MOD_USR0, at_sleep_handle, at_wakeup_handle);
        hal_pwrmgr_lock(MOD_USR0);///MOD_USR1
        // at_parameters init config
        at_parameters.system_clk_cfg = g_system_clk;
        AT_LOG("ble at init\n");
        return ( events ^ AT_START_DEVICE_EVT );
    }

    if ( events & AT_PROCESS_UART_RX_CMD_EVT )
    {
        if (('\r' == cmdstr[cmdlen - 2]) && ('\n' == cmdstr[cmdlen - 1]))
        {
            // gpio_write(P24,1);gpio_write(P24,0);
            uint16_t i = 0;
            bool at_cmd_flag = false;

            for(i = 0; i < (cmdlen-2); i++)
            {
                if((cmdstr[i] == 'A') && (cmdstr[i+1] == 'T') && (cmdstr[i+2] == '+')) /// wake up flag "AT+"
                {
                    at_cmd_flag = true;
                    break;
                }
            }

            if(at_cmd_flag) /// process at cmd
            {
                cmdstr[cmdlen] = '\0';
                CLI_process_line_manual
                (
                    &cmdstr[i],
                    (cmdlen-i)
                );
                memset(cmdstr, 0, AT_UART_RX_MAX_LEN);
                cmdlen = 0;
                // if( osal_get_timeoutEx(bleAT_TaskID, AT_ENTER_ATUO_SLEEP_MODE_EVT) )
                // {
                //     uint32_t timeout_val = osal_get_timeoutEx(bleAT_TaskID, AT_ENTER_ATUO_SLEEP_MODE_EVT);
                //     if(timeout_val < 5000)
                //     osal_start_timerEx( bleAT_TaskID, AT_ENTER_ATUO_SLEEP_MODE_EVT, timeout_val + 5*1000 );
                // }
            }
            else
            {
                // AT_LOG("TFALSE:");
                // for (size_t i = 0; i < cmdlen; i++)
                // {
                //     AT_LOG("%02x",cmdstr[i]);
                // }
                // AT_LOG("\n");
                memset(cmdstr, 0, AT_UART_RX_MAX_LEN);
                cmdlen = 0;

                if(at_parameters.sleep_mode == 1)
                {
                    osal_start_timerEx( bleAT_TaskID, AT_ENTER_ATUO_SLEEP_MODE_EVT,  1*10 );
                }
            }
        }
        else
        {
            // gpio_write(P25,1);gpio_write(P25,0);
        }

        return ( events ^ AT_PROCESS_UART_RX_CMD_EVT);
    }

    if ( events & AT_ENTER_ATUO_SLEEP_MODE_EVT )
    {
        if(at_parameters.sleep_mode)
        {
            // AT_LOG("unlock mode usr1\n");
            hal_pwrmgr_unlock(MOD_USR0);/// MOD_USR1
        }

        return ( events ^ AT_ENTER_ATUO_SLEEP_MODE_EVT);
    }

    return 0;
}





