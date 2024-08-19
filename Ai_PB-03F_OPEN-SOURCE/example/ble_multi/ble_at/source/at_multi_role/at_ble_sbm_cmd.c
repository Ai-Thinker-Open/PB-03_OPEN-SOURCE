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
    Filename:       at_ble_sbm_cmd.c
    Revised:
    Revision:

    Description:    This file contains the at ble sbm cmd sample application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "log.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"



#include "gapbondmgr.h"
#include "flash.h"
#include "ll.h"
#include "clock.h"
// #include "common.h"
#include "rflib.h"


#include <stdio.h>
#include "ble_at.h"
#include "at_ble_sbm_cmd.h"

#include "at_multi_schedule.h"
// #include "multi.h"
#include "at_svn_write_read.h"
#include "pwrmgr.h"
#include "jump_function.h"
#include "ll_sleep.h"
#include "uart.h"
#include "at_ble_service.h"

unsigned char cmdstr[AT_UART_RX_MAX_LEN];
uint16_t cmdlen;

at_env_param at_parameters;

///   0       1       2       3      4      5     6     7     8     9     10
///  0x01    0x02    0x03    0x04   0x05   0x06  0x08  0x0d  0x0f  0x12  0x1c
/// -20dbm  -15dbm  -10dbm  -6dbm  -5dbm  -3dbm  0dbm  3dbm  4dbm  5dbm  7dbm
uint8_t rf_tx_pwr_table[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x08,0x0d,0x0f,0x12,0x1c};

// static uint8_t addr_type,address[6] = {0};

extern uint8 ownPublicAddr[];
extern uint8 gapMultiRole_TaskID;
extern uint8 multiRole_TaskId;;
extern uint8 gapBond_PairingMode[];

#if ( MAX_CONNECTION_MASTER_NUM > 0 )
    extern uint8 MultiRole_CancelConn(void);
#endif

#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
    extern uint8 multiProfileServUUID[ATT_UUID_SIZE];
    extern uint8 multiProfilechar1UUID[ATT_UUID_SIZE];
    extern uint8 multiProfilechar2UUID[ATT_UUID_SIZE];
    extern uint8 multiProfileChar1Props;
    extern uint8 multiProfileChar2Props;
#endif

extern void ll_dbg_show(void);

/// union
static uint16_t at_set_clk(uint32_t argc, unsigned char* argv[]);
static uint16_t at_reset(uint32_t argc, unsigned char* argv[]);
static uint16_t at_set_mac_fun(uint32_t argc, unsigned char* argv[]);///necessarily?
static uint16_t at_set_rf_tx_power_fun(uint32_t argc, unsigned char* argv[]);///necessarily?
static uint16_t at_white_list(uint32_t argc, unsigned char* argv[]);
static uint16_t at_ll_stack_info(uint32_t argc, unsigned char* argv[]);
static uint16_t at_enter_sleep_mode(uint32_t argc, unsigned char* argv[]);
static uint16_t at_set_smp_mode(uint32_t argc, unsigned char* argv[]);
static uint16_t at_enter_smp(uint32_t argc, unsigned char* argv[]);
#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
    /// adv state
    static uint16_t at_set_adv_parameters(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_set_adv_filter_policy(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_set_peer_mac(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_set_adv_data(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_set_scan_rsp_data(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_set_adv_mode(uint32_t argc, unsigned char* argv[]);
    /// add service & characteristic
    static uint16_t at_add_suuid(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_add_cuuid(uint32_t argc, unsigned char* argv[]);
#endif
#if ( MAX_CONNECTION_MASTER_NUM > 0 )
    /// scan state
    static uint16_t at_set_scan_parameters(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_set_scan_fliter_policy(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_set_scan_mode(uint32_t argc, unsigned char* argv[]);
    /// init state
    static uint16_t at_conn_req(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_conn_fliter_policy(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_conn_cancel(uint32_t argc, unsigned char* argv[]);
#endif
/// conn state
//master & slave role support
static uint16_t at_conn_terminate(uint32_t argc, unsigned char* argv[]);
static uint16_t at_inquiry_curr_conn_param(uint32_t argc, unsigned char* argv[]);
static uint16_t at_dlu(uint32_t argc, unsigned char* argv[]);
static uint16_t at_feature_req(uint32_t argc, unsigned char* argv[]);
static uint16_t at_version_exchange(uint32_t argc, unsigned char* argv[]);
static uint16_t at_phy_req(uint32_t argc, unsigned char* argv[]);

#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
    //only slave role support
    static uint16_t at_conn_para_update(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_notify(uint32_t argc, unsigned char* argv[]);
#endif
#if ( MAX_CONNECTION_MASTER_NUM > 0 )
    //only master role support
    static uint16_t at_mtu(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_channel_map(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_read_req(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_write_req(uint32_t argc, unsigned char* argv[]);
    static uint16_t at_write_cmd(uint32_t argc, unsigned char* argv[]);
#endif

const CLI_COMMAND ble_at_cmd_list[AT_UART_AT_CMD_LIST_ELEM] =
{
    ///union cmd
    { "AT+RESET", "reset system", at_reset},
    { "AT+CLK", "set system clk", at_set_clk},
    { "AT+MAC", "set mac", at_set_mac_fun},
    { "AT+RFTXPW", "set system clk", at_set_rf_tx_power_fun},
    { "AT+WLIST", "white list fun", at_white_list},
    { "AT+LLINFO", "read ll stack info", at_ll_stack_info},
    { "AT+SLEEP", "enter sleep mode", at_enter_sleep_mode},
    { "AT+SETSMP",   "set smp mode", at_set_smp_mode},
    { "AT+ENTERSMP",   "enter smp", at_enter_smp},

    #if ( MAX_CONNECTION_SLAVE_NUM > 0 )
    ///adv cmd
    { "AT+ADVPARA", "set adv parameters", at_set_adv_parameters},
    { "AT+ADVPOLICY", "set adv filter policy", at_set_adv_filter_policy},
    { "AT+PEERMAC", "set peer address", at_set_peer_mac},
    { "AT+ADVDAT", "set adv data", at_set_adv_data},
    { "AT+RSPDAT", "set scan respond data", at_set_scan_rsp_data},
    { "AT+ADV", "set adv mode", at_set_adv_mode},
    { "AT+ADDS", "add service uuid", at_add_suuid},
    { "AT+ADDC", "add characteristic uuid", at_add_cuuid},
    #endif

    #if ( MAX_CONNECTION_MASTER_NUM > 0 )
    ///scan cmd
    { "AT+SCANPARA", "set scan parameters", at_set_scan_parameters},
    { "AT+SCANPOLICY", "set scan filter policy", at_set_scan_fliter_policy},
    { "AT+SCAN", "set scan mode", at_set_scan_mode},

    ///init cmd
    { "AT+CONNPOLICY", "set conn fliter policy", at_conn_fliter_policy},
    { "AT+CANCELCONN", "cancel conn", at_conn_cancel},
    { "AT+CONN", "set conn mode", at_conn_req},
    #endif

    ///conn cmd
    //master & slave role support
    { "AT+DISCONN", "llcp-terminate_conn", at_conn_terminate},
    { "AT+FEAREQ", "llcp-feature_req", at_feature_req},
    { "AT+VEREXC", "llcp-version_exchange", at_version_exchange},
    { "AT+DLU", "llcp-data_length_update", at_dlu},
    { "AT+PHYREQ", "llcp-phy_req", at_phy_req},
    { "AT+INQCONNPARAM", "update conn parameters", at_inquiry_curr_conn_param},

    #if ( MAX_CONNECTION_SLAVE_NUM > 0 )
    //only slave role support
    { "AT+UPDATECONN", "update conn parameters", at_conn_para_update},
    { "AT+NOTIFY", "notify", at_notify},
    #endif

    #if ( MAX_CONNECTION_MASTER_NUM > 0 )
    //only master role support
    { "AT+MTU", "set mtu size", at_mtu},
    { "AT+CHANMAP", "llcp-channel_map", at_channel_map},
    { "AT+RREQ", "read req", at_read_req},
    { "AT+WREQ", "write req", at_write_req},
    { "AT+WCMD", "write cmd", at_write_cmd},
    #endif

};

static uint16_t at_set_clk(uint32_t argc, unsigned char* argv[])
{
    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        sysclk_t cli_system_clk = (sysclk_t)CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(cli_system_clk > SYS_CLK_NUM)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        at_parameters.system_clk_cfg = cli_system_clk;
        uint8_t st = hal_system_clock_change_active(cli_system_clk,clk_change_mod_restore);
        AT_LOG("CLK CHANGE %d %d %d\n",g_system_clk,g_system_clk_change,st);
        // at_snv_write_flash(AT_SNV_ID_CLK_SET_OFFSET,4,&clk_set_value);
        AT_LOG("\r\n+OK=%d\r\n",cli_system_clk);
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d\r\n",g_system_clk);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

static uint16_t at_reset(uint32_t argc, unsigned char* argv[])
{
    AT_LOG("match\r\n");
    // if(argv[0][0])
    // {
    //   AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
    //   return ERROR_INVALID_PARAMETERS_NUM;
    // }
    AT_LOG("\r\n+OK\r\n");
    NVIC_SystemReset();
    return OK_RETURN_SUCCESS_STATE;
}
/// public_addr[6] LSB -- MSB   effect after reset adv , only modify adv adress, not really mac addr
static uint16_t at_set_mac_fun(uint32_t argc, unsigned char* argv[])
{
    __attribute__ ((aligned(4))) uint8_t public_addr[6]= {0};

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        uint8_t return_status = CLI_strtoarray(argv[1], CLI_strlen(argv[1]), public_addr,6);

        if(return_status != OK_RETURN_SUCCESS_STATE)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        osal_memcpy(ownPublicAddr,public_addr,6);
        osal_memcpy(at_parameters.bd_addr,public_addr,6);
        AT_LOG("MAC ADDR:");

        for(uint8_t i = 0; i < 6; i++)
        {
            AT_LOG("%02X",at_parameters.bd_addr[i]);
        }

        AT_LOG("\n");
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        // actually mac
        // uint8 *p = (uint8 *)pGlobal_config[MAC_ADDRESS_LOC];
        // public_addr[3] = *(p++);
        // public_addr[2] = *(p++);
        // public_addr[1] = *(p++);
        // public_addr[0] = *(p++);
        // public_addr[5] = *(p++);
        // public_addr[4] = *(p);
        AT_LOG("\r\n+OK=");

        for(uint8_t i = 0; i < 6; i++)
        {
            AT_LOG("%02X",at_parameters.bd_addr[i]);
        }

        AT_LOG("\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// rf_tx_power
static uint16_t at_set_rf_tx_power_fun(uint32_t argc, unsigned char* argv[])
{
    uint8_t rf_tx_power;

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        rf_tx_power = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(rf_tx_power > 10)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        at_parameters.tx_power = rf_tx_pwr_table[rf_tx_power];
        rf_phy_set_txPower(at_parameters.tx_power);
        AT_LOG("\r\n+OK=%d\r\n",at_parameters.tx_power);
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d\r\n",at_parameters.tx_power);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

static uint16_t at_white_list(uint32_t argc, unsigned char* argv[])
{
    uint8_t addr_type,address[6] = {0};

    if(argv[0][0] == '=')
    {
        if(0 == CLI_STR_COMPARE(argv[1], "clear"))
        {
            if(argc != 2)
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
                return ERROR_INVALID_PARAMETERS_NUM;
            }

            ///clear white list
            HCI_LE_ClearWhiteListCmd();
        }
        else if(0 == CLI_STR_COMPARE(argv[1], "add"))
        {
            if(argc != 4)
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
                return ERROR_INVALID_PARAMETERS_NUM;
            }

            /// Own address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
            addr_type =  CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
            uint8_t return_status = CLI_strtoarray(argv[3], CLI_strlen(argv[3]), address,6);

            if(addr_type > 3 || return_status != OK_RETURN_SUCCESS_STATE)
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                return ERROR_INVALID_PARAMETERS;
            }

            HCI_LE_AddWhiteListCmd(addr_type,address);
        }
        else if(0 == CLI_STR_COMPARE(argv[1], "remove"))
        {
            if(argc != 4)
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
                return ERROR_INVALID_PARAMETERS_NUM;
            }

            /// Own address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
            addr_type =  CLI_strtoi(argv[2], CLI_strlen(argv[2]),10);
            uint8_t return_status = CLI_strtoarray(argv[3], CLI_strlen(argv[3]), address,6);

            if(addr_type > 3 || return_status != OK_RETURN_SUCCESS_STATE)
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                return ERROR_INVALID_PARAMETERS;
            }

            HCI_LE_RemoveWhiteListCmd(addr_type,address);
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

static uint16_t at_ll_stack_info(uint32_t argc, unsigned char* argv[])
{
    if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        ll_dbg_show();
        extern uint32_t  __initial_sp;
        uint32_t* initial_sp_addr = &__initial_sp;
        uint32_t* g_intstackbase_addr = (initial_sp_addr - 0x200); ///0x200 means: (Stack_Size(0x800) / 4)
        AT_LOG("ll_Stack total : %d Bytes,",((initial_sp_addr - g_intstackbase_addr)<<2));

        for(uint32_t* addr=g_intstackbase_addr; addr<initial_sp_addr; addr++)
        {
            if(*addr != 0 )
            {
                AT_LOG("ll_used : %d Bytes\n",((initial_sp_addr - addr) << 2));
                break;
            }
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// 1.sleep_mode-- 0x00:exti sleep mode  0x01:deep sleep mode  0x02:system off mode
static uint16_t at_enter_sleep_mode(uint32_t argc, unsigned char* argv[])
{
    uint8_t sleep_mode;

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        sleep_mode = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(sleep_mode > 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(sleep_mode == 1)
        {
            at_parameters.sleep_mode = 1;
            osal_start_timerEx( bleAT_TaskID, AT_ENTER_ATUO_SLEEP_MODE_EVT, 5*100 );
        }
        else if (sleep_mode == 2)
        {
            at_parameters.sleep_mode = 2;
            // pwrmgr_unlock(MOD_USR1); // necessarily?
            hal_gpio_fmux(AT_UART_WAKEUP_PIN, Bit_DISABLE);
            pwroff_cfg_t io_cfg;
            io_cfg.pin = AT_UART_WAKEUP_PIN;
            io_cfg.type = NEGEDGE;
            hal_pwrmgr_poweroff(&io_cfg,1);
        }
        else
        {
            at_parameters.sleep_mode = 0;
            hal_pwrmgr_lock(MOD_USR0); ///  MOD_USR1
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d\r\n",at_parameters.sleep_mode);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}


// 1.ioCap 2.passkey 3.mitm 4.bonding_mode 5.oob_flag
static uint16_t at_set_smp_mode(uint32_t argc, unsigned char* argv[])
{
    if(argv[0][0] == '=')
    {
        if(argc != 6)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        {
            uint8 syncWL = TRUE;
            uint8 ioCap = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
            uint32 passkey = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
            uint8 mitm = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 10);
            uint8 bonding_mode = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 10);
            uint8 oob_flag = CLI_strtoi(argv[5], CLI_strlen(argv[5]), 10);
            uint8_t oob_data[16] = {0};

            if( ioCap > 4 || mitm > 1 || bonding_mode > 1 )
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                return ERROR_INVALID_PARAMETERS;
            }

            GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
            GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
            GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
            GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding_mode );
            // If a bond is created, the HID Device should write the address of the
            // HID Host in the HID Device controller's white list and set the HID
            // Device controller's advertising filter policy to 'process scan and
            // connection requests only from devices in the White List'.
            GAPBondMgr_SetParameter( GAPBOND_AUTO_SYNC_WL, sizeof( uint8 ), &syncWL );
            GAPBondMgr_SetParameter( GAPBOND_OOB_ENABLED, sizeof( uint8 ), &oob_flag );
            GAPBondMgr_SetParameter( GAPBOND_OOB_DATA, 16, &oob_data );
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// slave role: slave req pair req
/// master role: pair req
// 1.conn_handle
static uint16_t at_enter_smp(uint32_t argc, unsigned char* argv[])
{
    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        {
            uint8_t role = Idle_Role;
            uint16_t connectionHandle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

            if( connectionHandle >= 0x0EFF )
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                return ERROR_INVALID_PARAMETERS;
            }

            if(at_parameters.conn_param[connectionHandle].con_role_state == Slave_Role)
            {
                role = GAP_PROFILE_PERIPHERAL;
            }
            else if(at_parameters.conn_param[connectionHandle].con_role_state == Master_Role)
            {
                role = GAP_PROFILE_CENTRAL;
            }
            else
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
                return ERROR_UNSUPPORTED;
            }

            gapBond_PairingMode[ connectionHandle ] = GAPBOND_PAIRING_MODE_INITIATE;
            uint8 bondret = GAPBondMgr_LinkEst( at_parameters.conn_param[connectionHandle].at_conn_params.peer_addr_type, \
                                                at_parameters.conn_param[connectionHandle].at_conn_params.peer_addr, connectionHandle, role );
            LOG("GAPBondMgr_LinkEst bondret %d\n",bondret);
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
// 1.adv_interval 2.adv_type 3.adv_channel_map
static uint16_t at_set_adv_parameters(uint32_t argc, unsigned char* argv[])
{
    uint16_t adv_interval;
    uint8_t adv_type,adv_channel_map;

    if(argv[0][0] == '=')
    {
        if(argc != 4)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        adv_interval = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        adv_type = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
        adv_channel_map = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 10);

        if(adv_type >= AT_ADV_MAX_TYPE || adv_channel_map > 7)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        at_parameters.adv_param.adv_int = adv_interval;
        at_parameters.adv_param.adv_type = adv_type;
        at_parameters.adv_param.adv_chnl_map = adv_channel_map;
        // advertising channel map
        GAPMultiRole_SetParameter( GAPMULTIROLE_ADV_CHANNEL_MAP, sizeof(uint8), &at_parameters.adv_param.adv_chnl_map);
        // only support undirect connectable advertise ?
        GAPMultiRole_SetParameter( GAPMULTIROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &at_parameters.adv_param.adv_type);
        // limit & general & connection discovery mode use the same advertising interval
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN,at_parameters.adv_param.adv_int);
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX,at_parameters.adv_param.adv_int);
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN,at_parameters.adv_param.adv_int);
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN,at_parameters.adv_param.adv_int);
        GAP_SetParamValue( TGAP_CONN_ADV_INT_MIN,at_parameters.adv_param.adv_int);
        GAP_SetParamValue( TGAP_CONN_ADV_INT_MAX,at_parameters.adv_param.adv_int);
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d %d %d\r\n",at_parameters.adv_param.adv_int,at_parameters.adv_param.adv_type,at_parameters.adv_param.adv_chnl_map);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// adv_filter_policy: 0--allow all devices; 1--allow all CI and WL scan req devices; 2--allow all scan req and WL CI devices; 3--only allow WL decives
static uint16_t at_set_adv_filter_policy(uint32_t argc, unsigned char* argv[])
{
    uint8_t adv_filter_policy;

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        adv_filter_policy = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(adv_filter_policy > 4)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        at_parameters.adv_param.adv_fliter_policy = adv_filter_policy;
        GAPMultiRole_SetParameter( GAPMULTIROLE_ADV_FILTER_POLICY, sizeof(uint8), &at_parameters.adv_param.adv_fliter_policy);
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d\r\n",at_parameters.adv_param.adv_fliter_policy);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

///peer_addr_type   peer_addr
static uint16_t at_set_peer_mac(uint32_t argc, unsigned char* argv[])
{
    uint8_t peer_addr_type;
    struct at_bd_addr peer_addr;

    if(argv[0][0] == '=')
    {
        if(argc != 3)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        peer_addr_type = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        uint8_t return_status = CLI_strtoarray(argv[2], CLI_strlen(argv[2]), peer_addr.addr,6);

        if(peer_addr_type >= AT_PEER_MAX_ADDR_TYPE || return_status != OK_RETURN_SUCCESS_STATE)
        {
            // AT_LOG("peer type %d return status %d\n",peer_addr_type,return_status);
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        at_parameters.adv_param.peer_addr_type = peer_addr_type;
        osal_memcpy(at_parameters.adv_param.peer_addr.addr,peer_addr.addr,6);
        AT_LOG("PEER ADDR:");

        for(uint8_t i = 0; i < 6; i++)
        {
            AT_LOG("%02X",at_parameters.adv_param.peer_addr.addr[i]);
        }

        AT_LOG("\n");
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d %02x%02x%02x%02x%02x%02x\r\n",at_parameters.adv_param.peer_addr_type,at_parameters.adv_param.peer_addr.addr[0],at_parameters.adv_param.peer_addr.addr[1],\
               at_parameters.adv_param.peer_addr.addr[2],at_parameters.adv_param.peer_addr.addr[3],at_parameters.adv_param.peer_addr.addr[4],at_parameters.adv_param.peer_addr.addr[5]);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// adv_idx  total_length  adv_data
static uint16_t at_set_adv_data(uint32_t argc, unsigned char* argv[])
{
    uint8_t adv_idx,total_adv_len,adv_data_value[31] = {0};

    if(at_parameters.conn_slave_counter >= MAX_CONNECTION_SLAVE_NUM)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_OVERRUN);
        return ERROR_OVERRUN;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 4)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        adv_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        total_adv_len = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);

        if(total_adv_len > 31 || adv_idx > MAX_CONNECTION_SLAVE_NUM)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        uint8_t return_status = CLI_strtoarray(argv[3], CLI_strlen(argv[3]), adv_data_value,total_adv_len);

        if(return_status != OK_RETURN_SUCCESS_STATE)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        osal_memcpy(&at_parameters.adv_data[adv_idx][0],adv_data_value,total_adv_len);
        // AT_LOG("set adv data:");
        // for(uint8_t i = 0; i < total_adv_len; i++)
        // {
        //   AT_LOG("%02X",at_parameters.adv_data[adv_idx][i]);
        // }
        // AT_LOG("\n");
        //set adv data
        multiSchedule_advParam_init(adv_idx,GAPMULTIROLE_ADVERT_DATA,sizeof( at_parameters.adv_data[adv_idx] ), &at_parameters.adv_data[adv_idx][0]);
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1] < '0' ||  argv[0][1] > '9')
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        adv_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(adv_idx > MAX_CONNECTION_SLAVE_NUM)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        AT_LOG("\r\n+OK=");

        for(uint8_t i = 0; i < 31; i++)
        {
            AT_LOG("%02X",at_parameters.adv_data[adv_idx][i]);
        }

        AT_LOG("\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// adv_idx  total_length  scan_rsp_data
static uint16_t at_set_scan_rsp_data(uint32_t argc, unsigned char* argv[])
{
    uint8_t adv_idx,total_scan_rsp_len,scan_rsp_data_value[31] = {0};

    if(at_parameters.conn_slave_counter >= MAX_CONNECTION_SLAVE_NUM)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_OVERRUN);
        return ERROR_OVERRUN;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 4)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        adv_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        total_scan_rsp_len = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);

        if(total_scan_rsp_len > 31 || adv_idx > MAX_CONNECTION_SLAVE_NUM)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        uint8_t return_status = CLI_strtoarray(argv[3], CLI_strlen(argv[3]), scan_rsp_data_value,total_scan_rsp_len);

        if(return_status != OK_RETURN_SUCCESS_STATE)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        osal_memcpy(&at_parameters.scan_rsp_data[adv_idx][0],scan_rsp_data_value,total_scan_rsp_len);
        //set scan rsp data
        multiSchedule_advParam_init(adv_idx,GAPMULTIROLE_SCAN_RSP_DATA,sizeof( at_parameters.scan_rsp_data[adv_idx] ), &at_parameters.scan_rsp_data[adv_idx][0]);
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1] < '0' ||  argv[0][1] > '9')
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        adv_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(adv_idx > MAX_CONNECTION_SLAVE_NUM)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        AT_LOG("\r\n+OK=");

        for(uint8_t i = 0; i < 31; i++)
        {
            AT_LOG("%02X",at_parameters.scan_rsp_data[adv_idx][i]);
        }

        AT_LOG("\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}
/// adv_idx  adv_mode
static uint16_t at_set_adv_mode(uint32_t argc, unsigned char* argv[])
{
    uint8_t adv_idx,adv_mode;

    if(at_parameters.conn_slave_counter >= MAX_CONNECTION_SLAVE_NUM)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_OVERRUN);
        return ERROR_OVERRUN;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 3)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        adv_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        adv_mode = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);

        if(adv_mode > 1 || adv_idx > MAX_CONNECTION_SLAVE_NUM)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        ///enable -- insert adv node
        ///disable -- delete adv node   if conn, disable adv invalid
        ///terminate -- reinsert adv node  then, disable adv valid
        if(adv_mode == 1)
        {
            if(at_parameters.adv_param.adv_mode[adv_idx][0])
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_DUPLICATE_DEFINITION);
                return ERROR_DUPLICATE_DEFINITION;
            }

            uint32_t add_adv_node = 0;
            at_parameters.adv_param.adv_mode[adv_idx][0] = adv_mode;
            add_adv_node = (add_adv_node&0xFFFFFFFF) | 1<<(adv_idx+4);
            add_adv_node |= 0x01;

            if( SCH_SUCCESS == muliSchedule_config( MULTI_SCH_ADV_MODE, add_adv_node ) )
            {
                AT_LOG("Multi Role advertising scheduler success :%x\n",add_adv_node);
            }
        }
        else if (adv_mode == 0)
        {
            if(!at_parameters.adv_param.adv_mode[adv_idx][0])
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_DUPLICATE_DEFINITION);
                return ERROR_DUPLICATE_DEFINITION;
            }

            uint32_t add_adv_node = 0;
            at_parameters.adv_param.adv_mode[adv_idx][0] = adv_mode;
            add_adv_node = (add_adv_node&0xFFFFFFFF) | 1<<(adv_idx+4);
            add_adv_node &= ~(0x00000001);

            if( SCH_SUCCESS == muliSchedule_config( MULTI_SCH_ADV_MODE, add_adv_node ) )
            {
                AT_LOG("Multi Role delete adv :%x\n",add_adv_node);
            }
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1] < '0' ||  argv[0][1] > '9')
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        adv_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(adv_idx > MAX_CONNECTION_SLAVE_NUM)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        AT_LOG("\r\n+OK=%d\r\n",at_parameters.adv_param.adv_mode[adv_idx][0]);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// 1.suuid_len 2.service_uuid
static uint16_t at_add_suuid(uint32_t argc, unsigned char* argv[])
{
    uint8_t suuid_len,return_status;
    /// Bluetooth Base UUID
    uint8_t service_uuid[16] = {0xFB,0x34,0x9B,0x5F,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00};

    if(at_parameters.conn_slave_counter >= 1)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
        return ERROR_UNSUPPORTED;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 3)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        suuid_len = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if( suuid_len == 16 || suuid_len == 2 )
        {
            /// ok
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d %d\r\n",ERROR_INVALID_PARAMETERS,suuid_len);
            return ERROR_INVALID_PARAMETERS;
        }

        if(suuid_len == 2 )
            return_status = CLI_strtoarray(argv[2], CLI_strlen(argv[2]), (uint8_t*)&service_uuid[12],suuid_len);
        else
            return_status = CLI_strtoarray(argv[2], CLI_strlen(argv[2]), service_uuid,suuid_len);

        if( return_status != OK_RETURN_SUCCESS_STATE )
        {
            AT_LOG("\r\n+ERROR=%d %d\r\n",ERROR_INVALID_PARAMETERS,return_status);
            return ERROR_INVALID_PARAMETERS;
        }

        osal_memcpy(multiProfileServUUID,service_uuid,16);
        at_parameters.ser_charac_param.suuid_length = suuid_len;
        ///flash write necessarily?
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        if(at_parameters.ser_charac_param.suuid_length == 2)
        {
            AT_LOG("\r\n+OK=");

            for(uint8_t i = 0; i < 2; i++)
            {
                AT_LOG("%02x",multiProfileServUUID[12+i]);
            }

            AT_LOG("\r\n");
        }
        else
        {
            AT_LOG("\r\n+OK=");

            for(uint8_t i = 0; i < 16; i++)
            {
                AT_LOG("%02x",multiProfileServUUID[i]);
            }

            AT_LOG("\r\n");
        }
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// 1.charac_idx 2.cuuid_len 3.charac_uuid 4.charac_permis
static uint16_t at_add_cuuid(uint32_t argc, unsigned char* argv[])
{
    uint8_t charac_idx,cuuid_len,charac_permis,return_status;
    /// Bluetooth Base UUID
    uint8_t charac_uuid[16] = {0xFB,0x34,0x9B,0x5F,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00};

    if(at_parameters.conn_slave_counter >= 1)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
        return ERROR_UNSUPPORTED;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 5)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        charac_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        cuuid_len = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
        charac_permis = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 16);

        if( cuuid_len == 16 || cuuid_len == 2 )
        {
            if(charac_idx > 1)
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                return ERROR_INVALID_PARAMETERS;
            }
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(cuuid_len == 2 )
            return_status = CLI_strtoarray(argv[3], CLI_strlen(argv[3]), (uint8_t*)&charac_uuid[12],cuuid_len);
        else
            return_status = CLI_strtoarray(argv[3], CLI_strlen(argv[3]), charac_uuid,cuuid_len);

        if( return_status != OK_RETURN_SUCCESS_STATE )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(charac_idx == 0)
        {
            osal_memcpy(multiProfilechar1UUID,charac_uuid,16);
            multiProfileChar1Props = charac_permis;
            at_parameters.ser_charac_param.first_cuuid_length = cuuid_len;
        }
        else
        {
            osal_memcpy(multiProfilechar2UUID,charac_uuid,16);
            multiProfileChar2Props = charac_permis;
            at_parameters.ser_charac_param.second_cuuid_length = cuuid_len;
        }

        ///flash write necessarily?
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1] < '0' ||  argv[0][1] > '9')
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        charac_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(charac_idx == 0)
        {
            if(at_parameters.ser_charac_param.first_cuuid_length == 2)
            {
                AT_LOG("\r\n+OK=");

                for(uint8_t i = 0; i < 2; i++)
                {
                    AT_LOG("%02x",multiProfilechar1UUID[12+i]);
                }

                AT_LOG("\r\n");
            }
            else
            {
                AT_LOG("\r\n+OK=");

                for(uint8_t i = 0; i < 16; i++)
                {
                    AT_LOG("%02x",multiProfilechar1UUID[i]);
                }

                AT_LOG("\r\n");
            }
        }
        else if (charac_idx == 1)
        {
            if(at_parameters.ser_charac_param.second_cuuid_length == 2)
            {
                AT_LOG("\r\n+OK=");

                for(uint8_t i = 0; i < 2; i++)
                {
                    AT_LOG("%02x",multiProfilechar2UUID[12+i]);
                }

                AT_LOG("\r\n");
            }
            else
            {
                AT_LOG("\r\n+OK=");

                for(uint8_t i = 0; i < 16; i++)
                {
                    AT_LOG("%02x",multiProfilechar2UUID[i]);
                }

                AT_LOG("\r\n");
            }
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

#endif

#if ( MAX_CONNECTION_MASTER_NUM > 0 )
/// scan_interval  scan_window  scan_duration
static uint16_t at_set_scan_parameters(uint32_t argc, unsigned char* argv[])
{
    volatile  uint16_t scan_interval,scan_window,scan_duration;

    if(at_parameters.conn_master_counter >= MAX_CONNECTION_MASTER_NUM)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_OVERRUN);
        return ERROR_OVERRUN;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 4)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        scan_interval = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        scan_window = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
        scan_duration = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 10);

        if(scan_window > scan_interval)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        at_parameters.scan_param.scan_int = scan_interval;
        at_parameters.scan_param.scan_window = scan_window;
        #if ( MAX_CONNECTION_SLAVE_NUM > 0 )
        uint32_t adv_int = (at_parameters.adv_param.adv_int * 625)/1000;

        if(at_parameters.adv_param.adv_int > 0 &&  adv_int < scan_duration)
            at_parameters.scan_param.scan_duration = adv_int - 5;
        else
            at_parameters.scan_param.scan_duration = scan_duration;

        #endif
        /// scan_interval  scan_window = N*0.625ms
        GAP_SetParamValue( TGAP_GEN_DISC_SCAN_INT, at_parameters.scan_param.scan_int );
        GAP_SetParamValue( TGAP_GEN_DISC_SCAN_WIND, at_parameters.scan_param.scan_window );
        GAP_SetParamValue( TGAP_LIM_DISC_SCAN_INT, at_parameters.scan_param.scan_int );
        GAP_SetParamValue( TGAP_LIM_DISC_SCAN_WIND, at_parameters.scan_param.scan_window );
        /// scan duration = N*1ms
        GAP_SetParamValue( TGAP_GEN_DISC_SCAN, at_parameters.scan_param.scan_duration );
        GAP_SetParamValue( TGAP_LIM_DISC_SCAN, at_parameters.scan_param.scan_duration );
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d %d %d\r\n",at_parameters.scan_param.scan_int,at_parameters.scan_param.scan_window,at_parameters.scan_param.scan_duration);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// scan_type  scan_fliter_policy
static uint16_t at_set_scan_fliter_policy(uint32_t argc, unsigned char* argv[])
{
    uint8_t scan_type,scan_fliter_policy;

    if(at_parameters.conn_master_counter >= MAX_CONNECTION_MASTER_NUM)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_OVERRUN);
        return ERROR_OVERRUN;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 3)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        scan_type = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        scan_fliter_policy = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);

        if(scan_type > 1 || scan_fliter_policy > 1)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        GAPMultiRole_SetParameter(GAPMULTIROLE_ACTIVE_SCAN,sizeof( uint8 ), &scan_type);
        GAPMultiRole_SetParameter(GAPMULTIROLE_SCAN_WHITELIST,sizeof( uint8 ), &scan_fliter_policy);
        at_parameters.scan_param.scan_type = scan_type;
        at_parameters.scan_param.scan_fliter_policy = scan_fliter_policy;
        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d %d %d\r\n",at_parameters.scan_param.scan_type,at_parameters.scan_param.scan_fliter_policy);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// scan_mode
static uint16_t at_set_scan_mode(uint32_t argc, unsigned char* argv[])
{
    uint8_t scan_mode;

    if(at_parameters.conn_master_counter >= MAX_CONNECTION_MASTER_NUM)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_OVERRUN);
        return ERROR_OVERRUN;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        scan_mode = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(scan_mode > 1)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(scan_mode == 1)
        {
            if(at_parameters.scan_param.scan_mode)
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_DUPLICATE_DEFINITION);
                return ERROR_DUPLICATE_DEFINITION;
            }

            if( SCH_SUCCESS == muliSchedule_config( MULTI_SCH_SCAN_MODE, 0x01 ) )
            {
                at_parameters.scan_param.scan_mode = 1;
                AT_LOG("insert scan node success\n");
            }
        }
        // thinkabout scan node process when connection is established
        // only support one connection at now
        else if (scan_mode == 0)
        {
            if(!at_parameters.scan_param.scan_mode)
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_DUPLICATE_DEFINITION);
                return ERROR_DUPLICATE_DEFINITION;
            }

            uint8_t ret = GAPMultiRole_CancelDiscovery();

            if( SUCCESS == ret )
            {
                if( SCH_SUCCESS == muliSchedule_config( MULTI_SCH_ADV_MODE, 0x00 ) )
                {
                    at_parameters.scan_param.scan_mode = 0;
                    AT_LOG("delete scan node success\n");
                }
            }
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1])
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        AT_LOG("\r\n+OK=%d\r\n",at_parameters.scan_param.scan_mode);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}
/// peer_addr_type  peer_addr  con_intv  con_latency  superv_to

static uint16_t at_conn_req(uint32_t argc, unsigned char* argv[])
{
    ///Peer address type - public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    uint8_t        peer_addr_type;
    ///Peer BD address
    uint8_t        peer_addr[6];
    ///connection interval (N * 1.25 ms)  range: 6~3200
    uint16_t       con_intv;
    ///Connection latency
    uint16_t       con_latency;
    ///Link supervision timeout(N * 10 ms)
    uint16_t       superv_to;

    if(at_parameters.conn_master_counter >= MAX_CONNECTION_MASTER_NUM)
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_OVERRUN);
        return ERROR_OVERRUN;
    }

    if(argv[0][0] == '=')
    {
        if(argc != 6)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        peer_addr_type = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        uint8_t return_status = CLI_strtoarray(argv[2], CLI_strlen(argv[2]), peer_addr,6);
        con_intv = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 10);
        con_latency = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 10);
        superv_to = CLI_strtoi(argv[5], CLI_strlen(argv[5]), 10);

        if(peer_addr_type > 3 || return_status != OK_RETURN_SUCCESS_STATE)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        multiDelSlaveConnList(peer_addr);///prevent repeated insertion of the same address
        multiAddSlaveConnList(peer_addr_type,peer_addr);

        if(!multiLinkGetMasterConnNum())
        {
            GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, con_intv );
            GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, con_intv );
            GAP_SetParamValue( TGAP_CONN_EST_LATENCY, con_latency );
            GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT,superv_to );
        }

        // add init node? scan done while add init noode
        if( multiGetSlaveConnList() != NULL )
        {
            ///bugfix: multi add init node 2022 08 05
            uint8_t scan_init_node_num = 0, curr_master_conn_num = 0;
            scan_init_node_num =  multiRole_findInitScanNode();
            curr_master_conn_num = multiLinkGetMasterConnNum();

            if(scan_init_node_num < (MAX_CONNECTION_MASTER_NUM - curr_master_conn_num))
                muliSchedule_config( MULTI_SCH_INITIATOR_MODE, 0x01 );
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
        return ERROR_UNSUPPORTED;
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// fliter_policy
static uint16_t at_conn_fliter_policy(uint32_t argc, unsigned char* argv[])
{
    volatile uint8_t fliter_policy;

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        uint8_t fliter_policy = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(fliter_policy > 1)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        GAPMultiRole_SetParameter(GAPMULTIROLE_LINK_WHITELIST,sizeof( uint8 ), &fliter_policy);
        at_parameters.conn_fliter_policy = fliter_policy;
        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

/// peer_addr
static uint16_t at_conn_cancel(uint32_t argc, unsigned char* argv[])
{
    ///Peer BD address
    uint8_t        peer_addr[6];

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        uint8_t return_status = CLI_strtoarray(argv[1], CLI_strlen(argv[1]), peer_addr,6);

        if(return_status != OK_RETURN_SUCCESS_STATE)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        multiDelSlaveConnList(peer_addr);
        MultiRole_CancelConn();
        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}
#endif

// conn_handle
static uint16_t at_conn_terminate(uint32_t argc, unsigned char* argv[])
{
    ///conn handle
    uint16_t  conn_handle;

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(conn_handle >= 0x0EFF)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        GAPMultiRole_TerminateConnection(conn_handle);
        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// conn_handle
static uint16_t at_inquiry_curr_conn_param(uint32_t argc, unsigned char* argv[])
{
    if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1] < '0' ||  argv[0][1] > '9')
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        uint16_t con_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(con_idx > 9 || at_parameters.conn_param[con_idx].con_role_state == Idle_Role)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        AT_LOG("\r\n+OK=%d %d %d\r\n",at_parameters.conn_param[con_idx].at_conn_params.conn_int,\
               at_parameters.conn_param[con_idx].at_conn_params.conn_latenty,at_parameters.conn_param[con_idx].at_conn_params.conn_sup_to);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
        return ERROR_UNSUPPORTED;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// 1.conn_handle
static uint16_t at_feature_req(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if( conn_handle >= 0x0EFF)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        HCI_LE_ReadRemoteUsedFeaturesCmd(conn_handle);
        AT_LOG("\r\n+OK\r\n");
    }

    return OK_RETURN_SUCCESS_STATE;
}

//1.conn_handle
static uint16_t at_version_exchange(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;

    if(argv[0][0] == '=')
    {
        if(argc != 2)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if( conn_handle >= 0x0EFF)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        HCI_ReadRemoteVersionInfoCmd(conn_handle);
        AT_LOG("\r\n+OK\r\n");
    }

    return OK_RETURN_SUCCESS_STATE;
}

// conn_handle tx_octets
static uint16_t at_dlu(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;
    uint16_t       tx_octets;

    if(argv[0][0] == '=')
    {
        if(argc != 3)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        tx_octets = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);

        if( conn_handle >= 0x0EFF|| tx_octets < 0x001B || tx_octets > 0x00FB )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        uint8 status = HCI_LE_SetDataLengthCmd(conn_handle,tx_octets, 2120);

        if( status == SUCCESS )
        {
            at_parameters.conn_param[conn_handle].conn_tx_octets = tx_octets;
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1] < '0' ||  argv[0][1] > '9')
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        uint16_t con_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(con_idx > 9)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        AT_LOG("\r\n+OK=%d\r\n",at_parameters.conn_param[conn_handle].conn_tx_octets);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// 1.conn_handle 2.tx_phys 3.rx_phys  -- unsupport coded phy
static uint16_t at_phy_req(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;
    uint8_t       tx_phys,rx_phys;

    if(argv[0][0] == '=')
    {
        if(argc != 4)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        tx_phys = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
        rx_phys = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 10);

        if( conn_handle >= 0x0EFF|| tx_phys > 3 || rx_phys > 3 )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        HCI_LE_SetDefaultPhyMode(0,0x00,0x03,0x03);
        uint8 allPhy = 0x00;
        uint16 phyOption = 0x00;
        HCI_LE_SetPhyMode(conn_handle, allPhy, tx_phys, rx_phys, phyOption);
        AT_LOG("PHY[ %2d %2d %2d]\r\n", allPhy, tx_phys, rx_phys);
        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
// conn_handle  conn_interval  conn_latency  superv_to
// condside slave and master different process: only support slave update conn parameters
static uint16_t at_conn_para_update(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;
    uint16_t       con_intv;
    ///Connection latency
    uint16_t       con_latency;
    ///Link supervision timeout(N * 10 ms)
    uint16_t       superv_to;

    if(argv[0][0] == '=')
    {
        if(argc != 5)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        con_intv = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
        con_latency = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 10);
        superv_to = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 10);

        if( conn_handle >= 0x0EFF|| at_parameters.conn_param[conn_handle].con_role_state != Slave_Role )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        l2capParamUpdateReq_t updateReq;
        updateReq.intervalMin = con_intv;
        updateReq.intervalMax = con_intv;
        updateReq.slaveLatency = con_latency;
        updateReq.timeoutMultiplier = superv_to;
        uint8_t return_val = L2CAP_ConnParamUpdateReq( conn_handle, &updateReq, gapMultiRole_TaskID );

        if(return_val != OK_RETURN_SUCCESS_STATE)
        {
            AT_LOG("\r\n+ERROR=%d %d\r\n",ERROR_RETURN_ERROR,return_val);
            return ERROR_RETURN_ERROR;
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// conn_handle  ntf_length  ntf_value
static uint16_t at_notify(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;

    if(argv[0][0] == '=')
    {
        if(argc != 4)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if( conn_handle >= 0x0EFF )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(at_parameters.conn_param[conn_handle].con_role_state == Slave_Role)
        {
            attHandleValueNoti_t* ntf;
            ntf = osal_mem_alloc(sizeof(attHandleValueNoti_t));

            if(ntf)
            {
                osal_memset(ntf,0,sizeof(attHandleValueNoti_t));
                ntf->len = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);

                if(ntf->len > sizeof(ntf->value))
                {
                    osal_mem_free(ntf);
                    AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                    return ERROR_INVALID_PARAMETERS;
                }

                uint8_t return_status = CLI_strtoarray(argv[3], CLI_strlen(argv[3]), ntf->value,ntf->len);

                if( return_status != OK_RETURN_SUCCESS_STATE )
                {
                    osal_mem_free(ntf);
                    AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                    return ERROR_INVALID_PARAMETERS;
                }

                uint16_t current_mtu_limit = (ATT_GetCurrentMTUSize(conn_handle) - 3);

                if(ntf->len > current_mtu_limit)
                {
                    ntf->len = current_mtu_limit;
                }

                uint8_t return_s = MultiProfile_Notify(conn_handle,MULTIPROFILE_CHAR2,ntf->len,ntf->value);
                osal_mem_free(ntf);

                if( return_s != OK_RETURN_SUCCESS_STATE )
                {
                    AT_LOG("\r\n+ERROR=%d %d\r\n",ERROR_RETURN_ERROR,return_s);
                    return ERROR_RETURN_ERROR;
                }
            }
            else
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_RETURN_ERROR);
                return ERROR_RETURN_ERROR;
            }
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
            return ERROR_UNSUPPORTED;
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}
#endif

#if ( MAX_CONNECTION_MASTER_NUM > 0 )
// conn_handle mtu_size
static uint16_t at_mtu(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;
    uint16_t       mtu_size;

    if(argv[0][0] == '=')
    {
        if(argc != 3)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
        mtu_size = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);

        if( conn_handle >= 0x0EFF|| mtu_size < ATT_MTU_SIZE_MIN || mtu_size > ATT_MAX_MTU_SIZE )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(at_parameters.conn_param[conn_handle].con_role_state == Master_Role)
        {
            ATT_SetMTUSizeMax( mtu_size );
            attExchangeMTUReq_t pReq;
            pReq.clientRxMTU = mtu_size;
            uint8 status =GATT_ExchangeMTU( conn_handle,&pReq, gapMultiRole_TaskID );

            if(status == OK_RETURN_SUCCESS_STATE)
                at_parameters.conn_param[conn_handle].conn_mtu_size = mtu_size;
            else
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_RETURN_ERROR);
                return ERROR_RETURN_ERROR;
            }
        }
        else if (at_parameters.conn_param[conn_handle].con_role_state == Slave_Role)
        {
            ATT_SetMTUSizeMax( mtu_size );
            at_parameters.conn_param[conn_handle].conn_mtu_size = mtu_size;
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
            return ERROR_UNSUPPORTED;
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else if(argv[0][0] == '?')
    {
        if(argc != 1 || argv[0][1] < '0' ||  argv[0][1] > '9')
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        uint16_t con_idx = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if(con_idx > 9)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        AT_LOG("\r\n+OK=%d\r\n",at_parameters.conn_param[con_idx].conn_mtu_size);
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// conn_handle channel_map_arrary
static uint16_t at_channel_map(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;
    uint8 channel_map_arrary[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};

    if(argv[0][0] == '=')
    {
        if(argc != 3)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if( conn_handle >= 0x0EFF )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        uint8_t return_status = CLI_strtoarray(argv[2], CLI_strlen(argv[2]), channel_map_arrary,5);

        if( return_status != OK_RETURN_SUCCESS_STATE )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        HCI_LE_SetHostChanClassificationCmd(channel_map_arrary);
        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
        return ERROR_UNSUPPORTED;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// conn_handle  att_handle
static uint16_t at_read_req(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;

    if(argv[0][0] == '=')
    {
        if(argc != 3)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if( conn_handle >= 0x0EFF )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(at_parameters.conn_param[conn_handle].con_role_state == Master_Role)
        {
            attReadReq_t* Reqread;
            Reqread = osal_mem_alloc(sizeof(attReadReq_t));

            if(Reqread)
            {
                osal_memset(Reqread,0,sizeof(attReadReq_t));
                Reqread->handle = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
                bStatus_t status = GATT_ReadCharValue( conn_handle, Reqread, multiRole_TaskId );
                osal_mem_free(Reqread);

                if( status != OK_RETURN_SUCCESS_STATE )
                {
                    AT_LOG("\r\n+ERROR=%d %d\r\n",ERROR_RETURN_ERROR,status);
                    return ERROR_RETURN_ERROR;
                }
            }
            else
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_RETURN_ERROR);
                return ERROR_RETURN_ERROR;
            }
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
            return ERROR_UNSUPPORTED;
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// conn_handle  att_handle  wr_length  wr_value
static uint16_t at_write_req(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;

    if(argv[0][0] == '=')
    {
        if(argc != 5)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if( conn_handle >= 0x0EFF )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(at_parameters.conn_param[conn_handle].con_role_state == Master_Role)
        {
            attWriteReq_t* pReq;
            pReq = osal_mem_alloc(sizeof(attWriteReq_t));

            if(pReq)
            {
                osal_memset(pReq,0,sizeof(attWriteReq_t));
                pReq->sig = FALSE;
                pReq->cmd = FALSE;
                pReq->handle = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
                pReq->len = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 10);

                if(pReq->len > sizeof(pReq->value))
                {
                    osal_mem_free(pReq);
                    AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                    return ERROR_INVALID_PARAMETERS;
                }

                uint8_t return_status = CLI_strtoarray(argv[4], CLI_strlen(argv[4]), pReq->value,pReq->len);

                if( return_status != OK_RETURN_SUCCESS_STATE )
                {
                    osal_mem_free(pReq);
                    AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                    return ERROR_INVALID_PARAMETERS;
                }

                uint16_t current_mtu_limit = (ATT_GetCurrentMTUSize(conn_handle) - 3);

                if(pReq->len > current_mtu_limit)
                {
                    pReq->len = current_mtu_limit;
                }

                bStatus_t status = GATT_WriteCharValue(conn_handle, pReq,multiRole_TaskId);
                osal_mem_free(pReq);

                if( status != OK_RETURN_SUCCESS_STATE )
                {
                    AT_LOG("\r\n+ERROR=%d %x\r\n",ERROR_RETURN_ERROR,status);
                    return ERROR_RETURN_ERROR;
                }
            }
            else
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_RETURN_ERROR);
                return ERROR_RETURN_ERROR;
            }
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
            return ERROR_UNSUPPORTED;
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}

// conn_handle  att_handle  wr_length  wr_value
static uint16_t at_write_cmd(uint32_t argc, unsigned char* argv[])
{
    uint16_t       conn_handle;

    if(argv[0][0] == '=')
    {
        if(argc != 5)
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS_NUM);
            return ERROR_INVALID_PARAMETERS_NUM;
        }

        conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);

        if( conn_handle >= 0x0EFF )
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
            return ERROR_INVALID_PARAMETERS;
        }

        if(at_parameters.conn_param[conn_handle].con_role_state == Master_Role)
        {
            attWriteReq_t* pReq;
            pReq = osal_mem_alloc(sizeof(attWriteReq_t));

            if(pReq)
            {
                osal_memset(pReq,0,sizeof(attWriteReq_t));
                pReq->sig = FALSE;
                pReq->cmd = TRUE;
                pReq->handle = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 10);
                pReq->len = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 10);

                if(pReq->len > sizeof(pReq->value))
                {
                    osal_mem_free(pReq);
                    AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                    return ERROR_INVALID_PARAMETERS;
                }

                uint8_t return_status = CLI_strtoarray(argv[4], CLI_strlen(argv[4]), pReq->value,pReq->len);

                if( return_status != OK_RETURN_SUCCESS_STATE )
                {
                    osal_mem_free(pReq);
                    AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
                    return ERROR_INVALID_PARAMETERS;
                }

                uint16_t current_mtu_limit = (ATT_GetCurrentMTUSize(conn_handle) - 3);

                if(pReq->len > current_mtu_limit)
                {
                    pReq->len = current_mtu_limit;
                }

                uint8_t status = GATT_WriteNoRsp(conn_handle, pReq);
                osal_mem_free(pReq);

                if( status != OK_RETURN_SUCCESS_STATE )
                {
                    AT_LOG("\r\n+ERROR=%d %d\r\n",ERROR_RETURN_ERROR,status);
                    return ERROR_RETURN_ERROR;
                }
            }
            else
            {
                AT_LOG("\r\n+ERROR=%d\r\n",ERROR_RETURN_ERROR);
                return ERROR_RETURN_ERROR;
            }
        }
        else
        {
            AT_LOG("\r\n+ERROR=%d\r\n",ERROR_UNSUPPORTED);
            return ERROR_UNSUPPORTED;
        }

        AT_LOG("\r\n+OK\r\n");
    }
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_INVALID_PARAMETERS);
        return ERROR_INVALID_PARAMETERS;
    }

    return OK_RETURN_SUCCESS_STATE;
}
#endif


__ATTR_SECTION_SRAM__ void at_wakeup_handle(void)
{
    //1.init ble at config
    //2.judge wake up flag
    //3.read flash data(only system sleep mode used)
    //4.enter auto sleep mode
    ///dbg:enable rtc fun
    //*((volatile uint32 *)0x4000F024) |= (1<<0);
    // lock pwrmgr
    hal_pwrmgr_lock(MOD_USR0);///  MOD_USR1
    // ble at init config
    // hal_gpio_pull_set(AT_UART_WAKEUP_PIN, GPIO_PULL_UP);
    // hal_gpio_fmux_set(AT_UART_WAKEUP_PIN, FMUX_UART0_RX);
    // ble_at_uart_init();
    // gpio_write(P23,1);gpio_write(P23,0);
    ///enter auto sleep mode
    osal_start_timerEx( bleAT_TaskID, AT_ENTER_ATUO_SLEEP_MODE_EVT, 100*10 );
    // AT_LOG("wake up success\n");
}

void enter_sleep_off_mode2(Sleep_Mode mode)
{
    if(mode==SYSTEM_SLEEP_MODE)
        hal_system_clock_change_req(at_parameters.system_clk_cfg);

    enter_sleep_off_mode0(mode);
}

__ATTR_SECTION_SRAM__ void at_sleep_handle(void)
{
    // AT_LOG("enter sleep\n");
    /// 1.disable funllmx fun
    hal_gpio_fmux(AT_UART_WAKEUP_PIN, Bit_DISABLE);
    ///2.set and config wakeup pin
    hal_gpio_pin_init(AT_UART_WAKEUP_PIN, GPIO_INPUT);
    hal_gpio_pull_set(AT_UART_WAKEUP_PIN,GPIO_PULL_UP_S);
    hal_gpio_wakeup_set(AT_UART_WAKEUP_PIN, NEGEDGE); //  NEGEDGE
    /// 3.set change clk patch
    // JUMP_FUNCTION(ENTER_SLEEP_OFF_MODE)              =   (uint32_t)&enter_sleep_off_mode2;
    ///3.dbg: close rtc fun
    // *((volatile uint32 *)0x4000F024) &= ~(1<<0);
}



