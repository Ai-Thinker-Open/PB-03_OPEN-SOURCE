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
    Filename:       at_ble_sbm_cmd.h
    Revised:
    Revision:

    Description:    This file contains the at ble sbm cmd sample application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#ifndef __AT_BLE_SBM_CMD_H
#define __AT_BLE_SBM_CMD_H

#include "cliface.h"
#include "multi.h"
#include "clock.h"
#define OK_RETURN_SUCCESS_STATE                   0x00
/// error num
#define ERROR_UNKNOWN_AT_CMD                      0x01
#define ERROR_INVALID_PARAMETERS_NUM              0x02
#define ERROR_INVALID_PARAMETERS                  0x03
#define ERROR_UNSUPPORTED                         0x04
#define ERROR_DUPLICATE_DEFINITION                0x05        /// duplicate definition
#define ERROR_OVERRUN                             0x06        /// overrun
#define ERROR_RETURN_ERROR                        0x07        /// return error

#define ERROR_FLASH_WRITE                         0x10        ///
#define ERROR_FLASH_READ                          0x11        /// 

#define AT_UART_TX_PIN                            P9
#define AT_UART_RX_PIN                            P10
#define AT_UART_WAKEUP_PIN                        P10 //P14    
#define AT_UART_RX_MAX_LEN                        400 //long packet data is allowed 
#define AT_UART_AT_CMD_LIST_ELEM                  38
///Adv  Types
enum
{
    AT_ADV_IND = 0,
    AT_ADV_DIRECT_IND_HIGH_DUTY,
    AT_ADV_SCAN_IND,
    AT_ADV_NONCONN_IND,
    AT_ADV_DIRECT_IND_LOW_DUTY,
    AT_ADV_MAX_TYPE,
};

enum
{
    AT_PEER_PUBLIC_ADDR_TYPE = 0,
    AT_PEER_RANDOM_ADDR_TYPE,
    AT_PEER_MAX_ADDR_TYPE,
};

#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
struct at_bd_addr
{
    ///6-byte array address value
    uint8_t  addr[6];
};
struct at_adv_param
{
    uint16_t adv_int; // advertising interval.
    uint8_t  adv_type; // advertising types.
    uint8_t  adv_chnl_map; // advertising channel map
    uint8_t  peer_addr_type;// Peer address type: public=0 / random=1
    uint8_t  adv_fliter_policy;// Peer address type: public=0 / random=1
    struct at_bd_addr peer_addr;// Peer Bluetooth device address
    uint8_t adv_mode[MAX_CONNECTION_SLAVE_NUM][1]; // mode: disable=0 / enable=1
};

struct at_service_charac_param
{
    uint8_t suuid_length;
    // uint8_t suuid_value[16];
    uint8_t first_cuuid_length;
    // uint8_t first_cuuid_premi;
    uint8_t second_cuuid_length;
    // uint8_t second_cuuid_premi;
    // uint8_t first_cuuid_value[16];
    // uint8_t second_cuuid_value[16];
};

#endif

#if ( MAX_CONNECTION_MASTER_NUM > 0 )
struct at_scan_param
{
    uint16_t scan_int; // scan interval.
    uint16_t scan_window; // scan window.
    uint16_t scan_duration; // scan duration.
    bool scan_mode; // mode: disable=0 / enable=1
    uint8_t scan_cnt; // record scan number
    uint8_t scan_type; // passive=0 / active=1
    uint8_t scan_fliter_policy; // allow all device=0 / allow white list device=1

};
#endif

struct at_conn
{
    uint8_t peer_addr_type;
    uint8_t peer_addr[6];
    uint16_t conn_int; // conn interval.
    uint16_t conn_latenty; // conn latenty.
    uint16_t conn_sup_to; // conn supervision timeout.
};


struct at_conn_param
{

    struct at_conn at_conn_params;
    GAPMultiRole_State_t con_role_state;
    uint16_t conn_mtu_size;
    uint16_t conn_tx_octets;
};
///at struct used to save at setted info
typedef struct
{
    uint8_t tx_power;
    uint8_t bd_addr[6];
    sysclk_t system_clk_cfg;
    #if ( MAX_CONNECTION_SLAVE_NUM > 0 )
    struct at_adv_param adv_param;
    uint8_t adv_data[MAX_CONNECTION_SLAVE_NUM][31];
    uint8_t scan_rsp_data[MAX_CONNECTION_SLAVE_NUM][31];
    struct at_service_charac_param ser_charac_param;
    #endif

    #if ( MAX_CONNECTION_MASTER_NUM > 0 )
    struct at_scan_param scan_param;
    #endif
    struct at_conn_param conn_param[MAX_CONNECTION_NUM];
    uint8_t conn_fliter_policy;
    uint8_t conn_master_counter;
    uint8_t conn_slave_counter;
    uint8_t sleep_mode;

} at_env_param;

extern const CLI_COMMAND ble_at_cmd_list[AT_UART_AT_CMD_LIST_ELEM];
extern unsigned char cmdstr[AT_UART_RX_MAX_LEN];
extern uint16_t cmdlen;
extern at_env_param at_parameters;

void at_wakeup_handle(void);
void at_sleep_handle(void);


#endif
