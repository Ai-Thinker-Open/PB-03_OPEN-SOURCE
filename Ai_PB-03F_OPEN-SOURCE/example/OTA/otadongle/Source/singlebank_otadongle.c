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



/*********************************************************************
    INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_bufmgr.h"
#include "gatt.h"
#include "ll.h"
#include "ll_common.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile_ota.h"
#include "singlebank_otadongle.h"
#include "timer.h"
#include "log.h"
#include "ll_def.h"
#include "global_config.h"
#include "flash.h"
#include "rflib.h"
#include "clock.h"
#include "OSAL_Clock.h"
#include "ota_app_service.h"
#include "otam_protocol.h"
#include "otam_1clk_ota.h"
#include "crc16.h"
#include "stdio.h"
#include "ll_hw_drv.h"
/*********************************************************************
    MACROS
*/



// Length of bd addr as a string
#define B_ADDR_STR_LEN                          15

/*********************************************************************
    CONSTANTS
*/

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                    50

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                   200

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                  DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN           TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST            FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE            TRUE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST                 FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                     1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST           FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL        10

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL        300

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY            0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT             100

// Default passcode
#define DEFAULT_PASSCODE                        123456//19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                    GAPBOND_PAIRING_MODE_INITIATE//GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                       FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                    FALSE //TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES                 GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY             10

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID            TRUE

#define DEFAULT_OTA_UPGRADE_AGAIN_TIME          (1*1600)

#define DEFAULT_OTA_RECONNECT_TIME              (1*1600)



// Service and Characteristic




typedef enum
{
    OTAM_FSM_UNINIT = 0,
    OTAM_FSM_IDLE,
    OTAM_FSM_SCANING,
    OTAM_FSM_CONNECTING,
    OTAM_FSM_CONNECTED,
} otam_fsm_t;


typedef enum
{
    DISC_ST_INIT = 0,           // Idle
    DISC_ST_SVC,                // Service discovery
    DISC_ST_CHAR,               // Characteristic discovery
    DISC_ST_COMPLETED,          // discovery completed
    DISC_ST_FAILED = 0xff
} disc_st_t;


typedef struct
{
    uint8_t     addr_type;
    uint8_t     addr[B_ADDR_LEN];
    int8_t      rssi;
    uint8_t     flags;
    char        name[32];
    uint16_t    company_id;
} otac_scan_dev_t;


typedef struct
{
    ota_s_e     ota_status;
    uint8_t     flags;
    uint8_t     index;
    uint8_t     addr[B_ADDR_LEN];
    uint32_t    sys_tick;
} otac_conn_dev_t;


typedef struct
{
    uint16_t    char_cmd_hdl;
    uint16_t    char_rsp_hdl;
    uint16_t    char_rsp_cccd_hdl;
    uint16_t    char_data_hdl;
} otac_service_t;


typedef struct
{
    uint16_t    conn_hdl;
    disc_st_t   disc_state;
    uint8_t     run_mode;       // application mode or ota mode or unknow mode
    uint16_t    mtu;
    uint16_t    attr_start_hdl;
    uint16_t    attr_end_hdl;
    uint16_t    op_handle;      // save the characteristic handle of the operation

    otac_service_t ota_service;
} otac_dev_t;


typedef struct
{
    otam_fsm_t          fsm_state;

    //scan
    uint8_t             scan_dev_num;
    otac_scan_dev_t     scan_dev_list[DEFAULT_MAX_SCAN_RES];

    // conn
    otac_conn_dev_t     conn_dev_list[DEFAULT_MAX_SCAN_RES];

    //connected device
    uint8_t             mode_choose;
    otac_dev_t          otac_dev;
} otam_ctx_t;


static otam_ctx_t m_otam_ctx;
static int8_t m_conn_index = -1;

#define DEFAULT_OTAM_MTU_SIZE       247
#define OTAM_FSM_SET(new_state)     {m_otam_ctx.fsm_state = new_state;}



/*********************************************************************
    GLOBAL VARIABLES
*/
perStatsByChan_t g_perStatsByChanTest;
ota_s_e g_ota_status = OTA_IDLE;

/*********************************************************************
    EXTERNAL VARIABLES
*/
extern uint32 g_osal_mem_allo_cnt;
extern uint32 g_osal_mem_free_cnt;
extern l2capSARDbugCnt_t g_sarDbgCnt;
extern llGlobalStatistics_t g_pmCounters;
/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

// Task ID for internal task/event processing
uint8 otaMasterTaskId = 0;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";


/*********************************************************************
    LOCAL FUNCTIONS
*/
static void otaMaster_ProcessGATTMsg( gattMsgEvent_t* pMsg );
static void otaMasterRssiCB( uint16 connHandle, int8  rssi );
static void otaMasterEventCB( gapCentralRoleEvent_t* pEvent );
static void otaMasterPasscodeCB( uint8* deviceAddr, uint16 connectionHandle,
                                 uint8 uiInputs, uint8 uiOutputs );
static void otaMasterPairStateCB( uint16 connHandle, uint8 state, uint8 status );
//static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void otaMaster_ProcessOSALMsg( osal_event_hdr_t* pMsg );


/*********************************************************************
    PROFILE CALLBACKS
*/

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
    otaMasterRssiCB,       // RSSI callback
    otaMasterEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
    otaMasterPasscodeCB,
    otaMasterPairStateCB
};

const static uint8_t s_mac_zero[B_ADDR_LEN] = {0,0,0,0,0,0};


CONST uint8 ota_ServiceUUID[ATT_UUID_SIZE] =
{0x23, 0xf1, 0x6e, 0x53, 0xa4, 0x22, 0x42, 0x61, 0x91, 0x51, 0x8b, 0x9b, 0x01, 0xff, 0x33, 0x58};

//command characteristic
CONST uint8 ota_CommandUUID[ATT_UUID_SIZE] =
{0x23, 0xf1, 0x6e, 0x53, 0xa4, 0x22, 0x42, 0x61, 0x91, 0x51, 0x8b, 0x9b, 0x02, 0xff, 0x33, 0x58};

// Sensor location characteristic
CONST uint8 ota_ResponseUUID[ATT_UUID_SIZE] =
{0x23, 0xf1, 0x6e, 0x53, 0xa4, 0x22, 0x42, 0x61, 0x91, 0x51, 0x8b, 0x9b, 0x03, 0xff, 0x33, 0x58};

// Command characteristic
CONST uint8 ota_DataUUID[ATT_UUID_SIZE] =
{0x23, 0xf1, 0x6e, 0x53, 0xa4, 0x22, 0x42, 0x61, 0x91, 0x51, 0x8b, 0x9b, 0x04, 0xff, 0x33, 0x58};



static void print_hex(const uint8* data, uint16 len)
{
    #if(DEBUG_INFO>0)
    uint16 i;
    char strdata[5];

    for (i = 0; i < len - 1; i++)
    {
        sprintf(strdata, "%.2x", data[i]);
        OTA_LOG("%s ",strdata);
    }

    sprintf(strdata, "%.2x", data[i]);
    OTA_LOG("%s\n",strdata);
    #endif
}


static uint8_t otac_analyze_advdata(otac_scan_dev_t* pdev,gapDeviceInfoEvent_t* pData)
{
    uint8_t DataLength = 0;
    uint8_t New_ADStructIndex = 0;
    uint8_t AD_Length = 0;
    uint8_t AD_Type = 0;
    uint8_t conn_flag = 0;
    DataLength = pData->dataLen;
    pdev->rssi = pData->rssi;

    while(DataLength)
    {
        New_ADStructIndex += AD_Length;
        // DATA FORMAT : Length + AD Type + AD Data
        AD_Length = pData->pEvtData[pData->dataLen - DataLength];
        AD_Type   = pData->pEvtData[New_ADStructIndex+1];

        if(AD_Length<2 || AD_Length>0x1f)
        {
            break;
        }

        switch(AD_Type)
        {
        case GAP_ADTYPE_FLAGS:
            pdev->flags = pData->pEvtData[New_ADStructIndex+2];
            break;

        case GAP_ADTYPE_LOCAL_NAME_SHORT:
        case GAP_ADTYPE_LOCAL_NAME_COMPLETE:
            osal_memset(pdev->name, 0, 32);
            osal_memcpy(pdev->name,&(pData->pEvtData[New_ADStructIndex+2]), AD_Length-1);
            //OTA_LOG("%s\n",pdev->name); // TODO 解析蓝牙名称
            break;

        case GAP_ADTYPE_MANUFACTURER_SPECIFIC:
            pdev->company_id = pData->pEvtData[New_ADStructIndex+2] + (pData->pEvtData[New_ADStructIndex+3]<<8);
            break;

        default:
            break;
        }

        AD_Length++;
        DataLength -= AD_Length;
    }

    if(pdev->company_id == 0x0504 && pdev->rssi >= -70)
    {
        conn_flag = 1;
    }

    return conn_flag;
}


static int8_t otac_scan_update_devinfo( gapCentralRoleEvent_t* pEvent )
{
    otam_ctx_t* pctx = &m_otam_ctx;
    otac_scan_dev_t* p_scanlist = pctx->scan_dev_list;
    gapDeviceInfoEvent_t* p_devinfo = (gapDeviceInfoEvent_t*)pEvent;
    uint8_t i = 0;

    if(pEvent->deviceInfo.opcode != GAP_DEVICE_INFO_EVENT)
    {
        return -1;
    }

    if(pctx->fsm_state != OTAM_FSM_SCANING)
    {
        return -1;
    }

    if(p_devinfo->eventType != GAP_ADRPT_ADV_IND && p_devinfo->eventType != GAP_ADRPT_ADV_DIRECT_IND
            && p_devinfo->eventType != GAP_ADRPT_SCAN_RSP)
    {
        return -1;
    }

    //check if the mac address is in list
    //if true, update device info
    //if false, add new device
    //if device number is runout, just drop!
    for(; i < DEFAULT_MAX_SCAN_RES; i++)
    {
        //case new device
        if(osal_memcmp( p_scanlist[i].addr, s_mac_zero, B_ADDR_LEN ) != 0)
        {
            p_scanlist[i].addr_type = p_devinfo->addrType;
            osal_memcpy(p_scanlist[i].addr, p_devinfo->addr, 6);
            break;
        }
        else if(osal_memcmp(p_scanlist[i].addr, p_devinfo->addr, B_ADDR_LEN ) != 0)
        {
            break;
        }
    }

    if(i == DEFAULT_MAX_SCAN_RES)
    {
        GAPCentralRole_CancelDiscovery();
        return -1;
    }

    if(otac_analyze_advdata(&(p_scanlist[i]), &(pEvent->deviceInfo)) == 1)
    {
        uint32_t current_sys_tick = 0;
        char str_mac[20] = {0};
        current_sys_tick = hal_systick();
        LOG("current_sys_tick:%d\n", current_sys_tick);

        for(uint8_t i=0; i<DEFAULT_MAX_SCAN_RES; i++)
        {
            if(pctx->conn_dev_list[i].flags)
            {
                if(osal_memcmp(pctx->conn_dev_list[i].addr, p_devinfo->addr, 6) == 1)
                {
                    uint32_t cmp_val = current_sys_tick - pctx->conn_dev_list[i].sys_tick;

                    if(pctx->conn_dev_list[i].ota_status == OTA_SUCCESS ||
                            pctx->conn_dev_list[i].ota_status == OTA_NO_SERVICE)
                    {
                        if(cmp_val <= DEFAULT_OTA_UPGRADE_AGAIN_TIME)
                        {
                            OTA_LOG("oat mac:[%.2x:%.2x:%.2x:%.2x:%.2x:%.2x], status:%d, tick:%d, index:%d\n",
                                    p_devinfo->addr[5], p_devinfo->addr[4],
                                    p_devinfo->addr[3], p_devinfo->addr[2],
                                    p_devinfo->addr[1], p_devinfo->addr[0],
                                    pctx->conn_dev_list[i].ota_status,
                                    pctx->conn_dev_list[i].sys_tick, i);
                            return -1;
                        }
                    }
                    else if(pctx->conn_dev_list[i].ota_status == OTA_CONNECT_FAIL)
                    {
                        if(cmp_val <= DEFAULT_OTA_RECONNECT_TIME)
                        {
                            OTA_LOG("oat mac:[%.2x:%.2x:%.2x:%.2x:%.2x:%.2x], status:%d\n",
                                    p_devinfo->addr[5],
                                    p_devinfo->addr[4],
                                    p_devinfo->addr[3],
                                    p_devinfo->addr[2],
                                    p_devinfo->addr[1],
                                    p_devinfo->addr[0],
                                    pctx->conn_dev_list[i].ota_status);
                            return -1;
                        }
                    }
                }
            }
        }

        sprintf(str_mac, "[%.2x:%.2x:%.2x:%.2x:%.2x:%.2x]",
                p_scanlist[i].addr[5],
                p_scanlist[i].addr[4],
                p_scanlist[i].addr[3],
                p_scanlist[i].addr[2],
                p_scanlist[i].addr[1],
                p_scanlist[i].addr[0]);
        OTA_LOG("%d  %s rssi:%d name:\"%s\"\n", i, str_mac, p_scanlist[i].rssi, p_scanlist[i].name);
        GAPCentralRole_CancelDiscovery();
        return i;
    }

    return -1;
}


static int otac_cccd(bool enable)
{
    otac_dev_t* pdev = &(m_otam_ctx.otac_dev);
    attWriteReq_t wreq;
    bStatus_t bret;

    if(pdev->conn_hdl != 0 || pdev->disc_state != DISC_ST_COMPLETED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    if(pdev->run_mode < OTAC_RUNMODE_APP)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    osal_memset(&wreq, 0, sizeof(attWriteReq_t));
    wreq.cmd = 0;
    wreq.sig = 0;
    wreq.handle = pdev->ota_service.char_rsp_cccd_hdl;
    wreq.len = 2;

    if(enable)
    {
        wreq.value[0] = 1;
    }

    pdev->op_handle = wreq.handle;
    bret = GATT_WriteCharValue(pdev->conn_hdl, &wreq, otaMasterTaskId);
    osal_start_timerEx(otaMasterTaskId, OTA_PROCESS_TIMEOUT_EVT, 1000);
    osal_clear_event(otaMasterTaskId, OTA_PROCESS_TIMEOUT_EVT);

    if(bret!= SUCCESS)
    {
        return PPlus_ERR_BLE_BUSY;
    }
    else
    {
        return PPlus_SUCCESS;
    }
}


static int otaMaster_EstablishLink_wl(void)
{
    bStatus_t ret;
    ret = GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                       TRUE, ADDRTYPE_PUBLIC, NULL);

    if(ret == SUCCESS)
    {
        return PPlus_SUCCESS;
    }
    else
    {
        return PPlus_ERR_BLE_FAIL;
    }
}


static void otaMaster_StartDiscoveryService( void )
{
    otam_ctx_t* pctx = &m_otam_ctx;
    otac_dev_t* pdev = &(pctx->otac_dev);
    pdev->disc_state = DISC_ST_SVC;
    // Discovery simple BLE service
    bStatus_t status = GATT_DiscAllPrimaryServices(pdev->conn_hdl,otaMasterTaskId);

    if(status != PPlus_SUCCESS)
    {
        osal_start_timerEx(otaMasterTaskId, START_DISCOVERY_SERVICE_EVT, DEFAULT_SVC_DISCOVERY_DELAY);
    }

    OTA_LOG("GATT_DiscAllPrimaryServices:%02x\n", status);
}


static void otac_scan_completed(void)
{
    if(m_conn_index == -1 || m_conn_index >= DEFAULT_MAX_SCAN_RES)
    {
        osal_set_event(otaMasterTaskId, ONECLK_OP_OTA_AUTOSCAN_EVT);
    }
    else
    {
        oneclk_evt_t ev;
        otam_ctx_t* pctx = & m_otam_ctx;
        ev.ev = ONECLK_EVT_SCAN_RESULT;
        osal_memcpy(ev.param.mac, pctx->scan_dev_list[m_conn_index].addr, 6);
        otam_oneclick_evt(&ev);
        OTA_LOG("m_conn_index:%d mac:%02x %02x %02x %02x %02x %02x\n", m_conn_index, ev.param.mac[0],
                ev.param.mac[1], ev.param.mac[2], ev.param.mac[3], ev.param.mac[4], ev.param.mac[5]);
    }
}


static void otac_dle_phy_mtu(void)
{
    uint8 status = PPlus_SUCCESS;
    otam_ctx_t* pctx = & m_otam_ctx;
    otac_dev_t* pdev = &(pctx->otac_dev);
    ATT_SetMTUSizeMax( DEFAULT_OTAM_MTU_SIZE );
    attExchangeMTUReq_t pReq;
    pReq.clientRxMTU = DEFAULT_OTAM_MTU_SIZE;
    status = GATT_ExchangeMTU(pdev->conn_hdl, &pReq, otaMasterTaskId);
    OTA_LOG("GATT_ExchangeMTU:%d\n", status);
    status = HCI_LE_SetDataLengthCmd(pctx->otac_dev.conn_hdl, 251, 2120);
    OTA_LOG("HCI_LE_SetDataLengthCmd:%d\n", status);
    osal_start_timerEx(otaMasterTaskId, START_DISCOVERY_SERVICE_EVT, DEFAULT_SVC_DISCOVERY_DELAY);
}


static void otac_dev_connected(gapEstLinkReqEvent_t* pEvent)
{
    otam_ctx_t* pctx = & m_otam_ctx;
    otac_dev_t* pdev = &(pctx->otac_dev);
    osal_memset(pdev, 0, sizeof(otac_dev_t));
    pdev->conn_hdl = pEvent->connectionHandle;
    pdev->mtu = 23;
    pdev->disc_state = DISC_ST_SVC;
    OTA_LOG("otac_dev_connected Handle: 0x%x\n", pdev->conn_hdl);
    HCI_ReadRemoteVersionInfoCmd(pdev->conn_hdl);
    HCI_LE_ReadRemoteUsedFeaturesCmd(pdev->conn_hdl);
    gapUpdateLinkParamReq_t Params =
    {
        .connectionHandle = 0,  //!< Connection handle of the update
        .intervalMin = 6,       //!< Minimum Connection Interval
        .intervalMax = 6,       //!< Maximum Connection Interval
        .connLatency = 0,       //!< Connection Latency
        .connTimeout = 100,     //!< Connection Timeout
    };
    bStatus_t status = GAP_UpdateLinkParamReq(&Params);
    OTA_LOG("GAP_UpdateLinkParamReq:%02x\n", status);
    osal_start_timerEx(otaMasterTaskId, START_UPD_PHY_MODE_EVT, 10);
}


/********* method  ******/
static int otaMaster_method_clear(void)
{
    //otac_dev_t* pdev = &(m_otam_ctx.otac_dev);
    osal_stop_timerEx(otaMasterTaskId, OTA_DATA_DELAY_WRITE_EVT);
    osal_clear_event(otaMasterTaskId,  OTA_DATA_DELAY_WRITE_EVT);
    osal_stop_timerEx(otaMasterTaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT);
    osal_clear_event(otaMasterTaskId,  BLE_CMD_WRITE_OP_TIMEOUT_EVT);
    osal_stop_timerEx(otaMasterTaskId, OTA_PROCESS_TIMEOUT_EVT);
    osal_clear_event(otaMasterTaskId, OTA_PROCESS_TIMEOUT_EVT);
    return PPlus_SUCCESS;
}


static int otaMaster_write_data_delay(uint32_t ms_delay)
{
    if(ms_delay == 0)
    {
        osal_set_event(otaMasterTaskId, OTA_DATA_DELAY_WRITE_EVT);
    }
    else
    {
        osal_start_timerEx(otaMasterTaskId, OTA_DATA_DELAY_WRITE_EVT, ms_delay);
    }

    return PPlus_SUCCESS;
}


static int otaMaster_write_data(uint8_t* data, uint16_t len)
{
    otac_dev_t* pdev = &(m_otam_ctx.otac_dev);
    attWriteReq_t wreq;
    bStatus_t bret;

    if(pdev->conn_hdl != 0 || pdev->disc_state != DISC_ST_COMPLETED)
        return PPlus_ERR_INVALID_STATE;

    if(pdev->run_mode < OTAC_RUNMODE_OTA)
        return PPlus_ERR_INVALID_STATE;

    if(len > pdev->mtu -3)
        return PPlus_ERR_INVALID_LENGTH;

    osal_memset(&wreq, 0, sizeof(attWriteReq_t));
    wreq.cmd = TRUE;
    wreq.sig = 0;
    wreq.handle = pdev->ota_service.char_data_hdl;
    wreq.len = len;
    hal_flash_read((uint32_t)data, wreq.value, len);
    bret = GATT_WriteNoRsp(pdev->conn_hdl, &wreq);

    if(bret != SUCCESS)
    {
        return PPlus_ERR_BLE_BUSY;
    }
    else
    {
        osal_start_timerEx(otaMasterTaskId, OTA_PROCESS_TIMEOUT_EVT, 1000);
        osal_clear_event(otaMasterTaskId, OTA_PROCESS_TIMEOUT_EVT);
        return PPlus_SUCCESS;
    }
}


static int otaMaster_write_command(uint8_t* data, uint16_t len, uint32_t timeout)
{
    otac_dev_t* pdev = &(m_otam_ctx.otac_dev);
    attWriteReq_t wreq;
    bStatus_t bret;

    if(pdev->conn_hdl != 0 || pdev->disc_state != DISC_ST_COMPLETED)
        return PPlus_ERR_INVALID_STATE;

    if(pdev->run_mode < OTAC_RUNMODE_APP)
        return PPlus_ERR_INVALID_STATE;

    if(len > pdev->mtu -3)
        return PPlus_ERR_INVALID_LENGTH;

    osal_memset(&wreq, 0, sizeof(attWriteReq_t));
    wreq.cmd = 0;
    wreq.sig = 0;
    wreq.handle = pdev->ota_service.char_cmd_hdl;
    wreq.len = len;
    osal_memcpy(wreq.value, data, len);
    pdev->op_handle = wreq.handle;
    OTA_LOG("TXCMD:\n");
    print_hex(data, len);
    bret = GATT_WriteCharValue(pdev->conn_hdl, &wreq, otaMasterTaskId);

    if(bret!= SUCCESS)
        return PPlus_ERR_BLE_BUSY;

    osal_start_timerEx(otaMasterTaskId, OTA_PROCESS_TIMEOUT_EVT, 1000);
    osal_clear_event(otaMasterTaskId, OTA_PROCESS_TIMEOUT_EVT);

    if(timeout)
    {
        osal_start_timerEx(otaMasterTaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT, timeout);
    }

    return PPlus_SUCCESS;
}
/********end method******/

#define OTAM_CTRL_REG           0x4000f030



static void on_msg_scan()
{
    otam_ctx_t* pctx = &m_otam_ctx;

    if(pctx->fsm_state != OTAM_FSM_IDLE)
    {
        OTA_LOG("Can't do scan, incorrect state: %d\n", pctx->fsm_state);
        return;
    }

    osal_memset(&(pctx->scan_dev_list[0]), 0,sizeof(otac_scan_dev_t)*DEFAULT_MAX_SCAN_RES);
    pctx->scan_dev_num = 0;
    bStatus_t stu = GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE, TRUE, FALSE );

    if(stu != SUCCESS)
    {
        OTA_LOG("run scan failed");
        return;
    }

    OTA_LOG("scan start\n");
    OTAM_FSM_SET(OTAM_FSM_SCANING);
}


static void on_msg_scan_w()
{
    otam_ctx_t* pctx = &m_otam_ctx;
    uint8_t* pmac = NULL;
    uint8_t mac[6];
    //config scan white list
    LL_ClearWhiteList();
    pmac = pull_from_maclist();

    if(!pmac)
    {
        return;
    }

    mac[0] = pmac[5];
    mac[1] = pmac[4];
    mac[2] = pmac[3];
    mac[3] = pmac[2];
    mac[4] = pmac[1];
    mac[5] = pmac[0];
    print_hex(mac, 6);
    LL_AddWhiteListDevice(mac, ADDRTYPE_PUBLIC);

    if(pctx->fsm_state != OTAM_FSM_IDLE)
    {
        OTA_LOG("Can't do scan, incorrect state: %d\n", pctx->fsm_state);
        return;
    }

    osal_memset(&(pctx->scan_dev_list[0]), 0,sizeof(otac_scan_dev_t)*DEFAULT_MAX_SCAN_RES);
    pctx->scan_dev_num = 0;
    bStatus_t stu = GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE, TRUE, TRUE );

    if(stu != SUCCESS)
    {
        OTA_LOG("run scan failed");
        return;
    }

    OTAM_FSM_SET(OTAM_FSM_SCANING);
}


static void on_msg_connect(otam_cmd_conn_t* pMsg)
{
    bool bret = false;
    otam_ctx_t* pctx = &m_otam_ctx;
    otac_scan_dev_t* p_scandev = pctx->scan_dev_list;

    if(pMsg->id != 0xff)
    {
        if(pctx->scan_dev_num <= pMsg->id)
        {
            OTA_LOG("ID %d is out of range\n", pMsg->id);
            return;
        }

        if(pctx->fsm_state != OTAM_FSM_IDLE)
        {
            OTA_LOG("Error state [%d], will not do connect\n", pctx->fsm_state);
            return;
        }

        bret = otaMaster_EstablishLink(p_scandev[pMsg->id].addr_type, p_scandev[pMsg->id].addr);
    }
    else
    {
        bret = otaMaster_EstablishLink(ADDRTYPE_PUBLIC, pMsg->mac);
    }

    if(bret)
    {
        OTAM_FSM_SET(OTAM_FSM_CONNECTING);
    }
}


static void on_msg_connect_w(void)
{
    int ret = false;
    ret = otaMaster_EstablishLink_wl();

    if(ret == PPlus_SUCCESS)
    {
        OTAM_FSM_SET(OTAM_FSM_CONNECTING);
    }
}


static void on_msg_disconnect(void)
{
    otam_ctx_t* pctx = &m_otam_ctx;
    GAPCentralRole_TerminateLink(pctx->otac_dev.conn_hdl);
    OTA_LOG("conn_hdl:%d\n", pctx->otac_dev.conn_hdl);
    OTAM_FSM_SET(OTAM_FSM_IDLE);
}


static void on_msg_ota_choose_mode(otam_cmd_mode_t* pMsg)
{
    if(pMsg->mode != OTA_MODE_OTA && pMsg->mode != OTA_MODE_RESOURCE)
    {
        OTA_LOG("OTA Choose mode: invalid parameter! \n");
        return;
    }

    otamProtocol_app_start_ota(pMsg->mode);
    m_otam_ctx.mode_choose = pMsg->mode;
    osal_start_timerEx(otaMasterTaskId, OTA_APP_DISCONN_DELAY_EVT, 1000);
}


static void on_msg_ota_start()
{
    otamProtocol_start_ota(0);
}


static void on_msg_ota_stop()
{
    otamProtocol_stop_ota();
}


static void on_msg_ota_reboot(osal_event_hdr_t* pMsg)
{
}


static void on_msg_one_clk(otam_cmd_conn_t* pMsg)
{
    if(pMsg->id == 0)
    {
        otam_oneclick_ota();
    }
    else
    {
        if(m_otam_ctx.fsm_state == OTAM_FSM_IDLE)
        {
            oneclk_evt_t ev;
            ev.ev = ONECLK_EVT_SCAN_RESULT;
            osal_memcpy(ev.param.mac, pMsg->mac, 6);
            otam_oneclick_evt(&ev);
        }
        else
        {
            OTA_LOG("OTA auto mac config failed, state is not OTAM_FSM_IDLE (%d)\n", m_otam_ctx.fsm_state);
        }
    }
}


static void otaMaster_ProcessCMDMsg(otam_cmd_conn_t* pMsg)
{
    switch(pMsg->hdr.status)
    {
    case OTAM_CMD_SCAN:
        on_msg_scan();
        break;

    case OTAM_CMD_SCAN_FAST:
        on_msg_scan();
        break;

    case OTAM_CMD_SCAN_WL:
        on_msg_scan_w();
        break;

    case OTAM_CMD_CONNECT:
        on_msg_connect((otam_cmd_conn_t*)pMsg);
        break;

    case OTAM_CMD_CONNECT_WL:
        on_msg_connect_w();
        break;

    case OTAM_CMD_DISCONNECT:
        on_msg_disconnect();
        break;

    case OTAM_CMD_OTA_MODE:
        on_msg_ota_choose_mode((otam_cmd_mode_t*)pMsg);
        break;

    case OTAM_CMD_OTA_START:
        on_msg_ota_start();
        break;

    case OTAM_CMD_OTA_STOP:
        on_msg_ota_stop();
        break;

    case OTAM_CMD_OTA_REBOOT:
        on_msg_ota_reboot((osal_event_hdr_t*)pMsg);
        break;

    case OTAM_CMD_ONE_CLK:
        on_msg_one_clk((otam_cmd_conn_t*)pMsg);
        break;

    default:
        OTA_LOG("Unknow command!\n");
        break;
    }
}


static void on_gatt_read_rsp(attReadRsp_t* prsp)
{
}

static void on_gatt_write_rsp(void)
{
    otam_ctx_t* pctx = &m_otam_ctx;
    otac_dev_t* pdev = &(pctx->otac_dev);

    if(pdev->disc_state == DISC_ST_COMPLETED && pdev->op_handle == pdev->ota_service.char_rsp_cccd_hdl)
    {
        otap_evt_t otap_ev;
        otam_proto_conn_param_t conn_param;
        otap_ev.ev = OTAP_EVT_CONNECTED;
        otap_ev.len = sizeof(conn_param);
        conn_param.mtu = pdev->mtu;
        conn_param.run_mode = pdev->run_mode;
        otap_ev.data = (void*)(&conn_param);
        OTA_LOG("CCCD enabled\n");
        otamProtocol_event(&otap_ev);
        {
            oneclk_evt_t ev;
            ev.ev = ONECLK_EVT_CONNECTED;

            if(pdev->run_mode == OTAC_RUNMODE_OTA)
            {
                ev.ev = ONECLK_EVT_CONNECTED_OTA;
            }

            OTA_LOG("pdev->run_mode1:%d\n", pdev->run_mode);
            otam_oneclick_evt(&ev);
        }
    }
    else
    {
        oneclk_evt_t ev;
        ev.ev = ONECLK_EVT_WRITE_RSP;
        otam_oneclick_evt(&ev);
    }
}

static void on_gatt_notify(attHandleValueNoti_t* prsp)
{
    otam_ctx_t* pctx = &m_otam_ctx;
    otac_dev_t* pdev = &(pctx->otac_dev);

    if(pdev->disc_state == DISC_ST_COMPLETED && prsp->handle == pdev->ota_service.char_rsp_hdl)
    {
        osal_stop_timerEx(otaMasterTaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT);
        osal_clear_event(otaMasterTaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT);
        otap_evt_t otap_ev;
        otap_ev.ev = OTAP_EVT_NOTIFY;
        otap_ev.len = (uint16_t)(prsp->len);
        otap_ev.data = (void*)(prsp->value);
        otamProtocol_event(&otap_ev);
    }
}

static void on_gatt_indicate(attHandleValueInd_t* prsp)
{
    //do nothing
}

static void on_gatt_discovery(gattMsgEvent_t* pMsg)
{
    otam_ctx_t* pctx = &m_otam_ctx;
    otac_dev_t* pdev = &(pctx->otac_dev);
    uint8_t i;

    switch(pdev->disc_state)
    {
    case DISC_ST_INIT:
        //not started, just drop message
        break;

    case DISC_ST_SVC:
        if((pMsg->method == ATT_READ_BY_GRP_TYPE_RSP) && (pMsg->msg.readByGrpTypeRsp.numGrps > 0))
        {
            uint16_t len, numgrp;
            uint8_t* pdata;
            len = (uint16_t)(pMsg->msg.readByGrpTypeRsp.len);
            numgrp = (uint16_t)(pMsg->msg.readByGrpTypeRsp.numGrps);
            pdata = pMsg->msg.readByGrpTypeRsp.dataList;
            OTA_LOG("ATT_READ_BY_GRP_TYPE_RSP: numGrps %d\n", numgrp);
            print_hex(pdata, len * numgrp);

            if(len == 20)
            {
                for(i=0; i<numgrp; i++)
                {
                    //find ota service
                    if(osal_memcmp(pdata + 4, ota_ServiceUUID, ATT_UUID_SIZE) != 0)
                    {
                        pdev->attr_start_hdl = BUILD_UINT16( pdata[0], pdata[1]);
                        pdev->attr_end_hdl = BUILD_UINT16( pdata[2], pdata[3]);
                        break;
                    }
                }

                pdata += len;
            }
        }
        else if((pMsg->method == ATT_READ_BY_GRP_TYPE_RSP) && (pMsg->hdr.status == bleProcedureComplete))
        {
            bStatus_t bret = FAILURE;
            // Primary Service Discover OK , Prepare Discover Characteristic
            pdev->disc_state = DISC_ST_CHAR;
            osal_memset(&(pdev->ota_service), 0, sizeof(otac_service_t));

            if(pdev->attr_start_hdl != 0 &&  pdev->attr_end_hdl != 0)
            {
                bret = GATT_DiscAllChars(pdev->conn_hdl,pdev->attr_start_hdl,pdev->attr_end_hdl,otaMasterTaskId);
            }

            if(bret != SUCCESS)
            {
                OTA_LOG("Discovery failed, or can't find OTA service!\n");
                pdev->disc_state = DISC_ST_FAILED;
                g_ota_status = OTA_NO_SERVICE;
                osal_set_event(otaMasterTaskId, ONECLK_OP_NEXTDEV_EVT);
            }
        }

        break;

    case DISC_ST_CHAR:

        // Characteristic found, store handle
        if (pMsg->method==ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs>0)
        {
            uint16_t len, numpairs;
            uint8_t* pdata;
            len = (uint16_t)(pMsg->msg.readByTypeRsp.len);
            numpairs = (uint16_t)(pMsg->msg.readByTypeRsp.numPairs);
            pdata = pMsg->msg.readByTypeRsp.dataList;
            OTA_LOG("ATT_READ_BY_TYPE_RSP: numpairs %d\n", numpairs);
            print_hex(pdata, len * numpairs);

            for(i = 0; i < numpairs ; i++)
            {
                // Extract the starting handle, ending handle, and UUID of the current characteristic.
                // characteristic Handle
                if(osal_memcmp(pdata + 5, ota_CommandUUID, ATT_UUID_SIZE) != 0)
                {
                    pdev->ota_service.char_cmd_hdl = BUILD_UINT16( pdata[3],pdata[4]);
                }
                else if(osal_memcmp(pdata + 5, ota_ResponseUUID, ATT_UUID_SIZE) != 0)
                {
                    pdev->ota_service.char_rsp_hdl = BUILD_UINT16( pdata[3],pdata[4]);
                    pdev->ota_service.char_rsp_cccd_hdl = pdev->ota_service.char_rsp_hdl + 1;
                }
                else if(osal_memcmp(pdata + 5, ota_DataUUID, ATT_UUID_SIZE) != 0)
                {
                    pdev->ota_service.char_data_hdl = BUILD_UINT16( pdata[3],pdata[4]);
                }

                pdata += len;
            }
        }
        else if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->hdr.status == bleProcedureComplete )
        {
            //discovery completed
            OTA_LOG("discovery completed");
            OTA_LOG("Command handle: 0x%x\n", pdev->ota_service.char_cmd_hdl);
            OTA_LOG("Response handle: 0x%x\n", pdev->ota_service.char_rsp_hdl);
            OTA_LOG("Rsp CCCD handle: 0x%x\n", pdev->ota_service.char_rsp_cccd_hdl);
            OTA_LOG("Data handle: 0x%x\n", pdev->ota_service.char_data_hdl);
            pdev->disc_state = DISC_ST_COMPLETED;

            if(pdev->ota_service.char_data_hdl == 0)
            {
                pctx->mode_choose = 0;
                pdev->run_mode = OTAC_RUNMODE_APP;
            }
            else
            {
                pdev->run_mode = (pctx->mode_choose == OTA_MODE_RESOURCE) ? OTAC_RUNMODE_OTARES : OTAC_RUNMODE_OTA;
            }

            otac_cccd(true);
        }

        break;

    case DISC_ST_COMPLETED:
    default:
        break;
    }
}

static void on_gatt_err_rsp(attErrorRsp_t* prsp)
{
    switch(prsp->reqOpcode)
    {
    case ATT_READ_REQ:
    case ATT_WRITE_REQ:
    case ATT_HANDLE_VALUE_NOTI:
    case ATT_HANDLE_VALUE_IND:
    default:
        break;
    }
}

static void on_gatt_mtu_rsp(attExchangeMTURsp_t* prsp)
{
    otam_ctx_t* pctx = & m_otam_ctx;

    if(pctx->fsm_state == OTAM_FSM_CONNECTED)
    {
        pctx->otac_dev.mtu = (DEFAULT_OTAM_MTU_SIZE > prsp->serverRxMTU) ?
                             prsp->serverRxMTU : DEFAULT_OTAM_MTU_SIZE;
        OTA_LOG("MTU size rsp: %d | set: %d\n", prsp->serverRxMTU, pctx->otac_dev.mtu);
    }
}


#if 0
static void stack_printf(void)
{
    extern uint32_t  __initial_sp;
    uint32_t* initial_sp_addr = &__initial_sp;
    uint32_t* g_intstackbase_addr = (initial_sp_addr - 0x400); ///0x400 means: (Stack_Size(0x1000) / 4)
    OTA_LOG("ll_Stack total : %d Bytes,",((initial_sp_addr - g_intstackbase_addr)<<2));

    for(uint32_t* addr=g_intstackbase_addr; addr<initial_sp_addr; addr++)
    {
        if(*addr != 0 )
        {
            OTA_LOG("ll_used : %d Bytes\n",((initial_sp_addr - addr) << 2));
            break;
        }
    }
}
#endif



/*********************************************************************
    PUBLIC FUNCTIONS
*/
bool otaMaster_EstablishLink(uint8_t addr_type, uint8_t* addr)
{
    bStatus_t ret;
    char str_mac[20];
    sprintf(str_mac, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
    LOG("Start EstablishLink: %s\n", str_mac);
    ret = GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                       DEFAULT_LINK_WHITE_LIST, addr_type, addr);
    #if 0
    stack_printf();
    #endif

    if(ret == SUCCESS)
    {
        osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 20000);
        return true;
    }
    else
    {
        return false;
    }
}


/*********************************************************************
    @fn     Singlebank_OTAdongle_Init

    @brief  Initialization function for the OTA_Dongle App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notification).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void Singlebank_OTAdongle_Init( uint8 task_id )
{
    otaMasterTaskId = task_id;
    // Setup Central Profile
    {
        uint8 scanRes = DEFAULT_MAX_SCAN_RES;
        GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8), &scanRes);
    }
    // Setup GAP
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT,  40);     // 40*0.625ms = 25ms
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, 40);     // 40*0.625ms = 25ms
    GAP_SetParamValue( TGAP_CONN_EST_INT_MIN,  6);      // 6*1.25ms = 7.5ms
    GAP_SetParamValue( TGAP_CONN_EST_INT_MAX,  6);      // 6*1.25ms = 7.5ms
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_UPDATE_CONN_TIMEOUT);       // 10ms*100 = 1s
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8*)simpleBLEDeviceName);
    // Setup the GAP Bond Manager
    {
        uint32 passkey = DEFAULT_PASSCODE;
        uint8 pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING; //GAPBOND_PAIRING_MODE_INITIATE;//DEFAULT_PAIRING_MODE;    // GAPBOND_PAIRING_MODE_NO_PAIRING
        uint8 mitm = DEFAULT_MITM_MODE;
        uint8 ioCap = DEFAULT_IO_CAPABILITIES;
        uint8 bonding = DEFAULT_BONDING_MODE;
        GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8), &bonding);
    }
    // Initialize GATT Client
    VOID GATT_InitClient();
    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(otaMasterTaskId);
    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);         // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
    llInitFeatureSetDLE(TRUE);
    llInitFeatureSet2MPHY(TRUE);
    HCI_LE_SetDefaultPhyMode(0,0x00,0x03,0x03);
    ll_hw_set_tfifo_space(LL_HW_FIFO_TX_3K_RX_1K);
    osal_memset(m_otam_ctx.conn_dev_list, 0, sizeof(otac_conn_dev_t));
    otam_proto_meth_t op =
    {
        .clear            = otaMaster_method_clear,
        .write_cmd        = otaMaster_write_command,
        .write_data       = otaMaster_write_data,
        .write_data_delay = otaMaster_write_data_delay,
    };
    otamProtocol_init(&op);
    OTAM_FSM_SET(OTAM_FSM_IDLE);
    otam_oneclick_init();
    osal_set_event(otaMasterTaskId, START_DEVICE_EVT);
    osal_start_timerEx(otaMasterTaskId, ONECLK_OP_OTA_AUTOSCAN_EVT, 200);
    extern int load_fw(uint8_t fw_id);
    load_fw(0);
}

/*********************************************************************
    @fn     Singlebank_OTAdongle_ProcessEvent

    @brief  OTA_Dongle Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 Singlebank_OTAdongle_ProcessEvent(uint8 task_id, uint16 events)
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if(events & SYS_EVENT_MSG)
    {
        uint8* pMsg;

        if ((pMsg = osal_msg_receive(otaMasterTaskId)) != NULL)
        {
            otaMaster_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & START_DEVICE_EVT)
    {
        // Start the Device
        VOID GAPCentralRole_StartDevice((gapCentralRoleCB_t*)&simpleBLERoleCB);
        // Register with bond manager after starting device
        GAPBondMgr_Register((gapBondCBs_t*)&simpleBLEBondCB);
        return (events ^ START_DEVICE_EVT);
    }

    if(events & START_DISCOVERY_SERVICE_EVT)
    {
        otaMaster_StartDiscoveryService();
        return ( events ^ START_DISCOVERY_SERVICE_EVT );
    }

    if(events & OTA_DATA_DELAY_WRITE_EVT)
    {
        otap_evt_t ev;
        ev.ev = OTAP_EVT_DATA_WR_DELAY;
        ev.len = 0;
        ev.data = NULL;
        otamProtocol_event(&ev);
        return ( events ^ OTA_DATA_DELAY_WRITE_EVT );
    }

    if(events & BLE_CMD_WRITE_OP_TIMEOUT_EVT)
    {
        otap_evt_t ev;
        ev.ev = OTAP_EVT_BLE_TIMEOUT;
        ev.len = 0;
        ev.data = NULL;
        otamProtocol_event(&ev);
        return ( events ^ BLE_CMD_WRITE_OP_TIMEOUT_EVT );
    }

    if(events & OTA_APP_DISCONN_DELAY_EVT)
    {
        OTA_LOG("GAPCentralRole_TerminateLink\n");
        GAPCentralRole_TerminateLink(m_otam_ctx.otac_dev.conn_hdl);
        return (events ^ OTA_APP_DISCONN_DELAY_EVT);
    }

    if(events & START_PHY_DEL_MTU_EVT)
    {
        otac_dle_phy_mtu();
        return (events ^ START_PHY_DEL_MTU_EVT);
    }

    if(events & START_UPD_PHY_MODE_EVT)
    {
        extern uint8 llPermit_ctrl_procedure(uint16 connId);

        if(llPermit_ctrl_procedure(0))
        {
            uint8 allPhy = 0x00;
            uint8 txPhy = LE_2M_PHY;
            uint8 rxPhy = LE_2M_PHY;
            uint16 phyOption = 0x00;
            HCI_LE_SetPhyMode(0, allPhy, txPhy, rxPhy, phyOption);
            osal_start_timerEx(otaMasterTaskId, START_PHY_DEL_MTU_EVT, 10);
        }
        else
        {
            osal_start_timerEx(otaMasterTaskId, START_UPD_PHY_MODE_EVT, 10);
        }

        return ( events ^ START_UPD_PHY_MODE_EVT);
    }

    if(events & ONECLK_OP_TIMEOUT_EVT)
    {
        oneclk_evt_t ev;
        ev.ev = ONECLK_EVT_TIMER;
        otam_oneclick_evt(&ev);
        return (events ^ ONECLK_OP_TIMEOUT_EVT);
    }

    if(events & ONECLK_OP_NEXTDEV_EVT)
    {
        otam_ctx_t* pctx = &m_otam_ctx;
        uint8_t i = 0;
        static uint16_t success_cnt;
        static uint16_t failed_cnt;

        for(; i<DEFAULT_MAX_SCAN_RES; i++)
        {
            if(pctx->conn_dev_list[i].flags == 0)
            {
                pctx->conn_dev_list[i].index = i;
                pctx->conn_dev_list[i].flags = 1;
                break;
            }
        }

        if(i == DEFAULT_MAX_SCAN_RES)
        {
            for(uint8_t j=0; j<DEFAULT_MAX_SCAN_RES; j++)
            {
                if(pctx->conn_dev_list[j].index == 0)
                {
                    pctx->conn_dev_list[j].index = DEFAULT_MAX_SCAN_RES-1;
                    OTA_LOG("j:%d\n",j);
                    i = j;

                    for(uint8_t k=0; k<DEFAULT_MAX_SCAN_RES; k++)
                    {
                        if(k!=j) pctx->conn_dev_list[k].index--;
                    }

                    break;
                }
            }
        }

        pctx->conn_dev_list[i].ota_status = g_ota_status;
        pctx->conn_dev_list[i].sys_tick = hal_systick();
        osal_memcpy(pctx->conn_dev_list[i].addr, otam_oneclick_getmac(), B_ADDR_LEN);

        if(is_sencond_link()==1) pctx->conn_dev_list[i].addr[0]--;

        LOG("i:%d, ota_end %02x,%02x,%02x,%02x,%02x,%02x\n", i,
            pctx->conn_dev_list[i].addr[0],
            pctx->conn_dev_list[i].addr[1],
            pctx->conn_dev_list[i].addr[2],
            pctx->conn_dev_list[i].addr[3],
            pctx->conn_dev_list[i].addr[4],
            pctx->conn_dev_list[i].addr[5]);

        if(g_ota_status == OTA_SUCCESS)
        {
            success_cnt++;
            OTA_LOG("ota success:%d\n", success_cnt);
        }
        else
        {
            if(g_ota_status == OTA_NO_SERVICE)
            {
                otam_cmd_conn_t* pMsg = (otam_cmd_conn_t*)osal_msg_allocate(sizeof(otam_cmd_conn_t));

                if(pMsg)
                {
                    pMsg->hdr.event  = OTAM_MSG_EVENT;
                    pMsg->hdr.status = OTAM_CMD_DISCONNECT;
                    osal_msg_send(otaMasterTaskId, (uint8_t*)pMsg);
                }
                else
                {
                    NVIC_SystemReset();
                }
            }

            failed_cnt++;
            OTA_LOG("ota fail:%d %d\n", failed_cnt, g_ota_status);
        }

        g_ota_status = OTA_IDLE;
        otam_oneclick_ota();
        return (events ^ ONECLK_OP_NEXTDEV_EVT);
    }

    if(events & ONECLK_OP_CONN_PARAM_UPDATE)
    {
        oneclk_evt_t ev;
        ev.ev = ONECLK_EVT_PARAM_UPDATE;
        otam_oneclick_evt(&ev);
        return (events ^ ONECLK_OP_CONN_PARAM_UPDATE);
    }

    if(events & ONECLK_OP_OTA_AUTOSCAN_EVT)
    {
        otam_oneclick_ota();
        return (events ^ ONECLK_OP_OTA_AUTOSCAN_EVT);
    }

    if(events & OTA_PROCESS_TIMEOUT_EVT)
    {
        OTA_LOG("ota process timeout disconnect\n");
        GAPCentralRole_TerminateLink(0);
        return (events ^ OTA_PROCESS_TIMEOUT_EVT);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
    @fn      otaMaster_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void otaMaster_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    case GATT_MSG_EVENT:
        otaMaster_ProcessGATTMsg( (gattMsgEvent_t*) pMsg );
        break;

    case OTAM_MSG_EVENT:
        otaMaster_ProcessCMDMsg((otam_cmd_conn_t*)pMsg);
        break;

    default:
        break;
    }
}

/*********************************************************************
    @fn      otaMaster_ProcessGATTMsg

    @brief   Process GATT messages

    @return  none
*/

static void otaMaster_ProcessGATTMsg( gattMsgEvent_t* pMsg )
{
    otam_ctx_t* pctx = & m_otam_ctx;
    //OTA_LOG("[GATT] %x\n",pMsg->method);

    if(pctx->fsm_state != OTAM_FSM_CONNECTED)
    {
        // In case a GATT message came after a connection has dropped, ignore the message
        return;
    }

    if(pMsg->hdr.status == bleTimeout)
    {
        OTA_LOG("[GATT TO] %x\n",pMsg->method);
        on_msg_disconnect();
        otam_oneclick_ota();
        return;
    }

    switch(pMsg->method)
    {
    case ATT_READ_RSP:
        on_gatt_read_rsp(&(pMsg->msg.readRsp));
        break;

    case ATT_WRITE_RSP:
        on_gatt_write_rsp();
        break;

    case ATT_HANDLE_VALUE_NOTI:
        on_gatt_notify(&(pMsg->msg.handleValueNoti));
        break;

    case ATT_HANDLE_VALUE_IND:
        on_gatt_indicate(&(pMsg->msg.handleValueInd));
        break;

    case ATT_EXCHANGE_MTU_RSP:
        on_gatt_mtu_rsp(&(pMsg->msg.exchangeMTURsp));
        break;

    case ATT_ERROR_RSP:
        on_gatt_err_rsp(&(pMsg->msg.errorRsp));
        break;

    //below case is response for gatt discovery
    case ATT_READ_BY_GRP_TYPE_RSP:
    case ATT_READ_BY_TYPE_RSP:
        OTA_LOG("pMsg->method:%02x, status:%02x\n", pMsg->method, pMsg->hdr.status);
        on_gatt_discovery(pMsg);
        break;

    default:
        break;
    }
}

/*********************************************************************
    @fn      otaMasterRssiCB

    @brief   RSSI callback.

    @param   connHandle - connection handle
    @param   rssi - RSSI

    @return  none
*/
static void otaMasterRssiCB( uint16 connHandle, int8 rssi )
{
    OTA_LOG( "RSSI -[%02d]dB\r\n", (255-(uint8) (-rssi)) );
}

/*********************************************************************
    @fn      otaMasterEventCB

    @brief   Central event callback function.

    @param   pEvent - pointer to event structure

    @return  none
*/
extern uint8 ll_recv_scan_all_cnt;
static void otaMasterEventCB( gapCentralRoleEvent_t* pEvent )
{
    otam_ctx_t* pctx = &m_otam_ctx;

    switch ( pEvent->gap.opcode )
    {
    case GAP_DEVICE_INFO_EVENT:
        // A device has been detected
        m_conn_index = otac_scan_update_devinfo(pEvent);
        HAL_ENTER_CRITICAL_SECTION();

        if(ll_recv_scan_all_cnt != 0)
            ll_recv_scan_all_cnt--;

        HAL_EXIT_CRITICAL_SECTION();
        break;

    case GAP_DEVICE_DISCOVERY_EVENT:
        // Scanning is complete or terminated in advance
        OTAM_FSM_SET(OTAM_FSM_IDLE);
        otac_scan_completed();
        HAL_ENTER_CRITICAL_SECTION();
        ll_recv_scan_all_cnt = 0;
        HAL_EXIT_CRITICAL_SECTION();
        break;

    case GAP_LINK_ESTABLISHED_EVENT:
        OTA_LOG("\n== GAP_LINK_ESTABLISHED_EVENT ==\r\n");

        if ( pEvent->gap.hdr.status == SUCCESS )
        {
            osal_clear_event(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT);
            osal_stop_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT);
            otac_dev_connected(&(pEvent->linkCmpl));
            OTAM_FSM_SET(OTAM_FSM_CONNECTED);
        }
        else
        {
            OTA_LOG( "Connect Failed\n" );
            OTA_LOG( "Reason: 0x%02x\r\n", pEvent->gap.hdr.status);
            OTAM_FSM_SET(OTAM_FSM_IDLE);
            osal_set_event(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT);
        }

        break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        OTAM_FSM_SET(OTAM_FSM_IDLE);
        OTA_LOG( "Disconnected. " );
        OTA_LOG( "Reason: 0x%02x\r\n", pEvent->linkTerminate.reason);
        osal_memset(&pctx->otac_dev, 0, sizeof(otac_dev_t));
        pctx->otac_dev.conn_hdl = 0xffff;
        otap_evt_t ev;
        ev.ev = OTAP_EVT_DISCONNECTED;
        ev.data = NULL;
        ev.len = 0;
        otamProtocol_event(&ev);
        {
            oneclk_evt_t ev;
            ev.ev = ONECLK_EVT_TERMINATED;
            otam_oneclick_evt(&ev);
        }
    }
    break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
        gapLinkUpdateEvent_t* rsp = (gapLinkUpdateEvent_t*)pEvent;
        OTA_LOG( "Server Request for Param Update:%02x\r\n", rsp->status);
        OTA_LOG( "connInterval:%d, connLatency:%d, connTimeout:%d\n", rsp->connInterval, rsp->connLatency, rsp->connTimeout);
        osal_set_event(otaMasterTaskId, ONECLK_OP_CONN_PARAM_UPDATE);
    }
    break;

    default:
        break;
    }
}

/*********************************************************************
    @fn      pairStateCB

    @brief   Pairing state callback.

    @return  none
*/
static void otaMasterPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
    OTA_LOG("otaMasterPairStateCB in param state 0x%02X,status 0x%02X\r\n",state,status);

    if(state == GAPBOND_PAIRING_STATE_STARTED)
    {
        OTA_LOG( "Pairing started\n" );
    }
    else if(state == GAPBOND_PAIRING_STATE_COMPLETE)
    {
        if(status == SUCCESS)
        {
            OTA_LOG( "Pairing success\n" );
        }
        else
        {
            OTA_LOG( "Pairing fail\n" );
        }
    }
    else if(state == GAPBOND_PAIRING_STATE_BONDED)
    {
        if(status == SUCCESS)
        {
            OTA_LOG( "Bonding success\n" );
        }
    }
}

/*********************************************************************
    @fn      otaMasterPasscodeCB

    @brief   Passcode callback.

    @return  none
*/
static void otaMasterPasscodeCB( uint8* deviceAddr, uint16 connectionHandle,
                                 uint8 uiInputs, uint8 uiOutputs )
{
    OTA_LOG("otaMasterPasscodeCB\r\n");
}




/*********************************************************************
*********************************************************************/

