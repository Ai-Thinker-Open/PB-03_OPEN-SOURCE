/**************************************************************************************************
    Filename:       multi_role.c
    Revised:
    Revision:

    Description:    This file contains the Simple BLE Central sample application
                  for use with the Bluetooth Low Energy Protocol Stack.

**************************************************************************************************/

/*
    TODO:list
    1.multiple host lib
        (1).single master host lib -- maximum connection number equal to one ( 2K SRAM )
        (2).multi-role master host lib,maximum 8 commection supported
        (3).simultaneously 8 SMP
    2.multi role scheduler optimize(2's pointer)
    3.scan and initiating default parameter : eg scan/initiating whitelist parameter and so on.
    4.multi role active scan , no scan response data report
    5.multi role project system clock 16MHz not supported
    6.device in whitelist which has been bonded,  automatically link after reset(shall detected this in scan done event call back function ).
    7.what's high duty cycle in link mode
    8.multi role as master's action after establish sussess(first connection and re-connection should support different action--can use user flash area)
*/
/*********************************************************************
    INCLUDES
*/
#include <stdio.h>
#include <string.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_bufmgr.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gap.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "multi.h"
#include "gapbondmgr.h"
#include "ll_common.h"
#include "log.h"
#include "hci.h"
#include "error.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "devinfoservice.h"
#include "gatt_profile_uuid.h"
#include "multiRoleProfile.h"
#include "multi_role.h"
#include "multi_schedule.h"
#ifdef PHY_OTA_ENABLE
    #include "ota_app_service.h"
#endif
/*********************************************************************
    MACROS
*/



/*********************************************************************
    CONSTANTS
*/
// unit 1.25ms
#define DEFAULT_CONN_MIN_INTV                   40

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in 1000ms)
#define DEFAULT_CONN_PAUSE_PERIPHERAL           1

#define DEFAULT_ADV_INTV                        160

#define DEFAULT_SCAN_RSP_RSSI_MIN               -70

#define DEFAULE_SCAN_MAX_NUM                    5
// Scan duration in ms
#define DEFAULT_SCAN_DURATION                   50

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           200

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

#define DEFAULT_ACTION_AFTER_LINK           ( GAPMULTI_CENTRAL_MTU_EXCHANGE | GAPMULTI_CENTRAL_DLE_EXCHANGE | GAPMULTI_CENTRAL_SDP )

// Default passcode
#define DEFAULT_PASSCODE                      123456

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT //GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

#define INVALID_STATUS                          0xFFFF

#define DEFAULT_ENABLE_HID                      FALSE


/*********************************************************************
    GLOBAL VARIABLES
*/
uint16 MR_WakeupCnt = 0;

/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/
//static uint16 multiRole_wrChar_handle = 0;

// Task ID for internal task/event processing
uint8 multiRole_TaskId;
//
//MultiRoleApp_Link_t g_MRLink[MAX_CONNECTION_NUM];

/*********************************************************************
    LOCAL FUNCTIONS
*/
#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
    static void multiRoleAPP_AdvInit(void);
    static void multiRole_setup_adv_scanRspData(uint8 idx);
    static void multiRoleProfileChangeCB( uint16 connHandle,uint16 paramID, uint16 len );
#endif

#if( MAX_CONNECTION_MASTER_NUM > 0)
    static void multiRoleAPP_ScIn_Init(void);
    static void multiRoleSDPCB(void* msg );
    static void multiRoleNotifyCB(uint16 connHandle,uint16 len,uint8* data );
    static void multiRoleEachScanCB( gapDeviceInfoEvent_t* pPkt );
    static void multiRoleScanDoneCB( GAPMultiRolScanner_t* node );
#endif
static void multiRoleAPP_ComInit(void);
static void multiRoleEstablishCB( uint8 status, uint16 connHandle,GAPMultiRole_State_t role,uint8 perIdx,uint8* addr );
static void multiRoleTerminateCB( uint16 connHandle,GAPMultiRole_State_t role,uint8 perIdx,uint8 reason );
static void multiRoleApp_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void multiRoleAppProcessGATTMsg( gattMsgEvent_t* pMsg );
extern void ll_dbg_show(void);

extern void TRNG_INIT(void);
extern uint8_t TRNG_Rand(uint8_t* buf,uint8_t len);
extern uint8 gapMultiRole_TaskID;
/*********************************************************************
    LOCAL VARIABLES
*/
// TRUE if boot mouse enabled
uint8 hidBootMouseEnabled = FALSE;


/*********************************************************************
    PROFILE CALLBACKS
*/
// GAP Role Callbacks
static const gapMultiRolesCBs_t multiRoleCB =
{
    #if( MAX_CONNECTION_MASTER_NUM > 0)
    multiRoleEachScanCB,
    multiRoleScanDoneCB,
    multiRoleSDPCB,
    multiRoleNotifyCB,
    #endif
    multiRoleEstablishCB,
    multiRoleTerminateCB,
};



#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
// GATT Profile Callbacks
static multiProfileCBs_t multiRole_ProfileCBs =
{
    multiRoleProfileChangeCB,    // Charactersitic value change callback
//  #else
};
#endif
/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      SimpleBLECentral_Init

    @brief   Initialization function for the Simple BLE Central App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notification).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void multiRoleApp_Init( uint8 task_id )
{
    multiRole_TaskId = task_id;
    uint8   roleProfile = 0;
    TRNG_INIT();
    #if ( MAX_CONNECTION_SLAVE_NUM > 0 )
    {
        roleProfile |= GAP_PROFILE_PERIPHERAL;
        multiRoleAPP_AdvInit();
        llInitFeatureSetDLE(TRUE);
        // Initialize GATT attributes
        GGS_AddService( GATT_ALL_SERVICES );         // GAP
        GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
        #ifdef PHY_OTA_ENABLE
        ota_app_AddService();
        #endif
        MultiProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
        MultiProfile_RegisterAppCBs(&multiRole_ProfileCBs);
    }
    #endif
    #if ( MAX_CONNECTION_MASTER_NUM > 0 )
    {
        roleProfile |= GAP_PROFILE_CENTRAL;
        multiRoleAPP_ScIn_Init();
        LOG("Start muliSchedule_config \n");
    }
    #endif
    GAPMultiRole_SetParameter( GAPMULTIROLE_PROFILEROLE,sizeof(uint8),&roleProfile);
    multiRoleAPP_ComInit();
    #if( DEFAULT_PAIRING_MODE > GAPBOND_PAIRING_MODE_NO_PAIRING)
    // Setup the GAP Bond Manager
    {
        uint32 passkey = DEFAULT_PASSCODE;
        uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE;   // GAPBOND_PAIRING_MODE_NO_PAIRING
        uint8 mitm = DEFAULT_MITM_MODE;
        uint8 ioCap = DEFAULT_IO_CAPABILITIES;
        uint8 bonding = DEFAULT_BONDING_MODE;
        uint8 syncWL = TRUE;
        GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
        GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
        GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
        GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
        GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
        // If a bond is created, the HID Device should write the address of the
        // HID Host in the HID Device controller's white list and set the HID
        // Device controller's advertising filter policy to 'process scan and
        // connection requests only from devices in the White List'.
        GAPBondMgr_SetParameter( GAPBOND_AUTO_SYNC_WL, sizeof( uint8 ), &syncWL );
    }
    #endif
    osal_set_event( multiRole_TaskId, START_DEVICE_EVT );
}

/*********************************************************************
    @fn      SimpleBLECentral_ProcessEvent

    @brief   Simple BLE Central Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/

uint16 multiRoleApp_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
    uint8   numConns;

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( multiRole_TaskId )) != NULL )
        {
            multiRoleApp_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & START_DEVICE_EVT )
    {
        // Start the Device
        GAPMultiRole_StartDevice( (gapMultiRolesCBs_t*)&multiRoleCB, &numConns);

        if( numConns > MAX_NUM_LL_CONN )
            LOG("MAX Connection NUM configuration error\n");
        else
            LOG("MAX available connection %d\n",numConns);

        #if ( MAX_CONNECTION_SLAVE_NUM > 0 )
        {
            uint32_t add_adv_node = 0;

            for(uint8_t i=0; i<MAX_CONNECTION_SLAVE_NUM; i++)
            {
                add_adv_node = (add_adv_node&0xFFFFFFFF) | 1<<(i+4);
            }

            add_adv_node |= 0x01;

            if( SCH_SUCCESS == muliSchedule_config( MULTI_SCH_ADV_MODE, add_adv_node ) )
            {
                LOG("Multi Role advertising scheduler success :%x\n",add_adv_node);
            }
        }
        #endif
        #if ( MAX_CONNECTION_MASTER_NUM > 0 )
        {
            uint8 ret = muliSchedule_config( MULTI_SCH_SCAN_MODE, 0x01 );

            if( SCH_SUCCESS == ret )
            {
                LOG("Multi Role scanning scheduler success \n");
            }
            else
            {
                LOG("Multi Role scanning scheduler failure %d\n",ret);
            }
        }
        #endif
        osal_start_reload_timer(multiRole_TaskId,MULTIROLE_PERIOD_EVT,5000);
        return ( events ^ START_DEVICE_EVT );
    }

    if( events & MULTIROLE_PERIOD_EVT )
    {
        ll_dbg_show();
        return ( events ^ MULTIROLE_PERIOD_EVT );
    }

    // Discard unknown events
    return 0;
}


static void multiRoleAPP_ComInit(void)
{
    uint16 desired_min_interval = DEFAULT_CONN_MIN_INTV ;
    uint16 desired_slave_latency = DEFAULT_UPDATE_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_UPDATE_CONN_TIMEOUT;
    GAPMultiRole_SetParameter( GAPMULTIROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval);
    GAPMultiRole_SetParameter( GAPMULTIROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval);
    GAPMultiRole_SetParameter( GAPMULTIROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency);
    GAPMultiRole_SetParameter( GAPMULTIROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout);
}

/*********************************************************************
    @fn      simpleBLECentral_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void multiRoleApp_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    case GATT_MSG_EVENT:
        multiRoleAppProcessGATTMsg( (gattMsgEvent_t*) pMsg );
        break;
    }
}

static void multiRoleAppProcessGATTMsg( gattMsgEvent_t* pMsg )
{
    switch( pMsg->method )
    {
    case ATT_ERROR_RSP:
    {
        LOG("pMsg->msg.errorRsp.handle %d\n",pMsg->msg.errorRsp.handle);
        LOG("pMsg->msg.errorRsp.reqOpcode %d\n",pMsg->msg.errorRsp.reqOpcode);
        LOG("pMsg->msg.errorRsp.errCode %d\n",pMsg->msg.errorRsp.errCode);
    }
    break;

    case ATT_WRITE_RSP:
    {
        /// write multi service cccd
        attWriteReq_t pReq;
        pReq.sig = 0;
        pReq.cmd = 0;
        pReq.handle = 14;//multiRole_wrChar_handle ;
        pReq.len = 2;
        pReq.value[0] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY);
        pReq.value[1] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY >> 8);
        bStatus_t status = GATT_WriteCharValue(pMsg->connHandle,&pReq,multiRole_TaskId);

        if(status == SUCCESS)
        {
//              LOG("GATT_WriteCharValue success handle %d\n",pMsg->connHandle);
        }
        else
        {
            LOG("GATT_WriteCharValue ERROR %d\r\n",status);
        }
    }
    break;

    default:
        break;
    }
}

static void multiRoleEstablishCB( uint8 status,uint16 connHandle,GAPMultiRole_State_t role,uint8 perIdx,uint8* addr )
{
    if( status == SUCCESS )
    {
        LOG("Establish success connHandle %d,role %d\n",connHandle,role);
        LOG("Establish success connHandle %d,perIdx %d\n",connHandle,perIdx);

        if( role == Master_Role )
        {
            #if( MAX_CONNECTION_MASTER_NUM > 0)
            ///del the current node when the establishment is successful in multi_schedule.c
            // muliSchedule_config( MULTI_SCH_INITIATOR_MODE, 0x00 );

            if( multiGetSlaveConnList() != NULL )
            {
                ///bugfix: multi add init node 2022 08 05
                uint8_t scan_init_node_num = 0, curr_master_conn_num = 0;
                scan_init_node_num =  multiRole_findInitScanNode();
                curr_master_conn_num = multiLinkGetMasterConnNum();

                if(scan_init_node_num < (MAX_CONNECTION_MASTER_NUM - curr_master_conn_num))
                    muliSchedule_config( MULTI_SCH_INITIATOR_MODE, 0x01 );

                ///2022 08 08  prevent the number of connections  smaller than the configured master number,so add scan node to find others slave devices
                if( multiLinkGetMasterConnNum() < MAX_CONNECTION_MASTER_NUM )
                {
                    muliSchedule_config( MULTI_SCH_SCAN_MODE, 0x01 );
                }
            }
            else if( multiLinkGetMasterConnNum() < MAX_CONNECTION_MASTER_NUM )
            {
                muliSchedule_config( MULTI_SCH_SCAN_MODE, 0x01 );
            }
            else
            {
                ///2023 04 25 add: restart multi schedule state machine
                if(osal_get_timeoutEx(gapMultiRole_TaskID,MULTI_SCHEDULE_EVT) == 0)
                {
                    AT_LOG("restart multi schedule state machine\n");
                    osal_start_timerEx(gapMultiRole_TaskID,MULTI_SCHEDULE_EVT,MULTI_SCH_DELAY);
                }
            }

            #endif
        }
        else
        {
            // slave role establish success, delete advertising schedule node
            // note: est conn success, del the busy node (ADV node) by calling the multiConfigLink_status function in the multiconfig. c file
        }
    }
    else
    {
        LOG("Establish failure status %d\n",status);
        #if( MAX_CONNECTION_MASTER_NUM > 0)
        /// need judge curr node role 2022 08 05
        multiScehdule_t* bNode = multiRole_findBusyNode();

        if( bNode->role == initiator )
        {
            extern void multiChangeCurrentConnNodeOrder(void);
            /// change current conn node order,prevent constant connection to this conn node
            multiChangeCurrentConnNodeOrder();
            muliSchedule_config( MULTI_SCH_INITIATOR_MODE, 0x00 );

            if( multiGetSlaveConnList() != NULL )
            {
                muliSchedule_config( MULTI_SCH_INITIATOR_MODE, 0x01 );
            }
            else
            {
                AT_LOG("Establish failure\n");
                // muliSchedule_config( MULTI_SCH_SCAN_MODE, 0x01 );
            }
        }

        #endif
    }
}

static void multiRoleTerminateCB( uint16 connHandle,GAPMultiRole_State_t role,uint8 perIdx,uint8 reason )
{
    LOG("Terminate connHandle %d,role %d,perIdx %d, reason %d\n",connHandle,role,perIdx,reason );
    #if ( MAX_CONNECTION_SLAVE_NUM > 0 )

    if( role == Slave_Role )
    {
        // 0x01 : enable flag
        uint32 en_flag = ( 1 << ( 4 + perIdx) ) | 0x01 ;

        if( SCH_SUCCESS == muliSchedule_config( MULTI_SCH_ADV_MODE, en_flag ) )
        {
            LOG("Multi Role advertising scheduler re-enable success :%d\n",perIdx);
        }
    }

    #endif
    #if( MAX_CONNECTION_MASTER_NUM > 0)

    if( role == Master_Role )
    {
        if( multiGetSlaveConnList() != NULL )
        {
            ///bugfix: multi add init node 2022 08 05
            uint8_t scan_init_node_num = 0, curr_master_conn_num = 0;
            scan_init_node_num =  multiRole_findInitScanNode();
            curr_master_conn_num = multiLinkGetMasterConnNum();

            if(scan_init_node_num < (MAX_CONNECTION_MASTER_NUM - curr_master_conn_num))
                muliSchedule_config( MULTI_SCH_INITIATOR_MODE, 0x01 );
            else
            {
                ///2023 04 25 add: restart multi schedule state machine
                if(osal_get_timeoutEx(gapMultiRole_TaskID,MULTI_SCHEDULE_EVT) == 0)
                {
                    AT_LOG("restart multi schedule state machine\n");
                    osal_start_timerEx(gapMultiRole_TaskID,MULTI_SCHEDULE_EVT,MULTI_SCH_DELAY);
                }
            }
        }
        else
            AT_LOG("error\n");
    }

    #endif
}

#if ( MAX_CONNECTION_SLAVE_NUM > 0 )
static void multiRoleAPP_AdvInit(void)
{
    // advertising channel map
    uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39;
    GAPMultiRole_SetParameter( GAPMULTIROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);
    // only support undirect connectable advertise
    uint8 advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;
    GAPMultiRole_SetParameter( GAPMULTIROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType);
    // peripheral device delay of Conndelay seconds initiate  parameter update request
    uint8 param_update_enable = DEFAULT_ENABLE_UPDATE_REQUEST;
    GAPMultiRole_SetParameter( GAPMULTIROLE_PARAM_UPDATE_ENABLE,sizeof( uint8 ), &param_update_enable);
    uint16 Conndelay = DEFAULT_CONN_PAUSE_PERIPHERAL;
    GAPMultiRole_SetParameter( TGAP_CONN_PAUSE_PERIPHERAL,sizeof(uint16), &Conndelay);
    uint8 advFilterPolicy = GAP_FILTER_POLICY_ALL;
    GAPMultiRole_SetParameter(GAPMULTIROLE_ADV_FILTER_POLICY,sizeof( uint8 ),&advFilterPolicy);
    // limit & general & connection discovery mode use the same advertising interval
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN,DEFAULT_ADV_INTV);
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX,DEFAULT_ADV_INTV);
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN,DEFAULT_ADV_INTV);
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN,DEFAULT_ADV_INTV);
    GAP_SetParamValue( TGAP_CONN_ADV_INT_MIN,DEFAULT_ADV_INTV);
    GAP_SetParamValue( TGAP_CONN_ADV_INT_MAX,DEFAULT_ADV_INTV);

    for( uint8 i=0; i<MAX_CONNECTION_SLAVE_NUM; i++)
        multiRole_setup_adv_scanRspData(i);

    // GAP GATT Attributes -- device name & appearance ...
    uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE MultiRole";
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8*) simpleBLEDeviceName );
}
void multiRoleProfileChangeCB( uint16 connHandle,uint16 paramID, uint16 len )
{
    uint8 newValue[ATT_MTU_SIZE];

    switch( paramID )
    {
    case MULTIPROFILE_CHAR1:
        MultiProfile_GetParameter( connHandle,MULTIPROFILE_CHAR1, newValue );
        MultiProfile_Notify(connHandle,MULTIPROFILE_CHAR2,len,newValue);
        break;

    case MULTIPROFILE_CHAR2:
        MultiProfile_GetParameter( connHandle,MULTIPROFILE_CHAR2, newValue );
        break;

    default:
        break;
    }
}

static void multiRole_setup_adv_scanRspData(uint8 idx)
{
    // Multi-role Advertising Data & Scan Response Data
    uint8 gapMultiRole_AdvertData[] =
    {
        // flags
        0x02,
        GAP_ADTYPE_FLAGS,
        GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED
    };
    uint8  gapMultiRole_ScanRspData[] =
    {
        0x0C,                             // length of this data
        GAP_ADTYPE_LOCAL_NAME_COMPLETE, // AD Type = Complete local name
        'M','u','l','t','i','-','R','o','l','e','x'
    };
    uint8 bufLen = sizeof(gapMultiRole_ScanRspData);
    gapMultiRole_ScanRspData[ bufLen-1 ] = idx + 0x30 ;
    multiSchedule_advParam_init(idx,GAPMULTIROLE_ADVERT_DATA,sizeof( gapMultiRole_AdvertData ), gapMultiRole_AdvertData);
    multiSchedule_advParam_init(idx,GAPMULTIROLE_SCAN_RSP_DATA,sizeof( gapMultiRole_ScanRspData ), gapMultiRole_ScanRspData);
}

#endif

#if( MAX_CONNECTION_MASTER_NUM > 0)

static void multiRoleAPP_ScIn_Init(void)
{
    // expect connection parameter
    uint16 EstMIN = DEFAULT_CONN_MIN_INTV;
    GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, EstMIN );
    GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, EstMIN );
    GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT,DEFAULT_UPDATE_CONN_TIMEOUT );
    // default scan duration
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
    GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
    uint8 maxScanNum = DEFAULE_SCAN_MAX_NUM;
    GAPMultiRole_SetParameter(GAPMULTIROLE_MAX_SCAN_RES,sizeof( uint8 ), &maxScanNum);
    uint8 scanMode = DEFAULT_DISCOVERY_MODE;
    GAPMultiRole_SetParameter(GAPMULTIROLE_SCAN_MODE,sizeof( uint8 ), &scanMode);
    uint8 activeScan = DEFAULT_DISCOVERY_ACTIVE_SCAN;
    GAPMultiRole_SetParameter(GAPMULTIROLE_ACTIVE_SCAN,sizeof( uint8 ), &activeScan);
    uint8 scanWhitelist = DEFAULT_DISCOVERY_WHITE_LIST;
    GAPMultiRole_SetParameter(GAPMULTIROLE_SCAN_WHITELIST,sizeof( uint8 ), &scanWhitelist);
    uint8 linkDutyCycle = DEFAULT_LINK_HIGH_DUTY_CYCLE;
    GAPMultiRole_SetParameter(GAPMULTIROLE_LINK_HIGHDUTYCYCLE,sizeof( uint8 ), &linkDutyCycle);
    uint8 linkWhitelist = DEFAULT_LINK_WHITE_LIST;
    GAPMultiRole_SetParameter(GAPMULTIROLE_LINK_WHITELIST,sizeof( uint8 ), &linkWhitelist);
    uint16 actionAfterLink = DEFAULT_ACTION_AFTER_LINK;
    GAPMultiRole_SetParameter( GAPMULTIROLE_ACTION_AFTER_LINK, sizeof( uint16 ), &actionAfterLink );
    GAP_SetParamValue( TGAP_SCAN_RSP_RSSI_MIN, ((uint16)DEFAULT_SCAN_RSP_RSSI_MIN) );
    // prepare conn device MAC Addr
    // attention : addr LSB First
    uint8 addrType = ADDRTYPE_PUBLIC;
    uint8 addr[B_ADDR_LEN]= {0x00,0x45,0x56,0x89,0x66,0x66};

    for(uint8 i = 0; i < MAX_CONNECTION_MASTER_NUM ; i++)
    {
        addr[0] = 0x66 + i;
        multiConfigSlaveList(addrType,addr);
    }

    LOG("multi-role as master add slave MAC address\n");
}

static void multiRoleSDPCB( void* msg )
{
    GAPMultiRole_CentralSDP_t* sdp_info = (GAPMultiRole_CentralSDP_t*)msg;
//  while( sdp_info )
    {
        // should display peer mac address
        // MAC address display when connection established
        LOG("connHandle = %d\n",sdp_info->connHandle);
        GATTReadGroupRsp* primservice = sdp_info->service.PrimServ;
        LOG("primary service \n" );

        while( primservice )
        {
            LOG("start handle %d\n",primservice->StHandle);
            LOG("  end handle %d\n",primservice->EGHandle);
//          LOG("    uuid_Len %d\n",primservice->uuid_Len);
            LOG("        uuid ");

            for(uint8 k=0; k<primservice->uuid_Len; k++)
            {
                LOG("0x%02X,",primservice->uuid[k]);
            }

            LOG("\n\n");
            primservice = primservice->next;
        }

        Characteristic* charac = sdp_info->service.Charac;
        LOG("characteristic \n" );

        while( charac )
        {
            LOG(" char handle %d\n",charac->charHandle);
            LOG("  Properties 0x%02X\n",charac->Properties);
            LOG("value handle %d\n",charac->valueHandle);
//          LOG("    uuid_Len %d\n",charac->uuid_Len);
            LOG("        uuid ");

            for(uint8 k=0; k<charac->uuid_Len; k++)
            {
                LOG("0x%02X,",charac->uuid[k]);
            }

            LOG("\n\n");

//          multiRole_wrChar_handle = charac->valueHandle;
            if( charac->Properties & GATT_PROP_NOTIFY )
            {
                attWriteReq_t pReq;
                pReq.sig = 0;
                pReq.cmd = 0;
                pReq.handle = charac->valueHandle + 1 ;
                pReq.len = 2;
                pReq.value[0] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY);
                pReq.value[1] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY >> 8);
                bStatus_t status = GATT_WriteCharValue(sdp_info->connHandle,&pReq,multiRole_TaskId);

                if(status == SUCCESS)
                {
                    LOG("GATT_WriteCharValue Notify success handle %d\n",sdp_info->connHandle);
                }
                else
                {
                    LOG("GATT_WriteCharValue Notify ERROR %d\r\n",status);
                }
            }

            charac = charac->next;
        }

//      sdp_info = sdp_info->next;
    }
}

static void multiRoleNotifyCB(uint16 connHandle,uint16 len,uint8* data )
{
    LOG("Notify success connHanle %d,len %d\n",connHandle,len);

    for(unsigned char i = 0; i < len; i++)
    {
        LOG( "0x%02X,",data[i]);
    }

    LOG("\n");
}

static void multiRoleEachScanCB( gapDeviceInfoEvent_t* pPkt )
{
//  LOG( bdAddr2Str( pPkt->addr ) );
//  LOG("\n");
    // TODO: cancel disvocery check logic
//  GAPMultiRole_CancelDiscovery();
    // notes : Application can get more info form pointer pPkt
    // eg:
    // addrType , adv event type , rssi , advData ...
}

static void multiRoleScanDoneCB( GAPMultiRolScanner_t* node )
{
    muliSchedule_config( MULTI_SCH_SCAN_MODE, 0x00 );

    while( node )
    {
        if( ( multiDevInConfSlaveList( node->addr ) ) && ( multi_devInLinkList( node->addr ) == FALSE) )
        {
            multiAddSlaveConnList( node->addrtype,node->addr );
        }

//        LOG("node %p\n",node);
//        LOG( bdAddr2Str( node->addr ) );
//        LOG("--addrtype %d\n",node->addrtype);

        if( node->advDatalen > 0)
        {
//            LOG("advDatalen %d\n",node->advDatalen);
//            for( uint8 i=0; i<node->advDatalen; i++)
//                LOG("0x%02X,",node->advData[i]);
//            LOG("\n");
        }

        if( node->scanRsplen > 0 )
        {
//            LOG("scanRsplen %d\n",node->scanRsplen);
//            for( uint8 i=0; i<node->scanRsplen; i++)
//                LOG("0x%02X,",node->rspData[i]);
//            LOG("\n");
        }

//        LOG("\n");
        node = node->next;
    }

    if( multiGetSlaveConnList() != NULL )
    {
        ///bugfix: multi add init node 2022 08 05
        uint8_t scan_init_node_num = 0, curr_master_conn_num = 0;
        scan_init_node_num =  multiRole_findInitScanNode();
        curr_master_conn_num = multiLinkGetMasterConnNum();

        if(scan_init_node_num < (MAX_CONNECTION_MASTER_NUM - curr_master_conn_num))
            muliSchedule_config( MULTI_SCH_INITIATOR_MODE, 0x01 );
    }
    else
    {
        //2022 02 14 Increased probability of scanning adv devices
        uint8  rand_char;
        TRNG_Rand(&rand_char,1);
        GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION + (rand_char/*&0x0F*/) );
        GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION + (rand_char/*&0x0F*/) );
        muliSchedule_config( MULTI_SCH_SCAN_MODE, 0x01 );
    }
}

#endif

/*********************************************************************
*********************************************************************/
