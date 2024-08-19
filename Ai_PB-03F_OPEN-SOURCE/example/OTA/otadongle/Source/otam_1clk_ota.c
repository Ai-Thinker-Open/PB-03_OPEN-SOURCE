

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
#include "gap_internal.h"
#include "simpleGATTprofile_ota.h"
#include "singlebank_otadongle.h"
#include "timer.h"
#include "log.h"
#include "ll_def.h"
#include "global_config.h"
#include "flash.h"
#include "rflib.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "otam_protocol.h"
#include "otam_1clk_ota.h"
#include "error.h"


typedef struct
{
    uint8_t     state;
    uint8_t     relink;
    uint8_t     retry;
    uint8_t     mac_orig[6];
    int         mac_index;
    int         mac_num;
    uint8_t*    mac_list;
} oneclk_ota_ctx_t;

static oneclk_ota_ctx_t s_1clk_ctx;
static uint8_t s_mac_ota[6] = {0};


extern ota_s_e g_ota_status;

static void show_error(int errorcode, uint8_t state)
{
    OTA_LOG("show_error: %d, %d\n",errorcode, state);
}

static void show_info(const char* infostr)
{
    OTA_LOG("show_info: %s",infostr);
}

#define MAC_LIST_ADDR       0x11058000
#define MAC_LIST_ADDR_SIZE  2000
#define MAC_LIST_ADDR_END   (MAC_LIST_ADDR + MAC_LIST_ADDR_SIZE*6)

static const uint8_t s_mac_used[8] = {0x00,0x00,0x00,0x00,0x00,0x00, 0x00, 0x00};
static const uint8_t s_mac_uninit[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

void init_maclist(void)
{
    oneclk_ota_ctx_t* pctx = &s_1clk_ctx;
    uint8_t* p_mac = (uint8_t*)MAC_LIST_ADDR;
    pctx->mac_list = p_mac;
    pctx->mac_index = 0;
    pctx->mac_num = 0;

    while(1)
    {
        if(memcmp(p_mac, s_mac_uninit, 6) == 0)
        {
            break;
        }

        pctx->mac_num++;
        p_mac += 8;
    }
}


uint8_t* pull_from_maclist(void)
{
    oneclk_ota_ctx_t* pctx = &s_1clk_ctx;
    uint8_t* p_mac = (uint8_t*)(MAC_LIST_ADDR + (pctx->mac_index*8));

    while(1)
    {
        if(memcmp(p_mac, s_mac_uninit, 6) == 0)
        {
            return NULL;
        }

        if(memcmp(p_mac, s_mac_used, 6) != 0)
        {
            pctx->mac_index++;
            break;
        }

        pctx->mac_index++;
        p_mac = (uint8_t*)(MAC_LIST_ADDR + (pctx->mac_index*8));
    }

    return p_mac;
}


int otam_oneclick_app_scan_wl(void)
{
    osal_event_hdr_t* pev;
    uint8_t* pmac = NULL;
    uint8_t mac[6];
    //config scan white list
    LL_ClearWhiteList();
    pmac = pull_from_maclist();

    if(!pmac)
    {
        return PPlus_ERR_NOT_FOUND;
    }

    mac[0] = pmac[5];
    mac[1] = pmac[4];
    mac[2] = pmac[3];
    mac[3] = pmac[2];
    mac[4] = pmac[1];
    mac[5] = pmac[0];
    LL_AddWhiteListDevice(mac, ADDRTYPE_PUBLIC);
    //start scan
    pev = (osal_event_hdr_t*)osal_msg_allocate((uint16)(sizeof (osal_event_hdr_t)));

    if(pev)
    {
        pev->event = OTAM_MSG_EVENT;
        pev->status = OTAM_CMD_SCAN_WL;
        osal_msg_send( otaMasterTaskId, (uint8*)pev);
        return PPlus_SUCCESS;
    }

    return PPlus_ERR_NO_MEM;
}


static int otam_oneclick_app_scan_f(void)
{
    osal_event_hdr_t* pev;
    //start scan
    pev = (osal_event_hdr_t*)osal_msg_allocate( (uint16)(sizeof (osal_event_hdr_t)) );

    if(pev)
    {
        pev->event = OTAM_MSG_EVENT;
        pev->status = OTAM_CMD_SCAN_FAST;
        osal_msg_send( otaMasterTaskId, (uint8*)pev);
        return PPlus_SUCCESS;
    }

    return PPlus_ERR_NO_MEM;
}


static int otam_oneclick_app_conn(uint8_t* mac)
{
    bool bret;

    if(s_1clk_ctx.state != ONECLK_ST_APP_IDLE
            && s_1clk_ctx.state != ONECLK_ST_APP_CONNECTING
            && s_1clk_ctx.state != ONECLK_ST_OTA_FINISH)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    memcpy(s_mac_ota, mac, 6);
    bret = otaMaster_EstablishLink(ADDRTYPE_PUBLIC, mac);

    if(bret == FALSE)
    {
        return PPlus_ERR_BLE_FAIL;
    }

    return PPlus_SUCCESS;
}


int otam_oneclick_ota_conn(void)
{
    bStatus_t bret;

    if(s_1clk_ctx.state != ONECLK_ST_OTA_CONNECTING && s_1clk_ctx.state != ONECLK_ST_APP_MODE)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    bret = otaMaster_EstablishLink(ADDRTYPE_PUBLIC, (uint8_t*)s_mac_ota);
    OTA_LOG("otaMaster_EstablishLink %d\n", bret);
    return PPlus_SUCCESS;
}


uint8 checksum(const uint8* data, uint8 len)
{
    uint8 i = 0;
    uint8 chksum = 0;

    //return 0;
    for(i = 0; i < len; i ++)
    {
        chksum = chksum^data[i];
    }

    return chksum;
}


void otam_oneclick_evt(oneclk_evt_t* pev)
{
    OTA_LOG("s_1clk_ctx.state is %d,evt is %d\n", s_1clk_ctx.state,pev->ev);

    switch(pev->ev)
    {
    case ONECLK_EVT_CONNECTED:
        s_1clk_ctx.retry = 0;

        if(s_1clk_ctx.state == ONECLK_ST_APP_CONNECTING)
        {
            otamProtocol_app_start_ota(2);
            osal_start_timerEx(otaMasterTaskId, OTA_APP_DISCONN_DELAY_EVT, 100);
            s_1clk_ctx.state = ONECLK_ST_APP_MODE;
        }
        else if(s_1clk_ctx.state == ONECLK_ST_OTA_CONNECTING)
        {
            otamProtocol_start_ota(0);
            s_1clk_ctx.state = ONECLK_ST_OTAING;
        }

        break;

    case ONECLK_EVT_CONNECTED_OTA:
        s_1clk_ctx.state = ONECLK_ST_OTAING;
        otamProtocol_start_ota(0);
        break;

    case ONECLK_EVT_SCAN_RESULT:
        if(pev->param.mac[0] == 0 &&
                pev->param.mac[1] == 0 &&
                pev->param.mac[2] == 0 &&
                pev->param.mac[3] == 0 &&
                pev->param.mac[4] == 0 &&
                pev->param.mac[5] == 0 )
        {
            show_error(PPlus_ERR_INVALID_ADDR, s_1clk_ctx.state);
            break;
        }

        memcpy(s_1clk_ctx.mac_orig, pev->param.mac, 6);
        otam_oneclick_app_conn(pev->param.mac);
        s_1clk_ctx.state = ONECLK_ST_APP_CONNECTING;
        break;

    case ONECLK_EVT_TERMINATED:
        OTA_LOG("Terminated\n");
        osal_stop_timerEx(otaMasterTaskId, START_DISCOVERY_SERVICE_EVT);

        if(s_1clk_ctx.state == ONECLK_ST_APP_CONNECTING)
        {
            s_1clk_ctx.retry++;

            if(s_1clk_ctx.retry >= 3)
            {
                g_ota_status = OTA_CONNECT_FAIL;
                show_error(PPlus_ERR_BLE_BUSY, s_1clk_ctx.state);
                osal_set_event(otaMasterTaskId, ONECLK_OP_NEXTDEV_EVT);
                return;
            }

            OTA_LOG("app disconnect\n");
            otam_oneclick_app_conn(s_1clk_ctx.mac_orig);
            osal_clear_event(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT);
            osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 10000);
            s_1clk_ctx.state = ONECLK_ST_APP_CONNECTING;
        }
        else if(s_1clk_ctx.state == ONECLK_ST_OTA_CONNECTING || s_1clk_ctx.state == ONECLK_ST_OTAING || s_1clk_ctx.state == ONECLK_ST_OTA_CONN_UPDATE)
        {
            s_1clk_ctx.retry++;

            if(s_1clk_ctx.retry >= 3)
            {
                g_ota_status = OTA_CONNECT_FAIL;
                show_error(PPlus_ERR_BLE_BUSY, s_1clk_ctx.state);
                osal_set_event(otaMasterTaskId, ONECLK_OP_NEXTDEV_EVT);
                return;
            }

            OTA_LOG("ota disconnect\n");
            s_1clk_ctx.state = ONECLK_ST_OTA_CONNECTING;
            otam_oneclick_ota_conn();
            osal_clear_event(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT);
            osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 10000);
        }
        else if(s_1clk_ctx.state == ONECLK_ST_APP_MODE)
        {
            OTA_LOG("app disconnect relink\n");
            s_1clk_ctx.retry = 0;
            osal_memcpy(s_mac_ota, otam_oneclick_getmac(), 6);
            s_mac_ota[0]++;
            s_1clk_ctx.relink = 1;
            otam_oneclick_ota_conn();
            osal_memcpy(otam_oneclick_getmac(), s_mac_ota, 6);
            osal_clear_event(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT);
            osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 10000);
            s_1clk_ctx.state = ONECLK_ST_OTA_CONNECTING;
        }
        else if(g_ota_status != OTA_SUCCESS)
        {
            show_error(0xFE, s_1clk_ctx.state);
            otam_oneclick_ota();
        }

        break;

    case ONECLK_EVT_TIMER:
        if(s_1clk_ctx.state == ONECLK_ST_APP_CONNECTING)
        {
            OTA_LOG("app connect timeout\n");
            s_1clk_ctx.retry++;
            gapCancelLinkReq(otaMasterTaskId, GAP_CONNHANDLE_ALL);

            if(s_1clk_ctx.retry >= 3)
            {
                g_ota_status = OTA_CONNECT_FAIL;
                show_error(PPlus_ERR_NOT_FOUND, s_1clk_ctx.state);
                osal_set_event(otaMasterTaskId, ONECLK_OP_NEXTDEV_EVT);
                return;
            }

            otam_oneclick_app_conn(s_1clk_ctx.mac_orig);
        }
        else if(s_1clk_ctx.state == ONECLK_ST_OTA_CONNECTING)
        {
            OTA_LOG("ota connect timeout\n");
            s_1clk_ctx.retry++;
            gapCancelLinkReq(otaMasterTaskId, GAP_CONNHANDLE_ALL);

            if(s_1clk_ctx.retry >= 3)
            {
                g_ota_status = OTA_CONNECT_FAIL;
                show_error(PPlus_ERR_NOT_FOUND, s_1clk_ctx.state);
                osal_set_event(otaMasterTaskId, ONECLK_OP_NEXTDEV_EVT);
                return;
            }

            otam_oneclick_ota_conn();
            osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 10000);
        }
        else
        {
            show_error(0xFE, s_1clk_ctx.state);
            otam_oneclick_ota();
        }

        break;

    case ONECLK_EVT_OTA_FINISHED:
        show_info("OTA completed\n");
        g_ota_status = OTA_SUCCESS;
        osal_start_timerEx(otaMasterTaskId, ONECLK_OP_NEXTDEV_EVT, 100);
        s_1clk_ctx.state = ONECLK_ST_APP_IDLE;
        s_1clk_ctx.retry = 0;
        OTA_LOG("\n\n\nNext!\n\n");
        break;

    default:
        OTA_LOG("No Operation\n");
        break;
    }
}


uint8_t* otam_oneclick_getmac(void)
{
    return s_1clk_ctx.mac_orig;
}


uint8_t is_sencond_link(void)
{
    return s_1clk_ctx.relink;
}

/************************
    1) scan with white list
    2) connect app fw
    3) ota mode
    4) connect ota and do ota with new fw
    5) reboot device
************************/
void otam_oneclick_ota(void)
{
    int ret;
    ret= otam_oneclick_app_scan_f();

    if(ret)
    {
        show_error(ret, s_1clk_ctx.state);
    }
    else
    {
        memset(s_1clk_ctx.mac_orig, 0, 6);
        s_1clk_ctx.state = 0;
        s_1clk_ctx.relink = 0;
//        show_info("App Mode connecting\n");
    }
}


void otam_oneclick_init(void)
{
    memset(&s_1clk_ctx, 0, sizeof(s_1clk_ctx));
    //init_maclist();
}

