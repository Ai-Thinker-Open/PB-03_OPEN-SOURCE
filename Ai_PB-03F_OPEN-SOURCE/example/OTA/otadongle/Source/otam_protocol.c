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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "otam_protocol.h"
#include "ota_app_service.h"
#include "ota_protocol.h"
#include "otam_1clk_ota.h"
#include "error.h"
#include "crc16.h"

#define OTAF_BASE_ADDR          0x11000000
#define OTAF_END_ADDR           0x1107ffff


#define MAX_SECT_SUPPORT        16

enum
{
    OTAM_ST_DISCONNECT = 0,
    OTAM_ST_CONNECTED,  //connected, idle
    OTAM_ST_WAIT_STARTED,
    OTAM_ST_PARAM,
    OTAM_ST_WAIT_PARTITION_INFO,
    OTAM_ST_DATA,
    OTAM_ST_COMPLETE,
    OTAM_ST_ERROR,
    OTAM_ST_CANCELING,  //wait stop
};


typedef struct
{
    uint32_t  flash_addr;
    uint32_t  run_addr;
    uint32_t  size;
    uint16_t  checksum;
} ota_fw_part_t;

typedef struct
{
    uint8_t         part_num;
    uint8_t         part_current;
    uint32_t        total_size;
    uint32_t        total_offset;
    uint32_t        offset;
    ota_fw_part_t   part[MAX_SECT_SUPPORT];

    //data cache for a block transmit
    uint32_t        cache_size;
    uint32_t        cache_offset;
    uint8_t         cache_retry;
    uint8_t*        cache;
} otam_fw_t;


typedef struct
{
    uint8_t             state;
    uint8_t             run_mode;
    bool                reset_mode;
    uint16_t            mtu_a;
    uint16_t            burst_size;
    uint8_t             opcode;
    otam_fw_t           fw;
    otam_proto_meth_t   method;
} otam_proto_ctx_t;


static otam_proto_ctx_t s_otap_ctx_t;
static otam_fw_t m_otam_fw;




static int otam_proto_ctx_reset(uint8_t st)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_proto_meth_t* method = &(pctx->method);
    pctx->state = st;
    pctx->opcode = 0;
    memset(&(pctx->fw), 0, sizeof(otam_fw_t));
    osal_memcpy(&pctx->fw, &m_otam_fw, sizeof(otam_fw_t));
    method->clear();
    return PPlus_SUCCESS;
}


static void otam_proto_disconnect(void* param)
{
    otam_proto_ctx_reset(OTAM_ST_DISCONNECT);
}


static void otam_proto_connect(void* param)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_proto_conn_param_t* pconn =  (otam_proto_conn_param_t*)param;
    pctx->state = OTAM_ST_CONNECTED;
    pctx->mtu_a = pconn->mtu -3;
    pctx->opcode = 0xff;

    if(pconn->run_mode == OTAC_RUNMODE_APP)
    {
        pctx->run_mode = OTAC_RUNMODE_APP;
        pctx->reset_mode = 0;
    }
    else if(pconn->run_mode == OTAC_RUNMODE_OTA)
    {
        pctx->run_mode = OTAC_RUNMODE_OTA;

        if(pctx->reset_mode == OTAC_RUNMODE_OTARES)
        {
            pctx->run_mode = OTAC_RUNMODE_OTARES;
        }

        pctx->reset_mode = 0;
    }
    else
    {
        pctx->run_mode = 0;
        pctx->reset_mode = 0;
    }
}


static int send_patition_info(void)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_fw_t* pfw = &(pctx->fw);
    otam_proto_meth_t* method = &(pctx->method);
    uint8_t data[20];

    if(pctx->state != OTAM_ST_WAIT_STARTED && pctx->state != OTAM_ST_DATA)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    //partition cmd: 02 ID FA FA FA FA RA RA RA RA SZ SZ SZ SZ CS CS
    //          ID: index
    //          FA: flash address
    //          RA: run address
    //          SZ: partition size
    //          CS: checksum
    uint32_t val;
    uint16_t offset = 0;
    uint32_t flash_addr = 0;
    uint32_t run_addr = pfw->part[pfw->part_current].run_addr;

    if(run_addr > OTAF_BASE_ADDR && run_addr < OTAF_END_ADDR)
    {
        flash_addr = run_addr;
    }
    else
    {
        //calculate store address in flash
        for(int i = 0; i<pfw->part_current; i++ )
        {
            if(pfw->part[i].run_addr > OTAF_BASE_ADDR && pfw->part[i].run_addr < OTAF_END_ADDR)
                continue;

            val = pfw->part[i].size +7;
            val = val - (val%4);
            flash_addr += val;
        }
    }

    data[offset ++] = OTA_CMD_PARTITION_INFO;
    data[offset ++] = pfw->part_current;
    val = flash_addr;
    data[offset ++] = (uint8_t)(val&0xff);
    data[offset ++] = (uint8_t)((val>>8)&0xff);
    data[offset ++] = (uint8_t)((val>>16)&0xff);
    data[offset ++] = (uint8_t)((val>>24)&0xff);
    val = run_addr;
    data[offset ++] = (uint8_t)(val&0xff);
    data[offset ++] = (uint8_t)((val>>8)&0xff);
    data[offset ++] = (uint8_t)((val>>16)&0xff);
    data[offset ++] = (uint8_t)((val>>24)&0xff);
    val = pfw->part[pfw->part_current].size;
    data[offset ++] = (uint8_t)(val&0xff);
    data[offset ++] = (uint8_t)((val>>8)&0xff);
    data[offset ++] = (uint8_t)((val>>16)&0xff);
    data[offset ++] = (uint8_t)((val>>24)&0xff);
    val = (uint32_t)(pfw->part[pfw->part_current].checksum);
    data[offset ++] = (uint8_t)(val&0xff);
    data[offset ++] = (uint8_t)((val>>8)&0xff);
    pctx->state = OTAM_ST_WAIT_PARTITION_INFO;
    pfw->cache_offset = 0;
    pfw->cache_size = 0;
    pfw->cache_retry = 0;
    pfw->offset = 0;
    return method->write_cmd(data, offset, 1000);
}


static int load_data_cache(void)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_fw_t* pfw = &(pctx->fw);
    ota_fw_part_t* ppart = &(pfw->part[pfw->part_current]);
    uint16_t mtu_a = pctx->mtu_a;
    uint32_t size = mtu_a * pctx->burst_size;

    if(pfw->offset + size > ppart->size)
    {
        size = ppart->size - pfw->offset;
    }

    //memset(pfw->cache, 0, (ATT_MTU_SIZE-3));
    pfw->cache = (uint8_t*)(ppart->flash_addr + pfw->offset);
    pfw->cache_size = size;
    pfw->cache_offset = 0;
    pfw->cache_retry = 0;
    pfw->offset += size;
    return PPlus_SUCCESS;
}


static int send_data(void)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_fw_t* pfw = &(pctx->fw);
    otam_proto_meth_t* method = &(pctx->method);

    if(!(method->write_data))
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    if(pctx->state != OTAM_ST_DATA)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    int ret = PPlus_SUCCESS;
    uint16_t size = 0;
    uint16_t mtu_a = pctx->mtu_a;

    while(pfw->cache_size - pfw->cache_offset)
    {
        size = mtu_a;

        if((pfw->cache_size - pfw->cache_offset) < mtu_a)
        {
            size = pfw->cache_size - pfw->cache_offset;
        }

        ret = method->write_data(pfw->cache + pfw->cache_offset, size);

        if(ret != PPlus_SUCCESS)
        {
            method->write_data_delay(1);
            return PPlus_SUCCESS;
        }

        pfw->cache_offset += size;
    }

    return PPlus_SUCCESS;
}


void print_version(uint8_t* vinfo)
{
}


static void handle_app_notify_event(void* param, uint8_t len)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;

    switch(pctx->opcode)
    {
    case OTAAPP_CMD_START_OTA:
        break;

    case OTAAPP_CMD_VER:
        print_version(param);
        break;

    default:
        break;
    }
}


static void handle_ota_notify_event(void* param, uint8_t len)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_fw_t* pfw = &(pctx->fw);
    uint8_t* pnotify = (uint8_t*)param;
    otam_proto_meth_t* method = &(pctx->method);
    int retval = pnotify[0];

    if(len == 1)    //fatal error
    {
        otam_proto_ctx_reset(OTAM_ST_CONNECTED);
        return;
    }

    OTA_LOG("OTA Notif %x, %x\n",pnotify[0],pnotify[1]);

    //  print_hex(pnotify, len);
    switch(pnotify[1]) //response type
    {
    case OTA_RSP_START_OTA:
        if(retval == PPlus_SUCCESS && pctx->state == OTAM_ST_WAIT_STARTED)
        {
            send_patition_info();
        }
        else
        {
            pctx->state = OTAM_ST_ERROR;
        }

        break;

    case OTA_RSP_PARAM:
    case OTA_RSP_BLOCK_INFO:
    case OTA_RSP_BLOCK_COMPLETE:
        pctx->state = OTAM_ST_ERROR;
        break;

    case OTA_RSP_OTA_COMPLETE:
    {
        uint8_t data[20];

        if(method->write_cmd)
        {
            data[0] = OTA_CMD_REBOOT;
            data[1] = 1;
            pctx->state = OTAM_ST_COMPLETE;
            OTA_LOG("OTA completed!!\n");
            method->write_cmd(data, 2, 1000);
            oneclk_evt_t ev;
            ev.ev = ONECLK_EVT_OTA_FINISHED;
            otam_oneclick_evt(&ev);
        }
    }
    break;

    case OTA_RSP_PARTITION_INFO:
        pctx->state = OTAM_ST_DATA;

        if(pfw->part_current)
        {
            OTA_LOG(" ");
        }

        load_data_cache();
        send_data();
        break;

    case OTA_RSP_PARTITION_COMPLETE:
        pfw->total_offset += pfw->cache_size;
        pfw->part_current ++;
        send_patition_info();
        break;

    case OTA_RSP_BLOCK_BURST:
        if(pctx->state != OTAM_ST_DATA)
        {
            pctx->state = OTAM_ST_ERROR;
            break;
        }

        if(retval == PPlus_SUCCESS)
        {
            load_data_cache();
            send_data();
        }
        else if(retval == PPlus_ERR_OTA_BAD_DATA)
        {
            //case block data is not completed, retry block data
            pfw->cache_retry++;
            pfw->cache_offset = 0;

            if(pfw->cache_retry > 3)
            {
                pctx->state = OTAM_ST_ERROR;
                break;
            }

            send_data();
        }
        else
        {
            pctx->state = OTAM_ST_ERROR;
        }

        break;

    case OTA_RSP_REBOOT:
        OTA_LOG("[OTA_RSP_REBOOT]GAPCentralRole_TerminateLink\n");
        GAPCentralRole_TerminateLink(0);

    case OTA_RSP_ERASE:
    case OTA_RSP_ERROR:
    default:
        pctx->state = OTAM_ST_ERROR;
        break;
    }
}


void otamProtocol_event(otap_evt_t* pev)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;

    switch(pev->ev)
    {
    case OTAP_EVT_DISCONNECTED:
        otam_proto_disconnect(pev->data);
        break;

    case OTAP_EVT_CONNECTED:
        otam_proto_connect(pev->data);
        break;

    case OTAP_EVT_NOTIFY:
        if(pctx->run_mode == OTAC_RUNMODE_APP)
        {
            handle_app_notify_event(pev->data, pev->len);
        }
        else
        {
            handle_ota_notify_event(pev->data, pev->len);
        }

        break;

    case OTAP_EVT_DATA_WR_DELAY:
        send_data();
        break;

    case OTAP_EVT_BLE_TIMEOUT:
    default:
        GAPCentralRole_TerminateLink(0);
        OTA_LOG("OTAP_EVT_BLE_TIMEOUT\n");
        break;
    }
}
#define OTAM_FW_DATA_ADDR               0x11080000
#define OTAM_FW_DATA_ADDR1              0x110C0000

#define SINGLE_FLASH_FW_PART_RADDR(n)   (OTAM_FW_DATA_ADDR + 0x08 + 0x08*(n))
#define SINGLE_FLASH_FW_PART_RADDR1(n)  (OTAM_FW_DATA_ADDR1 + 0x08 + 0x08*(n))

static uint16_t fw_crc16(uint32_t p_data_addr, uint32_t size)
{
    uint32_t m_seed = 0;
    uint16_t m_size = size>=256?256:size;
    uint16_t offset = 0;
    uint8_t  temp_buf[256] = {0};

    while(offset != size)
    {
        hal_flash_read(p_data_addr+offset, temp_buf, m_size);
        m_seed = crc16(m_seed, temp_buf, m_size);
        offset += m_size;
        m_size = (size-offset)>=256?256:(size-offset);
    };

    return m_seed;
}


/*
    word      | desc:
    0         | flag: "OTAF"
    1         | partition number
    i*2 + 2   | run address
    i*2 + 3   | size
    N*2 +2    | data area

*/

int load_fw(uint8_t fw_id)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_fw_t* pfw = &(pctx->fw);

    if(pfw->part_num != 0)
    {
        return PPlus_SUCCESS;
    }

    uint8_t pdata[8] = {0};
    uint32_t offset = 0;
    uint32_t part_info[2] = {0};
    uint32_t faddr = fw_id == 0 ? OTAM_FW_DATA_ADDR : OTAM_FW_DATA_ADDR1;
    OTA_LOG("load fw %x\n", faddr);
    hal_flash_read(OTAM_FW_DATA_ADDR, pdata, 8);
    memset((void*)pfw, 0,sizeof(otam_fw_t));

    if(!((char)(pdata[0]) == 'O' && (char)(pdata[1]) == 'T' &&(char)(pdata[2]) == 'A'&&(char)(pdata[3]) == 'F'))
    {
        return PPlus_ERR_INVALID_DATA;
    }

    pfw->part_num = pdata[4] + pdata[5] + pdata[6] + pdata[7];
    OTA_LOG("pfw->part_num:%d\n", pfw->part_num);
    offset = 2 * 4 + 2 * 4 * pfw->part_num;
    pfw->total_size = 0;

    for (uint8_t i = 0; i < pfw->part_num; i++)
    {
        if(fw_id == 0)
            hal_flash_read(SINGLE_FLASH_FW_PART_RADDR(i), (uint8_t*)part_info, 0x08);
        else
            hal_flash_read(SINGLE_FLASH_FW_PART_RADDR1(i), (uint8_t*)part_info, 0x08);

        pfw->part[i].run_addr   = part_info[0];
        pfw->part[i].size       = part_info[1];
        pfw->part[i].flash_addr = faddr + offset;
        // OTA_LOG("flash_addr:%x, run_addr:%x, size:%x,", pfw->part[i].flash_addr,
        //     pfw->part[i].run_addr, pfw->part[i].size);
        pfw->part[i].checksum = fw_crc16(pfw->part[i].flash_addr, pfw->part[i].size);
        offset += pfw->part[i].size;
        pfw->total_size += pfw->part[i].size;
        // OTA_LOG(" checksum:%x\n", pfw->part[i].checksum);
    }

    pfw->total_size += 0;
    osal_memcpy(&m_otam_fw, pfw, sizeof(otam_fw_t));
    return PPlus_SUCCESS;
}

int load_res(void)
{
    return PPlus_ERR_NOT_IMPLEMENTED;
}


int otamProtocol_start_ota(uint8_t fw_id)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_fw_t* pfw = &(pctx->fw);
    otam_proto_meth_t* method = &(pctx->method);
    uint8_t data[20];

    if(pctx->state < OTAM_ST_CONNECTED)
    {
        return PPlus_ERR_BLE_NOT_READY;
    }

    if(pctx->run_mode == OTAC_RUNMODE_OTA)
    {
        if(load_fw(fw_id) != PPlus_SUCCESS)
        {
            return PPlus_ERR_INVALID_DATA;
        }
    }
    else if(pctx->run_mode == OTAC_RUNMODE_OTARES)
    {
        if(load_res() != PPlus_SUCCESS)
        {
            return PPlus_ERR_INVALID_DATA;
        }
    }
    else
    {
        return PPlus_ERR_INVALID_STATE;
    }

    if(method->write_cmd)
    {
        pctx->state = OTAM_ST_WAIT_STARTED;
        pfw->part_current = 0;

        if(pctx->mtu_a == 20)
        {
            pctx->burst_size = OTA_BURST_SIZE_DEFAULT;
            data[0] = OTA_CMD_START_OTA;
            data[1] = pfw->part_num;
            data[2] = 0;
        }
        else
        {
            pctx->burst_size = 0xffff;
            data[0] = OTA_CMD_START_OTA;
            data[1] = pfw->part_num;
            data[2] = OTA_BURST_SIZE_HISPEED;
        }

        return method->write_cmd(data, 3, 1000);
    }

    return PPlus_ERR_NOT_REGISTED;
}


int otamProtocol_stop_ota(void)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_proto_meth_t* method = &(pctx->method);
    uint8_t data[20];

    if(method->write_cmd)
    {
        data[0] = OTA_CMD_START_OTA;
        data[1] = 0xff;
        data[2] = 0;
        pctx->state = OTAM_ST_CANCELING;
        return method->write_cmd(data, 3, 1000);
    }

    return PPlus_ERR_NOT_REGISTED;
}


int otamProtocol_app_start_ota(uint8_t mode)
{
    otam_proto_ctx_t* pctx = &s_otap_ctx_t;
    otam_proto_meth_t* method = &(pctx->method);
    uint8_t data[20];

    if(pctx->run_mode != OTAC_RUNMODE_APP)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    //if(mode > OTA_MODE_RESOURCE)
    //  return PPlus_ERR_INVALID_PARAM;

    if(method->write_cmd)
    {
        data[0] = OTAAPP_CMD_START_OTA;
        data[1] = mode;
        data[2] = 1;
        return method->write_cmd(data, 3, 1000);
    }

    return PPlus_ERR_NOT_REGISTED;
}


int otamProtocol_init(otam_proto_meth_t* method)
{
    memset(&s_otap_ctx_t, 0, sizeof(s_otap_ctx_t));
    s_otap_ctx_t.method = *method;
    return PPlus_SUCCESS;
}

