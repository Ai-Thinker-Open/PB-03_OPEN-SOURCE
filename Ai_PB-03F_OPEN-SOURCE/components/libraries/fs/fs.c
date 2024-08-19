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

/*******************************************************************************
    @file     fs.c
    @brief    Contains all functions support for spi driver
    @version  0.0
    @date     18. Oct. 2017
    @author



*******************************************************************************/
#include "rom_sym_def.h"
#include "OSAL.h"
#include "fs.h"
#include "flash.h"
#include "error.h"
#include "log.h"

#ifndef FS_CRC_EN
    #define FS_CRC_EN 0
#endif

#if(FS_CRC_EN==1)
    #include "crc16.h"
#endif

#if(FS_CRC_EN == 1)
    #define FS_CRC_FIELD 2
#else
    #define FS_CRC_FIELD 0
#endif

//#define FS_DBBUG
#ifdef FS_DBBUG
    #define FS_LOG  LOG
#else
    #define FS_LOG(...)
#endif

//#define FS_CACHE_DBBUG
#ifdef FS_CACHE_DBBUG
    #define FS_CACH_LOG(...)  {LOG("[FS_CACHE]");LOG(__VA_ARGS__);};
#else
    #define FS_CACH_LOG(...)
#endif

static uint8_t  fs_sector_num;
static uint32_t fs_offset_address;

#define FS_ITEM_LEN_16BYTE 0
#define FS_ITEM_LEN_32BYTE 1
#define FS_ITEM_LEN_64BYTE 2

#ifndef FS_SETTING
    #define FS_SETTING FS_ITEM_LEN_16BYTE
#endif

#if (FS_SETTING == FS_ITEM_LEN_16BYTE)
    #define FS_ITEM_LEN                                                             16
#elif (FS_SETTING == FS_ITEM_LEN_32BYTE)
    #define FS_ITEM_LEN                                                             32
#elif (FS_SETTING == FS_ITEM_LEN_64BYTE)
    #define FS_ITEM_LEN                                                             64
#else
    #error please check your config parameter
#endif

/*
    fs struct:
    sector0
    sector_head
    file_head(4byte)+file_data
    ...
    file_head(4byte)+file_data
*/
//please do not modify the following parameters
#define FS_ITEM_HEAD_LEN                          4
#define FS_ITEM_DATA_LEN                          (FS_ITEM_LEN - FS_ITEM_HEAD_LEN)
#define FS_SECTOR_ITEM_NUM                        (4096/FS_ITEM_LEN - 1)
#define FS_SECTOR_NUM_BUFFER_SIZE                 (312/4)
#define FS_ABSOLUTE_ADDR(offset)                  (fs.cfg.sector_addr + offset)

typedef enum
{
    ITEM_DEL    = 0x00,//zone is deleted
    ITEM_UNUSED = 0x03,//zone is free
    ITEM_USED   = 0x02,//zone is used
    ITEM_RESERVED = 0x01//to be extend
} item_pro;

typedef enum
{
    ITEM_SF = 0x03,//single frame file
    ITEM_MF_F = 0x01,//multiple frame file,first frame
    ITEM_MF_C   = 0x02,//multiple frame file,continue frame
    ITEM_MF_E = 0x00//multiple frame file,end frame
} item_frame;

typedef enum
{
    FLASH_UNCHECK = 0,//before analysis fs
    FLASH_NEW = 1,//new fs,its are 0xFF
    FLASH_ORIGINAL_ORDER = 2,//fs has data,its order is the original
    FLASH_NEW_ORDER = 3,//fs has data,its order is not the original
    FLASH_CONTEXT_ERROR = 4,//fs has data,but data is broken
} FS_FLASH_TYPE;

/*
    file head struct:
    len(12bit)+frame(2bit)+pro(2bit)+id(16bit)
*/
typedef union
{
    struct
    {
        uint32_t id:16;//file id
        uint32_t pro:2;//file property
        uint32_t frame:2;//file frame
        uint32_t len:12;//file length
    } b;
    uint32_t reg;
} fs_item_t;

/*
    sector head struct:
    sector_addr(one word)+(ff+index+item_len+sector_num)(one word)+(0xffffffff)(one word)~(0xffffffff)(one word)
*/
typedef struct
{
    uint32_t sector_addr;//fs start address
    uint8_t  sector_num;//fs sector number
    uint8_t  item_len;//item length
    uint8_t  index;//sector index
    uint8_t  crc_en;//0xff: None; 0x01: CRC en
    uint8_t  reserved[FS_ITEM_LEN-8];
} fs_cfg_t;

typedef struct
{
    fs_cfg_t    cfg;
    uint8_t current_sector;//free sector index
    uint8_t exchange_sector;//exchange sector,only use it when garbage collect
    uint16_t offset;//free position in free sector index
} fs_t;

#if(FS_CACHE_NUM_MAX>0)

typedef struct
{
    uint16_t id[FS_CACHE_NUM_MAX];//fs cache id
    fsCachAddr_t addr[FS_CACHE_NUM_MAX];//fs cache addr
} fs_cache_t;
static fs_cache_t fs_cache;
#endif

static fs_t fs;
static bool fs_init_flag = false;
static bool fs_psr_protection = true;


typedef enum
{
    SEARCH_FREE_ITEM = 0,
    SEARCH_APPOINTED_ITEM = 1,
    SEARCH_DELETED_ITEMS = 2,
} search_type;

extern uint32_t __psr(void);//check if in int process

void fs_set_psr_protection(bool enable)
{
    fs_psr_protection=enable;
}

static uint32_t fs_check_psr(void)
{
    if(fs_psr_protection)
        return(__psr());

    return 0;
}

static void fs_erase_ucds_all_sector(void)
{
    int i;

    for(i=0; i<fs_sector_num; i++)
        hal_flash_erase_sector(fs_offset_address + (i*4096));
}

static void fs_erase_ucds_one_sector(uint32_t addr_erase)
{
    hal_flash_erase_sector(fs_offset_address + addr_erase);
}

static int fs_spif_write(uint32_t addr,uint8_t* value,uint16_t len)
{
    uint8_t retval;

    if(fs_check_psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    if((addr < fs_offset_address) || (addr >= (fs_offset_address+fs_sector_num*4096)) || ((addr&0x03)>0))
    {
        return PPlus_ERR_FS_PARAMETER;
    }

    retval = hal_flash_write_by_dma(addr,value,(uint32_t)len);
    return retval;
}

static uint32_t fs_spif_read(uint32_t addr,uint8_t* buf,uint32_t len)
{
    if ((addr < fs_offset_address) || (addr >= (fs_offset_address + fs_sector_num*4096)) || (buf == NULL) || (len == 0))
    {
        return PPlus_ERR_FS_PARAMETER;
    }

    return( hal_flash_read(addr,buf,len) );
}

static void check_addr(uint32_t* addr)
{
    if((*addr % 4096) == 0)
    {
        *addr += sizeof(fs_cfg_t);

        if(*addr >= 4096 *fs_sector_num)
            *addr -= 4096 *fs_sector_num;
    }
}

#if(FS_CACHE_NUM_MAX>0)
static void fs_addr_cache_init(void)
{
    for(int i=0; i<FS_CACHE_NUM_MAX; i++)
    {
        fs_cache.addr[i]=FS_ADDR_NULL;
        fs_cache.id[i]=FS_ID_NULL;
    }
}

static int fs_addr_cache_search(uint32_t id,uint32_t* addr)
{
    if(id == FS_ID_NULL)
        return PPlus_ERR_FSCACHE_INVALID_ID;

    for(int i=0; i<FS_CACHE_NUM_MAX; i++)
    {
        if(id==fs_cache.id[i] && fs_cache.addr[i]!=FS_ADDR_NULL)
        {
            *addr =(uint32_t)fs_cache.addr[i];
            FS_CACH_LOG("SCH OK ID %04x %08x\r\n",fs_cache.id[i],fs_cache.addr[i]);
            return PPlus_SUCCESS;
        }
    }

    FS_CACH_LOG("SCH FAIL ID %04x\r\n",id);
    return PPlus_ERR_FSCACHE_ID_NOT_FOUND;
}
static int fs_addr_cache_sync(uint32_t id,uint32_t addr)
{
    if(id == FS_ID_NULL)
        return PPlus_ERR_FSCACHE_INVALID_ID;

    //search for exsited id
    for(int i=0; i<FS_CACHE_NUM_MAX; i++)
    {
        if(id==fs_cache.id[i])
        {
            fs_cache.addr[i]=(fsCachAddr_t)addr;

            //del existed id
            if(addr == FS_ADDR_NULL)
            {
                fs_cache.id[i]=FS_ID_NULL;
            }

            FS_CACH_LOG("SYNC OLD ID %04x %08x\r\n",fs_cache.id[i],fs_cache.addr[i]);
            return PPlus_SUCCESS;
        }
    }

    //add new id
    for(int i=0; i<FS_CACHE_NUM_MAX; i++)
    {
        if(FS_ID_NULL==fs_cache.id[i])
        {
            fs_cache.id[i]=id;
            fs_cache.addr[i]=(fsCachAddr_t)addr;
            FS_CACH_LOG("SYNC NEW ID %04x %08x\r\n",fs_cache.id[i],fs_cache.addr[i]);
            return PPlus_SUCCESS;
        }
    }

    FS_CACH_LOG("SYNC FAIL ID %04x\r\n",id);
    return PPlus_ERR_FSCACHE_SYNC_ID_FAIL;
}
void dbg_show_fs_cache(void)
{
    LOG("-----dbg_show_fs_cache------\n");

    for(int i=0; i<FS_CACHE_NUM_MAX; i++)
    {
        LOG("[%03d]%04x:%08x\n",i,fs_cache.id[i],fs_cache.addr[i]);
    }
}
#endif
static int fs_search_items(search_type type,uint32_t* para1,uint32_t* para2)
{
    uint8_t m,n;
    uint16_t j,g_offset = 1;
    uint16_t file_len = 0;
    uint32_t sector_addr,ab_addr;
    fs_item_t i1;
    bool from_last_sector = false;

    for(m = 1; m < fs_sector_num; m++)
    {
        n = (m + fs.exchange_sector) % fs.cfg.sector_num;

        if(g_offset > FS_SECTOR_ITEM_NUM)
        {
            g_offset -= FS_SECTOR_ITEM_NUM;

            if(g_offset >= FS_SECTOR_ITEM_NUM)
            {
                continue;
            }
        }

        if(SEARCH_FREE_ITEM == type)
            fs.current_sector = (m + fs.exchange_sector) % fs.cfg.sector_num;

        sector_addr = n * 4096;

        for(j = 1; j <= FS_SECTOR_ITEM_NUM; j++)
        {
            if(from_last_sector == true)
            {
                from_last_sector = false;
                j += g_offset;
            }
            else
            {
                if(g_offset > 1)
                {
                    if((j - 2 + g_offset) < FS_SECTOR_ITEM_NUM)
                    {
                        j = j - 1 + g_offset;
                    }
                    else
                    {
                        g_offset -= (FS_SECTOR_ITEM_NUM + 2 - j);
                        from_last_sector = true;
                        break;
                    }
                }
            }

            ab_addr = sector_addr + (j * FS_ITEM_LEN);
            fs_spif_read(FS_ABSOLUTE_ADDR(ab_addr),(uint8_t*)&i1,FS_ITEM_HEAD_LEN);

            switch(type)
            {
            case SEARCH_FREE_ITEM:
            {
                switch(i1.b.pro)
                {
                case ITEM_DEL:
                case ITEM_USED:
                {
                    file_len = i1.b.len + FS_CRC_FIELD;

                    if(i1.b.frame == ITEM_MF_F)
                        g_offset = (file_len/FS_ITEM_DATA_LEN) + ((file_len % FS_ITEM_DATA_LEN)?1:0);
                    else
                        g_offset = 1;
                }
                break;

                case ITEM_UNUSED:
                    *para1 = ab_addr%4096;
                    return PPlus_SUCCESS;

                default:
                    break;
                }
            }
            break;

            case SEARCH_APPOINTED_ITEM:
            {
                switch(i1.b.pro)
                {
                case ITEM_DEL:
                case ITEM_USED:
                    if((ITEM_USED == i1.b.pro) && (i1.b.id == (uint16)(*para1)))
                    {
                        *para2 = ab_addr;
                        return PPlus_SUCCESS;
                    }
                    else
                    {
                        file_len = i1.b.len + FS_CRC_FIELD;

                        if(i1.b.frame == ITEM_MF_F)
                            g_offset = (file_len / FS_ITEM_DATA_LEN) + ((file_len % FS_ITEM_DATA_LEN)?1:0);
                        else
                            g_offset = 1;
                    }

                    break;

                case ITEM_UNUSED:
                    return PPlus_ERR_FS_NOT_FIND_ID;

                default:
                    break;
                }
            }
            break;

            case SEARCH_DELETED_ITEMS:
            {
                switch(i1.b.pro)
                {
                case ITEM_DEL:
                {
                    if(i1.b.frame == ITEM_MF_F)
                    {
                        file_len = i1.b.len + FS_CRC_FIELD;
                        g_offset = (file_len/FS_ITEM_DATA_LEN) + ((file_len % FS_ITEM_DATA_LEN)?1:0);
                        *para1 += g_offset*FS_ITEM_DATA_LEN;
                    }
                    else
                    {
                        g_offset = 1;
                        *para1 += FS_ITEM_DATA_LEN;
                    }

                    *para2 += 1;
                }
                break;

                case ITEM_USED:
                {
                    if(i1.b.frame == ITEM_MF_F)
                    {
                        file_len = i1.b.len + FS_CRC_FIELD;
                        g_offset = (file_len/FS_ITEM_DATA_LEN) + ((file_len % FS_ITEM_DATA_LEN)?1:0);
                    }
                    else
                    {
                        g_offset = 1;
                    }
                }
                break;

                case ITEM_UNUSED:
                    return PPlus_SUCCESS;

                default:
                    break;
                }
            }
            break;

            default:
                return PPlus_ERR_INVALID_PARAM;
            }
        }
    }

    switch(type)
    {
    case SEARCH_FREE_ITEM:
        return PPlus_ERR_FS_FULL;

    case SEARCH_APPOINTED_ITEM:
        return PPlus_ERR_FS_NOT_FIND_ID;

    default:
        return PPlus_SUCCESS;
    }
}

static int fs_get_free_item(void)
{
    int ret;
    uint32_t posiztion = 0;

    if(fs_init_flag == false)
        return  PPlus_ERR_FS_UNINITIALIZED;

    ret = fs_search_items(SEARCH_FREE_ITEM,&posiztion,0);

    if(PPlus_SUCCESS == ret)
    {
        fs.offset = posiztion;
    }
    else if(PPlus_ERR_FS_FULL == ret)
    {
        fs.offset = 4096;
    }

    return ret;
}

static int fs_init(void)
{
    uint8_t i = 0,sector_order[FS_SECTOR_NUM_BUFFER_SIZE],ret = PPlus_ERR_FS_UNINITIALIZED;
    FS_FLASH_TYPE flash = FLASH_UNCHECK;
    fs_cfg_t flash_rd_cfg;
    fs.cfg.sector_addr = fs_offset_address;
    fs.cfg.sector_num = fs_sector_num;;
    fs.cfg.index = 0xff;
    fs.cfg.item_len = FS_ITEM_LEN;
    fs.cfg.crc_en = (FS_CRC_EN==1)? 1: 0xff;
    osal_memset((fs.cfg.reserved),0xff,(FS_ITEM_LEN-7)*sizeof(uint8_t));
    osal_memset((sector_order),0x00,FS_SECTOR_NUM_BUFFER_SIZE);
    FS_LOG("fs_init:\n");

    for(i = 0; i < fs.cfg.sector_num; i++)
    {
        fs_spif_read(FS_ABSOLUTE_ADDR(4096*i),(uint8_t*)(&flash_rd_cfg),sizeof(fs_cfg_t));
        FS_LOG("flash_rd_cfg.sector_addr:%x\n",flash_rd_cfg.sector_addr);
        FS_LOG("flash_rd_cfg.sector_num:%x\n",flash_rd_cfg.sector_num);
        FS_LOG("flash_rd_cfg.item_len:%x\n",flash_rd_cfg.item_len);
        FS_LOG("flash_rd_cfg.index:%x\n",flash_rd_cfg.index);

        if((flash_rd_cfg.sector_addr == fs.cfg.sector_addr) &&
                (flash_rd_cfg.sector_num == fs.cfg.sector_num) &&
                (flash_rd_cfg.item_len == fs.cfg.item_len) &&
                (flash_rd_cfg.crc_en == fs.cfg.crc_en) )
        {
            if(flash_rd_cfg.index < (fs_sector_num - 1))
            {
                if(i == flash_rd_cfg.index)
                {
                    flash = FLASH_ORIGINAL_ORDER;
                    FS_LOG("FLASH_ORIGINAL_ORDER\n");
                }
                else
                {
                    flash = FLASH_NEW_ORDER;
                    FS_LOG("FLASH_NEW_ORDER\n");
                }

                sector_order[i] = flash_rd_cfg.index;
                fs.cfg.index = flash_rd_cfg.index;
            }
            else
            {
                flash = FLASH_CONTEXT_ERROR;
                FS_LOG("FLASH_CONTEXT_ERROR1\n");
                break;
            }
        }
        else if((flash_rd_cfg.sector_addr == 0xffffffff) &&
                (flash_rd_cfg.sector_num == 0xff) &&
                (flash_rd_cfg.item_len == 0xff))
        {
            sector_order[i] = 0xff;
        }
        else
        {
            flash = FLASH_CONTEXT_ERROR;
            FS_LOG("FLASH_CONTEXT_ERROR2\n");
            break;
        }
    }

    if(flash == FLASH_CONTEXT_ERROR)
    {
        return PPlus_ERR_FS_CONTEXT;
    }

    if(fs.cfg.index == 0xff)
        flash = FLASH_NEW;

    if(flash == FLASH_NEW)
    {
        for(i = 0; i < (fs.cfg.sector_num - 1); i++)
        {
            fs.cfg.index = i;

            if(PPlus_SUCCESS != fs_spif_write((FS_ABSOLUTE_ADDR(4096*i)),(uint8_t*)(&(fs.cfg)),sizeof(fs_cfg_t)))
            {
                FS_LOG("PPlus_ERR_FS_WRITE_FAILED\n");
                return PPlus_ERR_FS_WRITE_FAILED;
            }
        }

        fs.current_sector = 0;
        fs.exchange_sector = fs.cfg.sector_num - 1;
        fs.offset = sizeof(fs_cfg_t);
        fs_init_flag = TRUE;
    }
    else
    {
        for(i = 0; i < fs.cfg.sector_num; i++)
        {
            if(sector_order[i] == 0)
                break;
        }

        if(i < fs.cfg.sector_num)
        {
            fs.exchange_sector = (i + fs.cfg.sector_num - 1) % fs.cfg.sector_num;
            fs_init_flag = TRUE;
            ret = fs_get_free_item();

            if((ret != PPlus_ERR_FS_FULL) && (ret != PPlus_SUCCESS))
            {
                FS_LOG("PPlus_ERR_FS_RESERVED_ERROR\n");
                return PPlus_ERR_FS_RESERVED_ERROR;
            }
        }
    }

    #if(FS_CACHE_NUM_MAX>0)
    fs_addr_cache_init();
    #endif
    FS_LOG("PPlus_SUCCESS\n");
    return PPlus_SUCCESS;
}

int hal_fs_get_free_size(uint32_t* free_size)
{
    uint32_t size = 0;
    *free_size = 0;

    if(fs_init_flag == false)
    {
        //LOG("fs_init_flag = false,free\n");
        return PPlus_ERR_FS_UNINITIALIZED;
    }

    if(fs.offset < 4096)
    {
        size = ((fs.exchange_sector + fs.cfg.sector_num - fs.current_sector - 1)%fs.cfg.sector_num)*(4096-sizeof(fs_cfg_t));
        size += (4096 - fs.offset);
        size = size*FS_ITEM_DATA_LEN/FS_ITEM_LEN;
    }

    *free_size = size;
    return PPlus_SUCCESS;
}

int hal_fs_get_garbage_size(uint32_t* garbage_file_num)
{
    uint32_t garbage_size = 0,garbage_count = 0;
    int ret;

    if(fs_init_flag == false)
        return  PPlus_ERR_FS_UNINITIALIZED;

    ret = fs_search_items(SEARCH_DELETED_ITEMS,&garbage_size,&garbage_count);

    if(PPlus_SUCCESS == ret)
    {
        if(NULL != garbage_file_num)
            *garbage_file_num = garbage_count;

        return garbage_size;
    }

    return PPlus_ERR_FATAL;
}

int hal_fs_item_find_id(uint16_t id,uint32_t* id_addr)
{
    int ret;
    uint32_t file_id = 0;

    if(fs_init_flag == false)
        return  PPlus_ERR_FS_UNINITIALIZED;

    file_id = id & 0xffff;
    #if(FS_CACHE_NUM_MAX>0)

    if(PPlus_SUCCESS==fs_addr_cache_search(file_id,id_addr))
    {
        return PPlus_SUCCESS;
    }

    #endif
    ret = fs_search_items(SEARCH_APPOINTED_ITEM,&file_id,id_addr);
    #if(FS_CACHE_NUM_MAX>0)

    if(ret==PPlus_SUCCESS)
    {
        uint32_t addr = *id_addr;
        fs_addr_cache_sync(file_id, addr);
    }

    #endif
    return ret;
}


int hal_fs_item_write(uint16_t id,uint8_t* buf,uint16_t len)
{
    uint8_t frame_len,wr_buf[FS_ITEM_LEN];
    uint16_t i,write_len;
    uint32_t addr;
    fs_item_t i1;
    uint32_t free_size;
    #if(FS_CRC_EN == 1)
    uint8_t crc_buf[2*FS_ITEM_DATA_LEN];
    uint16_t crc = crc16(0, buf, len);
    #endif

    if(fs_check_psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    if(fs_init_flag == false)
    {
        //LOG("fs_init_flag = false,write\n");
        return PPlus_ERR_FS_UNINITIALIZED;
    }

    if((buf == NULL) || (len == 0)||(len > 4095))
        return PPlus_ERR_FS_PARAMETER;

    hal_fs_get_free_size(&free_size);

    if(len + FS_CRC_FIELD > free_size)
        return PPlus_ERR_FS_NOT_ENOUGH_SIZE;

    //if(hal_fs_item_find_id(id,&addr) == PPlus_SUCCESS)
    //  return PPlus_ERR_FS_EXIST_SAME_ID;
    if(hal_fs_item_find_id(id,&addr) == PPlus_SUCCESS)
    {
        if(PPlus_SUCCESS != hal_fs_item_del(id))
            return PPlus_ERR_FATAL;
    }

    write_len = len + FS_CRC_FIELD;
    i1.b.len = len;
    i1.b.id = id;
    i1.b.pro = ITEM_USED;

    if(len <= FS_ITEM_DATA_LEN)
        i1.b.frame = ITEM_SF;

    i = 0;
    #if(FLASH_PROTECT_FEATURE == 1)
    hal_flash_disable_lock(FS_ITEM_WRITE);
    #endif

    while(write_len > 0)
    {
        if(write_len > FS_ITEM_DATA_LEN)
        {
            if(write_len == write_len)
                i1.b.frame = ITEM_MF_F;
            else
                i1.b.frame = ITEM_MF_C;

            frame_len = FS_ITEM_DATA_LEN;
            write_len -= FS_ITEM_DATA_LEN;
        }
        else
        {
            if((i1.b.frame == ITEM_MF_C) || (i1.b.frame == ITEM_MF_F))
                i1.b.frame = ITEM_MF_E;

            frame_len = write_len;
            write_len = 0;
        }

        addr = FS_ABSOLUTE_ADDR((fs.current_sector * 4096) + fs.offset);
        osal_memcpy(wr_buf,(uint8_t*)(&i1.reg),FS_ITEM_HEAD_LEN);
        #if(FS_CRC_EN == 1)

        if(len <= frame_len)
        {
            if( len > 0)
            {
                osal_memcpy(crc_buf,(buf + i),frame_len);
                crc_buf[len] = (uint8_t)(crc & 0xff);
                crc_buf[len + 1] = (uint8_t)((crc>>8) & 0xff);
                i = 0;
            }

            len = 0;
            osal_memcpy((wr_buf + FS_ITEM_HEAD_LEN),(crc_buf + i),frame_len);
        }
        else
        {
            osal_memcpy((wr_buf + FS_ITEM_HEAD_LEN),(buf + i),frame_len);
            len -= frame_len;
        }

        #else
        osal_memcpy((wr_buf + FS_ITEM_HEAD_LEN),(buf + i),frame_len);
        #endif

        if(PPlus_SUCCESS != fs_spif_write(addr,wr_buf,(frame_len+FS_ITEM_HEAD_LEN)))
        {
            #if(FLASH_PROTECT_FEATURE == 1)
            hal_flash_enable_lock(FS_ITEM_WRITE);
            #endif
            return PPlus_ERR_FS_WRITE_FAILED;
        }

        #if(FS_CACHE_NUM_MAX>0)

        //update fs_addr_cache
        if(i==0 && len > 0)
            fs_addr_cache_sync(id,(fs.current_sector * 4096) + fs.offset);

        #endif
        i += frame_len;
        fs.offset += FS_ITEM_LEN;

        if(fs.offset == 4096)
        {
            if(((fs.current_sector + 1) % fs.cfg.sector_num) != fs.exchange_sector)
            {
                fs.offset = sizeof(fs_cfg_t);
                fs.current_sector = (fs.current_sector + 1) % fs.cfg.sector_num;
            }
        }
    }

    #if(FLASH_PROTECT_FEATURE == 1)
    hal_flash_enable_lock(FS_ITEM_WRITE);
    #endif
    return PPlus_SUCCESS;
}

int hal_fs_item_read(uint16_t id,uint8_t* buf,uint16_t buf_len,uint16_t* len)
{
    uint8_t rd_len;
    uint16_t i = 0,temp_len;
    uint32_t addr;
    fs_item_t i1;

    if(fs_check_psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    if(fs_init_flag == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    if((buf == NULL) || (buf_len == 0))
        return PPlus_ERR_FS_PARAMETER;

    if(hal_fs_item_find_id(id,&addr) == PPlus_SUCCESS)
    {
        fs_spif_read(FS_ABSOLUTE_ADDR(addr),(uint8_t*)&i1,FS_ITEM_HEAD_LEN);

        if(len != NULL)
        {
            *len = i1.b.len;
        }

        temp_len = i1.b.len;

        if(buf_len >= i1.b.len)
        {
            while(temp_len > 0)
            {
                rd_len = (temp_len >= FS_ITEM_DATA_LEN) ? FS_ITEM_DATA_LEN : temp_len;
                fs_spif_read(FS_ABSOLUTE_ADDR(addr + FS_ITEM_HEAD_LEN),(buf + i),rd_len);
                temp_len -= rd_len;
                #if(FS_CRC_EN == 1)

                if(temp_len == 0)
                {
                    uint8_t rd_offset = rd_len;
                    uint8_t fs_crc[2];
                    //figure buf crc
                    uint16_t buf_crc = crc16(0, buf, i1.b.len);

                    //get fs crc
                    if(rd_offset == FS_ITEM_DATA_LEN)
                    {
                        addr += FS_ITEM_LEN;
                        check_addr(&addr);
                        rd_offset = 0;
                    }

                    fs_spif_read(FS_ABSOLUTE_ADDR(addr + FS_ITEM_HEAD_LEN + rd_offset), fs_crc, 1);
                    rd_offset ++;

                    if(rd_offset == FS_ITEM_DATA_LEN)
                    {
                        addr += FS_ITEM_LEN;
                        check_addr(&addr);
                        rd_offset = 0;
                    }

                    fs_spif_read(FS_ABSOLUTE_ADDR(addr + FS_ITEM_HEAD_LEN + rd_offset), fs_crc+1, 1);

                    if((fs_crc[0] != (buf_crc & 0xff)) || (fs_crc[1] != ((buf_crc>>8) & 0xff)))
                    {
                        return PPlus_ERR_FS_CRC;
                    }
                }

                #endif /*FS_CRC_EN*/
                addr += FS_ITEM_LEN;
                i += rd_len;
                check_addr(&addr);
            }

            return PPlus_SUCCESS;
        }

        return PPlus_ERR_FS_BUFFER_TOO_SMALL;
    }

    return PPlus_ERR_FS_NOT_FIND_ID;
}

int hal_fs_item_set(uint16_t id, uint8_t pro, uint16_t new_id)
{
    uint16_t i = 0,count = 1;
    uint32_t addr = 0;
    uint16_t file_len = 0;
    fs_item_t i1;

    if(fs_check_psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    if(fs_init_flag == FALSE)
        return PPlus_ERR_FS_UNINITIALIZED;

    if(hal_fs_item_find_id(id,&addr) == PPlus_SUCCESS)
    {
        fs_spif_read(FS_ABSOLUTE_ADDR(addr),(uint8_t*)&i1,FS_ITEM_HEAD_LEN);
        file_len = i1.b.len + FS_CRC_FIELD;
        //LOG("file_len %d\n", file_len);
        count = file_len/FS_ITEM_DATA_LEN + ((file_len % FS_ITEM_DATA_LEN)?1:0);
        #if(FLASH_PROTECT_FEATURE == 1)
        hal_flash_disable_lock(FS_ITEM_DEL);
        #endif

        for(i = 0; i < count; i++)
        {
            fs_spif_read(FS_ABSOLUTE_ADDR(addr),(uint8_t*)&i1,FS_ITEM_HEAD_LEN);
            i1.b.pro &= pro;
            i1.b.id  &= new_id;

            if(PPlus_SUCCESS != fs_spif_write(FS_ABSOLUTE_ADDR(addr),(uint8_t*)(&i1.reg),FS_ITEM_HEAD_LEN))
            {
                #if (FLASH_PROTECT_FEATURE == 1)
                hal_flash_enable_lock(FS_ITEM_DEL);
                #endif
                return PPlus_ERR_FS_WRITE_FAILED;
            }

            addr += FS_ITEM_LEN;
            check_addr(&addr);
        }

        #if (FLASH_PROTECT_FEATURE == 1)
        hal_flash_enable_lock(FS_ITEM_DEL);
        #endif
        return PPlus_SUCCESS;
    }
    else
    {
        return PPlus_ERR_FS_NOT_FIND_ID;
    }
}

//debug interface
int hal_fs_item_assert(uint16_t id)
{
    int ret;
    uint32_t addr = 0;

    if(fs_check_psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    if(fs_init_flag == FALSE)
        return PPlus_ERR_FS_UNINITIALIZED;

    if(hal_fs_item_find_id(id,&addr) == PPlus_SUCCESS)
    {
        #if(FLASH_PROTECT_FEATURE == 1)
        hal_flash_disable_lock(FS_ITEM_DEL);
        #endif
        //just do in 1st item
        {
            uint32_t data[3] = {0};
            ret = fs_spif_write(FS_ABSOLUTE_ADDR(addr + FS_ITEM_HEAD_LEN),(uint8_t*)(data),FS_ITEM_DATA_LEN);
        }
        #if (FLASH_PROTECT_FEATURE == 1)
        hal_flash_enable_lock(FS_ITEM_DEL);
        #endif
        return ret;
    }
    else
    {
        return PPlus_ERR_FS_NOT_FIND_ID;
    }
}


int hal_fs_item_id_fading(uint16_t id, uint16_t new_id)
{
    int ret = hal_fs_item_set(id, 0xff, new_id);
    #if(FS_CACHE_NUM_MAX>0)
    fs_addr_cache_sync(id,FS_ADDR_NULL);
    #endif
    return ret;
}

int hal_fs_item_del(uint16_t id)
{
    int ret = hal_fs_item_set(id, ITEM_DEL, 0xffff);
    #if(FS_CACHE_NUM_MAX>0)

    if(ret == PPlus_SUCCESS)
        fs_addr_cache_sync(id,FS_ADDR_NULL);

    #endif
    return ret;
}


int hal_fs_garbage_collect(void)
{
    uint8_t  i,j,buf[FS_ITEM_DATA_LEN];
    uint8_t from_sector_index = 0,to_sector_index = 0;
    uint32_t addr_rd=0,addr_wr=0,addr_erase;
    fs_item_t i1;

    if(fs_check_psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    if(fs_init_flag == FALSE)
        return PPlus_ERR_FS_UNINITIALIZED;

    to_sector_index = fs.exchange_sector;
    from_sector_index = (fs.exchange_sector + 1) % fs.cfg.sector_num;
    addr_wr = 4096*to_sector_index;
    #if(FLASH_PROTECT_FEATURE == 1)
    hal_flash_disable_lock(FS_GARBAGE_COLLECT);
    #endif

    for(i = 0; i < (fs.cfg.sector_num - 1); i++)
    {
        addr_rd = 4096*((from_sector_index + i) % fs.cfg.sector_num);
        addr_erase = addr_rd;
        fs.cfg.index = i;

        if(PPlus_SUCCESS != fs_spif_write(FS_ABSOLUTE_ADDR((4096*((to_sector_index + i) % fs.cfg.sector_num))),(uint8_t*)(&(fs.cfg)),sizeof(fs_cfg_t)))
        {
            #if (FLASH_PROTECT_FEATURE == 1)
            hal_flash_enable_lock(FS_GARBAGE_COLLECT);
            #endif
            return PPlus_ERR_FS_WRITE_FAILED;
        }

        if(i == 0)
            addr_wr += sizeof(fs_cfg_t);

        addr_rd += sizeof(fs_cfg_t);

        for(j = 0; j < FS_SECTOR_ITEM_NUM; j++)
        {
            fs_spif_read(FS_ABSOLUTE_ADDR(addr_rd),(uint8_t*)&i1,FS_ITEM_HEAD_LEN);

            if(i1.b.pro == ITEM_USED)
            {
                fs_spif_read(FS_ABSOLUTE_ADDR(addr_rd + FS_ITEM_HEAD_LEN),buf,FS_ITEM_DATA_LEN);

                if(PPlus_SUCCESS != fs_spif_write(FS_ABSOLUTE_ADDR(addr_wr),(uint8_t*)(&i1.reg),FS_ITEM_HEAD_LEN))
                {
                    #if (FLASH_PROTECT_FEATURE == 1)
                    hal_flash_enable_lock(FS_GARBAGE_COLLECT);
                    #endif
                    return PPlus_ERR_FS_WRITE_FAILED;
                }

                if(PPlus_SUCCESS != fs_spif_write(FS_ABSOLUTE_ADDR(addr_wr + FS_ITEM_HEAD_LEN),buf,FS_ITEM_DATA_LEN))
                {
                    #if (FLASH_PROTECT_FEATURE == 1)
                    hal_flash_enable_lock(FS_GARBAGE_COLLECT);
                    #endif
                    return PPlus_ERR_FS_WRITE_FAILED;
                }

                addr_wr += FS_ITEM_LEN;
                check_addr(&addr_wr);
            }
            else if(i1.b.pro == ITEM_UNUSED)
            {
                break;
            }

            addr_rd += FS_ITEM_LEN;
        }

        fs_erase_ucds_one_sector(addr_erase);
    }

    int ret = fs_init();
    #if(FLASH_PROTECT_FEATURE == 1)
    hal_flash_enable_lock(FS_GARBAGE_COLLECT);
    #endif
    return (ret);
}

int hal_fs_format(uint32_t fs_start_address,uint8_t sector_num)
{
    if(fs_check_psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    fs_init_flag = FALSE;

    if((fs_start_address % 0x1000) || (sector_num < 3))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    fs_sector_num = sector_num;
    fs_offset_address = fs_start_address;
    #if(FLASH_PROTECT_FEATURE == 1)
    hal_flash_disable_lock(FS_FORMAT);
    #endif
    fs_erase_ucds_all_sector();
    #if(FLASH_PROTECT_FEATURE == 1)
    hal_flash_enable_lock(FS_FORMAT);
    #endif
    return hal_fs_init(fs_start_address,sector_num);
}

int hal_fs_init(uint32_t fs_start_address,uint8_t sector_num)
{
    #if(FLASH_PROTECT_FEATURE == 1)
    hal_flash_disable_lock(FS_INIT);
    #endif

    if(fs_init_flag == TRUE)
    {
        return PPlus_ERR_FS_UNINITIALIZED;
    }

    if((fs_start_address % 0x1000) || (sector_num < 2))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    fs_sector_num = sector_num;
    fs_offset_address = fs_start_address;
    int ret = fs_init();
    #if(FLASH_PROTECT_FEATURE == 1)
    hal_flash_enable_lock(FS_INIT);
    #endif
    return (ret);
}


bool hal_fs_initialized(void)
{
    return fs_init_flag;
}
