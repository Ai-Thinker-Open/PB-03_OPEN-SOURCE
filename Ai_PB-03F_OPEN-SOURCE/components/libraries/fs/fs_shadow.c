
#include "rom_sym_def.h"
#include "OSAL.h"
#include "fs.h"
#include "error.h"

#include "log.h"


#define FSS_BIT 14



int fss_assert(uint16_t id) //for debug, trigger crc error
{
    uint16_t id_s = id | BIT(FSS_BIT);
    return hal_fs_item_assert(id_s);
}


/*shadow OP for fs, if once read failed, try to read old copy*/

int fss_read(uint16_t id, uint16_t len, void* pbuf)
{
    int ret;

    if(id & BIT(FSS_BIT))
        return PPlus_ERR_FS_PARAMETER;

    ret = hal_fs_item_read(id | BIT(FSS_BIT), pbuf, len, NULL);

    if(ret == PPlus_ERR_FS_CRC || ret == PPlus_ERR_FS_NOT_FIND_ID)
    {
        hal_fs_item_del(id | BIT(FSS_BIT));
        return hal_fs_item_read(id, pbuf, len, NULL);
    }

    return ret;
}



int fss_write(uint16_t id, uint16_t len, uint8_t* pbuf)
{
    int ret = PPlus_SUCCESS;
    uint32_t tmp_addr;
    /*bit 15 used as shadow file id*/
    uint16_t id_s = id | BIT(FSS_BIT);

    if(id & BIT(FSS_BIT))
        return PPlus_ERR_FS_PARAMETER;

    /*  code for auto GC
        if(hal_fs_get_free_size() < len+32)
        {
        if(hal_fs_get_garbage_size(NULL) > len+32)
        {
            hal_fs_garbage_collect();
        }
        else
        {
            return NV_OPER_FAILED;
        }
        }
    */
    /*find shadow ID, del it*/
    if(hal_fs_item_find_id(id,&tmp_addr) == PPlus_SUCCESS)
    {
        if(PPlus_SUCCESS != hal_fs_item_del(id))
            return PPlus_ERR_FATAL;
    }

    /*find latest ID, move to shadow*/
    if(hal_fs_item_find_id(id_s,&tmp_addr) == PPlus_SUCCESS)
    {
        if(PPlus_SUCCESS != hal_fs_item_id_fading(id_s, id))
            return PPlus_ERR_FATAL;
    }

    ret = hal_fs_item_write(id_s, pbuf, len);

    if(ret !=0)
    {
        LOG("wr_ret:%d\n",ret);
        return NV_OPER_FAILED;
    }

    //LOG("Success\n");
    return SUCCESS;
}


int fss_del(uint16_t id)
{
    //int ret = 0;
    uint16_t id_s = id | BIT(FSS_BIT);
    hal_fs_item_del(id);
    //LOG("del id %d, %d\r\n",id, ret);
    hal_fs_item_del(id_s);
    //LOG("del id %d, %d\r\n",id, ret);
    return PPlus_SUCCESS;
}


