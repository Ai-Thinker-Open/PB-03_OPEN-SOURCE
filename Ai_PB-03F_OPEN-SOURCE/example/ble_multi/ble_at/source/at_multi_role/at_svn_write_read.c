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
    Filename:       at_svn_write_read.c
    Revised:
    Revision:

    Description:    This file contains the at svn flash write or read application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "at_svn_write_read.h"
#include "at_ble_sbm_cmd.h"


uint8_t at_snv_write_flash(uint16_t idx,uint8_t len,void* buf)
{
    uint8 ret = 1;

    switch(idx)
    {
    case AT_SNV_ID_CLK_SET_OFFSET: // write clk set into flash.
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), len, buf);
        break;

    default:
        break;
    }

    if(ret == OK_RETURN_SUCCESS_STATE)
        return OK_RETURN_SUCCESS_STATE;
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_FLASH_WRITE);
        return ERROR_FLASH_WRITE;
    }
}


uint8_t at_snv_read_flash(uint16_t idx,uint8_t len,void* buf)
{
    uint8 ret = 1;

    switch(idx)
    {
    case AT_SNV_ID_CLK_SET_OFFSET: // write clk set into flash.
        ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_CLK_SET_OFFSET), len, buf);
        break;

    default:
        break;
    }

    if(ret == OK_RETURN_SUCCESS_STATE)
        return OK_RETURN_SUCCESS_STATE;
    else
    {
        AT_LOG("\r\n+ERROR=%d\r\n",ERROR_FLASH_READ);
        return ERROR_FLASH_READ;
    }
}








