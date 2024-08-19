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

/*************************************************************************************************
    Filename:       slb_app.c

    Revised:

    Revision:

    Description:    This file is used for slbota applications.
**************************************************************************************************/

#ifdef PHY_SLB_OTA_ENABLE

#include "slb_app.h"
#include "ppsp_serv.h"
#include "ppsp_impl.h"
#include "OSAL.h"
#include "clock.h"


static void simple_reset_hdlr(void);


static uint8 slb_ota_TaskID;

static ppsp_impl_clit_hdlr_t ble_ppsp_impl_appl_hdlr =
{
    .ppsp_impl_appl_rset_hdlr = simple_reset_hdlr,
};


static void simple_reset_hdlr(void)
{
    osal_start_timerEx(slb_ota_TaskID, SLB_OTA_RESET_EVENT, 3000);
}


void SLB_OTA_Init(uint8 task_id)
{
    slb_ota_TaskID = task_id;
    ppsp_serv_add_serv(PPSP_SERV_CFGS_SERV_FEB3_MASK);
    ppsp_impl_reg_serv_appl(&ble_ppsp_impl_appl_hdlr);
}


uint16 SLB_OTA_ProcessEvent(uint8 task_id, uint16 events)
{
    if(events & SLB_OTA_RESET_EVENT)
    {
        hal_system_soft_reset();
        return (events ^ SLB_OTA_RESET_EVENT);
    }

    return 0;
}


#endif
