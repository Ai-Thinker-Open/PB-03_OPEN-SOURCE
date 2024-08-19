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
    Filename:       bsp_btn_demo.h
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

#ifndef __BSP_BTN_DEMO_H__
#define __BSP_BTN_DEMO_H__


#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bus_dev.h"
#include "log.h"

/*********************************************************************
    CONSTANTS
*/



/*********************************************************************
    MACROS
*/
#define BSP_BTN_EVT_SYSTICK                                 (0x0001)

// ! row gpio
#define KSCAN_ROW_0_GPIO        GPIO_P00
#define KSCAN_ROW_1_GPIO        GPIO_P24
#define KSCAN_ROW_2_GPIO        GPIO_P07
#define KSCAN_ROW_3_GPIO        GPIO_P34

// ! col gpio
#define KSCAN_COL_0_GPIO        GPIO_P11
#define KSCAN_COL_1_GPIO        GPIO_P23
#define KSCAN_COL_2_GPIO        GPIO_P25
#define KSCAN_COL_3_GPIO        GPIO_P18

/*
    Task Initialization for the Demo Application
*/
extern void Demo_Init( uint8 task_id );

/*
    Task Event Processor for the Demo Application
*/
extern uint16 Demo_ProcessEvent( uint8 task_id, uint16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
