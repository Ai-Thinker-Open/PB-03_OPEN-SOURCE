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
    Filename:       adc_config.h
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

#ifndef __ADC_CONFIG_H__
#define __ADC_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "voice.h"
#include "adc.h"

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/


/*********************************************************************
    MACROS
*/


#define ADC_RUNMODE_INTERRUPT               0

#define ADC_RUNMODE_POLLING                 1

#define ADC_RUNMODE_COMPARE                 2

#define VOICE_RUNMODE                       3

#define APP_RUN_MODE                        VOICE_RUNMODE

// ![amic]micphone bias gpio
#define VOICE_MIC_BIAS_GPIO                 GPIO_P15

// ![dmic]clk_gpio
#define VOICE_CLK_GPIO                      GPIO_P15

// ![dmic]data_gpio
#define VOICE_DATA_GPIO                     GPIO_P14

#define VOICE_COLLECT_RATE                  VOICE_RATE_8K

// ! rece origin audio data size
#define VOICE_FIFO_BUFF_SIZE                768  /* (((100-4)*4)*2) */

// ! adpcm encode target data size
#define VOICE_ENCODE_TARGER_SIZE            192  /* (((100-4)*4)/2  */

// ! request data size before adpcm encode
#define VOICE_REQUEST_CACHE_BUFF_SIZE       384  /* ((100-4)*2)     */


/*********************************************************************
    FUNCTIONS
*/



/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif

