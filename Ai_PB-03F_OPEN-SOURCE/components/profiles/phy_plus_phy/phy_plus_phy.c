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
    Filename:       phy_plus_phy.c
    Revised:
    Revision:

    Description:    This file contains the phyplus phy sample application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "log.h"
#include "timer.h"
#include "phy_plus_phy.h"
#include "ll.h"
#include "ll_hw_drv.h"
#include "clock.h"
#include "gpio.h"
#include "flash.h"
#include "rf_phy_nrf.h"

/*********************************************************************
    MACROS
*/
#define PHYPLUS_SET_SYNCWORD(x)                 PHY_REG_WT(0x4003004c,(x))
#define PHYPLUS_SET_CRC_SEED(x)                 subWriteReg(0x40030048,23,0,(x))
#define PHYPLUS_SET_WHITEN_SEED(x)              subWriteReg(0x40030048,31,24,(x))
#define PHYPLUS_GET_NEEDACK_BIT(x)              (x & 0x04)
#define PHYPLUS_GET_ACK_BIT(x)                  (x & 0x08)
#define PHYPLUS_GET_PID(x)                      (x & 0x03)
#define PHYPLUS_GET_DEVICEID(x)                 (x & 0x00FF)
#define PHYPLUS_GET_GROUPID(x)                  (x>>8)

/*********************************************************************
    CONSTANTS
*/
#define PHYPLUS_RFPHY_TX_ONLY                   (0x00)
#define PHYPLUS_RFPHY_RX_ONLY                   (0x01)
#define PHYPLUS_RFPHY_TRX_ONLY                  (0x02)
#define PHYPLUS_RFPHY_RX_TXACK                  (0x03)
#define PHYPLUS_RFPHY_TX_PENDING                (0x10)
#define PHYPLUS_RFPHY_RX_PENDING                (0x11)
#define PHYPLUS_RFPHY_TRX_PENDING               (0x12)
#define PHYPLUS_RFPHY_RX_TXACK_PENDING          (0x13)

#define PHYPLUS_RFPHY_IDLE                      (0xFF)

#define RFPHY_STATUS_SET_PENDING(x)             (0x10 |(x))
#define RFPHY_STATUS_CLR_PENDING(x)             (0x0F &(x))

#define PHYPLUS_AUTOACK_ENABLE                  (1)
#define PHYPLUS_AUTOACK_DISABLE                 (0)
#define PHYPLUS_NRF_DISABLE                     (0)
#define PHYPLUS_NRF_ENABLE                      (1)
#define PHYPLUS_MESH_DISABLE                     (0)
#define PHYPLUS_MESH_ENABLE                      (1)
#define LL_HW_MODE_STX                          (0x00)
#define LL_HW_MODE_SRX                          (0x01)
#define LL_HW_MODE_TRX                          (0x02)


#define PHYPLUS_HW_SCAN_DELAY                   (80)
#define PHYPLUS_HW_BB_DELAY                     (90)
#define PHYPLUS_HW_AFE_DELAY                    ( 8)
#define PHYPLUS_HW_PLL_DELAY                    (60)

#define DEFAULT_CRC_SEED                        (0x555555)

#define DEFAULT_WHITEN_SEED                     (0x37)
#define WHITEN_SEED_CH37                        (0x53)
#define WHITEN_SEED_CH38                        (0x33)
#define WHITEN_SEED_CH39                        (0x73)
#define DEFAULT_WHITEN_SEED                     (0x37)
#define WHITEN_OFF                              (0x00)

#define BLE_ADV_CHN37                           (02)
#define BLE_ADV_CHN38                           (26)
#define BLE_ADV_CHN39                           (80)

#define PHYPLUS_CHAN_MAP_NUM                    (1)

#define DEFAULT_SYNCWORD                        (0x8e89bed6)

#define PHYPLUS_PKT_FMT_1M                      (0x01)
#define PHYPLUS_PKT_FMT_2M                      (0x02)
#define PHYPLUS_PKT_FMT_500K                    (0x05)
#define PHYPLUS_PKT_FMT_100K                    (0x06)

#define PHYPLUS_HW_MAX_RX_TO                    (20000)
#define PHYPLUS_HW_MIN_SCAN_TIME_US             (2000)

#define PHYPLUS_RF_TXACK_PKT_LEN                (6)
#if DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE
    #define PHYPLUS_DATA_MAX_LENGTH                 (31)
#else
    #define PHYPLUS_DATA_MAX_LENGTH                 (32)
#endif
#define PHYPLUS_RF_MAX_LENGTH                   (PHYPLUS_DATA_MAX_LENGTH+12)





#define PHYPLUS_SPHY_RFCHN                      (2)

#define RFPHY_TX_PENDING_RETRY_DLY              (2)  //Ms
#define RFPHY_RX_PENDING_RETRY_DLY              (2)  //Ms


/*********************************************************************
    BUILD Config
*/
#ifndef DEF_PHYPLUS_NRF_SUPPORT
    #define DEF_PHYPLUS_NRF_SUPPORT PHYPLUS_NRF_DISABLE
#endif

#ifndef DEF_PHYPLUS_AUTOACK_SUPPORT
    #define DEF_PHYPLUS_AUTOACK_SUPPORT PHYPLUS_AUTOACK_DISABLE
#endif


#ifndef DEF_PHYPLUS_TRX_SUPPORT
    #define DEF_PHYPLUS_TRX_SUPPORT PHYPLUS_CONFIG_TRX_ALL
#endif


extern uint8 ll_hw_get_tr_mode(void);
extern volatile uint32 llWaitingIrq;
/*********************************************************************
    TYPE Define
*/
typedef struct pktCfg_s
{
    uint8_t     pktFmt;
    uint8_t     pduLen;
    uint8_t     wtSeed;
    uint8_t     crcFmt;
    uint32_t    crcSeed;
    uint32_t    syncWord;
    uint8_t*    p_txBuf;
    uint8_t*    p_rxBuf;
} pktCfg_t;

typedef struct phyCtx_s
{
    uint8_t     Status;
    uint32_t    txIntv;
    uint32_t    txDuration;
    uint32_t    rxIntv;
    uint32_t    rxDuration;
    uint8_t     rfChn;
    uint32_t    rxOnlyTO;
    uint16_t    rxAckTO;
    uint32_t    rxScanT0;
    uint8_t     reTxCnt;
    uint8_t     reTxMax;
    uint8_t     txAck;
    uint16_t    reTxDly;
    uint8_t     enAutoAck;
    uint8_t     phyMode;
} phyCtx_t;

typedef struct phySch_s
{
    uint32_t    txMargin;
    uint32_t    rxMargin;
} phySch_t;

typedef struct phyDebug_s
{
    uint32_t   rx_data_cnt;
    uint32_t   rx_data_ign;
    uint32_t   rx_crc_err;
    uint32_t   rx_txack_cnt;
    uint32_t   tx_data_cnt;
    uint32_t   tx_ack_cnt;
    uint32_t   tx_retry_cnt;
    uint32_t   rx_txack_t0;
    uint32_t   rx_txack_t1;
} phyDeubg_t;

typedef struct chanHop_s
{
    uint8_t chan_map[PHYPLUS_CHAN_MAP_NUM];
    uint8_t wt_map[PHYPLUS_CHAN_MAP_NUM];
    uint8_t curIdx;
} chanHop_t;

/*********************************************************************
    LOCAL VARIABLES
*/
uint8 PhyPlusPhy_TaskID; // Task ID for internal task/event processing
//volatile uint32 phyWaitingIrq = FALSE;
uint32 PHY_ISR_entry_time = 0;

__align(4) uint8_t  phyBufRx[256];
__align(4) uint8_t  phyBufTx[256];
//static uint8_t s_pubAddr[6];

uint8_t s_pubAddr[6];
static uint8_t advHead[2];


#if(DEF_PHYPLUS_AUTOACK_SUPPORT  ==   PHYPLUS_AUTOACK_ENABLE)
    static uint32_t s_rf_crc=0xFFFFFFFF;
    static uint32_t s_rf_lastcrc=0xFFFFFFFF;
#endif
static uint8_t s_rf_pid=0xFF;
#if(DEF_PHYPLUS_AUTOACK_SUPPORT  ==   PHYPLUS_AUTOACK_ENABLE && DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_DISABLE)
    static uint8_t s_rf_lastpid=0xFF;
#endif
static uint32_t s_txT0;
#if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE)
    uint16_t s_rf_netid=0x0a04;
    uint16_t s_rf_target_netid=0x0103;
#endif

static uint8_t adv_buffer[32];
#if(DEF_PHYPLUS_MESH_SUPPORT != PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE)
static uint8_t peer_addr[6]= {0x4,0x3,0x2,0x1,0x6,0x5};
#endif
#if(PHYPLUS_CHAN_MAP_NUM==1)
static chanHop_t s_chanHop =
{
    .chan_map={76},     //Ch37 != 2?
    .wt_map={WHITEN_SEED_CH37},
    .curIdx =0,
};
#else
static chanHop_t s_chanHop =
{
    .chan_map={BLE_ADV_CHN37,BLE_ADV_CHN38,BLE_ADV_CHN39},
    .wt_map={WHITEN_SEED_CH37,WHITEN_SEED_CH38,WHITEN_SEED_CH39},
    .curIdx =0,
};
#endif

uint16 phyFoff=0;
uint8  phyCarrSens=0;
uint8  phyRssi=0;

static pktCfg_t s_pktCfg;
static phyCtx_t s_phy;
static phyDeubg_t s_phyDbg;
static phySch_t s_phySch;

phy_comm_cb_t phy_data_cbfunc = NULL;
phy_comm_cb_t phy_opcode_cbfunc = NULL;
/*********************************************************************
    LOCAL FUNCTIONS
*/
extern void PLUSPHY_IRQHandler(void);
uint32 BLE_IRQHandler_Restore = NULL;
void PhyPlusPhy_Set_BLE_IRQHandler(void)
{
    if(BLE_IRQHandler_Restore)
    {
        JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   BLE_IRQHandler_Restore;
        ll_hw_set_crc_fmt(LL_HW_CRC_BLE_FMT,LL_HW_CRC_BLE_FMT);
        subWriteReg(0x40030040, 4, 4, 0);
        subWriteReg(0x40030008,8,8,1);
    }
}
uint8_t rf_rxdata_check(uint8_t* din)
{
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT  ==   PHYPLUS_AUTOACK_ENABLE)
    uint8_t ret = PPlus_ERR_FATAL;
    {
        #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_DISABLE)
        uint8_t* pData = din;

        if((PHYPLUS_GET_ACK_BIT(pData[0])) || !(PHYPLUS_GET_NEEDACK_BIT(pData[0])) || ((PHYPLUS_GET_PID(pData[0])) == s_rf_lastpid) || (s_rf_crc == s_rf_lastcrc))   //check ACK, NEEDACK, PID, CRC
        {
            ret = PPlus_ERR_INVALID_FLAGS;
            LOG_DEBUG("wrong flags: ACK:%d  NEEDACK:%d  PID:%d  CRCcode:%d\n",pData[0]&0x08,pData[0]&0x04,pData[0]&0x03,s_rf_crc);

            if(((pData[0]&0x03) == s_rf_lastpid) || (s_rf_crc == s_rf_lastcrc))
            {
                LOG_DEBUG("repeated message\n");
            }

            //gpio_write(P9,1);gpio_write(P9,0);gpio_write(P9,1);gpio_write(P9,0);
            //LOG_DEBUG(":%02x:%02x:%02x\n",pData[0],s_rf_crc,s_rf_lastpid);
            return ret;
        }

        s_rf_lastpid = pData[0]&0x03;
        #else       //20221020 ZONG:remove PID&Addr check when in mesh

        if(s_rf_crc == s_rf_lastcrc)
        {
            ret = PPlus_ERR_INVALID_FLAGS;
            LOG_DEBUG("repeated message\n");
            //gpio_write(P9,1);gpio_write(P9,0);gpio_write(P9,1);gpio_write(P9,0);
            //LOG_DEBUG(":%02x:%02x:%02x\n",pData[0],s_rf_crc,s_rf_lastpid);
            return ret;
        }

        #endif
    }
    s_rf_lastcrc = s_rf_crc;
    #endif
    return PPlus_SUCCESS;
}

uint8_t rf_txack_check(uint8_t* din)
{
    uint8_t ret = PPlus_ERR_FATAL;
    uint8_t* pData = din;

    if(!(PHYPLUS_GET_ACK_BIT(pData[0])) || (PHYPLUS_GET_NEEDACK_BIT(pData[0])) || (PHYPLUS_GET_PID(pData[0])!=s_rf_pid))   //check the bits of ACK, NEEDACK, PID
    {
        ret = PPlus_ERR_INVALID_DATA;
        // LOG("wrong flags: ACK:%d  NEEDACK:%d  PID:%d\n",pData[0]&0x08,pData[0]&0x04,pData[0]&0x03);
        // if(PHYPLUS_GET_PID(pData[0])!=s_rf_pid)
        // {
        //     LOG("wrong PID\n");
        // }
        return ret;
    }

    return PPlus_SUCCESS;
}

#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)

static uint32  read_ble_remainder_time(void)
{
    uint32 currentCount;
    uint32 g_tim1_pass = read_current_fine_time();
    currentCount = AP_TIM1->CurrentCount;

    if((currentCount < 6) || NVIC_GetPendingIRQ(TIM1_IRQn))
        return 0;
    else
        return (currentCount >> 2);
}

static uint8 phy_allow_tx(void)
{
    uint32 advTime, margin;
    uint32 remainTime;
    uint8 ret = FALSE;
    uint16 pktLen = phyBufTx[1]+2+3;//pdulen + header + crc
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)

    if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        pktLen = (nrfTxBuf.pldLen + 9);

    #endif

    if(s_pktCfg.pktFmt==PHYPLUS_PKT_FMT_1M)
    {
        pktLen = pktLen<<3;
    }
    else
    {
        pktLen = pktLen<<2;
    }

    // Hold off interrupts.
    HAL_ENTER_CRITICAL_SECTION( );
    // read global config to get advTime and margin
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT==PHYPLUS_AUTOACK_ENABLE)
    advTime = (pktLen+s_phy.reTxDly); //
    #else
    advTime = pktLen+200; //
    #endif
    margin = s_phySch.txMargin;
    // remain time before trigger LL HW
    remainTime = (AP_TIM1->ControlReg) ? read_ble_remainder_time() : 0xffff;
    // remainTime = (AP_TIM1->ControlReg) ?read_LL_remainder_time() :0xffff;

    if ((remainTime > advTime + margin)
            && !llWaitingIrq)
        ret = TRUE;

    HAL_EXIT_CRITICAL_SECTION();

    if(ret==FALSE)
    {
        //LOG("[DIS TX] s%d r%d a%d m%d\n",llWaitingIrq,remainTime,advTime,margin);
    }

    return ret;
}
#endif

#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
static uint8 phy_allow_rx(uint32_t* scanTimeAllow)
{
    uint32 scanTime, margin;
    uint32 remainTime;
    uint8 ret = FALSE;
    uint32_t t0=read_current_fine_time();
    // Hold off interrupts.
    HAL_ENTER_CRITICAL_SECTION( );
    // read global config to get advTime and margin
    margin = s_phySch.rxMargin;
    // remain time before trigger LL HW
    remainTime = (AP_TIM1->ControlReg) ? read_ble_remainder_time() : 0xffff;
    // remainTime = (AP_TIM1->ControlReg) ?read_LL_remainder_time() :0xffff;
    remainTime = remainTime>margin ?
                 remainTime - margin : 0;
    scanTime = (TIME_DELTA(t0, s_phy.rxScanT0)<s_phy.rxOnlyTO) ?
               s_phy.rxOnlyTO - TIME_DELTA(t0, s_phy.rxScanT0) :
               PHYPLUS_HW_MIN_SCAN_TIME_US;

    if(scanTime < PHYPLUS_HW_MIN_SCAN_TIME_US)
        scanTime = PHYPLUS_HW_MIN_SCAN_TIME_US;          //limit the minimum timeout for RX

    scanTime = remainTime>scanTime ? scanTime : remainTime;
    scanTime = scanTime<PHYPLUS_HW_MIN_SCAN_TIME_US ? 0 : scanTime;     //if remainTime is less than PHYPLUS_HW_MIN_SCAN_TIME_US, pending
    scanTime  = scanTime>0xffff ? 0xffff:scanTime;// max scan time out is 16bit

    if ((scanTime) && !llWaitingIrq)
        ret = TRUE;

    *scanTimeAllow = scanTime;
    HAL_EXIT_CRITICAL_SECTION();

    if(ret==FALSE)
    {
        //LOG("[DIS RX] s%d r%d a%d m%d\n",llWaitingIrq,remainTime,scanTime,margin);
    }

    return ret;
}
#endif

static uint8_t phy_rx_data_check(void)
{
    uint8_t ret=PPlus_SUCCESS;

    if(s_phy.Status==PHYPLUS_RFPHY_RX_ONLY || s_phy.Status==PHYPLUS_RFPHY_RX_TXACK)
    {
        #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
        {
            ret = nrf_rxdata_check(phyBufRx);
        }
        //else
        #else
        {
            ret = rf_rxdata_check(phyBufRx);
        }
        #endif

        //process data
        if(ret==PPlus_SUCCESS)
        {
            osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DATA_PROCESS_EVT);
            s_phyDbg.rx_data_cnt++;
        }
        else
        {
            s_phyDbg.rx_data_ign++;
        }
    }
    //process txAck
    else if(s_phy.Status==PHYPLUS_RFPHY_TRX_ONLY)
    {
        #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
        //process data check
        ret = nrf_txack_check(phyBufRx);//check from the addr[0]
        #else
        {
            ret = rf_txack_check(phyBufRx);     //reserve for rf txack check
        }
        #endif

        if(ret==PPlus_SUCCESS)
        {
            s_phy.txAck=1;
            s_phyDbg.rx_txack_cnt++;
        }

        return ret;
    }

    return ret;
}

void phy_set_channel(uint8 rfChnIdx)
{
    if(g_rfPhyFreqOffSet>=0)
        PHY_REG_WT(0x400300b4, (g_rfPhyFreqOffSet<<16)+(g_rfPhyFreqOffSet<<8)+rfChnIdx);
    else
        PHY_REG_WT(0x400300b4, ((255+g_rfPhyFreqOffSet)<<16)+((255+g_rfPhyFreqOffSet)<<8)+(rfChnIdx-1) );
}

void phy_hw_go(void)
{
    //20190115 ZQ recorded ll re-trigger
    if(llWaitingIrq==TRUE)
    {
        LOG_DEBUG("[PHY TRIG ERR]\n");
    }

    *(volatile uint32_t*)(LL_HW_BASE+ 0x14) = LL_HW_IRQ_MASK;   //clr  irq status
    *(volatile uint32_t*)(LL_HW_BASE+ 0x0c) = 0x0001;           //mask irq :only use mode done
    *(volatile uint32_t*)(LL_HW_BASE+ 0x00) = 0x0001;           //trig
    uint8_t rfChnIdx = PHY_REG_RD(0x400300b4)&0xff;

    if(rfChnIdx<2)
    {
        rfChnIdx=2;
    }
    else if(rfChnIdx>80)
    {
        rfChnIdx=80;
    }

    if(s_pktCfg.pktFmt==PKT_FMT_BLE2M)
        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0_2Mbps,g_rfPhyTpCal1_2Mbps,(rfChnIdx-2)>>1));
    else
        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0,g_rfPhyTpCal1,(rfChnIdx-2)>>1));

    //change to diff demod for nrf2.4G
    subWriteReg(0x40030004,14,8,60);
    subWriteReg(0x40030008,8,8,0);
}


void phy_hw_stop(void)
{
    uint8_t cnt=0;
    ll_hw_set_rx_timeout(33);//will trigger ll_hw_irq=RTO

    while(llWaitingIrq)
    {
        WaitRTCCount(3);
        cnt++;

        if(cnt>10)
        {
            LOG_DEBUG("[PHY STOP ERR]\n");
            break;
        }
    };
}

void phy_hw_set_srx(uint16 rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_srx();
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_set_stx(void)
{
    ll_hw_set_stx();
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_set_trx(uint16 rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_trx();
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_timing_setting(void)
{
    ll_hw_set_tx_rx_release (10,     1);
    ll_hw_set_rx_tx_interval(       60);        //T_IFS=150us for BLE 1M
    ll_hw_set_tx_rx_interval(       10);        //T_IFS=150us for BLE 1M, 20220510 for NRF T_IFS 130us,set to 10
    ll_hw_set_trx_settle    (57, 8, 52);        //TxBB,RxAFE,PL
}

void phy_hw_pktFmt_Config(pktCfg_t cfg)
{
    //baseband cfg
    rf_phy_bb_cfg(cfg.pktFmt);
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    //pktfmt
    {
        //fix length mode ,no hw crc gen/check(but need sw crc gen/check)
        ll_hw_set_pplus_pktfmt(cfg.pduLen);
        ll_hw_ign_rfifo(LL_HW_IGN_NONE);
    }
    #else
    {
        //crc
        ll_hw_set_crc_fmt(cfg.crcFmt,cfg.crcFmt);
        PHYPLUS_SET_CRC_SEED(cfg.crcSeed);
        ll_hw_ign_rfifo(LL_HW_IGN_CRC);
    }
    #endif
    //whiten
    PHYPLUS_SET_WHITEN_SEED(cfg.wtSeed);
    //syncword
    PHYPLUS_SET_SYNCWORD(cfg.syncWord);
}
#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)
uint8_t phy_rf_tx(void)
{
    if(phy_allow_tx()==FALSE)
    {
        return PPlus_ERR_BUSY;
    }
    else
    {
        JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&PLUSPHY_IRQHandler;
    }

    phy_hw_stop();
    HAL_ENTER_CRITICAL_SECTION();
    phy_hw_pktFmt_Config(s_pktCfg);
    phy_hw_timing_setting();
    phy_set_channel(s_phy.rfChn);

    if(s_phy.Status==PHYPLUS_RFPHY_TRX_ONLY)
        phy_hw_set_trx(s_phy.rxAckTO);
    else
        phy_hw_set_stx();

    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    {
        set_max_length(s_pktCfg.pduLen);
        ll_hw_write_tfifo(s_pktCfg.p_txBuf,s_pktCfg.pduLen);
    }
    #else
    {
        set_max_length(0xff);
        //need updata phyBufTx
        ll_hw_write_tfifo(s_pktCfg.p_txBuf,s_pktCfg.p_txBuf[1]+2);
    }
    #endif
    phy_hw_go();
    llWaitingIrq=TRUE;
    HAL_EXIT_CRITICAL_SECTION();
    return PPlus_SUCCESS;
}
#endif

#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
uint8_t phy_rf_rx(void)
{
    uint32_t scanTime;

    if(phy_allow_rx(&scanTime)==FALSE)
    {
        return PPlus_ERR_BUSY;
    }
    else
    {
        JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&PLUSPHY_IRQHandler;
    }

    phy_hw_stop();
    HAL_ENTER_CRITICAL_SECTION();
    phy_hw_pktFmt_Config(s_pktCfg);
    phy_hw_timing_setting();
    phy_set_channel(s_phy.rfChn);
    phy_hw_set_srx((0xffff&scanTime));
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    {
        set_max_length(s_pktCfg.pduLen);
    }
    #else
    {
        set_max_length(0xff);
    }
    #endif
    phy_hw_go();
    llWaitingIrq=TRUE;
    HAL_EXIT_CRITICAL_SECTION();
    return PPlus_SUCCESS;
}
#endif
void phy_rf_txack(void)
{
    uint32_t t0,t1,T2,delay;
    uint8_t dLen;
    ll_hw_set_stx();
    t0=read_current_fine_time();
    T2 = TIME_DELTA(t0,PHY_ISR_entry_time);
    delay = 127-T2;
    ll_hw_set_trx_settle(   delay,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
    phy_hw_go();
    ll_hw_rst_tfifo();
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    //if(s_pktCfg.crcFmt == LL_HW_CRC_NULL)
    {
        nrfAckBuf.pid = GET_NRF_PKT_PID(phyBufRx);
        // if(phy_opcode_cbfunc != NULL)
        // {
        //     phy_comm_evt_t gen_ack_evt;
        //     gen_ack_evt.type = PHYPLUS_SMART_NRF_TYPE;
        //     gen_ack_evt.data = nrfAckBuf.pdu;
        //     if(phy_opcode_cbfunc(&gen_ack_evt) == PPlus_SUCCESS)
        //     {
        //         nrfAckBuf.pldLen = gen_ack_evt.len;
        //     }
        // }
        nrf_pkt_enc(&nrfAckBuf,phyBufTx,&dLen);     //generate CRC number
        set_max_length(dLen);
        ll_hw_write_tfifo(s_pktCfg.p_txBuf,dLen);
    }
    #else
    #if(DEF_PHYPLUS_MESH_SUPPORT==PHYPLUS_MESH_DISABLE)
    {
        uint8_t rfRxPID;
        uint8_t rfTxAckBuf[40];
        //setup RF AutoAck Pkt
        rfRxPID = PHYPLUS_GET_PID(s_pktCfg.p_rxBuf[0]); //get PID
        rfTxAckBuf[0]= 0x08 | rfRxPID;    //pack type-flag and PID into 1st byte
        rfTxAckBuf[1]= PHYPLUS_RF_TXACK_PKT_LEN;
        osal_memcpy(&rfTxAckBuf[2],s_pubAddr,6);//20221020 ZONG: turn to send own addr

        if(phy_opcode_cbfunc != NULL &&
                PHYPLUS_GET_OPCODE(s_pktCfg.p_rxBuf[0]))      //check if it is available and necessary
        {
            phy_comm_evt_t gen_ack_evt;
            gen_ack_evt.type = s_pktCfg.p_rxBuf[0];
            gen_ack_evt.data = rfTxAckBuf+8;

            if(phy_opcode_cbfunc(&gen_ack_evt) == PPlus_SUCCESS)
            {
                rfTxAckBuf[0] |= (s_pktCfg.p_rxBuf[0] & 0xF0);
                rfTxAckBuf[1] = gen_ack_evt.len + PHYPLUS_RF_TXACK_PKT_LEN;
            }
        }

        dLen=rfTxAckBuf[1];
        set_max_length(44);
        ll_hw_write_tfifo(rfTxAckBuf,dLen+2);
    }
    #else
    {
        uint8_t rfRxPID;
        uint8_t rfTxAckBuf[40];
        //setup RF AutoAck Pkt
        rfRxPID = PHYPLUS_GET_PID(s_pktCfg.p_rxBuf[0]); //get PID
        rfTxAckBuf[0]= 0x08 | rfRxPID;    //pack type-flag and PID into 1st byte
        rfTxAckBuf[1]= PHYPLUS_RF_TXACK_PKT_LEN+4;
        osal_memcpy(&rfTxAckBuf[2],s_pubAddr,6);//20221020 ZONG: turn to send own addr
        rfTxAckBuf[8] = s_rf_netid & 0xFF;
        rfTxAckBuf[9] = s_rf_netid >> 8;
        rfTxAckBuf[10] = s_pktCfg.p_rxBuf[8];
        rfTxAckBuf[11] = s_pktCfg.p_rxBuf[9];

        if(phy_opcode_cbfunc != NULL &&
                PHYPLUS_GET_OPCODE(s_pktCfg.p_rxBuf[0]))      //check if it is available and necessary
        {
            phy_comm_evt_t gen_ack_evt;
            gen_ack_evt.type = s_pktCfg.p_rxBuf[0];
            gen_ack_evt.data = rfTxAckBuf+12;

            if(phy_opcode_cbfunc(&gen_ack_evt) == PPlus_SUCCESS)
            {
                rfTxAckBuf[0] |= (s_pktCfg.p_rxBuf[0] & 0xF0);
                rfTxAckBuf[1] = gen_ack_evt.len + PHYPLUS_RF_TXACK_PKT_LEN;
            }
        }

        dLen=rfTxAckBuf[1];
        set_max_length(44);
        ll_hw_write_tfifo(rfTxAckBuf,dLen+2);
    }
    #endif
    #endif
    t1=read_current_fine_time();
    llWaitingIrq=TRUE;
    s_phy.Status = PHYPLUS_RFPHY_RX_TXACK;
    s_phyDbg.rx_txack_t0 = TIME_DELTA(t0,PHY_ISR_entry_time);       //time from IRQ to Set_STX
    s_phyDbg.rx_txack_t1 = TIME_DELTA(t1,t0);                       //time from Set_STX to TX_Ready
    LOG_DEBUG("Delay: %d, Spent Time: %d\n",delay, s_phyDbg.rx_txack_t1);
    s_phyDbg.tx_ack_cnt++;
}

void phy_rx_data_process(void)
{
    HAL_ENTER_CRITICAL_SECTION();
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    nrfPkt_t pData;
    osal_memcpy(&pData,&nrfRxBuf,sizeof(nrfPkt_t));
    uint8_t pRssi = phyRssi;
    #else
    uint8_t pLen = s_pktCfg.p_rxBuf[1]+2;
    uint8_t pData[PHYPLUS_RF_MAX_LENGTH];
    osal_memcpy(pData,s_pktCfg.p_rxBuf,pLen);
    uint8_t pRssi = phyRssi;
    #endif
    HAL_EXIT_CRITICAL_SECTION();
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    //if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
    {
        if(phy_data_cbfunc == NULL)
        {
            LOG("-------------------------\n");
            LOG("[PHY RX] [-%03ddbm %4dKHz %02d CH] ",pRssi,phyFoff-512,s_phy.rfChn);
            LOG("PCF: L%2x pid %2x no_ack %2x ",pData.pldLen,pData.pid,pData.noAckBit);
            LOG("ADDR:");
            my_dump_byte(&pData.addr[0], 5);
            LOG("PDU:");
            my_dump_byte(&pData.pdu[0], pData.pldLen);
        }
        else
        {
            phy_comm_evt_t phy_pdu_cb;
            phy_pdu_cb.type = PHYPLUS_SMART_NRF_TYPE;
            phy_pdu_cb.rssi = pRssi;
            phy_pdu_cb.len = pData.pldLen;
            phy_pdu_cb.data = pData.pdu;
            phy_data_cbfunc(&phy_pdu_cb);
        }
    }
    #else
    {
        uint8_t pduLen = pLen;

        if(phy_data_cbfunc == NULL)
            //if(osal_memcmp(peer_addr,pData+2,6))     //len 5->6
        {
            //pduLen= pData[1];
            //pwm light control
            // #if(DEF_PHYPLUS_AUTOACK_SUPPORT==1)
            //     if(pData[1]==3+6   &&
            //        pData[8]==0xff &&
            //        pData[9]==0xf1)
            //        {
            //             pwmlight_phy_control(pData[10]);
            //        }
            // #endif
            LOG("-------------------------\n");
            LOG("[PHY RX] [-%03ddbm %4dKHz %02d CH] ",pRssi,phyFoff-512,s_phy.rfChn);
            my_dump_byte(&pData[0],pduLen);
        }
        else
        {
            phy_comm_evt_t phy_pdu_cb;
            phy_pdu_cb.type = pData[0];
            phy_pdu_cb.len = pData[1];
            phy_pdu_cb.rssi = pRssi;
            phy_pdu_cb.data = pData+2;
            phy_data_cbfunc(&phy_pdu_cb);
        }
    }
    #endif
}

void phy_tx_buf_updata(uint8_t* adva,uint8_t* txHead,uint8_t* txPayload,uint8_t dlen)
{
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT  ==   PHYPLUS_AUTOACK_ENABLE)
    {
        uint8_t lastpid = PHYPLUS_GET_PID(txHead[0]);
        s_rf_pid = (lastpid +1) & 0x03;
        txHead[0] = (txHead[0] & 0xFC) | s_rf_pid;
        #if(DEF_PHYPLUS_MESH_SUPPORT==PHYPLUS_MESH_ENABLE)

        if(s_phy.Status == PHYPLUS_RFPHY_TX_ONLY)
        {
            txHead[0] = txHead[0]&0xFB;       //remove need_ack bit
        }
        else
        {
            txHead[0] = txHead[0]|0x04;       //add need_ack bit
        }

        #endif
    }
    #endif
    txHead[1] = dlen + 6;
    osal_memcpy(&(phyBufTx[0]),&(txHead[0]),2);          //copy tx header
    osal_memcpy(&(phyBufTx[2]),&(adva[0]),6);              //copy AdvA
    osal_memcpy(&(phyBufTx[8]),&(txPayload[0]),dlen);      //copy payload
    LOG("\n-----------------------------------------------\n");
    LOG("PHY BUF Tx Dump\n");

    for(uint8_t i=0; i<phyBufTx[1]+2; i++)
        LOG("%02x ",phyBufTx[i]);

    LOG("\n-----------------------------------------------\n");
}
#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
void phy_rf_process_recv(void)
{
    if(s_phy.Status == PHYPLUS_RFPHY_IDLE)
        return;

    HAL_ENTER_CRITICAL_SECTION();
    s_phy.Status = RFPHY_STATUS_CLR_PENDING(s_phy.Status);
    uint8_t ret= phy_rf_rx();
    HAL_EXIT_CRITICAL_SECTION();

    if(ret==PPlus_ERR_BUSY)
    {
        s_phy.Status = RFPHY_STATUS_SET_PENDING(s_phy.Status);
        osal_start_timerEx(PhyPlusPhy_TaskID,PPP_RX_PENDING_PROCESS_EVT,RFPHY_RX_PENDING_RETRY_DLY);
    }
}
#endif
#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)
uint8_t phy_rf_process_tsmt(void)
{
    if(s_phy.Status == PHYPLUS_RFPHY_IDLE)
        return PPlus_ERR_BUSY;

    HAL_ENTER_CRITICAL_SECTION();
    s_phy.Status = RFPHY_STATUS_CLR_PENDING(s_phy.Status);
    uint8_t ret= phy_rf_tx();
    HAL_EXIT_CRITICAL_SECTION();

    if(ret==PPlus_ERR_BUSY)
    {
        s_phy.Status = RFPHY_STATUS_SET_PENDING(s_phy.Status);
        osal_start_timerEx(PhyPlusPhy_TaskID,PPP_TX_PENDING_PROCESS_EVT,RFPHY_TX_PENDING_RETRY_DLY);
        return ret;
    }

    return PPlus_SUCCESS;
}
#endif
void phy_rf_channel_hop(void)
{
    s_chanHop.curIdx++;

    if(s_chanHop.curIdx==PHYPLUS_CHAN_MAP_NUM)
        s_chanHop.curIdx = 0;

    s_phy.rfChn         = s_chanHop.chan_map[s_chanHop.curIdx];
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_DISABLE)
    s_pktCfg.wtSeed     = s_chanHop.wt_map[s_chanHop.curIdx];
    #endif
}
void phy_rf_schedule(void)
{
    if(s_phy.Status == PHYPLUS_RFPHY_TX_ONLY)
    {
        #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)
        phy_rf_channel_hop();

        if(0==s_chanHop.curIdx)         //what if not using s_chanHop.chan_map[0]?
        {
            uint32_t t0=read_current_fine_time();

            if(s_phy.txDuration)
            {
                if(TIME_DELTA(t0,s_txT0)>s_phy.txDuration)
                {
                    s_phy.Status = PHYPLUS_RFPHY_IDLE;
                    osal_set_event(PhyPlusPhy_TaskID,PPP_TX_DONE_EVT);
                }
                else
                {
                    osal_set_event(PhyPlusPhy_TaskID,PPP_TX_PENDING_PROCESS_EVT);
                }
            }
            else if(s_phy.txIntv==0)
            {
                s_phy.Status = PHYPLUS_RFPHY_IDLE;
                osal_set_event(PhyPlusPhy_TaskID,PPP_TX_DONE_EVT);
            }
            else
            {
                osal_start_timerEx(PhyPlusPhy_TaskID,PPP_TX_PENDING_PROCESS_EVT,s_phy.txIntv);
            }
        }
        else
        {
            phy_rf_process_tsmt();
        }

        #endif
    }
    else if(    s_phy.Status == PHYPLUS_RFPHY_RX_ONLY)
    {
        #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
        uint32_t t0=read_current_fine_time();

        if(s_phy.rxOnlyTO == 0)
        {
            s_phy.Status = PHYPLUS_RFPHY_IDLE;
            osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DONE_EVT);
            return;
        }

        if(TIME_DELTA(t0, s_phy.rxScanT0)<s_phy.rxOnlyTO)
        {
            phy_rf_process_recv();
            return;
        }
        else
        {
            //update rx scan time stamp on next chn
            s_phy.rxScanT0 = t0;
        }

        phy_rf_channel_hop();
        phy_rf_process_recv();
        #endif
    }

    #if(DEF_PHYPLUS_AUTOACK_SUPPORT  ==   PHYPLUS_AUTOACK_ENABLE)
    else if(s_phy.Status == PHYPLUS_RFPHY_RX_TXACK)
    {
        s_phy.Status = PHYPLUS_RFPHY_RX_ONLY;
    }

    #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)
    else if(s_phy.Status == PHYPLUS_RFPHY_TRX_ONLY)
    {
        phy_rf_channel_hop();

        if(0==s_chanHop.curIdx)
        {
            s_phy.reTxCnt++;
            s_phyDbg.tx_retry_cnt++;
        }

        uint8_t txdone = 0;

        if(s_phy.txDuration != 0)
        {
            uint32_t t0=read_current_fine_time();

            if(s_phy.txAck || TIME_DELTA(t0,s_txT0)>s_phy.txDuration)
                txdone = 1;
        }
        else
        {
            if(s_phy.txAck || s_phy.reTxCnt>s_phy.reTxMax )
                txdone = 1;
        }

        if(txdone == 1 )
        {
            txdone = 0;
            s_chanHop.curIdx = 0;       //reset chmap

            if(s_phy.txAck)
            {
                LOG("[TX OK]\n");
            }
            else
            {
                LOG("[TX FAIL]\n");
            }

            osal_set_event(PhyPlusPhy_TaskID,PPP_TRX_DONE_EVT);

            if(s_phy.txIntv==0)
            {
                s_phy.Status = PHYPLUS_RFPHY_IDLE;
            }
            else
            {
                osal_start_timerEx(PhyPlusPhy_TaskID,PPP_TX_PENDING_PROCESS_EVT,s_phy.txIntv);
            }
        }
        else
        {
            if(s_phy.reTxDly>s_phy.rxAckTO)
                WaitUs(s_phy.reTxDly-s_phy.rxAckTO);

            phy_rf_process_tsmt();
        }
    }

    #endif
    #endif
}
/*******************************************************************************
    @fn          PLUSPHY_IRQHandler

    @brief      Interrupt Request Handler for Link Layer

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None
*/
void PLUSPHY_IRQHandler(void)
{
    uint8         mode;
    uint32_t      irq_status;
    PHY_ISR_entry_time = read_current_fine_time();
    irq_status = ll_hw_get_irq_status();

    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
        PhyPlusPhy_Set_BLE_IRQHandler();
        return;
    }

    llWaitingIrq = FALSE;
    HAL_ENTER_CRITICAL_SECTION();
    mode = ll_hw_get_tr_mode();

    // ===================   mode TRX process 1
    if (mode == LL_HW_MODE_STX  &&
            (s_phy.Status == PHYPLUS_RFPHY_TX_ONLY)
       )
    {
        //osal_set_event(PhyPlusPhy_TaskID,PPP_TX_DONE_EVT);
    }

    #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
    else if(mode == LL_HW_MODE_SRX  &&
            (s_phy.Status == PHYPLUS_RFPHY_RX_ONLY)
           )
    {
        rf_phy_get_pktFoot(&phyRssi,&phyFoff,&phyCarrSens);
        #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
        //if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        {
            if(0==(irq_status & LIRQ_RTO))
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo_pplus(s_pktCfg.p_rxBuf, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);

                //do crc check
                if(nrf_pkt_crc_check(phyBufRx))
                {
                    {
                        #if(DEF_PHYPLUS_AUTOACK_SUPPORT == PHYPLUS_AUTOACK_ENABLE)       //SRX:RX->TXACK
                        {
                            phy_rf_txack();
                        }
                        #endif
                        phy_rx_data_check();
                    }
                }
                else
                    s_phyDbg.rx_crc_err++;
            }
        }
        #else
        {
            if(irq_status & LIRQ_COK)
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo(s_pktCfg.p_rxBuf, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE)

                if(osal_memcmp(s_pktCfg.p_rxBuf+10,&s_rf_netid,2))  //compare target_ID in data with local netID
                #else
                if(osal_memcmp(s_pktCfg.p_rxBuf+2,peer_addr,6))    //20221020 ZONG: move addr check here, and check remote addr instead of local addr
                #endif
                {
                    #if(DEF_PHYPLUS_AUTOACK_SUPPORT == PHYPLUS_AUTOACK_ENABLE)       //SRX:RX->TXACK

                    if(!(PHYPLUS_GET_ACK_BIT(s_pktCfg.p_rxBuf[0])) && (PHYPLUS_GET_NEEDACK_BIT(s_pktCfg.p_rxBuf[0])))
                    {
                        s_rf_crc=0;
                        osal_memcpy((uint8_t*)(&s_rf_crc),(uint8_t*)(&s_pktCfg.p_rxBuf[pktLen]),3);
                        phy_rf_txack();
                        LOG_DEBUG("%08x\n",s_rf_crc);
                    }

                    #endif
                    phy_rx_data_check();
                }

                #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE)
                else if((s_rf_netid != 0) && (s_pktCfg.p_rxBuf[10] == 0) && (s_pktCfg.p_rxBuf[11] == PHYPLUS_GET_GROUPID(s_rf_netid)))          //PHYPLUS_IS_GROUP_ADV((s_pktCfg.p_rxBuf+10),s_rf_netid)
                {
                    s_rf_crc=0;
                    osal_memcpy((uint8_t*)(&s_rf_crc),(uint8_t*)(&s_pktCfg.p_rxBuf[pktLen]),3);

                    if(s_rf_crc != s_rf_lastcrc)
                    {
                        osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DATA_PROCESS_EVT);
                    }

                    s_rf_lastcrc =  s_rf_crc;
                    s_phyDbg.rx_data_cnt++;
                }                    //20221020 ZONG: add NET_ID check for mesh mode

                #endif
                else
                {
                    // LOG("wrong device:\n");
                    // my_dump_byte((uint8_t*)s_pktCfg.p_rxBuf+10, 2);
                    // dbg_printf("%02x %02x\n",s_rf_netid>>8,s_rf_netid&0xFF);
                    // my_dump_byte(&pData[2], 6);
                }
            }
        }
        #endif
        //osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DONE_EVT);
    }

    #endif
    else if(mode == LL_HW_MODE_TRX  &&
            (s_phy.Status == PHYPLUS_RFPHY_TRX_ONLY)
           )
    {
        rf_phy_get_pktFoot(&phyRssi,&phyFoff,&phyCarrSens);
        s_phyDbg.tx_data_cnt++;
        #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
        //if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        {
            if(0==(irq_status & LIRQ_RTO))
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo_pplus(s_pktCfg.p_rxBuf, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                phy_rx_data_check();
            }
        }
        #else
        {
            if(irq_status & LIRQ_COK)
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo(s_pktCfg.p_rxBuf, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE)

                if(osal_memcmp(s_pktCfg.p_rxBuf+10,&s_rf_netid,2))  //compare target_ID in data with local netID
                #else
                if(osal_memcmp(s_pktCfg.p_rxBuf+2,peer_addr,6))         //20221020 ZONG: move addr check here, and check remote addr instead of local addr
                #endif
                {
                    phy_rx_data_check();
                }
                else
                {
                    LOG("wrong device:\n");
                    my_dump_byte((uint8_t*)s_pktCfg.p_rxBuf+10, 2);
                    #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE)
                    dbg_printf("%02x %02x\n",s_rf_netid>>8,s_rf_netid&0xFF);
                    #endif
                }
            }
        }
        #endif
        //osal_set_event(PhyPlusPhy_TaskID,PPP_TRX_DONE_EVT);
    }

    // post ISR process
    if(llWaitingIrq!=TRUE)
    {
        ll_hw_clr_irq();
        PhyPlusPhy_Set_BLE_IRQHandler();
    }

    HAL_EXIT_CRITICAL_SECTION();
    phy_rf_schedule();
}

/*********************************************************************
    @fn      PhyPlusPhy_Init

    @brief   Initialization function for the Simple BLE Peripheral App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void PhyPlusPhy_Init(uint8 task_id)
{
    PhyPlusPhy_TaskID = task_id;
    //set phy irq handeler
    //JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&PLUSPHY_IRQHandler;
    BLE_IRQHandler_Restore = JUMP_FUNCTION(V4_IRQ_HANDLER) ;
    osal_memset(adv_buffer,0,32);
    //phy pktfmt config
    s_phy.enAutoAck     =   DEF_PHYPLUS_AUTOACK_SUPPORT; // for NRF_MODE_ENHANCE_SHOCKBURST need enable autoack
    s_phy.Status        =   PHYPLUS_RFPHY_IDLE;
    s_chanHop.curIdx    =   0;
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    uint8_t nrf_addr[5] = {0xc7,0x34,0x89,0x71,0xd6};//NRF ADDR Format
    uint8_t nrf_addrLen = NRF_ADDR_LEN_5BYTE;
    uint8_t nrf_pudLen  = 32;
    uint8_t nrf_mode;

    if(s_phy.enAutoAck  ==   PHYPLUS_AUTOACK_ENABLE)
    {
        nrf_mode    = NRF_MODE_ENHANCE_SHOCKBURST;//NRF_MODE_ENHANCE_SHOCKBURST
    }
    else
    {
        nrf_mode    = NRF_MODE_SHOCKBURST;//NRF_MODE_ENHANCE_SHOCKBURST
    }

    uint8_t nrf_crcByte = CRC_ITU_16_LEN_2BYTE;//CRC_ITU_16_LEN_2BYTE
    //packet config setup
    s_pktCfg.crcFmt     =   LL_HW_CRC_NULL;//LL_HW_CRC_BLE_FMT;LL_HW_CRC_NULL
    s_pktCfg.crcSeed    =   DEFAULT_CRC_SEED;
    s_pktCfg.wtSeed     =   WHITEN_OFF;//WHITEN_SEED_CH37;//DEFAULT_WHITEN_SEED;
    s_pktCfg.pktFmt     =   PHYPLUS_PKT_FMT_1M;
    s_pktCfg.syncWord   =   nrf_pkt_init(nrf_addr, nrf_addrLen,nrf_pudLen,nrf_crcByte,nrf_mode);
    s_pktCfg.pduLen     =   nrf_pudLen+6;// (1+2+2+1);
    #else
    // read flash driectly becasue HW has do the address mapping for read Flash operation
    uint8_t p[6]= {0x01,0x2,3,4,5,6};
    s_pubAddr[3] = p[0];
    s_pubAddr[2] = p[1];
    s_pubAddr[1] = p[2];
    s_pubAddr[0] = p[3];
    s_pubAddr[5] = p[4];
    s_pubAddr[4] = p[5];
    s_pktCfg.syncWord   =   DEFAULT_SYNCWORD;
    s_pktCfg.pduLen     =   31+6;// (1+2+2+1);
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT == PHYPLUS_AUTOACK_DISABLE)
    advHead[0]= 0x02;
    #else
    advHead[0]= 0x04;
    #endif
    advHead[1]= s_pktCfg.pduLen ;
    //packet config setup
    s_pktCfg.crcFmt     =   LL_HW_CRC_BLE_FMT;//LL_HW_CRC_BLE_FMT;LL_HW_CRC_NULL
    s_pktCfg.crcSeed    =   DEFAULT_CRC_SEED;
    s_pktCfg.wtSeed     =   s_chanHop.wt_map[0];//WHITEN_SEED_CH37;//DEFAULT_WHITEN_SEED;
    s_pktCfg.pktFmt     =   PHYPLUS_PKT_FMT_1M;
    #endif
    s_phySch.txMargin   =   1500;//us
    s_phySch.rxMargin   =   2500;//us
    s_phy.rxOnlyTO      =   1000*1000;//us
    s_phy.rfChn         =   s_chanHop.chan_map[0];// RF Freq  = 2400 + rfchn    //BLE_ADV_CHN37
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT == PHYPLUS_AUTOACK_DISABLE)
    {
        s_phy.rxAckTO       =   0;//us, Set to 0 switch to STX instead of TRX mode for send data
        s_phy.reTxMax       =   0;//tx retry count Max, Set to 0 to turn off Auto-ReTx
        s_phy.reTxDly       =   0;//auto retry delay
    }
    #else
    {
        s_phy.rxAckTO       =   500;//us, Set to 0 switch to STX instead of TRX mode for send data
        s_phy.reTxMax       =   5;//tx retry count Max
        s_phy.reTxDly       =   500;//auto retry delay
    }
    #endif
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    s_pktCfg.p_txBuf    =   &(phyBufTx[4]);
    s_pktCfg.p_rxBuf    =   &(phyBufRx[4]);
    LOG("[NRF_CFG] :v%08x  Mode %d  pdu len %d CRCB %d addrLen %d Addr:",
        nrfConfig.version,nrfConfig.mode,nrfTxBuf.pldLen,nrfConfig.crcByte,nrfConfig.addrLen);
    my_dump_byte(nrfTxBuf.addr,nrfConfig.addrLen);
    #else
    s_pktCfg.p_txBuf    =   &(phyBufTx[0]);
    s_pktCfg.p_rxBuf    =   &(phyBufRx[0]);
    LOG("NRF Disabled\n");
    #endif
//    gpio_dir(P11, GPIO_INPUT);
//    gpio_pull_set(P11, GPIO_PULL_UP);
//    if(0==gpio_read(P11))
//    {
//        LOG("START PPP_PERIODIC_TX_EVT\n");
//        VOID osal_start_timerEx(PhyPlusPhy_TaskID, PPP_PERIODIC_TX_EVT, 1000);
//    }
//    else
//    {
//        LOG("START PPP_PERIODIC_RX_EVT\n");
//        VOID osal_start_timerEx(PhyPlusPhy_TaskID, PPP_PERIODIC_RX_EVT, 100);
//    }
    LOG("[PHY] init done %d rfchn%d AutoAck %d SW[%8x] CRC[%d %8x] WT[%2x]\n"\
        ,s_phy.Status,s_phy.rfChn,s_phy.enAutoAck,s_pktCfg.syncWord,s_pktCfg.crcFmt, s_pktCfg.crcSeed,s_pktCfg.wtSeed);
}
static void show_phy_debug_info(void)
{
    LOG("[PHY DBG]st %02x [TX]dat %d ack %d rty %d [ackT]%d %d [RX]dat %d ack%d crc %d ign %d \n"
        ,s_phy.Status,s_phyDbg.tx_data_cnt,s_phyDbg.tx_ack_cnt,s_phyDbg.tx_retry_cnt
        ,s_phyDbg.rx_txack_t0,s_phyDbg.rx_txack_t1
        ,s_phyDbg.rx_data_cnt,s_phyDbg.rx_txack_cnt,s_phyDbg.rx_crc_err,s_phyDbg.rx_data_ign);
}

#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
static void process_rx_done_evt(void)
{
    //LOG("rx done evt\n");
//    osal_set_event(PhyPlusPhy_TaskID,PPP_PERIODIC_RX_EVT);
    show_phy_debug_info();
}
#endif

#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)
static void process_tx_done_evt(void)
{
    LOG("tx done evt\n");
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT == PHYPLUS_AUTOACK_ENABLE)
    phy_comm_evt_t phy_pdu_cb;

    if(phy_data_cbfunc != NULL)
    {
        phy_pdu_cb.type = PHYPLUS_STX_DONE_TYPE;
        phy_pdu_cb.len = NULL;
        phy_pdu_cb.rssi = NULL;
        phy_pdu_cb.data = NULL;
        phy_data_cbfunc(&phy_pdu_cb);
    }

    #endif
}
static void process_trx_done_evt(void)
{
    phy_comm_evt_t phy_pdu_cb;

    if(s_phy.txAck)
    {
        // adv_buffer[0]+=1;
        LOG("[TX OK]\n");

        if(phy_data_cbfunc != NULL)
        {
            #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
            phy_pdu_cb.type = PHYPLUS_SMART_NRF_TYPE;
            phy_pdu_cb.rssi = phyRssi;
            phy_pdu_cb.len = nrfRxBuf.pldLen;
            phy_pdu_cb.data = nrfRxBuf.pdu;
            phy_data_cbfunc(&phy_pdu_cb);
            #else
            phy_pdu_cb.type = s_pktCfg.p_rxBuf[0];
            phy_pdu_cb.len = s_pktCfg.p_rxBuf[1];
            phy_pdu_cb.rssi = phyRssi;
            phy_pdu_cb.data = &s_pktCfg.p_rxBuf[2];
            phy_data_cbfunc(&phy_pdu_cb);
            #endif
        }
    }
    else
    {
        LOG("[TX Fail]\n");

        if(phy_data_cbfunc != NULL)
        {
            phy_pdu_cb.type = NULL;
            phy_pdu_cb.len = NULL;
            phy_pdu_cb.rssi = NULL;
            phy_pdu_cb.data = NULL;
            phy_data_cbfunc(&phy_pdu_cb);
        }

        //osal_start_timerEx(PhyPlusPhy_TaskID,PPP_PERIODIC_TX_EVT,10);
    }

    LOG_DEBUG("trx done evt reTry %d reMax %d\n",s_phy.reTxCnt,s_phy.reTxMax);
    s_phy.reTxCnt = 0;
    s_phy.txAck   = 0;
    show_phy_debug_info();
}
#endif

uint8_t phy_nrf_send_data(uint8_t* din, uint8_t dLen)
{
    if(PHYPLUS_RFPHY_IDLE !=s_phy.Status)
        return PPlus_ERR_BUSY;

    if(s_phy.enAutoAck== PHYPLUS_AUTOACK_DISABLE)
        s_phy.Status = PHYPLUS_RFPHY_TX_ONLY;

    #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE)
    else if(s_rf_target_netid != 0 && PHYPLUS_GET_DEVICEID(s_rf_target_netid) == 0)
    {
        s_phy.Status = PHYPLUS_RFPHY_TX_ONLY;
    }

    #endif
    else
        s_phy.Status = PHYPLUS_RFPHY_TRX_ONLY;

    s_phy.reTxCnt = 0;
    s_phy.txAck   = 0;
    phy_adv_data_update(din, dLen);
    s_txT0 = read_current_fine_time();
    uint8_t ret = phy_rf_process_tsmt();
    return ret;
}
#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)
uint8_t phy_rf_stop_tx(void)
{
    if( PHYPLUS_RFPHY_TX_ONLY ==s_phy.Status ||
            PHYPLUS_RFPHY_TX_PENDING ==s_phy.Status ||
            PHYPLUS_RFPHY_TRX_ONLY ==s_phy.Status   ||
            PHYPLUS_RFPHY_TRX_PENDING ==s_phy.Status  )
    {
        s_phy.txIntv = 0;
        s_phy.Status = PHYPLUS_RFPHY_IDLE;
        osal_stop_timerEx(PhyPlusPhy_TaskID,PPP_TX_PENDING_PROCESS_EVT);
        return PPlus_SUCCESS;
    }
    else
        return PPlus_ERR_INVALID_STATE;
}

uint8_t phy_rf_start_tx(uint8_t* din, uint8_t dLen, uint32_t txintv, uint16_t targetnetid)
{
    if(PHYPLUS_RFPHY_IDLE !=s_phy.Status)
        return PPlus_ERR_BUSY;

    s_phy.txIntv = txintv;
    #if(DEF_PHYPLUS_MESH_SUPPORT == PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT == PHYPLUS_NRF_DISABLE)
    s_rf_target_netid = targetnetid;
    #endif
    // osal_set_event(PhyPlusPhy_TaskID,PPP_PERIODIC_TX_EVT);
    uint8_t ret = phy_nrf_send_data(din, dLen);
    return ret;
}
#endif
#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
uint8_t phy_rf_stop_rx(void)
{
    if( PHYPLUS_RFPHY_RX_ONLY ==s_phy.Status ||
            PHYPLUS_RFPHY_RX_PENDING ==s_phy.Status)
    {
        s_phy.rxOnlyTO=0;
        s_chanHop.curIdx = 0;

        if(PHYPLUS_RFPHY_RX_ONLY ==s_phy.Status)
            phy_hw_stop();
        else
        {
            s_phy.Status = PHYPLUS_RFPHY_IDLE;
            osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DONE_EVT);
        }

        return PPlus_SUCCESS;
    }
    else
        return PPlus_ERR_INVALID_STATE;
}

uint8_t phy_rf_start_rx(uint32 rxTimeOut)
{
    if(PHYPLUS_RFPHY_IDLE !=s_phy.Status)
        return PPlus_ERR_BUSY;

    s_phy.rxOnlyTO = rxTimeOut;
    s_phy.Status = PHYPLUS_RFPHY_RX_ONLY;
    s_phy.rxScanT0 = read_current_fine_time();
    phy_rf_process_recv();
    return PPlus_SUCCESS;
}
#endif
uint8_t phy_rf_get_current_status(void)
{
    return s_phy.Status;
}

/*********************************************************************
    @fn      PhyPlusPhy_ProcessEvent

    @brief   Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 PhyPlusPhy_ProcessEvent(uint8 task_id, uint16 events)
{
    VOID task_id;
    #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)

    if(events & PPP_RX_PENDING_PROCESS_EVT)
    {
        phy_rf_process_recv();
        return(events ^ PPP_RX_PENDING_PROCESS_EVT);
    }

    if(events & PPP_RX_DONE_EVT)
    {
        process_rx_done_evt();
        return(events ^ PPP_RX_DONE_EVT);
    }

    #endif
    #if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)

    if(events & PPP_TX_PENDING_PROCESS_EVT)
    {
        phy_rf_process_tsmt();
        return(events ^ PPP_TX_PENDING_PROCESS_EVT);
    }

    if(events & PPP_TX_DONE_EVT)
    {
        process_tx_done_evt();
        return(events ^ PPP_TX_DONE_EVT);
    }

    if(events & PPP_TRX_DONE_EVT)
    {
        process_trx_done_evt();
        return(events ^ PPP_TRX_DONE_EVT);
    }

    #endif

    if(events & PPP_RX_DATA_PROCESS_EVT)            //20220711  change the sequence to try to finish the data process before the next rx
    {
        phy_rx_data_process();
        return(events ^ PPP_RX_DATA_PROCESS_EVT);
    }

    return 0;
}
uint8_t phy_adv_data_update(uint8_t* din, uint8_t dLen)
{
    #if(DEF_PHYPLUS_MESH_SUPPORT==PHYPLUS_MESH_ENABLE && DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_DISABLE)
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    {
        if(dLen > 28)
        {
            LOG("Invalid DataLength!\n");
            return PPlus_ERR_INVALID_LENGTH;
        }
    }
    #else
    {
        if(dLen > 27)
        {
            LOG("Invalid DataLength!\n");
            return PPlus_ERR_INVALID_LENGTH;
        }
    }
    #endif
    osal_memset(adv_buffer,0,32);           //reset adv_buffer
    adv_buffer[0] = (uint8_t)(s_rf_netid & 0x00FF);
    adv_buffer[1] = (uint8_t)((s_rf_netid & 0xFF00) >> 8);
    adv_buffer[2] = (uint8_t)(s_rf_target_netid & 0x00FF);
    adv_buffer[3] = (uint8_t)((s_rf_target_netid & 0xFF00) >> 8);
    osal_memcpy(adv_buffer+4,din,dLen);      //copy payload
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    {
        nrf_pkt_gen(adv_buffer, dLen+4, phyBufTx);
    }
    #else
    {
        phy_tx_buf_updata(s_pubAddr,advHead,adv_buffer,dLen+4);
    }
    #endif
    return PPlus_SUCCESS;
    #else
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    {
        if(dLen > 32)
        {
            LOG("Invalid DataLength!\n");
            return PPlus_ERR_INVALID_LENGTH;
        }
    }
    #else
    {
        if(dLen > 31)
        {
            LOG("Invalid DataLength!\n");
            return PPlus_ERR_INVALID_LENGTH;
        }
    }
    #endif
    osal_memset(adv_buffer,0,32);           //reset adv_buffer
    osal_memcpy(adv_buffer,din,dLen);      //copy payload
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    {
        nrf_pkt_gen(adv_buffer, dLen, phyBufTx);
    }
    #else
    {
        phy_tx_buf_updata(s_pubAddr,advHead,adv_buffer,dLen);
    }
    #endif
    return PPlus_SUCCESS;
    #endif
}

void phy_adv_opcode_update(uint8_t opcode)
{
    advHead[0] = (advHead[0] & 0x0F) | (opcode << 4);
}


uint8_t phy_cbfunc_regist(int8_t cbfunc_type, uint8_t (*phy_comm_cb_t)(phy_comm_evt_t* pev))
{
    if(cbfunc_type == PHY_DATA_CB)
    {
        phy_data_cbfunc = phy_comm_cb_t;
    }
    else if(cbfunc_type == PHY_OPCODE_CB)
    {
        phy_opcode_cbfunc = phy_comm_cb_t;
    }
    else
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    return PPlus_SUCCESS;
}

uint8_t phy_update_syncword(uint32_t syncword)
{
    s_pktCfg.syncWord = syncword;
    return PPlus_SUCCESS;
}

uint8_t phy_update_chmap(uint8_t chnum, uint8_t* chmap, uint8_t* wtmap)
{
    if(chnum != PHYPLUS_CHAN_MAP_NUM)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    osal_memcpy(s_chanHop.chan_map, chmap, chnum);
    osal_memcpy(s_chanHop.wt_map, wtmap, chnum);
    return PPlus_SUCCESS;
}

void phy_set_tx_maxtime(uint32_t txdura)
{
    s_phy.txDuration = txdura; //μs
}

/*********************************************************************
*********************************************************************/
