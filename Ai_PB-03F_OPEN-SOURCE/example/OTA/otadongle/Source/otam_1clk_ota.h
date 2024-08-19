
#ifndef __OTAM_1CLK_OTA_H
#define __OTAM_1CLK_OTA_H

enum
{
    ONECLK_ST_APP_IDLE = 0,
    ONECLK_ST_APP_CONNECTING,
    ONECLK_ST_APP_INFO,
    ONECLK_ST_APP_MODE,
    ONECLK_ST_OTA_IDLE,
    ONECLK_ST_OTA_CONNECTING,
    ONECLK_ST_OTA_CONN_UPDATE,
    ONECLK_ST_OTAING,
    ONECLK_ST_OTA_FINISH,
    ONECLK_ST_ERROR,
};


enum
{
    ONECLK_EVT_CONNECTED,
    ONECLK_EVT_CONNECTED_OTA,
    ONECLK_EVT_TERMINATED,
    ONECLK_EVT_SCAN_RESULT,
    ONECLK_EVT_TIMER,
    ONECLK_EVT_OTA_FINISHED,
    ONECLK_EVT_WRITE_RSP,
    ONECLK_EVT_PARAM_UPDATE
};

typedef struct
{
    uint8_t ev;
    union
    {
        uint8_t mac[6];
    } param;
} oneclk_evt_t;


uint8_t* pull_from_maclist(void);
void otam_oneclick_evt(oneclk_evt_t* pev);

uint8_t* otam_oneclick_getmac(void);
uint8_t is_sencond_link(void);
void otam_oneclick_ota(void);
void otam_oneclick_init(void);

#endif

