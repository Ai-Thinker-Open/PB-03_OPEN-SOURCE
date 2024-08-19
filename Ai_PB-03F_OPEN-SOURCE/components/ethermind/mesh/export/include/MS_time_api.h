/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_MS_TIME_API_
#define _H_MS_TIME_API_


/* --------------------------------------------- Header File Inclusion */
#include "MS_access_api.h"


/* --------------------------------------------- Global Definitions */
/* --------------------------------------------- Data Types/ Structures */
typedef API_RESULT (* MS_TIME_SERVER_CB)
(
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    MS_ACCESS_MODEL_EXT_PARAMS*          ext_params

) DECL_REENTRANT;

typedef API_RESULT (* MS_TIME_CLIENT_CB)
(
    MS_ACCESS_MODEL_HANDLE* handle,
    UINT32                   opcode,
    UCHAR*                   data_param,
    UINT16                   data_len
) DECL_REENTRANT;
typedef struct MS_time_struct
{
    UCHAR  tai_seconds[5];

    UCHAR  subsecond;

    UCHAR  uncertainty;

    UCHAR  time_authority;

    UINT16 tai_utc_delta;

    UCHAR  time_zone_offset;

} MS_TIME_STRUCT;

typedef struct MS_time_zone_set_struct
{
    UCHAR  time_zone_offset_new;

    UCHAR  tai_of_zone_change[5];

} MS_TIME_ZONE_SET_STRUCT;

typedef struct MS_time_zone_status_struct
{
    UCHAR  time_zone_offset_current;

    UCHAR  time_zone_offset_new;

    UCHAR  tai_of_zone_change[5];

} MS_TIME_ZONE_STATUS_STRUCT;

typedef struct MS_tai_utc_delta_set_struct
{
    UINT16 tai_utc_delta_new;

    UCHAR  padding;

    UCHAR  tai_of_delta_change[5];

} MS_TAI_UTC_DELTA_SET_STRUCT;

typedef struct MS_tai_utc_delta_status_struct
{
    UINT16 tai_utc_delta_current;

    UCHAR  padding_1;

    UINT16 tai_utc_delta_new;

    UCHAR  padding_2;

    UCHAR  tai_of_delta_change[5];

} MS_TAI_UTC_DELTA_STATUS_STRUCT;

typedef struct MS_time_role_struct
{
    UCHAR  time_role;

} MS_TIME_ROLE_STRUCT;

/* --------------------------------------------- Function */
API_RESULT MS_time_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     time_model_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     time_setup_model_handle,
    /* IN */    MS_TIME_SERVER_CB           appl_cb
);

API_RESULT MS_time_server_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
);
API_RESULT MS_time_client_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_TIME_CLIENT_CB appl_cb
);

API_RESULT MS_time_client_get_model_handle
(
    /* OUT */ MS_ACCESS_MODEL_HANDLE*   model_handle
);

API_RESULT MS_time_client_send_reliable_pdu
(
    /* IN */ UINT32    req_opcode,
    /* IN */ void*     param,
    /* IN */ UINT32    rsp_opcode
);

#define MS_time_set(param) \
    MS_time_client_send_reliable_pdu \
    (\
     MS_ACCESS_TIME_SET_OPCODE,\
     param,\
     MS_ACCESS_TIME_STATUS_OPCODE\
    )

#define MS_time_zone_get() \
    MS_time_client_send_reliable_pdu \
    (\
     MS_ACCESS_TIME_ZONE_GET_OPCODE,\
     NULL,\
     MS_ACCESS_TIME_ZONE_STATUS_OPCODE\
    )

#define MS_tai_utc_delta_set(param) \
    MS_time_client_send_reliable_pdu \
    (\
     MS_ACCESS_TAI_UTC_DELTA_SET_OPCODE,\
     param,\
     MS_ACCESS_TAI_UTC_DELTA_STATUS_OPCODE\
    )

#define MS_time_role_set(param) \
    MS_time_client_send_reliable_pdu \
    (\
     MS_ACCESS_TIME_ROLE_SET_OPCODE,\
     param,\
     MS_ACCESS_TIME_ROLE_STATUS_OPCODE\
    )

#define MS_tai_utc_delta_get() \
    MS_time_client_send_reliable_pdu \
    (\
     MS_ACCESS_TAI_UTC_DELTA_GET_OPCODE,\
     NULL,\
     MS_ACCESS_TAI_UTC_DELTA_STATUS_OPCODE\
    )

#define MS_time_zone_set(param) \
    MS_time_client_send_reliable_pdu \
    (\
     MS_ACCESS_TIME_ZONE_SET_OPCODE,\
     param,\
     MS_ACCESS_TIME_ZONE_STATUS_OPCODE\
    )

#define MS_time_role_get() \
    MS_time_client_send_reliable_pdu \
    (\
     MS_ACCESS_TIME_ROLE_GET_OPCODE,\
     NULL,\
     MS_ACCESS_TIME_ROLE_STATUS_OPCODE\
    )

#define MS_time_get() \
    MS_time_client_send_reliable_pdu \
    (\
     MS_ACCESS_TIME_GET_OPCODE,\
     NULL,\
     MS_ACCESS_TIME_STATUS_OPCODE\
    )

#endif /*_H_MS_TIME_API_ */
