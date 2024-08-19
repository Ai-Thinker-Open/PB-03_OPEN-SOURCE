/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_MS_SCHEDULER_API_
#define _H_MS_SCHEDULER_API_


/* --------------------------------------------- Header File Inclusion */
#include "MS_access_api.h"


/* --------------------------------------------- Global Definitions */
/* --------------------------------------------- Data Types/ Structures */
typedef API_RESULT (* MS_SCHEDULER_SERVER_CB)
(
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    MS_ACCESS_MODEL_EXT_PARAMS*          ext_params

) DECL_REENTRANT;

typedef API_RESULT (* MS_SCHEDULER_CLIENT_CB)
(
    MS_ACCESS_MODEL_HANDLE* handle,
    UINT32                   opcode,
    UCHAR*                   data_param,
    UINT16                   data_len
) DECL_REENTRANT;

typedef struct MS_scheduler_action_set_struct
{
    /** Index of the Schedule Register entry */
    UCHAR  index;

    /** Scheduled year for the action */
    UCHAR  year;

    /** Scheduled month for the action */
    UINT16 month;

    /** Scheduled day of the month for the action */
    UCHAR  day;

    /** Scheduled hour for the action */
    UCHAR  hour;

    /** Scheduled minute for the action */
    UCHAR  minute;

    /** Scheduled second for the action */
    UCHAR  second;

    /** Schedule days of the week for the action */
    UCHAR  dayofweek;

    /** Action to be performed at the scheduled time */
    UCHAR  action;

    /** Transition time for this action */
    UCHAR  transition_time;

    /** Scene number to be used for some actions */
    UINT16 scene_number;
} MS_SCHEDULER_ACTION_SET_STRUCT;

typedef struct MS_scheduler_action_get_struct
{
    /** Index of the Schedule Register entry */
    UCHAR  index;
} MS_SCHEDULER_ACTION_GET_STRUCT;

/* --------------------------------------------- Function */
API_RESULT MS_scheduler_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     time_model_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     time_setup_model_handle,
    /* IN */    MS_SCHEDULER_SERVER_CB           appl_cb
);

API_RESULT MS_scheduler_server_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
);
API_RESULT MS_scheduler_client_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_SCHEDULER_CLIENT_CB appl_cb
);

API_RESULT MS_scheduler_client_get_model_handle
(
    /* OUT */ MS_ACCESS_MODEL_HANDLE*   model_handle
);

API_RESULT MS_scheduler_client_send_reliable_pdu
(
    /* IN */ UINT32    req_opcode,
    /* IN */ void*     param,
    /* IN */ UINT32    rsp_opcode
);

#define MS_scheduler_action_get(param) \
    MS_scheduler_client_send_reliable_pdu \
    (\
     MS_ACCESS_SCHEDULER_ACTION_GET_OPCODE,\
     param,\
     MS_ACCESS_SCHEDULER_ACTION_STATUS_OPCODE\
    )

#define MS_scheduler_action_set(param) \
    MS_scheduler_client_send_reliable_pdu \
    (\
     MS_ACCESS_SCHEDULER_ACTION_SET_OPCODE,\
     param,\
     MS_ACCESS_SCHEDULER_ACTION_STATUS_OPCODE\
    )

#define MS_scheduler_action_set_unacknowledged(param) \
    MS_scheduler_client_send_reliable_pdu \
    (\
     MS_ACCESS_SCHEDULER_ACTION_SET_UNACKNOWLEDGED_OPCODE,\
     param,\
     0xFFFFFFFF\
    )

#define MS_scheduler_get() \
    MS_scheduler_client_send_reliable_pdu \
    (\
     MS_ACCESS_SCHEDULER_GET_OPCODE,\
     NULL,\
     MS_ACCESS_SCHEDULER_STATUS_OPCODE\
    )


#endif /*_H_MS_SCHEDULER_API_ */
