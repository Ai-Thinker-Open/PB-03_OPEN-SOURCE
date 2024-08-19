/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_MS_SENSOR_API_
#define _H_MS_SENSOR_API_


/* --------------------------------------------- Header File Inclusion */
#include "MS_access_api.h"


/* --------------------------------------------- Global Definitions */
/* --------------------------------------------- Data Types/ Structures */
typedef API_RESULT (* MS_SENSOR_SERVER_CB)
(
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    MS_ACCESS_MODEL_EXT_PARAMS*          ext_params

) DECL_REENTRANT;

typedef API_RESULT (* MS_SENSOR_CLIENT_CB)
(
    MS_ACCESS_MODEL_HANDLE* handle,
    UINT32                   opcode,
    UCHAR*                   data_param,
    UINT16                   data_len
) DECL_REENTRANT;

typedef struct MS_sensor_cadence_get_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16 property_id;

} MS_SENSOR_CADENCE_GET_STRUCT;

typedef struct MS_sensor_cadence_set_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16 property_id;

    /** Divisor for the Publish Period */
    UCHAR  fast_cadence_period_divisor;

    /** Defines the unit and format of the Status Trigger Delta fields */
    UCHAR  status_trigger_type;

    /** Delta down value that triggers a status message */
    UCHAR* status_trigger_delta_down;
    UINT16 status_trigger_delta_down_len;

    /** Delta up value that triggers a status message */
    UCHAR* status_trigger_delta_up;
    UINT16 status_trigger_delta_up_len;

    /** Minimum interval between two consecutive Status messages */
    UCHAR  status_min_interval;

    /** Low value for the fast cadence range */
    UCHAR* fast_cadence_low;
    UINT16 fast_cadence_low_len;

    /** High value for the fast cadence range */
    UCHAR* fast_cadence_high;
    UINT16 fast_cadence_high_len;

} MS_SENSOR_CADENCE_SET_STRUCT;

typedef struct MS_sensor_column_get_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16 property_id;

    /** Raw value representing the left corner of a column on the X axis */
    UCHAR* raw_value_x;
    UINT16 raw_value_x_len;

} MS_SENSOR_COLUMN_GET_STRUCT;

typedef struct MS_sensor_descriptor_get_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16  property_id;

    UINT8   optional_fields_present;

} MS_SENSOR_DESCRIPTOR_GET_STRUCT;

typedef struct MS_sensor_get_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16  property_id;

    UINT8   optional_fields_present;

} MS_SENSOR_GET_STRUCT;

typedef struct MS_sensor_series_get_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16  property_id;

    /** Raw value representing the left corner of a column on the X axis */
    UCHAR*  raw_value_x1;
    UINT16  raw_value_x1_len;

    /** Raw value representing the height of the column on the Y axis */
    UCHAR*  raw_value_x2;
    UINT16  raw_value_x2_len;

    UINT8   optional_fields_present;

} MS_SENSOR_SERIES_GET_STRUCT;

typedef struct MS_sensor_setting_get_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16  sensor_property_id;

    /** Property ID of a setting within a sensor */
    UINT16  sensor_setting_property_id;

} MS_SENSOR_SETTING_GET_STRUCT;

typedef struct MS_sensor_setting_set_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16  sensor_property_id;

    /** Property ID of a setting within a sensor */
    UINT16  sensor_setting_property_id;

    /** Raw value of a setting within a sensor */
    UCHAR*  sensor_setting_raw;
    UINT16  sensor_setting_raw_len;

} MS_SENSOR_SETTING_SET_STRUCT;

typedef struct MS_sensor_settings_set_struct
{
    /**
        Sensor Property ID field is a 2-octet value referencing a device property
        that describes the meaning and the format of data reported by a sensor
    */
    UINT16  sensor_property_id;

} MS_SENSOR_SETTINGS_GET_STRUCT;


/* --------------------------------------------- Function */
API_RESULT MS_sensor_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     time_model_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     time_setup_model_handle,
    /* IN */    MS_SENSOR_SERVER_CB           appl_cb
);

API_RESULT MS_sensor_server_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
);
API_RESULT MS_sensor_client_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_SENSOR_CLIENT_CB appl_cb
);

API_RESULT MS_sensor_client_get_model_handle
(
    /* OUT */ MS_ACCESS_MODEL_HANDLE*   model_handle
);

API_RESULT MS_sensor_client_send_reliable_pdu
(
    /* IN */ UINT32    req_opcode,
    /* IN */ void*     param,
    /* IN */ UINT32    rsp_opcode
);

#define MS_sensor_cadence_get(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_CADENCE_GET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_CADENCE_STATUS_OPCODE\
    )

#define MS_sensor_cadence_set(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_CADENCE_SET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_CADENCE_STATUS_OPCODE\
    )

#define MS_sensor_cadence_set_unacknowledged(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_CADENCE_SET_UNACKNOWLEDGED_OPCODE,\
     param,\
     0xFFFFFFFF\
    )

#define MS_sensor_column_get(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_COLUMN_GET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_COLUMN_STATUS_OPCODE\
    )

#define MS_sensor_descriptor_get(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_DESCRIPTOR_GET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_DESCRIPTOR_STATUS_OPCODE\
    )

#define MS_sensor_get(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_GET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_STATUS_OPCODE\
    )

#define MS_sensor_series_get(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_SERIES_GET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_SERIES_STATUS_OPCODE\
    )

#define MS_sensor_setting_get(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_SETTING_GET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_SETTING_STATUS_OPCODE\
    )

#define MS_sensor_setting_set(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_SETTING_SET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_SETTING_STATUS_OPCODE\
    )

#define MS_sensor_setting_set_unacknowledged(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_SETTING_SET_UNACKNOWLEDGED_OPCODE,\
     param,\
     0xFFFFFFFF\
    )

#define MS_sensor_settings_get(param) \
    MS_sensor_client_send_reliable_pdu \
    (\
     MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE,\
     param,\
     MS_ACCESS_SENSOR_SETTINGS_STATUS_OPCODE\
    )

#endif /*_H_MS_TIME_API_ */
