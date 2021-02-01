/***************************************************************************//**
* \file   <wiced_bt_hfp_hf.h>
*
* \brief
* 	Contains Hand Free Profile - Hands Free Device APIs and definitions.
*
*//*****************************************************************************
* Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_types.h"
#include "wiced_result.h"

/******************************************************************************
*
* \addtogroup  wicedbt_hfp          Hands Free Profile (HFP)
* \ingroup wicedbt_av
*
* This section talks about the API's required to add Hands Free Profile to the user application.
* The typical use case for this profile is, a headset wirelessly connected to a mobile phone
* enabling the user to perform telephone functions through the headset .
* This library can also connect to an AG supporting HSP only, if the application using this library supports HSP in the SDP.
*
* @{
*
******************************************************************************/

/******************************************************************************
*                      Macros
******************************************************************************/
#define WICED_BT_HFP_HF_CALLER_NUMBER_MAX_LENGTH        32      /* Maximum length of caller number */
#define WICED_BT_HFP_HF_AT_CMD_RESULT_CODE_MAX_LENGTH   256     /* Maximum length of AT command result code */

/* Maximum number of HFP HF connections supported */
#ifndef WICED_BT_HFP_HF_MAX_CONN
#define WICED_BT_HFP_HF_MAX_CONN 2
#endif

/* SDP SupportedFeatures attribute bit mapping for HF.
   Table 5.2 of Hand-Free Profile 1.7.1 */
#define WICED_BT_HFP_HF_SDP_FEATURE_ECNR            0x0001  /* EC and/or NR function (yes:1, no:0) */
#define WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING    0x0002  /* Call waiting or three-way calling (yes:1, no:0) */
#define WICED_BT_HFP_HF_SDP_FEATURE_CLIP            0x0004  /* CLI presentation capability (yes:1, no:0) */
#define WICED_BT_HFP_HF_SDP_FEATURE_VRECG           0x0008  /* Voice recognition activation (yes:1, no:0) */
#define WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL 0x0010  /* Remote volume control (yes:1, no:0) */
#define WICED_BT_HFP_HF_SDP_FEATURE_WIDEBAND_SPEECH 0x0020  /* Wide band speech (yes:1, no:0) */

/******************************************************************************
*                    Constants
******************************************************************************/
/* HandsFree Indicator Id */
#define WICED_BT_HFP_HF_IND_ID_ENHANCED_SAFETY      1       /***< Enhanced Driver Safety */
#define WICED_BT_HFP_HF_IND_ID_BATTERY              2       /***< Battery Level */

/******************************************************************************
*                   Typedefs
******************************************************************************/
/* HF caller number */
typedef char wiced_bt_hfp_hf_caller_num_t[WICED_BT_HFP_HF_CALLER_NUMBER_MAX_LENGTH];

/** HF AT result code */
typedef char wiced_bt_hfp_hf_at_result_code_t[WICED_BT_HFP_HF_AT_CMD_RESULT_CODE_MAX_LENGTH];

/******************************************************************************
*                   Enumerations
******************************************************************************/

/* HF device supported feature flags. */
typedef enum
{
    WICED_BT_HFP_HF_FEATURE_ECNR                         = 0x00000001,
    WICED_BT_HFP_HF_FEATURE_3WAY_CALLING                 = 0x00000002,
    WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY              = 0x00000004,
    WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION = 0x00000008,
    WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL        = 0x00000010,
    WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS         = 0x00000020,
    WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL        = 0x00000040,
    WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION            = 0x00000080,
    WICED_BT_HFP_HF_FEATURE_HF_INDICATORS                = 0x00000100,
    WICED_BT_HFP_HF_FEATURE_ESCO_S4_T2_SETTINGS_SUPPORT  = 0x00000200,
    WICED_BT_HFP_HF_FEATURE_ENHANCED_VOICE_RECOGNITION   = 0x00004000
} wiced_bt_hfp_hf_supported_features_t;

/* AG supported feature flags. */
typedef enum
{
    WICED_BT_HFP_AG_FEATURE_3WAY_CALLING                 = 0x00000001,
    WICED_BT_HFP_AG_FEATURE_ECNR                         = 0x00000002,
    WICED_BT_HFP_AG_FEATURE_VOICE_RECOGNITION_ACTIVATION = 0x00000004,
    WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY  = 0x00000008,
    WICED_BT_HFP_AG_FEATURE_ATTACH_NUMBER_TO_VOICE_TAG   = 0x00000010,
    WICED_BT_HFP_AG_FEATURE_ABILITY_TO_REJECT_CALL       = 0x00000020,
    WICED_BT_HFP_AG_FEATURE_ENHANCED_CALL_STATUS         = 0x00000040,
    WICED_BT_HFP_AG_FEATURE_ENHANCED_CALL_CONTROL        = 0x00000080,
    WICED_BT_HFP_AG_FEATURE_EXTENDED_ERROR_RESULT_CODES  = 0x00000100,
    WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION            = 0x00000200,
    WICED_BT_HFP_AG_FEATURE_HF_INDICATORS                = 0x00000400,
    WICED_BT_HFP_AG_FEATURE_ESCO_S4_T2_SETTINGS_SUPPORT  = 0x00000800,
    WICED_BT_HFP_AG_FEATURE_ENHANCED_VOICE_RECOGNITION   = 0x00001000
} wiced_bt_hfp_ag_supported_features_t;

/* HF Events. These are received via wiced_bt_hfp_hf_event_cb_t() callback function.
   See wiced_bt_hfp_hf_event_data_t for payload. */
typedef enum
{
    WICED_BT_HFP_HF_CONNECTION_STATE_EVT,   /* Received on control path connection state change */
    WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT, /* Indicates HFP features supported in AG */
    WICED_BT_HFP_HF_SERVICE_STATE_EVT,      /* Indicates AG's cellular network connection state */
    WICED_BT_HFP_HF_SERVICE_TYPE_EVT,       /* Indicates whether AG is connected to home or romaing network */
    WICED_BT_HFP_HF_CALL_SETUP_EVT,         /* Received when there is a change in call state, e.g., incoming call, call termination */
    WICED_BT_HFP_HF_RING_EVT,               /* Ring indication (during incoming call) received */
    WICED_BT_HFP_HF_INBAND_RING_STATE_EVT,  /* Indicates if the AG supports sending the ring-tone to HF over audio connection */
    WICED_BT_HFP_HF_RSSI_IND_EVT,           /* Indicates AG's cellular signal strength */
    WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT, /* Indicates AG's battery status */
    WICED_BT_HFP_HF_VOLUME_CHANGE_EVT,      /* Received when there is a change in the microphone or speaker volume is changed by AG */
    WICED_BT_HFP_HF_CLIP_IND_EVT,           /* Indicates calling line identification name/number */
    WICED_BT_HFP_HFP_CODEC_SET_EVT,         /* Received when codec is selected (If Codec negotiation is supported) */
    WICED_BT_HFP_HFP_ACTIVE_CALL_EVT,       /* Response of AT+CLCC command */
    WICED_BT_HFP_HF_OK_EVT,                 /* Received AT+OK response */
    WICED_BT_HFP_HF_ERROR_EVT,              /* Received AT+ERROR response */
    WICED_BT_HFP_HF_CME_ERROR_EVT,          /* Received AT+CME ERROR response */
    WICED_BT_HFP_HF_CNUM_EVT,               /* Received AT+CNUM response */
    WICED_BT_HFP_HF_BINP_EVT,               /* Received AT+BINP response */
    WICED_BT_HFP_HF_VOICE_RECOGNITION_EVT,  /* Received AT+BVRA response */
    WICED_BT_HFP_HF_BIND_EVT,               /* Indicators Update event */

} wiced_bt_hfp_hf_event_t;

/* HF Control Connection States */
typedef enum
{
    WICED_BT_HFP_HF_STATE_DISCONNECTED, /* HF control connection is closed */
    WICED_BT_HFP_HF_STATE_CONNECTED,    /* HF control connection established */
    WICED_BT_HFP_HF_STATE_SLC_CONNECTED /* HF synchronized with AG's state, ready to send/recive commands/notifications */
} wiced_bt_hfp_hf_connection_state_t;

/* AG's serivce states */
typedef enum
{
    WICED_BT_HFP_HF_SERVICE_STATE_NOT_AVAILABLE, /* AG's cellular services not available */
    WICED_BT_HFP_HF_SERVICE_STATE_AVAILABLE      /* AG is connected to cellular services */
} wiced_bt_hfp_hf_service_state_t;

/* AG's serivce type */
typedef enum
{
    WICED_BT_HFP_HF_SERVICE_TYPE_HOME,   /* AG is connected to home network */
    WICED_BT_HFP_HF_SERVICE_TYPE_ROAMING /* AG is connected to a romaing network */
} wiced_bt_hfp_hf_service_type_t;

/* States of a call during setup procedure */
typedef enum
{
    WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE,     /* No call set up in progress */
    WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING, /* There is an incoming call */
    WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING,  /* Outgoing call is being setup up */
    WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING, /* Remote party is being alterted of the call */
    WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING   /* Incoming call is waiting (received when a call is already active) */
} wiced_bt_hfp_hf_callsetup_state_t;

/* In-band ring tone setting in AG*/
typedef enum
{
    WICED_BT_HFP_HF_INBAND_RING_DISABLED, /* AG will not send ring-tone thru audio connection, HF will use some means to alert the user of an incoming call. */
    WICED_BT_HFP_HF_INBAND_RING_ENABLED   /* AG will send the ring-tone thru audio connection */
} wiced_bt_hfp_hf_inband_ring_state_t;

/* Audio input/output device on the HF Device */
typedef enum
{
    WICED_BT_HFP_HF_SPEAKER, /* Refers to speaker on the HF Device */
    WICED_BT_HFP_HF_MIC      /* Refers to microphone on the HF Device */
} wiced_bt_hfp_hf_volume_type_t;

/* Call action command */
typedef enum
{
    WICED_BT_HFP_HF_CALL_ACTION_DIAL,   /* Place an outgoing call request */
    WICED_BT_HFP_HF_CALL_ACTION_ANSWER, /* Answer an incoming call */
    WICED_BT_HFP_HF_CALL_ACTION_HANGUP, /* Hangup an active call, reject an incoming call, end an outgoing call (which is being setup) */
    WICED_BT_HFP_HF_CALL_ACTION_HOLD_0, /* Release a held call, or reject a waiting call (UDUB) */
    WICED_BT_HFP_HF_CALL_ACTION_HOLD_1, /* Release active call and activate a held or waiting call */
    WICED_BT_HFP_HF_CALL_ACTION_HOLD_2, /* Place active call on hold, and accept a waiting call or retrieve a held call */
    WICED_BT_HFP_HF_CALL_ACTION_HOLD_3, /* Adds a held call to the conversation */
    WICED_BT_HFP_HF_CALL_ACTION_HOLD_4, /* Connects the two calls and disconnects the subscriber from both calls (Explicit Call Transfer). */
} wiced_bt_hfp_hf_call_action_t;

/* Codec for HF profile */
typedef enum
{
    WICED_BT_HFP_HF_CVSD_CODEC  =   1,
    WICED_BT_HFP_HF_MSBC_CODEC  =   2
}wiced_bt_hfp_hf_codec_t;

/* HF Call direction */
typedef enum
{
    WICED_BT_HFP_HF_OUTGOING  =   0,
    WICED_BT_HFP_HF_INCOMING  =   1
}wiced_bt_hfp_hf_call_dir_t;

/* Profile type */
typedef enum
{
    WICED_BT_HFP_PROFILE,
    WICED_BT_HSP_PROFILE
}wiced_bt_profile_type_t;

/* HF Call status */
typedef enum
{
    WICED_BT_HFP_HF_CALL_ACTIVE  =   0,
    WICED_BT_HFP_HF_CALL_HELD,
    WICED_BT_HFP_HF_CALL_DIALING,
    WICED_BT_HFP_HF_CALL_ALERING,
    WICED_BT_HFP_HF_CALL_INCOMING,
    WICED_BT_HFP_HF_CALL_WAITING,
    WICED_BT_HFP_HF_CALL_HELD_BY_RSP_HOLD
}wiced_bt_hfp_hf_call_status_t;

/* HF Call mode */
typedef enum
{
    WICED_BT_HFP_HF_MODE_VOICE = 0,
    WICED_BT_HFP_HF_MODE_DATA,
    WICED_BT_HFP_HF_MODE_FAX,
}wiced_bt_hfp_hf_call_mode_t;
/******************************************************************************
*                 Type Definitions
******************************************************************************/

/******************************************************************************
*                    Structures
******************************************************************************/

/* Call State event data */
typedef struct
{
    wiced_bt_hfp_hf_callsetup_state_t setup_state;         /* Call setup progress indicator */
    wiced_bool_t                      held_call_present;   /* TRUE if a held call is present, else FALSE */
    wiced_bool_t                      active_call_present; /* TRUE if an active call is present, else FALSE */
} wiced_bt_hfp_hf_call_data_t;

/* Volume Change event data */
typedef struct
{
    wiced_bt_hfp_hf_volume_type_t type;  /* Whether HF volume change is being requested for mic or spkr */
    uint8_t                       level; /* Volume level from 0 to 15 */
} wiced_bt_hfp_hf_volume_data_t;

/* WICED HF config */
typedef struct
{
    uint8_t  mic_volume;                     /* Default/initial mic volume level from 0 to 15 */
    uint8_t  speaker_volume;                 /* Default/initial speaker volume level from 0 to 15 */
    uint32_t feature_mask;                   /* HFP HF features supported bitmask - A combination of wiced_bt_hfp_hf_supported_features_t values */
    uint8_t  num_server;                     /* Number of HFP+HSP HF servers to start during init */
    uint8_t  scn[WICED_BT_HFP_HF_MAX_CONN];  /* Array of num_server HFP+HSP HF server channel number. This should be the same as configured in the SDP server */
    uint16_t uuid[WICED_BT_HFP_HF_MAX_CONN]; /* The UUID of the service corresponding to the SCN specified at nth index in scn[] above. */
} wiced_bt_hfp_hf_config_data_t;

/* Clip event data */
typedef struct
{
    wiced_bt_hfp_hf_caller_num_t caller_num;    /* Caller number */
    uint8_t                      type;          /* Specify format of the caller number */
} wiced_bt_hfp_hf_clip_data_t;

/* HF AT result */
typedef struct
{
    wiced_bt_hfp_hf_at_result_code_t arg;       /* AT result arguments */
    uint16_t                         num;       /* AT result number */
    uint8_t                          event;     /* AT Event code  */
} wiced_bt_hfp_hf_at_result_t;

/* HF connection data */
typedef struct
{
    wiced_bt_device_address_t           remote_address;    /* BD address of the remote device */
    wiced_bt_hfp_hf_connection_state_t  conn_state;        /* Connection state */
    wiced_bt_profile_type_t             connected_profile; /* To check whether connected to HSP - if the peer does not support HFP but only HSP. */
} wiced_bt_hfp_hf_connection_data_t;

/* Active call event data */
typedef struct
{
    uint8_t                          idx;           /* Sequence id */
    wiced_bt_hfp_hf_call_dir_t       dir;           /* Call Direction */
    wiced_bt_hfp_hf_call_status_t    status;        /* Call Status */
    wiced_bt_hfp_hf_call_mode_t      mode;          /* Call mode */
    wiced_bool_t                     is_conference; /* True for conference call */
    wiced_bt_hfp_hf_caller_num_t     num;           /* Caller Number */
    uint8_t                          type;          /* Specify format of the caller number */
}wiced_bt_hfp_hf_active_call_t;

/* Bind event data */
typedef struct
{
    uint8_t                          ind_id;           /* Indicator id */
    uint8_t                          ind_value;        /* Indicator Value */
}wiced_bt_hfp_hf_bind_data_t;

/* HF Event Data */
typedef struct
{
    uint16_t                                 handle;           /* Connection Handle */
    union
    {
        wiced_bt_hfp_hf_connection_data_t    conn_data;        /* Payload for WICED_BT_HFP_HF_CONNECTION_STATE_EVT */
        uint32_t                             ag_feature_flags; /* Payload for WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT */
        wiced_bt_hfp_hf_service_state_t      service_state;    /* Payload for WICED_BT_HFP_HF_SERVICE_STATE_EVT */
        wiced_bt_hfp_hf_service_type_t       service_type;     /* Payload for WICED_BT_HFP_HF_SERVICE_TYPE_EVT */
        wiced_bt_hfp_hf_call_data_t          call_data;        /* Payload for WICED_BT_HFP_HF_CALL_STATE_EVT */
        wiced_bt_hfp_hf_inband_ring_state_t  inband_ring;      /* Payload for WICED_BT_HFP_HF_INBAND_RING_STATE_EVT */
        uint8_t                              rssi;             /* Payload for WICED_BT_HFP_HF_RSSI_IND_EVT */
        uint8_t                              battery_level;    /* Payload for WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT */
        wiced_bt_hfp_hf_volume_data_t        volume;           /* Payload for WICED_BT_HFP_HF_VOLUME_CHANGE_EVT */
        wiced_bt_hfp_hf_clip_data_t          clip;             /* Payload for WICED_BT_HFP_HF_CLIP_IND_EVT */
        wiced_bt_hfp_hf_codec_t              selected_codec;   /* Payload for WICED_BT_HFP_HFP_CODEC_SET_EVT */
        wiced_bt_hfp_hf_active_call_t        active_call;      /* Payload for WICED_BT_HFP_HFP_ACTIVE_CALL_EVT */
        uint8_t                              error_code;       /* Payload for WICED_BT_HFP_HF_CME_ERROR_EVT */
        wiced_bt_hfp_hf_at_result_code_t     cnum_data;        /* Payload for WICED_BT_HFP_HF_CNUM_EVT */
        wiced_bt_hfp_hf_clip_data_t          binp_data;        /* Payload for WICED_BT_HFP_HF_BINP_EVT */
        uint8_t                              voice_recognition;/* Payload for WICED_BT_HFP_HF_VOICE_RECOGNITION_EVT */
        wiced_bt_hfp_hf_bind_data_t          bind_data;        /* Payload for WICED_BT_HFP_HF_BINP_EVT */
    };
}wiced_bt_hfp_hf_event_data_t;

/******************************************************************************
*                 Callback Type Definitions
******************************************************************************/

/******************************************************************************
*
* \name The HF control path callback type.
*
* \brief The application implements a callback of this type to receive HF events and commands.
*
*  \param event    HF event.
*  \param p_data   pointer to event data.
*
*  \return None.
*
******************************************************************************/
typedef void (*wiced_bt_hfp_hf_event_cb_t)( wiced_bt_hfp_hf_event_t event,
    wiced_bt_hfp_hf_event_data_t* p_data);

/******************************************************************************
*               Function Declarations
******************************************************************************/

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_init
*
* \brief The API to initialize the HFP-HF component and register with the stack.
*
*  Called by the application before any other API is called.
*  Application provides the SINK configuration data and callback to receive control events.
*
*  \param num_server    Number of Handsfree server to be created
*  \param a_config_data    HF configuration parameters array for each server. See wiced_bt_hfp_hf_config_data_t.
*  \param event_cb         Callback function for receiving HF events.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_init( wiced_bt_hfp_hf_config_data_t *p_config_data,
    wiced_bt_hfp_hf_event_cb_t event_cb );

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_deinit
*
* \brief The API to deregister the HFP-HF component from the stack and to clean up internal data structures.
*
*  Called by the application when the HFP-HF component is no longer needed by it.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_deinit( void );

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_connect
*
* \brief The API to initiate a HFP connection to an AG.
*
*  Called by the application to connect to an AG with the given address.
*
*  \param bd_address   BD address of the AG.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_connect( wiced_bt_device_address_t bd_address );

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_disconnect
*
* \brief The API to disconnect from an AG.
*
*  Called by the application to disconnect from an AG with a given address.
*
*  \param handle       Connection handle.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_disconnect( uint16_t handle );

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_perform_call_action
*
* \brief The API to manipulate a call (i.e., to answer, hold, hangup, reject, etc).
*
*  Allows the application to take actions indicated in wiced_bt_hfp_hf_call_action_t.
*
*  \param handle       Connection handle.
*  \param action       Action to be initiated, see wiced_bt_hfp_hf_call_action_t.
*  \param number       Contains a NUL terminated number to be called,
*                      if NULL, the last number redial (LNR) is initiated.
*                      valid when action is WICED_BT_HFP_HF_CALL_ACTION_DIAL,
*                      for all other actions this is ignored.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_perform_call_action( uint16_t handle,
    wiced_bt_hfp_hf_call_action_t action, char* number );

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_notify_volume
*
* \brief The API to send the current speaker/mic volume level to AG.
*
*  Called by the application to notify the AG of the change in volume of mic or speaker.
*
*  \param handle       Connection handle.
*  \param volume_type  Mic or speaker for which the volume was changed.
*  \param volume_level Volume level from 0 to 15.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_notify_volume( uint16_t handle,
    wiced_bt_hfp_hf_volume_type_t volume_type, uint8_t volume_level );

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_send_at_cmd
*
* \brief API to send the at command to the AG
*
*  Called by the application to send an command to AG. The command sent is passed thru
*  for the library. The response is received thru WICED_BT_HFP_HF_AT_RESULT_CODE_IND_EVT.
*
*  \param handle       Connection handle.
*  \param at_cmd       Null terminated at command string to be sent to AG.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_send_at_cmd( uint16_t handle, char* at_cmd );

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_lrac_switch_get
*
* \brief The API to get LRAC Switch data.
*
*  Called by the application to get the LRAC Switch Data.
*
*  \param p_opaque     The pointer to a buffer which will be filled with LRAC Switch data (current
*                      HFP Sink State).
*  \param p_opaque     Size of the buffer (IN), size filled (OUT)
*
*  \return None.
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/******************************************************************************
*
* Function Name: wiced_bt_hfp_hf_lrac_switch_set
*
* \brief The API to set LRAC switch data.
*
*  Called by the application to set the LRAC Switch Data
*
*  \param p_opaque     The pointer to a buffer which contains LRAC Switch data (new HFP Sink State).
*  \param p_opaque     Size of the buffer (IN *
*  \return None.
*
******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_lrac_switch_set(void *p_opaque, uint16_t sync_data_len);

/* @} */
/* end of hfp_hf */
#ifdef __cplusplus
}
/* extern "C" */
#endif /* _WICED_BT_HFP_HF_H_ */
