/***************************************************************************//**
* \file <wiced_bt_a2dp_source.h>
*
* \brief
* 	Contains A2DP Source APIs and definitions.
*
*//*****************************************************************************
* Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
* Cypress Semiconductor Corporation. All Rights Reserved.
*
* This software, including source code, documentation and related
* materials ("Software"), is owned by Cypress Semiconductor Corporation
* or one of its subsidiaries ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products. Any reproduction, modification, translation,
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

#include "wiced.h"
#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2d_m12.h"
#include "wiced_bt_a2d_m24.h"
#include "wiced_bt_a2dp_defs.h"


/******************************************************************************
*
* @defgroup    wicedbt_av_profiles Profiles
* @ingroup     wicedbt_av
*
* @addtogroup  wicedbt_a2dp        Advanced Audio Profile (A2DP) Source
* @ingroup     wicedbt_av_profiles
* @ingroup     wicedbt_av
*
*  Advanced Audio Distribution Profile Source interfaces
*
* @{
*
******************************************************************************/

/******************************************************************************
*                    Constants
******************************************************************************/


/******************************************************************************
*                   Enumerations
******************************************************************************/

/* Events in wiced_bt_a2dp_source_control_cb_t() callback,
   for payload see wiced_bt_a2dp_source_event_data_t */
typedef enum
{
    WICED_BT_A2DP_SOURCE_CONNECT_EVT,     /* Connected event, received on establishing connection to a peer device */
    WICED_BT_A2DP_SOURCE_DISCONNECT_EVT,  /* Disconnected event, received on disconnection from a peer device */
    WICED_BT_A2DP_SOURCE_START_IND_EVT,   /* Start stream indication event, received when start req is received */
    WICED_BT_A2DP_SOURCE_START_CFM_EVT,   /* Start stream confirm event, received when start req is sent and response is received */
    WICED_BT_A2DP_SOURCE_SUSPEND_EVT,     /* Suspend stream event, received when audio streaming is suspended */
} wiced_bt_a2dp_source_event_t;


/* A2DP Source features masks */
typedef enum
{
    WICED_BT_A2DP_SOURCE_FEAT_PROTECT   = 0x0001, /* Streaming media content protection */
    WICED_BT_A2DP_SOURCE_FEAT_DELAY_RPT = 0x0002, /* Use delay reporting */
} wiced_bt_a2dp_source_feature_mask_t;


/******************************************************************************
*                 Type Definitions
******************************************************************************/

/******************************************************************************
*                    Structures
******************************************************************************/

/* Codec capability information list structure,
   used to indicate the supported codecs and their capabilities */
typedef struct
{
    uint8_t                     count; /* Number of codecs present in the list */
    wiced_bt_a2dp_codec_info_t* info;  /* Codec information list */
} wiced_bt_a2dp_codec_info_list_t;

/* A2DP source configuration data structure */
typedef struct
{
    wiced_bt_a2dp_source_feature_mask_t         feature_mask;       /* Supported features */
    wiced_bt_a2dp_codec_info_list_t           codec_capabilities;   /* List of supported codecs and their capabilities */
    wiced_bt_a2dp_codec_info_t                default_codec_config;
} wiced_bt_a2dp_config_data_t;

/* Generic event status info */
typedef struct
{
    wiced_result_t            result;       /* Whether the event indicates failure or success, WICED_BT_XXX */
    wiced_bt_device_address_t bd_addr;      /* Peer bluetooth device address */
    uint16_t                  handle;       /* Peer connection handle */
    uint16_t                  lcid;         /* Local identifier */
    uint8_t                   is_accepter;  /* True if device is accepter, otherwise false */
} wiced_bt_a2dp_source_status_t;

/* A2DP Start event data structure */
typedef struct
{
    wiced_result_t            result;  /* Whether the event indicates failure or success, WICED_BT_XXX */
    uint8_t                   label;   /* Transaction label */
    uint16_t                  handle;  /* Peer connection handle */
} wiced_bt_a2dp_source_start_t;

/* Control callback event data */
typedef union
{
    wiced_bt_a2dp_source_status_t         connect;        /* WICED_BT_A2DP_SOURCE_CONNECT_EVT payload */
    wiced_bt_a2dp_source_status_t         disconnect;     /* WICED_BT_A2DP_SOURCE_DISCONNECT_EVT payload */
    wiced_bt_a2dp_source_start_t          start_ind;      /* WICED_BT_A2DP_SOURCE_START_IND_EVT payload */
    wiced_bt_a2dp_source_status_t         start_cfm;      /* WICED_BT_A2DP_SOURCE_START_CFM_EVT payload */
    wiced_bt_a2dp_source_status_t         suspend;        /* WICED_BT_A2DP_SOURCE_SUSPEND_EVT payload */
} wiced_bt_a2dp_source_event_data_t;

/******************************************************************************
*                 Callback Type Definitions
******************************************************************************/

/******************************************************************************
*
* \name The A2DP Control path callback type.
*
* \brief The application implements a callback of this type to receive A2DP control path events.
*
*  \param event    ID of event being notified to app.
*  \param p_data   The pointer to data associated with the event.
*
*  \return None.
*
******************************************************************************/
typedef void (*wiced_bt_a2dp_source_control_cb_t)( wiced_bt_a2dp_source_event_t event,
    wiced_bt_a2dp_source_event_data_t* p_data );

/******************************************************************************
*               Function Declarations
******************************************************************************/

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_init
*
* \brief API to initialize the A2DP SOURCE component and register with the stack.
*
* \details Called by the application before any other API is called.
*          The application provides source configuration data and control and data callbacks
*          to receive control events and data packets, respectively.
*
*  \param p_config_data    A2DP source configuration parameters.
*                          This should remain valid until deinit is called
*                          as the pointer is stored and used inside the library.
*  \param control_cb       Callback function for receiving source events.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_init( wiced_bt_a2dp_config_data_t* p_config_data,
    wiced_bt_a2dp_source_control_cb_t control_cb );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_deinit
*
* \brief The API to deregister from the stack and to clean up the memory of A2DP source component.
*        Called by the application when A2DP source component is no longer needed by it.
*
*  \param channel  Media type to be handled by the source.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_deinit(void);


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_connect
*
* \brief The API to connect to a peer device.
*        Called by the app to establish A2DP connection with a peer device.
*
*  \param bd_address    The Bluetooth device address of the device to which connection is requested.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_connect( wiced_bt_device_address_t bd_address );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_disconnect
*
* \brief The API to disconnect the connection from a connected peer device.
*        Called by the application to disconnect from a connected A2DP source.
*
*  \param handle    Connection handle corresponding to the peer device to disconnect from.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_disconnect( uint16_t handle );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_start
*
* \brief The API to start streaming.
*        Called by the application when it wants to indicate the peer to start streaming.
*
*  \param handle        Connection handle corresponding to peer device
*                       to create a streaming connection.
*  \param codec_info    Codec configuration structure pointer.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_start( uint16_t handle, wiced_bt_a2dp_codec_info_t *codec_info );

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_send_start_response
*
* \brief The API to send a start response on receiving a start request from the peer.
*        Called by the application when it wants to indicate the peer that it is ready to start streaming.
*
*  \param handle        Connection handle corresponding to peer device
*  \param label         Transaction label
*  \param status        Indicates if a start request is accepted(AVDT_SUCCESS) or rejected(AVDT Error codes)
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_send_start_response( uint16_t handle, uint8_t label, uint8_t status );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_suspend
*
* \brief The API to suspend streaming.
*        Called by the application when the streaming is to be suspended.
*
*  \param handle        Connection handle corresponding to peer device
*                       for which streaming is suspended.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_suspend( uint16_t handle );

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_update_route_config
*
* \brief To configure an audio route.
*        Called by the application to configure audio data route path.
*        This API is called after receiving a WICED_BT_A2DP_SOURCE_CODEC_CONFIG_EVT event
*        and before accepting an audio start request.
*
*   \param handle        Connection handle corresponding to the peer device
*                        to which route needs to be configured.
*   \param route_config  route config parameters
*
*   \return WICED_TRUE in success case,otherwise WICED_FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_a2dp_source_update_route_config( uint16_t handle, wiced_bt_a2dp_source_route_config *route_config );

/* @} */
/* end of wicedbt_a2dp */
#ifdef __cplusplus
} /* extern "C" */
#endif /* _WICED_BT_A2DP_SOURCE_H_ */
