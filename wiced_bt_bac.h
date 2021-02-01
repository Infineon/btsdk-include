/*
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
 */

/**************************************************************************//**
* \file
*
* \brief Provides definitions of the Battery Client (BAC) library interface.
*
******************************************************************************/


#ifndef BATTERY_CLIENT_H
#define BATTERY_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
* \addtogroup  wiced_bt_bac_api_functions        BAC Library API
* \ingroup     wicedbt
* @{
* The BAC library of the WICED SDK provide a simple method for an application to integrate the BAC
* service functionality. The application calls the library APIs to control the Battery service of
* a peer device using GATT.
*/

/** BAC Events received by the applicaton's BAC callback (see \ref wiced_bt_bac_callback_t)
*
*/
typedef enum
{
    WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE = 1,      /**< GATT Discovery Complete */
    WICED_BT_BAC_EVENT_BATTERY_LEVEL_RSP,           /**< Battery Level Read Response */
    WICED_BT_BAC_EVENT_BATTERY_LEVEL_NOTIFICATION   /**< Battery Level Notification received */
} wiced_bt_bac_event_t;


/**
 * \brief Data associated with \ref WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE.
 *
 * This event is received when a GATT Discovery Operation is complete
 *
 */
typedef struct
{
    uint16_t conn_id;                               /**< Connection Id */
    wiced_bt_gatt_status_t status;                  /**< Discovery Status */
    wiced_bool_t notification_supported;            /**< Indicate if Notification supported */
} wiced_bt_bac_discovery_complete_t;

/**
 * \brief Data associated with \ref WICED_BT_BAC_EVENT_BATTERY_LEVEL_RSP.
 *
 * This event is received when a Battery Level Response is Received.
 *
 */
typedef struct
{
    uint16_t conn_id;
    wiced_bt_gatt_status_t status;
    uint8_t battery_level;
} wiced_bt_bac_battery_level_rsp_t;

/**
 * \brief Data associated with \ref WICED_BT_BAC_EVENT_BATTERY_LEVEL_NOTIFICATION.
 *
 * This event is received when a Battery Level Notification is Received.
 *
 */
typedef struct
{
    uint16_t conn_id;
    uint8_t battery_level;
} wiced_bt_bac_battery_level_notification_t;


/**
 * \brief Union of data associated with BAC events
 *
 */
typedef union
{
    wiced_bt_bac_discovery_complete_t           discovery;
    wiced_bt_bac_battery_level_rsp_t            battery_level_rsp;
    wiced_bt_bac_battery_level_notification_t   battery_level_notification;
} wiced_bt_bac_event_data_t;

/**
* BAC Callback function type wiced_bt_bac_callback_t
*
*                  This function is called to send BAC events to the application.
*                  This function is registered with the \ref wiced_bt_bac_initialize function.
*
* \param[in]       event  : BAC Event.
* \param[in]       p_data : Data (pointer on union of structure) associated with the event.
*
* \return NONE.
*/
typedef void (wiced_bt_bac_callback_t)(wiced_bt_bac_event_t event, wiced_bt_bac_event_data_t *p_data);

/******************************************************************************
*                         Function Prototypes
******************************************************************************/


/******************************************************************************
* Function Name: wiced_bt_bac_init
***************************************************************************//**
*
* The application calls this function to initialize the Battery Client profile.
* The application registered callbacks get called when requested operations are completed.
* After a connection is success, the application does the BAC service discovery.
* If the BAC service is discovered successfully, the application can read the Battery Level or
* Enable Battery Level Notifications.
*
* \param          p_callback : Pointer to application BAC callback function.
*
* \return         WICED_SUCCESS if BAC initialized successfully, error otherwise.
*
******************************************************************************/
wiced_result_t wiced_bt_bac_init(wiced_bt_bac_callback_t *p_callback);

/******************************************************************************
*
* Function Name: wiced_bt_bac_discover
*
***************************************************************************//**
*
* The application calls this function to performs Battery Service characteristics discovery and
* characteristic descriptor discovery.
* After the discovery is complete, a registered application p_op_complete_callback is called with
* the result of the operation.
*
* \param           conn_id      : GATT connection ID.
* \param           start_handle : Start GATT handle of the ANP service.
* \param           end_handle   : End GATT handle of the ANP service.
*
* \return          Returns the status of the GATT operation.
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_bac_discover(uint16_t conn_id, uint16_t start_handle, uint16_t end_handle);

/******************************************************************************
*
* Function Name: wiced_bt_bac_discovery_result
*
***************************************************************************//**
*
* The application calls this API during GATT discovery to pass discovery results for the BAC
* service to the BAC Library.
* The library needs to find BAC service characteristics and associated characteristic client
* configuration descriptors.
*
* \param           p_data : Discovery result data as passed from the stack.
*
* \return          None.
*
******************************************************************************/
void wiced_bt_bac_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);

/******************************************************************************
*
* Function Name: wiced_bt_bac_client_discovery_complete
*
***************************************************************************//**
*
* The application calls this API during GATT discovery to pass discovery complete
* information for the BAC service to the BAC Library.
* As the GATT discovery is performed in multiple steps this function initiates
* the next discovery request.
*
* \param           p_data : Discovery complete data as passed from the stack.
*
* \return          None.
*
******************************************************************************/
void wiced_bt_bac_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);

/******************************************************************************
*
* Function Name: wiced_bt_bac_read_battery_level
*
***************************************************************************//**
*
* If a discovery complete is a success, the application can read the battery
* level provided by the server.
*
* \param           conn_id : GATT connection ID.
*
* \return          Returns the status of the GATT operation.
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_bac_read_battery_level(uint16_t conn_id);

/******************************************************************************
*
* Function Name: wiced_bt_bac_enable_notification
*
***************************************************************************//**
*
* Writes a client characteristic configuration descriptor to receive battery
* level notifications from the server.
*
* \param          conn_id : GATT connection ID.
*
* \return         Returns the status of the GATT operation.
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_bac_enable_notification(uint16_t conn_id);

/******************************************************************************
*
* Function Name: wiced_bt_bac_disable_notification
*
***************************************************************************//**
*
* Writes a client characteristic configuration descriptor to stop receiving
* battery level notifications from the server.
*
* \param          conn_id : GATT connection ID.
*
* \return         Returns the status of the GATT operation.
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_bac_disable_notification(uint16_t conn_id);

/******************************************************************************
*
* Function Name: wiced_bt_bac_read_rsp
*
***************************************************************************//**
*
* The application calls this function when it receives GATT Read Response
* for the attribute handle that belongs to the BAC service.
*
* \param           p_data : The pointer to a GATT operation complete data structure.
*
* \return          None.
*
******************************************************************************/
void wiced_bt_bac_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);

/******************************************************************************
*
* Function Name: wiced_bt_bac_process_notification
*
***************************************************************************//**
*
* The application calls this when it receives notification from the server.
* The Profile processes the notification and sends it as a callback to the application.
*
* \param          p_data  : The pointer to a GATT operation complete data structure.
*
* \return         None.
*
******************************************************************************/
void wiced_bt_bac_process_notification(wiced_bt_gatt_operation_complete_t *p_data);

/******************************************************************************
*
* Function Name: wiced_bt_bac_client_connection_up
*
***************************************************************************//**
*
* The application calls this function when BLE connection with a peer
* device established.
*
* \param           p_conn_status  : The pointer to a wiced_bt_gatt_connection_status_t
*                                   that includes the address and connection ID.
*
* \return          None.
*
******************************************************************************/
void wiced_bt_bac_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);

/******************************************************************************
*
* Function Name: wiced_bt_bac_client_connection_down
*
***************************************************************************//**
*
* The application calls this function when BLE connection with a peer
* device disconnected.
*
* \param           p_conn_status  : The pointer to a wiced_bt_gatt_connection_status_t
*                                   which includes the address and connection ID.
*
* \return          None.
*
******************************************************************************/
void wiced_bt_bac_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* BATTERY_CLIENT_H */
