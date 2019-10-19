/**************************************************************************//**
*
* \file <btle_homekit2_lightbulb.h>
*
* \brief
* 	Bluetooth Low Energy Homekit accessory sample
*
*//*****************************************************************************
* Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef __BTLE_HOMEKIT2_LIGHTBULB_H
#define __BTLE_HOMEKIT2_LIGHTBULB_H

/* BLE handles */
enum
{
    HDLS_GATT                                       = 0x01,

    HDLS_GAP                                        = 0x14,
    HDLC_GAP_DEVICE_NAME                            = 0x15,
    HDLC_GAP_DEVICE_NAME_VALUE                      = 0x16,
    HDLC_GAP_APPEARANCE                             = 0x17,
    HDLC_GAP_APPEARANCE_NAME_VALUE                  = 0x18,

    HDLS_ACCESSORY_INFO                             = 0x28,
    HDLC_ACCESSORY_INFO_INSTANCE_ID                 = 0x29,
    HDLC_ACCESSORY_INFO_INSTANCE_ID_VALUE           = 0x2A,
    HDLC_ACCESSORY_INFO_IDENTIFY                    = 0x2B,
    HDLC_ACCESSORY_INFO_IDENTIFY_VALUE              = 0x2C,
    HDLD_ACCESSORY_INFO_IDENTIFY_INSTANCE_ID        = 0x2D,
    HDLC_ACCESSORY_INFO_MANUFACTURER                = 0x2E,
    HDLC_ACCESSORY_INFO_MANUFACTURER_VALUE          = 0x2F,
    HDLD_ACCESSORY_INFO_MANUFACTURER_INSTANCE_ID    = 0x30,
    HDLC_ACCESSORY_INFO_MODEL                       = 0x31,
    HDLC_ACCESSORY_INFO_MODEL_VALUE                 = 0x32,
    HDLD_ACCESSORY_INFO_MODEL_INSTANCE_ID           = 0x33,
    HDLC_ACCESSORY_INFO_NAME                        = 0x34,
    HDLC_ACCESSORY_INFO_NAME_VALUE                  = 0x35,
    HDLD_ACCESSORY_INFO_NAME_INSTANCE_ID            = 0x36,
    HDLC_ACCESSORY_INFO_SERIAL_NUMBER               = 0x37,
    HDLC_ACCESSORY_INFO_SERIAL_NUMBER_VALUE         = 0x38,
    HDLD_ACCESSORY_INFO_SERIAL_NUMBER_INSTANCE_ID   = 0x39,
    HDLC_ACCESSORY_INFO_FIRMWARE_REVISION           = 0x3A,
    HDLC_ACCESSORY_INFO_FIRMWARE_REVISION_VALUE     = 0x3B,
    HDLD_ACCESSORY_INFO_FIRMWARE_REVISION_INSTANCE_ID = 0x3C,

    HDLS_PROTOCOL_INFO                              = 0x40,
    HDLC_PROTOCOL_INFO_INSTANCE_ID                  = 0x41,
    HDLC_PROTOCOL_INFO_INSTANCE_ID_VALUE            = 0x42,
    HDLC_PROTOCOL_INFO_VERSION                      = 0x43,
    HDLC_PROTOCOL_INFO_VERSION_VALUE                = 0x44,
    HDLD_PROTOCOL_INFO_VERSION_INSTANCE_ID          = 0x45,

    HDLS_LIGHTBULB                                  = 0x50,
    HDLC_LIGHTBULB_INSTANCE_ID                      = 0x51,
    HDLC_LIGHTBULB_INSTANCE_ID_VALUE                = 0x52,
    HDLC_LIGHTBULB_SERVICE_SIGNATURE                = 0x53,
    HDLC_LIGHTBULB_SERVICE_SIGNATURE_VALUE          = 0x54,
    HDLD_LIGHTBULB_SERVICE_SIGNATURE_INSTANCE_ID    = 0x55,
    HDLC_LIGHTBULB_BRIGHTNESS                       = 0x56,
    HDLC_LIGHTBULB_BRIGHTNESS_VALUE                 = 0x57,
    HDLD_LIGHTBULB_BRIGHTNESS_INSTANCE_ID           = 0x58,
    HDLD_LIGHTBULB_BRIGHTNESS_CLNT_CHAR_CFG         = 0x59,
    HDLC_LIGHTBULB_ON                               = 0x5a,
    HDLC_LIGHTBULB_ON_VALUE                         = 0x5b,
    HDLD_LIGHTBULB_ON_INSTANCE_ID                   = 0x5c,
    HDLD_LIGHTBULB_ON_CLNT_CHAR_CFG                 = 0x5d,
    HDLC_LIGHTBULB_NAME                             = 0x5e,
    HDLC_LIGHTBULB_NAME_VALUE                       = 0x5f,
    HDLD_LIGHTBULB_NAME_INSTANCE_ID                 = 0x60,
    HDLC_LIGHTBULB_HUE                              = 0x61,
    HDLC_LIGHTBULB_HUE_VALUE                        = 0x62,
    HDLD_LIGHTBULB_HUE_INSTANCE_ID                  = 0x63,
    HDLD_LIGHTBULB_HUE_CLNT_CHAR_CFG                = 0x64,
    HDLC_LIGHTBULB_SATURATION                       = 0x65,
    HDLC_LIGHTBULB_SATURATION_VALUE                 = 0x66,
    HDLD_LIGHTBULB_SATURATION_INSTANCE_ID           = 0x67,
    HDLD_LIGHTBULB_SATURATION_CLNT_CHAR_CFG         = 0x68,

};


/* HCI Control API */

/* Apple HomeKit group code */
#define HCI_CONTROL_GROUP_HK                                  0xFE

/* Apple HomeKit commands */
#define HCI_CONTROL_HK_COMMAND_READ                         ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x01 )    /* Read characteristic */
#define HCI_CONTROL_HK_COMMAND_WRITE                        ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x02 )    /* Write characteristic */
#define HCI_CONTROL_HK_COMMAND_LIST                         ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x03 )    /* List all characteristics */
#define HCI_CONTROL_HK_COMMAND_FACTORY_RESET                ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x04 )    /* Factory reset */
#define HCI_CONTROL_HK_COMMAND_GET_TOKEN                    ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x05 )    /* Get software authentication token */

/* Apple HomeKit events */
#define HCI_CONTROL_HK_EVENT_READ_RESPONSE                  ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x01 )    /* Response to read characteristic command */
#define HCI_CONTROL_HK_EVENT_UPDATE                         ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x02 )    /* Characteristic value update */
#define HCI_CONTROL_HK_EVENT_LIST_ITEM                      ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x03 )    /* Characteristic list item */
#define HCI_CONTROL_HK_EVENT_TOKEN_DATA                     ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x04 )    /* Software token data */

#define HCI_TOKEN_DATA_FLAG_START                           0x01
#define HCI_TOKEN_DATA_FLAG_END                             0x02
#define HCI_TOKEN_DATA_FLAG_UUID                            0x04

#endif /* _BTLE_HOMEKIT2_LIGHTBULB_H_ */
