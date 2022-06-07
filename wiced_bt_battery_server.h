/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

/****************************************************************************
* \file
*
* \brief Provides definitions of the Battery Client (BAC) library interface.
*
******************************************************************************/
#ifndef __BATTERY_SERVER_LIB_H
#define __BATTERY_SERVER_LIB_H

#include "bt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \addtogroup  wiced_bt_bas_api_functions        BAS Library API
* \ingroup     wicedbt
* @{
* The BAS library of the AIROC BTSDK provide a simple method for an application to integrate the battery
* service functionality. The application calls the library APIs to control the Battery service of
* a peer device using GATT.
*/

typedef UINT16 SFLOAT;
typedef UINT8 UINT24[3];

#ifdef BAS_1_1

#pragma pack(1)

#ifdef BATTERY_LEVEL_STATUS
typedef struct {
    UINT16  batt_present: 1;                // Battery Present: 0:No, 1:Yes
    UINT16  wired_ext_pwr_connected: 2;     // Wired External Power Source Connected: 0:No, 1:Yes, 2:Unknown, 3:RFU
    UINT16  wireless_ext_pwr_connected: 2;  // Wireless External Power Source Connected: 0:No, 1:Yes, 2:Unknown, 3:RFU
    UINT16  batt_charge_state: 2;           // Battery Charge State: 0:Unknown, 1:Charging, 2:Discharging, 3:Not Charging (Full), 4:Not Charging - Insufficient Power, 5: Not Charging - Over Temperature, 6: Not Charging - Under Temperature, 7: Not Charging - Fault
    UINT16  batt_charge_level: 2;           // Battery Charge Level: 0:Unknown, 1:Good, 2:Low, 3:Critical
    UINT16  batt_charge_type: 3;            // Battery Charge type: 0:Unknown or Not Charging, 1: Constant Current, 2: Constant Voltage, 3: Trickle, 4: Float, 5-7: RFU
    UINT16  batt_charge_fault: 3;           // Battery Charge fault reason, 0: Battery, 1:External Power source, 2: Others
    UINT16  rfu: 1;                         // RFU
} batt_power_state_t;

typedef enum {
    BAS_ADDITIONAL_STATUS_SERVICE_REQUIRED_FALSE,
    BAS_ADDITIONAL_STATUS_SERVICE_REQUIRED_TRUE,
    BAS_ADDITIONAL_STATUS_SERVICE_REQUIRED_UNKNOWN,
} bas_additional_status_service_required_e;

typedef UINT8 bas_additional_status_service_required_t;   // see bas_additional_status_service_required_e

typedef struct {
    UINT8  service_required: 2;             // 0: False, 1: True, 2: Unknown 3: RFU
    UINT8  rfu: 6;
} batt_add_status_t;

#ifdef BATTERY_LEVEL_STATUS_FLAG_ID
 #define BATTERY_LEVEL_STATUS_ID_PRESENT    (1<<0)
#else
 #define BATTERY_LEVEL_STATUS_ID_PRESENT    0
#endif
#ifdef BATTERY_LEVEL_STATUS_FLAG_LEVEL
 #define BATTERY_LEVEL_STATUS_LEVEL_PRESENT (1<<1)
#else
 #define BATTERY_LEVEL_STATUS_LEVEL_PRESENT 0
#endif
#ifdef BATTERY_LEVEL_STATUS_FLAG_ADDTIONAL_STATUS
 #define BATTERY_LEVEL_STATUS_ADD_ST_PRESENT (1<<2)
#else
 #define BATTERY_LEVEL_STATUS_ADD_ST_PRESENT 0
#endif

#define BATTERY_LEVEL_STATUS_FLAGS ( \
        BATTERY_LEVEL_STATUS_ID_PRESENT | \
        BATTERY_LEVEL_STATUS_LEVEL_PRESENT | \
        BATTERY_LEVEL_STATUS_ADD_ST_PRESENT)

typedef struct {
    UINT8               flags;
    batt_power_state_t  power_state;
#ifdef BATTERY_LEVEL_STATUS_FLAG_ID
    UINT16              identifier;
#endif
#ifdef BATTERY_LEVEL_STATUS_FLAG_LEVEL
    UINT8               battery_level;
#endif
#ifdef BATTERY_LEVEL_STATUS_FLAG_ADDTIONAL_STATUS
    batt_add_status_t               additional_status;
#endif
} batt_level_status_t;

/**
 * Function     wiced_bt_bas_set_battery_level_status_power_state
 *
 *              Set batter level status power state.
 *
 *  @param[in]  power_state : The power state data, see batt_power_state_t
 *
 *  @return     none
 */
void wiced_bt_bas_set_battery_level_status_power_state(batt_power_state_t power_state);

/**
 * Function     wiced_bt_bas_get_battery_level_status_power_state
 *
 *              Get batter level status power state.
 *
 *  @param[in]  : none
 *
 *  @return batt_power_state_t
 */
batt_power_state_t wiced_bt_bas_get_battery_level_status_power_state();

/**
 * Function     wiced_bt_bas_set_battery_level_additional_status_service_required
 *
 *              Send a handle value notification to a client.
 *
 *  @param[in]  service : see bas_additional_status_service_required_t
 *
 *  @return     none
 */
void wiced_bt_bas_set_battery_level_additional_status_service_required(bas_additional_status_service_required_t service);

/**
 * Function     wiced_bt_bas_get_battery_level_additional_status_service_required()
 *
 *              Get additional status for service required
 *
 *  @param[in]  : none
 *
 *  @return bas_additional_status_service_required_t
 *
 */
bas_additional_status_service_required_t wiced_bt_bas_get_battery_level_additional_status_service_required();

#endif // BATTERY_LEVEL_STATUS

#ifdef ESTIMATED_SERVICE_DATE

typedef struct {
    UINT24 estimated_service_date;
} batt_estimated_service_date_t;

#endif // ESTIMATED_SERVICE_DATE

#ifdef BATTERY_CRITICAL_STATUS

typedef struct {
    UINT8 critical_power_state: 1;
    UINT8 immediate_service_required: 1;
    UINT8 rfu: 6;
} batt_bt_bas_critical_status_t;

typedef struct {
    batt_bt_bas_critical_status_t batt_critical_status;
} batt_critical_status_t;

#endif // BATTERY_CRITICAL_STATUS

#ifdef BATTERY_ENERGY_STATUS

#ifdef BATTERY_ENERGY_STATUS_FLAG_EXT_SOURCE
 #define BATTERY_ENERGY_STATUS_FLAG_EXT_SOURCE_PRESENT    (1<<0)
#else
 #define BATTERY_ENERGY_STATUS_FLAG_EXT_SOURCE_PRESENT    0
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_PRESENT_VOLTAGE
 #define BATTERY_ENERGY_STATUS_FLAG_PRESENT_VOLTAGE_PRESENT (1<<1)
#else
 #define BATTERY_ENERGY_STATUS_FLAG_PRESENT_VOLTAGE_PRESENT 0
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY
 #define BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_PRESENT (1<<2)
#else
 #define BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_PRESENT 0
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_CAPACITY
 #define BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_CAPACITY_PRESENT (1<<3)
#else
 #define BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_CAPACITY_PRESENT 0
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_CHANGE_RATE
 #define BATTERY_ENERGY_STATUS_FLAG_CHANGE_RATE_PRESENT (1<<4)
#else
 #define BATTERY_ENERGY_STATUS_FLAG_CHANGE_RATE_PRESENT 0
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_ENERGY_AT_LAST_CHANGE
 #define BATTERY_ENERGY_STATUS_FLAG_ENERGY_AT_LAST_CHANGE_PRESENT (1<<5)
#else
 #define BATTERY_ENERGY_STATUS_FLAG_ENERGY_AT_LAST_CHANGE_PRESENT 0
#endif

#define BATTERY_ENERGY_STATUS_FLAG ( \
    BATTERY_ENERGY_STATUS_FLAG_EXT_SOURCE_PRESENT | \
    BATTERY_ENERGY_STATUS_FLAG_PRESENT_VOLTAGE_PRESENT | \
    BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_PRESENT | \
    BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_CAPACITY_PRESENT | \
    BATTERY_ENERGY_STATUS_FLAG_CHANGE_RATE_PRESENT | \
    BATTERY_ENERGY_STATUS_FLAG_ENERGY_AT_LAST_CHANGE_PRESENT )

typedef struct {
    UINT8   flags;
#ifdef BATTERY_ENERGY_STATUS_FLAG_EXT_SOURCE
    SFLOAT  external_source_power;              // The total power being consumed from an external power source in watts for its Battery Aggregation Group.
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_PRESENT_VOLTAGE
    SFLOAT  present_voltage;                    // The present terminal voltage of the battery in volts.
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY
    SFLOAT  available_energy;                   // The available energy of the battery in kilowatt-hours in its current charge state.
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_CAPACITY
    SFLOAT  available_power_capacity;           // The capacity of the battery in kilowatt-hours at full charge in its current (not new) condition.
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_CHANGE_RATE
    SFLOAT  change_rate;                        // The energy flowing into the battery in watts. Positive values indicate charging, and negative values indicate discharging.
#endif
#ifdef BATTERY_ENERGY_STATUS_FLAG_ENERGY_AT_LAST_CHANGE
    SFLOAT  available_energy_at_last_change;    // The available energy of the battery in kilowatt-hours in its last charge state.
#endif
} batt_energy_status_t;

#endif // BATTERY_ENERGY_STATUS

#ifdef BATTERY_TIME_STATUS

#ifdef BATTERY_TIME_STATUS_FLAG_TIME_DISCHARGED_ON_STANDBY
 #define BATTERY_TIME_STATUS_FLAG_TIME_DISCHARGED_ON_STANDBY_PRESENT (1<<0)
#else
 #define BATTERY_TIME_STATUS_FLAG_TIME_DISCHARGED_ON_STANDBY_PRESENT 0
#endif
#ifdef BATTERY_TIME_STATUS_FLAG_TIME_UNTIL_RECHARGED
 #define BATTERY_TIME_STATUS_FLAG_TIME_UNTIL_RECHARGED_PRESENT (1<<1)
#else
 #define BATTERY_TIME_STATUS_FLAG_TIME_UNTIL_RECHARGED_PRESENT 0
#endif

#define BATTERY_TIME_STATUS_FLAG ( \
    BATTERY_TIME_STATUS_FLAG_TIME_DISCHARGED_ON_STANDBY_PRESENT | \
    BATTERY_TIME_STATUS_FLAG_TIME_UNTIL_RECHARGED_PRESENT)

typedef struct {
    UINT8   flags;
    UINT24  time_until_discharged;
#ifdef BATTERY_TIME_STATUS_FLAG_TIME_DISCHARGED_ON_STANDBY
    UINT24  time_until_discharged_on_standby;
#endif
#ifdef BATTERY_TIME_STATUS_FLAG_TIME_UNTIL_RECHARGED
    UINT24  time_until_recharged;
#endif
} batt_time_status_t;
#endif //  BATTERY_TIME_STATUS

#ifdef BATTERY_HEALTH_STATUS

#ifdef BATTERY_HEALTH_STATUS_FLAG_SUMMARY
 #define BATTERY_HEALTH_STATUS_FLAG_SUMMARY_PRESENT (1<<0)
#else
 #define BATTERY_HEALTH_STATUS_FLAG_SUMMARY_PRESENT 0
#endif
#ifdef BATTERY_HEALTH_STATUS_FLAG_CYCLE_COUNT
 #define BATTERY_HEALTH_STATUS_FLAG_CYCLE_COUNT_PRESENT (1<<1)
#else
 #define BATTERY_HEALTH_STATUS_FLAG_CYCLE_COUNT_PRESENT 0
#endif
#ifdef BATTERY_HEALTH_STATUS_FLAG_CURRENT_TEMP
 #define BATTERY_HEALTH_STATUS_FLAG_CURRENT_TEMP_PRESENT (1<<2)
#else
 #define BATTERY_HEALTH_STATUS_FLAG_CURRENT_TEMP_PRESENT 0
#endif
#ifdef BATTERY_HEALTH_STATUS_FLAG_DEEP_DISCHARGE_COUNT
 #define BATTERY_HEALTH_STATUS_FLAG_DEEP_DISCHARGE_COUNT_PRESENT (1<<3)
#else
 #define BATTERY_HEALTH_STATUS_FLAG_DEEP_DISCHARGE_COUNT_PRESENT 0
#endif

#define BATTERY_HEALTH_STATUS_FLAG ( \
    BATTERY_HEALTH_STATUS_FLAG_SUMMARY_PRESENT | \
    BATTERY_HEALTH_STATUS_FLAG_CYCLE_COUNT_PRESENT | \
    BATTERY_HEALTH_STATUS_FLAG_CURRENT_TEMP_PRESENT | \
    BATTERY_HEALTH_STATUS_FLAG_DEEP_DISCHARGE_COUNT_PRESENT)

typedef struct {
    UINT8   flags;
#ifdef BATTERY_HEALTH_STATUS_FLAG_SUMMARY
    UINT8   health_summary;
#endif
#ifdef BATTERY_HEALTH_STATUS_FLAG_CYCLE_COUNT
    UINT16  cycle_count;
#endif
#ifdef BATTERY_HEALTH_STATUS_FLAG_CURRENT_TEMP
    UINT8   current_temp;
#endif
#ifdef BATTERY_HEALTH_STATUS_FLAG_DEEP_DISCHARGE_COUNT
    UINT16  deep_discharge_count;
#endif
} batt_health_status_t;

#endif // BATTERY_HEALTH_STATUS

#ifdef BATTERY_HEALTH_INFO
#ifdef BATTERY_HEALTH_INFO_CYCLE_COUNT_DESIGNED_LIFETIME
 #define BATTERY_HEALTH_INFO_CYCLE_COUNT_DESIGNED_LIFETIME_PRESENT (1<<0)
#else
 #define BATTERY_HEALTH_INFO_CYCLE_COUNT_DESIGNED_LIFETIME_PRESENT 0
#endif
#ifdef BATTERY_HEALTH_INFO_MIN_DESIGNED_OP_TEMP
 #define BATTERY_HEALTH_INFO_MIN_DESIGNED_OP_TEMP_PRESENT (1<<1)
#else
 #define BATTERY_HEALTH_INFO_MIN_DESIGNED_OP_TEMP_PRESENT 0
#endif

#define BATTERY_HEALTH_INFO_FLAG ( \
    BATTERY_HEALTH_INFO_CYCLE_COUNT_DESIGNED_LIFETIME_PRESENT | \
    BATTERY_HEALTH_INFO_MIN_DESIGNED_OP_TEMP_PRESENT )

typedef struct {
    UINT8   flags;
#ifdef BATTERY_HEALTH_INFO_CYCLE_COUNT_DESIGNED_LIFETIME
    UINT16  cycle_count_designed_lifetime;
#endif
#ifdef BATTERY_HEALTH_STATUS_FLAG_CURRENT_TEMP
    UINT8   min_designed_op_temp;
    UINT8   max_designed_op_temp;
#endif
} batt_health_info_t;
#endif // #ifdef BATTERY_HEALTH_INFO

#ifdef BATTERY_INFO

typedef struct {
    UINT8  batt_replaceable: 1;             // Battery replaceable by user 0:False, 1: True
    UINT8  batt_rechargeable: 1;            // Battery rechargeable in the system  0:False, 1: True
    UINT8  rfu: 6;
} batt_features_t;


#ifdef BATTERY_INFO_FLAG_BATT_MANUF_DATE
 #define BATTERY_INFO_FLAG_BATT_MANUF_DATE_PRESENT (1<<0)
#else
 #define BATTERY_INFO_FLAG_BATT_MANUF_DATE_PRESENT 0
#endif
#ifdef BATTERY_INFO_FLAG_BATT_EXPRATION_DATE
 #define BATTERY_INFO_FLAG_BATT_EXPRATION_DATE_PRESENT (1<<1)
#else
 #define BATTERY_INFO_FLAG_BATT_EXPRATION_DATE_PRESENT 0
#endif
#ifdef BATTERY_INFO_FLAG_BATT_DESIGNED_CAP
 #define BATTERY_INFO_FLAG_BATT_DESIGNED_CAP_PRESENT (1<<2)
#else
 #define BATTERY_INFO_FLAG_BATT_DESIGNED_CAP_PRESENT 0
#endif
#ifdef BATTERY_INFO_FLAG_BATT_LOW_ENGERGY
 #define BATTERY_INFO_FLAG_BATT_LOW_ENGERGY_PRESENT (1<<3)
#else
 #define BATTERY_INFO_FLAG_BATT_LOW_ENGERGY_PRESENT 0
#endif
#ifdef BATTERY_INFO_FLAG_BATT_CRITICAL_ENERGY
 #define BATTERY_INFO_FLAG_BATT_CRITICAL_ENERGY_PRESENT (1<<4)
#else
 #define BATTERY_INFO_FLAG_BATT_CRITICAL_ENERGY_PRESENT 0
#endif
#ifdef BATTERY_INFO_FLAG_BATT_CHEMISTRY
 #define BATTERY_INFO_FLAG_BATT_CHEMISTRY_PRESENT (1<<5)
#else
 #define BATTERY_INFO_FLAG_BATT_CHEMISTRY_PRESENT 0
#endif
#ifdef BATTERY_INFO_FLAG_NOMINAL_VOLTAGE
 #define BATTERY_INFO_FLAG_NOMINAL_VOLTAGE_PRESENT (1<<6)
#else
 #define BATTERY_INFO_FLAG_NOMINAL_VOLTAGE_PRESENT 0
#endif
#ifdef BATTERY_INFO_FLAG_BATT_AGGR_GRP
 #define BATTERY_INFO_FLAG_BATT_AGGR_GRP_PRESENT (1<<7)
#else
 #define BATTERY_INFO_FLAG_BATT_AGGR_GRP_PRESENT 0
#endif

#define BATTERY_INFO_FLAG ( \
    BATTERY_INFO_FLAG_BATT_MANUF_DATE_PRESENT | \
    BATTERY_INFO_FLAG_BATT_EXPRATION_DATE_PRESENT | \
    BATTERY_INFO_FLAG_BATT_DESIGNED_CAP_PRESENT | \
    BATTERY_INFO_FLAG_BATT_LOW_ENGERGY_PRESENT | \
    BATTERY_INFO_FLAG_BATT_CRITICAL_ENERGY_PRESENT | \
    BATTERY_INFO_FLAG_BATT_CHEMISTRY_PRESENT | \
    BATTERY_INFO_FLAG_NOMINAL_VOLTAGE_PRESENT | \
    BATTERY_INFO_FLAG_BATT_AGGR_GRP_PRESENT)

typedef struct {
    UINT16   flags;
    batt_features_t    battery_features;
#ifdef BATTERY_INFO_FLAG_BATT_MANUF_DATE
    UINT24   batt_manuf_date;
#endif
#ifdef BATTERY_INFO_FLAG_BATT_EXPRATION_DATE
    UINT24   batt_expr_date;
#endif
#ifdef BATTERY_INFO_FLAG_BATT_DESIGNED_CAP
    SFLOAT   batt_designed_cap;
#endif
#ifdef BATTERY_INFO_FLAG_BATT_LOW_ENGERGY
    SFLOAT   batt_low_energy;
#endif
#ifdef BATTERY_INFO_FLAG_BATT_CRITICAL_ENERGY
    SFLOAT   batt_critical_energy;
#endif
#ifdef BATTERY_INFO_FLAG_BATT_CHEMISTRY
    UINT8    batt_chemistry;
#endif
#ifdef BATTERY_INFO_FLAG_NOMINAL_VOLTAGE
    SFLOAT   batt_nominal_volage;
#endif
#ifdef BATTERY_INFO_FLAG_BATT_AGGR_GRP
    UINT8    batt_aggr_grp;
#endif
} batt_info_t;

#endif // BATTERY_INFO

#pragma pack()

#endif // BAS_1_1

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // BATTERY_SERVER_LIB_H
