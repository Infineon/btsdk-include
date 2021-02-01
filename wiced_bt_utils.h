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
 *
 * Bluetooth WICED Utility functions
 *
 */

#ifndef __WICED_BT_UTILS_H
#define __WICED_BT_UTILS_H

#include "wiced_bt_types.h"

/*******************************************************************
** WICED BT related definitions and declarations
*******************************************************************/
extern void GKI_freebuf (void *memPtr);
typedef wiced_bt_device_address_t BD_ADDR;
extern BD_ADDR bd_addr_any;
extern BD_ADDR bd_addr_null;

void utl_freebuf(void **p);
void utl_bdcpy(BD_ADDR a, const BD_ADDR b);
int utl_bdcmp(const BD_ADDR a, const BD_ADDR b);
int utl_bdcmpany(const BD_ADDR a);
void utl_bdsetany(BD_ADDR a);

/*******************************************************************
** string and character related definitions and declarations
*******************************************************************/
int utl_strncmp(const char *s1, const char *s2, int n);
char *utl_strcat(char *s1, const char *s2);
char *utl_strcpy(char *p_dst, const char *p_src);
char *utl_strrchr(char *s, int c);
int16_t utl_str2int(const char *p_s);
int utl_strucmp(const char *p_s, const char *p_t);
unsigned long utl_strtoul(const char *nptr, char **endptr, int base);
char utl_toupper(char c);
uint8_t utl_itoa(uint16_t i, char *p_s);

#if defined (CYW20706A2) || defined (CYW43012C0)
// strchr defined in ROM for most chips, define here for the missing ones
#define strchr(x,y) utl_strchr(x,y)
char *utl_strchr(const char *s, int c);
#endif
#endif // __WICED_BT_UTILS_H
