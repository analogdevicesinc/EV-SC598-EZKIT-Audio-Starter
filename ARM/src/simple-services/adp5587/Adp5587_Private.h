/**
 * Copyright (c) 2025 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/***********************************************************************************
**
** MODULE TITLE:
**     Adp5587_Private.h
**
** MODULE FUNCTION:
**      This is the private header file of the ADP5587 driver source file. 
**      This file should NOT be modified by the user. 
**
************************************************************************************
*/

#ifndef ADP5587_PRIVATE_H_
#define ADP5587_PRIVATE_H_

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/
 
#include <stdbool.h>
#include "Adp5587_cfg.h"
 
/***********************************************************************************
*  CONFIGURATION VERIFICATION
***********************************************************************************/

/* None */

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

 /* None */
 
/***********************************************************************************
 *  GLOBAL VARIABLES
 ***********************************************************************************/
 
extern const T_ADP5587_DEVICE_CONFIG asAdp5587ConfigRegistry[E_ADP5587_DEVICE_ID_MAX];

/***********************************************************************************
 *  GLOBAL PROTOTYPES
 **********************************************************************************/

extern void Adp5587_CfgIf_ReportStatus(const T_ADP5587_STATUS eStatus);
extern bool Adp5587_CfgIf_I2cWrite(const T_ADP5587_DEVICE_ID eDeviceId, const uint8_t * const pu8Data, const uint8_t u8Length);
extern bool Adp5587_CfgIf_I2cRead(const T_ADP5587_DEVICE_ID eDeviceId, const uint8_t * const pu8WData, const uint8_t u8WLength, uint8_t * const pu8RData, const uint8_t u8RLength);

#endif /* ADP5587_PRIVATE_H_ */
