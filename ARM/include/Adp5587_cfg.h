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
**     Adp5587_cfg.h
**
** MODULE FUNCTION:
**      Configuration file for the ADP5587 component to be included by 
**      the user. 
**      This file is required and should be modified by the user.  
**
************************************************************************************
*/

#ifndef ADP5587_CFG_H_
#define ADP5587_CFG_H_

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/
 
/* None */

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

/* 
 * This macro allows for run-time error checking of the read and write pin API 
 * interfaces. This can be used to validate input parameters, etc. at run-time 
 * for invalid settings. If users don't want to validate input parameters at run-time
 * (for extremely critical timing), leave this macro disabled.
 * 
 * Valid options are below (Do NOT remove these!) 
 */
#define C_ADP5587_RUN_TIME_ERROR_CHECK_DISABLED (0U)
#define C_ADP5587_RUN_TIME_ERROR_CHECK_ENABLED  (1U)

#define C_ADP5587_RUN_TIME_ERROR_CHECK          (C_ADP5587_RUN_TIME_ERROR_CHECK_ENABLED)

/*
 * Defines the available devices for communication
 * with multiple devices. The names of the enumeration values are 
 * defined by the user/integrator of this module, where the
 * naming/format may align to:
 * <E_ADP5587_DEVICE_ID_xxx>, where xxx is a descriptive 
 * name for the device.
 *
 */
typedef enum
{
   /* User defined enumerations */
   E_ADP5587_DEVICE_SC598_REV_E_SOM,           /* EV-SC598-SOM Rev E Device */
   E_ADP5587_DEVICE_SC598_REV_E_SOM_HW_PROBE,  /* EV-SC598-SOM Rev E Device Minimal Setting during HW Check */

   /* Must always be in the list and last */
   E_ADP5587_DEVICE_ID_MAX  
}T_ADP5587_DEVICE_ID;

#endif /* ADP5587_CFG_H_ */
