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
**     Adp5587.h
**
** MODULE FUNCTION:
**     This is the public interface header for the ADP5587 driver component.
**     This file is required and should NOT be modified by the user. 
**
**     The purpose of this module is to provide a modular and scalable interface to 
**     the ADP5587 I2C to GPIO extender IC. This module provides GPIO only
**     (read/write only) interfacing and only supports I2C as the interface.
**     Interrupt handling is NOT supported by this module. 
**      
**     This module may be used for the ADP5588 as well but should not be used
**     for any other parts in the same part number family. 
**
************************************************************************************
*/

#ifndef ADP5587_H_
#define ADP5587_H_

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/
 
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "Adp5587_cfg.h"

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/
 
typedef enum
{
   E_ADP5587_STATUS_OK,
   E_ADP5587_STATUS_NOT_INITALIZED,
   E_ADP5587_STATUS_INVALID_DEVICE_ID,
   E_ADP5587_STATUS_INVALID_PORT_ID,
   E_ADP5587_STATUS_INVALID_PIN_ID,
   E_ADP5587_STATUS_NULL_PTR,
   E_ADP5587_STATUS_CFGIF_ERROR,
   E_ADP5587_STATUS_ALREADY_INITIALIZED,
   E_ADP5587_STATUS_ALREADY_UNINITIALIZED,
   E_ADP5587_STATUS_NO_SYSTEM_MAP,
   E_ADP5587_STATUS_PIN_NOT_FOUND,
   E_ADP5587_STATUS_PIN_NOT_SUPPORTED,
}T_ADP5587_STATUS;

typedef enum
{
    E_ADP5587_PORT_R,
    E_ADP5587_PORT_C,
    E_ADP5587_PORT_MAX
}T_ADP5587_PORT;

typedef enum
{
    E_ADP5587_PIN_0,
    E_ADP5587_PIN_1,
    E_ADP5587_PIN_2,
    E_ADP5587_PIN_3,
    E_ADP5587_PIN_4,
    E_ADP5587_PIN_5,
    E_ADP5587_PIN_6,
    E_ADP5587_PIN_7,
    E_ADP5587_PIN_8, /* Only avaible on port C */
    E_ADP5587_PIN_9, /* Only avaible on port C */
    E_ADP5587_PIN_MAX
}T_ADP5587_PIN;

typedef struct
{
    int32_t s32PinId;
    T_ADP5587_PORT ePort;
    T_ADP5587_PIN  ePin;
}T_ADP5587_SYSTEM_MAP;

typedef struct 
{
    uint32_t u32PinDirMask;
    uint32_t u32PullUpDisMask;
    uint32_t u32PinInitValueMask;
    uint32_t u32InputDebounceDisMask;
    uint16_t u16PinMapLength;
    const T_ADP5587_SYSTEM_MAP * const psPinMap;
}T_ADP5587_DEVICE_CONFIG;


/***********************************************************************************
 *  GLOBAL PROTOTYPES
 **********************************************************************************/

/******************************************************************************
 *   FUNCTION:     Adp5587_Init
 *   DESCRIPTION:  This function resets the Adp5587 device to it's default state
 *                 and initializes the device according to the device settings
 *                 in the asAdp5587ConfigRegistry. Note that this device, by default
 *                 starts up with pull-ups enabled. Users should set the disable 
 *                 mask in the asAdp5587ConfigRegistry to disable these if 
 *                 pull-ups are not required. A value of '1' disables the pull-up
 *                 and a value of '0' enables the pull-up. 
 *                 Additionally, this device starts up with a 275us debounce enabled 
 *                 on inputs reads. Users should set the disable mask in the 
 *                 asAdp5587ConfigRegistry to disable if debounce is not required. 
 *                 A value of '1' disables this debounce and a value of '0' enables the
 *                 debounce. 
 *                 This device's nomenclature has inputs = 0 and outputs = 1 which is
 *                 not typical so users should take care when setting the GPIO
 *                 direction value in the asAdp5587ConfigRegistry.
 *              
 *                 This function is REQUIRED to be called during project initialization
 *                 to use this module. It is NOT thread/context safe.
 *                 Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The specific device in the registry to initialize
 *                             and configure
 *   OUTPUT(S):    None  
 *   RETURN VALUE: T_ADP5587_STATUS      
 ******************************************************************************/
extern T_ADP5587_STATUS Adp5587_Init(const T_ADP5587_DEVICE_ID eDeviceId);

/******************************************************************************
 *   FUNCTION:     Adp5587_ReadPin
 *   DESCRIPTION:  This function reads the level on the requested hardware ICs
 *                 GPIO port/pin. Note that this device returns the value on 
 *                 the port/pin only when it is configured as an input. 
 *                 if the pin is configured as an output, the value read
 *                 from the device will be 0.
 *  
 *                 This function is NOT REQUIRED to be used. It is NOT thread/context
 *                 safe. Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The specific hardware device to read from
 *                 ePort     - The specific GPIO port (R/C) to read from
 *                 ePin      - The specic pin whose value is to be read
 *   OUTPUT(S):    pbPinVal  - A pointer to port/pin value read from the device 
 *   RETURN VALUE: T_ADP5587_STATUS
 ******************************************************************************/
extern T_ADP5587_STATUS Adp5587_ReadPin(const T_ADP5587_DEVICE_ID eDeviceId, const T_ADP5587_PORT ePort, const T_ADP5587_PIN ePin, bool * const pbPinVal);

/******************************************************************************
 *   FUNCTION:     Adp5587_WritePin
 *   DESCRIPTION:  This function writes a 1 or a 0 (high/low) to a specific
 *                 hardware ICs GPIO port/pin for those pins configured
 *                 as an output. 
 *  
 *                 This function is NOT REQUIRED to be used. It is NOT thread/context
 *                 safe. Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The specific hardware device to write to
 *                 ePort     - The specific GPIO port (R/C) to write to
 *                 ePin      - The specic pin whose value is to be written
 *                 bPinVal   - The value to write on the pin
 *   OUTPUT(S):    None  
 *   RETURN VALUE: T_ADP5587_STATUS
 ******************************************************************************/
extern T_ADP5587_STATUS Adp5587_WritePin(const T_ADP5587_DEVICE_ID eDeviceId, const T_ADP5587_PORT ePort, const T_ADP5587_PIN ePin, const bool bPinVal);

/******************************************************************************
 *   FUNCTION:     Adp5587_GetPortPin
 *   DESCRIPTION:  This retrieves the hardware port and pin configuration defined
 *                 in the asAdp5587ConfigRegistry table. This can be useful for 
 *                 applications that have an upper level abstraction for handling
 *                 multiple I2C to GPIO expanders in the system. This allows
 *                 the system level configuration to be maintained at the hardware
 *                 layer and keep a common upper layer for handling all of the 
 *                 pins in the system. This API is optional for users who 
 *                 wish to use this module by itself. Users may mark the psPinMap
 *                 parameter in the asAdp5587ConfigRegistry as NULL in this case and 
 *                 this API should NOT be called. 
 *  
 *                 This function is NOT REQUIRED to be used. It is NOT thread/context
 *                 safe. Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The device whose hardware port and pin is to be
 *                             retrieved. 
 *                 s32PinId  - The system ID representing the port and pin for 
 *                             the specific device.
 *   OUTPUT(S):    pePort    - A pointer to the port representing the system
 *                             pin.
 *                 pePin     - A pointer to the pin representing the system 
 *                             pin. 
 *   RETURN VALUE: T_ADP5587_STATUS
 ******************************************************************************/
extern T_ADP5587_STATUS Adp5587_GetPortPin(const T_ADP5587_DEVICE_ID eDeviceId, int32_t s32PinId, T_ADP5587_PORT * const pePort, T_ADP5587_PIN * const pePin);

/******************************************************************************
 *   FUNCTION:     Adp5587_DeInit
 *   DESCRIPTION:  This function resets the requested device to it's specific
 *                 POR reset values. 
 *              
 *                 This function is NOT REQUIRED to be called if system
 *                 deinitialization is not required. 
 *                 It is NOT thread/context safe.
 *                 Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The device to deinitialize/reset      
 *   OUTPUT(S):    None  
 *   RETURN VALUE: T_ADP5587_STATUS
 ******************************************************************************/
extern T_ADP5587_STATUS Adp5587_DeInit(const T_ADP5587_DEVICE_ID eDeviceId);

#endif /* ADP5587_H_ */
