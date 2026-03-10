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
**     Adp5587_cfg.c
**
** MODULE FUNCTION:
**      This is the project specific configurable portion of the Adp5587 
**      component. This file includes configurable interface functions which
**      should be filled in based on the users project specific needs as well
**      as the device specific registration table and pin mapping for each 
**      device. 
**      This file is required and should be modified by the user. 
**
************************************************************************************
*/

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/

#include "Adp5587.h"

/* Project specific includes */
#include "context.h"
#include "twi_simple.h"
#include "ss.h"
      
/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

/***********************************************************************************
 *                          SC598 SOM Rev E Details
 **********************************************************************************/

#if defined(SDCARD_USE_EMMC) /* USING EMMC ON BOARD SOM */
 /* Port C                                   Port R   
    9------------------- DS1
    | 8----------------- DS2
    | | 7--------------- DS3                 |   7--------------- NOT USED
    | | | 6------------- NOT USED            |   | 6------------- NOT USED
    | | | | 5----------- NOT USED            |   | | 5----------- NOT USED
    | | | | | 4--------- NOT USED            |   | | | 4--------- NOT USED
    | | | | | | 3------- NOT USED            |   | | | | 3------- ~SPI2FLASH_CS_EN
    | | | | | | | 2----- NOT USED            |   | | | | | 2----- ~SPID2_D3_EN
    | | | | | | | | 1--- ~EMMC_SOM_EN        |   | | | | | | 1--- ~UART0_FLOW_EN
    | | | | | | | | | 0- ~EMMC_EN            |   | | | | | | | 0- ~UART0_EN
    | | | | | | | | | |                      |   | | | | | | | |
    N N N X X X X X N Y                      |   X X X X Y Y N Y            ( Active Y or N - After applying these settings )
    1 1 1 0 0 0 0 0 1 1  (0x383)             |   0 0 0 0 1 1 1 1 (0x0F)     ( DIR_VALUE  = GPIO Direction 1 = Output, 0 = Input )
    1 1 1 0 0 0 0 0 1 0  (0x382)             |   0 0 0 0 0 0 1 0 (0x02)     ( INIT_VALUE = Value being set )
    1 1 1 1 1 1 1 1 1 1  (0x3FF)             |   1 1 1 1 1 1 1 1 (0xFF)     ( PUP_DIS_VALUE = Pull-up disable value - pull-ups disabled)
    1 1 1 1 1 1 1 1 1 1  (0x3FF)             |   1 1 1 1 1 1 1 1 (0xFF)     ( DEBOUNCE_DIS_VALUE = Debounce disable value - debounce disabled)
*/

/* 
 * Initial values are U32 value or'd values from above 10-bit and 8-bit values - PORTC | PORTR.
 * Note the following settings: 
 * GPIO Direction Setting: 1 = Output,   0 = Input (Default setting is 0 = input)
 * Pull-Up Setting       : 1 = Disabled, 0 = Enabled (Default setting is 0 = enabled)
 * Debounce Setting      : 1 = Disabled, 0 = Enabled @275us (Default setting is 0 = enabled) 
 */
#define C_SC598_REVE_DIR_VALUE           (0x0003830FU)
#define C_SC598_REVE_INIT_VALUE          (0x00038202U)
#define C_SC598_REVE_PUP_DIS_VALUE       (0x0003FFFFU)
#define C_SC598_REVE_DEBOUNCE_DIS_VALUE  (0x0003FFFFU)

#else
 /* Port C                                   Port R   
    9------------------- DS1
    | 8----------------- DS2
    | | 7--------------- DS3                 |   7--------------- NOT USED
    | | | 6------------- NOT USED            |   | 6------------- NOT USED
    | | | | 5----------- NOT USED            |   | | 5----------- NOT USED
    | | | | | 4--------- NOT USED            |   | | | 4--------- NOT USED
    | | | | | | 3------- NOT USED            |   | | | | 3------- ~SPI2FLASH_CS_EN
    | | | | | | | 2----- NOT USED            |   | | | | | 2----- ~SPID2_D3_EN
    | | | | | | | | 1--- ~EMMC_SOM_EN        |   | | | | | | 1--- ~UART0_FLOW_EN
    | | | | | | | | | 0- ~EMMC_EN            |   | | | | | | | 0- ~UART0_EN
    | | | | | | | | | |                      |   | | | | | | | |
    N N N X X X X X Y N                      |   X X X X Y Y N Y            ( Active Y or N - After applying these settings )
    1 1 1 0 0 0 0 0 1 1  (0x383)             |   0 0 0 0 1 1 1 1 (0x0F)     ( DIR_VALUE  = GPIO Direction 1 = Output, 0 = Input )
    1 1 1 0 0 0 0 0 0 1  (0x381)             |   0 0 0 0 0 0 1 0 (0x02)     ( INIT_VALUE = Value being set )
    1 1 1 1 1 1 1 1 1 1  (0x3FF)             |   1 1 1 1 1 1 1 1 (0xFF)     ( PUP_DIS_VALUE = Pull-up disable value - pull-ups disabled)
    1 1 1 1 1 1 1 1 1 1  (0x3FF)             |   1 1 1 1 1 1 1 1 (0xFF)     ( DEBOUNCE_DIS_VALUE = Debounce disable value - debounce disabled)
*/

/* 
 * Initial values are U32 value or'd values from above 10-bit and 8-bit values - PORTC | PORTR.
 * Note the following settings: 
 * GPIO Direction Setting: 1 = Output,   0 = Input (Default setting is 0 = input)
 * Pull-Up Setting       : 1 = Disabled, 0 = Enabled (Default setting is 0 = enabled)
 * Debounce Setting      : 1 = Disabled, 0 = Enabled @275us (Default setting is 0 = enabled) 
 */
#define C_SC598_REVE_DIR_VALUE           (0x0003830FU)
#define C_SC598_REVE_INIT_VALUE          (0x00038102U)
#define C_SC598_REVE_PUP_DIS_VALUE       (0x0003FFFFU)
#define C_SC598_REVE_DEBOUNCE_DIS_VALUE  (0x0003FFFFU)

#endif /* USING CARRIER SD CARD */

 /* Port C                                   Port R   
    9------------------- DS1
    | 8----------------- DS2
    | | 7--------------- DS3                 |   7--------------- NOT USED
    | | | 6------------- NOT USED            |   | 6------------- NOT USED
    | | | | 5----------- NOT USED            |   | | 5----------- NOT USED
    | | | | | 4--------- NOT USED            |   | | | 4--------- NOT USED
    | | | | | | 3------- NOT USED            |   | | | | 3------- ~SPI2FLASH_CS_EN //THIS ONE
    | | | | | | | 2----- NOT USED            |   | | | | | 2----- ~SPID2_D3_EN
    | | | | | | | | 1--- ~EMMC_SOM_EN        |   | | | | | | 1--- ~UART0_FLOW_EN
    | | | | | | | | | 0- ~EMMC_EN            |   | | | | | | | 0- ~UART0_EN
    | | | | | | | | | |                      |   | | | | | | | |
    X X X X X X X X X X                      |   X X X X Y X X X            ( Active Y or N - After applying these settings )
    0 0 0 0 0 0 0 0 0 0  (0x000)             |   0 0 0 0 1 0 0 0 (0x08)     ( DIR_VALUE  = GPIO Direction 1 = Output, 0 = Input )
    0 0 0 0 0 0 0 0 0 0  (0x000)             |   0 0 0 0 0 0 0 0 (0x00)     ( INIT_VALUE = Value being set )
    0 0 0 0 0 0 0 0 0 0  (0x000)             |   0 0 0 0 0 0 0 0 (0x00)     ( PUP_DIS_VALUE = Pull-up disable value - pull-ups disabled)
    0 0 0 0 0 0 0 0 0 0  (0x000)             |   0 0 0 0 0 0 0 0 (0x00)     ( DEBOUNCE_DIS_VALUE = Debounce disable value - debounce disabled)
*/

/* 
 * Initial values are U32 value or'd values from above 10-bit and 8-bit values - PORTC | PORTR.
 * Note the following settings: 
 * GPIO Direction Setting: 1 = Output,   0 = Input (Default setting is 0 = input)
 * Pull-Up Setting       : 1 = Disabled, 0 = Enabled (Default setting is 0 = enabled)
 * Debounce Setting      : 1 = Disabled, 0 = Enabled @275us (Default setting is 0 = enabled) 
 */
#define C_SC598_REVE_PROBE_DIR_VALUE           (0x00000008U)
#define C_SC598_REVE_PROBE_INIT_VALUE          (0x00000000U)
#define C_SC598_REVE_PROBE_PUP_DIS_VALUE       (0x00000000U)
#define C_SC598_REVE_PROBE_DEBOUNCE_DIS_VALUE  (0x00000000U)

/***********************************************************************************
 *  FUNCTION PROTOTYPES
 **********************************************************************************/

 /* None */

/***********************************************************************************
 *  MODULE VARIABLES
 **********************************************************************************/

/* This is the system configuration pin mapping for SC598 Rev E SOM */
static const T_ADP5587_SYSTEM_MAP sPinMapSoMSC598RevE[] = 
{
    { .s32PinId = (int32_t)SS_PIN_ID_nUART0_EN,        .ePort = E_ADP5587_PORT_R, .ePin = E_ADP5587_PIN_0 },
    { .s32PinId = (int32_t)SS_PIN_ID_nUART0_FLOW_EN,   .ePort = E_ADP5587_PORT_R, .ePin = E_ADP5587_PIN_1 },
    { .s32PinId = (int32_t)SS_PIN_ID_nSPID2_D3_EN,     .ePort = E_ADP5587_PORT_R, .ePin = E_ADP5587_PIN_2 },
    { .s32PinId = (int32_t)SS_PIN_ID_nSPI2FLASH_CS_EN, .ePort = E_ADP5587_PORT_R, .ePin = E_ADP5587_PIN_3 },
    { .s32PinId = (int32_t)SS_PIN_ID_DS3,              .ePort = E_ADP5587_PORT_C, .ePin = E_ADP5587_PIN_7 },
    { .s32PinId = (int32_t)SS_PIN_ID_DS2,              .ePort = E_ADP5587_PORT_C, .ePin = E_ADP5587_PIN_8 },
    { .s32PinId = (int32_t)SS_PIN_ID_DS1,              .ePort = E_ADP5587_PORT_C, .ePin = E_ADP5587_PIN_9 },
    { .s32PinId = (int32_t)SS_PIN_ID_EMMC_SOM_EN,      .ePort = E_ADP5587_PORT_C, .ePin = E_ADP5587_PIN_1 },
    { .s32PinId = (int32_t)SS_PIN_ID_EMMC_EN,          .ePort = E_ADP5587_PORT_C, .ePin = E_ADP5587_PIN_0 },
};

/* This is the system configuration pin mapping for SC598 Rev E SOM During the HW Probe Check */
static const T_ADP5587_SYSTEM_MAP sPinMapSoMSC598RevEHwCheck[] = 
{
    { .s32PinId = (int32_t)SS_PIN_ID_nSPI2FLASH_CS_EN, .ePort = E_ADP5587_PORT_R, .ePin = E_ADP5587_PIN_3 },
};

/* 
 * App context for TWI Handles 
 */
static APP_CONTEXT *context = &mainAppContext;

/* 
 * Module TWI Addresses 
 */
static const uint8_t au8AdpTwiAddresses[E_ADP5587_DEVICE_ID_MAX] = {0x34U, 0x34U};

/* 
 * Device specific configurations
 */
const T_ADP5587_DEVICE_CONFIG asAdp5587ConfigRegistry[E_ADP5587_DEVICE_ID_MAX] = 
{
    /* E_ADP5587_DEVICE_SC598_REV_D_SOM */ 
    {
        .u32PinDirMask = C_SC598_REVE_DIR_VALUE,  .u32PullUpDisMask = C_SC598_REVE_PUP_DIS_VALUE, .u32PinInitValueMask = C_SC598_REVE_INIT_VALUE,  .u32InputDebounceDisMask = C_SC598_REVE_DEBOUNCE_DIS_VALUE, .psPinMap = sPinMapSoMSC598RevE, .u16PinMapLength = sizeof(sPinMapSoMSC598RevE)/sizeof(T_ADP5587_SYSTEM_MAP)
    },
    /* E_ADP5587_DEVICE_SC598_REV_E_SOM_HW_PROBE */ 
    {
        .u32PinDirMask = C_SC598_REVE_PROBE_DIR_VALUE,  .u32PullUpDisMask = C_SC598_REVE_PROBE_PUP_DIS_VALUE, .u32PinInitValueMask = C_SC598_REVE_PROBE_INIT_VALUE,  .u32InputDebounceDisMask = C_SC598_REVE_PROBE_DEBOUNCE_DIS_VALUE, .psPinMap = sPinMapSoMSC598RevEHwCheck, .u16PinMapLength = sizeof(sPinMapSoMSC598RevEHwCheck)/sizeof(T_ADP5587_SYSTEM_MAP)
    },
};

/***********************************************************************************
 *  CONFIGURABLE INTERFACE FUNCTIONS
 **********************************************************************************/
 
/******************************************************************************
*   FUNCTION:     Adp5587_CfgIf_I2cWrite
*
*   DESCRIPTION:  This configurable interface function should be filled in 
*                 with the project specific I2C implementation for performing
*                 a synchronous write operation. User's should keep track 
*                 of their own device I2C address when calling this function
*                 as it is not provided as an input to this function.
*
*   INPUT(S):     eDeviceId - The hardware device to write to
*                 pu8Data   - A pointer to a buffer of I2C write data. The 
*                             data does NOT contain the I2C address or any
*                             read/write bits. That must be handled here or by
*                             the project specific I2C API. 
*                 u8Length  - The length of the I2C data buffer
*   OUTPUT(S):    None
*   RETURN VALUE: true if successful, false otherwise
******************************************************************************/
bool Adp5587_CfgIf_I2cWrite(const T_ADP5587_DEVICE_ID eDeviceId, const uint8_t * const pu8Data, const uint8_t u8Length)
{
    bool bSuccess;

    /* Local Inits */
    bSuccess = false;
    
    if(twi_write(context->softSwitchHandle, au8AdpTwiAddresses[eDeviceId], (uint8_t *)pu8Data, u8Length) == TWI_SIMPLE_SUCCESS)
    {
        bSuccess = true;
    }
    
    return(bSuccess);
}

/******************************************************************************
*   FUNCTION:     Adp5587_CfgIf_I2cRead
*
*   DESCRIPTION:  This configurable interface function should be filled in 
*                 with the project specific I2C implementation for performing
*                 a synchronous read operation. User's should keep track 
*                 of their own device I2C address when calling this function
*                 as it is not provided as an input to this function.
*
*   INPUT(S):     eDeviceId - The hardware device to read from
*                 pu8WData  - A pointer to a buffer of write data which 
*                             contains the device register address(es)
*                 u8WLength - The length of the write data
*                 u8RLength - The length of read data pointer buffer to 
*                             read into
*   OUTPUT(S):    pu8RData  - A pointer to a buffer to put the read 
*                             data into
*   RETURN VALUE: true if successful, false otherwise
******************************************************************************/
bool Adp5587_CfgIf_I2cRead(const T_ADP5587_DEVICE_ID eDeviceId, const uint8_t * const pu8WData, const uint8_t u8WLength, uint8_t * const pu8RData, const uint8_t u8RLength)
{
    bool bSuccess;

    /* Local Inits */
    bSuccess = false;
    
    if(twi_writeRead(context->softSwitchHandle, au8AdpTwiAddresses[eDeviceId], (uint8_t *)pu8WData, u8WLength, (uint8_t *)pu8RData, u8RLength) == TWI_SIMPLE_SUCCESS)
    {
        bSuccess = true;
    }
    
    return(bSuccess);
}

/******************************************************************************
 *   FUNCTION:     Adp5587_CfgIf_ReportStatus
 *   DESCRIPTION:  This function callback reports any non-OK statuses from 
 *                 the module. Useful for project specific assertions or 
 *                 traps.
 *   INPUT(S):     eStatus - Error Status
 *   OUTPUT(S):    None
 *   RETURN VALUE: None
 ******************************************************************************/
void Adp5587_CfgIf_ReportStatus(const T_ADP5587_STATUS eStatus)
{
    if((eStatus != E_ADP5587_STATUS_ALREADY_INITIALIZED) && (eStatus != E_ADP5587_STATUS_ALREADY_UNINITIALIZED))
    {
        asm("nop");
    }
}