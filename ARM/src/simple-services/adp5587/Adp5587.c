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
**     Adp5587.c
**
** MODULE FUNCTION:
**     This is the core implementation for the ADP5587 module. 
**     This file is required and should NOT be modified by the user. 
**
************************************************************************************
*/

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/

#include "Adp5587.h"
#include "Adp5587_Private.h"

/*******************************************************************************
 *  CONFIGURATION VERIFICATION
 ******************************************************************************/

#if !defined(C_ADP5587_RUN_TIME_ERROR_CHECK)
    #error "Missing C_ADP5587_RUN_TIME_ERROR_CHECK definition in Adp5587_cfg.h!"
#elif (C_ADP5587_RUN_TIME_ERROR_CHECK > C_ADP5587_RUN_TIME_ERROR_CHECK_ENABLED)
    #error "Invalid configuration setting for C_ADP5587_RUN_TIME_ERROR_CHECK. See Adp5587_cfg.h for details!"
#endif

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

/* Number of registers to reset */
#define C_ADP5587_NUM_REGS  (47U)

/* Configurable register addresses */
#define C_GPIO_DAT_STAT1_ADDR       (0x14U)
#define C_GPIO_DAT_STAT2_ADDR       (0x15U)
#define C_GPIO_DAT_STAT3_ADDR       (0x16U)
#define C_GPIO_DAT_OUT1_ADDR        (0x17U)
#define C_GPIO_DAT_OUT2_ADDR        (0x18U)
#define C_GPIO_DAT_OUT3_ADDR        (0x19U)
#define C_GPIO_DIR1_ADDR            (0x23U)
#define C_GPIO_DIR2_ADDR            (0x24U)
#define C_GPIO_DIR3_ADDR            (0x25U)
#define C_GPIO_PULL1_ADDR           (0x2CU)
#define C_GPIO_PULL2_ADDR           (0x2DU)
#define C_GPIO_PULL3_ADDR           (0x2EU)
#define C_DEBOUNCE_DIS1_ADDR        (0x29U)
#define C_DEBOUNCE_DIS2_ADDR        (0x2AU)
#define C_DEBOUNCE_DIS3_ADDR        (0x2BU)

/* Masks for handling U32 configurations port C (10 pins) and port R (8 pins) */
#define C_REG_PORTR_MASK       (0x000000FFU)
#define C_REG_PORTC_LOWER_MASK (0x0000FF00U)
#define C_REG_PORTC_UPPER_MASK (0x00030000U)

/*******************************************************************************
 *  MODULE VARIABLES
 ******************************************************************************/
 
static bool abInitialized[E_ADP5587_DEVICE_ID_MAX]           = {false};
static const uint8_t au8GpioInPortAddr[E_ADP5587_PORT_MAX]   = {C_GPIO_DAT_STAT1_ADDR, C_GPIO_DAT_STAT2_ADDR};
static const uint8_t au8GpioOutPortAddr[E_ADP5587_PORT_MAX]  = {C_GPIO_DAT_OUT1_ADDR, C_GPIO_DAT_OUT2_ADDR};

/***********************************************************************************
 *  FUNCTION PROTOTYPES
 **********************************************************************************/

static T_ADP5587_STATUS Adp5587_ResetDevice(const T_ADP5587_DEVICE_ID eDeviceId);
static T_ADP5587_STATUS Adp5587_ConfigureDevice(const T_ADP5587_DEVICE_ID eDeviceId);
static T_ADP5587_STATUS Adp5587_VerifyInputs(const T_ADP5587_DEVICE_ID eDeviceId, const T_ADP5587_PORT ePort, const T_ADP5587_PIN ePin, const bool * const pbPinVal);

/******************************************************************************
 *   DESCRIPTION:   Refer to Adp5587.h for detailed description.
 ******************************************************************************/
T_ADP5587_STATUS Adp5587_Init(const T_ADP5587_DEVICE_ID eDeviceId)
{
    T_ADP5587_STATUS eStatus;
    
    if(eDeviceId < E_ADP5587_DEVICE_ID_MAX)
    {
        if(abInitialized[eDeviceId] == false)
        { 
            /* Reset the registers to their default values */
            eStatus = Adp5587_ResetDevice(eDeviceId);

            /* Configure the device */
            if(eStatus == E_ADP5587_STATUS_OK)
            {
                eStatus = Adp5587_ConfigureDevice(eDeviceId);
                if(eStatus == E_ADP5587_STATUS_OK)
                {
                    abInitialized[eDeviceId] = true;
                }
            }
        }
        else
        {
            eStatus = E_ADP5587_STATUS_ALREADY_INITIALIZED;
            Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_ALREADY_INITIALIZED);
        }
    }
    else
    {
        eStatus = E_ADP5587_STATUS_INVALID_DEVICE_ID;
        Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_INVALID_DEVICE_ID);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to Adp5587.h for detailed description.
 ******************************************************************************/
T_ADP5587_STATUS Adp5587_DeInit(const T_ADP5587_DEVICE_ID eDeviceId)
{
    T_ADP5587_STATUS eStatus;
    
    if(eDeviceId < E_ADP5587_DEVICE_ID_MAX)
    {
        if(abInitialized[eDeviceId] == true)
        {
            eStatus = Adp5587_ResetDevice(eDeviceId);
            
            if(eStatus == E_ADP5587_STATUS_OK)
            {
                abInitialized[eDeviceId] = false;
            }
        }
        else
        {
            eStatus = E_ADP5587_STATUS_ALREADY_UNINITIALIZED;
            Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_ALREADY_UNINITIALIZED);
        }
    }
    else
    {
        eStatus = E_ADP5587_STATUS_INVALID_DEVICE_ID;
        Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_INVALID_DEVICE_ID);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to Adp5587.h for detailed description.
 ******************************************************************************/
T_ADP5587_STATUS Adp5587_ReadPin(const T_ADP5587_DEVICE_ID eDeviceId, const T_ADP5587_PORT ePort, const T_ADP5587_PIN ePin, bool * const pbPinVal)
{
    T_ADP5587_STATUS eStatus;
    uint8_t          u8BitPos;
    uint8_t          au8ReadBuff[1];
    uint8_t          au8WriteBuff[1];
    
    eStatus = Adp5587_VerifyInputs(eDeviceId, ePort, ePin, pbPinVal);
    
    if(eStatus == E_ADP5587_STATUS_OK)
    {
        au8WriteBuff[0] = (ePin > E_ADP5587_PIN_7) ? C_GPIO_DAT_STAT3_ADDR : au8GpioInPortAddr[ePort];
        if(Adp5587_CfgIf_I2cRead(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff), au8ReadBuff, sizeof(au8ReadBuff)) == true)
        {
            /* 
             * For pins 8 and 9 on Port C, those pins reside in bit0 and bit1 respectively - 
             * Otherwise the direct pin maps to the bit position accordingly. I.e., pin 0
             * maps to bit0, pin 1 maps to bit1, and so on.
             */
            u8BitPos  = (ePin > E_ADP5587_PIN_7) ? ((uint8_t)ePin % 8U) : (uint8_t)ePin;
            *pbPinVal = (bool)((au8ReadBuff[0] & (1U << u8BitPos)) >> u8BitPos);
        }
        else
        {
            eStatus = E_ADP5587_STATUS_CFGIF_ERROR;
            Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_CFGIF_ERROR);
        }
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to Adp5587.h for detailed description.
 ******************************************************************************/
T_ADP5587_STATUS Adp5587_WritePin(const T_ADP5587_DEVICE_ID eDeviceId, const T_ADP5587_PORT ePort, const T_ADP5587_PIN ePin, const bool bPinVal)
{
    T_ADP5587_STATUS eStatus;
    uint8_t          u8BitPos;
    uint8_t          u8PinMask;
    uint8_t          au8ReadBuff[1];
    uint8_t          au8WriteBuff[2];
    
    eStatus = Adp5587_VerifyInputs(eDeviceId, ePort, ePin, &bPinVal);
    
    if(eStatus == E_ADP5587_STATUS_OK)
    {
        /* Read back data first to mask with requested value */
        au8WriteBuff[0] = (ePin > E_ADP5587_PIN_7) ? C_GPIO_DAT_OUT3_ADDR : au8GpioOutPortAddr[ePort];
        if(Adp5587_CfgIf_I2cRead(eDeviceId, au8WriteBuff, 1U, au8ReadBuff, 1U) == true)
        {
            /* 
             * For pins 8 and 9 on Port C, those pins reside in bit0 and bit1 respectively - 
             * Otherwise the direct pin maps to the bit position accordingly. I.e., pin 0
             * maps to bit0, pin 1 maps to bit1, and so on.
             */
            u8BitPos = (ePin > E_ADP5587_PIN_7) ? ((uint8_t)ePin % 8U) : (uint8_t)ePin;

            if(bPinVal == true)
            {
                u8PinMask = au8ReadBuff[0] | (1U << u8BitPos);
            }
            else
            {
                u8PinMask = au8ReadBuff[0] & (~(1U << u8BitPos));
            }
            
            /* Write the data */
            au8WriteBuff[0] = (ePin > E_ADP5587_PIN_7) ? C_GPIO_DAT_OUT3_ADDR : au8GpioOutPortAddr[ePort];
            au8WriteBuff[1] = u8PinMask;
            if(Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) == false)
            {
                eStatus = E_ADP5587_STATUS_CFGIF_ERROR;
                Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_CFGIF_ERROR);
            }
        }
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to Adp5587.h for detailed description.
 ******************************************************************************/
T_ADP5587_STATUS Adp5587_GetPortPin(const T_ADP5587_DEVICE_ID eDeviceId, int32_t s32PinId, T_ADP5587_PORT * const pePort, T_ADP5587_PIN * const pePin)
{
    bool bPinFound;
    uint16_t u16PinMapIdx;
    uint16_t u16PinMapLength;
    T_ADP5587_STATUS eStatus;
    
    if(eDeviceId < E_ADP5587_DEVICE_ID_MAX)
    {
        if(abInitialized[eDeviceId] == true)
        {
            /* Search for a valid port and pin setting from the system map */
            if(asAdp5587ConfigRegistry[eDeviceId].psPinMap != NULL)
            {
                u16PinMapIdx    = 0U;
                u16PinMapLength = asAdp5587ConfigRegistry[eDeviceId].u16PinMapLength;
                do
                {
                    if(s32PinId == asAdp5587ConfigRegistry[eDeviceId].psPinMap[u16PinMapIdx].s32PinId)
                    {
                        bPinFound = true;
                        *pePort   = asAdp5587ConfigRegistry[eDeviceId].psPinMap[u16PinMapIdx].ePort;
                        *pePin    = asAdp5587ConfigRegistry[eDeviceId].psPinMap[u16PinMapIdx].ePin;
                        eStatus   = E_ADP5587_STATUS_OK;
                    }
                    u16PinMapIdx++;
                }while((bPinFound == false) && (u16PinMapIdx < u16PinMapLength));

                if(bPinFound == false)
                {
                    eStatus = E_ADP5587_STATUS_PIN_NOT_FOUND;
                    Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_PIN_NOT_FOUND);
                }
            }
            else
            {
                eStatus = E_ADP5587_STATUS_NO_SYSTEM_MAP;
                Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_NO_SYSTEM_MAP);
            }
        }
        else
        {
            eStatus = E_ADP5587_STATUS_NOT_INITALIZED;
            Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_NOT_INITALIZED);
        }
    }
    else
    {
        eStatus = E_ADP5587_STATUS_INVALID_DEVICE_ID;
        Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_INVALID_DEVICE_ID);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     Adp5587_ConfigureDevice
 *   DESCRIPTION:  Writes the device configurations for the specific hardware
 *                 device
 *   INPUT(S):     eDeviceId - Device to configure
 *   OUTPUT(S):    None
 *   RETURN VALUE: T_ADP5587_STATUS
 ******************************************************************************/
static T_ADP5587_STATUS Adp5587_ConfigureDevice(const T_ADP5587_DEVICE_ID eDeviceId)
{
    T_ADP5587_STATUS eStatus;
    bool             bSuccess;
    uint8_t          au8WriteBuff[2];
    
    /* Local Inits */
    eStatus = E_ADP5587_STATUS_OK;

    /* 
     * Write the device configuration - setting final direction in/out last to avoid 
     * spurious behavior on the outputs when changing the pins state/characteristics.
     */
    au8WriteBuff[0] = C_GPIO_PULL1_ADDR;
    au8WriteBuff[1] = (uint8_t)(asAdp5587ConfigRegistry[eDeviceId].u32PullUpDisMask & C_REG_PORTR_MASK);
    bSuccess        = Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff));

    au8WriteBuff[0] = C_GPIO_PULL2_ADDR;
    au8WriteBuff[1] = (uint8_t)((asAdp5587ConfigRegistry[eDeviceId].u32PullUpDisMask & C_REG_PORTC_LOWER_MASK) >> 8U);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_GPIO_PULL3_ADDR;
    au8WriteBuff[1] = (uint8_t)((asAdp5587ConfigRegistry[eDeviceId].u32PullUpDisMask & C_REG_PORTC_UPPER_MASK) >> 16U);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;
    
    au8WriteBuff[0] = C_DEBOUNCE_DIS1_ADDR;
    au8WriteBuff[1] = (uint8_t)(asAdp5587ConfigRegistry[eDeviceId].u32InputDebounceDisMask & C_REG_PORTR_MASK);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_DEBOUNCE_DIS2_ADDR;
    au8WriteBuff[1] = (uint8_t)((asAdp5587ConfigRegistry[eDeviceId].u32InputDebounceDisMask & C_REG_PORTC_LOWER_MASK) >> 8U);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_DEBOUNCE_DIS3_ADDR;
    au8WriteBuff[1] = (uint8_t)((asAdp5587ConfigRegistry[eDeviceId].u32InputDebounceDisMask & C_REG_PORTC_UPPER_MASK) >> 16U);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;
    
    au8WriteBuff[0] = C_GPIO_DAT_OUT1_ADDR;
    au8WriteBuff[1] = (uint8_t)(asAdp5587ConfigRegistry[eDeviceId].u32PinInitValueMask & C_REG_PORTR_MASK);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_GPIO_DAT_OUT2_ADDR;
    au8WriteBuff[1] = (uint8_t)((asAdp5587ConfigRegistry[eDeviceId].u32PinInitValueMask & C_REG_PORTC_LOWER_MASK) >> 8U);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_GPIO_DAT_OUT3_ADDR;
    au8WriteBuff[1] = (uint8_t)((asAdp5587ConfigRegistry[eDeviceId].u32PinInitValueMask & C_REG_PORTC_UPPER_MASK) >> 16U);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_GPIO_DIR1_ADDR;
    au8WriteBuff[1] = (uint8_t)(asAdp5587ConfigRegistry[eDeviceId].u32PinDirMask & C_REG_PORTR_MASK);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_GPIO_DIR2_ADDR;
    au8WriteBuff[1] = (uint8_t)((asAdp5587ConfigRegistry[eDeviceId].u32PinDirMask & C_REG_PORTC_LOWER_MASK) >> 8U);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_GPIO_DIR3_ADDR;
    au8WriteBuff[1] = (uint8_t)((asAdp5587ConfigRegistry[eDeviceId].u32PinDirMask & C_REG_PORTC_UPPER_MASK) >> 16U);
    bSuccess        = (bSuccess == true) ? Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;
    
    if(bSuccess == false)
    {
        eStatus = E_ADP5587_STATUS_CFGIF_ERROR;
        Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_CFGIF_ERROR);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     Adp5587_ResetDevice
 *   DESCRIPTION:  Resets device to POR values
 *   INPUT(S):     eDeviceId - Device to reset
 *   OUTPUT(S):    None
 *   RETURN VALUE: T_ADP5587_STATUS
 ******************************************************************************/
static T_ADP5587_STATUS Adp5587_ResetDevice(const T_ADP5587_DEVICE_ID eDeviceId)
{
    T_ADP5587_STATUS eStatus;
    bool             bSuccess;
    uint8_t          u8RegAddr;
    uint8_t          au8WriteBuff[2];
    
    /* Local Inits */
    eStatus   = E_ADP5587_STATUS_OK;
    u8RegAddr = 0U;
    
    do
    {
        au8WriteBuff[0] = u8RegAddr;
        au8WriteBuff[1] = 0U;
        bSuccess        = Adp5587_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff));
        u8RegAddr++;
    }while((bSuccess == true) && (u8RegAddr < C_ADP5587_NUM_REGS));
    
    if(bSuccess == false)
    {
        eStatus = E_ADP5587_STATUS_CFGIF_ERROR;
        Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_CFGIF_ERROR);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     Adp5587_VerifyInputs
 *   DESCRIPTION:  Verifies common API inputs for validity
 *   INPUT(S):     eDeviceId - The specific device being requested
 *   OUTPUT(S):    None
 *   RETURN VALUE: T_ADP5587_STATUS
 ******************************************************************************/
static T_ADP5587_STATUS Adp5587_VerifyInputs(const T_ADP5587_DEVICE_ID eDeviceId, const T_ADP5587_PORT ePort, const T_ADP5587_PIN ePin, const bool * const pbPinVal)
{
    T_ADP5587_STATUS eStatus;

#if(C_ADP5587_RUN_TIME_ERROR_CHECK == C_ADP5587_RUN_TIME_ERROR_CHECK_ENABLED)
    if(eDeviceId < E_ADP5587_DEVICE_ID_MAX)
    {
        if(abInitialized[eDeviceId] == true)
        {
            if(ePort < E_ADP5587_PORT_MAX)
            {
                if(ePin < E_ADP5587_PIN_MAX)
                {
                    if(((ePort == E_ADP5587_PORT_R) && (ePin < E_ADP5587_PIN_8)) || (ePort == E_ADP5587_PORT_C))
                    {
                        if(pbPinVal != NULL)
                        {
                            eStatus = E_ADP5587_STATUS_OK;
                        }
                        else
                        {
                            eStatus = E_ADP5587_STATUS_NULL_PTR;
                            Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_NULL_PTR);
                        }
                    }
                    else 
                    {
                        eStatus = E_ADP5587_STATUS_PIN_NOT_SUPPORTED;
                        Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_PIN_NOT_SUPPORTED);
                    }
                }
                else
                {
                    eStatus = E_ADP5587_STATUS_INVALID_PIN_ID;
                    Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_INVALID_PIN_ID);
                }
            }
            else
            {
                eStatus = E_ADP5587_STATUS_INVALID_PORT_ID;
                Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_INVALID_PORT_ID);
            }
        }
        else
        {
            eStatus = E_ADP5587_STATUS_NOT_INITALIZED;
            Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_NOT_INITALIZED);
        }
    }
    else
    {
        eStatus = E_ADP5587_STATUS_INVALID_DEVICE_ID;
        Adp5587_CfgIf_ReportStatus(E_ADP5587_STATUS_INVALID_DEVICE_ID);
    }
#else
    eStatus = E_ADP5587_STATUS_OK;
#endif
    
    return(eStatus);
}
