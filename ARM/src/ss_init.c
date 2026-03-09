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

#include "ss.h"
#include "ss_init.h"
#include "Mcp2301x.h"
#include "Adp5587.h"

/* MCP2301x Prototypes and function pointers */
static void ss_mcp_init(int deviceId);
static void ss_mcp_deinit(int deviceId);
static int ss_mcp_set(int deviceId, int portId, int pinId, bool value);
static int ss_mcp_get(int deviceId, int portId, int pinId, bool *value);
static int ss_mcp_get_portpin(int deviceId, int sysPinId, int * portId, int * pinId);

static const T_SS_HW_FP sSSFPMcp = 
{
    .pfSSInit       = ss_mcp_init, 
    .pfSSDeInit     = ss_mcp_deinit, 
    .pfSSGet        = ss_mcp_get, 
    .pfSSSet        = ss_mcp_set, 
    .pfSSGetPortPin = ss_mcp_get_portpin
};

 /* ADP5587 Prototypes and function pointers */
static void ss_adp_init(int deviceId);
static void ss_adp_deinit(int deviceId);
static int ss_adp_set(int deviceId, int portId, int pinId, bool value);
static int ss_adp_get(int deviceId, int portId, int pinId, bool *value);
static int ss_adp_get_portpin(int deviceId, int sysPinId, int * portId, int * pinId);

static const T_SS_HW_FP sSSFPAdp = 
{
    .pfSSInit       = ss_adp_init, 
    .pfSSDeInit     = ss_adp_deinit, 
    .pfSSGet        = ss_adp_get, 
    .pfSSSet        = ss_adp_set, 
    .pfSSGetPortPin = ss_adp_get_portpin
};

/* 
 * Hardware Configuration for all available pins on all soft switches -
 * Noting that anything that is switchable due to hardware differences in 
 * SOM or EZKIT revisions of the same type are updated during initialization 
 * below.
 * 
 * Note that the currently supported carrier boards all use the MCP2301X soft-switch
 * hardware ICs. Therefore, we don't need to update the function pointers, just the
 * device ID which distinguishes the settings for the soft-switch. However,
 * the currently supported SOMs vary in both soft-switch settings and hardware IC 
 * (MCP2301X or the ADP5587), therefore both the device ID and the function 
 * pointer will have to be updated during initialization (and is kept NULL at compilation).
 */
static T_SS_PIN_CONFIG asSSPinConfigRegistry[SS_PIN_ID_MAX] = 
{
   /* Carrier */
   {.ePinId = SS_PIN_ID_nADAU1979_EN,      .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nADAU_1962_EN,     .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nADAU_RESET,       .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nCAN_EN,           .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nFTDI_USB_EN,      .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nMicroSD_SPI,      .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_PUSHBUTTON_EN,     .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_EEPROM_EN,         .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nGIGe_RESET,       .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nETH1_RESET,       .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nETH1_EN,          .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nMLB_EN,           .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_AUDIO_JACK_SEL,    .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nSPDIF_OPTICAL_EN, .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_nSPDIF_DIGITAL_EN, .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   {.ePinId = SS_PIN_ID_OCTAL_SPI_CS_EN,   .deviceId = -1, .psHwFP = (T_SS_HW_FP *)&sSSFPMcp},
   /* SOM */
   {.ePinId = SS_PIN_ID_nUART0_FLOW_EN,    .deviceId = -1, .psHwFP = NULL                   },
   {.ePinId = SS_PIN_ID_nUART0_EN,         .deviceId = -1, .psHwFP = NULL                   },
   {.ePinId = SS_PIN_ID_nSPID2_D3_EN,      .deviceId = -1, .psHwFP = NULL                   },
   {.ePinId = SS_PIN_ID_nSPI2FLASH_CS_EN,  .deviceId = -1, .psHwFP = NULL                   },
   {.ePinId = SS_PIN_ID_DS1,               .deviceId = -1, .psHwFP = NULL                   },
   {.ePinId = SS_PIN_ID_DS2,               .deviceId = -1, .psHwFP = NULL                   },
   {.ePinId = SS_PIN_ID_DS3,               .deviceId = -1, .psHwFP = NULL                   },
   {.ePinId = SS_PIN_ID_EMMC_EN,           .deviceId = -1, .psHwFP = NULL                   },
   {.ePinId = SS_PIN_ID_EMMC_SOM_EN,       .deviceId = -1, .psHwFP = NULL                   },
};

bool ss_get(APP_CONTEXT *context, int pinId, bool *value)
{
    bool bSuccess;

    /* Local Inits */
    bSuccess = false;

    if(softswitch_get(pinId, value) == E_SS_STATUS_OK)
    {
        bSuccess = true;
    }

    return(bSuccess);
}

bool ss_set(APP_CONTEXT *context, int pinId, bool value)
{
    bool bSuccess;

    /* Local Inits */
    bSuccess = false;

    if(softswitch_set(pinId, value) == E_SS_STATUS_OK)
    {
        bSuccess = true;
    }
    
    return(bSuccess);
}

void ss_init(APP_CONTEXT *context)
{
    uint8_t      u8PinIdx;
    int          hwSoMDeviceId;
    int          hwCarrierDeviceId;
    T_SS_HW_FP * hwSoftSwitchFP;

    /* Set device IDs and function pointers for carrier board */
    if(context->SoMCRRVersion == SOMCRR_REV_D)
    {
        hwCarrierDeviceId = (int)E_MCP2301X_DEVICE_EZKIT_REV_D;
    }
    else if(context->SoMCRRVersion == SOMCRR_REV_A)
    {
        hwCarrierDeviceId = (int)E_MCP2301X_DEVICE_EZKIT_REV_A;
    }
    else
    {
        hwCarrierDeviceId = (int)E_MCP2301X_DEVICE_EZKIT_HW_PROBE;
    }

    /* Set device IDs and function pointers for SOM */
    if(context->SoMVersion == SOM_REV_E)
    {
        hwSoMDeviceId  = (hwCarrierDeviceId == (int)E_MCP2301X_DEVICE_EZKIT_HW_PROBE) ? E_ADP5587_DEVICE_SC598_REV_E_SOM_HW_PROBE : (int)E_ADP5587_DEVICE_SC598_REV_E_SOM;
        hwSoftSwitchFP = (T_SS_HW_FP *)&sSSFPAdp;
    }
    else
    {
        hwSoMDeviceId  = (int)E_MCP2301X_DEVICE_SC598_REV_D_SOM;
        hwSoftSwitchFP = (T_SS_HW_FP *)&sSSFPMcp;
    }
    
    /* Update the Carrier Board and SOM Soft-Switch settings */
    for(u8PinIdx = 0U; u8PinIdx < SS_PIN_ID_MAX; u8PinIdx++)
    {
        if((u8PinIdx >= SS_PIN_ID_nADAU1979_EN) && (u8PinIdx < SS_PIN_ID_nUART0_FLOW_EN))
        {
            asSSPinConfigRegistry[u8PinIdx].deviceId = hwCarrierDeviceId;
        }

        if((u8PinIdx >= SS_PIN_ID_nUART0_FLOW_EN) && (u8PinIdx < SS_PIN_ID_MAX))
        {
            asSSPinConfigRegistry[u8PinIdx].deviceId = hwSoMDeviceId;
            asSSPinConfigRegistry[u8PinIdx].psHwFP   = hwSoftSwitchFP;
        }
    }
    
    softswitch_init((T_SS_PIN_CONFIG *)&asSSPinConfigRegistry);
}

void ss_deinit(APP_CONTEXT *context)
{
    softswitch_deinit();
}

/******************************************************************************
 *   Static Helpers
 ******************************************************************************/
static void ss_mcp_init(int deviceId)
{
    (void)Mcp2301x_Init(deviceId);
}

static void ss_mcp_deinit(int deviceId)
{
    (void)Mcp2301x_DeInit(deviceId);
}

static int ss_mcp_get(int deviceId, int portId, int pinId, bool *value)
{
    return((int)Mcp2301x_ReadPin(deviceId, portId, pinId, value));
}

static int ss_mcp_set(int deviceId, int portId, int pinId, bool value)
{
    return((int)Mcp2301x_WritePin(deviceId, portId, pinId, value));
}

static int ss_mcp_get_portpin(int deviceId, int sysPinId, int * portId, int * pinId)
{
    return((int)Mcp2301x_GetPortPin(deviceId, sysPinId, (T_MCP2301X_PORT * const)portId, (T_MCP2301X_PIN * const)pinId));
}

static void ss_adp_init(int deviceId)
{
    (void)Adp5587_Init(deviceId);
}

static void ss_adp_deinit(int deviceId)
{
    (void)Adp5587_DeInit(deviceId);
}

static int ss_adp_get(int deviceId, int portId, int pinId, bool *value)
{
    return((int)Adp5587_ReadPin(deviceId, portId, pinId, value));
}

static int ss_adp_set(int deviceId, int portId, int pinId, bool value)
{
    return((int)Adp5587_WritePin(deviceId, portId, pinId, value));
}

static int ss_adp_get_portpin(int deviceId, int sysPinId, int * portId, int * pinId)
{
    return((int)Adp5587_GetPortPin(deviceId, sysPinId, (T_ADP5587_PORT * const)portId, (T_ADP5587_PIN * const)pinId));
}
