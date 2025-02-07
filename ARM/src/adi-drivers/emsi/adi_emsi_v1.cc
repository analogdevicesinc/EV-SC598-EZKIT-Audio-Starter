/*!*****************************************************************************
 * @file:     adi_emsi_v1.c
 * @brief:    EMSI device driver implementation
 * @version: $Revision: $
 * @date:    $Date: 2021-07-29$
 ******************************************************************************

 Copyright (c) 2022 Analog Devices.  All Rights Reserved.

 This software is proprietary.  By using this software you agree
 to the terms of the associated Analog Devices License Agreement.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

 *****************************************************************************/

/* \mainpage EMSI Driver
 * This document provides details about EMSI Device driver APIs and their usage.
 */

/** @addtogroup EMSI_Driver EMSI Device Driver
 *  @{
 *
 *
  eMSI controller performs various activities such as eMMC device and SD card identification, data transfer, command queuing, etc.
  There is one EMSI instance available, which can be accessed through ADI_EMSI_0.
  EMSI driver supports read/write APIs in DMA mode.
  SDMA Mode is blocking mode APIs wherein once application submits the number of blocks to be read/written,
  the API blocks until all the blocks are read/written.
  In the DMA mode of transfer (ADMA2 and ADMA3), based on the application requirement, user can configure
  one of the different DMA modes available. The DMA mode of operation is non-blocking and
  application is notified in the registered callback with a relevant event when the transfer is done.
 */

#ifndef __ADI_EMSI_V1_C__
/*! Guarding macro to avoid reinclusion */
#define __ADI_EMSI_V1_C__
/*==========  I N C L U D E  ==========*/
#include <stddef.h>
#include <string.h>
#include <adi_osal.h>
#include <assert.h>
#include <math.h>
#if defined(__ADSPARM__)
/* Cortex-A55 core */
#include <services/int/adi_gic.h>
#include <runtime/cache/adi_cache.h>
#else
/* SHARC core */
#include <sys/cache.h>
#include <services/int/adi_int.h>
#endif
#include <sys/platform.h>
#include <services/pwr/adi_pwr.h>
#include <services/gpio/adi_gpio.h>
#include <services/tmr/adi_tmr.h>
#include "adi_emsi_def_SC59x.h"
#include "adi_emsi_data_SC59x.cc"
#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_17_4:"Array indexing shall be the only allowed form of pointer arithmetic.")
#pragma diag(suppress:misra_rule_14_7:"Allow functions to have multiple exits for better readability and optimized code")
#pragma diag(suppress:misra_rule_10_3:"The value of a complex expression of integer type shall only be cast to a type of the same signedness that is no wider than the underlying type of the expression.")
#pragma diag(suppress:misra_rule_12_7:"Bitwise operators shall not be applied to operands whose underlying type is signed (Used on boolean)  ")
#pragma diag(suppress:misra_rule_8_7:"Objects shall be defined at block scope if they are only accessed from within a single function.")
#endif
#if defined (__ADSPGCC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Woverflow"
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
/************************EMSI APIs available for application to use************************/
/**
 * @brief       Open a desired EMSI device instance and set the default configuration.
 *
 * @details     Initializes the EMSI device as per the user setting provided in the
 *              static configuration file and installs the required interrupt handlers.
 *              Also initialize the eMSI clock to less than 400 kHz for identification.
 *              This API must be called before operating EMSI device.
 *
 * @param [in]  nDeviceNum          EMSI device instance to be opened.
 *
 * @param [in]  eCardType           Type Of the card connected to interface.\n
 *                                  #ADI_EMSI_CARD_TYPE_EMMC: eMMC card connected to interface.\n
 *                                  #ADI_EMSI_CARD_TYPE_SDCARD: SD card connected to interface.\n
 *
 * @param [in]  pMemory             Pointer to a memory required for driver operation.
 *
 * @param [in]  nMemSize            Size of the given memory to which pMemory points.\n
 *                                  ADI_EMSI_DRIVER_MEMORY_SIZE : Size of the memory passed to driver.
 *
 * @param [out] phInfo              Pointer to a location where the handle to the
 *                                  opened device is written.
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS               If successfully opened the given instance.
 *
 *    - #ADI_EMSI_FAILURE               If Clock initialization fails.
 *
 *    - #ADI_EMSI_INTERRUPT_FAILURE     Failed to install one of the interrupt handlers.
 *
 * @sa          adi_emsi_Close()
 *
 * @note        No other EMSI APIs should be called until adi_emsi_Open() API is called.
 *
 * @note        Memory supplied will be used by the driver for managing the EMSI device.
 *              This memory can be reused once device is closed.
 *
 **/
ADI_EMSI_RESULT adi_emsi_Open(
                uint8_t  nDeviceNum,
                ADI_EMSI_CARD_TYPE eCardType,
                void *pMemory,
                uint32_t nMemSize,
                ADI_EMSI_HANDLE*  phInfo)
{

    /*eResult to be returned*/
    ADI_EMSI_RESULT eResult = ADI_EMSI_SUCCESS;

    /* Pointer to the EMSI Device instance Data */
    ADI_EMSI_DEVICE  *pDevice;

    /* Pointer to the EMSI Device information */
    ADI_EMSI_INFO    *pDevInfo;

    /* Pointer to track the memory available for allocation */
    uint8_t  *pAvailableMem = pMemory;

#if defined (ADI_DEBUG)
    /* Check if the given device number is within the range of EMSI devices present in the processor.
     * There is one device instance available for EMSI.*/
    assert(nDeviceNum < ADI_EMSI_MAX_INSTANCES);

    /* Verify that the device is not already open.*/
    assert(gEMSIDevInfo[nDeviceNum].pDevice == NULL);

    /* Check if the pointer to device handle is valid */
    assert(phInfo != NULL);

    /* Check if the given memory pointer is valid */
    assert(pMemory != NULL);

    /* Check if the given memory is sufficient for the operation.*/
    assert(nMemSize >= ADI_EMSI_DRIVER_MEMORY_SIZE);
#endif /* ADI_DEBUG */

    /* Get pointer to the device information */
    pDevInfo = &gEMSIDevInfo[nDeviceNum];

    /* Point the memory provided by user to device data address */
    pDevice  = (ADI_EMSI_DEVICE *)(pAvailableMem);

    /* Update the device data address such that it now points to the
       memory provided by the user */
    pDevInfo->pDevice  =  pDevice;

    /* Zero initialize the device memory so that we do not have to explicitly initialize
       the structure members to 0 */
    memset(pMemory, 0, nMemSize);

    /* Increment the memory tracking pointer by used amount */
    pAvailableMem   +=  sizeof(ADI_EMSI_DEVICE);

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /*Reset the eMSI registers for the EMSI instance opened for*/
    ResetEmsiRegs(pDevInfo->pEMSIRegs);

    /*Reset and Clear eMSI Interrupt registers to default*/
    ResetEmsiIntRegs(pDevInfo->pEMSIRegs);

    /*Configuring the Card type*/
    SetCardType(pDevInfo,eCardType);

    /*Initializing  the eMSI controller as per card type*/
    ControllerSetup(pDevInfo->pEMSIRegs,pDevInfo->pDevice);

    /*Enable the interrupts and interrupt status*/
    SetInterruptMask(pDevInfo,ENABLE_NORMAL_INT_STAT,ENABLE_ERROR_INT_STAT,
                     ENABLE_NORMAL_INT,ENABLE_ERROR_INT,true);

    /*Setting default application usage to Normal data transfer*/
    pDevInfo->pDevice->eAppUsage = ADI_EMSI_APP_USE_NORMAL_DATATRANSFER;

    /*Setting default boot acknowledgement status to disable*/
    pDevInfo->pDevice->bBoot_Ack = (int8_t)ADI_EMSI0_CFG_BOOTACK;

    /*On board eMMC device is used*/
    pDevInfo->pDevice->bExtSocketUsed = (int8_t)ADI_EMSI0_CFG_EXTSOCKETUSE;

    /*Disable response check by controller*/
    pDevInfo->pDevice->bControllerRespChk = (int8_t)ADI_EMSI0_CFG_CONTROLLERRESPCHK;

    /*Setting up clock for eMSI and enabling the clock to card with frequency (<400 kHz)*/
    if(ADI_EMSI_SUCCESS != SetClock(pDevInfo,ADI_EMSI0_CLOCK_DIVISOR))
    {
        return ADI_EMSI_FAILURE;
    }
    /*Disable preset register use*/
    pDevInfo->pEMSIRegs->CTL2 |= ADI_EMSI0_CFG_PRESETREGUSE;

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /*Wait for 74 eMSI clock cycles before we issue 1st command*/
    if(ADI_EMSI_SUCCESS != EmsiDelay(pDevInfo, SEVENTY_FOUR_CYCLES))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Installing interrupt handlers*/
    if(adi_int_InstallHandler(pDevInfo->StatusIntID,EmsiStatusHandler,pDevInfo,true) != ADI_INT_SUCCESS)
    {
        /* Free up the device */
        pDevInfo->pDevice = NULL;

        return ADI_EMSI_INTERRUPT_FAILURE;
    }
    if(adi_int_InstallHandler(pDevInfo->WakupIntID,EmsiWakeupHandler,pDevInfo,true) != ADI_INT_SUCCESS)
    {
        /* Free up the device */
        pDevInfo->pDevice = NULL;

        /* Uninstall already installed handlers */
        adi_int_UninstallHandler(pDevInfo->StatusIntID);

        return ADI_EMSI_INTERRUPT_FAILURE;
    }

    /*Return the device handle to the application*/
    *phInfo   =   pDevInfo;

    return eResult;
}

/**
 * @brief       Set the Relative Card Address (RCA).
 *
 * @details     For eMMC device: After EMSI device is initialized by adi_emsi_Open() API. User will use this API
 *              to submit the RCA (Relative Card Address). The same RCA should be used throughout
 *              application (For card initialization in CMD3 etc.). Usually this API is used only
 *              once before eMMC device identification process starts.
 *
 *              For SD cards: After EMSI device is initialized by adi_emsi_Open() API. User will do
 *              card identification process and submit the RCA published by card (RCA received as response to CMD3)
 *              in this API as nRca parameter. The same RCA should be used throughout application. Usually this
 *              API is used only once after SD card identification is done.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  nRca            Relative card address of the card.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_SetRca(
                ADI_EMSI_HANDLE const hInfo,
                uint16_t nRca)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Saving the relative card address for future use*/
    pInfo->pDevice->nRca=nRca;

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       Set the application usage.
 *
 * @details     For eMMC: After EMSI device is initialized by adi_emsi_Open() API and RCA is set by
 *              adi_emsi_SetRca(). User gives details about whether the application is used
 *              for booting or for normal data transfer. User will set this parameter at the
 *              start of the application. Usually this API is called only once.
 *              By default, controller is initialized for Normal data transfer
 *              (#ADI_EMSI_APP_USE_NORMAL_DATATRANSFER) in adi_emsi_Open() API.
 *
 *              For SD card: After EMSI device is initialized by adi_emsi_Open() API and RCA is set by
 *              adi_emsi_SetRca(). User will set this parameter at the start of the application.
 *              Usually this API is called only once. For SD card only, Normal data transfer
 *              (#ADI_EMSI_APP_USE_NORMAL_DATATRANSFER) is valid argument. By default, controller is
 *              initialized for Normal data transfer (#ADI_EMSI_APP_USE_NORMAL_DATATRANSFER) in adi_emsi_Open() API.
 *
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  eAppUse         Whether application is used for data transfer or for boot purpose.\n
 *                              #ADI_EMSI_APP_USE_NORMAL_DATATRANSFER (Valid for eMMC and SD devices)\n
 *                              When user performs user performs card identification (CMD1->CMD2->CMD3)
 *                              and performs data transfer using Data transfer commands(CMD17, CMD18, CMD24, CMD25 etc.)\n
 *                              #ADI_EMSI_APP_USE_BOOTING_OPERATION (Valid for eMMC devices)\n
 *                              When user performs booting operation.
 *
 * @param [in]  bBootAck        If user is expecting boot acknowledgment from eMMC device side while booting operation.
 *                              when eAppuse is set to #ADI_EMSI_APP_USE_BOOTING_OPERATION (Application is used for booting operation).
 *                              In case of normal transfer user can ignore this parameter.\n
 *
 *                              -true   Boot acknowledgment is expected from eMMC device side while booting operation.\n
 *                              -false  Boot acknowledgment is not expected from eMMC device side while booting operation.\n
 *
 *
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS       API call was successful
 *
 *    - #ADI_EMSI_BUSY          Data Transfer is not finished
 *
 * @sa          adi_emsi_Open()
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_SetAppUse(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_APP_USE eAppUse,
                bool bBootAck)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    if(eAppUse == ADI_EMSI_APP_USE_NORMAL_DATATRANSFER)
    {
        pInfo->pDevice->eAppUsage = ADI_EMSI_APP_USE_NORMAL_DATATRANSFER;
    }
    else
    {
        pInfo->pDevice->eAppUsage = ADI_EMSI_APP_USE_BOOTING_OPERATION;
    }

    /*Saving boot acknowledgement status*/
    pInfo->pDevice->bBoot_Ack = bBootAck;

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       Change eMSI bus clock frequency.
 *
 * @details     After eMSI clock is initialized by adi_emsi_Open() API by static configuration,
 *              if user wants to change eMSI bus clock frequency then this API is used.
 *              we need stop the eMSI bus clock then configure new divider values and
 *              then activate the eMSI bus clock.
 *
 *              If Preset register use is enabled in adi_emsi_SetPresetRegUse() API then divider values
 *              will be selected as per bus speed mode and corresponding preset register values.
 *              E.g. if bus mode is selected as HSDDR then corresponding PRESET value register will
 *              be PRESET_HSDDR and divider values will be 0 (default value). For more information
 *              refer to register section of HRM. While using this feature it should be ensured that
 *              corresponding bus speed mode is set using adi_emsi_SetSpeedMode() API. nDivFreq
 *              value can be ignored while using the preset register feature.
 *
 *              User are advised to pass divider value in this API instead of using preset
 *              register as it gives option for large range of clock divider values.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  nDivFreq        Clock divider value to be applied to eMSI input clock.
 *                              Minimum value for nDivisor = 0,
 *                              (Here, Output_EMSI_Clk_Frequency = Input_EMSI_Clk_Frequency).\n
 *
 *                              For Rest Values,
 *                              Output_Clk_Frequency = Input_Clk_Frequency/(2*nDivFreq)
 *                              Maximum value for nDivFreq = 1023.
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           Successfully changed for eMSI bus clock frequency
 *
 *    - #ADI_EMSI_TIMED_OUT         Time out during synchronizing operation
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_ChangeClockFreq(
                ADI_EMSI_HANDLE const hInfo,
                uint16_t nDivFreq)
{
    /*Temporary Variable*/
    uint16_t nLower_Freq_Divider;
    uint16_t nUpper_Freq_Divider;
    volatile uint32_t nTimeout_Sync=0u;

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /*Disabling the clock to card before clock frequency change*/
    ClktoCard(pInfo,false);

    /*Disabling CDU Clock out 13 and Clock out 14*/
    adi_pwr_EnableCduClockOutput(ADI_PWR_CDU_CLKOUT_13,false);
    adi_pwr_EnableCduClockOutput(ADI_PWR_CDU_CLKOUT_14,false);

    /*Deactivates the eMSI PLL clock*/
    pInfo->pEMSIRegs->CLK_CTL &= ~(BITM_EMSI_CLK_CTL_PLL_EN);

    /*Select divider value as per preset register if preset register use set in CTL2 register
     * otherwise use nDivFreq parameter*/
    if(!(pInfo->pEMSIRegs->CTL2 & BITM_EMSI_CTL2_PRESET_VAL_EN))
    {
        /*Setting the divider values*/
        /*Clearing the Bit before setting it*/
        pInfo->pEMSIRegs->CLK_CTL &= ~(BITM_EMSI_CLK_CTL_FREQ_SEL);
        pInfo->pEMSIRegs->CLK_CTL &= ~(BITM_EMSI_CLK_CTL_UPPER_FREQ_SEL);

        /*Configuring new divider values*/
        nLower_Freq_Divider = (nDivFreq & LOWER_FREQ_SEL);
        pInfo->pEMSIRegs->CLK_CTL |= (nLower_Freq_Divider<<BITP_EMSI_CLK_CTL_FREQ_SEL);
        nUpper_Freq_Divider = ((nDivFreq & UPPER_FREQ_SEL)>>BITP_UPPER_FREQ_SEL);
        pInfo->pEMSIRegs->CLK_CTL |= (nUpper_Freq_Divider<<BITP_EMSI_CLK_CTL_UPPER_FREQ_SEL);
    }

    /*Activates the eMSI PLL clock*/
    pInfo->pEMSIRegs->CLK_CTL |=BITM_EMSI_CLK_CTL_PLL_EN;

    /*Wait for synchronized value of the card_clk_stable signal after the PLL Enable bit is set to 1*/
    while(!(pInfo->pEMSIRegs->CLK_CTL & BITM_EMSI_CLK_CTL_INTERNAL_CLK_STABLE))
    {
        nTimeout_Sync++;
        if(nTimeout_Sync >= TIMEOUT)
        {
            /*Timeout Occurred*/
            return ADI_EMSI_TIMED_OUT;
        }
    }

    /*Enabling  CDU Clock out 13 and Clock out 14*/
    adi_pwr_EnableCduClockOutput(ADI_PWR_CDU_CLKOUT_13,true);
    adi_pwr_EnableCduClockOutput(ADI_PWR_CDU_CLKOUT_14,true);

    /*Reset the Data and Command lines to avoid the effect of any glitch on sampling clock.*/
    ResetCmdDataLine(pInfo);

    /*Enabling the clock to card after clock frequency is changed*/
    ClktoCard(pInfo,true);

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       Sending the command to the card.
 *
 * @details     This API is used for sending command to the card. Command will be issued once
 *              we write command index (nCmd_Idx) bits into command register. After command is
 *              send, API polls on command completion interrupt status bit, bit will be set if
 *              command is sent, and response is received from the card (for commands where response
 *              is expected) (except for Auto CMD12/CMD23). The R1b responses are handled by
 *              Status handler. Error in Received response will be checked using application callback.
 *
 *              Note: For issuing command after booting sequence (not applicable for normal operation):
 *              Application should take care that it is issuing next command (Such as CMD1) after 56 clock cycles
 *              after normal boot is finished. (Refer to timing diagram 6.15.5 for more information from JEDEC specification 5.1 ).
 *              Also keep frequency of operation to <= 400 kHz for eMMC device identification.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  nCmd_Idx        Index of the command to be send to the Card.
 *
 * @param [in]  nArg            Argument of the command to be send to the card.
 *
 * @param [in]  pCmdParameters  Parameters which are related to each command. For each command
 *                              there are set of parameters as per command use. Please refer to HRM
 *                              for more usage.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           Send command complete and received response packet successfully
 *
 *    - #ADI_EMSI_TIMED_OUT         Command completion timeout
 *
 *    - #ADI_EMSI_BUSY              Data transfer is not finished
 *
 **/
ADI_EMSI_RESULT adi_emsi_SendCommand(
                ADI_EMSI_HANDLE const hInfo,
                uint8_t nCmd_Idx,
                uint32_t nArg,
                ADI_EMSI_CMD_PARA *pCmdParameters)
{

    /*Temporary Variable*/
    volatile uint32_t nTimeout_Cmdcomplete=0u;

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on and command is not sent in internal API*/
    if((pInfo->pDevice->bInternal_Cmd == false) && (pInfo->pDevice->bData_Trans == true))
    {
        return ADI_EMSI_BUSY;
    }

    if(pCmdParameters->eDataPresent == ADI_EMSI_DATA_PRESENT_TRUE)
    {
        /*Data Present setting to true.*/
        pInfo->pDevice->bData_Trans = true;
    }

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /*Setting CMD register parameters*/
    CmdRegSet(pInfo,pCmdParameters);

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /*Reset the count*/
    nTimeout_Cmdcomplete = 0u;
    /*Wait till CMD line become free to issue command*/
    while(pInfo->pEMSIRegs->PSTATE & BITM_EMSI_PSTATE_CMD_INHIBIT)
    {
        nTimeout_Cmdcomplete++;
        if(nTimeout_Cmdcomplete >= TIMEOUT)
        {
            return ADI_EMSI_TIMED_OUT;
        }
    }

    /*Reset the count*/
    nTimeout_Cmdcomplete = 0u;
    /*Check if command uses DATA line including (R1b) response and it is not Abort command so wait till flag gets cleared*/
    if(((pInfo->pDevice->nCmdParameters & BITM_EMSI_CMD_DATA_PRESENT_SEL) ||
       ((pInfo->pDevice->nCmdParameters & BITM_EMSI_CMD_RESP_TYPE_SELECT) == (uint16_t)ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY)) &&
       ((pInfo->pDevice->nCmdParameters & BITM_EMSI_CMD_TYPE) == ENUM_EMSI_CMD_NORMAL_CMD))
    {
        /* Can issue command which used DAT line*/
        while(pInfo->pEMSIRegs->PSTATE & BITM_EMSI_PSTATE_CMD_INHIBIT_DAT)
        {
            nTimeout_Cmdcomplete++;
            if(nTimeout_Cmdcomplete >= TIMEOUT)
            {
                return ADI_EMSI_TIMED_OUT;
            }
        }
    }

    /*Clearing all the interrupts status bits raised by previous Operations*/
    pInfo->pEMSIRegs->ISTAT = DEFAULT_INT_STAT_MASK_VALUE;
    /*Clearing all the error interrupts status bits raised by previous operations*/
    pInfo->pEMSIRegs->ERR_STAT = DEFAULT_ERROR_INT_STAT_MASK_VALUE;

    /* Argument to be send with command*/
    pInfo->pEMSIRegs->ARG = (uint32_t)nArg;

    /* Configuring Command registers with CMD register flags and CMD index*/
    pInfo->pEMSIRegs->CMD = ((uint16_t)((uint16_t)nCmd_Idx<<BITP_EMSI_CMD_INDEX)|pInfo->pDevice->nCmdParameters);

    /*Reset the count*/
    nTimeout_Cmdcomplete = 0u;
    /*Command completion check by driver Transfer mode register is reset while opening EMSI device and RESP_INT_DIS is never set*/
    if(!(pInfo->pEMSIRegs->TRNSFRMODE & BITM_EMSI_TRNSFRMODE_RESP_INT_DIS))
    {
        /*Wait till command gets completed and response packet received*/
        while(!(pInfo->pEMSIRegs->ISTAT & BITM_EMSI_ISTAT_CMD_COMPLETE))
        {
            nTimeout_Cmdcomplete++;
            if(nTimeout_Cmdcomplete >= TIMEOUT)
            {
                return ADI_EMSI_TIMED_OUT;
            }
        }
        /* Clearing the command complete interrupt status*/
        pInfo->pEMSIRegs->ISTAT=BITM_EMSI_ISTAT_CMD_COMPLETE;
    }

    /*Wait till D0 line gets cleared*/
    if(((pInfo->pEMSIRegs->CMD & ENUM_EMSI_CMD_RESP_LEN_48B) ==
       (uint16_t)ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY)&&
       ((pInfo->pEMSIRegs->CMD & BITM_EMSI_CMD_DATA_PRESENT_SEL) ==
       (uint16_t)ADI_EMSI_DATA_PRESENT_FALSE))
    {
        while(pInfo->pEMSIRegs->PSTATE & BITM_EMSI_PSTATE_DAT_LINE_ACTIVE)
        {
            ;
            /*Wait for D0 line to get clear*/
        }
    }

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       Giving access of the response back to application.
 *
 * @details     User can access the received response for previously send command (if response is expected
 *              from the command).
 *
 * @param [in]  hInfo       Handle to the EMSI device to be configured.
 *
 * @param [in]  pRespAcc    Pointer to buffer where response to be stored.
 *                          This is usually array of size 4 with each location having 32-bit storage.
 *                          (e.g. uint32_t Resp_Store[4])
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS       API call successfully.
 *
 *    - #ADI_EMSI_NO_RESPONSE   No response is expected for previously send command.
 **/
ADI_EMSI_RESULT adi_emsi_GetResponse(
                ADI_EMSI_HANDLE const hInfo,
                uint32_t*  pRespAcc)
{

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on and command is not sent in internal API*/
    if((pInfo->pDevice->bInternal_Cmd == false) && (pInfo->pDevice->bData_Trans == true))
    {
        return ADI_EMSI_BUSY;
    }

    /*No Response is expected for previously send command*/
    if((pInfo->pEMSIRegs->CMD & BITM_EMSI_CMD_RESP_TYPE_SELECT) ==
       (uint16_t)ADI_EMSI_RESPONSE_TYPE_NONE)
    {
        return ADI_EMSI_NO_RESPONSE;
    }

    else
    {
        /*Response received without any error storing the response*/
        *pRespAcc++ = pInfo->pEMSIRegs->RESP0;
        *pRespAcc++ = pInfo->pEMSIRegs->RESP1;
        *pRespAcc++ = pInfo->pEMSIRegs->RESP2;
        *pRespAcc++ = pInfo->pEMSIRegs->RESP3;
    }

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       Selects or deselects the card.
 *
 * @details     This API sends CMD7 to card for selecting and deselecting the card.
 *              Which puts card from standby state to transfer state or transfer state to standby
 *              state respectively.
 *              This API should be called after adi_emsi_SetRca() API. Also, this API is valid only
 *              when application usage is set to #ADI_EMSI_APP_USE_NORMAL_DATATRANSFER in adi_emsi_SetAppUse() API.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  bSelDesel       Select/Deselect the card.\n
 *                              -true           Puts card from standby state to transfer state\n
 *                              -false          Puts card from transfer state to standby state
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful
 *
 *    - #ADI_EMSI_BUSY              Data transfer is not finished.
 *
 *    - #ADI_EMSI_INVALID_APIUSE    API use is not valid for selected application use
 *
 *    - #ADI_EMSI_FAILURE           Failure in sending command.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_SelectCard(
                ADI_EMSI_HANDLE const hInfo,
                bool bSelDesel)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Return if Application is used for booting*/
    if(pInfo->pDevice->eAppUsage != ADI_EMSI_APP_USE_NORMAL_DATATRANSFER)
    {
        return ADI_EMSI_INVALID_APIUSE;
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    /*Select the card using CMD7*/
    if (bSelDesel==true)
    {
        ADI_EMSI_CMD_PARA CMD_PARA7 ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                      ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                      ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SELECT_CARD,
                                                     (uint32_t)((uint32_t)pInfo->pDevice->nRca<<16u),
                                                     &CMD_PARA7))
        {
            return ADI_EMSI_FAILURE;
        }
    }
    else
    {
        /*No response from deselected card also argument will be zero for deselecting the card*/
        ADI_EMSI_CMD_PARA CMD_PARA7 ={ADI_EMSI_RESPONSE_TYPE_NONE,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                      ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                      ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,DESELECT_CARD,0u,&CMD_PARA7))
        {
            return ADI_EMSI_FAILURE;
        }
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = false;

    return ADI_EMSI_SUCCESS;
}
/**
 * @brief       Configures the speed mode for EMSI operation.
 *
 * @details     When the application usage is set to #ADI_EMSI_APP_USE_NORMAL_DATATRANSFER in adi_emsi_SetAppUse() API,
 *              this API configures speed mode for upcoming transfers from controller and card side,
 *              and this API should be called after adi_emsi_SelectCard() API, which ensure that card is in transfer state.
 *
 *              When the application usage is set to #ADI_EMSI_APP_USE_BOOTING_OPERATION in adi_emsi_SetAppUse() API,
 *              user should ensure that all the configuration from card side should beforehand (e.g. BOOT_BUS_CONDITIONS [177],
 *              PARTITION_CONFIG (before BOOT_CONFIG) [179] etc. in EXT_CSD register),in this case this API configures
 *              speed only from controller side.
 *
 * @param [in]  hInfo       Handle to the EMSI device to be configured.
 *
 * @param [in]  eSpeedMode  The Speed to be used for eMSI operation.\n
 *                          -eMMC supported speed modes:\n
 *                          #ADI_EMSI_SPEED_MODE_LEGACY_OR_DSSDR: Legacy (0-25MHz)\n
 *                          #ADI_EMSI_SPEED_MODE_HIGHSPEED_SDR: High Speed SDR (0-50 MHz)\n
 *                          #ADI_EMSI_SPEED_MODE_HIGHSPEED_DDR: High Speed DDR (0-50 MHz) (Not supported in booting operation (#ADI_EMSI_APP_USE_BOOTING_OPERATION))\n
 *
 *                          -SD card supported Speed modes:\n
 *                          #ADI_EMSI_SPEED_MODE_LEGACY_OR_DSSDR: Default speed SDR (0-25MHz)\n
 *                          #ADI_EMSI_SPEED_MODE_HIGHSPEED_SDR: High Speed SDR (0-44 MHz)\n
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS                   eMSI speed mode set successful.
 *
 *    - #ADI_EMSI_BUSY                      Data transfer is not finished
 *
 *    - #ADI_EMSI_FAILURE                   Failure in sending command or Switch command failed.
 *
 *    - #ADI_EMSI_INVALID_CONFIGURATION     Invalid speed mode.
 *
 * @sa          adi_emsi_SetAppUse()
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_SetSpeedMode(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_SPEED_MODE eSpeedMode)
{
    /*Temporary variables*/
    uint32_t nTemp;
    bool bHs_Timings=false;

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    /*Saving speed mode for future use*/
    pInfo->pDevice->eSpeed_Mode = eSpeedMode;

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_14_10:"All if ... else if constructs shall be terminated with an else clause.")
#endif /* _MISRA_RULES */

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /*Checking card type*/
    if(pInfo->pDevice->eCardType == ADI_EMSI_CARD_TYPE_EMMC)
    {
        /*Change the speed mode from eMSI Controller side*/
        switch(eSpeedMode)
        {
        case ADI_EMSI_SPEED_MODE_LEGACY_OR_DSSDR:
            pInfo->pEMSIRegs->CTL2 &= ~BITM_EMSI_CTL2_EMMC_MODE_SEL;
            pInfo->pEMSIRegs->CTL2 |= (BITM_EMSI_CTL2_EMMC_MODE_SEL &
                                       ENUM_EMSI_CTL2_LEGACY);
            break;
        case ADI_EMSI_SPEED_MODE_HIGHSPEED_SDR:
            pInfo->pEMSIRegs->CTL2 &= ~BITM_EMSI_CTL2_EMMC_MODE_SEL;
            pInfo->pEMSIRegs->CTL2 |= (BITM_EMSI_CTL2_EMMC_MODE_SEL &
                                       ENUM_EMSI_CTL2_HSSDR);
            break;
        case ADI_EMSI_SPEED_MODE_HIGHSPEED_DDR:
            pInfo->pEMSIRegs->CTL2 &= ~BITM_EMSI_CTL2_EMMC_MODE_SEL;
            pInfo->pEMSIRegs->CTL2 |= (BITM_EMSI_CTL2_EMMC_MODE_SEL &
                                       ENUM_EMSI_CTL2_HSDDR);
            break;
        default:
            return ADI_EMSI_INVALID_CONFIGURATION;
        }
    }
    else if(pInfo->pDevice->eCardType == ADI_EMSI_CARD_TYPE_SDCARD)
    {
        /*Change the speed mode from EMSI Controller side*/
        switch(eSpeedMode)
        {
        case ADI_EMSI_SPEED_MODE_LEGACY_OR_DSSDR:
        {
            /*Default speed SDR (0-25MHz)*/
            pInfo->pEMSIRegs->CTL2 &= ~BITM_EMSI_CTL2_EMMC_MODE_SEL;
            pInfo->pEMSIRegs->CTL2 |= (BITM_EMSI_CTL2_EMMC_MODE_SEL &
                                       ENUM_EMSI_CTL2_LEGACY);
            break;
        }
        case ADI_EMSI_SPEED_MODE_HIGHSPEED_SDR:
        {
            /*High speed SDR (0-44MHz)*/
            pInfo->pEMSIRegs->CTL2 &= ~BITM_EMSI_CTL2_EMMC_MODE_SEL;
            pInfo->pEMSIRegs->CTL2 |= (BITM_EMSI_CTL2_EMMC_MODE_SEL &
                                       ENUM_EMSI_CTL2_HSSDR);
            break;
        }
        default:
            return ADI_EMSI_INVALID_CONFIGURATION;
        }
    }

    /*Enabling high speed mode from controller side (>25-50 MHz for SDR eMMC/>25-44 MHz for SDR SD, for DDR always Set)*/
    if((eSpeedMode == ADI_EMSI_SPEED_MODE_HIGHSPEED_SDR)||
       (eSpeedMode == ADI_EMSI_SPEED_MODE_HIGHSPEED_DDR))
    {
        pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_HIGH_SPEED_EN;
        pInfo->pEMSIRegs->CTL1 |= BITM_EMSI_CTL1_HIGH_SPEED_EN;
        bHs_Timings=true;
    }
    else
    {
        /*High speed mode disabling from eMSI controller side (<25 MHz)*/
        pInfo->pEMSIRegs->CTL1 &= ~(BITM_EMSI_CTL1_HIGH_SPEED_EN);
        bHs_Timings=false;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /*Configuring the speed mode from card side if it is normal transfer*/
    if(pInfo->pDevice->eAppUsage == ADI_EMSI_APP_USE_NORMAL_DATATRANSFER)
    {
        if(pInfo->pDevice->eCardType == ADI_EMSI_CARD_TYPE_EMMC)
        {
            /*Parameters for CMD13*/
            ADI_EMSI_CMD_PARA CMD_PARA13 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                            ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                            ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

            /*Sending CMD6 (Switch) for switching speed mode from card side*/
            ADI_EMSI_CMD_PARA CMD_PARA6 = {ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                           ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                           ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
            if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SWITCH,((uint32_t)((uint32_t)bHs_Timings<<BITP_VALUE) |
                                                         ((uint32_t)HS_TIMING_EXTCSD<<BITP_INDEX) |
                                                         ((uint32_t)ACCESS_WRITEBYTE<<BITP_ACCESS)),
                                                         &CMD_PARA6))
            {
                return ADI_EMSI_FAILURE;
            }


            /*Check if the switch was successful by reading the card status using CMD13*/
            if(ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SEND_STATUS,
                                                        (uint32_t)((uint32_t)pInfo->pDevice->nRca<<16),
                                                        &CMD_PARA13))
            {
                return ADI_EMSI_FAILURE;
            }
            adi_emsi_GetResponse(pInfo,&InternalResp[0]);
            /*Checking Switch error bit from response received*/
            nTemp = InternalResp[0]>>BITP_SWITCH_CMD_ERROR_CHK;
            nTemp = nTemp & SWITCH_CMD_ERROR_MASK;
            if(nTemp==SWITCH_CMD_ERROR)
            {
                /*Failure in CMD6 (Switch error bit is set in R1 response)*/
                return ADI_EMSI_FAILURE;
            }
        }
        else if (pInfo->pDevice->eCardType == ADI_EMSI_CARD_TYPE_SDCARD)
        {
            /*Configuring speed mode from card side*/
            if (ADI_EMSI_SUCCESS != SdHighSpeedMode(pInfo,bHs_Timings))
            {
                /*Failure in switching SD card speed mode*/
                return ADI_EMSI_FAILURE;
            }
        }
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = false;
#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       Configuring the bus width for eMSI operation.
 *
 * @details     When the application usage is set to #ADI_EMSI_APP_USE_NORMAL_DATATRANSFER in adi_emsi_SetAppUse() API,
 *              this API configures bus width for upcoming transfers from controller and card side,
 *              and this API should be called after adi_emsi_SetSpeedMode() API.
 *
 *              When the application usage is set to #ADI_EMSI_APP_USE_BOOTING_OPERATION in adi_emsi_SetAppUse() API,
 *              user should ensure that all the configuration from card side should beforehand (e.g. BOOT_BUS_CONDITIONS [177],
 *              PARTITION_CONFIG (before BOOT_CONFIG) [179] etc. in EXT_CSD register),in this case this API configures
 *              bus width only from controller side and this API should be called after adi_emsi_SetSpeedMode() API.
 *
 * @param [in]  hInfo      Handle to the EMSI device to be configured.
 *
 * @param [in]  eWidth     The bus width to be used for eMSI transfers:\n
 *                         -eMMC supported bus widths:\n
 *                          #ADI_EMSI_BUS_WIDTH_1BIT = SDR - 1 bit/ Legacy - 1 bit,\n
 *                          #ADI_EMSI_BUS_WIDTH_4BIT = SDR - 4 bit/ Legacy - 4 bit/ DDR - 4 bit(Not supported while booting operation),\n
 *                          #ADI_EMSI_BUS_WIDTH_8BIT = SDR - 8 bit/ Legacy - 8 bit/ DDR - 8 bit(Not supported while booting operation).\n
 *
 *                         -SD card supported bus widths:\n
 *                          #ADI_EMSI_BUS_WIDTH_1BIT = SDR - 1 bit\n
 *                          #ADI_EMSI_BUS_WIDTH_4BIT = SDR - 4 bit\n
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS                   eMSI bus width set successful.
 *
 *    - #ADI_EMSI_BUSY                      Data transfer is not finished
 *
 *    - #ADI_EMSI_FAILURE                   Failure in sending command or Switch command failed.
 *
 *    - #ADI_EMSI_INVALID_CONFIGURATION     Invalid bus width.
 *
 * @sa          adi_emsi_SetSpeedMode()
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_SetBusWidth(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_BUS_WIDTH eWidth)
{
    /*Temporary variables*/
    uint32_t nTemp;
    uint32_t nTempSDStat;
    /*Card bus width*/
    uint8_t nBus_Width=0u;

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_14_10:"All if ... else if constructs shall be terminated with an else clause.")
#endif /* _MISRA_RULES */

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    if(pInfo->pDevice->eCardType == ADI_EMSI_CARD_TYPE_EMMC)
    {
        /*Change the bus width from eMSI controller side*/
        switch(eWidth)
        {
        case ADI_EMSI_BUS_WIDTH_1BIT:
            pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_DAT_XFER_WIDTH;
            pInfo->pEMSIRegs->CTL1 |= ENUM_EMSI_CTL1_ONE_BIT;
            nBus_Width = SDR_1_BIT_BUS_WIDTH;
            break;
        case ADI_EMSI_BUS_WIDTH_4BIT:
            pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_DAT_XFER_WIDTH;
            pInfo->pEMSIRegs->CTL1 |= ENUM_EMSI_CTL1_FOUR_BIT;
            if(pInfo->pDevice->eSpeed_Mode != ADI_EMSI_SPEED_MODE_HIGHSPEED_DDR)
            {
                /*SDR 4 bit from card side*/
                nBus_Width = SDR_4_BIT_BUS_WIDTH;
            }
            else
            {
                /*DDR 4 bit from Card Side*/
                nBus_Width = DDR_4_BIT_BUS_WIDTH;
            }
            break;
        case ADI_EMSI_BUS_WIDTH_8BIT:
            pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_EXT_DAT_XFER;
            pInfo->pEMSIRegs->CTL1 |= ENUM_EMSI_CTL1_EIGHT_BIT;
            if(pInfo->pDevice->eSpeed_Mode != ADI_EMSI_SPEED_MODE_HIGHSPEED_DDR)
            {
                /*SDR 8 bit from card side*/
                nBus_Width = SDR_8_BIT_BUS_WIDTH;
            }
            else
            {
                /*DDR 8 bit from Card Side*/
                nBus_Width = DDR_8_BIT_BUS_WIDTH;
            }
            break;
        default:
            return ADI_EMSI_INVALID_CONFIGURATION;
        }
    }
    else if(pInfo->pDevice->eCardType == ADI_EMSI_CARD_TYPE_SDCARD)
    {
        /*SD card support only 1 bit and 4 bit */
        /*Change the bus width from EMSI controller side*/
        switch(eWidth)
        {
        case ADI_EMSI_BUS_WIDTH_1BIT:
            pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_DAT_XFER_WIDTH;
            pInfo->pEMSIRegs->CTL1 |= ENUM_EMSI_CTL1_ONE_BIT;
            /*SDR 1 bit from card side*/
            nBus_Width = SDR_1_BIT_BUS_WIDTH_SD;
            break;
        case ADI_EMSI_BUS_WIDTH_4BIT:
            pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_DAT_XFER_WIDTH;
            pInfo->pEMSIRegs->CTL1 |= ENUM_EMSI_CTL1_FOUR_BIT;
            /*SDR 4 bit from card side*/
            nBus_Width = SDR_4_BIT_BUS_WIDTH_SD;
            break;
        default:
            return ADI_EMSI_INVALID_CONFIGURATION;
        }
    }
    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /*Configuring the bus width from card side if it is normal transfer*/
    if(pInfo->pDevice->eAppUsage == ADI_EMSI_APP_USE_NORMAL_DATATRANSFER)
    {
        if(pInfo->pDevice->eCardType == ADI_EMSI_CARD_TYPE_EMMC)
        {
            /*Parameters for CMD13*/
            ADI_EMSI_CMD_PARA CMD_PARA13 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                            ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                            ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

            /*Sending CMD6 (Switch) for switching bus width from card side*/
            ADI_EMSI_CMD_PARA CMD_PARA6 = {ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                           ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                           ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
            if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SWITCH,(((uint32_t)((uint32_t)nBus_Width<<BITP_VALUE) |
                                                         ((uint32_t)BUS_WIDTH_EXTCSD<<BITP_INDEX) |
                                                         ((uint32_t)ACCESS_WRITEBYTE<<BITP_ACCESS))),
                                                         &CMD_PARA6))
            {
                return ADI_EMSI_FAILURE;
            }

            /*Check if the switch was successful by reading the card status using CMD13*/
            if(ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SEND_STATUS,
                                                        (uint32_t)((uint32_t)pInfo->pDevice->nRca<<16),
                                                        &CMD_PARA13))
            {
                return ADI_EMSI_FAILURE;
            }
            adi_emsi_GetResponse(pInfo,&InternalResp[0]);
            /*Checking Switch error bit from response received*/
            nTemp = InternalResp[0]>>BITP_SWITCH_CMD_ERROR_CHK;
            nTemp = nTemp & SWITCH_CMD_ERROR_MASK;
            if(nTemp==SWITCH_CMD_ERROR)
            {
                /*Failure in CMD6 (Switch error bit is set in R1 response)*/
                return ADI_EMSI_FAILURE;
            }
        }
        else if(pInfo->pDevice->eCardType == ADI_EMSI_CARD_TYPE_SDCARD)
        {
            /*Parameters for CMD55 APP CMD*/
            ADI_EMSI_CMD_PARA CMD_PARA55 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                            ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                            ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

            /*Issue APP_CMD to indicate that the command following is application specific*/
            if(ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,APP_CMD,
                                                        (uint32_t)((uint32_t)pInfo->pDevice->nRca<<16),
                                                        &CMD_PARA55))
            {
                return ADI_EMSI_FAILURE;
            }
            /*Checking Status APP CMD bit in response*/
            adi_emsi_GetResponse(pInfo,&InternalResp[0]);
            nTemp = InternalResp[0]>>BITP_APP_CMD_CHK;
            nTemp = nTemp & APP_CMD_CHK_MASK;
            if (nTemp != APP_CMD_CHK)
            {
                return ADI_EMSI_FAILURE;
            }
            else
            {
                /*Sending ACMD6 (SET_BUS_WIDTH) for switching bus width from card side*/
                ADI_EMSI_CMD_PARA APP_CMD_PARA6 = {ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                                   ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                                   ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
                if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SET_BUS_WIDTH,
                                                             ((uint32_t)nBus_Width),
                                                             &APP_CMD_PARA6))
                {
                    return ADI_EMSI_FAILURE;
                }
                /*Read the SD status register to know if the bus width was changed successfully*/
                if(ADI_EMSI_SUCCESS != GetSdStatus(pInfo))
                {
                    return ADI_EMSI_FAILURE;
                }
                nTempSDStat = InternalSDStatus[0];
                nTempSDStat = nTempSDStat&SD_STATUS_MASK;
                nTempSDStat = nTempSDStat>>BITP_DAT_BUS_WIDTH;
                if(nTempSDStat != nBus_Width)
                {
                    return ADI_EMSI_FAILURE ;
                }
            }

        }
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = false;
#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

    return ADI_EMSI_SUCCESS;
}



/**
 * @brief       Sets DMA type for an upcoming eMSI transfer(s).
 *
 * @details     This API sets the DMA type for upcoming eMSI transfer.\n
 *              If user selects SDMA based transfers then adi_emsi_Blocking_Transfer()
 *              API should be used for data transfer.\n
 *              If user selects (ADMA2) DMA mode transfers then adi_emsi_NonBlocking_Transfer() or adi_emsi_CmdQue_Transfer()
 *              API should be used for data transfer.
 *              If user selects (ADMA3) DMA mode transfers then adi_emsi_Chained_Transfer()
 *              API should be used for data transfer. Also, ADMA3 based transfers are not supported if application is used for
 *              booting operations(ADI_EMSI_APP_USE_BOOTING_OPERATION).
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  eDmaType        DMA used for upcoming transfer:\n
 *                              #ADI_EMSI_DMA_USED_SDMA  : Use adi_emsi_Blocking_Transfer() API for upcoming transfers\n
 *                              #ADI_EMSI_DMA_USED_ADMA2 : Use adi_emsi_NonBlocking_Transfer() API for upcoming transfers
 *                                                         or For Command queuing operation ADMA2 should be selected,
 *                                                         in that case use adi_emsi_CmdQueTransfers() API\n
 *                              #ADI_EMSI_DMA_USED_ADMA3 : Use adi_emsi_Chained_Transfer() API for upcoming transfers\n
 *
 * @param [in]  eSdmaBufBdary   SDMA buffer boundary after which DMA interrupt will occur:
 *                              (This Parameter should be ignored if user uses any other DMA type except SDMA)\n
 *                              #ADI_EMSI_SDMA_BUF_BDARY_4K   : Buffer Boundary = 4KB(4096 Bytes)\n
 *                              #ADI_EMSI_SDMA_BUF_BDARY_8K   : Buffer Boundary = 8KB(8192 Bytes)\n
 *                              #ADI_EMSI_SDMA_BUF_BDARY_16K  : Buffer Boundary = 16KB(16384 Bytes)\n
 *                              #ADI_EMSI_SDMA_BUF_BDARY_32K  : Buffer Boundary = 32KB(32768 Bytes)\n
 *                              #ADI_EMSI_SDMA_BUF_BDARY_64K  : Buffer Boundary = 64KB(65536 Bytes)\n
 *                              #ADI_EMSI_SDMA_BUF_BDARY_128K : Buffer Boundary = 128KB(131072 Bytes)\n
 *                              #ADI_EMSI_SDMA_BUF_BDARY_256K : Buffer Boundary = 256KB(262144 Bytes)\n
 *                              #ADI_EMSI_SDMA_BUF_BDARY_512K : Buffer Boundary = 512KB(524288 Bytes)\n
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS       API call was successful.
 *
 *    - #ADI_EMSI_BUSY          Data transfer is not finished
 *
 * @note        This API should not be called when transfer is going on.
 */
ADI_EMSI_RESULT adi_emsi_SetDmaType(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_DMA_USED eDmaType,
                ADI_EMSI_SDMA_BUF_BDARY eSdmaBufBdary)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Storing the used DMA type*/
    pInfo->pDevice->eDmaUsed = eDmaType;

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    switch(eDmaType)
    {
    case ADI_EMSI_DMA_USED_ADMA2:
    {
        /*Choosing ADMA2 as DMA*/
        pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_DMA_SEL;
        pInfo->pEMSIRegs->CTL1 |= ENUM_EMSI_CTL1_ADMA2;
        break;
    }
    case ADI_EMSI_DMA_USED_SDMA:
    {
        /*Choosing SDMA as DMA*/
        pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_DMA_SEL;
        pInfo->pEMSIRegs->CTL1 |= ENUM_EMSI_CTL1_SDMA;
        /*Configuring buffer boundary*/
        pInfo->pEMSIRegs->BLKSZ &= ~BITM_EMSI_BLKSZ_SDMA_BUF_BDARY;
        switch(eSdmaBufBdary)
        {
        case(ADI_EMSI_SDMA_BUF_BDARY_4K):
            {
                pInfo->pEMSIRegs->BLKSZ |= ENUM_EMSI_BLKSZ_BYTES_4K;
                pInfo->pDevice->nSdmaBufBdary = SDMA_BUFFER_BOUNDARY_4K;
                break;
            }
        case(ADI_EMSI_SDMA_BUF_BDARY_8K):
            {
                pInfo->pEMSIRegs->BLKSZ |= ENUM_EMSI_BLKSZ_BYTES_8K;
                pInfo->pDevice->nSdmaBufBdary = SDMA_BUFFER_BOUNDARY_8K;
                break;
            }
        case(ADI_EMSI_SDMA_BUF_BDARY_16K):
            {
                pInfo->pEMSIRegs->BLKSZ |= ENUM_EMSI_BLKSZ_BYTES_16K;
                pInfo->pDevice->nSdmaBufBdary = SDMA_BUFFER_BOUNDARY_16K;
                break;
            }
        case(ADI_EMSI_SDMA_BUF_BDARY_32K):
            {
                pInfo->pEMSIRegs->BLKSZ |= ENUM_EMSI_BLKSZ_BYTES_32K;
                pInfo->pDevice->nSdmaBufBdary = SDMA_BUFFER_BOUNDARY_32K;
                break;
            }
        case(ADI_EMSI_SDMA_BUF_BDARY_64K):
            {
                pInfo->pEMSIRegs->BLKSZ |= ENUM_EMSI_BLKSZ_BYTES_64K;
                pInfo->pDevice->nSdmaBufBdary = SDMA_BUFFER_BOUNDARY_64K;
                break;
            }
        case(ADI_EMSI_SDMA_BUF_BDARY_128K):
            {
                pInfo->pEMSIRegs->BLKSZ |= ENUM_EMSI_BLKSZ_BYTES_128K;
                pInfo->pDevice->nSdmaBufBdary = SDMA_BUFFER_BOUNDARY_128K;
                break;
            }
        case(ADI_EMSI_SDMA_BUF_BDARY_256K):
            {
                pInfo->pEMSIRegs->BLKSZ |= ENUM_EMSI_BLKSZ_BYTES_256K;
                pInfo->pDevice->nSdmaBufBdary = SDMA_BUFFER_BOUNDARY_256K;
                break;
            }
        case(ADI_EMSI_SDMA_BUF_BDARY_512K):
            {
                pInfo->pEMSIRegs->BLKSZ |= ENUM_EMSI_BLKSZ_BYTES_512K;
                pInfo->pDevice->nSdmaBufBdary = SDMA_BUFFER_BOUNDARY_512K;
                break;
            }
        default:
            {
                return ADI_EMSI_INVALID_CONFIGURATION;
            }
        }

        break;
    }
    case ADI_EMSI_DMA_USED_ADMA3:
    {
        /*Choosing ADMA3 as DMA*/
        pInfo->pEMSIRegs->CTL1 &= ~BITM_EMSI_CTL1_DMA_SEL;
        pInfo->pEMSIRegs->CTL1 |= ENUM_EMSI_CTL1_ADMA2_3;
        /*Enable AUTO CMD23 from host control register2 hence AUTO_CMD_AUTO_SEL will select CMD23 as auto command */
        pInfo->pEMSIRegs->CTL2 &= ~BITM_EMSI_CTL2_CMD23_EN;
        pInfo->pEMSIRegs->CTL2 |= BITM_EMSI_CTL2_CMD23_EN;
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       Sets the block size for an upcoming EMSI transfer(s).
 *
 * @details     When the application usage is set to #ADI_EMSI_APP_USE_NORMAL_DATATRANSFER in adi_emsi_SetAppUse() API,
 *              this API sets the block size for upcoming EMSI transfers from card and controller side.
 *              While using DDR mode, block size should be set to 512 bytes (Card supports block size of 512 bytes in DDR mode as per
 *              JEDEC spec 5.1 for eMMC devices). This API should be called just before adi_emsi_Blocking_Transfer() or
 *              adi_emsi_NonBlocking_Transfer() API or adi_emsi_Chained_Transfer() API i.e., before going into transfers.
 *              Also, user should ensure that card is in transfer state while calling this API.
 *
 *              When the application usage is set to #ADI_EMSI_APP_USE_BOOTING_OPERATION in adi_emsi_SetAppUse() API,
 *              this API sets the block size only from controller side. This API should be called before going into
 *              transfers.
 *
 *              This API should be called after adi_emsi_SetSpeedMode() API.
 *
 *
 * @param [in]  hInfo               Handle to the EMSI device to be configured.
 *
 * @param [in]  nBlkSize            The number of bytes per block (User are requested to use 512 Bytes of block size as
 *                                  it is supported by most of the eMMC devices and SD cards).
 *                                  (Maximum value of block size is depends on READ_BL_LEN,WRITE_BL_LEN fields from card
 *                                   CSD registers).
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS               API call was successful.
 *
 *    - #ADI_EMSI_BLOCKSIZE_INVALID     Invalid block Size.
 *
 *    - #ADI_EMSI_FAILURE               Failure in sending command.
 *
 *    - #ADI_EMSI_BUSY                  Data transfer is not finished.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_SetBlkSze(
                ADI_EMSI_HANDLE const hInfo,
                uint16_t nBlkSize)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /* Check that the block size is within the limit */
    if ((nBlkSize <= BITM_EMSI_BLKSZ_XFER_BLKSZ))
    {
        if(pInfo->pDevice->eSpeed_Mode != ADI_EMSI_SPEED_MODE_HIGHSPEED_DDR)
        {
            /*Configuring the Block size*/
            pInfo->pEMSIRegs->BLKSZ &= ~BITM_EMSI_BLKSZ_XFER_BLKSZ;
            pInfo->pEMSIRegs->BLKSZ |= nBlkSize;
            pInfo->pDevice->nBlkSize = nBlkSize;
        }
        else
        {
            /*Configuring the Block size for DDR transfer, DDR transfer has default block size of 512 Bytes*/
            pInfo->pEMSIRegs->BLKSZ &= ~BITM_EMSI_BLKSZ_XFER_BLKSZ;
            pInfo->pEMSIRegs->BLKSZ |= DDR_TRANSFER_BLOCKSIZE;
            pInfo->pDevice->nBlkSize = DDR_TRANSFER_BLOCKSIZE;
        }
    }
    else
    {
        return ADI_EMSI_BLOCKSIZE_INVALID;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();


    /*Set block size from card side using SET_BLOCKLEN (CMD16) command (for DDR mode card block size will be 512 bytes)*/
    if ((pInfo->pDevice->eSpeed_Mode != ADI_EMSI_SPEED_MODE_HIGHSPEED_DDR) && (pInfo->pDevice->eAppUsage == ADI_EMSI_APP_USE_NORMAL_DATATRANSFER))
    {
        ADI_EMSI_CMD_PARA CMD_PARA16 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                        ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                        ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if(ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SET_BLOCKLEN,((uint32_t)pInfo->pDevice->nBlkSize),
                                                    &CMD_PARA16))
        {
            return ADI_EMSI_FAILURE;
        }
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = false;

    return ADI_EMSI_SUCCESS;
}



/**
 * @brief       Registers or unregisters a callback with the driver.
 *
 * @details     This callback will be called if any of the EMSI events listed under #ADI_EMSI_EVENT occurs.
 *
 * @param [in]  hInfo               Handle to the EMSI device with which the callback
 *                                  to be registered.
 *
 * @param [in]  pfCallback          Pointer to the callback function. The callback function
 *                                  has the prototype
 *                                  void callback(void *pCBParam, uint32_t nEvent, void *pArg)
 *
 * @param [in]  pCBParam            Callback parameter which will be returned back to the
 *                                  application when the callback function is called.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           If successfully registered a callback.
 **/

ADI_EMSI_RESULT adi_emsi_RegisterCallback(
                ADI_EMSI_HANDLE const hInfo,
                ADI_CALLBACK const pfCallback,
                void *const pCBParam)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /* Set the callback function and callback parameter */
    pInfo->pDevice->pfCallback     =   pfCallback;
    pInfo->pDevice->pCBParam       =   pCBParam;

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       Closes the given EMSI to uninitialized the EMSI instances memory.
 *
 * @details     Close the instance of the EMSI device. This API should be called when the services of the
 *              opened EMSI device are no longer needed by the application.
 *
 * @param [in]  hInfo               EMSI device handle whose operation is to be closed.
 *                                  This handle was obtained when the EMSI device instance was opened successfully.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS               Successfully closed the given instance.
 *
 *    - #ADI_EMSI_INTERRUPT_FAILURE     An interrupt error occurred.
 *
 *    - #ADI_EMSI_BUSY                  Data transfer is not finished
 *
 *
 * @sa          adi_emsi_Open()
 *
 * @note        No EMSI APIs should be called after adi_emsi_Close() API is called until the EMSI device is
 *              reinitialized using adi_emsi_Open() API again.
 **/
ADI_EMSI_RESULT adi_emsi_Close(
                ADI_EMSI_HANDLE const hInfo)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /* uninstall the interrupt handlers */
    if(adi_int_UninstallHandler(pInfo->StatusIntID) != ADI_INT_SUCCESS)
    {
        return ADI_EMSI_INTERRUPT_FAILURE;
    }
    if(adi_int_UninstallHandler(pInfo->WakupIntID) != ADI_INT_SUCCESS)
    {
        return ADI_EMSI_INTERRUPT_FAILURE;
    }

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /* Reset the EMSI registers for the EMSI instance opened for */
    ResetEmsiRegs(pInfo->pEMSIRegs);

    /* Reset and Clear EMSI Interrupt registers to default */
    ResetEmsiIntRegs(pInfo->pEMSIRegs);

    /*Reset the whole controller*/
    pInfo->pEMSIRegs->SWRST &= ~BITM_EMSI_SWRST_ALL;
    pInfo->pEMSIRegs->SWRST |= BITM_EMSI_SWRST_ALL;

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /* Free up the device */
    pInfo->pDevice = NULL;

    return ADI_EMSI_SUCCESS;
}



/**
 * @brief       Perform a Blocking transfer using SDMA mode (DMA mode).
 *
 * @details     This API performs Blocking transfer using SDMA mode (DMA mode).
 *              Where user selects the transfer type, submit the buffer for read or write
 *              operations, number of blocks to be transferred or received (nBlkCnt) and
 *              Start address of the block of the card (for >2GB cards)/ Byte address of the card (for <2GB cards)
 *              User waits in this API till transfer is complete. DMA type can be selected using adi_emsi_SetDmaType().
 *
 *              When the application usage is set to #ADI_EMSI_APP_USE_NORMAL_DATATRANSFER in adi_emsi_SetAppUse() API,
 *              This API performs all operations regarding data transfer including
 *              sending specific command related to data transfer (e.g. CMD17/18, CMD 23, CMD24/25 etc.) to card.
 *
 *              When the application usage is set to #ADI_EMSI_APP_USE_BOOTING_OPERATION in adi_emsi_SetAppUse() API,
 *              This API performs normal or alternate boot mode operation in SDMA mode (DMA mode).
 *
 *              It is advisable to use SDMA based data transfer for short chunks of data(Few kB's) for larger transfer
 *              ADMA usage is advised for performance improvement.
 *
 *              This API should be called after adi_emsi_SetBlkSze() API.
 *
 * @param [in]  hInfo           Handle to the EMSI device.
 *
 * @param [in]  eTranstype      Transfer Types:\n
 *                              #ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD = Single Block transfer (Read) (CMD17),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR = Single Block transfer (Write)(CMD24),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF = Multi Block Predefined transfer (Read) (CMD23->CMD18),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF = Multi Block Predefined transfer (Write) (CMD23->CMD25),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT = Normal boot (Not valid for SD card),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT = Alternate boot (Not valid for SD card).\n
 *                              *Note- Open-ended transfer not supported in blocking mode.
 *
 * @param [in]  nBlkCnt         The number of blocks to read or write.
 *
 * @param [in]  nStart_Address  Start address of the block of the card (for >2GB cards)/
 *                              Byte address of the card (for <=2GB cards) where data to be read or written.
 *                              In case of Normal and alternate boot this parameter should be ignored as card starts
 *                              sending data from 0 th byte/block.
 *
 * @param [in]  pBuffer         Pointer to  buffer to write or read the data to and from the card, respectively.
 *                              For SDMA based transfers user should ensure that buffers are aligned with memory boundary.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           Data transfer is finished
 *
 *    - #ADI_EMSI_BUSY              Data transfer is not finished
 *
 *    - #ADI_EMSI_FAILURE           Failure in sending commands
 *
 *    - #ADI_EMSI_INVALID_APIUSE    API use is not valid for selected application use
 *
 * @sa                          adi_emsi_SetDmaType()
 *
 **/
ADI_EMSI_RESULT adi_emsi_Blocking_Transfer(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_TRANSFER_TYPE eTranstype,
                uint16_t nBlkCnt,
                uint32_t nStart_Address,
                void *pBuffer)
{
    /*Temporary variable for storing buffer address of SDMA buffer*/
    uint32_t *pSdmaBufferAdd;
    /*Remaining Data (To Receive or sent)*/
    uint32_t TransferdataremainSdma = 0u;

    /*Updating eResult as not finished*/
    ADI_EMSI_RESULT eResult = ADI_EMSI_BUSY;

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Return if Application is not selected for booting and
     * transfer type is passed as ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT
     * or ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT*/
    if((pInfo->pDevice->eAppUsage != ADI_EMSI_APP_USE_BOOTING_OPERATION) &&
       ((pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT)||
        (pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT)))
    {
        /*This API use is not valid for selected application use*/
        return ADI_EMSI_INVALID_APIUSE;
    }

    /*Updating data progress as busy*/
    pInfo->pDevice->bData_Trans = true;

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /*Setting 16-bit Block Count for upcoming transfers*/
    pInfo->pEMSIRegs->BLKCNT &= (~(uint16_t)BITM_EMSI_BLKCNT_VALUE);
    pInfo->pEMSIRegs->BLKCNT = (nBlkCnt & BITM_EMSI_BLKCNT_VALUE);
    pInfo->pDevice->nBlkCnt = (nBlkCnt & BITM_EMSI_BLKCNT_VALUE);



    /*Setting Common Parameters for Transfer Mode register which will remain same for Non-Blocking mode of transfer*/
    pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_DMA_EN|
                                      BITM_EMSI_TRNSFRMODE_RESP_TYPE|
                                      BITM_EMSI_TRNSFRMODE_RESP_ERR_CHK_EN|
                                      BITM_EMSI_TRNSFRMODE_RESP_INT_DIS);

    pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_DMA_DISABLED;

    if((pInfo->pDevice->bControllerRespChk == true)&&(pInfo->pDevice->eAppUsage == ADI_EMSI_APP_USE_NORMAL_DATATRANSFER))
    {
        /*Not polling on command complete interrupt and response check by controller enabled*/
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                        ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_RESP_INT_DISABLED;
    }
    else
    {
        /*Polling on command complete interrupt and response check by controller disabled*/
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                        ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_RESP_INT_ENABLED;

    }

    /*Setting the transfer mode register parameters which will depend on mode of transfer*/
    switch(eTranstype)
    {
    case ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_WRITE|
                                        ENUM_EMSI_TRNSFRMODE_SINGLE;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_SINGLE;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_WRITE|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;
        /* For Non-DMA mode (Normal Boot) and SDMA mode operation Block size will always equal to 512B
         * ensuring block size to be 512 if user passes block size other than 512*/
        pInfo->pEMSIRegs->BLKSZ &= ~BITM_EMSI_BLKSZ_XFER_BLKSZ;
        pInfo->pEMSIRegs->BLKSZ = NORMAL_BOOT_BLOCKSIZE;
        pInfo->pDevice->nBlkSize = NORMAL_BOOT_BLOCKSIZE;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;

        /*Clearing the register*/
        pInfo->pEMSIRegs->BOOT_CTL = TWO_BYTE_ZERO;

        /*Whether boot acknowledgement is expected from card side setting same in controller*/
        if(pInfo->pDevice->bBoot_Ack == true)
        {
            /*Clearing before setting the bit*/
            pInfo->pEMSIRegs->BOOT_CTL &= ~BITM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE;
            pInfo->pEMSIRegs->BOOT_CTL |= ENUM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE_TRUE;
        }
        else
        {
            pInfo->pEMSIRegs->BOOT_CTL &= ~ENUM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE_TRUE;
        }

        /*Setting the boot control register*/
        pInfo->pEMSIRegs->BOOT_CTL |= ((uint16_t)((uint16_t)ADI_EMSI_TOUT_CNT_VALUE14<<BITP_EMSI_BOOT_CTL_BOOT_TOUT_CNT));
        /*The issuing CMD0 with 0xfffffffa will Start the alternate boot process
         * which will be done in Predefined_Command_Send function*/
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }


    /*Disabling the transfer complete interrupt to core as it is Blocking transfer*/
    pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

    if (pInfo->pDevice->eDmaUsed == ADI_EMSI_DMA_USED_SDMA)
    {
        /*Enable DMA in Transfer mode register for SDMA based transfers*/
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_DMA_EN);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_DMA_ENABLED;

        /*Disabling the DMA interrupt to core as it is SDMA based transfer and this interrupt occurs at each page boundary
         * as per set in adi_emsi_SetDmaType() API*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_DMA_INTERRUPT);

        /*Checking L1 Memory declaration*/
        pSdmaBufferAdd = (uint32_t *)adi_rtl_internal_to_system_addr((uint32_t)pBuffer,1);

        /*Remaining transfer data Depending on block size and block count*/
        TransferdataremainSdma = (uint32_t)((pInfo->pDevice->nBlkCnt) * (pInfo->pDevice->nBlkSize));

        /* Flush the buffer from the data cache(s).*/
        FlushDataCache(pSdmaBufferAdd, (pSdmaBufferAdd + (TransferdataremainSdma/4u)), true);

        /* Writing the start address of the buffer into system address register*/
        pInfo->pEMSIRegs->ADMA_ADDR_LO= (uint32_t)pSdmaBufferAdd;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /*Sending predefined commands based on transfer type*/
    if(ADI_EMSI_SUCCESS != Predefined_Command_Send(pInfo,nStart_Address))
    {
        return ADI_EMSI_FAILURE;
    }

    /*If blocking transfer is used for Normal boot operation*/
    if(eTranstype == ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT)
    {
        /*Clearing the register*/
        pInfo->pEMSIRegs->BOOT_CTL = TWO_BYTE_ZERO;

        /*Whether boot acknowledgement is expected from card side setting same in controller*/
        if(pInfo->pDevice->bBoot_Ack == true)
        {
            /*Clearing before setting the bit*/
            pInfo->pEMSIRegs->BOOT_CTL &= ~BITM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE;
            pInfo->pEMSIRegs->BOOT_CTL |= ENUM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE_TRUE;
        }
        else
        {
            pInfo->pEMSIRegs->BOOT_CTL &= ~ENUM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE_TRUE;
        }

        /*Setting the boot control register*/
        pInfo->pEMSIRegs->BOOT_CTL |= ((uint16_t)((uint16_t)ADI_EMSI_TOUT_CNT_VALUE14<<BITP_EMSI_BOOT_CTL_BOOT_TOUT_CNT)|
                                                  ENUM_EMSI_BOOT_CTL_VALIDATE_BOOT_TRUE|ENUM_EMSI_BOOT_CTL_MAN_BOOT_EN);
        /*This will Start the normal boot process*/
    }

    if (pInfo->pDevice->eDmaUsed == ADI_EMSI_DMA_USED_SDMA)
    {
        /*Updating the system address register if transfer is not finished*/
        if (ADI_EMSI_DATA_TRANS_FINISHED == SdmaUpdateBuffAddress(pInfo,pSdmaBufferAdd))
        {
            eResult = ADI_EMSI_SUCCESS;
        }

        /*Enabling the DMA interrupt to core if the next transfer is not based on SDMA*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_DMA_INTERRUPT);
        pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_DMA_INTERRUPT);
    }

    /*Enabling the transfer complete interrupt to core if the next transfer is non-blocking or based on ADMA3 transfers*/
    pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);
    pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

    /*Disabling boot ack for future data transfers*/
    if (pInfo->pEMSIRegs->BOOT_CTL & BITM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE)
    {
        /*Clearing the bit*/
        pInfo->pEMSIRegs->BOOT_CTL &= ~BITM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE;
    }

    /*Updating data progress as false as data transfer is finished*/
    pInfo->pDevice->bData_Trans = false;

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = false;


    return eResult;
}



/**
 * @brief       Perform a Non-blocking transfer using DMA mode (ADMA2).
 *
 * @details     This API performs Non-blocking transfer using DMA mode (ADMA2).
 *              Where user selects the transfer type, submit the buffer for read or write
 *              operations, number of blocks to be transferred  or received (nBlkCnt) and
 *              Start address of the block of the card (for >2GB cards)/ Byte address of the card (for <2GB cards)
 *              User waits in application for transfer to complete. DMA type can be selected using adi_emsi_SetDmaType().
 *
 *              When the application usage is set to #ADI_EMSI_APP_USE_NORMAL_DATATRANSFER in adi_emsi_SetAppUse() API,
 *              This API performs all operations regarding data transfer including
 *              sending specific command related to data transfer (e.g. CMD17/18, CMD 23, CMD24/25 etc.) to card.
 *
 *              When the application usage is set to #ADI_EMSI_APP_USE_BOOTING_OPERATION in adi_emsi_SetAppUse() API,
 *              This API performs normal or alternate boot mode operation in DMA mode (ADMA2).
 *
 *              Please note that for Open ended data transfer (#ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_OPEN_ENDED, #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_OPEN_ENDED)
 *              size depends on number of descriptor (NUM_DMA_DESCRIPTORS) given in  adi_emsi_config_SC59x.h,
 *              for example if number of descriptor given as 5 then maximum total data transfer size is one API call will  be (64MB * 5 = 320MB).
 *
 *              Note: This API can be called multiple times for large chunk(>32MB) of predefined transfers as one API call can
 *              perform ~32 MB of data transfer.
 *
 *
 *              This API should be called after adi_emsi_SetBlkSze() API.
 *
 * @param [in]  hInfo           Handle to the EMSI device.
 *
 * @param [in]  eTranstype      Transfer Types:\n
 *                              #ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD = Single Block transfer (Read) (CMD17),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR = Single Block transfer (Write)(CMD24),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF = Multi Block Predefined transfer (Read) (CMD23->CMD18),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF = Multi Block Predefined transfer (Write) (CMD23->CMD25),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_OPEN_ENDED = Multi Block Open ended transfer (Read) (CMD18) (Not valid for SD card),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_OPEN_ENDED = Multi Block Open ended transfer (Write) (CMD25)(Not valid for SD card),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT = Normal boot (Not valid for SD card),\n
 *                              #ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT = Alternate boot (Not valid for SD card).
 *
 * @param [in]  nBlkCnt         The number of blocks to read or write.
 *
 * @param [in]  nStart_Address  Start address of the block of the card (for >2GB cards)/
 *                              Byte address of the card (for <2GB cards) where data to be read or written.
 *                              In case of Normal and alternate boot this parameter should be ignored as card starts
 *                              sending data from 0 th byte/block.
 *
 * @param [in]  pBuffer         Pointer to  buffer to write or read the data to and from the card, respectively.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           If successfully data is configured for non-blocking transfer.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished, wait transfer complete interrupt.
 *
 *    - #ADI_EMSI_FAILURE           Failure in sending commands
 *
 *    - #ADI_EMSI_INVALID_APIUSE    API use is not valid for selected application use
 *
 * @sa                          adi_emsi_SetDmaType()
 **/
ADI_EMSI_RESULT adi_emsi_NonBlocking_Transfer(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_TRANSFER_TYPE eTranstype,
                uint16_t nBlkCnt,
                uint32_t nStart_Address,
                void *pBuffer)
{

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Return if Application is not selected for booting and
     * transfer type is passed as ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT
     * or ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT*/
    if((pInfo->pDevice->eAppUsage != ADI_EMSI_APP_USE_BOOTING_OPERATION) &&
       ((pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT)||
        (pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT)))
    {
        /*This API use is not valid for selected application use*/
        return ADI_EMSI_INVALID_APIUSE;
    }

    /*Updating data progress as busy*/
    pInfo->pDevice->bData_Trans = true;

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /*Setting Block Count for upcoming transfers*/
    pInfo->pEMSIRegs->BLKCNT &=(~(uint16_t)BITM_EMSI_BLKCNT_VALUE);
    pInfo->pEMSIRegs->BLKCNT = (nBlkCnt & BITM_EMSI_BLKCNT_VALUE);
    pInfo->pDevice->nBlkCnt = (nBlkCnt & BITM_EMSI_BLKCNT_VALUE);

    /*Setting Common Parameters for Transfer Mode register which will remain same for Blocking mode of transfer*/
    pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_DMA_EN|
                                      BITM_EMSI_TRNSFRMODE_RESP_TYPE|
                                      BITM_EMSI_TRNSFRMODE_RESP_ERR_CHK_EN|
                                      BITM_EMSI_TRNSFRMODE_RESP_INT_DIS);

    pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_DMA_ENABLED;

    if((pInfo->pDevice->bControllerRespChk == true)&&(pInfo->pDevice->eAppUsage == ADI_EMSI_APP_USE_NORMAL_DATATRANSFER))
    {
        /*Not polling on command complete interrupt and response check by controller enabled*/
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                        ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_RESP_INT_DISABLED;
    }
    else
    {
        /*Polling on command complete interrupt and response check by controller disabled*/
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                        ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_RESP_INT_ENABLED;

    }



    /*Setting the transfer mode register parameters which will depend on mode of transfer*/
    switch(eTranstype)
    {
    case ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_WRITE|
                                        ENUM_EMSI_TRNSFRMODE_SINGLE;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_SINGLE;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_WRITE|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_OPEN_ENDED:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_OPEN_ENDED;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_WRITE|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;

        /*Ensuring block count is set to zero for open ended transfer*/
        pInfo->pEMSIRegs->BLKCNT = TWO_BYTE_ZERO;
        pInfo->pDevice->nBlkCnt = TWO_BYTE_ZERO;
        /*32-bit block count gets enabled once 16-bit block count is zero
        Configuring 32-bit block count to zero for open ended transfers*/
        pInfo->pEMSIRegs->SDMA_ADDR = FOUR_BYTE_ZERO;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_OPEN_ENDED:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_OPEN_ENDED;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;

        /*Ensuring block count is set to zero for open ended transfer*/
        pInfo->pEMSIRegs->BLKCNT = TWO_BYTE_ZERO;
        pInfo->pDevice->nBlkCnt = TWO_BYTE_ZERO;
        /*32-bit block count get enabled once 16-bit block count is zero
        Configuring 32-bit block count to zero for open ended transfers*/
        pInfo->pEMSIRegs->SDMA_ADDR = FOUR_BYTE_ZERO;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT:
    {
        pInfo->pDevice->eTransferType = ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;

        /*Clearing the register*/
        pInfo->pEMSIRegs->BOOT_CTL = TWO_BYTE_ZERO;

        /*Whether boot acknowledgement is expected from card side setting same in controller*/
        if(pInfo->pDevice->bBoot_Ack == true)
        {
            /*Clearing before setting the bit*/
            pInfo->pEMSIRegs->BOOT_CTL &= ~BITM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE;
            pInfo->pEMSIRegs->BOOT_CTL |= ENUM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE_TRUE;
        }
        else
        {
            pInfo->pEMSIRegs->BOOT_CTL &= ~ENUM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE_TRUE;
        }

        /*setting the boot control register*/
        pInfo->pEMSIRegs->BOOT_CTL |= ((uint16_t)((uint16_t)ADI_EMSI_TOUT_CNT_VALUE14<<BITP_EMSI_BOOT_CTL_BOOT_TOUT_CNT));
        /*The issuing CMD0 with 0xfffffffa will Start the alternate boot process
         * which will be done in Predefined_Command_Send function*/
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

    /*Configure the DMA descriptors for data transfer */
    if(ADI_EMSI_SUCCESS != DmaConfig(pInfo,pBuffer))
    {
        return ADI_EMSI_FAILURE;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /*Sending predefined commands based on transfer type*/
    if(ADI_EMSI_SUCCESS != Predefined_Command_Send(pInfo,nStart_Address))
    {
        return ADI_EMSI_FAILURE;
    }

    /*If Nonblocking transfer is used for Normal boot operation*/
    if(eTranstype == ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT)
    {
        /*Clearing the register*/
        pInfo->pEMSIRegs->BOOT_CTL = TWO_BYTE_ZERO;

        /*Whether boot acknowledgement is expected from card side setting same in controller*/
        if(pInfo->pDevice->bBoot_Ack == true)
        {
            /*Clearing before setting the bit*/
            pInfo->pEMSIRegs->BOOT_CTL &= ~BITM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE;
            pInfo->pEMSIRegs->BOOT_CTL |= ENUM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE_TRUE;
        }
        else
        {
            pInfo->pEMSIRegs->BOOT_CTL &= ~ENUM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE_TRUE;
        }

        /*setting the boot control register*/
        pInfo->pEMSIRegs->BOOT_CTL |= ((uint16_t)((uint16_t)ADI_EMSI_TOUT_CNT_VALUE14<<BITP_EMSI_BOOT_CTL_BOOT_TOUT_CNT)|
                                                  ENUM_EMSI_BOOT_CTL_VALIDATE_BOOT_TRUE|ENUM_EMSI_BOOT_CTL_MAN_BOOT_EN);
        /*This will Start the boot process*/
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = false;

    /*Check for transfer complete in call back*/
    return ADI_EMSI_SUCCESS;
}



/**
 * @brief       Automatic command send feature to issue CMD12 or CMD23 .
 *
 * @details     This API saves the type of auto command CMD 12 or CMD23, which
 *              should be send to the card. Also configures transfer mode register accordingly.
 *              This API should be called before adi_emsi_NonBlocking_Transfer() in case
 *              of non-blocking transfer or before adi_emsi_Blocking_TransType() in case
 *              of blocking transfer. This should be called only when application usage is set to
 *              #ADI_EMSI_APP_USE_NORMAL_DATATRANSFER in adi_emsi_SetAppUse() API.
 *              *Note: Automatic command feature should be set to #ADI_EMSI_AUTO_CMD_DIS in case of ADMA3 based transfers.
 *              (As ADMA3 uses CMD23 as auto command by default selections.)
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  eAutoCmd        Auto Command to be send:\n
 *                              #ADI_EMSI_AUTO_CMD_DIS:No auto command to be send,\n
 *
 *                              #ADI_EMSI_AUTO_CMD12EN:CMD 12 as auto command
 *                              (This Should not be used in case of predefined transfer (Single or pre-defined multiblock),
 *                              also should not be used while open ended transfers
 *                              as open ended read or write will be terminated using adi_emsi_AbortTrasnfer() API).
 *                              This feature is useful only when there is no multi block predefined transfer (i.e. CMD23 is not used) from card side
 *                              , block count>0 and block count is enabled.\n
 *
 *
 *                              #ADI_EMSI_AUTO_CMD23EN:CMD 23 as auto command
 *                              (This Should be used in case of #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF
 *                              and #ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF transfers or in any pre-defined multi-block transfer).
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 *    - #ADI_EMSI_BUSY              Data transfer is not finished
 *
 *    - #ADI_EMSI_INVALID_APIUSE    API use is not valid for selected application use
 **/

ADI_EMSI_RESULT adi_emsi_AutoCmd(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_AUTO_CMD eAutoCmd)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif


    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Return if Application is used for booting*/
    if(pInfo->pDevice->eAppUsage != ADI_EMSI_APP_USE_NORMAL_DATATRANSFER)
    {
        return ADI_EMSI_INVALID_APIUSE;
    }

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    switch(eAutoCmd)
    {
    case ADI_EMSI_AUTO_CMD_DIS:
    {
        pInfo->pDevice->eAutoCmdType = ADI_EMSI_AUTO_CMD_DIS;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_AUTO_CMD_EN);
        pInfo->pEMSIRegs->TRNSFRMODE |= (ENUM_EMSI_TRNSFRMODE_AUTO_CMD_DISABLED);
        break;
    }
    case ADI_EMSI_AUTO_CMD12EN:
    {
        pInfo->pDevice->eAutoCmdType = ADI_EMSI_AUTO_CMD12EN;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_AUTO_CMD_EN);
        pInfo->pEMSIRegs->TRNSFRMODE |= (ENUM_EMSI_TRNSFRMODE_AUTO_CMD12_ENABLED);
        break;
    }
    case ADI_EMSI_AUTO_CMD23EN:
    {
        pInfo->pDevice->eAutoCmdType = ADI_EMSI_AUTO_CMD23EN;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_AUTO_CMD_EN);
        pInfo->pEMSIRegs->TRNSFRMODE |= (ENUM_EMSI_TRNSFRMODE_AUTO_CMD23_ENABLED);
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       To abort the ongoing open-ended/predefined transfers.
 *
 * @details     This API is used to stop the ongoing open-ended/predefined transfers using CMD 12.
 *              Asynchronous abort stops the transfer abruptly. This API should used
 *              after adi_emsi_NonBlocking_Transfer() or adi_emsi_Chained_Transfer() or adi_emsi_General_Transfer() API.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  eAborttype      Asynchronous abort sequence:\n
 *                              #ADI_EMSI_ABORT_TYPE_ASYNC : Stops transfer abruptly\n
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful card is in transfer state.
 *
 *    - #ADI_EMSI_FAILURE           Card Fail to enter in transfer state abort failed.
 *
 *    - #ADI_EMSI_INVALID_APIUSE    API use is not valid for selected application use
 **/

ADI_EMSI_RESULT adi_emsi_AbortTransfer(
        ADI_EMSI_HANDLE const hInfo,
        ADI_EMSI_ABORT_TYPE eAborttype)
{

    ADI_EMSI_RESULT eResult = ADI_EMSI_SUCCESS;

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    /*Return if Application is used for booting*/
    if(pInfo->pDevice->eAppUsage != ADI_EMSI_APP_USE_NORMAL_DATATRANSFER)
    {
        return ADI_EMSI_INVALID_APIUSE;
    }

    switch (eAborttype)
    {
    case ADI_EMSI_ABORT_TYPE_ASYNC:
    {
        /*Issue Abort Sequence to card*/
        eResult = Abort_Command_Seq(pInfo);

        /*Reset Command and data lines*/
        ResetCmdDataLine(pInfo);

        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

    /*General transfer aborted hence clearing the flag*/
    bGeneral_Transfer = false;

    /*Updating data progress as finished*/
    pInfo->pDevice->bData_Trans = false;

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = false;

    return eResult;
}

/**
 * @brief       Set whether external socket is used for card.
 *
 * @details     This API sets whether external socket is used for connecting the card.
 *              User selects whether card is inserted in external socket or on board eMMC device
 *              is used. By default on board is eMMC is selected in adi_emsi_Open() API.
 *              Also this API returns status whether card is present in socket or not.
 *              Usually this API is used only once at the starting of application.
 *
 * @param [in]  hInfo       Handle to the EMSI device to be configured.
 *
 * @param [in]  bExtSocket  External socket used:\n
 *                          - true: Card is inserted in external socket
 *                          - false: On board eMMC device is used.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS       API call was successful.
 *
 *    - #ADI_EMSI_BUSY          Data transfer is not finished
 *
 *    - #ADI_EMSI_CARD_INSERTED Card is inserted into external socket
 *
 *    - #ADI_EMSI_CARD_REMOVED  Card is not inserted into external socket
 *
 * @sa          adi_emsi_Open()
 *
 * @note        This API should not be called when transfer is going on.
 */
ADI_EMSI_RESULT adi_emsi_SetCardConnector(
                ADI_EMSI_HANDLE const hInfo,
                bool bExtSocket)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    if(bExtSocket == true)
    {
        /*Enabling the Card removal and card insertion interrupts to notify the application*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~((BITM_EMSI_ISTAT_EN_CARD_INSERTION)|(BITM_EMSI_ISTAT_EN_CARD_REMOVAL));
        pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_EN_CARD_INSERTION)|(BITM_EMSI_ISTAT_EN_CARD_REMOVAL);

        /*External card connector is used for connecting the card*/
        pInfo->pDevice->bExtSocketUsed = true;

        /*Check card insertion using Present STATE register*/
        if(pInfo->pEMSIRegs->PSTATE & BITM_EMSI_PSTATE_CARD_INSERTED)
        {
            /*Card is inserted into external socket*/
            return ADI_EMSI_CARD_INSERTED;
        }
        else
        {
            /*Card is not present in external socket*/
            return ADI_EMSI_CARD_REMOVED;
        }
    }
    else
    {
        /*Disabling the Card removal and card insertion interrupts*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~((BITM_EMSI_ISTAT_EN_CARD_INSERTION)|(BITM_EMSI_ISTAT_EN_CARD_REMOVAL));

        /*On board eMMC device is used*/
        pInfo->pDevice->bExtSocketUsed = false;
    }

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       Perform a Non-blocking general read/write using DMA mode (ADMA2).
 *
 * @details     This API configures the DMA descriptors  for ADMA2 mode.
 *              User also selects the transfer type, submit the buffer for read or write
 *              operations, number of blocks to be transferred  or received (nBlkCnt).
 *              Once API call is complete user issue the command using adi_emsi_SendCommand() API in application. And wait for
 *              transfer complete interrupt in application after read or write command is issued.
 *              DMA type should be set to ADMA2 using adi_emsi_SetDmaType() API.
 *
 *              This API should be called after adi_emsi_SetBlkSze() API.
 *
 *              This API main purpose is to do miscellaneous (adtc (addressed (point-to-point) data transfer commands (adtc),
 *              data transfer on DAT lines.)) data transfers (e.g. CMD8(SEND_EXT_CSD) to receive EXT_CSD register data block, CMD19(BUSTEST_W) Bus test).
 *
 * @param [in]  hInfo           Handle to the EMSI device.
 *
 * @param [in]  eGenTransType   Transfer Types:\n
 *                              #ADI_EMSI_GENERAL_TRANS_TYPE_READ = Predefined Multiblock read\n
 *                              #ADI_EMSI_GENERAL_TRANS_TYPE_WRITE = Predefined Multiblock write\n
 *
 * @param [in]  nBlkCnt         The number of blocks to read or write to card.
 *
 * @param [in]  pBuffer         Pointer to  buffer to write or read the data to and from the card, respectively.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           If successfully data is configured for non-blocking transfer.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished, wait transfer complete interrupt.
 *
 * @sa                          adi_emsi_SetDmaType()
 *
 **/
ADI_EMSI_RESULT adi_emsi_General_Transfer(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_GENERAL_TRANS_TYPE eGenTransType,
                uint16_t nBlkCnt,
                void *pBuffer)
{

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /*Setting Block Count for upcoming transfers*/
    pInfo->pEMSIRegs->BLKCNT &=(~(uint16_t)BITM_EMSI_BLKCNT_VALUE);
    pInfo->pEMSIRegs->BLKCNT = (nBlkCnt & BITM_EMSI_BLKCNT_VALUE);
    pInfo->pDevice->nBlkCnt = (nBlkCnt & BITM_EMSI_BLKCNT_VALUE);

    /*Setting Common Parameters for Transfer Mode register which will remain same for ADMA2 based transfer*/
    pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_DMA_EN|
                                      BITM_EMSI_TRNSFRMODE_RESP_TYPE|
                                      BITM_EMSI_TRNSFRMODE_RESP_ERR_CHK_EN|
                                      BITM_EMSI_TRNSFRMODE_RESP_INT_DIS);

    pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_DMA_ENABLED;

    if((pInfo->pDevice->bControllerRespChk == true)&&(pInfo->pDevice->eAppUsage == ADI_EMSI_APP_USE_NORMAL_DATATRANSFER))
    {
        /*Not polling on command complete interrupt and response check by controller enabled*/
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                        ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_RESP_INT_DISABLED;
    }
    else
    {
        /*Polling on command complete interrupt and response check by controller disabled*/
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                        ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_DISABLED|
                                        ENUM_EMSI_TRNSFRMODE_RESP_INT_ENABLED;
    }



    /*Setting the transfer mode register parameters which will depend on mode of transfer*/
    switch(eGenTransType)
    {

    case ADI_EMSI_GENERAL_TRANS_TYPE_WRITE:
    {
        /*Set general transfer flag*/
        bGeneral_Transfer = true;
        pInfo->pDevice->eGenTransType = ADI_EMSI_GENERAL_TRANS_TYPE_WRITE;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_WRITE|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;
        break;
    }
    case ADI_EMSI_GENERAL_TRANS_TYPE_READ:
    {
        /*Set general transfer flag*/
        bGeneral_Transfer = true;
        pInfo->pDevice->eGenTransType = ADI_EMSI_GENERAL_TRANS_TYPE_READ;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                          BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR|
                                          BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);
        pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                        ENUM_EMSI_TRNSFRMODE_READ|
                                        ENUM_EMSI_TRNSFRMODE_MULTI;
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

    /*Configure the DMA descriptors for data transfer */
    if(ADI_EMSI_SUCCESS != DmaConfig(pInfo,pBuffer))
    {
        return ADI_EMSI_FAILURE;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /*Check for transfer complete in call back after required command is send*/
    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       Perform a chain descriptor based transfers using DMA mode (ADMA3).
 *
 * @details     This API performs chain descriptor based transfers. chain descriptor based transfers are tasks based transfers.
 *              User submits tasks using pAdma3InputArray structure array. This tasks are encoded in chained descriptors.
 *              Transfer complete interrupt will get raised once all tasks are completed. For more information refer to HRM. Before calling this API,
 *              DMA type should be set to ADMA3 using adi_emsi_SetDmaType() API. Also for each task maximum buffer size is valid up to (32 MB -512 bytes)(for block size of 512 bytes) only.
 *
 *              *Note: Booting operations and open ended transfers are not possible with ADMA3 (Block Count =0)
 *
 * @param [in]  hInfo               Handle to the EMSI device to be configured.
 *
 * @param [in]  pAdma3InputArray    Input parameters for creating ADMA3 descriptors:\n
 *                                  1)Block Count: The number of blocks to read or write.\n
 *                                  2)Block Size (Same block size should be used while calling adi_emsi_SetBlkSze() API):
 *                                    The number of bytes per block (User are requested to use 512 Bytes of block size as it is supported
 *                                    by most of the eMMC devices and SD cards). (Maximum value of block size is depends
 *                                    on READ_BL_LEN,WRITE_BL_LEN fields from card CSD registers).\n
 *                                  3)Start address of the block of the card (for >2GB cards)/
 *                                    Byte address of the card (for <2GB cards) where data to be read or written.\n
 *                                  4)Data transfer direction:\n
 *                                    Write transfers (from host to card)\n
 *                                    Read transfers (from card to host)\n
 *                                  5)Pointer to  buffer to write or read the data to and from the card, respectively.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_Chained_Transfer(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_CHAIN_TRANSFER_INPUT *pAdma3InputArray)
{
    /*Temporary variables*/
    /*Descriptor pair index*/
    uint32_t Index;
    /*ADMA3 Integrated Descriptor pair index*/
    uint32_t InteIndex;
    /*Transfer data remaining*/
    uint32_t TransferdataremainADMA3;
    /*Descriptor related variables*/
    uint16_t Lower_16_bits;
    uint16_t Upper_10_bits;
    uint32_t Upper_10_bits_temp;
    uint32_t *BufferAddress;
    /*Actual command descriptor pairs*/
    uint8_t CommandDesPairs=0u;

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Updating data progress as busy*/
    pInfo->pDevice->bData_Trans = true;

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    /*Creating command descriptor pairs*/
    for (Index=0u; Index<ADMA3_DMA_DESCRIPTORS_PAIRS; Index++)
    {
        /*Checking if descriptor needs to be created*/
        if (pAdma3InputArray[Index].nBlkCntAdma3 != (uint32_t)NULL)
        {
            /*Configuring Block Count in command descriptor*/
            vADMA3DesPair[Index].DES0 = VALID_SET;
            vADMA3DesPair[Index].DES1 = pAdma3InputArray[Index].nBlkCntAdma3;

            /*Configuring Block Size in command descriptor*/
            vADMA3DesPair[Index].DES2 = VALID_SET;
            vADMA3DesPair[Index].DES3 = pAdma3InputArray[Index].nBlksizeAdma3;

            /*Configuring Start Address of the card from where data will be read or write in command descriptor*/
            vADMA3DesPair[Index].DES4 = VALID_SET;
            vADMA3DesPair[Index].DES5 = pAdma3InputArray[Index].nStartAddressAdma3;

            /*Configuring CMD Register and Transfer Register in command descriptor
             * END is not set as this descriptor is used for data transfer*/
            vADMA3DesPair[Index].DES6 = VALID_SET;
            if(pAdma3InputArray[Index].eDataDirectionAdma3 == ADI_EMSI_DATA_XFER_DIR_WRITE)
            {
                /*Write Multiblock predefined*/
                vADMA3DesPair[Index].DES7 = ((((ENUM_EMSI_CMD_RESP_LEN_48|
                                                ENUM_EMSI_CMD_MAIN|
                                                ENUM_EMSI_CMD_CMD_CRC_CHK_ENABLED|
                                                ENUM_EMSI_CMD_CMD_IDX_CHK_ENABLED|
                                                ENUM_EMSI_CMD_DATA|
                                                ENUM_EMSI_CMD_NORMAL_CMD|
                                                ((uint32_t)((uint32_t)WRITE_MULTIPLE_BLOCK<<BITP_EMSI_CMD_INDEX)))&CMD_REG_MASK)<<BITP_CMD_REG)|
                                                ((ENUM_EMSI_TRNSFRMODE_DMA_ENABLED|
                                                ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                                ENUM_EMSI_TRNSFRMODE_AUTO_CMD_AUTO_SEL|
                                                ENUM_EMSI_TRNSFRMODE_WRITE|
                                                ENUM_EMSI_TRNSFRMODE_MULTI)&TRANSFER_MODE_REG_MASK));

                if((pInfo->pDevice->bControllerRespChk == true)&&(pInfo->pDevice->eAppUsage == ADI_EMSI_APP_USE_NORMAL_DATATRANSFER))
                {
                    /*Response check by controller enabled*/
                    vADMA3DesPair[Index].DES7 |= ((ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                                   ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_ENABLED|
                                                   ENUM_EMSI_TRNSFRMODE_RESP_INT_DISABLED)&TRANSFER_MODE_REG_MASK);

                }
                else
                {

                    /*Response check by controller disabled*/
                    vADMA3DesPair[Index].DES7 |= ((ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                                   ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_DISABLED|
                                                   ENUM_EMSI_TRNSFRMODE_RESP_INT_ENABLED)&TRANSFER_MODE_REG_MASK);

                }

            }
            else
            {
                /*Read Multiblock predefined*/
                vADMA3DesPair[Index].DES7 = ((((ENUM_EMSI_CMD_RESP_LEN_48|
                                                ENUM_EMSI_CMD_MAIN|
                                                ENUM_EMSI_CMD_CMD_CRC_CHK_ENABLED|
                                                ENUM_EMSI_CMD_CMD_IDX_CHK_ENABLED|
                                                ENUM_EMSI_CMD_DATA|
                                                ENUM_EMSI_CMD_NORMAL_CMD|
                                                ((uint32_t)((uint32_t)READ_MULTIPLE_BLOCK<<BITP_EMSI_CMD_INDEX)))&CMD_REG_MASK)<<BITP_CMD_REG)|
                                                ((ENUM_EMSI_TRNSFRMODE_DMA_ENABLED|
                                                ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                                ENUM_EMSI_TRNSFRMODE_AUTO_CMD_AUTO_SEL|
                                                ENUM_EMSI_TRNSFRMODE_READ|
                                                ENUM_EMSI_TRNSFRMODE_MULTI)&TRANSFER_MODE_REG_MASK));

                if((pInfo->pDevice->bControllerRespChk == true)&&(pInfo->pDevice->eAppUsage != ADI_EMSI_APP_USE_NORMAL_DATATRANSFER))
                {
                    /*Response check by controller enabled*/
                    vADMA3DesPair[Index].DES7 |= ((ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                                   ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_ENABLED|
                                                   ENUM_EMSI_TRNSFRMODE_RESP_INT_DISABLED)&TRANSFER_MODE_REG_MASK);
                }
                else
                {

                    /*Response check by controller disabled*/
                    vADMA3DesPair[Index].DES7 |= ((ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                                   ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_DISABLED|
                                                   ENUM_EMSI_TRNSFRMODE_RESP_INT_ENABLED)&TRANSFER_MODE_REG_MASK);
                }


            }

            /*Remaining Data (To Receive or sent)*/
            TransferdataremainADMA3 = pAdma3InputArray[Index].nBlkCntAdma3 * pAdma3InputArray[Index].nBlksizeAdma3;
            Lower_16_bits = (uint16_t)((uint32_t)TransferdataremainADMA3 & UPPER_16BIT_DATA_LENGTH_MASK);
            Upper_10_bits_temp = TransferdataremainADMA3 & LOWER_10BIT_DATA_LENGTH_MASK;
            Upper_10_bits = (uint16_t)(Upper_10_bits_temp>>BITP_26_DATA_LENGTH_LO_16BIT);

            /*ADMA2 Descriptor pair*/
            if(TransferdataremainADMA3 ==  DESCRIPTOR_DATA_TRANSFER_LIMIT)
            {
                vADMA3DesPair[Index].DES8 = (uint32_t)(VALID_INTER_END_SET|
                                                      (uint32_t)((uint32_t)UPPER_16BIT_DATA_LENGTH<<
                                                                 BITP_26_DATA_LENGTH_LO_16BIT)|
                                                      (uint32_t)((uint32_t)LOWER_10BIT_DATA_LENGTH<<
                                                                 BITP_26_DATA_LENGTH_UP_10BIT));
            }
            else
            {
                vADMA3DesPair[Index].DES8 = ((uint32_t)(VALID_INTER_END_SET|
                                                       (uint32_t)((uint32_t)Lower_16_bits<<
                                                                  BITP_26_DATA_LENGTH_LO_16BIT)|
                                                       (uint32_t)((uint32_t)Upper_10_bits<<
                                                                  BITP_26_DATA_LENGTH_UP_10BIT)));

            }
            /*Checking L1 Memory declaration*/
            BufferAddress = (uint32_t *)adi_rtl_internal_to_system_addr((uint32_t)pAdma3InputArray[Index].pBufferAdma3,1);

            /*Assigning Buffer address*/
#if defined(__ADSPARM__)
            vADMA3DesPair[Index].DES9 = (uintptr_t)BufferAddress;
#else
            vADMA3DesPair[Index].DES9 = (uint32_t*)BufferAddress;
#endif
            /* Flush the buffer from the data cache(s).*/
            FlushDataCache(BufferAddress, (BufferAddress + (TransferdataremainADMA3/4u)), true);
            CommandDesPairs++;
        }
    }

    /*Creating ADMA3 Integrated descriptors*/
    for (InteIndex=0u; InteIndex<CommandDesPairs; InteIndex++)
    {
        if(InteIndex != (uint32_t)(CommandDesPairs-1u))
        {
            /*Normal Descriptor*/
            vADMA3IntDescriptor[InteIndex].DES0 = VALID_SET_INTEGRATED_DES;
        }
        else
        {
            /*Last Descriptor*/
            vADMA3IntDescriptor[InteIndex].DES0 = VALID_END_SET_INTEGRATED_DES;
        }

        /*Write Start address of each command descriptor into integrated descriptors*/
#if defined(__ADSPARM__)
        vADMA3IntDescriptor[InteIndex].DES1 = (uint32_t)adi_rtl_internal_to_system_addr((&vADMA3DesPair[InteIndex].DES0),1);
#else
        vADMA3IntDescriptor[InteIndex].DES1 = (uint32_t *)adi_rtl_internal_to_system_addr(((uint32_t)&vADMA3DesPair[InteIndex].DES0),1);
#endif

    }

    /*Flush the descriptors from the cache so that the DMA engine can see them*/
    FlushDataCache((uint32_t*)(&vADMA3DesPair),(uint32_t*)(&vADMA3DesPair + 1u), true);

    /*Flush the descriptors from the cache so that the DMA engine can see them*/
    FlushDataCache((uint32_t*)(&vADMA3IntDescriptor),(uint32_t*)(&vADMA3IntDescriptor + 1u), true);

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    /* Write the start of the integrated descriptor list to the ADMA3 Integrated Descriptor Address Register to initiate the transfer*/
    pInfo->pEMSIRegs->ADMA_DESADDR_LO = (uint32_t)adi_rtl_internal_to_system_addr(((uint32_t)&vADMA3IntDescriptor[0].DES0),1);

    /*Wait for transfer complete interrupt*/
    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       Perform a transfers using Command queuing engine(CQE) for eMMC device.
 *
 * @details     This API performs CQE based transfers. CQE based transfers are tasks based transfers.
 *              User submits tasks using pCqeDes structure array. Tasks submitted are converted into
 *              Task Descriptor List (TDL from Slot #0 to Slot #30).Tasks are submitted for execution
 *              by ringing Command Queuing DoorBell register. Task complete interrupt will get raised
 *              after all tasks are completed (interrupt coalescing (INT=0)). For more information on command queuing refer to HRM.
 *              Before calling this API, DMA type should be set to ADMA2 using adi_emsi_SetDmaType() API.
 *              Also for each task maximum buffer size is valid up to 64 kB (for block size of 512 bytes) only.
 *              Maximum 31 tasks can be scheduled while interrupt coalescing is set. DCMD(Slot #31) option is not available
 *              while interrupt coalescing is used.
 *
 *              *Note: While submitting descriptors please ensure that any buffer which crossing 128 MB boundary is taken care
 *              while creating descriptors. Please refer HRM more information.
 *
 *              *Note: Booting operations and open ended transfers(Block Count =0) are not possible with CQE.
 *
 * @param [in]  hInfo               Handle to the EMSI device to be configured.
 *
 * @param [in]  pCqeDes             Pointer to Task descriptors list. Task descriptors are created in application
 *                                  only pointer is submitted here.\n
 *                                  (Some basic task parameters field which required in creating task parameters:
 *                                  1)Block Count: The number of blocks to read or write.\n
 *                                  2)Start address of the block of the card (for >2GB cards)/
 *                                    Byte address of the card (for <2GB cards) where data to be read or written.\n
 *                                  3)Data transfer direction:\n
 *                                    Write transfers (from host to card)\n
 *                                    Read transfers (from card to host)\n
 *                                  4)Pointer to  buffer to write or read the data to and from the card, respectively.
 *                                  5)Task priority: Higher priority tasks are executed  first compared to lower priority tasks
 *                                    when multiple tasks are set for execution.\n
 *                                    Two types of task priorities\n
 *                                    Lower priority task\n
 *                                    Higher priority task
 *                                  Above parameters are basic parameters which are required for each task.
 *                                  While creating descriptor user can use other parameters also such as QBR,
 *                                  Tag Request etc.)
 *
 *
 * @param [in]  MaxTasksScheduled   Number of task scheduled (Maximum 31 tasks)
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished.
 *
 *    - #ADI_EMSI_FAILURE           API call was unsuccessful.
 *
 * @note        This API should not be called when transfer is going on.\n
 *              Only 512 bytes per block are supported in CQE.
 **/
ADI_EMSI_RESULT adi_emsi_CmdQueTransfers(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_CQE_TASK_DES *pCqeDes,
                uint8_t MaxTasksScheduled)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

    uint8_t DataDirection;
    uint32_t No_Of_Tasks=0u;

    pInfo->pDevice->pCQETaskDescriptor = pCqeDes;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Check Command queuing feature supported by card and calculating supported queue depth*/
    if(ADI_EMSI_SUCCESS != GetExtCsdcontent(pInfo))
    {
        return ADI_EMSI_FAILURE;
    }

    if((MaxTasksScheduled>pInfo->pDevice->nCmdQueDepth)||(MaxTasksScheduled>MAXIMUM_SUPPORTED_CQE_TASKS))
    {
        /*Number of task submitted are more than queue depth supported*/
        return ADI_EMSI_QUEUE_DEPTH_INSUFFICIENT;
    }

    /*Reset data lines*/
    pInfo->pEMSIRegs->SWRST &= ~BITM_EMSI_SWRST_DAT;
    pInfo->pEMSIRegs->SWRST |= BITM_EMSI_SWRST_DAT;
    while(pInfo->pEMSIRegs->SWRST & BITM_EMSI_SWRST_DAT)
    {
        /*Wait for Data line to reset*/
        ;
    }

    /*Configuring the Block size for CQ transfer, CQ transfer has default block size of 512 Bytes always*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SetBlkSze(pInfo,CQ_TRANSFER_BLOCKSIZE))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Enable Command queuing feature from card side*/
    if(ADI_EMSI_SUCCESS != SetCmdQue(pInfo,true))
    {
        return ADI_EMSI_FAILURE;
    }


    /*Updating data progress as busy*/
    pInfo->pDevice->bData_Trans = true;

    /*Setting Common Parameters for Transfer Mode register*/
    pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_DMA_EN|
                                      BITM_EMSI_TRNSFRMODE_RESP_TYPE|
                                      BITM_EMSI_TRNSFRMODE_RESP_ERR_CHK_EN|
                                      BITM_EMSI_TRNSFRMODE_RESP_INT_DIS|
                                      BITM_EMSI_TRNSFRMODE_BLOCK_COUNT_EN|
                                      BITM_EMSI_TRNSFRMODE_MULTI_BLK_SEL);

    pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_DMA_ENABLED|
                                    ENUM_EMSI_TRNSFRMODE_RESP_R1|
                                    ENUM_EMSI_TRNSFRMODE_RESP_ERR_CHK_DISABLED|
                                    ENUM_EMSI_TRNSFRMODE_RESP_INT_ENABLED|
                                    ENUM_EMSI_TRNSFRMODE_BLOCK_COUNT_ENABLED|
                                    ENUM_EMSI_TRNSFRMODE_MULTI;


    /*Ensuring block count is set to zero for CQ*/
    pInfo->pEMSIRegs->BLKCNT = TWO_BYTE_ZERO;
    pInfo->pDevice->nBlkCnt = TWO_BYTE_ZERO;
    /*32-bit block count gets enabled once 16-bit block count is zero
    Configuring 32-bit block count to zero for open ended transfers*/
    pInfo->pEMSIRegs->SDMA_ADDR = FOUR_BYTE_ZERO;

    /*Interrupt Coalescing enabled as we are not using interrupt in descriptors
     * So TCC will get raised after all tasks (task threshold set in
     * CQ_IC register is reached) are completed*/

    pInfo->pEMSIRegs->CQ_IC = (uint32_t)(BITM_EMSI_CQ_IC_INTC_EN|BITM_EMSI_CQ_IC_INTC_TH_WEN|
                                        ((uint32_t)MaxTasksScheduled<<BITP_EMSI_CQ_IC_INTC_TH));

    /*Disabling  the interrupts and interrupt status as per requirement of CQE*/
    pInfo->pEMSIRegs->ISTAT_EN &= ~(BITM_EMSI_ISTAT_EN_CMD_COMPLETE|BITM_EMSI_ISTAT_EN_XFER_COMPLETE);
    pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_CMD_COMPLETE|BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

    /*Enabling the required interrupts*/
    SetCqInterruptMask(pInfo,ENABLE_CQ_INT_STAT,ENABLE_CQ_INT,true);

    /*Configuring Send status register for CQ*/
    pInfo->pEMSIRegs->CQ_SSCFG1 &= ~(BITM_EMSI_CQ_SSCFG1_SQSCMD_IDLE_TMR|BITM_EMSI_CQ_SSCFG1_SQSCMD_BLK_CNT);
    pInfo->pEMSIRegs->CQ_SSCFG1 |= (((uint32_t)CQE_SQSCMD_IDLE_TMR)<<BITP_EMSI_CQ_SSCFG1_SQSCMD_IDLE_TMR)|
                                    (((uint32_t)CQE_SQSCMD_LAST_BLK<<BITP_EMSI_CQ_SSCFG1_SQSCMD_BLK_CNT));

    /*RCA for CMD13 to be sent to retrieve status*/
    pInfo->pEMSIRegs->CQ_SSCFG2 &= ~BITM_EMSI_CQ_SSCFG2_SQSCMD_RCA;
    pInfo->pEMSIRegs->CQ_SSCFG2 |= (pInfo->pDevice->nRca & BITM_EMSI_CQ_SSCFG2_SQSCMD_RCA);

    /* Write the start of the task descriptor list to the Task descriptor list base address register*/
    pInfo->pEMSIRegs->CQ_TDL_BADDR = (uint32_t)adi_rtl_internal_to_system_addr(((uint32_t)&pCqeDes[0]),1);

    /*CQ configuration register  with 64 bits of task descriptor size and 31st slot used for transfer descriptor*/
    pInfo->pEMSIRegs->CQ_CFG &= ~(BITM_EMSI_CQ_CFG_TASK_DESC_SIZE|BITM_EMSI_CQ_CFG_DCMD_EN|BITM_EMSI_CQ_CFG_CQ_EN);
    pInfo->pEMSIRegs->CQ_CFG |= (ENUM_EMSI_CQ_CFG_TASK_DESC_64B|ENUM_EMSI_CQ_CFG_SLOT31_DCMD_DISABLE|ENUM_EMSI_CQ_CFG_CQE_ENABLE);


    for(No_Of_Tasks=0u;No_Of_Tasks<MaxTasksScheduled;No_Of_Tasks++)
    {

        DataDirection = (uint8_t)(((pInfo->pDevice->pCQETaskDescriptor[No_Of_Tasks].DES0)>>BITP_DATA_TRANSFER_DIR)
                                    &BITM_DATA_TRANSFER_DIR_MASK);

        if (DataDirection == (uint8_t)ADI_EMSI_DATA_XFER_DIR_WRITE)
        {
            /*Write Multiblock predefined*/
            pInfo->pEMSIRegs->TRNSFRMODE &= ~BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR;
            pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_WRITE;

        }
        else
        {
            /*Read Multiblock predefined*/
            pInfo->pEMSIRegs->TRNSFRMODE &= ~BITM_EMSI_TRNSFRMODE_DATA_XFER_DIR;
            pInfo->pEMSIRegs->TRNSFRMODE |= ENUM_EMSI_TRNSFRMODE_READ;
        }

        /*Submitting task for execution*/
        pInfo->pEMSIRegs->CQ_TDBR = (uint32_t)(1u<<No_Of_Tasks);
    }

    /*Wait For all task completion in application*/
    return ADI_EMSI_SUCCESS;

}

/**
 * @brief       Configuring Wakeup Control Register .
 *
 * @details     User will use this API to configure the wakeup control register. For more information on wakeup control
 *              please refer HRM. The Card removal interrupt, card insertion interrupt should be used
 *              either through Wake up control register or ISTAT_INTEN register (They should not set in both places).
 *              If the SD card is removed and inserted back then EMSI bus clock should be supplied again.
 *
 * @param [in]  hInfo               Handle to the EMSI device to be configured.
 *
 * @param [in]  bCardInsert         Wakeup Event Enable on SD Card Insertion
 *                                  true  - enable
 *                                  false - Disable
 *
 * @param [in]  bCardRemoval        Wakeup Event Enable on SD Card Removal
 *                                  true  - enable
 *                                  false - Disable
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished.
 **/
ADI_EMSI_RESULT adi_emsi_SetWakeUpCntrl(
                ADI_EMSI_HANDLE const hInfo,
                bool bCardInsert,
                bool bCardRemoval)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    if(bCardInsert ==  true)
    {
        /*Disabling the card insertion interrupt*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~((BITM_EMSI_ISTAT_EN_CARD_INSERTION));

        /*enable wakeup event on card insertion*/
        pInfo->pEMSIRegs->WU_CTL  &= ~BITM_EMSI_WU_CTL_CARD_INSERT;
        pInfo->pEMSIRegs->WU_CTL  |= ENUM_EMSI_WU_CTL_CARD_INSERT_ENABLED;
    }
    else
    {
        /*Disable wakeup event on card insertion*/
        pInfo->pEMSIRegs->WU_CTL  &= ~BITM_EMSI_WU_CTL_CARD_INSERT;

        /*Enabling the card insertion interrupt to notify the application*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~((BITM_EMSI_ISTAT_EN_CARD_INSERTION));
        pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_EN_CARD_INSERTION);
    }

    if(bCardRemoval ==  true)
    {
        /*Disabling the Card removal interrupt*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~((BITM_EMSI_ISTAT_EN_CARD_REMOVAL));

        /*enable wakeup event on card removal*/
        pInfo->pEMSIRegs->WU_CTL  &= ~BITM_EMSI_WU_CTL_CARD_REMOVAL;
        pInfo->pEMSIRegs->WU_CTL  |= ENUM_EMSI_WU_CTL_CARD_REMOVAL_ENABLED;
    }
    else
    {
        /*Disable wakeup event on card removal*/
        pInfo->pEMSIRegs->WU_CTL  &= ~BITM_EMSI_WU_CTL_CARD_REMOVAL;

        /*Enabling the Card removal interrupt to notify the application*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~((BITM_EMSI_ISTAT_EN_CARD_REMOVAL));
        pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_EN_CARD_REMOVAL);
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       Data CRC status check disable for bus testing procedure.
 *
 * @details     For eMMC device: In bus testing procedure eMMC device doesn’t
 *              send back CRC status(Positive CRC/Negative CRC) for CMD 19 (write block)
 *              this may lead to data CRC error, to suppress this error DISABLE_DATA_CRC_CHK
 *              bit is enabled. Using this API user can set this bit.
 *              As per JEDEC specification, the bus testing procedure is valid in SDR mode only.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  bEnable         Enable or Disable Data CRC status check.
 *                              -true - Disable data CRC status check
 *                              -false - Enable data CRC status check
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_DisableDataCrcCheck(
                ADI_EMSI_HANDLE const hInfo,
                bool bEnable)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    if(bEnable == true)
    {
        /*Disable data CRC checksum check*/
        pInfo->pEMSIRegs->EMMC_CTL &= ~BITM_EMSI_EMMC_CTL_DISABLE_DATA_CRC_CHK;
        pInfo->pEMSIRegs->EMMC_CTL |= ENUM_EMSI_EMMC_CTL_DISABLE_DATA_CRC_CHK;
    }
    else
    {
        /*Enable data CRC checksum check*/
        pInfo->pEMSIRegs->EMMC_CTL &= ~BITM_EMSI_EMMC_CTL_DISABLE_DATA_CRC_CHK;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       Using preset register for configuring clock divider values.
 *
 * @details     The preset registers can be used for configuring clock divider values instead of
 *              passing divider values into adi_emsi_ChangeClockFreq() API. Controller reads
 *              particular preset register as per bus speed mode selected using adi_emsi_SetSpeedMode ()
 *              API and divider values will be set according to content of particular preset register.
 *              E.g. if bus mode is selected as HSDDR then corresponding PRESET value register will
 *              be PRESET_HSDDR and divider values will be 0 (default value). For more information
 *              refer to register section of HRM.
 *
 *              By default preset register usage is disabled in adi_emsi_open() API.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  bEnable         Enable or Disable preset register use.
 *                              -true - Enable preset register use.
 *                              -false - Disable preset register use.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished.
 *
 * @note        This API should not be called when transfer is going on.
 *              This API should be called after adi_emsi_SetSpeedMode() API.
 **/
ADI_EMSI_RESULT adi_emsi_SetPresetRegUse(
                ADI_EMSI_HANDLE const hInfo,
                bool bEnable)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    if(bEnable == true)
    {
        /*Enable preset register use*/
        pInfo->pEMSIRegs->CTL2 &= ~BITM_EMSI_CTL2_PRESET_VAL_EN;
        pInfo->pEMSIRegs->CTL2 |= BITM_EMSI_CTL2_PRESET_VAL_EN;
    }
    else
    {
        /*Disable preset register use*/
        pInfo->pEMSIRegs->CTL2 &= ~BITM_EMSI_CTL2_PRESET_VAL_EN;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       Enable or Disable response check by controller.
 *
 * @details     This API enable or disable response check by controller. The eMSI Controller
 *              supports response check function to avoid overhead of response error check by
 *              the driver. Response types of only R1 can be checked by the
 *              Controller. While enabling this user should ensure that command which are send
 *              after enabling the feature should have R1 response.
 *              Error which are checked by controller in R1 response are mentioned in HRM
 *              (Register section EMSI_TRANSFERMODE.RESP_TYPE).
 *
 *              Usually this API is called before starting data transfer as most of the data
 *              transfer commands have R1 response. Also it will reduce overhead of response check
 *              by controller and improves performance further.
 *
 *              By default response check by controller is disabled in adi_emsi_open() API.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  bEnable         Enable or Disable response check by controller.
 *                              -true - Enable response check by controller.
 *                              -false - Disable response check by controller.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 *    - #ADI_EMSI_BUSY              Data Transfer is not finished.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_ControllerRespChek(
                ADI_EMSI_HANDLE const hInfo,
                bool bEnable)
{
    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    if(bEnable == true)
    {
        /*Enable response check by controller*/
        pInfo->pDevice->bControllerRespChk = true;
    }
    else
    {
        /*Disable response check by controller*/
        pInfo->pDevice->bControllerRespChk = false;
        pInfo->pEMSIRegs->TRNSFRMODE &= ~(BITM_EMSI_TRNSFRMODE_RESP_TYPE|
                                          BITM_EMSI_TRNSFRMODE_RESP_ERR_CHK_EN|
                                          BITM_EMSI_TRNSFRMODE_RESP_INT_DIS);

    }

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       To add tuning support for correct sampling of data.
 *
 * @details     This API is used to add tuning support for correct sampling of data(Due to
 *              silicon anomaly). Tuning should be added for SDR mode transfers.
 *              and should be disabled for DDR mode transfers. This API is valid for eMMC devices only.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  bTuning         Enable or Disable tuning\n
 *                              -true       Enable tuning (While SDR transfers)
 *                              -false      Disable tuning (While DDR transfers)
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_SetTuning(
                ADI_EMSI_HANDLE const hInfo,
                bool bTuning)
{

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

    /*Setting structure parameters*/
    ADI_PADS_PCFG1_PARAMS Config;
    Config.EMSI_TUNING_MUX_SEL = EMSI_TUNING_MUX_SEL4;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    if (bTuning == true)
    {
        /*Tuning support enable*/
        adi_pads_Config(EMSI_TUNING_EN, true);
        adi_pads_Config1(&Config);
    }
    else
    {
        /*Tuning support disable*/
        adi_pads_Config(EMSI_TUNING_EN, false);
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       To change eMMC QoS priority.
 *
 * @details     This API is used to change eMSI QoS priority so eMSI peripheral gets sufficient bandwidth
 *              so that FIFO is never full, and external eMSI clock is never gated.
 *              This can prevent end bit error which may occur due to clock gating in between the data transfer.
 *              Applicable for SDR mode and DDR mode transfers.
 *
 *              Please refer to anomlay number 20000119 from ADSP-SC595/SC596/SC598 silicon anomalies document.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  bChangeQosPrio  Change QoS priority: \n
 *                              -true       Change eMMC QoS priority to 12
 *                              -false      Keep eMMC QoS priority to 7 (default)
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_SetPriority(
                ADI_EMSI_HANDLE const hInfo,
                bool bChangeQosPrio)
{

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    if (bChangeQosPrio == true)
    {
        /*Change eMMC QoS priority to 12*/
        *pREG_SCB0_SDIO0_IB_WRITE_QOS=0xCu;

    }
    else
    {
        /*Keep eMMC QoS priority to 7 (default)*/
        *pREG_SCB0_SDIO0_IB_WRITE_QOS=0x7u;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       To reset eMMC device using reset signal.
 *
 * @details     This API is used to reset eMMC device using reset_n signal. Same should be enabled in extended csd register
 *              before using it from controller side.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  bEnableRst      Enbale/Disable reset signal: \n
 *                              -true       Enable reset signal(active low) or reset eMMC device.
 *                              -false      Disable reset signal.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_ResetEmmc(
                ADI_EMSI_HANDLE const hInfo,
                bool bEnableRst)
{

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /* Disable Interrupts */
    adi_osal_EnterCriticalRegion();

    if (bEnableRst == true)
    {
        /*Reset to eMMC device asserted (active low)*/
        pInfo->pEMSIRegs->EMMC_CTL &= ~BITM_EMSI_EMMC_CTL_EMMC_RST_N;

    }
    else
    {
        /*Reset to eMMC device is de-asserted*/
        pInfo->pEMSIRegs->EMMC_CTL |= ENUM_EMSI_EMMC_CTL_RST_DEASSERT;
    }

    /* Re-enable interrupts */
    adi_osal_ExitCriticalRegion();

    return ADI_EMSI_SUCCESS;
}

/**
 * @brief       To control CQE from application.
 *
 * @details     This API can be used in two scenarios: \n
 *              1) when user wants to close CQE engine before all scheduled tasks are completed.
 *              User can halt the CQE engine (using bEnableHalt) and
 *              issue a Discard all Task command (CMDQ_TASK_MGMT) CMD48 in application and disable the
 *              CQE engine (using bEnableCqe)(this will clear all tasks from controller side).
 *              2) When RED (response error detected) occurs the CQE will be halted in status handler
 *              user can check CQ_TERRINFO register to check for which task error has occurred.
 *              That task can be discarded using CMD48 then clear that task from controller side using
 *              (nTaskClear) then update the Interrupt Coalescing Counter Threshold then unmask bits in Command
 *              response mode error mask register (using bCqrmem) so that if RED occurs again then it will be reported.
 *              Finally resume the CQE. So remining tasks will be completed
 *
 *              Note: Use this API only when user wants to close the CQE engine or for response
 *              error handling during CQE operation, before all scheduled tasks are completed.
 *              Otherwise status handler will take care of all actions when all scheduled tasks are completed.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  bEnableHalt     CQE halt/Resume: \n
 *                              -true       Halt the CQE.
 *                              -false      Resume the CQE.
 *
 * @param [in]  nTaskClear      Clear required tasks from controller side.
 *                              - Providing "0" Clear all taks from controller side.
 *                              - whereas providing a specific task number will clear that task
 *                              (We can't clear multiple tasks at a time using this option) only from
 *                              controller side user has to send CMD 48 separately after CQE halt to
 *                              discarded task from eMMC device side. Each task has to be discarded one by one
 *                              if non zero option is used.
 *
 * @param [in]  bEnableCqe      CQE Enable/Disable: \n
 *                              -true       Enable the CQE.
 *                              -false      Disable the CQE.
 *
 * @param [in]  bCqrmem         Response error detect mask: \n
 *                              -true       Unmask all device status bits so RED interrupt will take effect .
 *                              -false      Mask all device status bits so RED won't occur.
 *
 * @param [in]  nUpdateIntcTh  Update the Interrupt Coalescing Counter Threshold in case we discrded any task due to error
 *                             or any other reason. So that when we resume operation
 *                             after discarding the task, when remaining tasks are completed TCC (Task complete interrupt)
 *                             will be raised.
 *                             providing "0" will keep threshold as set previously in adi_emsi_CmdQueTransfers().
 *                             Maximum value is 31 for Interrupt Coalescing Counter Threshold. Please refer HRM for more information.
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS           API call was successful.
 *
 **/
ADI_EMSI_RESULT adi_emsi_CqeCntrl(
                ADI_EMSI_HANDLE const hInfo,
                bool bEnableHalt,
                uint32_t nTaskClear,
                bool bEnableCqe,
                bool bCqrmem,
                uint8_t nUpdateIntcTh)
{

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    if (bEnableHalt == true)
    {
        /*Halting the CQE to gain access of bus*/
        pInfo->pEMSIRegs->CQ_CTL |= BITM_EMSI_CQ_CTL_HALT;

        /*wait till eMMC bus control is released by CQE*/
        while (!(pInfo->pEMSIRegs->CQ_CTL & BITM_EMSI_CQ_CTL_HALT))
        {
            ;
            /*wait till eMMC bus control is released by CQE**/
        }
        /*Clearing the Halt status*/
        pInfo->pEMSIRegs->CQ_ISTAT = BITM_EMSI_CQ_ISTAT_HAC;


        /*Dummy write for next writes to go through*/
        pInfo->pEMSIRegs->ARG &= BITM_EMSI_ARG_VALUE;

        /*Enabling the interrupt status for next transfers So application can send command from application*/
        pInfo->pEMSIRegs->ISTAT_EN |= (BITM_EMSI_ISTAT_EN_CMD_COMPLETE|BITM_EMSI_ISTAT_EN_XFER_COMPLETE);
        /*Enabling the interrupts if next transfers are not based on CQE*/
        pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

        /*Halting transfers hence clearing data transfer flag*/
        pInfo->pDevice->bData_Trans = false;
    }
    else
    {
        /*Clearing the task clear notification register for all completed task*/
        pInfo->pEMSIRegs->CQ_TCN = CQE_TCN_REG_CLEAR;

        /*Disabling  the interrupts and interrupt status as per requirement of CQE*/
        pInfo->pEMSIRegs->ISTAT_EN &= ~(BITM_EMSI_ISTAT_EN_CMD_COMPLETE|BITM_EMSI_ISTAT_EN_XFER_COMPLETE);
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_CMD_COMPLETE|BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

        /*Data transfer flag set as operation resumed*/
        pInfo->pDevice->bData_Trans = true;

        /*Resume CQE*/
        pInfo->pEMSIRegs->CQ_CTL &= ~(BITM_EMSI_CQ_CTL_HALT);
    }

    if(nTaskClear)
    {
        /*Clear task in CQE*/
        pInfo->pEMSIRegs->CQ_TCLR = nTaskClear;

        /*wait till all task queue cleared by CQE*/
        while (!(pInfo->pEMSIRegs->CQ_ISTAT & BITM_EMSI_CQ_ISTAT_TCL))
        {
            ;
            /*wait till all task getting cleared*/
        }
        /*Clearing the task clear interrupt status*/
        pInfo->pEMSIRegs->CQ_ISTAT = BITM_EMSI_CQ_ISTAT_TCL;
    }

    if (bEnableCqe == true)
    {
        /*CQ configuration register Enabling the CQE */
        pInfo->pEMSIRegs->CQ_CFG |= (BITM_EMSI_CQ_CFG_CQ_EN);

    }
    else
    {
        /*Clear all task in CQE*/
        pInfo->pEMSIRegs->CQ_CTL |= BITM_EMSI_CQ_CTL_CLR_ALL_TASKS;

        /*wait till all task queue cleared by CQE*/
        while (!(pInfo->pEMSIRegs->CQ_ISTAT & BITM_EMSI_CQ_ISTAT_TCL))
        {
            ;
            /*wait till all task getting cleared*/
        }
        /*Clearing the task clear interrupt status*/
        pInfo->pEMSIRegs->CQ_ISTAT = BITM_EMSI_CQ_ISTAT_TCL;

        /*Clearing the task clear notification register for all completed task*/
        pInfo->pEMSIRegs->CQ_TCN = CQE_TCN_REG_CLEAR;

        /*CQ configuration register disabling the CQE */
        pInfo->pEMSIRegs->CQ_CFG &= ~(BITM_EMSI_CQ_CFG_CQ_EN);

        /*Closing CQE by application request so clearing data transfer flag*/
        pInfo->pDevice->bData_Trans = false;

        /*Resetting the CQ_IC counter as all task are completed*/
        pInfo->pEMSIRegs->CQ_IC |= (BITM_EMSI_CQ_IC_INTC_RST);
        /*Resetting the bit for future use (Write only bit)*/
        pInfo->pEMSIRegs->CQ_IC &= ~(BITM_EMSI_CQ_IC_INTC_RST);

        /*Clearing the threshold value for future use and Disabling CQ_IC*/
        pInfo->pEMSIRegs->CQ_IC = (uint32_t)((BITM_EMSI_CQ_IC_INTC_TH_WEN|((uint32_t)(CLR_CQ_IC_THRESHOLD)<<BITP_EMSI_CQ_IC_INTC_TH))
                                             |((uint32_t)((uint32_t)0u << BITP_EMSI_CQ_IC_INTC_EN)));

        /*Clearing the bit for future use(Write only bit)*/
        pInfo->pEMSIRegs->CQ_IC &= ~(BITM_EMSI_CQ_IC_INTC_TH_WEN);

        /*Clearing for future use*/
        pInfo->pEMSIRegs->CQ_SSCFG2 &= ~BITM_EMSI_CQ_SSCFG2_SQSCMD_RCA;

        /*Clearing up all interrupts of CQE*/
        SetCqInterruptMask(pInfo,ENABLE_CQ_INT_STAT,ENABLE_CQ_INT,false);

        /*Clearing up Task Descriptor List Base Address register*/
        pInfo->pEMSIRegs->CQ_TDL_BADDR &=  ~ BITM_EMSI_CQ_TDL_BADDR_TDLBA;

        /*Enabling the interrupt status for next transfers*/
        pInfo->pEMSIRegs->ISTAT_EN |= (BITM_EMSI_ISTAT_EN_CMD_COMPLETE|BITM_EMSI_ISTAT_EN_XFER_COMPLETE);
        /*Enabling the interrupts if next transfers are not based on CQE*/
        pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

        /*Clearing the bit for future use, clearing after CQE disable*/
        pInfo->pEMSIRegs->CQ_CTL &= ~(BITM_EMSI_CQ_CTL_HALT);
    }



    if (bCqrmem == true)
    {
        /*When RED (response error detect) interrupt was occurred for clearing the interrupt
         * we masked all interrupts.
         * Now, If we want to resume operation after RED occurred then unmask response error fields so
         * that when next response error occurs RED will be raised again.*/
        pInfo->pEMSIRegs->CQ_RMEM = RESP_ERR_DETECT_MASK;
    }
    else
    {
        /*Mask all fields so RED won't occur*/
        pInfo->pEMSIRegs->CQ_RMEM = 0u;
    }


    if(nUpdateIntcTh)
    {
        /*if discard the task then Updating Interrupt Coalescing Counter Threshold is required
         * so that for when reduced number of tasks (Total tasks - discarded tasks) will get
         * completed then TCC will get raised.
         * For e.g. We scheduled 31 tasks and due to error or some other reason we discarded 2 tasks then
         * we can update threshold to 29 so that when remaining tasks re completed interrupt TCC will get raised
         * This field should be updated when CQE is at halt*/

        pInfo->pEMSIRegs->CQ_IC = (uint32_t)(BITM_EMSI_CQ_IC_INTC_EN|BITM_EMSI_CQ_IC_INTC_TH_WEN|
                                            ((uint32_t)nUpdateIntcTh<<BITP_EMSI_CQ_IC_INTC_TH));

    }

    return ADI_EMSI_SUCCESS;
}


/**
 * @brief       To test functional pins on the bus.
 *
 * @details     This API is used to do bus test. The data pattern sent by the eMSI controller and with the reversed pattern
 *              sent back by the eMMC Device, the functional pins on the bus can be detected. While using this API please ensure
 *              that card is in transfer state. This API is valid for eMMC only.
 *
 * @param [in]  hInfo           Handle to the EMSI device to be configured.
 *
 * @param [in]  eWidthBusTest   The bus width to be used for bus test:\n
 *                              -eMMC supported bus widths:\n
 *                              #ADI_EMSI_BUS_WIDTH_1BIT = Use when DAT 0 connected to eMMC device,\n
 *                              #ADI_EMSI_BUS_WIDTH_4BIT = Use when DAT 0-3 connected to eMMC device,\n
 *                              #ADI_EMSI_BUS_WIDTH_8BIT = Use when DAT 0-7 connected to eMMC device.\n
 *
 * @return      Status
 *
 *    - #ADI_EMSI_SUCCESS                   Bus test was successful.
 *    - #ADI_EMSI_INVALID_CONFIGURATION     Invalid bus width.
 *    - #ADI_EMSI_FAILURE                   Bus test was unsuccessful.
 *
 * @note        This API should not be called when transfer is going on.
 **/
ADI_EMSI_RESULT adi_emsi_BusTest(
                ADI_EMSI_HANDLE const hInfo,
                ADI_EMSI_BUS_WIDTH eWidthBusTest)
{

    /* Pointer to EMSI device instance */
    ADI_EMSI_INFO  *pInfo = (ADI_EMSI_INFO *)hInfo;

    /*Bus Test Tx Buffer*/
    ADI_CACHE_ALIGN static uint8_t Bus_Test_TxBuff[ADI_CACHE_ROUND_UP_SIZE((8u), uint8_t)];
    /*Bus Test Rx Buffer*/
    ADI_CACHE_ALIGN static uint8_t Bus_Test_RxBuff[ADI_CACHE_ROUND_UP_SIZE((8u), uint8_t)];

    /*Temporary Variables*/
    uint32_t nTemp=0u;
    uint32_t nTemp1=0u;

    /*Parameters for CMD13*/
    ADI_EMSI_CMD_PARA CMD_PARA13 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                    ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                    ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*Parameters for CMD19*/
    ADI_EMSI_CMD_PARA CMD_PARA19 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                    ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                    ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*Parameters for CMD14*/
    ADI_EMSI_CMD_PARA CMD_PARA14 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                    ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                    ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

#if defined (ADI_DEBUG)
    /* Validate the given handle */
    assert(ValidateHandle(pInfo) == ADI_EMSI_SUCCESS);
#endif

    /*Return if data progress is on*/
    if(pInfo->pDevice->bData_Trans == true)
    {
        return ADI_EMSI_BUSY;
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    /*Clearing the response storing space*/
    InternalResp[0]=0u;
    /*Sending CMD13 for Checking card status*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SEND_STATUS,(uint32_t)((uint32_t)pInfo->pDevice->nRca<<16u),
                                                 &CMD_PARA13))
    {
        /*CMD13 Failure*/
        return ADI_EMSI_FAILURE;
    }

    adi_emsi_GetResponse(pInfo,&InternalResp[0]);
    /*Cards current state in R1 response*/
    nTemp=InternalResp[0]>>CARD_CURRENT_STATE;
    nTemp1=nTemp & CARD_CURRENT_STATE_MASK;

    if (nTemp1 != CARD_IN_TRANSFER_STATE)
    {
        /*Card is not in transfer state hence bus test can't be done*/
        return ADI_EMSI_FAILURE;
    }

    /*Initialize buffer with bus test patterns*/
    switch(eWidthBusTest)
    {
    case ADI_EMSI_BUS_WIDTH_1BIT:
    {
        Bus_Test_TxBuff[0] = 0x80u;
        break;
    }
    case ADI_EMSI_BUS_WIDTH_4BIT:
    {
        Bus_Test_TxBuff[0] = 0x5au;
        break;
    }
    case ADI_EMSI_BUS_WIDTH_8BIT:
    {
        Bus_Test_TxBuff[0] = 0x55u;
        Bus_Test_TxBuff[1] = 0xaau;
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

    /*Setting app usage to Booting operation so that all next configuration will be done from controller side
     * This is nothing to do with booting operation just an way to set all configuration form controller side*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SetAppUse(pInfo,ADI_EMSI_APP_USE_BOOTING_OPERATION,false))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Selecting the Speed Mode from controller side*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SetSpeedMode(pInfo,ADI_EMSI_SPEED_MODE_HIGHSPEED_SDR))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Set the bus width for bus test from controller side.*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SetBusWidth(pInfo,eWidthBusTest))
    {
        return ADI_EMSI_FAILURE;
    }

    /* Set the block size for upcoming bus test.*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SetBlkSze(pInfo,BUS_TEST_TRANSFER_BLOCKSIZE))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Setting app usage to Normal data transfer so that all next data transfers happen correctly*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SetAppUse(pInfo,ADI_EMSI_APP_USE_NORMAL_DATATRANSFER,false))
    {
        return ADI_EMSI_FAILURE;
    }

    /* Sets DMA type for an upcoming eMSI transfer(s) ignoring buffer boundary parameter for ADMA2 based transfers.*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SetDmaType(pInfo,ADI_EMSI_DMA_USED_ADMA2,ADI_EMSI_SDMA_BUF_BDARY_4K))
    {
        return ADI_EMSI_FAILURE;
    }

    /*AutoCMD Feature use*/
    if (ADI_EMSI_SUCCESS != adi_emsi_AutoCmd(pInfo,ADI_EMSI_AUTO_CMD_DIS))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Disable crc status check*/
    if (ADI_EMSI_SUCCESS != adi_emsi_DisableDataCrcCheck(pInfo,true))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Disabling the transfer complete interrupt to core, polling it internally*/
    pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

    /*Sending BUSTEST_W*/


    /*Sending CMD19*/
    /*Using general transfers sending CMD19 block*/
    if (ADI_EMSI_SUCCESS != adi_emsi_General_Transfer(pInfo,ADI_EMSI_GENERAL_TRANS_TYPE_WRITE,NO_OF_BLOCKS_BUS_TEST,&Bus_Test_TxBuff[0]))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Sending CMD19 (BUSTEST_W) command*/
    /*Sending CMD19 command*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,BUSTEST_W,0x1u/*Stuff bits*/,&CMD_PARA19))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Check for transfer complete*/
    if(ADI_EMSI_DATA_TRANS_FINISHED != CheckXfrStatus(pInfo))
    {
        return ADI_EMSI_FAILURE;
    }
    else
    {
        /*Updating data transfer flag to complete*/
        pInfo->pDevice->bData_Trans = false;

        /*Wait for 8 eMSI clock cycles till we send next command*/
        if(ADI_EMSI_SUCCESS != EmsiDelay(pInfo, EIGHT_CYCLES))
        {
            return ADI_EMSI_FAILURE;
        }

        /*Write command successful now read pattern from device*/
        /*Sending CMD14*/
        /*Using general transfers sending CMD14 block*/
        if (ADI_EMSI_SUCCESS != adi_emsi_General_Transfer(pInfo,ADI_EMSI_GENERAL_TRANS_TYPE_READ,NO_OF_BLOCKS_BUS_TEST,&Bus_Test_RxBuff[0]))
        {
            return ADI_EMSI_FAILURE;
        }

        /*Sending CMD14 command*/
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,BUSTEST_R,0x1u/*Stuff bits*/,&CMD_PARA14))
        {
            return ADI_EMSI_FAILURE;
        }

        /*Check for transfer complete*/
        if(ADI_EMSI_DATA_TRANS_FINISHED != CheckXfrStatus(pInfo))
        {
            return ADI_EMSI_FAILURE;
        }
        else
        {
            switch(eWidthBusTest)
            {
            case ADI_EMSI_BUS_WIDTH_1BIT:
            {
                if (Bus_Test_RxBuff[0] != 0x40u)
                {
                    return ADI_EMSI_FAILURE;
                }
                break;
            }
            case ADI_EMSI_BUS_WIDTH_4BIT:
            {
                if (Bus_Test_RxBuff[0] != 0xa5u)
                {
                    return ADI_EMSI_FAILURE;
                }
                break;
            }
            case ADI_EMSI_BUS_WIDTH_8BIT:
            {
                if ((Bus_Test_RxBuff[0] != 0xaau) && (Bus_Test_RxBuff[1] != 0x55u))
                {
                    return ADI_EMSI_FAILURE;
                }
                break;
            }
            default:
            {
                return ADI_EMSI_INVALID_CONFIGURATION;
            }
            }

            /*Updating data transfer flag to complete*/
            pInfo->pDevice->bData_Trans = false;

            /*Flag for send command and resp access API being used internally*/
            pInfo->pDevice->bInternal_Cmd = false;

            /*General transfer completed*/
            bGeneral_Transfer = false;

            /*Enabling the transfer complete interrupt to core for the next transfers*/
            pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);
            pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

            /*Enable crc status check*/
            if (ADI_EMSI_SUCCESS != adi_emsi_DisableDataCrcCheck(pInfo,false))
            {
                return ADI_EMSI_FAILURE;
            }
            return ADI_EMSI_SUCCESS;
        }

    }
}
/***********************Internal Functions******************************/

/*Reset the command and data lines*/
static void ResetCmdDataLine (ADI_EMSI_INFO *pInfo)
{

    /*Reset CMD line*/
    pInfo->pEMSIRegs->SWRST &= ~BITM_EMSI_SWRST_CMD;
    pInfo->pEMSIRegs->SWRST |= BITM_EMSI_SWRST_CMD;
    while(pInfo->pEMSIRegs->SWRST & BITM_EMSI_SWRST_CMD)
    {
        ;
        /*Wait for CMD line to reset*/
    }

    /*Reset data lines*/
    pInfo->pEMSIRegs->SWRST &= ~BITM_EMSI_SWRST_DAT;
    pInfo->pEMSIRegs->SWRST |= BITM_EMSI_SWRST_DAT;
    while(pInfo->pEMSIRegs->SWRST & BITM_EMSI_SWRST_DAT)
    {
        ;
        /*Wait for Data line to reset*/
    }

}


/* Reset the eMSI registers */
static void ResetEmsiRegs(ADI_EMSI_TypeDef *pEMSIRegs)
{
    /* Clear the eMSI registers */
    pEMSIRegs->TO_CTL = ONE_BYTE_ZERO;
    pEMSIRegs->CTL2 = TWO_BYTE_ZERO;
    pEMSIRegs->EMMC_CTL = EMMC_CTL_RESET;
    pEMSIRegs->ARG = FOUR_BYTE_ZERO;
    pEMSIRegs->TRNSFRMODE   = TWO_BYTE_ZERO;
    pEMSIRegs->CTL1 = ONE_BYTE_ZERO;
    pEMSIRegs->ADMA_ADDR_LO = FOUR_BYTE_ZERO;
    pEMSIRegs->BLKSZ = TWO_BYTE_ZERO;
    pEMSIRegs->BLKCNT = TWO_BYTE_ZERO;
    pEMSIRegs->SDMA_ADDR = FOUR_BYTE_ZERO;
    pEMSIRegs->SWRST = ONE_BYTE_ZERO;
    pEMSIRegs->BOOT_CTL =TWO_BYTE_ZERO;
    /*Don't Reset CMD register as it send command if cmd idx is set*/
}

/* Reset and Clear eMSI Interrupt registers to default */
static void ResetEmsiIntRegs(ADI_EMSI_TypeDef *pEMSIRegs)
{
    pEMSIRegs->ISTAT =  DEFAULT_INT_STAT_MASK_VALUE;
    pEMSIRegs->ERR_STAT =  DEFAULT_ERROR_INT_STAT_MASK_VALUE;
    /*Mask all interrupts*/
    pEMSIRegs->ISTAT_EN = TWO_BYTE_ZERO;
    pEMSIRegs->ERR_STAT_EN = TWO_BYTE_ZERO;
    pEMSIRegs->ISTAT_INTEN = TWO_BYTE_ZERO;
    pEMSIRegs->ERR_STAT_INTEN = TWO_BYTE_ZERO;

}



/* eMSI Status interrupt handler */
static void EmsiStatusHandler(uint32_t SID, void *pCBParam)
{

    ADI_EMSI_INFO     *pInfo = (ADI_EMSI_INFO  *)pCBParam;

    ADI_EMSI_DEVICE   *pDevice = pInfo->pDevice ;

    volatile uint32_t Event = (uint32_t)ADI_EMSI_NOEVENT;


    /* Obtain the value of the status register */
    volatile uint32_t IntStatusReg = pInfo->pEMSIRegs->ISTAT;
    volatile uint32_t ErrStatusReg = pInfo->pEMSIRegs->ERR_STAT;


    /*Interrupt Stat*/
    /*!< CMD_COMPLETE: Command Complete. */
    if(((IntStatusReg & BITM_EMSI_ISTAT_CMD_COMPLETE)
       & pInfo->pEMSIRegs->ISTAT_INTEN) == ENUM_EMSI_ISTAT_CMD_COMPLETE_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CMD_COMPLETE);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_CMD_COMPLETE;
    }

    /*!< XFER_COMPLETE: Command Execution is Completed. */
    if(((IntStatusReg & BITM_EMSI_ISTAT_XFER_COMPLETE)
       & pInfo->pEMSIRegs->ISTAT_INTEN ) == ENUM_EMSI_ISTAT_XFER_COMPLETE_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_XFER_COMPLETE);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_XFER_COMPLETE;

        /*General transfer completed*/
        bGeneral_Transfer = false;

        /*Disabling boot ack for future data transfers*/
        if (pInfo->pEMSIRegs->BOOT_CTL & BITM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE)
        {
            /*Clearing the bit*/
            pInfo->pEMSIRegs->BOOT_CTL &= ~BITM_EMSI_BOOT_CTL_BOOT_ACK_ENABLE;
        }

        /*Data Transfer complete so updating the flag to false*/
        pInfo->pDevice->bData_Trans = false;
    }

    /*!< DMA_INTERRUPT: DMA Interrupt is Generated. */
    if(((IntStatusReg & BITM_EMSI_ISTAT_DMA_INTERRUPT)
       & pInfo->pEMSIRegs->ISTAT_INTEN) == ENUM_EMSI_ISTAT_DMA_INTERRUPT_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_DMA_INTERRUPT);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_DMA_INTERRUPT;
    }

    /*!< CARD_INSERTION: Card Inserted. */
    if(((IntStatusReg & BITM_EMSI_ISTAT_CARD_INSERTION)
       & pInfo->pEMSIRegs->ISTAT_INTEN) == ENUM_EMSI_ISTAT_CARD_INSERTION_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CARD_INSERTION);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_CARD_INSERTION;
    }

    /*!< CARD_REMOVAL: Card Removed. */
    if(((IntStatusReg & BITM_EMSI_ISTAT_CARD_REMOVAL)
       & pInfo->pEMSIRegs->ISTAT_INTEN) == ENUM_EMSI_ISTAT_CARD_REMOVAL_TRUE)
    {
        Event   |=  (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CARD_REMOVAL);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_CARD_REMOVAL;

        /*Data Transfer stopped so updating the flag to false*/
        pInfo->pDevice->bData_Trans = false;

        /*General transfer stopped*/
        bGeneral_Transfer = false;
    }

    /*!< FX_EVENT: FX Event is Detected. */
    if(((IntStatusReg & BITM_EMSI_ISTAT_FX_EVENT)
       & pInfo->pEMSIRegs->ISTAT_INTEN) == ENUM_EMSI_ISTAT_FX_EVENT_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_FX_EVENT);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_FX_EVENT;
    }
    /*!< CQE_EVENT: Command Queuing Event is Detected. */
    if(((IntStatusReg & BITM_EMSI_ISTAT_CQE_EVENT)
       & pInfo->pEMSIRegs->ISTAT_INTEN) == ENUM_EMSI_ISTAT_CQE_EVENT_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CQE_EVENT);

        if (pInfo->pEMSIRegs->CQ_ISTAT & BITM_EMSI_CQ_ISTAT_RED)
        {
            /*Halting the CQE to gain access of bus*/
            pInfo->pEMSIRegs->CQ_CTL |= BITM_EMSI_CQ_CTL_HALT;

            /*wait till eMMC bus control is released by CQE*/
            while (!(pInfo->pEMSIRegs->CQ_CTL & BITM_EMSI_CQ_CTL_HALT))
            {
                ;
                /*wait till eMMC bus control is released by CQE**/
            }

            /*Mask all response error so that we can clear the error
             * we can unmask them again when task is discarded in eMMC device and cleared
             * from controller using adi_emsi_CqeCntrl() API*/
            pInfo->pEMSIRegs->CQ_RMEM = 0u;

            /*Error Occurred during tasks completion*/
            Event |= (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_RED);

            /*Clearing the task complete interrupt status*/
            pInfo->pEMSIRegs->CQ_ISTAT = BITM_EMSI_CQ_ISTAT_RED;

            /*Clearing the interrupt bit*/
            pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_CQE_EVENT;

            /*Dummy write for next writes to go through*/
            pInfo->pEMSIRegs->ARG &= BITM_EMSI_ARG_VALUE;

            /*If error occurred then clear the transfer flag*/
            pInfo->pDevice->bData_Trans = false;
        }
        else
        {
            /*All tasks  are completed successfully*/
            /*Disabling CQE from card end controller end*/
            if (ADI_EMSI_SUCCESS != CqeCleanup(pInfo))
            {
                /*Error during disabling the command queuing*/
                Event |= (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_CQE_CLEANUP_FAIL);
            }
        }
        /*Clearing the CQE interrupt inside cleanup*/
    }


    /* ERROR interrupt Status*/
    /*!< CMD_TOUT_ERR: Time Out. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_CMD_TOUT_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_CMD_TOUT_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CMD_TOUT_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_CMD_TOUT_ERR;
    }

    /*!< CMD_CRC_ERR: CRC Error Generated. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_CMD_CRC_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_CMD_CRC_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CMD_CRC_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_CMD_CRC_ERR;
    }

    /*!< CMD_END_BIT_ERR: End Bit Error Generated. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_CMD_END_BIT_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_CMD_END_BIT_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CMD_END_BIT_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_CMD_END_BIT_ERR;
    }

    /*!< CMD_IDX_ERR: Error. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_CMD_IDX_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_CMD_IDX_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CMD_IDX_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_CMD_IDX_ERR;
    }

    /*!< DATA_TOUT_ERR: Time Out. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_DATA_TOUT_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_DATA_TOUT_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_DATA_TOUT_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_DATA_TOUT_ERR;
    }

    /*!< DATA_CRC_ERR: Error. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_DATA_CRC_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_DATA_CRC_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_DATA_CRC_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_DATA_CRC_ERR;
    }

    /*!< DATA_END_BIT_ERR: Error. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_DATA_END_BIT_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_DATA_END_BIT_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_DATA_END_BIT_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_DATA_END_BIT_ERR;
    }

    /*!< AUTO_CMD_ERR: Error. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_AUTO_CMD_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_AUTO_CMD_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_AUTO_CMD_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_AUTO_CMD_ERR;
    }
    /*!< ADMA_ERR: Error. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_ADMA_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_ADMA_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_ADMA_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_ADMA_ERR;
    }

    /*!< RESP_ERR: Error. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_RESP_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_RESP_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_RESP_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_RESP_ERR;
    }

    /*!< BOOT_ACK_ERR: Error. */
    if(((ErrStatusReg & BITM_EMSI_ERR_STAT_BOOT_ACK_ERR)
       & pInfo->pEMSIRegs->ERR_STAT_INTEN) == ENUM_EMSI_ERR_STAT_BOOT_ACK_ERR_TRUE)
    {
        Event   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_BOOT_ACK_ERR);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ERR_STAT = BITM_EMSI_ERR_STAT_BOOT_ACK_ERR;
    }

    /*In case of error, clear the transfer going on flag so we can issue abort command (in case of data transfer unfinished)
     *or we can close eMSI instance gracefully. Error is be reported through application callback.*/
    if(ErrStatusReg != 0u)
    {
        pInfo->pDevice->bData_Trans = false;
    }

    /* If there is a callback registered notify application via callback */
    if(pDevice->pfCallback != NULL)
    {
        if ((pInfo->pEMSIRegs->CMD & ENUM_EMSI_CMD_RESP_LEN_48B) !=
            (uint16_t)ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY)
        {
            /* Notify the event to application */
            pDevice->pfCallback(pDevice->pCBParam,Event,NULL);

        }

    }

}

/* EMSI Wakeup interrupt handler */
static void EmsiWakeupHandler(uint32_t SID, void *pCBParam)
{
    ADI_EMSI_INFO     *pInfo = (ADI_EMSI_INFO  *)pCBParam;

    ADI_EMSI_DEVICE   *pDevice = pInfo->pDevice ;

    uint32_t WUEvent = (uint32_t)ADI_EMSI_NOEVENT;

    /* Obtain the value of the status register */
    uint16_t IntStatusReg = pInfo->pEMSIRegs->ISTAT;

    /*!< CARD_INSERTION: Card Inserted. */
    if((IntStatusReg & BITM_EMSI_ISTAT_CARD_INSERTION)
       && (pInfo->pEMSIRegs->WU_CTL & BITM_EMSI_WU_CTL_CARD_INSERT))
    {
        WUEvent   |=   (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CARD_INSERTION);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_CARD_INSERTION;
    }

    /*!< CARD_REMOVAL: Card Removed. */
    if((IntStatusReg & BITM_EMSI_ISTAT_CARD_REMOVAL)
       && (pInfo->pEMSIRegs->WU_CTL & BITM_EMSI_WU_CTL_CARD_REMOVAL))
    {
        WUEvent   |=  (uint32_t)((uint32_t)1<<(uint32_t)ADI_EMSI_EVENT_CARD_REMOVAL);
        /*Clearing the interrupt bit*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_CARD_REMOVAL;

        /*Data Transfer stopped so updating the flag to false*/
        pInfo->pDevice->bData_Trans = false;

        /*General transfer stopped*/
        bGeneral_Transfer = false;
    }

    /* If there is a callback registered notify application via callback */
    if(pDevice->pfCallback != NULL)
    {
        /* Notify the event to application */
        pDevice->pfCallback(pDevice->pCBParam,WUEvent,NULL);
    }

}


/* eMSI Controller Setup for eMMC device and SD card */
static ADI_EMSI_RESULT ControllerSetup(ADI_EMSI_TypeDef *pEMSIRegs, ADI_EMSI_DEVICE *pDevice)
{

    switch(pDevice->eCardType)
    {
    case ADI_EMSI_CARD_TYPE_EMMC:
    {
        /*Setting up the common parameters for all versions*/
        /*eMMC Bus Voltage select set to VDD(3.3V) by default set*/

        /*DATA TIMEOUT VALUES*/
        pEMSIRegs->TO_CTL  &= ~BITM_EMSI_TO_CTL_VALUE;
        pEMSIRegs->TO_CTL  |= ((uint8_t)ADI_EMSI_TOUT_CNT_VALUE14<<BITP_EMSI_TO_CTL_VALUE);

        /*ENABLE eMMC INTERFACE by default set*/

        /*Card connected to Controller is an eMMC card*/
        pEMSIRegs->EMMC_CTL  &= ~(BITM_EMSI_EMMC_CTL_CARD_IS_EMMC |
                                  BITM_EMSI_EMMC_CTL_EMMC_RST_N |
                                  BITM_EMSI_EMMC_CTL_EMMC_RST_N_OE);
        pEMSIRegs->EMMC_CTL  |= BITM_EMSI_EMMC_CTL_CARD_IS_EMMC |
                                BITM_EMSI_EMMC_CTL_EMMC_RST_N |
                                BITM_EMSI_EMMC_CTL_EMMC_RST_N_OE ;

        /*Setting up Host Version 4 Parameters (Refer bit field 12 in CTL2 for features of HOST CTRL 4 )*/
        /*Host Version 4 Enable is required for eMMC interface and 26-bit length for ADMA 2 is set*/
        pEMSIRegs->CTL2 &= ~(ENUM_EMSI_CTL2_HOST_VER4_ENABLE_TRUE|ENUM_EMSI_CTL2_ADMA2_LEN_MODE_TRUE);
        pEMSIRegs->CTL2 |= ENUM_EMSI_CTL2_HOST_VER4_ENABLE_TRUE|ENUM_EMSI_CTL2_ADMA2_LEN_MODE_TRUE;

        /* Supporting 32-bit Addressing by default set*/

        break;
    }
    case ADI_EMSI_CARD_TYPE_SDCARD:
    {
        /*Bus Voltage select set to VDD(3.3V) by default set*/

        /*DATA TIMEOUT VALUES*/
        pEMSIRegs->TO_CTL  &= ~BITM_EMSI_TO_CTL_VALUE;
        pEMSIRegs->TO_CTL  |= ((uint8_t)ADI_EMSI_TOUT_CNT_VALUE14<<BITP_EMSI_TO_CTL_VALUE);

        /*ENABLE SD INTERFACE by default set*/

        /*Setting up Host Version 4 Parameters (Refer bit field 12 in CTL2 for features of HOST CTRL 4 )*/
        /*Host Version 4 Enable is required for SD interface and 26-bit length for ADMA 2 is set*/
        pEMSIRegs->CTL2 &= ~(ENUM_EMSI_CTL2_HOST_VER4_ENABLE_TRUE|ENUM_EMSI_CTL2_ADMA2_LEN_MODE_TRUE);
        pEMSIRegs->CTL2 |= ENUM_EMSI_CTL2_HOST_VER4_ENABLE_TRUE|ENUM_EMSI_CTL2_ADMA2_LEN_MODE_TRUE;

        /* Supporting 32-bit Addressing by default set*/

        /*Configuring Controller in DS mode for SD card*/
        pEMSIRegs->CTL2 |= ENUM_EMSI_CTL2_LEGACY;

        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

    return ADI_EMSI_SUCCESS;

}


/*Check for transfer complete*/
static ADI_EMSI_RESULT CheckXfrStatus(ADI_EMSI_INFO *pInfo)
{
    /*Temporary variables*/
    ADI_EMSI_RESULT eResult;

    eResult = ADI_EMSI_BUSY;

    /* Check for success */
    if((pInfo->pEMSIRegs->ISTAT_EN & BITM_EMSI_ISTAT_EN_XFER_COMPLETE))
    {
        /*wait till data transfer is complete*/
        while (!(pInfo->pEMSIRegs->ISTAT & BITM_EMSI_ISTAT_XFER_COMPLETE))
        {
            ;
            /*wait till data transfer is complete*/
        }
        /* Data Transfer over */
        eResult = ADI_EMSI_DATA_TRANS_FINISHED;
        /*Clearing the transfer complete interrupt status*/
        pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_XFER_COMPLETE;

    }

    return eResult;

}

/*Configuring the descriptors for DMA mode of transfer*/
static ADI_EMSI_RESULT DmaConfig(ADI_EMSI_INFO *pInfo, uint32_t *pBuffer)
{

    switch (pInfo->pDevice->eDmaUsed)
    {

    case ADI_EMSI_DMA_USED_ADMA2:
    {
        /*Remaining Data (To Receive or sent)*/
        uint32_t Transferdataremain = 0u;

        /*Checking L1 Memory declaration*/
        uint32_t *pByteBuffer = (uint32_t *)adi_rtl_internal_to_system_addr((uint32_t)pBuffer,1);

        /*Descriptor index set to zero*/
        uint32_t descIndex = 0u;

        /*Checking if it is open ended transfer*/
        if((pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_OPEN_ENDED)||
           (pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_OPEN_ENDED))
        {
            /*Remaining transfer data will be complete 4032 MB (Each descriptor of 64MB * 63 descriptor)*/
            Transferdataremain = (DESCRIPTOR_DATA_TRANSFER_LIMIT * NUM_DMA_DESCRIPTORS);
        }
        else
        {
            /*Remaining transfer data Depending on block size and block count*/
            Transferdataremain = (uint32_t)((pInfo->pDevice->nBlkCnt) * (pInfo->pDevice->nBlkSize));
        }

        /* Flush the buffer from the data cache(s).*/
        FlushDataCache(pByteBuffer, (pByteBuffer+(Transferdataremain/4u)), true);

        /* If more than one descriptor is needed then chain then all and handling 128 MB boundary
         * For predefined transfer :- As one transfer is of ~32MB. logic will come inside  this while loop only when
         * transfer is crossing 128MB boundary (That is also once) otherwise only single descriptor will be created as one descriptor can transfer
         * 64 MB of data.
         * For Open ended transfers: While loop will be true whenever transfer size is bigger then DESCRIPTOR_DATA_TRANSFER_LIMIT
         * Also if condition will be true only once in 256 MB transfer as 128 MB boundary will be crossed otherwise else condition will be true*/
        while ((((Transferdataremain > DESCRIPTOR_DATA_TRANSFER_LIMIT)) ||
                (((((uintptr_t)pByteBuffer & 0x7FFFFFFu) < 0x7FFFFFFu) && ((((uintptr_t)pByteBuffer + (Transferdataremain/4u)) & 0xFFFFFFFu) > 0x7FFFFFFu))
                  &&(((uintptr_t)pByteBuffer & 0xFFFFFFFu) < 0x8000000u))) && (descIndex < NUM_DMA_DESCRIPTORS))
        {

            if((((uintptr_t)pByteBuffer & 0x7FFFFFFu) < 0x7FFFFFFu) && ((((uintptr_t)pByteBuffer+(Transferdataremain/4u)) & 0xFFFFFFFu) > 0x7FFFFFFu)
                 &&(((uintptr_t)pByteBuffer & 0xFFFFFFFu) < 0x8000000u))
            {
                uint16_t Boundary_Lower_16_bit = (uint16_t)((uint32_t)(0x8000000u - ((uintptr_t)pByteBuffer & 0x7FFFFFFu)) & UPPER_16BIT_DATA_LENGTH_MASK);
                uint32_t Boundary_Upper_10_bits_temp = (0x8000000u - ((uintptr_t)pByteBuffer & 0x7FFFFFFu)) & LOWER_10BIT_DATA_LENGTH_MASK;
                uint16_t Boundary_Upper_10_bits = Boundary_Upper_10_bits_temp>>BITP_26_DATA_LENGTH_LO_16BIT;

                /*Setting the Descriptors Valid , Interrupt bit set*/
                vDmaDescriptors[descIndex].DES0 = (uint32_t)(VALID_INTERRUPT_SET|
                                                  (uint32_t)((uint32_t)Boundary_Lower_16_bit<<
                                                              BITP_26_DATA_LENGTH_LO_16BIT)|
                                                  (uint32_t)((uint32_t)Boundary_Upper_10_bits<<
                                                              BITP_26_DATA_LENGTH_UP_10BIT));
                /*Assigning Buffer address*/
#if defined(__ADSPARM__)
                vDmaDescriptors[descIndex].DES1 = (uintptr_t)pByteBuffer;
#else
                vDmaDescriptors[descIndex].DES1 = pByteBuffer;
#endif
                Transferdataremain -= (0x8000000u - ((uintptr_t)pByteBuffer & 0x7FFFFFFu));
                pByteBuffer     = (uint32_t*)(((uintptr_t)pByteBuffer&0xF0000000u)|0x8000000u);
                ++descIndex;
            }
            else
            {
                /*Setting the Descriptors Valid , Interrupt bit set*/
                vDmaDescriptors[descIndex].DES0 = (uint32_t)(VALID_INTERRUPT_SET|
                                                  (uint32_t)((uint32_t)UPPER_16BIT_DATA_LENGTH<<
                                                              BITP_26_DATA_LENGTH_LO_16BIT)|
                                                  (uint32_t)((uint32_t)LOWER_10BIT_DATA_LENGTH<<
                                                              BITP_26_DATA_LENGTH_UP_10BIT));
                /*Assigning Buffer address*/
#if defined(__ADSPARM__)
                vDmaDescriptors[descIndex].DES1 = (uintptr_t)pByteBuffer;
#else
                vDmaDescriptors[descIndex].DES1 = pByteBuffer;
#endif
                pByteBuffer    += (DESCRIPTOR_DATA_TRANSFER_LIMIT/4u);
                Transferdataremain -= DESCRIPTOR_DATA_TRANSFER_LIMIT;
                ++descIndex;
            }

        }
        /* Check that we haven't hit the limit of the number of
         * descriptors. There must still be at least one free descriptor
         * left at this point.
         */
        if (descIndex > NUM_DMA_DESCRIPTORS)
        {
            return ADI_EMSI_DATA_LENGTH_INVALID;
        }
        else
        {
            uint16_t Lower_16_bits = (uint16_t)((uint32_t)Transferdataremain & UPPER_16BIT_DATA_LENGTH_MASK);
            uint32_t Upper_10_bits_temp = Transferdataremain & LOWER_10BIT_DATA_LENGTH_MASK;
            uint16_t Upper_10_bits = Upper_10_bits_temp>>BITP_26_DATA_LENGTH_LO_16BIT;

            /*Last descriptor in Chain
             Setting the Descriptors Valid , Interrupt bit, end bit set*/
            if(Transferdataremain ==  DESCRIPTOR_DATA_TRANSFER_LIMIT)
            {
                /*Maximum Data length*/
                vDmaDescriptors[descIndex].DES0 = (uint32_t)(VALID_INTER_END_SET|
                                                            (uint32_t)((uint32_t)UPPER_16BIT_DATA_LENGTH<<
                                                                        BITP_26_DATA_LENGTH_LO_16BIT)|
                                                            (uint32_t)((uint32_t)LOWER_10BIT_DATA_LENGTH<<
                                                                        BITP_26_DATA_LENGTH_UP_10BIT));
            }
            else
            {
                /*As per transfer data remain*/
                vDmaDescriptors[descIndex].DES0 = ((uint32_t)(VALID_INTER_END_SET|
                                                              (uint32_t)((uint32_t)Lower_16_bits<<
                                                               BITP_26_DATA_LENGTH_LO_16BIT)|
                                                               (uint32_t)((uint32_t)Upper_10_bits<<
                                                               BITP_26_DATA_LENGTH_UP_10BIT)));

            }

            /*Assigning Buffer address*/
#if defined(__ADSPARM__)
            vDmaDescriptors[descIndex].DES1 = (uintptr_t)pByteBuffer;
#else
            vDmaDescriptors[descIndex].DES1 = pByteBuffer;
#endif
            /*Flush the descriptors from the cache so that the DMA engine can see them*/
            FlushDataCache((uint32_t*)(&vDmaDescriptors),(uint32_t*)(&vDmaDescriptors + 1u), true);

            /* Write the start of the descriptor list to the IDMA controller to initiate the transfer*/
            pInfo->pEMSIRegs->ADMA_ADDR_LO= (uint32_t)adi_rtl_internal_to_system_addr(((uint32_t)vDmaDescriptors),1);
        }

        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }
    return ADI_EMSI_SUCCESS;
}



/*Update the buffer address in system address register for SDMA mode*/
static ADI_EMSI_RESULT SdmaUpdateBuffAddress(ADI_EMSI_INFO *pInfo, uint32_t *pBuffer)
{
    /*Temporary Variable*/
    ADI_EMSI_RESULT eResult;

    /*Address of the updated buffer*/
    uint32_t *pUpdatedBuffaddress;

    /*Assigning Updated buffer address*/
    pUpdatedBuffaddress = pBuffer;

    /*Wait till DMA interrupt or transfer complete interrupt occurs*/
    while ((!(pInfo->pEMSIRegs->ISTAT & BITM_EMSI_ISTAT_DMA_INTERRUPT))||(!(pInfo->pEMSIRegs->ISTAT & BITM_EMSI_ISTAT_XFER_COMPLETE)))
    {
        if ((pInfo->pEMSIRegs->ISTAT & BITM_EMSI_ISTAT_DMA_INTERRUPT) == BITM_EMSI_ISTAT_DMA_INTERRUPT)
        {
            /*Clearing the DMA complete interrupt status*/
            pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_DMA_INTERRUPT;

            /*Updating buffer address by buffer boundary size so that next block data will be written at next location*/
            pUpdatedBuffaddress += (pInfo->pDevice->nSdmaBufBdary/4u);

            /* Writing the updated address of the buffer into system address register*/
            pInfo->pEMSIRegs->ADMA_ADDR_LO = (uint32_t)pUpdatedBuffaddress;

        }
        if ((pInfo->pEMSIRegs->ISTAT & BITM_EMSI_ISTAT_XFER_COMPLETE) == BITM_EMSI_ISTAT_XFER_COMPLETE)
        {
            /*Check for transfer complete*/
            eResult = CheckXfrStatus(pInfo);
            return eResult;
        }
    }
    return ADI_EMSI_SUCCESS;
}

/*Sending predefined commands based on transfer type*/
static ADI_EMSI_RESULT Predefined_Command_Send(ADI_EMSI_INFO *pInfo, uint32_t Start_Add)
{
    /*Sending Read or Write Commands Based on Transfer Type*/
    switch(pInfo->pDevice->eTransferType)
    {
    case ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR:
    {
        /*There will be no CMD12 or CMD23 (Auto/ By User) for Single block transfer.
         * Sending CMD 24 for single block write */
        ADI_EMSI_CMD_PARA CMD_PARA24 ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                       ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                       ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,WRITE_SINGLE_BLOCK,Start_Add,&CMD_PARA24))
        {
            return ADI_EMSI_FAILURE;
        }
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF:
    {
        ADI_EMSI_CMD_PARA CMD_PARA25 ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                       ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                       ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

        /*Auto CMD23 is not set*/
        if(pInfo->pDevice->eAutoCmdType != ADI_EMSI_AUTO_CMD23EN)
        {

            /*if Auto CMD 23 is not set
             * CMD23 must be sent for predefined number of blocks.
             * Otherwise, it becomes open ended transfer from card side.*/
            ADI_EMSI_CMD_PARA CMD_PARA23 ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                           ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                           ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
            if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SET_BLOCK_COUNT,
                                                         (uint32_t)(pInfo->pDevice->nBlkCnt),
                                                         &CMD_PARA23))
            {
                return ADI_EMSI_FAILURE;
            }
        }

        /*Auto CMD23 is Set, So send only Write Command
         * Sending CMD 25 for Multi block Write */
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,WRITE_MULTIPLE_BLOCK,Start_Add,&CMD_PARA25))
        {
            return ADI_EMSI_FAILURE;
        }

        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD:
    {
        /*There will be no CMD12 or CMD23 (Auto/ By User) for Single block transfer.
         * Sending CMD 17 for single block read */
        ADI_EMSI_CMD_PARA CMD_PARA17 ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                       ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                       ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,READ_SINGLE_BLOCK,Start_Add,&CMD_PARA17))
        {
            return ADI_EMSI_FAILURE;
        }
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF:
    {
        ADI_EMSI_CMD_PARA CMD_PARA18 ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                       ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                       ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        /*Auto CMD23 is not set*/
        if(pInfo->pDevice->eAutoCmdType != ADI_EMSI_AUTO_CMD23EN)
        {

            /*if Auto CMD 23 is not set
             * CMD23 must be sent for predefined number of blocks.
             * Otherwise, it becomes open ended transfer from card side.*/
            ADI_EMSI_CMD_PARA CMD_PARA23 ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                           ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                           ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
            if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SET_BLOCK_COUNT,
                                                         (uint32_t)(pInfo->pDevice->nBlkCnt),
                                                         &CMD_PARA23))
            {
                return ADI_EMSI_FAILURE;
            }
        }
        /*Auto CMD23 is Set, So send only Write Command
         * Sending CMD 18 for Multi block Read */
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,READ_MULTIPLE_BLOCK,Start_Add,&CMD_PARA18))
        {
            return ADI_EMSI_FAILURE;
        }
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_OPEN_ENDED:
    {
        /*Sending CMD 18 for Multi block Read as CMD23 won't be used for open ended transfer */
        ADI_EMSI_CMD_PARA CMD_PARA18OP ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                         ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                         ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,READ_MULTIPLE_BLOCK,Start_Add,&CMD_PARA18OP))
        {
            return ADI_EMSI_FAILURE;
        }
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_OPEN_ENDED:
    {
        /*Sending CMD 25 for Multi block write as CMD23 won't be used for open ended transfer */
        ADI_EMSI_CMD_PARA CMD_PARA25OP ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                         ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                         ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,WRITE_MULTIPLE_BLOCK,Start_Add,&CMD_PARA25OP))
        {
            return ADI_EMSI_FAILURE;
        }
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT:
    {
        /*Sending CMD 0 with 0xF0F0F0F0 argument to put card in pre-idle state for Normal boot operation  */
        ADI_EMSI_CMD_PARA CMD_PARA_PRE_IDLE ={ADI_EMSI_RESPONSE_TYPE_NONE,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                              ADI_EMSI_CMD_CRC_CHECK_DISABLE,ADI_EMSI_CMD_IDX_CHECK_DISABLE,
                                              ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,GO_IDLE_STATE,0xF0F0F0F0u,&CMD_PARA_PRE_IDLE))
        {
            return ADI_EMSI_FAILURE;
        }
        break;
    }
    case ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT:
    {
        /*CMD 0 parameters for pre idle argument*/
        ADI_EMSI_CMD_PARA CMD_PARA_PRE_IDLE_ALT_BOOT ={ADI_EMSI_RESPONSE_TYPE_NONE,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                                       ADI_EMSI_CMD_CRC_CHECK_DISABLE,ADI_EMSI_CMD_IDX_CHECK_DISABLE,
                                                       ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

        /*Sending CMD 0 with 0xF0F0F0F0 argument to put card in pre-idle state for Alternate boot operation  */
        ADI_EMSI_CMD_PARA CMD_PARA_PRE_IDLE ={ADI_EMSI_RESPONSE_TYPE_NONE,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                              ADI_EMSI_CMD_CRC_CHECK_DISABLE,ADI_EMSI_CMD_IDX_CHECK_DISABLE,
                                              ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,GO_IDLE_STATE,0xF0F0F0F0u,&CMD_PARA_PRE_IDLE))
        {
            return ADI_EMSI_FAILURE;
        }

        /*Sending CMD 0 with 0xFFFFFFFA argument to start the alternate boot process  */
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,GO_IDLE_STATE,0xFFFFFFFAu,&CMD_PARA_PRE_IDLE_ALT_BOOT))
        {
            return ADI_EMSI_FAILURE;
        }
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }
    return ADI_EMSI_SUCCESS;
}




/*Set the card type for eMSI operations.*/
static ADI_EMSI_RESULT SetCardType(ADI_EMSI_INFO *pInfo, ADI_EMSI_CARD_TYPE eCardType)
{

    /* Save the card type*/
    pInfo->pDevice->eCardType = eCardType;

    return ADI_EMSI_SUCCESS;
}



/*Starting or stopping the clock to card
 *
 * bEnable                  Flags:
 *                          'true'  - Start clock to card
 *                          'false' - Stop clock to card
 */
static ADI_EMSI_RESULT ClktoCard(ADI_EMSI_INFO *pInfo,bool bEnable)
{
    if(bEnable)
    {
        /*Start Clock to card*/
        pInfo->pEMSIRegs->CLK_CTL &=  ~BITM_EMSI_CLK_CTL_EMSI_BUS_CLK_EN;
        pInfo->pEMSIRegs->CLK_CTL |=  BITM_EMSI_CLK_CTL_EMSI_BUS_CLK_EN;
    }
    else
    {   /*Stop Clock to card*/
        pInfo->pEMSIRegs->CLK_CTL &= ~(BITM_EMSI_CLK_CTL_EMSI_BUS_CLK_EN);
    }
    return ADI_EMSI_SUCCESS;
}


/*Abort Command sequence*/
static ADI_EMSI_RESULT Abort_Command_Seq(ADI_EMSI_INFO *pInfo)
{
    /*Temporary Variables*/
    uint32_t nTemp=0u;
    uint32_t nTemp1=0u;

    /*CMD13 Parameters*/
    ADI_EMSI_CMD_PARA CMD_PARA13 ={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                   ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                   ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*Send STOP_TRANSMISSION(CMD12) command as subcommand as
     * it will be sent while transfer is going on and abort command*/
    if((pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_OPEN_ENDED)||
       (pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF)||
       (pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR)||
       ((pInfo->pDevice->eGenTransType == ADI_EMSI_GENERAL_TRANS_TYPE_WRITE) && bGeneral_Transfer))
    {
        /*Response type will be short response with busy check for write transfers*/
        ADI_EMSI_CMD_PARA CMD_PARA12={ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY,ADI_EMSI_SUBCMD_TYPE_SUB,
                                      ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                      ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_ABORT};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,STOP_TRANSMISSION,0u,&CMD_PARA12))
        {
            return ADI_EMSI_FAILURE;
        }
    }
    /*Response type will be short response for read transfers*/
    else if((pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_OPEN_ENDED)||
            (pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF)||
            (pInfo->pDevice->eTransferType == ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD)||
            ((pInfo->pDevice->eGenTransType == ADI_EMSI_GENERAL_TRANS_TYPE_READ) && bGeneral_Transfer))
    {
        ADI_EMSI_CMD_PARA CMD_PARA12={ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_SUB,
                                      ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                      ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_ABORT};
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,STOP_TRANSMISSION,0u,&CMD_PARA12))
        {
            return ADI_EMSI_FAILURE;
        }
    }

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_14_10:"All if ... else if constructs shall be terminated with an else clause.")
#endif /* _MISRA_RULES */
    /*Sending CMD13 till card is in transfer state*/
    while(nTemp1!=CARD_IN_TRANSFER_STATE)
    {
        /*Clearing the response storing space*/
        InternalResp[0]=0u;
        /*Sending CMD13 for Checking card status*/
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SEND_STATUS,
                                                     (uint32_t)((uint32_t)pInfo->pDevice->nRca<<16u),
                                                     &CMD_PARA13))
        {
            /*CMD13 Failure*/
            return ADI_EMSI_FAILURE;
        }
        adi_emsi_GetResponse(pInfo,&InternalResp[0]);
        /*Cards current state in R1 response*/
        nTemp=InternalResp[0]>>CARD_CURRENT_STATE;
        nTemp1=nTemp & CARD_CURRENT_STATE_MASK;
    }
#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */
    if (nTemp1==CARD_IN_TRANSFER_STATE)
    {
        /*Card in transfer state*/
        return ADI_EMSI_SUCCESS;
    }
    else
    {
        /*Card failed to enter in transfer state*/
        return ADI_EMSI_FAILURE;
    }
}


/*Enable or Disable eMSI normal interrupts status and error interrupt status and interrupts signals to core.
 *
 * nIstatmask    The set of interrupts status to be enabled, Passing 1 will enable interrupt status bit.
 *
 * nErrstatmask  The set of Error interrupts status to be enabled, Passing 1 will enable Error interrupt status bit.
 *
 * nInterruptmask  The set of interrupts to be enabled to generated  from Controller side to core .
 *
 * nErrInterruptmask  The set of error interrupts to be enabled to generated  from Controller side to core.
 *
 * bEnable       True- Unmask/Enable the interrupt bits
 *               False- Mask the interrupt bits
 */
static ADI_EMSI_RESULT SetInterruptMask(
                       ADI_EMSI_INFO *pInfo,
                       uint16_t nIstatmask,
                       uint16_t nErrstatmask,
                       uint16_t nInterruptmask,
                       uint16_t nErrInterruptmask,
                       bool bEnable)
{

    if (bEnable ==true)
    {
        /*Enable interrupt Status bits*/
        pInfo->pEMSIRegs->ISTAT_EN |= nIstatmask;
        /*Enable error interrupt Status bits*/
        pInfo->pEMSIRegs->ERR_STAT_EN  |= nErrstatmask;
        /*Enable interrupt to core*/
        pInfo->pEMSIRegs->ISTAT_INTEN |= nInterruptmask;
        /*Enable error interrupt to core*/
        pInfo->pEMSIRegs->ERR_STAT_INTEN  |=nErrInterruptmask;
    }
    else
    {
        /*Disable interrupt Status bits*/
        pInfo->pEMSIRegs->ISTAT_EN &= ~nIstatmask;
        /*Disable error interrupt Status bits*/
        pInfo->pEMSIRegs->ERR_STAT_EN  &= ~nErrstatmask;
        /*Disable interrupt to core*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~nInterruptmask;
        /*Disable error interrupt to core*/
        pInfo->pEMSIRegs->ERR_STAT_INTEN  &= ~nErrInterruptmask;
    }

    return ADI_EMSI_SUCCESS;
}

/*Enable or Disable eMSI Command queuing interrupts status and interrupts signals to core.
 *
 * nCqIstatmask    The set of interrupts status to be enabled, Passing 1 will enable interrupt status bit.
 *
 * nCqInterruptmask  The set of interrupts to be enabled to generated  from Controller side to core .
 *
 * bEnable       True- Unmask/Enable the interrupt bits
 *               False- Mask the interrupt bits
 */
static ADI_EMSI_RESULT SetCqInterruptMask(ADI_EMSI_INFO *pInfo, uint32_t nCqIstatmask, uint32_t nCqInterruptmask,bool bEnable)
{
    if (bEnable ==true)
    {
        /*Enable interrupt Status bits*/
        pInfo->pEMSIRegs->CQ_ISTAT_EN |= nCqIstatmask;
        /*Enable interrupt to core*/
        pInfo->pEMSIRegs->CQ_ISTAT_INTEN |= nCqInterruptmask;
    }
    else
    {
        /*Disable interrupt Status bits*/
        pInfo->pEMSIRegs->CQ_ISTAT_EN &= ~nCqIstatmask;
        /*Disable interrupt to core*/
        pInfo->pEMSIRegs->CQ_ISTAT_INTEN &=  ~nCqInterruptmask;
    }

    return ADI_EMSI_SUCCESS;
}

/*Getting EXT_CSD register content using CMD8*/
static ADI_EMSI_RESULT GetExtCsdcontent(ADI_EMSI_INFO *pInfo)
{
    /*Checking if command queuing supported by the eMMC device by reading EXT_CSD register*/
    /*Parameters for CMD8*/
    ADI_EMSI_CMD_PARA CMD_PARA8 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                   ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                   ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*Using ADMA2 for data transfers*/
    adi_emsi_SetDmaType(pInfo,ADI_EMSI_DMA_USED_ADMA2,ADI_EMSI_SDMA_BUF_BDARY_4K);

    /* Set the block size for upcoming transfer(s) 512 Bytes.*/
    adi_emsi_SetBlkSze(pInfo,BLOCK_SIZE_EXT_CSD_REG);

    /*Disabling the transfer complete interrupt to core, polling it internally*/
    pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

    /*Sending CMD8 to check whether command queuing  is supported by card*/
    /*Using general read transfers for saving EXT_CSD register content in buffer*/
    adi_emsi_General_Transfer(pInfo,ADI_EMSI_GENERAL_TRANS_TYPE_READ,NO_OF_BLOCKS_EXT_CSD_REG,
                              &Ext_CsdRegBuff[0]);

    /*Sending CMD8 (Switch) command*/
    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SEND_EXT_CSD,(uint32_t)pInfo->pDevice->nRca,&CMD_PARA8))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Check for transfer complete*/
    if(ADI_EMSI_DATA_TRANS_FINISHED != CheckXfrStatus(pInfo))
    {
        return ADI_EMSI_FAILURE;
    }
    else
    {
        /*Updating data transfer flag to complete*/
        pInfo->pDevice->bData_Trans = false;

        /*Flag for send command and resp access API being used internally*/
        pInfo->pDevice->bInternal_Cmd = false;

        /*General transfer completed*/
        bGeneral_Transfer = false;

        /*Enabling the transfer complete interrupt to core for the next transfers*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);
        pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

        /*Check whether command queuing supported by card*/
        if (Ext_CsdRegBuff[308] == CMD_QUE_CARD_SUPPORT)
        {
            /*Command queuing depth supported by card*/
            pInfo->pDevice->nCmdQueDepth = Ext_CsdRegBuff[307];
            /*(N+1) is the depth of the queue (e.g. if Ext_CsdRegBuff[307] =15 then queue depth is 16)*/
            pInfo->pDevice->nCmdQueDepth++;
            return ADI_EMSI_SUCCESS;
        }
        else
        {
            /*Command queuing is not supported*/
            return ADI_EMSI_FAILURE;
        }
    }

}

/*Enabling/Disabling command queuing from card side*/
static ADI_EMSI_RESULT SetCmdQue(ADI_EMSI_INFO *pInfo,bool bCmdQueEnable)
{
    /*Temporary Variables*/
    uint32_t nTemp=0u;

    /*Parameters for CMD13*/
    ADI_EMSI_CMD_PARA CMD_PARA13 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                    ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                    ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*Sending CMD6 (Switch) for switching bus width from card side*/
    ADI_EMSI_CMD_PARA CMD_PARA6 = {ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                   ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                   ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SWITCH,(((uint32_t)((uint32_t)bCmdQueEnable<<BITP_VALUE) |
                                                 ((uint32_t)CMDQ_MODE_EN<<BITP_INDEX) |
                                                 ((uint32_t)ACCESS_WRITEBYTE<<BITP_ACCESS))),
                                                 &CMD_PARA6))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Check if the switch was successful by reading the card status using CMD13*/
    if(ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SEND_STATUS,
                                                (uint32_t)((uint32_t)pInfo->pDevice->nRca<<16),
                                                &CMD_PARA13))
    {
        return ADI_EMSI_FAILURE;
    }
    adi_emsi_GetResponse(pInfo,&InternalResp[0]);
    /*Checking Switch error bit from response received*/
    nTemp = InternalResp[0]>>BITP_SWITCH_CMD_ERROR_CHK;
    nTemp = nTemp & SWITCH_CMD_ERROR_MASK;
    if(nTemp==SWITCH_CMD_ERROR)
    {
        /*Failure in CMD6 (Switch error bit is set in R1 response)*/
        return ADI_EMSI_FAILURE;
    }

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = false;

    return ADI_EMSI_SUCCESS;
}

/*
 * Configures eMSI bus clock frequency. Set up the dividers and synchronizing the clock.
 *
 * To Configure eMSI bus clock for upcoming transactions. eMSI input clock will be divided by (2*Divider Value).
 * This divided clock will be the clock to the card.
 * Once the divider values are set this API synchronizing the clock to avoid glitches.
 *
 *nDivisor      Clock divider value to be applied to eMSI input clock.
 *              Minimum value for nDivisor = 0 (Here, Output_EMSI_Clk_Frequency = Input_EMSI_Clk_Frequency)
 *              For Rest Values: Output_Clk_Frequency = Input_Clk_Frequency/(2*nDivisor)
 *              Maximum value for nDivisor = 1023.
 *
 * Status
 *
 *    - #ADI_EMSI_SUCCESS       Successfully configured for eMSI bus clock.
 *
 *    - #ADI_EMSI_TIMED_OUT     Time out during synchronizing operation.
 */

static ADI_EMSI_RESULT SetClock(
                       ADI_EMSI_INFO *pInfo,
                       uint16_t nDivisor)
{
    /*Temporary Variable*/
    uint16_t nLower_Freq_Divider;
    uint16_t nUpper_Freq_Divider;
    uint32_t nTimeout_Sync=0u;

    /*By default selecting Divided Clock Mode For Operation*/

    /*Clearing the Bit before setting it*/
    pInfo->pEMSIRegs->CLK_CTL &= ~(BITM_EMSI_CLK_CTL_FREQ_SEL);
    pInfo->pEMSIRegs->CLK_CTL &= ~(BITM_EMSI_CLK_CTL_UPPER_FREQ_SEL);

    /*Configuring divider value*/
    nLower_Freq_Divider = (nDivisor & LOWER_FREQ_SEL);
    pInfo->pEMSIRegs->CLK_CTL |= (nLower_Freq_Divider<<BITP_EMSI_CLK_CTL_FREQ_SEL);
    nUpper_Freq_Divider = ((nDivisor & UPPER_FREQ_SEL)>>BITP_UPPER_FREQ_SEL);
    pInfo->pEMSIRegs->CLK_CTL |= (nUpper_Freq_Divider<<BITP_EMSI_CLK_CTL_UPPER_FREQ_SEL);


    /*Clearing the Bit before setting it*/
    pInfo->pEMSIRegs->CLK_CTL &= ~(BITM_EMSI_CLK_CTL_INTERNAL_CLK_EN);
    /*Enable the internal clock Signal*/
    pInfo->pEMSIRegs->CLK_CTL |= BITM_EMSI_CLK_CTL_INTERNAL_CLK_EN;


    /*Wait For Clock Synchronization*/
    while(!(pInfo->pEMSIRegs->CLK_CTL & BITM_EMSI_CLK_CTL_INTERNAL_CLK_STABLE))
    {
        nTimeout_Sync++;
        if(nTimeout_Sync >= TIMEOUT)
        {
            /*Timeout Occurred*/
            return ADI_EMSI_TIMED_OUT;
        }
    }

    /*Clearing the Bit before setting it*/
    pInfo->pEMSIRegs->CLK_CTL &= ~(BITM_EMSI_CLK_CTL_PLL_EN);
    /*To activate the PLL clock (applicable when Host Version 4 Enable = 1)*/
    pInfo->pEMSIRegs->CLK_CTL |= BITM_EMSI_CLK_CTL_PLL_EN;

    /*Resetting the count*/
    nTimeout_Sync=0u;
    /*Wait For Clock Synchronization*/
    while(!(pInfo->pEMSIRegs->CLK_CTL & BITM_EMSI_CLK_CTL_INTERNAL_CLK_STABLE))
    {
        nTimeout_Sync++;
        if(nTimeout_Sync >= TIMEOUT)
        {
            /*Timeout Occurred*/
            return ADI_EMSI_TIMED_OUT;
        }
    }

    /*Enable clock to the card as per programmed value*/
    ClktoCard(pInfo,true);

    return ADI_EMSI_SUCCESS;
}


/*
 * Set the command register.
 *
 * This function sets the bits of CMD register which will be used in adi_emsi_SendCommand() API.
 * Configuration for each command is different.(Please refer to CMD register in eMSI chapter
 * of the HRM for more details).
 *
 * pCmdparameters   Pointer to CMD register parameter structure.
 *
 * Status
 * - #ADI_EMSI_SUCCESS                CMD register parameters set successfully.
 *
 */
static ADI_EMSI_RESULT CmdRegSet(
                       ADI_EMSI_INFO *pInfo,
                       ADI_EMSI_CMD_PARA *pCmdparameters)
{

    /*Clearing all the parameters set by previous command*/
    pInfo->pDevice->nCmdParameters = FOUR_BYTE_ZERO;

    /*Response Type*/
    switch(pCmdparameters->eRespType)
    {
    case ADI_EMSI_RESPONSE_TYPE_NONE:
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_NO_RESP;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_NO_RESP;
        break;
    }
    case ADI_EMSI_RESPONSE_TYPE_LONG:
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_RESP_LEN_136;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_RESP_LEN_136;
        break;
    }
    case ADI_EMSI_RESPONSE_TYPE_SHORT:
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_RESP_LEN_48;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_RESP_LEN_48;
        break;
    }
    case ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY:
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_RESP_LEN_48B;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_RESP_LEN_48B;
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_14_10:"All if ... else if constructs shall be terminated with an else clause.")
#endif /* _MISRA_RULES */
    /*Sub-command*/
    if(pCmdparameters->eSubCmdType == ADI_EMSI_SUBCMD_TYPE_MAIN)
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_MAIN;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_MAIN;

    }
    else if(pCmdparameters->eSubCmdType == ADI_EMSI_SUBCMD_TYPE_SUB)
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_SUB;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_SUB;
    }

    /*CMD CRC Check*/
    if (pCmdparameters->eCmdCrcCheck == ADI_EMSI_CMD_CRC_CHECK_DISABLE)
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_CMD_CRC_CHK_DISABLED;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_CMD_CRC_CHK_DISABLED;
    }
    else if(pCmdparameters->eCmdCrcCheck == ADI_EMSI_CMD_CRC_CHECK_ENABLE)
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_CMD_CRC_CHK_ENABLED;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_CMD_CRC_CHK_ENABLED;
    }

    /*CMD Index Check*/
    if (pCmdparameters->eCmdIdxCheck == ADI_EMSI_CMD_IDX_CHECK_DISABLE)
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_CMD_IDX_CHK_DISABLED;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_CMD_IDX_CHK_DISABLED;
    }
    else if (pCmdparameters->eCmdIdxCheck == ADI_EMSI_CMD_IDX_CHECK_ENABLE)
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_CMD_IDX_CHK_ENABLED;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_CMD_IDX_CHK_ENABLED;
    }

    /*Data Present Check*/
    if (pCmdparameters->eDataPresent == ADI_EMSI_DATA_PRESENT_FALSE)
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_NO_DATA;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_NO_DATA;
    }
    else if (pCmdparameters->eDataPresent == ADI_EMSI_DATA_PRESENT_TRUE)
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_DATA;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_DATA;
    }

    /*CMD Type*/
    switch(pCmdparameters->eCmdType)
    {
    case ADI_EMSI_SND_CMD_TYPE_NORMAL:
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_NORMAL_CMD;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_NORMAL_CMD;
        break;
    }
    case ADI_EMSI_SND_CMD_TYPE_ABORT:
    {
        pInfo->pDevice->nCmdParameters &= ~(uint8_t)ENUM_EMSI_CMD_ABORT_CMD;
        pInfo->pDevice->nCmdParameters |= (uint8_t)ENUM_EMSI_CMD_ABORT_CMD;
        break;
    }
    default:
    {
        return ADI_EMSI_INVALID_CONFIGURATION;
    }
    }

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

    return ADI_EMSI_SUCCESS;
}
/*
 * Storing the SD card status in buffer.
 */
static ADI_EMSI_RESULT GetSdStatus(ADI_EMSI_INFO *pInfo)
{
    /*Temporary variables*/
    uint32_t nTemp;

    /*Parameters for CMD55 APP CMD*/
    ADI_EMSI_CMD_PARA CMD_PARA55 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                    ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                    ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*Using ADMA2 for data transfers*/
    adi_emsi_SetDmaType(pInfo,ADI_EMSI_DMA_USED_ADMA2,ADI_EMSI_SDMA_BUF_BDARY_4K);

    /* Set the block size for upcoming transfer(s) 64 Bytes -512 bits.*/
    adi_emsi_SetBlkSze(pInfo,BLOCK_SIZE_SD_STATUS);

    /*Issue APP_CMD to indicate that the command following is application specific*/
    if(ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,APP_CMD,
                                                (uint32_t)((uint32_t)pInfo->pDevice->nRca<<16),
                                                &CMD_PARA55))
    {
        return ADI_EMSI_FAILURE;
    }
    /*Checking Status APP CMD bit in response*/
    adi_emsi_GetResponse(pInfo,&InternalResp[0]);
    nTemp = InternalResp[0]>>BITP_APP_CMD_CHK;
    nTemp = nTemp & APP_CMD_CHK_MASK;
    if (nTemp != APP_CMD_CHK)
    {
        return ADI_EMSI_FAILURE;
    }
    else
    {
        /*Parameters for CMD13 APP CMD*/
        ADI_EMSI_CMD_PARA APP_CMD_PARA13 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                            ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                            ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

        /*Disabling the transfer complete interrupt to core, polling it internally*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

        /*Using general read transfers for saving SD status in buffer*/
        adi_emsi_General_Transfer(pInfo,ADI_EMSI_GENERAL_TRANS_TYPE_READ,NO_OF_BLOCKS_SD_STATUS,
                                  &InternalSDStatus[0]);

        /*Flag for send command and resp access API being used internally*/
        pInfo->pDevice->bInternal_Cmd = true;

        /*Sending ACMD13 (SD_STATUS) command*/
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SEND_STATUS,
                                                     (uint32_t)((uint32_t)pInfo->pDevice->nRca<<16/*Stuff Bits*/),
                                                     &APP_CMD_PARA13))
        {
            return ADI_EMSI_FAILURE;
        }

        /*Check for transfer complete*/
        if(ADI_EMSI_DATA_TRANS_FINISHED != CheckXfrStatus(pInfo))
        {
            return ADI_EMSI_FAILURE;
        }
        else
        {
            /*Updating data transfer flag to complete*/
            pInfo->pDevice->bData_Trans = false;

            /*General transfer completed*/
            bGeneral_Transfer = false;
        }

        /*Enabling the transfer complete interrupt to core for the next transfers*/
        pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);
        pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

    }

    return ADI_EMSI_SUCCESS;
}


/*
 * Switching speed mode in SD card.
 */
static ADI_EMSI_RESULT SdHighSpeedMode(ADI_EMSI_INFO *pInfo,bool bHs_Timings)
{
    /*Temporary variables*/
    uint32_t nTempSwitchStat;
    /*Parameters for CMD6*/
    ADI_EMSI_CMD_PARA CMD_PARA6 = {ADI_EMSI_RESPONSE_TYPE_SHORT,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                   ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                   ADI_EMSI_DATA_PRESENT_TRUE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*Using ADMA2 for data transfers*/
    adi_emsi_SetDmaType(pInfo,ADI_EMSI_DMA_USED_ADMA2,ADI_EMSI_SDMA_BUF_BDARY_4K);

    /* Set the block size for upcoming transfer(s) 64 Bytes -512 bits.*/
    adi_emsi_SetBlkSze(pInfo,BLOCK_SIZE_SD_STATUS);

    /*Disabling the transfer complete interrupt to core, polling it internally*/
    pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

    /*Sending CMD6 to check whether speed mode is supported by card*/
    /*Using general read transfers for saving SD status in buffer*/
    adi_emsi_General_Transfer(pInfo,ADI_EMSI_GENERAL_TRANS_TYPE_READ,NO_OF_BLOCKS_SD_STATUS,
                              &SwitchFuncStatus[0]);

    /*Sending CMD6 (Switch) command*/
    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    if(bHs_Timings == true)
    {
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SWITCH,((uint32_t)((uint32_t)SD_HIGH_SPEED_ACCESS_MODE) |
                                                     ((uint32_t)SD_CHECK_FUNCTION<<BITP_SWITCH_FUNCTION_MODE)),
                                                     &CMD_PARA6))
        {
            return ADI_EMSI_FAILURE;
        }
    }
    else
    {
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SWITCH,((uint32_t)((uint32_t)SD_DEFAULT_ACCESS_MODE) |
                                                     ((uint32_t)SD_CHECK_FUNCTION<<BITP_SWITCH_FUNCTION_MODE)),
                                                     &CMD_PARA6))
        {
            return ADI_EMSI_FAILURE;
        }

    }

    /*Check for transfer complete*/
    if(ADI_EMSI_DATA_TRANS_FINISHED != CheckXfrStatus(pInfo))
    {
        return ADI_EMSI_FAILURE;
    }
    else
    {
        /*Updating data transfer flag to complete*/
        pInfo->pDevice->bData_Trans = false;
    }
#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_10_1_a:"The value of an expression of integer type shall not be implicitly converted to a different underlying type if it is not a conversion to a wider integer type of the same signedness.")
#endif /* _MISRA_RULES */
    /*Check if required speed mode is supported*/
    nTempSwitchStat = SwitchFuncStatus[3];
    nTempSwitchStat = nTempSwitchStat>>BITP_SPEED_MODE_SUPPORT;
    nTempSwitchStat = nTempSwitchStat&SPEED_MODE_SUPPORT_MASK;

    if((((bHs_Timings)&&(((nTempSwitchStat)&(HS_MODE_MASK))==0u)))||(((!bHs_Timings)&&(((nTempSwitchStat)&(DS_MODE_MASK))==0u))))
    {
        /*Required speed mode not supported by the card connected*/
        return ADI_EMSI_FAILURE;
    }


    /*Check if the function is ready*/
    nTempSwitchStat=SwitchFuncStatus[7];
    nTempSwitchStat=nTempSwitchStat>>BITP_SPEED_MODE_READY;
    nTempSwitchStat= nTempSwitchStat&SPEED_MODE_READY_MASK;

    if((((bHs_Timings)&&(((nTempSwitchStat)&(HS_MODE_READY_MASK))!=0u)))||(((!bHs_Timings)&&(((nTempSwitchStat)&(DS_MODE_READY_MASK))!=0u))))
    {
        /*Function is busy currently and can't be switched now*/
        return ADI_EMSI_FAILURE;
    }


    /*Sending CMD6 again to change the speed mode*/
    /* Set the block size for upcoming transfer(s).*/
    adi_emsi_SetBlkSze(pInfo,BLOCK_SIZE_SD_STATUS);

    /*Using general read transfers for saving SD status in buffer*/
    adi_emsi_General_Transfer(pInfo,ADI_EMSI_GENERAL_TRANS_TYPE_READ,NO_OF_BLOCKS_SD_STATUS,
                              &SwitchFuncStatus[0]);

    /*Flag for send command and resp access API being used internally*/
    pInfo->pDevice->bInternal_Cmd = true;

    if(bHs_Timings == true)
    {
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SWITCH,((uint32_t)((uint32_t)SD_HIGH_SPEED_ACCESS_MODE) |
                                                     ((uint32_t)SD_SWITCH_FUNCTION<<BITP_SWITCH_FUNCTION_MODE)),
                                                     &CMD_PARA6))
        {
            return ADI_EMSI_FAILURE;
        }
    }
    else
    {
        if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,SWITCH,((uint32_t)((uint32_t)SD_DEFAULT_ACCESS_MODE) |
                                                     ((uint32_t)SD_SWITCH_FUNCTION<<BITP_SWITCH_FUNCTION_MODE)),
                                                     &CMD_PARA6))
        {
            return ADI_EMSI_FAILURE;
        }
    }
        /*Check for transfer complete*/
    if(ADI_EMSI_DATA_TRANS_FINISHED != CheckXfrStatus(pInfo))
    {
        return ADI_EMSI_FAILURE;
    }
    else
    {
        /*Updating data transfer flag to complete*/
        pInfo->pDevice->bData_Trans = false;

        /*General transfer completed*/
        bGeneral_Transfer = false;
    }

    /*Check if the function is ready*/
    nTempSwitchStat=SwitchFuncStatus[7];
    nTempSwitchStat=nTempSwitchStat>>BITP_SPEED_MODE_READY;
    nTempSwitchStat= nTempSwitchStat&SPEED_MODE_READY_MASK;

    if((((bHs_Timings)&&(((nTempSwitchStat)&(HS_MODE_READY_MASK))!=0u)))||(((!bHs_Timings)&&(((nTempSwitchStat)&(DS_MODE_READY_MASK))!=0u))))
    {
        /*Function is busy currently and can't be switched now*/
        return ADI_EMSI_FAILURE;
    }
    else
    {
        nTempSwitchStat = SwitchFuncStatus[4];
        nTempSwitchStat=nTempSwitchStat&STATUS_ERROR_MASK;
        if(((bHs_Timings)&&(nTempSwitchStat!=STATUS_ERROR_HS))||((!bHs_Timings)&&(nTempSwitchStat!=STATUS_ERROR_DS)))
        {
            /*Error in the status code*/
            return ADI_EMSI_FAILURE;
        }
    }
#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

    /*Enabling the transfer complete interrupt to core for the next transfers*/
    pInfo->pEMSIRegs->ISTAT_INTEN &= ~(BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);
    pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);


    /*Successfully changed SD speed mode*/
    return ADI_EMSI_SUCCESS;
}

/*CQ cleanup*/
static ADI_EMSI_RESULT CqeCleanup(ADI_EMSI_INFO *pInfo)
{
    ADI_EMSI_CMD_PARA CMD_PARA48 = {ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY,ADI_EMSI_SUBCMD_TYPE_MAIN,
                                    ADI_EMSI_CMD_CRC_CHECK_ENABLE,ADI_EMSI_CMD_IDX_CHECK_ENABLE,
                                    ADI_EMSI_DATA_PRESENT_FALSE,ADI_EMSI_SND_CMD_TYPE_NORMAL};

    /*wait till all tasks are completed*/
    while (!(pInfo->pEMSIRegs->CQ_ISTAT & BITM_EMSI_CQ_ISTAT_TCC))
    {
        ;
        /*wait till all task is completed*/
    }

    /*Resetting the CQ_IC counter as all task are completed*/
    pInfo->pEMSIRegs->CQ_IC |= (BITM_EMSI_CQ_IC_INTC_RST);
    /*Resetting the bit for future use (Write only bit)*/
    pInfo->pEMSIRegs->CQ_IC &= ~(BITM_EMSI_CQ_IC_INTC_RST);

    /*Clearing the task complete interrupt status*/
    pInfo->pEMSIRegs->CQ_ISTAT = BITM_EMSI_CQ_ISTAT_TCC;

    /*Clearing the task clear notification register for all completed task*/
    pInfo->pEMSIRegs->CQ_TCN = CQE_TCN_REG_CLEAR;

    /*Halting the CQE to gain access of bus for clearing all tasks from controller and card end*/
    pInfo->pEMSIRegs->CQ_CTL |= BITM_EMSI_CQ_CTL_HALT;

    /*wait till eMMC bus control is released by CQE*/
    while (!(pInfo->pEMSIRegs->CQ_CTL & BITM_EMSI_CQ_CTL_HALT))
    {
        ;
        /*wait till eMMC bus control is released by CQE**/
    }

    /*Clear all task in CQE*/
    pInfo->pEMSIRegs->CQ_CTL |= BITM_EMSI_CQ_CTL_CLR_ALL_TASKS;

    /*wait till all task queue cleared by CQE*/
    while (!(pInfo->pEMSIRegs->CQ_ISTAT & BITM_EMSI_CQ_ISTAT_TCL))
    {
        ;
        /*wait till all task getting cleared*/
    }

    /*Clearing the task clear interrupt status and Halt status*/
    pInfo->pEMSIRegs->CQ_ISTAT = BITM_EMSI_CQ_ISTAT_TCL|BITM_EMSI_CQ_ISTAT_HAC;

    /*Updating data progress as complete*/
    pInfo->pDevice->bData_Trans = false;

    /*CQ configuration register disabling the CQE */
    pInfo->pEMSIRegs->CQ_CFG &= ~(BITM_EMSI_CQ_CFG_CQ_EN);

    /*Clearing the interrupt bit*/
    pInfo->pEMSIRegs->ISTAT = BITM_EMSI_ISTAT_CQE_EVENT;

    /*Enabling the interrupt status for next transfers*/
    pInfo->pEMSIRegs->ISTAT_EN |= (BITM_EMSI_ISTAT_EN_CMD_COMPLETE|BITM_EMSI_ISTAT_EN_XFER_COMPLETE);
    /*Enabling the interrupts if next transfers are not based on CQE*/
    pInfo->pEMSIRegs->ISTAT_INTEN |= (BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE);

    /*Dummy write for next writes to go through*/
    pInfo->pEMSIRegs->ARG &= BITM_EMSI_ARG_VALUE;

    /*Issuing CMD48 to clear all task from card end(For Sanity)*/
    if (ADI_EMSI_SUCCESS != adi_emsi_SendCommand(pInfo,CMDQ_TASK_MGMT,EMPTY_CARD_QUEUE,&CMD_PARA48))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Disable Command queuing feature from card side*/
    if(ADI_EMSI_SUCCESS != SetCmdQue(pInfo,false))
    {
        return ADI_EMSI_FAILURE;
    }

    /*Clearing the threshold value for future use and Disabling CQ_IC*/
    pInfo->pEMSIRegs->CQ_IC = (uint32_t)((BITM_EMSI_CQ_IC_INTC_TH_WEN|((uint32_t)(CLR_CQ_IC_THRESHOLD)<<BITP_EMSI_CQ_IC_INTC_TH))
                                          |((uint32_t)((uint32_t)0u << BITP_EMSI_CQ_IC_INTC_EN)));

    /*Clearing the bit for future use(Write only bit)*/
    pInfo->pEMSIRegs->CQ_IC &= ~(BITM_EMSI_CQ_IC_INTC_TH_WEN);

    /*Clearing for future use*/
    pInfo->pEMSIRegs->CQ_SSCFG2 &= ~BITM_EMSI_CQ_SSCFG2_SQSCMD_RCA;

    /*Clearing up all interrupts of CQE*/
    SetCqInterruptMask(pInfo,ENABLE_CQ_INT_STAT,ENABLE_CQ_INT,false);

    /*Clearing up Task Descriptor List Base Address register*/
     pInfo->pEMSIRegs->CQ_TDL_BADDR &=  ~ BITM_EMSI_CQ_TDL_BADDR_TDLBA;

    /*Clearing the bit for future use*/
    pInfo->pEMSIRegs->CQ_CTL &= ~(BITM_EMSI_CQ_CTL_HALT);

    return ADI_EMSI_SUCCESS;
}

/*Delay in terms of clock cycles*/
static ADI_EMSI_RESULT EmsiDelay(ADI_EMSI_INFO *pInfo, uint32_t DelayTermsClockCycles)
{
    /*Temporary Variables*/
    ADI_PWR_RESULT ePwrResult;
    ADI_TMR_RESULT eTmrResult;
    ADI_TMR_CFG_PARAMS ConfigParams;
    ADI_TMR_HANDLE phTMR;
    uint8_t TimerMemory[ADI_TMR_MEMORY];
    double Delay_Clock_Cycles;
    uint32_t No_Of_Cycles;
    uint32_t Divider = 0u;
    /* System Clock Frequencies*/
    uint32_t   fsysclk = 0u;   /* System Clock */
    uint32_t   fsclk0  = 0u;   /* System Clock0 */
    uint32_t   fsclk1  = 0u;   /* System Clock1 */

    /*Delay is not over*/
    bInit_Sequence_Delay = false;

    /* Initialize the power services */
    ePwrResult = adi_pwr_Init(0u, ADI_EMSI0_SYS_CLKIN);
    if(((ePwrResult != ADI_PWR_SUCCESS) && (ePwrResult != ADI_PWR_DEVICE_IN_USE)))
    {
        return ADI_EMSI_PWR_CONFIG_FAILURE;
    }

    /* Using power services get system clock frequencies */
    ePwrResult =  adi_pwr_GetSystemFreq(0u, &fsysclk, &fsclk0, &fsclk1);
    if(ePwrResult != ADI_PWR_SUCCESS)
    {
        return ADI_EMSI_PWR_CONFIG_FAILURE;
    }

    /*Calculating delay required in seconds for given clock cycles*/
    Divider = (uint32_t)((uint32_t)((pInfo->pEMSIRegs->CLK_CTL & BITM_EMSI_CLK_CTL_FREQ_SEL)>>(8u))|
                         ((uint32_t)(pInfo->pEMSIRegs->CLK_CTL & BITM_EMSI_CLK_CTL_UPPER_FREQ_SEL)<<(2u)));

    if(Divider)
    {
        Delay_Clock_Cycles = (double)DelayTermsClockCycles*((double)1/((double)(ADI_EMSI0_CLKIN/(Divider*2u))));
    }
    else
    {
        Delay_Clock_Cycles = (double)DelayTermsClockCycles*((double)1/((double)(ADI_EMSI0_CLKIN)));
    }

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_10_1_a:"The value of an expression of integer type shall not be implicitly converted to a different underlying type if it is not a conversion to a wider integer type of the same signedness.")
#pragma diag(suppress:misra_rule_10_2_a:"The value of an expression of floating type shall not be implicitly converted to a different type if it is not a conversion to a wider floating type.")
#endif /* _MISRA_RULES */
    /*No_of_cycles sclk cycles for given cycles delay*/
    No_Of_Cycles = (fsclk0 * Delay_Clock_Cycles);
#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

    /*Open the timer device*/
    eTmrResult =   adi_tmr_Open (ADI_EMSI0_TMR_NUM,TimerMemory,ADI_TMR_MEMORY,TimerEmsiCallback,NULL,&phTMR);
    if (eTmrResult != ADI_TMR_SUCCESS)
    {
        return ADI_EMSI_TMR_CONFIG_FAILURE;
    }

    /*Configure the device in PWM Out Mode*/
    ConfigParams.Mode= ADI_TMR_MODE_SINGLE_PWMOUT;
    ConfigParams.IRQMode= ADI_TMR_IRQMODE_WIDTH_DELAY;
    ConfigParams.PulsePolarity= false;
    ConfigParams.ClkInSource= ADI_TMR_CLKIN_SYSCLK;
    ConfigParams.InputAuxIn= false;
    ConfigParams.Width= No_Of_Cycles/2u;
    ConfigParams.Delay= No_Of_Cycles/2u;
    ConfigParams.EnableGracefulStop= true;

    eTmrResult =  adi_tmr_Config(phTMR, &ConfigParams);
    if (eTmrResult != ADI_TMR_SUCCESS)
    {
        return ADI_EMSI_TMR_CONFIG_FAILURE;
    }

    /*Enable the timer*/
    eTmrResult =   adi_tmr_Enable(phTMR,true);
    if (eTmrResult != ADI_TMR_SUCCESS)
    {
        return ADI_EMSI_TMR_CONFIG_FAILURE;
    }
    /* Wait for timer interrupts - exit the loop after a while */
    while(!(bInit_Sequence_Delay))
    {
        ; /*Waiting till delay complete*/
    }

    /* Disable the timer*/
    eTmrResult = adi_tmr_Enable(phTMR, false);
    if (eTmrResult != ADI_TMR_SUCCESS)
    {
        return ADI_EMSI_TMR_CONFIG_FAILURE;
    }

    /* Close the timer*/
    eTmrResult = adi_tmr_Close(phTMR);
    if (eTmrResult != ADI_TMR_SUCCESS)
    {
        return ADI_EMSI_TMR_CONFIG_FAILURE;
    }

    return ADI_EMSI_SUCCESS;

}

static void TimerEmsiCallback(void *pCBParam, uint32_t nEvent, void *pArg)
{
    switch ((ADI_TMR_EVENT)nEvent)
    {
    case ADI_TMR_EVENT_DATA_INT:
        bInit_Sequence_Delay = true;
        break;
    default:
        break;
    }
}

/**************************Cache Management*******************************/
static void FlushDataCache(uint32_t *pBufferStart, uint32_t *pBufferEnd, bool invalidate)
{
#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_11_5:"Can't change the prototype of flush_data_buffer() to take const *.")
#pragma diag(suppress:misra_rule_17_4:"We can't get the end address without pointer arithmetic.")
#endif /* _MISRA_RULES */
#if defined (__ADSPGCC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif
    /*Rounding off logic for end address if end address is not aligned with cache line(64B).
     *(Not doing this will cause cache to flushed improperly)
     *As pBufferEnd is integer pointer increasing it by 0x10 (64 Bytes) will align the end address
     *to the next possible cache align location. So we are assured that required memory is flushed.
     */
    if(((uint32_t)pBufferEnd & (CACHE_LINE_ALIGNMENT_MASK)) != 0u)
    {
        pBufferEnd += ALIGN_END_ADDRESS;
    }
    flush_data_buffer(pBufferStart, pBufferEnd, (int)invalidate);
#if defined (__ADSPGCC__)
#pragma GCC diagnostic pop
#endif
#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */
}

/* ==================  Debug mode function =============================== */
#if defined (ADI_DEBUG)
static ADI_EMSI_RESULT  ValidateHandle(ADI_EMSI_INFO *pInfo)
{
    uint32_t i;
    if(pInfo != NULL)
    {
        for (i = 0u; i < ADI_EMSI_MAX_INSTANCES; i++)
        {
            if ((pInfo ==  &gEMSIDevInfo[i]))
            {
                if(pInfo->pDevice != NULL)
                {
                    return ADI_EMSI_SUCCESS;
                }
            }
        }
    }
    return ADI_EMSI_INVALID_HANDLE;
}
#endif

#if defined (__ADSPGCC__)
#pragma GCC diagnostic pop
#endif
#ifdef _MISRA_RULES
#pragma diag(pop)
#endif
#endif /* __ADI_EMSI_V1_C__ */

/** @}*/
