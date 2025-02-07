/******************************************************************************
 * @file:    adi_emsi_config_SC59x.h
 * @brief:   EMSI driver static configuration
 * @details  Header File which has the static configuration for EMSI instances.
 * @version: $Revision:  $
 * @date:    $Date: 2021-07-29 $
 *****************************************************************************

 Copyright (c) 2022 Analog Devices.  All Rights Reserved.

 This software is proprietary.  By using this software you agree
 to the terms of the associated Analog Devices License Agreement.

 *******************************************************************************/


/** @addtogroup EMSI_Driver EMSI Device Driver
 *  @{
 */

/** @addtogroup EMSI_Driver_Static_Configuration EMSI Device Driver Static Configuration
 *  @{


  Default Static Configuration for EMSI0:
  The default configuration for EMSI:
        EMSI Clock Frequency :  125 kHz

  Specifies the EMSI0_CLK register Divisor\n
    ADI_EMSI0_CLOCK_CLK_CTL_FREQ_SEL  200u

 */

#ifndef __ADI_EMSI_CONFIG_SC59x_H__
#define __ADI_EMSI_CONFIG_SC59x_H__

#include <sys/platform.h>
#include <stdint.h>

/**************************************************************************************************************/
/*! Specifies the number of active EMSI devices accordingly the memory allotted will be optimized */
#if defined(__ADSPSC598_FAMILY__)
#define ADI_EMSI_NUM_DEVICES				1ul
#else
#error "Unknown processor family"
#endif
/**************************************************************************************************************/

/**************************************************************************************************************/
/*! Select/deselect EMSI0 Instance */
#define ADI_EMSI0_INSTANCE					1ul
/**************************************************************************************************************/
/**************************************************************************************************************/
#define MHZTOHZ                             1000000u

/*! EMSI0 CLKIN (Default 50 MHz) for calculation of 74 clock cycles delay*/
#define ADI_EMSI0_CLKIN   					(50u*MHZTOHZ)

/*! System CLKIN (Default 25 MHz) for calculation of 74 clock cycles delay*/
#define ADI_EMSI0_SYS_CLKIN   				(25u*MHZTOHZ)

/*! Specifies EMSI0 Clock Divider Value for card identification resulting
 * Frequency should be less than 400 kHz. for e.g. (ADI_EMSI0_CLKIN/200*2) =
 * 125 kHz*/
#define ADI_EMSI0_CLOCK_DIVISOR   			200u

/*! Timer instance used in EMSI driver for calculation of 74 clock cycles delay*/
#define ADI_EMSI0_TMR_NUM					15u

/*! Number of descriptors for ADMA2 Open ended transfers. Each descriptors is of 64 MB (MegaBytes)
 * Maximum value which can be passed here is 63. For example If user submitted number of
 * descriptors as 63 then user should take care that transmit and receive buffer declared should be of 63*64MB
 * = 4032 MB each (User should take care enough on board memory is present).
 *  For predefined transfers the buffer sizes should be as per data transfer size (it is advised to keep
 *  this parameter as 63 while predefined transfers).
 */
#define ADI_EMSI0_CFG_ADMA2_DESCRIPTORS		63u

/*! Number of tasks for ADMA3 based transfers. Each Task is of maximum size (32 MB -512 bytes)
 * (for block size of 512 bytes). That is one task can be used for maximum (32 MB -512 bytes) of
 * data transfer. The Maximum value can be 30 tasks.
 */
#define ADI_EMSI0_CFG_ADAM3_TASKS			5u

/*! Boot acknowledgment configuration, when application use is set to #ADI_EMSI_APP_USE_BOOTING_OPERATION
 * in adi_emsi_SetAppUse()
 *
 * 0u(False) - No Boot acknowledgment is expected from card side while booting operation.
 * 1u(True)  - Boot acknowledgment is expected from card side while booting operation.*/
#define ADI_EMSI0_CFG_BOOTACK				0u

/*! Usage of External socket, when card is inserted into external socket rather than using
 * on board eMMC device then usage of external socket is set to true in adi_emsi_SetCardConnector()
 * API.
 *
 * 0u(False) - On board eMMC device is used.
 * 1u(True)  - Card is inserted in external socket.*/
#define ADI_EMSI0_CFG_EXTSOCKETUSE			0u

/*! Usage of preset register, when preset register usage is enabled then clock divider values are
 * set as per values in corresponding preset value register.
 *
 * 0u(False) - Disable preset value usage(0x00000000u)
 * 1u(True)  - Enable preset value usage(0x00008000u).*/
#define ADI_EMSI0_CFG_PRESETREGUSE			0x00000000u

/*! Response check by controller, EMSI controller checks error in R1 responses to avoid overhead by
 * the driver for improving throughput performance. User should ensure that command sent after enabling
 * this feature should have R1 response.
 *
 * 0u(False) - Response check by the controller disabled.
 * 1u(True)  - Response check by the controller enabled.*/
#define ADI_EMSI0_CFG_CONTROLLERRESPCHK		0u

/*! Timeout value for clock sync and send command API. By default it is set to maximum 0xFFFFFFFF value.
 * This value can adjusted as per delay needed but the delay should be maximum of 150 ms or maximum
 * time expected for response from eMMC device or SD card (usually 64 eMSI bus clock cycles).*/
#define ADI_EMSI0_CFG_TIMEOUT				1000000u

/**************************************************************************************************************/
#endif /* __ADI_EMSI_CONFIG_SC59x_H__ */

/**@}*/

/**@}*/

