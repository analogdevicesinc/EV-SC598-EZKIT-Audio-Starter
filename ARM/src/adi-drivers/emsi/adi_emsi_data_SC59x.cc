/******************************************************************************
 * @file:    adi_emsi_data_SC59x.c
 * @brief:   Local data structure initialization related to EMSI driver.
 * @details   Data structure for eMSI register base addresses assignment.
 * @version: $Revision: $
 * @date:    $Date: 2021-07-29$
 *****************************************************************************

 Copyright(c) 2022 Analog Devices, Inc. All Rights Reserved.

 This software is proprietary.  By using this software you agree
 to the terms of the associated Analog Devices License Agreement.

 *********************************************************************************/


#ifndef __ADI_EMSI_DATA_SC59x_C__
#define __ADI_EMSI_DATA_SC59x_C__

#if (ADI_EMSI_NUM_DEVICES < 1u) || (ADI_EMSI_NUM_DEVICES > 1u)
#error "Invalid device number"
#endif

#include "adi_emsi_config_SC59x.h"


/*! \var gEMSIDevInfo[ADI_EMSI_MAX_INSTANCES]
 * Device info structure to which has a set of pointers which point to the base address of
 * eMSI registers and have the Interrupt Id's of eMSI peripheral.
 **/
static ADI_EMSI_INFO gEMSIDevInfo[ADI_EMSI_MAX_INSTANCES]   =
{
		{

				/* EMSI0 */
				(ADI_EMSI_TypeDef *)ADI_EMSI0_BASE,                /* EMSI0 register base address */
				(uint32_t)INTR_EMSI0_STAT,                         /* EMSI0 status interrupt ID */
				(uint32_t)INTR_EMSI0_WAKEUP,                       /* EMSI0 wakeup interrupt ID */
				NULL                                               /* Pointer to EMSI0 device instance data */

		}
};

#endif /* __ADI_EMSI_DATA_SC59x_C__ */
