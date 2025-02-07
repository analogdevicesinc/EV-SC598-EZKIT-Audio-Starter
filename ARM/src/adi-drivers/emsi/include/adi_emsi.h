/******************************************************************************
 * @file    adi_emsi.h
 * @brief   EMSI Device driver definitions and APIs
 * @details Header File for the EMSI driver API functions and definitions
 * @version $Revision:
 * @date    $Date:2021-07-29$
 *****************************************************************************

Copyright(c) 2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

 *********************************************************************************/

/*! @addtogroup EMSI_Driver  EMSI Device Driver
 *  @{
 */
#ifndef _ADI_EMSI_H_
#define _ADI_EMSI_H_


#if defined(__ADSPSC598_FAMILY__)
#include "adi_emsi_v1.h"
#else
#error This processor is not supported.
#endif



#endif /* __ADI_EMSI_H__ */
/**@}*/
