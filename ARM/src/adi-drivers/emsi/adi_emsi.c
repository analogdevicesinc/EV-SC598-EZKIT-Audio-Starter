/* $Id: adi_emsi.c$ */
/*********************************************************************************
Copyright(c) 2022 Analog Devices, Inc. All Rights Reserved.

This software is proprietary.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

 *********************************************************************************/
/*!
@file      adi_emsi.c

@brief     Global EMSI driver file.

@details   This a global file which includes a specific file based on the processor family.
           This included file will be  containing  EMSI device  driver functions.

$Revision:  $
$Date: 2021-07-29 $
 */

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_5_1:"Identifiers shall not rely on the significance of more than 31 characters")
#pragma diag(suppress:misra_rule_8_5:"Allow definitions in included C file")
#pragma diag(suppress:misra_rule_19_15:"No multi-inclusion guard in C files")
#endif /* _MISRA_RULES */

#include <sys/platform.h>

#if defined(__ADSPSC598_FAMILY__)
#include "adi_emsi_v1.cc"
#endif

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */
