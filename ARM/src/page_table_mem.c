/*******************************************************************************

Copyright(c) 2014-2021 Analog Devices, Inc. All Rights Reserved.

This software is proprietary.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*******************************************************************************/
/*!
* @file      page_table_mem.c
*
* @brief     Page table used by Memory Management Unit (MMU)
*
* @details
*            This contains the raw memory array that is used to hold the actual
*            page tables used by the hardware.
*/

#include <stdint.h>

#include <runtime/mmu/adi_mmu.h>

#if defined(__ADSPCORTEXA55__)
  #define TABLE_SIZE_IN_KBYTES 40
  #define TABLE_ALIGN_IN_KBYTES 4
#elif defined(__ADSPSC573_FAMILY__) || defined(__ADSPSC594_FAMILY__)
  #if __NUM_SHARC_CORES__ > 1
    #define TABLE_SIZE_IN_KBYTES 24
  #else
    #define TABLE_SIZE_IN_KBYTES 22
  #endif
  #define TABLE_ALIGN_IN_KBYTES  16
#else
  #if __NUM_SHARC_CORES__ > 1
    #define TABLE_SIZE_IN_KBYTES 30
  #else
    #define TABLE_SIZE_IN_KBYTES 26
  #endif
  #define TABLE_ALIGN_IN_KBYTES  16
#endif

#define KBYTES(x) ((x)*1024)

uint8_t _adi_mmu_tableMemory[KBYTES(TABLE_SIZE_IN_KBYTES)] __attribute__ ((aligned (KBYTES(TABLE_ALIGN_IN_KBYTES))));
#if defined(__ADSPCORTEXA55__)
const uint32_t _adi_mmu_tableMemSize = sizeof(_adi_mmu_tableMemory);
#else
const size_t _adi_mmu_tableMemSize = sizeof(_adi_mmu_tableMemory);
#endif

