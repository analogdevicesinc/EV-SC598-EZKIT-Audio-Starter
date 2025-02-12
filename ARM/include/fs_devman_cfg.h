/**
 * Copyright (c) 2021 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#ifndef _fs_devman_cfg_h
#define _fs_devman_cfg_h

#include "umm_malloc.h"
#define FS_DEVMAN_CALLOC  umm_calloc
#define FS_DEVMAN_FREE    umm_free

#define FS_DEVMAN_MAX_DEVICES 5

#define FS_DEVMAN_ENABLE_FATFS
#define FS_DEVMAN_ENABLE_SPIFFS

#endif
