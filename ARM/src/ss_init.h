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

#ifndef _init_ss_h
#define _init_ss_h

#include "context.h"
#include "ss.h"

void ss_init(APP_CONTEXT *context);
void ss_deinit(APP_CONTEXT *context);
bool ss_get(APP_CONTEXT *context, int pinId, bool *value);
bool ss_set(APP_CONTEXT *context, int pinId, bool value);

#endif
