/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include "context.h"
#include "twi_simple.h"
#include "ss_init.h"

#pragma pack(push)
#pragma pack(1)
typedef struct {
    uint8_t reg;
    uint8_t value;
} SWITCH_CONFIG;
#pragma pack(pop)

#define SOFT_SWITCH_SOM_U24_I2C_ADDR   0x20
#define PORTA 0x12u
#define PORTB 0x13u

/* switch 1 register settings */
static SWITCH_CONFIG somU24Config[] =
{

 /*
       U24 Port A                                  U24 Port B
  7--------------- NOT USED             |       7--------------- NOT USED
  | 6------------- ~UART0_FLOW_EN       |       | 6------------- NOT USED
  | | 5----------- ~UART0_EN            |       | | 5----------- NOT USED
  | | | 4--------- ~SPID2_D3_EN         |       | | | 4--------- NOT USED
  | | | | 3------- ~SPI2FLASH_CS_EN     |       | | | | 3------- NOT USED
  | | | | | 2----- DS3                  |       | | | | | 2----- NOT USED
  | | | | | | 1--- DS2                  |       | | | | | | 1--- ~EMMC_SOM_EN
  | | | | | | | 0- DS1                  |       | | | | | | | 0- ~EMMC_EN
  | | | | | | | |                       |       | | | | | | | |
  X N Y Y Y N N N                       |       X X X X X X Y N  ( Active Y or N )
  0 1 0 0 0 1 1 1                       |       0 0 0 0 0 0 0 1  ( value being set )
*/
  { PORTA, 0x47u },                             { PORTB, 0x01u },

  { 0x0u, 0x00u },  /* Set IODIRA direction (all output) */
  { 0x1u, 0x00u },  /* Set IODIRB direction (all output) */
  { 0xCu, 0xFFu },  /* Set GPPUA direction (all pull-ups enabled) */
  { 0xDu, 0xFFu },  /* Set GPPUB direction (all pull-ups enabled) */
};

void ss_init_sc598_som(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT twiResult;
    uint8_t configLen;
    uint8_t i;

    /* Program carrier U24 soft switches */
    configLen = sizeof(somU24Config) / sizeof(somU24Config[0]);
    for (i = 0; i < configLen; i++) {
        twiResult = twi_write(context->softSwitchHandle, SOFT_SWITCH_SOM_U24_I2C_ADDR,
            (uint8_t *)&somU24Config[i], sizeof(somU24Config[i]));
    }

}

typedef struct _SS_SOM_PIN {
    int pinId;
    uint8_t port;
    uint8_t bitp;
} SS_SOM_PIN;

static SS_SOM_PIN somPins[] = {
    { .pinId = SS_PIN_ID_nUART0_FLOW_EN, .port = PORTA, .bitp = 6 },
    { .pinId = SS_PIN_ID_nUART0_EN, .port = PORTA, .bitp = 5 },
    { .pinId = SS_PIN_ID_nSPID2_D3_EN, .port = PORTA, .bitp = 4 },
    { .pinId = SS_PIN_ID_nSPI2FLASH_CS_EN, .port = PORTA, .bitp = 3 },
    { .pinId = SS_PIN_ID_DS3, .port = PORTA, .bitp = 2 },
    { .pinId = SS_PIN_ID_DS2, .port = PORTA, .bitp = 1 },
    { .pinId = SS_PIN_ID_DS1, .port = PORTA, .bitp = 0},
    { .pinId = SS_PIN_ID_EMMC_SOM_EN, .port = PORTB, .bitp = 1 },
    { .pinId = SS_PIN_ID_EMMC_EN, .port = PORTB, .bitp = 0},
};

static SS_SOM_PIN *findPin(int pinId)
{
    SS_SOM_PIN *somPin;

    somPin = somPins;
    while (somPin->pinId != SS_PIN_ID_MAX) {
        if (somPin->pinId == pinId) {
            break;
        }
        somPin++;
    }
    if (somPin->pinId == SS_PIN_ID_MAX) {
        somPin = NULL;
    }
    return(somPin);
}

bool ss_get_sc598_som(APP_CONTEXT *context, int pinId, bool *value)
{
    TWI_SIMPLE_RESULT twiResult;
    SS_SOM_PIN *somPin;
    uint8_t reg;
    uint8_t val;

    somPin = findPin(pinId);
    if (somPin == NULL) {
        return(false);
    }

    reg = somPin->port;
    twiResult = twi_writeRead(context->softSwitchHandle,
        SOFT_SWITCH_SOM_U24_I2C_ADDR, &reg, 1, &val, 1);
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        return(false);
    }

    if (value) {
        *value = (val & (1 << somPin->bitp));
    }

    return(true);
}

bool ss_set_sc598_som(APP_CONTEXT *context, int pinId, bool value)
{
    TWI_SIMPLE_RESULT twiResult;
    SS_SOM_PIN *somPin;
    SWITCH_CONFIG sw;

    somPin = findPin(pinId);
    if (somPin == NULL) {
        return(false);
    }

    sw.reg = somPin->port;
    twiResult = twi_writeRead(context->softSwitchHandle,
        SOFT_SWITCH_SOM_U24_I2C_ADDR, &sw.reg, 1, &sw.value, 1);
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        return(false);
    }

    if (value) {
        sw.value |= (1 << somPin->bitp);
    } else {
        sw.value &= ~(1 << somPin->bitp);
    }

    twiResult = twi_write(context->softSwitchHandle,
        SOFT_SWITCH_SOM_U24_I2C_ADDR, (uint8_t *)&sw, sizeof(sw));
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        return(false);
    }

    return(true);
}
