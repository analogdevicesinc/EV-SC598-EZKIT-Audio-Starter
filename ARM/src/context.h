/**
 * Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */
#ifndef _context_h
#define _context_h

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "spi_simple.h"
#include "twi_simple.h"
#include "uart_simple.h"
#include "uart_simple_cdc.h"
#include "sport_simple.h"
#include "sdcard_simple.h"
#include "flash.h"
#include "shell.h"
#include "pa_ringbuffer.h"
#include "uac2_soundcard.h"
#include "sae.h"
#include "ipc.h"
#include "a2b_to_sport_cfg.h"
#include "wav_file.h"
#include "clock_domain_defs.h"
#include "spiffs.h"
#include "route.h"
#include "uac2.h"
#include "rtp_stream.h"
#include "vban_stream.h"

#include "lwip_adi_ether_netif.h"
#include "lwip/netif.h"

/* Misc defines */
#define UNUSED(expr) do { (void)(expr); } while (0)

/* SoM Carrier Versions */
#define SOMCRR_REV_A   100
#define SOMCRR_REV_D   400

/*
 * WARNING: Do not change SYSTEM_AUDIO_TYPE from int32_t
 *
 */
#define SYSTEM_MCLK_RATE               (24576000)
#define SYSTEM_SAMPLE_RATE             (48000)
#define SYSTEM_BLOCK_SIZE              (64)
#define SYSTEM_AUDIO_TYPE              int32_t
#define SYSTEM_MAX_CHANNELS            (32)
#define WAV_MAX_CHANNELS               (64)

#define USB_DEFAULT_IN_AUDIO_CHANNELS  (16)       /* USB IN endpoint audio */
#define USB_DEFAULT_OUT_AUDIO_CHANNELS (16)       /* USB OUT endpoint audio */
#define USB_DEFAULT_WORD_SIZE          (sizeof(int16_t))
#define USB_TIMER                      (0)
#define USB_VENDOR_ID                  (0x064b)  /* Analog Devices Vendor ID */
#define USB_PRODUCT_ID                 (0x0007)  /* CLD UAC+CDC */
#define USB_MFG_STRING                 "Analog Devices, Inc."
#define USB_PRODUCT_STRING             "EV-SC598 (16x16x16bit)"
#define USB_SERIAL_NUMBER_STRING       NULL
#define USB_OUT_RING_BUFF_FRAMES       (6 * SYSTEM_BLOCK_SIZE)
#define USB_IN_RING_BUFF_FRAMES        (6 * SYSTEM_BLOCK_SIZE)
#define USB_OUT_RING_BUFF_FILL         (USB_OUT_RING_BUFF_FRAMES / 2)
#define USB_IN_RING_BUFF_FILL          (USB_IN_RING_BUFF_FRAMES / 2)

#define WAV_RING_BUF_SAMPLES           (128 * 1024)
#define RTP_RING_BUF_SAMPLES           (128 * 1024)
#define VBAN_RING_BUF_SAMPLES          (128 * 1024)
#define FILE_RING_BUF_SAMPLES          (128 * 1024)

#define WAV_MAX_CHANNELS               (64)
#define VU_MAX_CHANNELS                (64)

#define ADC_AUDIO_CHANNELS             (4)
#define ADC_DMA_CHANNELS               (8)
#define DAC_AUDIO_CHANNELS             (12)
#define DAC_DMA_CHANNELS               (16)

#define SPDIF_AUDIO_CHANNELS           (2)
#define SPDIF_DMA_CHANNELS             (2)

#define A2B_AUDIO_CHANNELS             (32)
#define A2B_DMA_CHANNELS               (32)

#define SHARC0_AUDIO_IN_CHANNELS    (SYSTEM_MAX_CHANNELS)
#define SHARC0_AUDIO_OUT_CHANNELS   (SYSTEM_MAX_CHANNELS)
#define SHARC1_AUDIO_IN_CHANNELS    (SYSTEM_MAX_CHANNELS)
#define SHARC1_AUDIO_OUT_CHANNELS   (SYSTEM_MAX_CHANNELS)

/*
 * The A2B I2C addresses are latched at power up and cannot be changed
 * at runtime. Use the 'a2b' command to set the I2C address at runtime to
 * match the HW.
 *
 * By default AD2428MINI boards have address 0x68 and AD2433MINI boards have
 * address 0x6A
 *
 * It is not possible to have two AD2433MINI modules attached to J10 and
 * J11 simultaneously.  Two AD2428MINI modules can be attached as long as
 * they are configured for different I2C addresses via P4.
 *
 * With the default settings, AD242x modules should be plugged into
 * J10 and AD243x modules should be plugged into J11.
 *
 * WARNING: AD2433MINI modules cannot be attached to J11 when using a SC598 SoM
 * due to a conflict on the SC598 pin PB_15. This pin is shared between the
 * onboard SoM eMMC and the SoM carrier board A2B2_RESET (AD2433MINI HW_RST).
 * When using an SC598 SoM and an AD2433MINI, plug the AD2433MINI module
 * into J10 (A2B) and invert the default I2C addresses below.
 *
 */
#define DEFAULT_A2B_I2C_ADDR        (0x68)
#define DEFAULT_A2B2_I2C_ADDR       (0x6A)

#define SPIFFS_VOL_NAME             "sf:"
#define SDCARD_VOL_NAME             "sd:"
#define EMMC_VOL_NAME               "mmc:"

typedef enum A2B_BUS_MODE {
    A2B_BUS_MODE_UNKNOWN = 0,
    A2B_BUS_MODE_MAIN,
    A2B_BUS_MODE_SUB
} A2B_BUS_MODE;

/*
 * FS = Frame Sync, BCLK = Bit Clock
 *
 * TDM 16 x 32-bit
 * Rising edge FS (pulse high)
 * Early FS (data MSb delayed 1 BCLK)
 * Assert FS and data on BCLK rising edge
 * Sample FS and data on BCLK falling edge
 *
 */
#define SYSTEM_I2SGCFG                 (0x44)
#define SYSTEM_I2SCFG                  (0xF7)

/* Audio routing */
#define MAX_AUDIO_ROUTES               (20)

/* Ethernet defines */
#define DEFAULT_ETH0_IP_ADDR       "169.254.0.0"
#define DEFAULT_ETH0_NETMASK       "255.255.0.0"
#define DEFAULT_ETH0_GW_ADDR       "0.0.0.0"
#define DEFAULT_ETH0_BASE_HOSTNAME "sc598"
#define DEFAULT_ETH0_STATIC_IP     false
#define DEFAULT_ETH0_DEFAULT_IFACE true

#define DEFAULT_ETH1_IP_ADDR       "169.254.0.0"
#define DEFAULT_ETH1_NETMASK       "255.255.0.0"
#define DEFAULT_ETH1_GW_ADDR       "0.0.0.0"
#define DEFAULT_ETH1_BASE_HOSTNAME "sc598"
#define DEFAULT_ETH1_STATIC_IP     false
#define DEFAULT_ETH1_DEFAULT_IFACE false

typedef struct ETH_CFG {
    ADI_ETHER_EMAC_PORT port;
    char *ip_addr;
    char *gateway_addr;
    char *netmask;
    bool static_ip;
    bool default_iface;
    char *base_hostname;
} ETH_CFG;

typedef struct ETH {
    struct netif netif;
    adi_ether_netif *adi_ether;
    uint8_t macaddr[6];
    char hostname[32];
} ETH;

/* System Configuration */
typedef struct APP_CFG {
    ETH_CFG eth0;
    ETH_CFG eth1;
    int usbOutChannels;
    int usbInChannels;
    int usbWordSize;
    unsigned a2bI2CAddr;
    unsigned a2b2I2CAddr;
} APP_CFG;

/* Task notification values */
enum {
    UAC2_TASK_NO_ACTION,
    UAC2_TASK_AUDIO_DATA_READY,
};

/* USB Audio OUT (Rx) endpoint stats */
typedef struct _USB_AUDIO_RX_STATS {
    uint32_t usbRxOverRun;
    uint32_t usbRxUnderRun;
    UAC2_ENDPOINT_STATS ep;
} USB_AUDIO_RX_STATS;

/* USB Audio IN (Tx) endpoint stats */
typedef struct _USB_AUDIO_TX_STATS {
    uint32_t usbTxOverRun;
    uint32_t usbTxUnderRun;
    UAC2_ENDPOINT_STATS ep;
} USB_AUDIO_TX_STATS;

typedef struct _USB_AUDIO_STATS {
    USB_AUDIO_RX_STATS rx;
    USB_AUDIO_TX_STATS tx;
} USB_AUDIO_STATS;

/* WAV src stats */
typedef struct _WAV_SRC_STATS {
    unsigned underrun;
    unsigned slowReads;
} WAV_SRC_STATS;

/* WAV sink stats */
typedef struct _WAV_SINK_STATS {
    unsigned overrun;
    unsigned slowWrites;
} WAV_SINK_STATS;

/*
 * The main application context.  Used as a container to carry a
 * variety of useful pointers, handles, etc., between various
 * modules and subsystems.
 */
typedef struct _APP_CONTEXT {

    /* SoM Carrier Version */
    int SoMCRRVersion;

    /* Core clock frequency */
    uint32_t cclk;

    /* Device handles */
    sUART *stdioHandle;
    sSPI *spi2Handle;
    sSPIPeriph *spiFlashHandle;
    FLASH_INFO *flashHandle;
    sTWI *twi0Handle;
    sTWI *twi1Handle;
    sTWI *twi2Handle;
    sTWI *si5356Handle;
    sTWI *softSwitchHandle;
    sTWI *adau1962TwiHandle;
    sTWI *adau1979TwiHandle;
    sTWI *a2bTwiHandle;
    sSPORT *dacSportOutHandle;
    sSPORT *adcSportInHandle;
    sSPORT *spdifSportOutHandle;
    sSPORT *spdifSportInHandle;
    sSPORT *a2bSportOutHandle;
    sSPORT *a2bSportInHandle;
    sSPORT *a2b2SportOutHandle;
    sSPORT *a2b2SportInHandle;
    spiffs *spiffsHandle;
    sSDCARD *sdcardHandle;

    /* SHARC status */
    volatile bool sharc0Ready;
    volatile bool sharc1Ready;

    /* Shell context */
    SHELL_CONTEXT shell;

    /* SHARC Audio Engine context */
    SAE_CONTEXT *saeContext;

    /* UAC2 related variables and settings
     *
     * Rx/Tx are from the target's perspective.
     * Rx = UAC2 OUT, Tx = UAC2 IN
     */
    PaUtilRingBuffer *uac2OutRx;
    void *uac2OutRxData;
    PaUtilRingBuffer *uac2InTx;
    void *uac2InTxData;
    bool uac2RxEnabled;
    bool uac2TxEnabled;
    USB_AUDIO_STATS uac2stats;
    UAC2_APP_CONFIG uac2cfg;

    /* Task handles (used in 'stacks' command) */
    TaskHandle_t houseKeepingTaskHandle;
    TaskHandle_t pollStorageTaskHandle;
    TaskHandle_t pushButtonTaskHandle;
    TaskHandle_t uac2TaskHandle;
    TaskHandle_t startupTaskHandle;
    TaskHandle_t idleTaskHandle;
    TaskHandle_t wavSrcTaskHandle;
    TaskHandle_t wavSinkTaskHandle;
    TaskHandle_t a2bSlaveTaskHandle;
    TaskHandle_t a2bIrqTaskHandle;
    TaskHandle_t telnetTaskHandle;
    TaskHandle_t vuTaskHandle;
    TaskHandle_t rtpRxTaskHandle;
    TaskHandle_t rtpTxTaskHandle;
    TaskHandle_t vbanRxTaskHandle;
    TaskHandle_t vbanTxTaskHandle;

    /* A2B XML init items */
    void *a2bInitSequence;
    uint32_t a2bIinitLength;

    /* Audio ping/pong buffer pointers */
    void *codecAudioIn[2];
    void *codecAudioOut[2];
    void *spdifAudioIn[2];
    void *spdifAudioOut[2];
    void *a2bAudioIn[2];
    void *a2bAudioOut[2];
    void *sharc0AudioIn[2];
    void *sharc0AudioOut[2];
    void *sharc1AudioIn[2];
    void *sharc1AudioOut[2];
    void *a2b2AudioIn[2];
    void *a2b2AudioOut[2];

    /* Audio ping/pong buffer lengths */
    unsigned codecAudioInLen;
    unsigned codecAudioOutLen;
    unsigned spdifAudioInLen;
    unsigned spdifAudioOutLen;
    unsigned a2bAudioInLen;
    unsigned a2bAudioOutLen;
    unsigned sharc0AudioInLen;
    unsigned sharc0AudioOutLen;
    unsigned sharc1AudioInLen;
    unsigned sharc1AudioOutLen;
    unsigned a2b2AudioInLen;
    unsigned a2b2AudioOutLen;

    /* SAE buffer pointers */
    SAE_MSG_BUFFER *sharc0MsgIn[2];
    SAE_MSG_BUFFER *sharc0MsgOut[2];
    SAE_MSG_BUFFER *sharc1MsgIn[2];
    SAE_MSG_BUFFER *sharc1MsgOut[2];

    /* SDCARD / eMMC */
    char *cardName;
    bool sdPresent;
    bool sdRemount;

    /* Audio routing table */
    ROUTE_INFO *routingTable;

    /* APP config */
    APP_CFG cfg;

    /* Current time in mS */
    uint64_t now;

    /* SHARC Cycles */
    uint32_t sharc0Cycles[CLOCK_DOMAIN_MAX];
    uint32_t sharc1Cycles[CLOCK_DOMAIN_MAX];

    /* WAV file related variables and settings */
    WAV_FILE wavSrc;
    WAV_FILE wavSink;
    PaUtilRingBuffer *wavSrcRB;
    void *wavSrcRBData;
    PaUtilRingBuffer *wavSinkRB;
    void *wavSinkRBData;
    WAV_SRC_STATS wavSrcStats;
    WAV_SINK_STATS wavSinkStats;

    /* RTP related variables and settings */
    RTP_STREAM rtpRx;
    RTP_STREAM rtpTx;
    PaUtilRingBuffer *rtpRxRB;
    void *rtpRxRBData;
    PaUtilRingBuffer *rtpTxRB;
    void *rtpTxRBData;

    /* VBAN related variables and settings */
    VBAN_STREAM vbanRx;
    VBAN_STREAM vbanTx;
    PaUtilRingBuffer *vbanRxRB;
    void *vbanRxRBData;
    PaUtilRingBuffer *vbanTxRB;
    void *vbanTxRBData;

    /* A2B mode */
    A2B_BUS_MODE a2bmode;
    A2B_TO_SPORT_CFG_XCVR a2bxcvr;
    bool a2bPresent;
    bool a2bSlaveActive;
    bool discoverCmdStatus;
    bool a2bIrqDisable;
    unsigned a2bOutChannels;
    unsigned a2bInChannels;

    /* A2B2 mode */
    A2B_BUS_MODE a2b2mode;
    A2B_TO_SPORT_CFG_XCVR a2b2xcvr;
    bool a2b2Present;
    bool a2b2SlaveActive;
    unsigned a2b2OutChannels;
    unsigned a2b2InChannels;

    /* Clock domain management */
    uint32_t clockDomainMask[CLOCK_DOMAIN_MAX];
    uint32_t clockDomainActive[CLOCK_DOMAIN_MAX];

    /* Ethernet Network interface */
    ETH eth[2];

} APP_CONTEXT;

/*
 * Make the application context global for convenience
 */
extern APP_CONTEXT mainAppContext;

#endif
