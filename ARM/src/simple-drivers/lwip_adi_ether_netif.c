/**
 * Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <machine/endian.h>

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/etharp.h"

#include "lwip_adi_ether_netif.h"

/* Make sure ETH_PAD_SIZE is set to 2 */
#if !defined(ETH_PAD_SIZE) || (ETH_PAD_SIZE != 0)
#error ETH_PAD_SIZE must be 0
#endif

#ifndef ADI_ETHER_MAX_NETIFS
#define ADI_ETHER_MAX_NETIFS 2
#endif

#ifndef UNUSED
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

/* Network interface name */
#define IFNAME0 'a'
#define IFNAME1 'd'

/* This is a continuation of the hack in lwIP sys_arch.c FreeRTOS
 * porting layer.
 */
extern long xInsideISR;

enum WORKER_TODO {
    WORKER_UNKNOWN = 0,
    WORKER_LINK_UP,
    WORKER_LINK_DOWN,
    WORKER_PKT_RX,
    WORKER_PKT_TX
};

/*************************************************************************
 * Private APIs
 ************************************************************************/
static void
adi_ether_netif_reset_rx_desc(ADI_EMAC_DMA_DESC *rxDesc,
    LWIP_ADI_ETHER_PACKET_DATA *pktData)
{
    rxDesc->RxDescRead.Buffer1 = (uintptr_t)pktData;
    rxDesc->RxDescRead.Status =
        ADI_EMAC_RX_DESC_READ_OWN | ADI_EMAC_RX_DESC_READ_BUF1V |
        ADI_EMAC_RX_DESC_READ_IOC;
}

/* This function initializes Rx DMA descriptors and queue */
static void
adi_ether_netif_init_rx_desc(adi_ether_netif *adi_ether)
{
    LWIP_ADI_ETHER_QUEUE *rxPktQEntry;
    int i;

    for (i = 0; i < ADI_ETHER_NUM_RX_BUFFS; i++) {
        rxPktQEntry = &adi_ether->pktRxQueue[i];
        rxPktQEntry->data = &adi_ether->rxPktData[i];
        rxPktQEntry->desc = &adi_ether->rxDmaDesc[i];
        adi_ether_netif_reset_rx_desc(rxPktQEntry->desc, rxPktQEntry->data);
    }
}

/* This function initializes Tx DMA descriptors */
static void
adi_ether_netif_init_tx_desc(adi_ether_netif *adi_ether)
{
    LWIP_ADI_ETHER_QUEUE *txPktQEntry;
    int i;

    for (i = 0; i < ADI_ETHER_NUM_TX_BUFFS; i++) {
        txPktQEntry = &adi_ether->pktTxQueue[i];
        txPktQEntry->data = &adi_ether->txPktData[i];
        txPktQEntry->desc = &adi_ether->txDmaDesc[i];
        txPktQEntry->desc->TxDescRead.Buffer1 = (uintptr_t)txPktQEntry->data;
        txPktQEntry->desc->TxDescRead.BufferLen = sizeof(txPktQEntry->data->data);
        txPktQEntry->desc->TxDescRead.FrameLenCtrl = 0;
    }
}

/*
 * Queue a frame received from ptp or lwIP for transmit.
 */
static err_t
adi_ether_netif_tx_frame(struct netif *netif, struct pbuf *p, bool ptpCB)
{
    adi_ether_netif *adi_ether = netif->state;
    LWIP_ADI_ETHER_QUEUE *txPktQEntry;
    ADI_EMAC_DMA_DESC *txDesc;
    uint16_t txPktNext;
    uint16_t len;

    /* Make sure to never exceed the TX buffer pool */
    txPktNext = adi_ether->txPktHead + 1;
    if (txPktNext == ADI_ETHER_NUM_TX_BUFFS) {
        txPktNext = 0;
    }
    if (txPktNext == adi_ether->txPktTail) {
        LINK_STATS_INC(link.drop);
        return(ERR_OK);
    }

    /* Grab the next TX queue entry */
    txPktQEntry = &adi_ether->pktTxQueue[adi_ether->txPktHead];

    /* Insert the lwIP payload into the frame */
    len = pbuf_copy_partial(p,
        txPktQEntry->data->data, ETHERNET_MAX_SIZE, 0);

    /* Request a timestamp for PTP packets */
    if (ptpCB) {
        // TODO
    }

    /* Send it out if the link is up */
    if (adi_ether->linkUp) {
        txDesc = txPktQEntry->desc;
        txDesc->TxDescRead.Buffer1 = (uintptr_t)txPktQEntry->data->data;
        txDesc->TxDescRead.BufferLen = ADI_EMAC_TX_DESC_READ_IOC | len;
        txDesc->TxDescRead.FrameLenCtrl =
            ADI_EMAC_TX_DESC_READ_FD | ADI_EMAC_TX_DESC_READ_LD |
            ADI_EMAC_TX_DESC_READ_CIC_TCP_PSEUDO_CS | len;
        adi_emac_ReSubmitDescriptorTx(txPktQEntry->desc, len, 0, true);
        adi_emac_TxDescriptorPoll(adi_ether->hEthernet, 0);
        adi_ether->txPktHead = txPktNext;
        LINK_STATS_INC(link.xmit);
    } else {
        LINK_STATS_INC(link.drop);
    }

    return(ERR_OK);
}

/*
 * Queue a frame received from lwIP for transmit.
 */
static err_t
adi_ether_netif_lwip_tx_frame(struct netif *netif, struct pbuf *p)
{
    adi_ether_netif *adi_ether = netif->state;
    err_t err;

    sys_mutex_lock(&adi_ether->writeLock);
    err = adi_ether_netif_tx_frame(netif, p, false);
    sys_mutex_unlock(&adi_ether->writeLock);

    return (err);
}

/*
 * Queue a ptp frame for transmit.
 */
err_t
adi_ether_netif_ptp_tx_frame(struct netif *netif, uint8_t *srcMacAddr,
    uint8_t *dstMacAddr, uint16_t etherType, uint8_t *data, uint16_t len,
    bool txCallback)
{
    adi_ether_netif *adi_ether = netif->state;
    struct pbuf p[5];
    uint8_t pad[2];
    uint8_t eType[2];
    err_t err;

    /* 2 byte alignment padding */
    p[0].len = 2;
    p[0].payload = pad;
    p[0].next = &p[1];

    /* Dest MAC */
    p[1].len = 6;
    p[1].payload = dstMacAddr;
    p[1].next = &p[2];

    /* Src MAC */
    p[2].len = 6;
    p[2].payload = srcMacAddr;
    p[2].next = &p[3];

    /* Ethertype */
    p[3].len = 2;
    p[3].payload = eType;
    p[3].next = &p[4];
    eType[0] = (etherType >> 8) & 0xFF;
    eType[1] = (etherType >> 0) & 0xFF;

    /* Data */
    p[4].len = len;
    p[4].payload = data;
    p[4].next = NULL;

    p[0].tot_len = p[0].len + p[1].len +p[2].len + p[3].len + p[4].len;

    sys_mutex_lock(&adi_ether->writeLock);
    err = adi_ether_netif_tx_frame(netif, p, txCallback);
    sys_mutex_unlock(&adi_ether->writeLock);

    return (err);
}

/*
 * Queue a 1722 frame for transmit.
 */
err_t
adi_ether_netif_1722_tx_frame(struct netif *netif, uint8_t *srcMacAddr,
    uint8_t *dstMacAddr, uint16_t etherType, uint8_t *p1722, uint16_t p1722Len,
    uint8_t *audio, uint16_t audioLen)
{
    adi_ether_netif *adi_ether = netif->state;
    struct pbuf p[6];
    uint8_t pad[2];
    uint8_t eType[2];
    err_t err;

    /* 2 byte alignment padding */
    p[0].len = 2;
    p[0].payload = pad;
    p[0].next = &p[1];

    /* Dest MAC */
    p[1].len = 6;
    p[1].payload = dstMacAddr;
    p[1].next = &p[2];

    /* Src MAC */
    p[2].len = 6;
    p[2].payload = srcMacAddr;
    p[2].next = &p[3];

    /* Ethertype */
    p[3].len = 2;
    p[3].payload = eType;
    p[3].next = &p[4];
    eType[0] = (etherType >> 8) & 0xFF;
    eType[1] = (etherType >> 0) & 0xFF;

    /* 1722 Data */
    p[4].len = p1722Len;
    p[4].payload = p1722;
    p[4].next = NULL;

    p[0].tot_len = p[0].len + p[1].len +p[2].len + p[3].len + p[4].len;

    /* Audio Data */
    if (audio) {
        p[4].next = &p[5];
        p[5].len = audioLen;
        p[5].payload = audio;
        p[5].next = NULL;
        p[0].tot_len += p[5].len;
    }

    sys_mutex_lock(&adi_ether->writeLock);
    err = adi_ether_netif_tx_frame(netif, p, false);
    sys_mutex_unlock(&adi_ether->writeLock);

    return(err);
}

/*
 * Receive a frame into lwIP.
 */
static void
adi_ether_netif_lwip_rx_frame(struct adi_ether_netif *adi_ether, LWIP_ADI_ETHER_QUEUE *rxPktQEntry)
{
    ADI_EMAC_DMA_DESC *desc = rxPktQEntry->desc;
    ADI_EMAC_RESULT result;
    struct pbuf *p = NULL;
    uint16_t len = 0;
    uint8_t *in = NULL;
    err_t ok;

    /* Only accept full frames with normal descriptors */
    if ( ((desc->RxDescWB.PLenStatus & ADI_EMAC_RX_DESC_WB_CTXT) == 0) &&
         (desc->RxDescWB.PLenStatus & ADI_EMAC_RX_DESC_WB_FD) &&
         (desc->RxDescWB.PLenStatus & ADI_EMAC_RX_DESC_WB_LD) ) {
        len = desc->RxDescWB.PLenStatus & ADI_EMAC_RX_DESC_WB_PL_BITM;
        in = rxPktQEntry->data->data;
    }

    /* Copy packet into lwIP pbuf */
    if (in) {
        p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
        if (p) {
            ok = pbuf_take(p, in, len);
            if (ok != ERR_OK) {
                pbuf_free(p); p = NULL;
            }
        }
    }

    /* Submit packet */
    if (p) {
        if (adi_ether->netif->input(p, adi_ether->netif) == ERR_OK) {
            LINK_STATS_INC(link.recv);
        } else {
            LINK_STATS_INC(link.drop);
            pbuf_free(p);
        }
    } else {
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);
    }

    /* Return DMA descriptor to the driver */
    adi_ether_netif_reset_rx_desc(rxPktQEntry->desc, rxPktQEntry->data);
    result = adi_emac_ReSubmitDescriptorRx(
        rxPktQEntry->desc, sizeof(LWIP_ADI_ETHER_PACKET_DATA), 0, true
    );
    adi_emac_RxDescriptorPoll(adi_ether->hEthernet, 0);

}

/*
 * Receive a ptp rx/tx frame into ptp.
 */
static void
adi_ether_netif_ptp_frame(struct adi_ether_netif *adi_ether,
    LWIP_ADI_ETHER_QUEUE *pktBuffer, uint8_t pktType)
{
#if 0
    uint16_t len;
    uint8_t *in;

    /* Get the length from the first 2 bytes of the frame */
    in = (uint8_t *)pktBuffer->Data;
    len = *((uint16_t *)in);

    /* Skip the length field */
    in += sizeof(uint16_t);

    /* Call the callback */
    if (adi_ether->ptpPktCb) {
        adi_ether->ptpPktCb(adi_ether, in, len, pktType,
            pktBuffer->TimeStamp.LSecond, pktBuffer->TimeStamp.NanoSecond);
    }

    /* Return receive pktBuffers to the driver */
    if (pktType == ADI_ETHER_PKT_TYPE_RX) {
        //adi_ether_netif_reset_rx_buff(pktBuffer);
        sys_mutex_lock(&adi_ether->readLock);
#if 0
        adi_ether_Read(adi_ether->hEthernet, pktBuffer);
#endif
        sys_mutex_unlock(&adi_ether->readLock);
    }
#endif
}

/*
 * Check if a frame is a 1722 frame
 */
#define ETHTYPE_AVBTP   0x22f0
static bool isP1722(ADI_EMAC_DMA_DESC *pktBuffer)
{
#if 0
    struct eth_hdr *ethhdr;
    struct eth_vlan_hdr *vlan;
    bool is1722 = false;

    /* Examine the Ethernet header */
    ethhdr = (struct eth_hdr *)pktBuffer->Data;

    /* Identify and process 1722 AAF frames */
    if (ethhdr->type == __htons(ETHTYPE_VLAN)) {
        vlan = (struct eth_vlan_hdr *)((uint8_t *)pktBuffer->Data + SIZEOF_ETH_HDR);
        if (vlan->tpid == __htons(ETHTYPE_AVBTP)) {
            is1722 = true;
        }
    }

    return(is1722);
#else
    return(false);
#endif
}

/* This function returns a 1722 Rx frame to the driver */
void
adi_ether_netif_p1722_free(struct adi_ether_netif *adi_ether, void *p)
{
#if 0
    ADI_ETHER_BUFFER *pktBuffer = (ADI_ETHER_BUFFER *)p;

    UNUSED(pktBuffer);

    /* Return receive pktBuffers to the driver */
    //adi_ether_netif_reset_rx_buff(pktBuffer);
    sys_mutex_lock(&adi_ether->readLock);
#if 0
    adi_ether_Read(adi_ether->hEthernet, pktBuffer);
#endif
    sys_mutex_unlock(&adi_ether->readLock);
#endif
}


/*
 * Receive a p1722 rx frame into 1722 stack.
 */
static void
adi_ether_netif_p1722_frame(struct adi_ether_netif *adi_ether,
    LWIP_ADI_ETHER_QUEUE *pktBuffer)
{
#if 0
    uint16_t len;
    uint8_t *in;

    /* Get the length from the first 2 bytes of the frame */
    in = (uint8_t *)pktBuffer->Data;
    len = *((uint16_t *)in);

    /* Call the callback */
    if (adi_ether->p1722PktCb) {
        adi_ether->p1722PktCb(adi_ether, in, len, pktBuffer);
    }
#endif
}

/* Worker thread to process events from the driver */
static void
adi_ether_netif_worker(void *pvParameters)
{
    struct adi_ether_netif *adi_ether = pvParameters;
    struct netif *netif = adi_ether->netif;
    LWIP_ADI_ETHER_QUEUE *rxPktQEntry, *txPktQEntry;
    sys_prot_t protect;
    void *toDo;
    bool busy;

    while (1) {
        sys_arch_mbox_fetch(&adi_ether->workerToDo, &toDo, 0);
        switch ((uintptr_t)toDo) {
            case WORKER_LINK_UP:
                netif_set_link_up(netif);
                break;
            case WORKER_LINK_DOWN:
                netif_set_link_down(netif);
                break;
            case WORKER_PKT_RX:
                adi_ether->rxPktProcessing = true;
                do {
                    rxPktQEntry = &adi_ether->pktRxQueue[adi_ether->rxPktIdx];
                    protect = sys_arch_protect();
                    adi_emac_IsDescriptorBusyRx(rxPktQEntry->desc, &busy);
                    adi_ether->rxPktProcessing = (busy == false);
                    sys_arch_unprotect(protect);
                    if (adi_ether->rxPktProcessing) {
                        if (0) { // timestamp available
                            adi_ether_netif_ptp_frame(adi_ether, rxPktQEntry, ADI_ETHER_PKT_TYPE_RX);
                        } else if (0) { // (isP1722(pktBuffer))
                            adi_ether_netif_p1722_frame(adi_ether, rxPktQEntry);
                        } else {
                            adi_ether_netif_lwip_rx_frame(adi_ether, rxPktQEntry);
                        }
                        if (++adi_ether->rxPktIdx == ADI_ETHER_NUM_RX_BUFFS) {
                            adi_ether->rxPktIdx = 0;
                        }
                    }
                } while (adi_ether->rxPktProcessing);
                break;
            case WORKER_PKT_TX:
                adi_ether->txPktProcessing = true;
                do {
                    txPktQEntry = &adi_ether->pktTxQueue[adi_ether->txPktTail];
                    protect = sys_arch_protect();
                    adi_emac_IsDescriptorBusyTx(txPktQEntry->desc, &busy);
                    adi_ether->txPktProcessing =
                        (busy == false) && (adi_ether->txPktTail != adi_ether->txPktHead);
                    sys_arch_unprotect(protect);
                    if (adi_ether->txPktProcessing) {
                        /* Process Tx DESC here */
                        if (++adi_ether->txPktTail == ADI_ETHER_NUM_TX_BUFFS) {
                            adi_ether->txPktTail = 0;
                        }
                    }
                } while (adi_ether->txPktProcessing);
                break;
            default:
                break;
        }
    }
}

/* Driver callback handling status events */
static void
adi_ether_netif_status_callback(void *usr, uint32_t event, void *param)
{
    adi_ether_netif *adi_ether = (adi_ether_netif *)usr;
    UBaseType_t isr;

    UNUSED(adi_ether);

    isr = taskENTER_CRITICAL_FROM_ISR();  // FIXME to sys_arch_xxx
    xInsideISR++;
    taskEXIT_CRITICAL_FROM_ISR(isr);

    switch(event)
    {
        case ADI_EMAC_EVENT_PHY_INT:
            break;
        default:
            break;
    }

    isr = taskENTER_CRITICAL_FROM_ISR();
    xInsideISR--;
    taskEXIT_CRITICAL_FROM_ISR(isr);
}

/* Driver callback handling DMA events */
static void
adi_ether_netif_dma_callback(void *usr, uint32_t event, void *param)
{
    adi_ether_netif *adi_ether = (adi_ether_netif *)usr;
    UBaseType_t isr;

    isr = taskENTER_CRITICAL_FROM_ISR();
    xInsideISR++;
    taskEXIT_CRITICAL_FROM_ISR(isr);

    switch(event)
    {
        case ADI_EMAC_EVENT_TX_PROCESSED:
            if (!adi_ether->txPktProcessing) {
                sys_mbox_trypost(&adi_ether->workerToDo, (void *)WORKER_PKT_TX);
            }
            break;
        case ADI_EMAC_EVENT_RX_PROCESSED:
            if (!adi_ether->rxPktProcessing) {
                sys_mbox_trypost(&adi_ether->workerToDo, (void *)WORKER_PKT_RX);
            }
            break;
        default:
            break;
    }

    isr = taskENTER_CRITICAL_FROM_ISR();
    xInsideISR--;
    taskEXIT_CRITICAL_FROM_ISR(isr);
}

static void
adi_ether_init_dma(struct adi_ether_netif *adi_ether)
{
    ADI_EMAC_RESULT etherResult;

    bool sf_mode = (adi_ether->port == EMAC0) ? true :false;

    if (adi_ether->port == EMAC0) {
        /* Enable Rx Queue 0 */
        etherResult = adi_emac_EnableRxQueue(adi_ether->hEthernet, 0,
            ADI_EMAC_RXQ_ENABLE_DCB_GEN);
        /* Use maximum queue size for EMAC0 = 64*256 = 16384 bytes */
        etherResult = adi_emac_SetRxQueueSize(adi_ether->hEthernet, 0, 63);
        /* Enable Tx Queue 0 */
        etherResult = adi_emac_EnableTxQueue(adi_ether->hEthernet, 0,
            ADI_EMAC_TXQ_ENABLE_GEN);
        /* Use maximum queue size for EMAC0 = 64*256 = 16384 bytes */
        etherResult = adi_emac_SetTxQueueSize(adi_ether->hEthernet, 0, 63);
    }

    /* Configure opmode settings for Rx queue 0 */
    etherResult = adi_emac_SetRxQueueOpmode(adi_ether->hEthernet, 0,
        sf_mode, false, false, ADI_EMAC_OPMODE_RTC_64);
    /* Configure opmode settings for Tx queue 0 */
    etherResult = adi_emac_SetTxQueueOpmode(adi_ether->hEthernet, 0,
        sf_mode, ADI_EMAC_OPMODE_TTC_64);
    // CONFIRM: Not in ADI
    etherResult = adi_emac_ConfigureHWFlowControl(adi_ether->hEthernet, 0,
        false, 0u, 0u);
    /* Configure DMA in OSF mode if needed */
    etherResult = adi_emac_TxDMAEnableOSFMode(adi_ether->hEthernet, 0, true);

    /* Configure RX buffer size  */
    etherResult = adi_emac_SetRxBuffSizeLimit(adi_ether->hEthernet, 0,
        false, 1546u, 0u);

    /* Configure SCB interface */
    if (adi_ether->port == EMAC0)
    {
        // CONFIRM: Not in ADI
        etherResult = adi_emac_SetRxQueueDMAMapType(adi_ether->hEthernet, 0,
            0, false);
        etherResult = adi_emac_ConfigureSCBInterface(adi_ether->hEthernet,
            BITM_EMAC_DMA_SYSBMODE_BLEN16, BITM_EMAC_DMA_SYSBMODE_BLEN16,
            ADI_EMAC_SCB_BURSTMODE_16_8_4, true, true, false);
    } else {
        etherResult = adi_emac_ConfigureSCBInterface(adi_ether->hEthernet,
            BITM_EMAC_DMA_SYSBMODE_BLEN4, BITM_EMAC_DMA_SYSBMODE_BLEN4,
            ADI_EMAC_SCB_BURSTMODE_16_8_4, true, true, false);
    }

    /*
     * Configure DMA burst - 32 bytes burst for TX, 1 byte burst for RX,
     * fixed burst - false, PBL8 = false
     */
    etherResult = adi_emac_ConfigureDMABurst(adi_ether->hEthernet, 0,
        ADI_EMAC_TRANSMIT_32, ADI_EMAC_RECEIVE_BL_32, false);

    /* Submit Rx descriptors */
    etherResult = adi_emac_SubmitDescriptorListRx(
       adi_ether->hEthernet,
       0, adi_ether->rxDmaDesc, ADI_ETHER_NUM_RX_BUFFS, true);

    /* Submit Tx descriptors */
    etherResult = adi_emac_SubmitDescriptorListTx(
       adi_ether->hEthernet,
       0, adi_ether->txDmaDesc, ADI_ETHER_NUM_TX_BUFFS, true);
}

/***********************************************************************
 * PHY
 **********************************************************************/
// FIXME
#include "init.h"

#define ADIN1300_VENDOR  0x0283
#define DP83865_VENDOR   0x2000
#define DP83867_VENDOR   0x2000

#define ADIN1300_MODEL   0xBC30
#define DP83865_MODEL    0x5C7A
#define DP83867_MODEL    0xA231

#define EMAC_DEVICE_NO   0u
#define PHY_DEVICE_ADDR  0u

// ADIN1300
#define REG_PHY_INT_MASK            (0x18)
 #define PHY_HW_IRQ_EN              (0x0001)
 #define PHY_INT_MASK_SPEED         (0x0002)
 #define PHY_INT_MASK_AUTO_NEGO_COM (0x0100)
 #define PHY_INT_MASK_LINK          (0x0004)

#define REG_PHY_INT_READ_CLEAR      (0x19)
 #define PHY_HW_IRQ_PENDING         (0x0001)

#define REG_PHY_LINK_AN_STATUS      (0x1A)
 #define PHY_LINK_STATUS_UP                 (0x0040)
 #define PHY_LINK_STATUS_FULL_DUPLEX        (0x0080)
 #define BITM_PHY_LINK_STATUS_SPEED         (0x0300)
 #define ENUM_PHY_LINK_STATUS_SPEED_1000    (0x0200)
 #define ENUM_PHY_LINK_STATUS_SPEED_100     (0x0100)
 #define ENUM_PHY_LINK_STATUS_SPEED_10      (0x0000)
 #define PHY_LINK_STATUS_AUTO_NEFO_COM      (0x1000)

#define ADIN1x00_INT_MASK \
    PHY_INT_MASK_AUTO_NEGO_COM | PHY_INT_MASK_SPEED | \
    PHY_INT_MASK_LINK

// DP83867
#define DP83867_PHYSTS_REG          (0x0011)
#define DP83867_MICR_REG            (0x0012)
#define DP83867_ISR_REG             (0x0013)
#define DP83867_CFG3_REG            (0x001E)

#define DP83867_PHYSTS_LINK_STATUS  (0x0400)
#define DP83867_PHYSTS_SPEED_BITM   (0xC000)
#define DP83867_PHYSTS_SPEED_10     (0x0000)
#define DP83867_PHYSTS_SPEED_100    (0x4000)
#define DP83867_PHYSTS_SPEED_1000   (0x8000)
#define DP83867_PHYSTS_DUPLEX       (0x2000)

#define DP83867_MICR_LINK           (0x0400)
#define DP83867_MICR_AUTO_NEGO_COM  (0x0800)

#define DP83867_ISR_LINK            (0x0400)
#define DP83867_ISR_AUTO_NEGO_COM   (0x0800)

#define DP83867_CFG3_INT_OE         (0x0080)

#define DP83867_INT_MASK \
    DP83867_MICR_LINK | DP83867_MICR_AUTO_NEGO_COM

static ADI_EMAC_PHY_RESULT
adi_ether_adin1x00_phy_init(ADI_EMAC_HANDLE hemac, uint32_t phyAddr)
{
    ADI_EMAC_PHY_RESULT ePhyResult;
    uint16_t ANAR;
    uint16_t value;
    uint16_t phyId[2];

    /* Get PHY device ID */
    ePhyResult = adi_emac_phy_GetBasicPhyIdentifiers(hemac, phyId);

    /* Disable Autonegotiation */
    ePhyResult = adi_emac_phy_AutonegEnable(hemac, false);

    /* Get Autonegotiation Advertisement Register (ANAR) value */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
         ePhyResult = adi_emac_phy_GetAutonegAdvertisemetValues(hemac, &ANAR);
    }

    /* Set Auto-Negotiation Advertisement Register (ANAR) values */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ANAR |= ADI_EMAC_PHY_PAUSE_CAPABLE;
        ePhyResult = adi_emac_phy_SetAutonegAdvertisemetValues(hemac, ANAR );
    }

    /* Set 1000Mbps configuration values */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_Set1000BasicConfiguration(
            hemac, false, false, false, ADI_EMAC_PHY_1000BT_FULL_HALF_ADV, false);
    }

    /* Set interrupt mask */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_RegisterWrite(hemac, REG_PHY_INT_MASK, ADIN1x00_INT_MASK);
    }

    /* Clear pending interrupts */
    ePhyResult = adi_emac_phy_RegisterRead(hemac, REG_PHY_INT_READ_CLEAR, &value);

    /* Enable PHY interrupt pin */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_RegisterRead(hemac, REG_PHY_INT_MASK, &value);
    }
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        value |= PHY_HW_IRQ_EN;
        ePhyResult = adi_emac_phy_RegisterWrite(hemac, REG_PHY_INT_MASK, value);
    }

    /* Enable Autonegotiation */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_AutonegEnable(hemac, true);
    }

    /* Restart Autonegotiation */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_RestartAutoNeg(hemac);
    }

    return(ePhyResult);
}

static void
adi_ether_adin1x00_phy_irq(struct adi_ether_netif *adi_ether)
{
    ADI_EMAC_HANDLE hemac = adi_ether->hEthernet;
    ADI_EMAC_PHY_RESULT ePhyResult;
    ADI_EMAC_RESULT eMACResult;
    ADI_EMAC_MAC_SPEED speed;
    uint16_t ISR;
    uint16_t PHYSTS;

    do {

        /* Read and clear IRQ */
        ePhyResult = adi_emac_phy_RegisterRead(hemac, REG_PHY_INT_READ_CLEAR, &ISR);

        /* Process link status */
        if (ISR & PHY_INT_MASK_LINK) {
            ePhyResult = adi_emac_phy_RegisterRead(hemac, REG_PHY_LINK_AN_STATUS, &PHYSTS);
            if (PHYSTS & PHY_LINK_STATUS_UP) {
                /* Link Up */
                adi_ether->linkUp = true;
                sys_mbox_trypost(&adi_ether->workerToDo, (void *)WORKER_LINK_UP);
            } else {
                /* Link Dn */
                adi_ether->linkUp = false;
                sys_mbox_trypost(&adi_ether->workerToDo, (void *)WORKER_LINK_DOWN);
            }
        }

        /* Process speed and duplex */
        if (ISR & PHY_INT_MASK_AUTO_NEGO_COM) {
            ePhyResult = adi_emac_phy_RegisterRead(hemac, REG_PHY_LINK_AN_STATUS, &PHYSTS);
            if (PHYSTS & PHY_LINK_STATUS_AUTO_NEFO_COM) {
                switch (PHYSTS & BITM_PHY_LINK_STATUS_SPEED) {
                    case ENUM_PHY_LINK_STATUS_SPEED_10:
                        speed = ADI_EMAC_MAC_SPEED_10M;
                        break;
                    case ENUM_PHY_LINK_STATUS_SPEED_100:
                        speed = ADI_EMAC_MAC_SPEED_100M;
                        break;
                    case ENUM_PHY_LINK_STATUS_SPEED_1000:
                        speed = ADI_EMAC_MAC_SPEED_1000M;
                        break;
                    default:
                        speed = ADI_EMAC_MAC_SPEED_1000M;
                        break;
                }
                eMACResult = adi_emac_ConfigureMACSpeed(hemac, speed);
                if (PHYSTS && PHY_LINK_STATUS_FULL_DUPLEX) {
                    adi_ether->EMAC_MAC_CFG |= ADI_EMAC_MAC_CONFIG_DUPLEX_ENABLE;
                } else {
                    adi_ether->EMAC_MAC_CFG &= ~ADI_EMAC_MAC_CONFIG_DUPLEX_ENABLE;
                }
                eMACResult = adi_emac_EnableMACConfigurationModes(hemac,
                    adi_ether->EMAC_MAC_CFG);
            }
        }

    } while (ISR);
}

static ADI_EMAC_PHY_RESULT
adi_ether_dp83867_phy_init(ADI_EMAC_HANDLE hemac, uint32_t phyAddr)
{
    ADI_EMAC_PHY_RESULT ePhyResult;
    uint16_t ANAR;
    uint16_t value;

    /* Disable Autonegotiation */
    ePhyResult = adi_emac_phy_AutonegEnable(hemac, false);

    /* Get Autonegotiation Advertisement Register (ANAR) value */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
         ePhyResult = adi_emac_phy_GetAutonegAdvertisemetValues(hemac, &ANAR);
    }

    /* Set Auto-Negotiation Advertisement Register (ANAR) values */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ANAR |= ADI_EMAC_PHY_PAUSE_CAPABLE;
        ePhyResult = adi_emac_phy_SetAutonegAdvertisemetValues(hemac, ANAR );
    }

    /* Set 1000Mbps configuration values */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_Set1000BasicConfiguration(
            hemac, false, false, false, ADI_EMAC_PHY_1000BT_FULL_HALF_ADV, false);
    }

    /* Set interrupt mask */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_RegisterWrite(hemac, DP83867_MICR_REG, DP83867_INT_MASK);
    }

    /* Clear pending interrupts */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_RegisterRead(hemac, DP83867_ISR_REG, &value);
    }

    /* Enable PHY interrupt pin */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_RegisterRead(hemac, DP83867_CFG3_REG, &value);
    }
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        value |= DP83867_CFG3_INT_OE;
        ePhyResult = adi_emac_phy_RegisterWrite(hemac, DP83867_CFG3_REG, value);
    }

    /* Enable Autonegotiation */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_AutonegEnable(hemac, true);
    }

    /* Restart Autonegotiation */
    if (ePhyResult == ADI_EMAC_PHY_SUCCESS) {
        ePhyResult = adi_emac_phy_RestartAutoNeg(hemac);
    }

    return(ePhyResult);
}

static void
adi_ether_dp83867_phy_irq(struct adi_ether_netif *adi_ether)
{
    ADI_EMAC_HANDLE hemac = adi_ether->hEthernet;
    ADI_EMAC_PHY_RESULT ePhyResult;
    ADI_EMAC_RESULT eMACResult;
    ADI_EMAC_MAC_SPEED speed;
    uint16_t ISR;
    uint16_t PHYSTS;

    do {

        /* Read and clear IRQ */
        ePhyResult = adi_emac_phy_RegisterRead(hemac, DP83867_ISR_REG, &ISR);

        /* Process link status */
        if (ISR & DP83867_ISR_LINK) {
            ePhyResult = adi_emac_phy_RegisterRead(hemac, DP83867_PHYSTS_REG, &PHYSTS);
            if (PHYSTS & DP83867_PHYSTS_LINK_STATUS) {
                /* Link Up */
                adi_ether->linkUp = true;
                sys_mbox_trypost(&adi_ether->workerToDo, (void *)WORKER_LINK_UP);
            } else {
                /* Link Dn */
                adi_ether->linkUp = false;
                sys_mbox_trypost(&adi_ether->workerToDo, (void *)WORKER_LINK_DOWN);
            }
        }

        /* Process speed and duplex */
        if (ISR & DP83867_MICR_AUTO_NEGO_COM) {
            ePhyResult = adi_emac_phy_RegisterRead(hemac, DP83867_PHYSTS_REG, &PHYSTS);
            switch (PHYSTS & DP83867_PHYSTS_SPEED_BITM) {
                case DP83867_PHYSTS_SPEED_10:
                    speed = ADI_EMAC_MAC_SPEED_10M;
                    break;
                case DP83867_PHYSTS_SPEED_100:
                    speed = ADI_EMAC_MAC_SPEED_100M;
                    break;
                case DP83867_PHYSTS_SPEED_1000:
                    speed = ADI_EMAC_MAC_SPEED_1000M;
                    break;
                default:
                    speed = ADI_EMAC_MAC_SPEED_1000M;
                    break;
            }
            eMACResult = adi_emac_ConfigureMACSpeed(hemac, speed);
            if (PHYSTS && DP83867_PHYSTS_DUPLEX) {
                adi_ether->EMAC_MAC_CFG |= ADI_EMAC_MAC_CONFIG_DUPLEX_ENABLE;
            } else {
                adi_ether->EMAC_MAC_CFG &= ~ADI_EMAC_MAC_CONFIG_DUPLEX_ENABLE;
            }
            eMACResult = adi_emac_EnableMACConfigurationModes(hemac,
                adi_ether->EMAC_MAC_CFG);
        }

    } while (ISR);
}

static void
adi_ether_phy_callback(void *usr, uint32_t event, void *arg)
{
    struct adi_ether_netif *adi_ether = usr;
    UBaseType_t isr;

    isr = taskENTER_CRITICAL_FROM_ISR();
    xInsideISR++;
    taskEXIT_CRITICAL_FROM_ISR(isr);

    switch (event) {
        case ADI_EMAC_EVENT_PHY_INT:
            if (adi_ether->phyIrq) {
                adi_ether->phyIrq(adi_ether);
            }
            break;
        default:
            break;
    }

    isr = taskENTER_CRITICAL_FROM_ISR();
    xInsideISR--;
    taskEXIT_CRITICAL_FROM_ISR(isr);
}

/***********************************************************************
 * PHY
 **********************************************************************/

static void
adi_ether_init_phy(struct adi_ether_netif *adi_ether)
{
    ADI_EMAC_HANDLE hemac = adi_ether->hEthernet;
    ADI_EMAC_PHY_RESULT ePhyResult;
    uint16_t phyId[2];

    /* Set PHY interface parameters */
    ePhyResult = adi_emac_phy_Config(hemac,
        PHY_DEVICE_ADDR, ADI_SCLK_RANGE_100_150, false, false, 0, false, false
    );

    // FIXME: Register the PHY IRQ callback
    eth_phy_cb_init(0, adi_ether_phy_callback, ADI_EMAC_EVENT_PHY_INT, adi_ether);

    /* Get PHY device ID */
    ePhyResult = adi_emac_phy_GetBasicPhyIdentifiers(hemac, phyId);

    /* Attach PHY handler */
    if ((phyId[0] == DP83867_VENDOR) && (phyId[1] == DP83867_MODEL)) {
        adi_ether->phyInit = adi_ether_dp83867_phy_init;
        adi_ether->phyIrq = adi_ether_dp83867_phy_irq;
    } else if ((phyId[0] == ADIN1300_VENDOR) && (phyId[1] == ADIN1300_MODEL)) {
        adi_ether->phyInit = adi_ether_adin1x00_phy_init;
        adi_ether->phyIrq = adi_ether_adin1x00_phy_irq;
    } else {
        adi_ether->phyInit = NULL;
        adi_ether->phyIrq = NULL;
    }

    /* Configure the PHY */
    if (adi_ether->phyInit) {
        adi_ether->phyInit(hemac, PHY_DEVICE_ADDR);
    }
}

/* This function initializes the ADI EMAC driver */
static void
adi_ether_netif_low_level_init(struct netif *netif)
{
    struct adi_ether_netif *adi_ether = netif->state;
    ADI_EMAC_MAC_FILTER_MODES filter = 0;
    ADI_EMAC_RESULT etherResult;

    /* maximum transfer unit */
    netif->mtu = 1500;

    /* device capabilities */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

    /* Open the ADI EMAC driver */
    etherResult = adi_emac_Open(
        adi_ether->port == EMAC0 ? 0 : 1,
        adi_ether->ADI_EMAC_MEMORY, sizeof(adi_ether->ADI_EMAC_MEMORY),
        &adi_ether->hEthernet
    );

    /* Register the status event callback */
    etherResult = adi_emac_RegisterStatusCallback(adi_ether->hEthernet,
        adi_ether_netif_status_callback, adi_ether);

    /* Register the Rx DMA event callback */
    etherResult = adi_emac_RegisterRxDmaDataCallback(adi_ether->hEthernet,
        0, adi_ether_netif_dma_callback, adi_ether);

    /* Register the Tx DMA event callback */
    etherResult = adi_emac_RegisterTxDmaDataCallback(adi_ether->hEthernet,
        0, adi_ether_netif_dma_callback, adi_ether);

    /* Set the MAC hardware address in the driver */
    etherResult = adi_emac_SetMACAddress(
        adi_ether->hEthernet, 0, adi_ether->ethAddr.addr
    );

    /* Set the MAC hardware address in lwIP */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    memcpy(netif->hwaddr, adi_ether->ethAddr.addr, netif->hwaddr_len);

    /* Create a semaphore for the worker thread */
    sys_mbox_new(&adi_ether->workerToDo, ADI_ETHER_NUM_MBOX_EVENTS);

    /* Create read/write locks for lwIP/PTP accesses */
    sys_mutex_new(&adi_ether->readLock);
    sys_mutex_new(&adi_ether->writeLock);

    /* Spin up the worker thread */
    adi_ether->worker = sys_thread_new(
        "adi_ether_netif_worker", adi_ether_netif_worker, adi_ether,
        TCPIP_THREAD_STACKSIZE, ETHER_WORKER_PRIO
    );

    /* Initialize the DMA */
    adi_ether_init_dma(adi_ether );

    /* Set packet filter modes */
    //filter = ADI_EMAC_MACFRMFILT_PR; // Promiscious mode
    //filter = ADI_EMAC_MACFRMFILT_RA; // Receive all
    filter = ADI_EMAC_MACFRMFILT_PM; // Pass multicast
    etherResult = adi_emac_EnableRXFrameFilters(adi_ether->hEthernet, filter);

    /* Enable the MAC Rx */
    etherResult = adi_emac_EnableRx(adi_ether->hEthernet, true);
    etherResult = adi_emac_RxDescriptorPoll(adi_ether->hEthernet, 0);

    /* Enable the MAC Tx */
    etherResult = adi_emac_EnableTx(adi_ether->hEthernet, true);
    etherResult = adi_emac_TxDescriptorPoll(adi_ether->hEthernet, 0);

    /* Init the PHY */
    adi_ether_init_phy(adi_ether);

    /* Indicate initialization OK */
    adi_ether->initOk = true;
}

/*************************************************************************
 * Public APIs
 ************************************************************************/
err_t
adi_ether_netif_init(struct netif *netif)
{
    struct adi_ether_netif *adi_ether = netif->state;
#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    if (adi_ether->hostName) {
        netif->hostname = adi_ether->hostName;
    }
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    netif->num = adi_ether->idx;

#if LWIP_IPV4
    netif->output = etharp_output;
#endif
    netif->linkoutput = adi_ether_netif_lwip_tx_frame;

    /* Save a ref to the netif for callback link processing */
    adi_ether->netif = netif;

    /* Initialize the ADI EMAC driver Rx/Tx DMA descriptors */
    adi_ether_netif_init_rx_desc(adi_ether);
    adi_ether_netif_init_tx_desc(adi_ether);

    /* Do low-level ADI EMAC hardware init */
    adi_ether_netif_low_level_init(netif);

    return ERR_OK;
}

// FIXME: Switch to cached buffers
#if 1
static adi_ether_netif netifs[ADI_ETHER_MAX_NETIFS]
    __attribute__ ((section(".l3_uncached_data"))) = { 0 };
#else
// FIXME: Array elements not cache aligned?
ADI_CACHE_ALIGN adi_ether_netif netifs[ADI_ETHER_MAX_NETIFS] = { 0 };
#endif

adi_ether_netif *
adi_ether_netif_new(void *usrPtr)
{
    adi_ether_netif *adi_ether;
    sys_prot_t prot;
    int i;

    /* Find an available netif */
    prot = sys_arch_protect();
    for (i = 0; i < ADI_ETHER_MAX_NETIFS; i++) {
        adi_ether = &netifs[i];
        if (!adi_ether->allocated) {
            break;
        }
    }

    /* Return NULL if no netifs available */
    if (i == ADI_ETHER_MAX_NETIFS) {
        sys_arch_unprotect(prot);
        return(NULL);
    }

    /* Mark instance as being allocated */
    adi_ether->allocated = true;
    adi_ether->idx = i;

    /* Save the user pointer */
    adi_ether->usrPtr = usrPtr;

    sys_arch_unprotect(prot);

    return(adi_ether);
}

void
adi_ether_netif_delete(adi_ether_netif *adi_ether)
{
    sys_prot_t prot;

    if (adi_ether->hEthernet) {
        adi_emac_Close(adi_ether->hEthernet);
    }

    /* TODO: kill the worker thread here */

    prot = sys_arch_protect();
    ADI_ETHER_MEMSET(adi_ether, 0, sizeof(*adi_ether));
    sys_arch_unprotect(prot);
}

err_t
adi_ether_netif_set_src_addr_filt(struct netif *netif,
    const uint32_t ipAddr, const uint8_t ipMaskBits, const bool invert)
{
    struct adi_ether_netif *adi_ether = netif->state;
    //adi_ether_SetSrcAddrFilt(adi_ether->hEthernet, ipAddr, ipMaskBits, invert);
    UNUSED(adi_ether);
    return ERR_OK;
}

err_t
adi_ether_netif_set_src_addr_filt_enable(struct netif *netif,
    const bool enable)
{
    struct adi_ether_netif *adi_ether = netif->state;
    //adi_ether_SetSrcAddrFiltEnable(adi_ether->hEthernet, enable);
    UNUSED(adi_ether);
    return ERR_OK;
}

err_t
adi_ether_netif_set_dst_port_filt(struct netif *netif,
    const uint16_t port, const bool udp, const bool invert)
{
    struct adi_ether_netif *adi_ether = netif->state;
    //adi_ether_SetDstPortFilt(adi_ether->hEthernet, port, udp, invert);
    UNUSED(adi_ether);
    return ERR_OK;
}

err_t
adi_ether_netif_set_dst_port_filt_enable(struct netif *netif,
    const bool enable)
{
    struct adi_ether_netif *adi_ether = netif->state;
    //adi_ether_SetDstPortFiltEnable(adi_ether->hEthernet, enable);
    UNUSED(adi_ether);
    return ERR_OK;
}
