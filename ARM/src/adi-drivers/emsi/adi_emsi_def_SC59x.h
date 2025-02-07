/******************************************************************************
 * @file:    adi_emsi_def_SC59x.h
 * @brief:   EMSI driver local Header file
 * @details  Header File for the EMSI driver local API functions and definitions
 * @version: $Revision:$
 * @date:    $Date: 2021-07-29$
 *****************************************************************************

 Copyright (c) 2022 Analog Devices.  All Rights Reserved.

 This software is proprietary.  By using this software you agree
 to the terms of the associated Analog Devices License Agreement.

 *******************************************************************************/

#ifndef _ADI_EMSI_DEF_SC59x_H_
#define _ADI_EMSI_DEF_SC59x_H_

/*==========  I N C L U D E  ==========*/
#include <adi_osal.h>
#include "adi_emsi.h"
#if defined(__ADSPSC598_FAMILY__)
#include <ADSP-SC59x_device.h>
#else
#error "Unknown processor family"
#endif



#if defined(__ADSPSC598_FAMILY__)
/*! Maximum number of available EMSI instances */
#define ADI_EMSI_MAX_INSTANCES					0x1u
#endif

/*!Upper Frequency divider select mask*/
#define UPPER_FREQ_SEL    						0x300u
#define BITP_UPPER_FREQ_SEL						8u

/*!Lower Frequency divider select mask*/
#define LOWER_FREQ_SEL    						0xFFu

/*!Timeout Count*/
#if defined(ADI_EMSI0_CFG_TIMEOUT)
#define TIMEOUT     						    (ADI_EMSI0_CFG_TIMEOUT)
#else
#define TIMEOUT     							0xFFFFFFFFu
#endif

/*!Macros for reset values of the registers */
#define ONE_BYTE_ZERO  							0x00u
#define TWO_BYTE_ZERO							0x0000u
#define EMMC_CTL_RESET						    0x000cu
#define FOUR_BYTE_ZERO							0X00000000u

/*!Default Mask value for the EMSI interrupt mask register to clear all the interrupts(W1C)*/
#define DEFAULT_INT_STAT_MASK_VALUE				0x40FFu
#define DEFAULT_ERROR_INT_STAT_MASK_VALUE 		0xFFFFu
/*!Set the interrupt status bits*/
#define ENABLE_ERROR_INT_STAT					(BITM_EMSI_ERR_STAT_EN_BOOT_ACK_ERR|\
												 BITM_EMSI_ERR_STAT_EN_RESP_ERR|\
												 BITM_EMSI_ERR_STAT_EN_ADMA_ERR|\
												 BITM_EMSI_ERR_STAT_EN_AUTO_CMD_ERR|\
												 BITM_EMSI_ERR_STAT_EN_DATA_END_BIT_ERR|\
												 BITM_EMSI_ERR_STAT_EN_DATA_CRC_ERR|\
												 BITM_EMSI_ERR_STAT_EN_DATA_TOUT_ERR|\
												 BITM_EMSI_ERR_STAT_EN_CMD_IDX_ERR|\
												 BITM_EMSI_ERR_STAT_EN_CMD_END_BIT_ERR_STAT_EN|\
												 BITM_EMSI_ERR_STAT_EN_CMD_CRC_ERR|\
												 BITM_EMSI_ERR_STAT_EN_CMD_TOUT_ERR)

#define ENABLE_NORMAL_INT_STAT			 		(BITM_EMSI_ISTAT_EN_CQE_EVENT|\
												 BITM_EMSI_ISTAT_EN_FX_EVENT|\
												 BITM_EMSI_ISTAT_EN_CARD_REMOVAL|\
												 BITM_EMSI_ISTAT_EN_CARD_INSERTION|\
												 BITM_EMSI_ISTAT_EN_DMA_INTERRUPT|\
												 BITM_EMSI_ISTAT_EN_XFER_COMPLETE|\
												 BITM_EMSI_ISTAT_EN_CMD_COMPLETE)

/*!Enable the transfer complete interrupt, CQE Event interrupt to core to the core*/
#define ENABLE_NORMAL_INT						(BITM_EMSI_ISTAT_INTEN_CQE_EVENT|\
												 BITM_EMSI_ISTAT_INTEN_XFER_COMPLETE)

/*!Enable the Command Timeout error interrupt, Command CRC error interrupt,
 * Command end bit error interrupt, Command index error interrupt,
 * Data CRC error interrupt, Data end bit error interrupt,
 * Data timeout error interrupt, Auto command error interrupt,
 * ADMA error interrupt, Boot acknowledgement error interrupt, Response Error (R1 Response)
 * check interrupt to the core*/
#define ENABLE_ERROR_INT						(BITM_EMSI_ERR_STAT_INTEN_BOOT_ACK_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_RESP_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_ADMA_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_AUTO_CMD_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_DATA_END_BIT_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_DATA_CRC_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_DATA_TOUT_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_CMD_IDX_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_CMD_END_BIT_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_CMD_CRC_ERR|\
												 BITM_EMSI_ERR_STAT_INTEN_CMD_TOUT_ERR)

/*!Set the CQ interrupt status bits*/
#define ENABLE_CQ_INT_STAT						(BITM_EMSI_CQ_ISTAT_EN_TCL|\
 	 	 	 	 	 	 	 	 	 	 	 	 BITM_EMSI_CQ_ISTAT_EN_RED|\
												 BITM_EMSI_CQ_ISTAT_EN_TCC|\
												 BITM_EMSI_CQ_ISTAT_EN_HAC)

/*!Enable the Response error detected and Task complete interrupt to the core
 * (This interrupt is mapped to CQE_EVENT interrupt in ISTAT register)*/
#define ENABLE_CQ_INT					 		(BITM_EMSI_CQ_ISTAT_INTEN_RED|\
												 BITM_EMSI_CQ_ISTAT_INTEN_TCC)

/*!For DDR transfers default block size is 512 Bytes*/
#define DDR_TRANSFER_BLOCKSIZE					512u
/*!Normal boot default block size is 512 Bytes in Non-DMA mode*/
#define NORMAL_BOOT_BLOCKSIZE					512u
/*!For CQ transfers default block size is 512 Bytes*/
#define CQ_TRANSFER_BLOCKSIZE					512u

/*!Block Size SD Status block (512 bits of SD status)*/
#define BLOCK_SIZE_SD_STATUS					64u
/*!SD card status block count (1 block * 512 bits = 512 bits (64 Bytes))*/
#define NO_OF_BLOCKS_SD_STATUS					1u

/*!For Bus test maximum default block size is 8 Bytes*/
#define BUS_TEST_TRANSFER_BLOCKSIZE				8u
/*!Block count (1 block * 8 bytes = 8 bytes) for bus test*/
#define NO_OF_BLOCKS_BUS_TEST					1u

/*!Block Size EXT_CSD register (512 bytes of EXT_CSD register)*/
#define BLOCK_SIZE_EXT_CSD_REG					512u
/*!Block count (1 block * 512 bytes = 512 bytes) for EXT_CSD register (CMD8)*/
#define NO_OF_BLOCKS_EXT_CSD_REG				1u
/*!Start address (Redundant)*/
#define START_ADDRESS_EXT_CSD_REG				0u


/*!For 26-bit descriptor length resulting into 64MB descriptor payload size
 * all 26 0's corresponds to 64 MB all 1's corresponds to 64MB-1 Byte*/
#define DESCRIPTOR_DATA_TRANSFER_LIMIT		    0x4000000u
/*!Upper 16-bit data length in 26-bit data length */
#define UPPER_16BIT_DATA_LENGTH					0x0000u
/*!Lower 10-bit data length in 26-bit data length */
#define LOWER_10BIT_DATA_LENGTH					0x000u
/*!Upper 16-bit data length in 26-bit data length mask */
#define UPPER_16BIT_DATA_LENGTH_MASK			0xFFFFu
/*!Lower 10-bit data length in 26-bit data length mask */
#define LOWER_10BIT_DATA_LENGTH_MASK			0x3FF0000u
/*!Maximum Number of descriptors to be used in ADMA2 operations (Maximum value is 63)*/
#define NUM_DMA_DESCRIPTORS 					(ADI_EMSI0_CFG_ADMA2_DESCRIPTORS)
/*!Bit position for 26-bit descriptor length*/
#define BITP_26_DATA_LENGTH_UP_10BIT 			6u
#define BITP_26_DATA_LENGTH_LO_16BIT			16u
/*!Attribute Configuration for ADMA2 descriptor*/
#define VALID_INTERRUPT_SET						0x25u
#define VALID_INTER_END_SET						0x27u


/*!Attribute Configuration ADMA3 descriptor pairs
 * (for more information refer to HRM ADMA3 configuration)*/
#define VALID_SET								0x9u
#define VALID_END_SET							0xbu
#define VALID_SET_INTEGRATED_DES				0x39u
#define VALID_END_SET_INTEGRATED_DES			0x3bu

/*!Attribute Configuration CQE TASK descriptors
 * (for more information refer to HRM CQE configuration)*/
#define	TASK_DES_ATTRIBUTES						0x2bu


/*!ADMA3 number of descriptors pairs*/
#define ADMA3_DMA_DESCRIPTORS_PAIRS 			(ADI_EMSI0_CFG_ADAM3_TASKS)

/*!CQE number of Task descriptors*/
#define MAXIMUM_SUPPORTED_CQE_TASKS		 	    31u


/*!Bus width configuration for eMMC and SD card (from card side)*/
#define SDR_1_BIT_BUS_WIDTH						0x0u
#define SDR_4_BIT_BUS_WIDTH						0x1u
#define SDR_8_BIT_BUS_WIDTH						0x2u
#define DDR_4_BIT_BUS_WIDTH						0x5u
#define DDR_8_BIT_BUS_WIDTH						0x6u
#define SDR_1_BIT_BUS_WIDTH_SD					0x0u
#define SDR_4_BIT_BUS_WIDTH_SD					0x2u

/*!EMSI Command definitions*/
#define GO_IDLE_STATE 							0u
#define SEND_OP_COND 							1u
#define SET_RELATIVE_ADDR 						3u
#define SWITCH 									6u
#define SEND_EXT_CSD 							8u
#define ALL_SEND_CID 							2u
#define WRITE_BLOCK 							24u
#define SEND_CSD 								9u
#define SET_BLOCKLEN 							16u
#define SELECT_CARD 							7u
#define DESELECT_CARD 							7u
#define READ_SINGLE_BLOCK 						17u
#define READ_MULTIPLE_BLOCK 					18u
#define SET_BLOCK_COUNT							23u
#define WRITE_SINGLE_BLOCK 						24u
#define WRITE_MULTIPLE_BLOCK 					25u
#define STOP_TRANSMISSION 						12u
#define SEND_STATUS 							13u
#define CMDQ_TASK_MGMT							48u
#define BUSTEST_W								19u
#define BUSTEST_R								14u

/*!SD Command definitions*/
#define RESET 									0u
#define SEND_IF_COND 							8u
#define APP_CMD 								55u
#define SD_SEND_OP_COND 						41u
#define SEND_RELATIVE_ADDR 						3u
#define SEND_STATUS 							13u
#define CMD_QUE 								44u
#define SEND_SCR 								51u
#define SET_BUS_WIDTH 							6u
#define SD_STATUS 								13u
#define ERASE_WR_BLK_START 						32u
#define ERASE_WR_BLK_END 						33u
#define SWITCH_FUNC 							6u

/*!Definitions for Switch Command (CMD6)*/
#define BITP_VALUE 								8u
#define BITP_INDEX 								16u
#define BITP_ACCESS								24u
#define ACCESS_CMDSET 							0u
#define ACCESS_SETBITS 							1u
#define ACCESS_CLEARBITS 						2u
#define ACCESS_WRITEBYTE 						3u
#define HS_TIMING_EXTCSD						185u
#define BUS_WIDTH_EXTCSD						183u
#define CMDQ_MODE_EN							15u

/*!Switch command (CMD6) masks*/
#define BITP_SWITCH_CMD_ERROR_CHK				7u
#define SWITCH_CMD_ERROR_MASK					1u
#define SWITCH_CMD_ERROR						1u

/*!Application specific command (ACMD6) masks*/
#define BITP_APP_CMD_CHK						5u
#define APP_CMD_CHK_MASK						1u
#define APP_CMD_CHK								1u

/*! SD Status Masks*/
#define SD_STATUS_MASK							0xFFu
#define	BITP_DAT_BUS_WIDTH						6u

/*! SD Switch function macros*/
#define SD_HIGH_SPEED_ACCESS_MODE			 	1u
#define SD_DEFAULT_ACCESS_MODE 					0u
#define SD_CHECK_FUNCTION 						0u
#define SD_SWITCH_FUNCTION 						1u
#define BITP_SWITCH_FUNCTION_MODE				31u

/*! SD Switch function check masks*/
#define BITP_SPEED_MODE_SUPPORT					8u
#define SPEED_MODE_SUPPORT_MASK					0xFFu
#define HS_MODE_MASK							0x2u
#define DS_MODE_MASK							0x1u
#define BITP_SPEED_MODE_READY					8u
#define SPEED_MODE_READY_MASK					0xFFu
#define HS_MODE_READY_MASK						0x2u
#define DS_MODE_READY_MASK						0x1u
#define STATUS_ERROR_HS							1u
#define STATUS_ERROR_DS							0u
#define STATUS_ERROR_MASK						0xFu

/*!Status command (CMD13) masks*/
#define CARD_IN_TRANSFER_STATE					4u
#define	CARD_CURRENT_STATE						9u
#define CARD_CURRENT_STATE_MASK					0xFu

/*!SDMA buffer boundary*/
#define SDMA_BUFFER_BOUNDARY_4K					(1024u*4u)
#define SDMA_BUFFER_BOUNDARY_8K					(1024u*8u)
#define SDMA_BUFFER_BOUNDARY_16K				(1024u*16u)
#define SDMA_BUFFER_BOUNDARY_32K				(1024u*32u)
#define SDMA_BUFFER_BOUNDARY_64K				(1024u*64u)
#define SDMA_BUFFER_BOUNDARY_128K				(1024u*128u)
#define SDMA_BUFFER_BOUNDARY_256K				(1024u*256u)
#define SDMA_BUFFER_BOUNDARY_512K				(1024u*512u)

/*!ADMA3 Descriptors masks*/
#define TRANSFER_MODE_REG_MASK					0XFFFFu
#define CMD_REG_MASK							0xFFFFu
#define BITP_CMD_REG							16u

/*!CQE Task Descriptors masks*/
#define BITP_DATA_TRANSFER_DIRECTION			12u
#define BITP_TASK_PRIORITY						13u
#define BITP_BLOCK_COUNT						16u

/*!CQE Task related masks*/
#define EMPTY_CARD_QUEUE						0x1u
#define CMD_QUE_CARD_SUPPORT					0x1u
#define BITP_DATA_TRANSFER_DIR 					12u
#define BITM_DATA_TRANSFER_DIR_MASK				0x1u
#define CQE_TCN_REG_CLEAR						0xFFFFFFFFu
#define CQE_SQSCMD_LAST_BLK						1u
#define CQE_SQSCMD_IDLE_TMR						0x1000u
#define CLR_CQ_IC_THRESHOLD						0x00u
#define RESP_ERR_DETECT_MASK					0xFDF9A080u

/*!Cache Alignment masks*/
#define CACHE_LINE_ALIGNMENT_MASK				0x3Fu
#define ALIGN_END_ADDRESS						0x10u

/*Delays*/
#define EIGHT_CYCLES							8u
#define SEVENTY_FOUR_CYCLES						74u

/*! \struct ADI_EMSI_DMADescriptor
 * Used for configuring the descriptor in ADMA2 mode of transfer
 **/
typedef struct
{
	uint32_t DES0;				/*!< Stores descriptors attributes and data length*/
#if defined(__ADSPARM__)
	uint32_t DES1;				/*!< Stores buffer address ARM Core*/
#else
	void* DES1;					/*!< Stores buffer address SHARC Core*/
#endif
}ADI_EMSI_DMADescriptor;


/*! \struct ADI_EMSI_ADAM3DescriptorPair
 * Used for configuring the descriptors in ADMA3 mode of transfer(Command and ADMA2 pairs)
 **/
typedef struct
{
	uint32_t DES0;				/*!<32 bit Block Count Descriptor field attributes*/

	uint32_t DES1;				/*!<32 bit Block Count Descriptor field
	 	 	 	 	 	 	 	 	programmed in 32-bit block count register*/

	uint32_t DES2;				/*!<16 bit Block Count and Block Size Descriptor field attributes*/

	uint32_t DES3;				/*!<16 bit Block Count and Block Size Descriptor field
	 	 	 	 	 	 	 	 	 programmed in 16-bit block count register
	 	 	 	 	 	 	 	 	 and block size register */

	uint32_t DES4;				/*!<Argument Descriptor field attributes*/

	uint32_t DES5;				/*!<Argument Descriptor field
	 	 	 	 	 	 	 	 	argument of the command to be send(Start address for read
	 	 	 	 	 	 	 	 	write commands)*/

	uint32_t DES6;				/*!<Command and Transfer Mode Descriptor field attributes*/

	uint32_t DES7;				/*!<Command and Transfer Mode Descriptor field
	 	 	 	 	 	 	 	 	programmed in command register and
	 	 	 	 	 	 	 	 	transfer mode register*/

	uint32_t DES8;				/*!< Stores descriptors attributes and data length ADMA2*/

#if defined(__ADSPARM__)
	uint32_t DES9;				/*!< Stores buffer address ARM Core ADMA2*/
#else
	void* DES9;					/*!< Stores buffer address SHARC Core ADMA2*/
#endif

}ADI_EMSI_ADAM3DescriptorPair;


/*! \struct ADI_EMSI_ADAM3IntegratedDes
 * ADMA3 integrated descriptors. These descriptor has pointer to each command descriptor.
 **/
typedef struct
{
	uint32_t DES0;				/*!<32 bit Pointer Descriptor field attributes*/

#if defined(__ADSPARM__)
	uint32_t DES1;				/*!<32 bit address pointer to command descriptor base address*/
#else
	void* DES1;					/*!<32 bit address pointer to command descriptor base address*/
#endif

}ADI_EMSI_ADAM3IntegratedDes;


/*! \struct ADI_EMSI_DEVICE
 * EMSI Device structure:
 * This structure contains all the EMSI parameters that will be used in EMSI driver.
 * This is an internal data structure and not exported to application
 **/
typedef struct
{
	ADI_CALLBACK                    pfCallback;        	 /*!< Application supplied callback */

	ADI_EMSI_CARD_TYPE              eCardType;			 /*!< Application supplied card type */

	uint8_t						    nCmdParameters;		 /*!< CMD register Parameters*/

	ADI_EMSI_AUTO_CMD				eAutoCmdType;        /*!< Auto CMD type (CMD12/CMD23)*/

	bool 							bData_Trans;		 /*!< Polls on whether data transfer is in progress*/

	uint16_t						nRca;				 /*!< Relative card address*/

	bool							bControllerRespChk;  /*!<R1 Response check by controller*/

	bool							bInternal_Cmd;		 /*!< When send command and resp acc API is
															  used inside the API*/

	ADI_EMSI_APP_USE				eAppUsage;			 /*!< Application usage*/

	bool 							bBoot_Ack;			 /*!< Whether boot acknowledgement needed from card side*/

	ADI_EMSI_DMA_USED				eDmaUsed;			 /*!< DMA used for the operation*/

	ADI_EMSI_SPEED_MODE				eSpeed_Mode;		 /*!< Application Supplied Speed Mode for EMSI operation*/

	ADI_EMSI_TRANSFER_TYPE			eTransferType;       /*!< Transfer Type Single Block, Multi Block(Predefined, Open ended)*/

	ADI_EMSI_GENERAL_TRANS_TYPE		eGenTransType;		 /*!< Transfer Type Read or Write*/

	uint32_t						nSdmaBufBdary;		 /*!< SDMA buffer boundary*/

	uint16_t                       	nBlkSize;			 /*!< Block Size of the transfer*/

	uint16_t                        nBlkCnt;			 /*!< Block Count for the transfer*/

	bool							bExtSocketUsed;		 /*!< Whether external socket is used for
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  card connection or on board eMMC is used*/

	uint32_t 						nCmdQueDepth;		 /*!< Command queuing depth supported by card*/

	ADI_EMSI_CQE_TASK_DES			*pCQETaskDescriptor; /*!<CQE Task Descriptors*/

	void                            *pCBParam;           /*!< Application supplied callback parameter */

} ADI_EMSI_DEVICE;

/*!\struct ADI_EMSI_INFO
 * This structure contains the pointer to the  eMSI registers.
 * These values do not vary at runtime and set upfront before the execution of the code.
 * This is an internal data structure and not exported to application
 **/
typedef struct
{

	ADI_EMSI_TypeDef *const			pEMSIRegs;          /*!< Pointer to base address of EMSI Memory Mapped Registers */

	uint32_t						StatusIntID;        /*!< EMSI0 status interrupt ID */

	uint32_t						WakupIntID;         /*!< EMSI0 wakeup interrupt ID */

	ADI_EMSI_DEVICE					*pDevice;           /*!< Pointer to EMSI device instance data */

} ADI_EMSI_INFO;

/*! \var bGeneral_Transfer
 * General transfer used. In abort transfer API to poll
 * on whether General transfer was used or not.
 **/
volatile bool bGeneral_Transfer;

/*! \var bInit_Sequence_Delay
 * Initialization sequence delay (74 clock cycles delay) check.
 **/
volatile bool bInit_Sequence_Delay;

 /*! \var InternalResp[4]
  * used for storing response for internally send command
  **/
ADI_CACHE_ALIGN uint32_t InternalResp[4];

/*! \var InternalSDStatus[16];
 * Stores the contents of the SD status (512 bits - 64 bytes) - for SD card
 **/
ADI_CACHE_ALIGN uint32_t InternalSDStatus[16];

/*! \var SwitchFuncStatus[16]
 * Stores the contents of the SWITCH function status (512 bits - 64 bytes) - for SD card
 **/
ADI_CACHE_ALIGN uint32_t SwitchFuncStatus[16];

/*! \var Ext_CsdRegBuff[512]
 * Stores the contents of the EXT_CSD register (512 bytes) - for eMMC
 */
ADI_CACHE_ALIGN uint8_t	Ext_CsdRegBuff[512];

/*! \var vDmaDescriptors[NUM_DMA_DESCRIPTORS]
 * Stores the ADAM2 descriptors (attributes and buffer address)
 */
ADI_CACHE_ALIGN ADI_EMSI_DMADescriptor vDmaDescriptors[NUM_DMA_DESCRIPTORS];

/*! \var vADMA3DesPair[ADMA3_DMA_DESCRIPTORS_PAIRS]
 * Stores the ADAM3 Command descriptors and ADMA2 Descriptors (Required for ADMA3 operations)
 */
ADI_CACHE_ALIGN ADI_EMSI_ADAM3DescriptorPair vADMA3DesPair[ADMA3_DMA_DESCRIPTORS_PAIRS];

/*! \var vADMA3IntDescriptor[ADMA3_DMA_DESCRIPTORS_PAIRS]
 * Stores the ADAM3 Integrated Descriptors
 */
ADI_CACHE_ALIGN ADI_EMSI_ADAM3IntegratedDes	vADMA3IntDescriptor[ADMA3_DMA_DESCRIPTORS_PAIRS];

/* Local function definitions visible to EMSI driver only */
/* EMSI Status interrupt handler */
static void EmsiStatusHandler(uint32_t SID, void *pCBParam);

/* EMSI Wakeup interrupt handler */
static void EmsiWakeupHandler(uint32_t SID, void *pCBParam);

/* Reset the eMSI registers */
static void ResetEmsiRegs(ADI_EMSI_TypeDef *pEMSIRegs);

/* Reset and Clear eMSI Interrupt registers to default */
static void ResetEmsiIntRegs(ADI_EMSI_TypeDef *pEMSIRegs);

/*eMSI Controller Setup Sequence*/
static ADI_EMSI_RESULT ControllerSetup(ADI_EMSI_TypeDef *pEMSIRegs, ADI_EMSI_DEVICE *pDevice);

/*Reset the command and data lines*/
static void ResetCmdDataLine(ADI_EMSI_INFO *pInfo);

/*Set the Descriptors and configure the DMA*/
static ADI_EMSI_RESULT DmaConfig(ADI_EMSI_INFO *pInfo, uint32_t *pBuffer);

/*Update the buffer address in system address register for SDMA mode*/
static ADI_EMSI_RESULT SdmaUpdateBuffAddress(ADI_EMSI_INFO *pInfo, uint32_t *pBuffer);

/*Sending predefined commands based on transfer type*/
static ADI_EMSI_RESULT Predefined_Command_Send(ADI_EMSI_INFO *pInfo, uint32_t Start_Add);

/*Check for transfer complete*/
static ADI_EMSI_RESULT CheckXfrStatus(ADI_EMSI_INFO *pInfo);

/*Set the type of card that is connected to interface.*/
static ADI_EMSI_RESULT SetCardType(ADI_EMSI_INFO *pInfo, ADI_EMSI_CARD_TYPE eCardType);

/*Starting or stopping the clock to card.*/
static ADI_EMSI_RESULT ClktoCard(ADI_EMSI_INFO *pInfo, bool bEnable);

/* Enable or Disable EMSI normal interrupts status and error interrupt status*/
static ADI_EMSI_RESULT SetInterruptMask(
					   ADI_EMSI_INFO *pInfo,
					   uint16_t nIstatmask,
					   uint16_t nErrstatmask,
					   uint16_t nInterruptmask,
					   uint16_t nErrInterruptmask,
					   bool bEnable);

/*Setup and configures eMSI bus clock frequency by set up dividers and synchronizing the clock.*/
static ADI_EMSI_RESULT SetClock(ADI_EMSI_INFO *pInfo,uint16_t nDivisor);

/* Set the flags for CMD register*/
static ADI_EMSI_RESULT CmdRegSet(ADI_EMSI_INFO *pInfo,ADI_EMSI_CMD_PARA *pCmdparameters);

/*Data Cache Flush*/
static void FlushDataCache(uint32_t *pBufferStart, uint32_t *pBufferEnd, bool invalidate);

/*Abort Command sequence*/
static ADI_EMSI_RESULT Abort_Command_Seq(ADI_EMSI_INFO *pInfo);

/*Storing the SD card status in buffer(for SD card).*/
static ADI_EMSI_RESULT GetSdStatus(ADI_EMSI_INFO *pInfo);

/*Switching to SD speed mode(for SD card)*/
static ADI_EMSI_RESULT SdHighSpeedMode(ADI_EMSI_INFO *pInfo,bool bHs_Timings);

/*Enabling/Disabling command queuing from card side*/
static ADI_EMSI_RESULT SetCmdQue(ADI_EMSI_INFO *pInfo,bool bCmdQueEnable);

/*Enable or Disable eMSI Command queuing interrupts status and interrupts signals to core*/
static ADI_EMSI_RESULT SetCqInterruptMask(ADI_EMSI_INFO *pInfo, uint32_t nCqIstatmask, uint32_t nCqInterruptmask,bool bEnable);

/*Getting EXT_CSD register content using CMD8*/
static ADI_EMSI_RESULT GetExtCsdcontent(ADI_EMSI_INFO *pInfo);

/*Command queuing disable from card and controller end*/
static ADI_EMSI_RESULT CqeCleanup(ADI_EMSI_INFO *pInfo);

/*eMSI clock cycles delay sequence*/
static ADI_EMSI_RESULT EmsiDelay(ADI_EMSI_INFO *pInfo, uint32_t DelayTermsClockCycles);

/*Timer callback*/
static void TimerEmsiCallback(void *pCBParam, uint32_t nEvent, void *pArg);

#if defined (ADI_DEBUG)
/* Validate the given Task Handle */
static ADI_EMSI_RESULT ValidateHandle (ADI_EMSI_INFO *pInfo);
#endif

#endif /* _ADI_EMSI_DEF_SC59x_H_ */
