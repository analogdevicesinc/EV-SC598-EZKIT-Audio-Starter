/******************************************************************************
 * @file:    adi_emsi_v1.h
 * @brief:   EMSI driver API header file
 * @details  This is the primary header file for the EMSI driver, which
 *           contains the API declarations, data, and constant definitions used
 *           in the APIs. This file should be included in the application.
 * @version: $Revision: $
 * @date:    $Date: 2021-07-29$
 *****************************************************************************

 Copyright (c) 2022 Analog Devices.  All Rights Reserved.

 This software is proprietary.  By using this software you agree
 to the terms of the associated Analog Devices License Agreement.

 *******************************************************************************/

/** @addtogroup EMSI_Driver EMSI Device Driver
 *  @{
 */

#ifndef __ADI_EMSI_V1_H__
#define __ADI_EMSI_V1_H__


/*==========  I N C L U D E  ==========*/
#include <adi_osal.h>
#include <adi_types.h>
#include <services/int/adi_int.h>
#include "adi_emsi_config_SC59x.h"

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_5_1:"Identifiers shall not rely on the significance of more than 31 characters")
#endif /* _MISRA_RULES */

/*! Amount of memory(In bytes) required by the EMSI driver.
 *  This memory is completely owned by the driver till the end of the operation.*/
#if defined(__ADSPCORTEXA55__)
#define ADI_EMSI_DRIVER_MEMORY_SIZE     (0x70u + (ADI_OSAL_MAX_SEM_SIZE_CHAR)) /* GCC Compiler */
#else
#define ADI_EMSI_DRIVER_MEMORY_SIZE		(0x58u + (ADI_OSAL_MAX_SEM_SIZE_CHAR))  /* SHARC Compiler*/
#endif

/*! EMSI device handle */
typedef void* ADI_EMSI_HANDLE;

/*! EMSI device 0 */
#define ADI_EMSI_0                (0u)

/*!
 *  \enum ADI_EMSI_RESULT
 * Enumeration is used by the driver to indicate the return status of requested operation.
 */
typedef enum
{
	ADI_EMSI_SUCCESS = 0,           	/*!< The API call succeeded. */
	ADI_EMSI_FAILURE,               	/*!< The API call failed. */
	ADI_EMSI_BAD_DEVICE_NUMBER,     	/*!< Invalid DeviceNum argument. */
	ADI_EMSI_BUSY,         		    	/*!< Operation currently incomplete. */
	ADI_EMSI_DATA_TRANS_FINISHED,		/*!< Operation complete. */
	ADI_EMSI_TIMED_OUT,             	/*!< An operation did not complete in the expected time. */
	ADI_EMSI_CRC_ERROR,             	/*!< CRC (Cyclic Redundancy Check) error */
	ADI_EMSI_STARTBIT_ERROR,        	/*!< Start bit error */
	ADI_EMSI_ENDBIT_ERROR,        		/*!< END bit error */
	ADI_EMSI_CMDIDX_ERROR,        		/*!< Command index error */
	ADI_EMSI_BLOCKSIZE_INVALID,     	/*!< Invalid block Size */
	ADI_EMSI_BUS_WIDTH_INVALID,     	/*!< Invalid bus width */
	ADI_EMSI_DATA_LENGTH_INVALID,		/*!< Invalid Data length for read write operation*/
	ADI_EMSI_DATA_END_BIT_ERROR,    	/*!< Data End bit error */
	ADI_EMSI_DATA_CRC_ERROR,        	/*!< Data CRC error. */
	ADI_EMSI_ADMA_ERROR,            	/*!< ADMA error. */
	ADI_EMSI_SEMAPHORE_FAILED,      	/*!< Semaphore operation failed. */
	ADI_EMSI_CMDRESPONSE_ERR,        	/*!< Command response error. */
	ADI_EMSI_INVALID_HANDLE ,       	/*!< Invalid EMSI handle. */
	ADI_EMSI_INTERRUPT_FAILURE,			/*!< Interrupt handler failure*/
	ADI_EMSI_NO_RESPONSE,				/*!< No Response to given command*/
	ADI_EMSI_INVALID_APIUSE,			/*!< API is not valid for selected application use*/
	ADI_EMSI_INVALID_CONFIGURATION,		/*!< Invalid EMSI configuration*/
	ADI_EMSI_CARD_INSERTED,				/*!< Card is inserted into external socket*/
	ADI_EMSI_CARD_REMOVED,				/*!< Card is removed from external socket*/
	ADI_EMSI_QUEUE_DEPTH_INSUFFICIENT,	/*!< Command queuing depth of the card is less than task submitted*/
	ADI_EMSI_TMR_CONFIG_FAILURE,		/*!< Timer configuration failed*/
	ADI_EMSI_PWR_CONFIG_FAILURE			/*!< Power configuration failed*/
} ADI_EMSI_RESULT;


/*!
 *  \enum ADI_EMSI_CARD_TYPE
 * Specifies the kind of card connected to interface.
 */
typedef enum
{
	ADI_EMSI_CARD_TYPE_EMMC = 0,			/*!< Embedded MMC (eMMC) card. */
	ADI_EMSI_CARD_TYPE_SDCARD				/*!< SD card. */
} ADI_EMSI_CARD_TYPE;


/*!
 *  \enum ADI_EMSI_RESP_TYPE
 * Specifies the response type from the  CMD register (Response which is expected from the command).
 */
typedef enum
{
	ADI_EMSI_RESPONSE_TYPE_NONE = 0, 		/*!< No response. */
	ADI_EMSI_RESPONSE_TYPE_LONG,			/*!< 136-bit response. */
	ADI_EMSI_RESPONSE_TYPE_SHORT,    		/*!< 48-bit response. */
	ADI_EMSI_RESPONSE_TYPE_SHORT_CHK_BSY 	/*!< 48-bit response with busy check after response (for R1b type of response). */
} ADI_EMSI_RESP_TYPE;

/*!
 *  \enum ADI_EMSI_SUBCMD_TYPE
 * Specifies the type of command (refer to HRM eMSI chapter for more details).
 */
typedef enum
{
	ADI_EMSI_SUBCMD_TYPE_MAIN = 0,			/*!< Main Command. */
	ADI_EMSI_SUBCMD_TYPE_SUB				/*!< Sub Command. */
} ADI_EMSI_SUBCMD_TYPE;

/*!
 *  \enum ADI_EMSI_CMD_CRC_CHECK
 * Command CRC Check Enable/Disable this enables the eMSI Controller to check the CRC field in the response.
 */
typedef enum
{
	ADI_EMSI_CMD_CRC_CHECK_DISABLE= 0,		/*!< CRC check disabled for response. */
	ADI_EMSI_CMD_CRC_CHECK_ENABLE			/*!< CRC check enabled for response. */
} ADI_EMSI_CMD_CRC_CHECK;


/*!
 *  \enum ADI_EMSI_CMD_IDX_CHECK
 *  Command Index Check enables the eMSI Controller to check the command index field in the response
 *  to verify if it has the same value as the command index.
 */
typedef enum
{
	ADI_EMSI_CMD_IDX_CHECK_DISABLE = 0,		/*!< Command index check disabled for response.*/
	ADI_EMSI_CMD_IDX_CHECK_ENABLE			/*!< Command index check enabled for response. */
} ADI_EMSI_CMD_IDX_CHECK;


/*!
 *  \enum ADI_EMSI_DATA_PRESENT
 * ADI_EMSI_DATA_PRESENT indicates  that the data will/will not be transferred using the DAT line(i.e. Check for data or non-data command).
 */
typedef enum
{
	ADI_EMSI_DATA_PRESENT_FALSE = 0,		/*!< Command without Data. */
	ADI_EMSI_DATA_PRESENT_TRUE				/*!< Command With Data. */
} ADI_EMSI_DATA_PRESENT;


/*!
 *  \enum ADI_EMSI_SND_CMD_TYPE
 * Specifies the type of command to be send to card.
 */
typedef enum
{
	ADI_EMSI_SND_CMD_TYPE_NORMAL = 0,		/*!< Normal command (e.g. CMD1,CMD2,CMD3). */
	ADI_EMSI_SND_CMD_TYPE_ABORT				/*!< Abort command (e.g. CMD12). */
} ADI_EMSI_SND_CMD_TYPE;


/*!
 *  \enum ADI_EMSI_AUTO_CMD
 * Determines use of Auto Command functions.
 */
typedef enum
{
	ADI_EMSI_AUTO_CMD_DIS = 0,				/*!< Auto command disable. */
	ADI_EMSI_AUTO_CMD12EN,					/*!< Auto command12 enable.*/
	ADI_EMSI_AUTO_CMD23EN				    /*!< Auto command23 enable.*/
} ADI_EMSI_AUTO_CMD;


/*!
 *  \enum ADI_EMSI_BUS_WIDTH
 * Specifies the bus width for eMSI operation
 */
typedef enum
{
	ADI_EMSI_BUS_WIDTH_1BIT= 0,				 /*!< 1-bit SDR/Legacy.*/
	ADI_EMSI_BUS_WIDTH_4BIT,				 /*!< 4-bit SDR/DDR/Legacy .*/
	ADI_EMSI_BUS_WIDTH_8BIT					 /*!< 8-bit SDR/DDR/Legacy.*/
} ADI_EMSI_BUS_WIDTH;


/*!
 *  \enum ADI_EMSI_SPEED_MODE
 * Specifies the Speed mode for eMMC operation or SD card operation*/
typedef enum
{
	ADI_EMSI_SPEED_MODE_LEGACY_OR_DSSDR= 0,	/*!< Legacy mode of transfer(eMMC) or
		 	 	 	 	 	 	 	 	 	 	 Default Speed SDR mode of transfer(SD card)(0-25 MHz).*/
	ADI_EMSI_SPEED_MODE_HIGHSPEED_SDR,		/*!< High Speed SDR mode of transfer(eMMC) (0-50 MHz)
												 High Speed SDR mode of transfer(SD card) (0-44 MHz).*/
	ADI_EMSI_SPEED_MODE_HIGHSPEED_DDR		/*!< High Speed DDR mode of transfer(eMMC) (0-50 MHz).*/
} ADI_EMSI_SPEED_MODE;

/*!
 *  \enum ADI_EMSI_TRANSFER_TYPE
 * Transfer type: Single Block or Multi Block (Predefined, Open-ended)
 * and Normal or alternate boot of operation
 */
typedef enum
{
	ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_RD= 0,			/*!< Single Block Read (uses CMD17).*/
	ADI_EMSI_TRANSFER_TYPE_SINGLE_BLOCK_WR,				/*!< Single Block Write (uses CMD24).*/
	ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_PREDEF,		/*!< Multi Block read predefined (uses CMD23->CMD18).*/
	ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_RD_OPEN_ENDED,	/*!< Multi Block read open ended (uses CMD18).*/
	ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_PREDEF,		/*!< Multi Block write predefined (uses CMD23->CMD25).*/
	ADI_EMSI_TRANSFER_TYPE_MULTI_BLOCK_WR_OPEN_ENDED,	/*!< Multi Block write open ended (uses CMD25).*/
	ADI_EMSI_TRANSFER_TYPE_NORMAL_BOOT,					/*!< Normal Boot.*/
	ADI_EMSI_TRANSFER_TYPE_ALTERNATE_BOOT				/*!< Alternate Boot.*/
} ADI_EMSI_TRANSFER_TYPE;

/*!
 *  \enum ADI_EMSI_GENERAL_TRANS_TYPE
 * Transfer type: Read or Write. This enum is used in ADMA2 based general transfers.
 */
typedef enum
{
	ADI_EMSI_GENERAL_TRANS_TYPE_WRITE=0,				/*!< General read transfer type.*/
	ADI_EMSI_GENERAL_TRANS_TYPE_READ					/*!< General write transfer type.*/
} ADI_EMSI_GENERAL_TRANS_TYPE;


/*!
 *  \enum ADI_EMSI_ABORT_TYPE
 *   The Asynchronous abort.
 */
typedef enum
{
	ADI_EMSI_ABORT_TYPE_ASYNC= 0						/*!< Asynchronous abort ending transfer abruptly.*/
} ADI_EMSI_ABORT_TYPE;

/*!
 *  \enum ADI_EMSI_DMA_USED
 *  Specifies the DMA USED for eMSI operation
 */
typedef enum
{
	ADI_EMSI_DMA_USED_SDMA=0,							/*!< SDMA used for transfer, use blocking mode of transfer API*/
	ADI_EMSI_DMA_USED_ADMA2,							/*!< ADMA2 used for transfer, use Non-blocking or
															 Command queuing mode of transfer API*/
	ADI_EMSI_DMA_USED_ADMA3								/*!< ADMA3 used for transfer, use chained transfer API*/
} ADI_EMSI_DMA_USED;


/*!
 *  \enum ADI_EMSI_APP_USE
 *  Specifies the Use of the application.
 *  User can use one of the enum as per the use of application.
 *  In application if user is doing normal data transfer using normal data transfer commands (CMD17,
 *  CMD23, CMD1, CMD2, CMD3 etc.) then user can choose ADI_EMSI_APP_USE_NORMAL_DATATRANSFER or
 *  if user is using application for Boot operation (for performing normal or alternate boot)
 *  then user can choose ADI_EMSI_APP_USE_BOOTING_OPERATION.
 */
typedef enum
{
	ADI_EMSI_APP_USE_NORMAL_DATATRANSFER= 0,				/*!< Used when user performs transfer using Data Transfer
														     *	 commands(CMD17,CMD18,CMD24,CMD25)
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 *   also, identification is performed CMD1->CMD2->CMD3*/

	ADI_EMSI_APP_USE_BOOTING_OPERATION						/*!< Used when user performs booting operation*/
} ADI_EMSI_APP_USE;

/*!
 *  \enum ADI_EMSI_TOUT_CNT_VALUE
 * Specifies the TOUT_CNT (Data Timeout Counter Value) in TO_CTL register.
 */
typedef enum
{
	ADI_EMSI_TOUT_CNT_VALUE0 = 0,		/*!< TOUT_CNT_VALUE = TMCLK*2^13. */
	ADI_EMSI_TOUT_CNT_VALUE1,			/*!< TOUT_CNT_VALUE = TMCLK*2^14. */
	ADI_EMSI_TOUT_CNT_VALUE2,			/*!< TOUT_CNT_VALUE = TMCLK*2^15. */
	ADI_EMSI_TOUT_CNT_VALUE3,			/*!< TOUT_CNT_VALUE = TMCLK*2^16. */
	ADI_EMSI_TOUT_CNT_VALUE4,			/*!< TOUT_CNT_VALUE = TMCLK*2^17. */
	ADI_EMSI_TOUT_CNT_VALUE5,			/*!< TOUT_CNT_VALUE = TMCLK*2^18. */
	ADI_EMSI_TOUT_CNT_VALUE6,			/*!< TOUT_CNT_VALUE = TMCLK*2^19. */
	ADI_EMSI_TOUT_CNT_VALUE7,			/*!< TOUT_CNT_VALUE = TMCLK*2^20. */
	ADI_EMSI_TOUT_CNT_VALUE8,			/*!< TOUT_CNT_VALUE = TMCLK*2^21. */
	ADI_EMSI_TOUT_CNT_VALUE9,			/*!< TOUT_CNT_VALUE = TMCLK*2^22. */
	ADI_EMSI_TOUT_CNT_VALUE10,			/*!< TOUT_CNT_VALUE = TMCLK*2^23. */
	ADI_EMSI_TOUT_CNT_VALUE11,			/*!< TOUT_CNT_VALUE = TMCLK*2^24. */
	ADI_EMSI_TOUT_CNT_VALUE12,			/*!< TOUT_CNT_VALUE = TMCLK*2^25. */
	ADI_EMSI_TOUT_CNT_VALUE13,			/*!< TOUT_CNT_VALUE = TMCLK*2^26. */
	ADI_EMSI_TOUT_CNT_VALUE14,			/*!< TOUT_CNT_VALUE = TMCLK*2^27. */
	ADI_EMSI_TOUT_CNT_VALUE15_RESERVED	/*!< TOUT_CNT_VALUE = Reserved. */
} ADI_EMSI_TOUT_CNT_VALUE;


/*! \enum ADI_EMSI_EVENT
 *  Enumeration of events returned by EMSI driver
 */
typedef enum
{
	ADI_EMSI_NOEVENT = 0,			/*!< No event has occurred yet. */
	ADI_EMSI_EVENT_CMD_COMPLETE,	/*!< CMD_COMPLETE: Command Complete. */
	ADI_EMSI_EVENT_XFER_COMPLETE,	/*!< XFER_COMPLETE: Command Execution is Completed. */
	ADI_EMSI_EVENT_BGAP_EVENT,		/*!< BGAP_EVENT: Transaction Stopped at Block Gap. */
	ADI_EMSI_EVENT_DMA_INTERRUPT,	/*!< DMA_INTERRUPT: DMA Interrupt is Generated. */
	ADI_EMSI_EVENT_BUF_WR_READY,	/*!< BUF_WR_READY: Ready to Write Buffer. */
	ADI_EMSI_EVENT_BUF_RD_READY,	/*!< BUF_RD_READY: Ready to Read Buffer. */
	ADI_EMSI_EVENT_CARD_INSERTION,	/*!< CARD_INSERTION: Card Inserted. */
	ADI_EMSI_EVENT_CARD_REMOVAL,	/*!< CARD_REMOVAL: Card Removed. */
	ADI_EMSI_EVENT_CARD_INTERRUPT,	/*!< CARD_INTERRUPT: Generate Card Interrupt. */
	ADI_EMSI_EVENT_INT_A,			/*!< INT_A (Embedded): Interrupt Enable. */
	ADI_EMSI_EVENT_INT_B,			/*!< INT_B (Embedded): Interrupt Enable. */
	ADI_EMSI_EVENT_INT_C,			/*!< INT_C (Embedded): Interrupt Enable. */
	ADI_EMSI_EVENT_FX_EVENT,		/*!< FX_EVENT: FX Event is Detected. */
	ADI_EMSI_EVENT_CQE_EVENT,		/*!< CQE_EVENT: Command Queuing Event is Detected. */
	ADI_EMSI_EVENT_CMD_TOUT_ERR,	/*!< CMD_TOUT_ERR: Time Out. */
	ADI_EMSI_EVENT_CMD_CRC_ERR,		/*!< CMD_CRC_ERR: CRC Error Generated. */
	ADI_EMSI_EVENT_CMD_END_BIT_ERR, /*!< CMD_END_BIT_ERR: End Bit Error Generated. */
	ADI_EMSI_EVENT_CMD_IDX_ERR,		/*!< CMD_IDX_ERR: Error. */
	ADI_EMSI_EVENT_DATA_TOUT_ERR,	/*!< DATA_TOUT_ERR: Time Out. */
	ADI_EMSI_EVENT_DATA_CRC_ERR,	/*!< DATA_CRC_ERR: Error. */
	ADI_EMSI_EVENT_DATA_END_BIT_ERR,/*!< DATA_END_BIT_ERR: Error. */
	ADI_EMSI_EVENT_CUR_LMT_ERR,		/*!< CUR_LMT_ERR: Power Fail. */
	ADI_EMSI_EVENT_AUTO_CMD_ERR,	/*!< AUTO_CMD_ERR: Error. */
	ADI_EMSI_EVENT_ADMA_ERR,		/*!< ADMA_ERR: Error. */
	ADI_EMSI_EVENT_RESP_ERR,		/*!< RESP_ERR: Error. */
	ADI_EMSI_EVENT_BOOT_ACK_ERR,	/*!< BOOT_ACK_ERR: Error. */
	ADI_EMSI_EVENT_RED,				/*!< Response error detected during CQE*/
	ADI_EMSI_CQE_CLEANUP_FAIL		/*!< CQE clean up failure(Unable to disable CQE)*/
}ADI_EMSI_EVENT;

/*!
 *  \enum ADI_EMSI_DATA_XFER_DIR
 * Specifies the data transfers direction .
 */
typedef enum
{
	ADI_EMSI_DATA_XFER_DIR_WRITE = 0,
	ADI_EMSI_DATA_XFER_DIR_READ
} ADI_EMSI_DATA_XFER_DIR;

/*!
 *  \enum ADI_EMSI_TASK_PRIORITY
 * Specifies the task priority for command queuing operation.
 */
typedef enum
{
	ADI_EMSI_TASK_PRIORITY_LOW = 0,
	ADI_EMSI_TASK_PRIORITY_HIGH
}ADI_EMSI_TASK_PRIORITY;

/*!\struct ADI_EMSI_CMD_PARA
 * This structure contains value of CMD register  bit fields which
 * will be used while sending command to card.
 **/
typedef struct
{
	/** Response type */
	ADI_EMSI_RESP_TYPE eRespType;
	/**Sub command type */
	ADI_EMSI_SUBCMD_TYPE eSubCmdType;
	/**CRC check enabled/disabled for the response */
	ADI_EMSI_CMD_CRC_CHECK eCmdCrcCheck;
	/**CMD index check enabled/disabled for the response */
	ADI_EMSI_CMD_IDX_CHECK eCmdIdxCheck;
	/**Command used for data transfer */
	ADI_EMSI_DATA_PRESENT eDataPresent;
	/**Command type */
	ADI_EMSI_SND_CMD_TYPE eCmdType;
}ADI_EMSI_CMD_PARA;


/*!\struct ADI_EMSI_CHAINED_TRANSFER_INPUT
 * This structure contains the inputs for ADMA3 based transfers. User gives input about Block size,
 * Block count, Start address of the card from where user wants to read or write data,
 * data transfer direction, buffer for writing and reading data.
 */
typedef struct
{

	uint32_t 						nBlkCntAdma3;			/*!< Block count*/

	uint32_t						nBlksizeAdma3;			/*!< Block size */

	uint32_t 						nStartAddressAdma3;		/*!< Start address of the card */

	ADI_EMSI_DATA_XFER_DIR			eDataDirectionAdma3;    /*!< Direction of data transfer */

	void							*pBufferAdma3;			/*!< Buffer address */

} ADI_EMSI_CHAIN_TRANSFER_INPUT;

/*! \struct ADI_EMSI_CQE_TASK_DES
 * CQE Task descriptors. These descriptors used in scheduling the tasks.
 **/
typedef struct
{
	uint32_t DES0;				/*!<Stores Task Parameters Field (task priority, data direction etc.), attributes
	 	 	 	 	 	 	 	 Block count*/

	uint32_t DES1;				/*!<Store Argument(card sector address) for the command CMD45*/

	uint32_t DES2;				/*!< Stores descriptors attributes and data length ADMA2 transfer descriptor*/

#if defined(__ADSPARM__)
	uint32_t DES3;				/*!< Stores buffer address ARM Core ADMA2 transfer descriptor*/
#else
	void* DES3;					/*!< Stores buffer address SHARC Core ADMA2 transfer descriptor*/
#endif


}ADI_EMSI_CQE_TASK_DES;


/*!\struct ADI_EMSI_CQE_INPUT
 * This structure contains the inputs for Command queuing based transfers. User gives input about Block size,
 * Block count, Start address of the card from where user wants to read or write data,
 * data transfer direction, buffer for writing and reading data, and task priority.
 */
typedef struct
{

	uint32_t 						nBlkCnt;				/*!< Block count*/

	uint32_t 						nStartAddress;			/*!< Start address of the card */

	ADI_EMSI_DATA_XFER_DIR			eDataDirection;    		/*!< Direction of data transfer */

	void							*pBuffer;				/*!< Buffer address */

	ADI_EMSI_TASK_PRIORITY			eTaskPriority;			/*!< Task Priority (High priority or less priority)*/

} ADI_EMSI_CQE_INPUT;


/*!
 *  \enum ADI_EMSI_SDMA_BUF_BDARY
 * Specifies the SDMA Buffer Boundary. These enum specify the size of contiguous buffer in system memory.
 * The SDMA transfer waits at every boundary specified by these fields and
 * the eMSI Controller generates the DMA interrupt to request the Host Driver to update the
 * SDMA System Address register.
 */
typedef enum
{
	ADI_EMSI_SDMA_BUF_BDARY_4K = 0,		/*!< Buffer Boundary = 4KB(4096 Bytes). */
	ADI_EMSI_SDMA_BUF_BDARY_8K,			/*!< Buffer Boundary = 8KB(8192 Bytes). */
	ADI_EMSI_SDMA_BUF_BDARY_16K,		/*!< Buffer Boundary = 16KB(16384 Bytes). */
	ADI_EMSI_SDMA_BUF_BDARY_32K,		/*!< Buffer Boundary = 32KB(32768 Bytes). */
	ADI_EMSI_SDMA_BUF_BDARY_64K,		/*!< Buffer Boundary = 64KB(65536 Bytes). */
	ADI_EMSI_SDMA_BUF_BDARY_128K,		/*!< Buffer Boundary = 128KB(131072 Bytes). */
	ADI_EMSI_SDMA_BUF_BDARY_256K,		/*!< Buffer Boundary = 256KB(262144 Bytes). */
	ADI_EMSI_SDMA_BUF_BDARY_512K		/*!< Buffer Boundary = 512KB(524288 Bytes). */
} ADI_EMSI_SDMA_BUF_BDARY;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/******************************EMSI APIs available for application to use**********************/

/* Opens EMSI device with required memory for EMSI */
ADI_EMSI_RESULT adi_emsi_Open(
				uint8_t  nDeviceNum,
				ADI_EMSI_CARD_TYPE eCardType,
				void *pMemory,
				uint32_t nMemSize,
				ADI_EMSI_HANDLE*  phInfo
				);

/* Set the Relative Card Address (RCA)*/
ADI_EMSI_RESULT adi_emsi_SetRca(
				ADI_EMSI_HANDLE const hInfo,
				uint16_t nRca
				);

/*Set the application usage*/
ADI_EMSI_RESULT adi_emsi_SetAppUse(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_APP_USE eAppUse,
				bool bBootAck
				);

/* Closes the EMSI handle */
ADI_EMSI_RESULT adi_emsi_Close(
				ADI_EMSI_HANDLE const hInfo
				);

/*Change eMSI bus clock frequency.*/
ADI_EMSI_RESULT adi_emsi_ChangeClockFreq(
				ADI_EMSI_HANDLE const hInfo,
				uint16_t nDivFreq
				);

/*Send a command to the card.*/
ADI_EMSI_RESULT adi_emsi_SendCommand(
				ADI_EMSI_HANDLE const hInfo,
				uint8_t nCmd_Idx,
				uint32_t nArg,
				ADI_EMSI_CMD_PARA *pCmdParameters
				);

/*Getting response access back to application*/
ADI_EMSI_RESULT adi_emsi_GetResponse(
				ADI_EMSI_HANDLE const hInfo,
				uint32_t*  pRespAcc
				);

/*Select Deselect the card using CMD7*/
ADI_EMSI_RESULT adi_emsi_SelectCard(
				ADI_EMSI_HANDLE const hInfo,
				bool bSelDesel
				);

/*Set the bus width for EMSI operations.*/
ADI_EMSI_RESULT adi_emsi_SetBusWidth(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_BUS_WIDTH eWidth
				);

/*Configuring speed mode for eMSI operations.*/
ADI_EMSI_RESULT adi_emsi_SetSpeedMode(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_SPEED_MODE eSpeedMode
				);

/* Set the block size for upcoming transfer(s).*/
ADI_EMSI_RESULT adi_emsi_SetBlkSze(
				ADI_EMSI_HANDLE const hInfo,
				uint16_t nBlkSize
				);

/* Registers a callback event to notify the application of any EMSI event */
ADI_EMSI_RESULT adi_emsi_RegisterCallback(
				ADI_EMSI_HANDLE const hInfo,
				ADI_CALLBACK const pfCallback,
				void *const pCBParam
				);

/*Auto Command issuing (CMD12/CMD23)*/
ADI_EMSI_RESULT adi_emsi_AutoCmd(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_AUTO_CMD eAutoCmd
				);

/* Sets DMA type for an upcoming eMSI transfer(s).*/
ADI_EMSI_RESULT adi_emsi_SetDmaType(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_DMA_USED eDmaType,
				ADI_EMSI_SDMA_BUF_BDARY eSdmaBufBdary
				);

/*Perform a Blocking transfer from the eMSI.*/
ADI_EMSI_RESULT adi_emsi_Blocking_Transfer(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_TRANSFER_TYPE eTranstype,
				uint16_t nBlkCnt,
				uint32_t nStart_Address,
				void *pBuffer
				);


/*Perform a Non-blocking transfer from the eMSI.*/
ADI_EMSI_RESULT adi_emsi_NonBlocking_Transfer(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_TRANSFER_TYPE eTranstype,
				uint16_t nBlkCnt,
				uint32_t nStart_Address,
				void *pBuffer
				);

/*Performing transfer abort in Non-blocking multiblock transfer in open ended mode*/
ADI_EMSI_RESULT adi_emsi_AbortTransfer(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_ABORT_TYPE eAborttype
				);

/*Set whether external socket is used for card.*/
ADI_EMSI_RESULT adi_emsi_SetCardConnector(
				ADI_EMSI_HANDLE const hInfo,
				bool bExtSocket
				);

/*Perform a General transfers from the eMSI.*/
ADI_EMSI_RESULT adi_emsi_General_Transfer(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_GENERAL_TRANS_TYPE eGenTransType,
				uint16_t nBlkCnt,
				void *pBuffer
				);

/*Chained descriptor based transfers*/
ADI_EMSI_RESULT adi_emsi_Chained_Transfer(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_CHAIN_TRANSFER_INPUT *pAdma3InputArray
				);

/*Configuring Wakeup Control Register*/
ADI_EMSI_RESULT adi_emsi_SetWakeUpCntrl(
				ADI_EMSI_HANDLE const hInfo,
				bool bCardInsert,
				bool bCardRemoval
				);

/*Command queuing based transfers*/
ADI_EMSI_RESULT adi_emsi_CmdQueTransfers(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_CQE_TASK_DES *pCqeDes,
				uint8_t MaxTasksScheduled
				);

/*Disable CRC status check for bus testing procedure*/
ADI_EMSI_RESULT adi_emsi_DisableDataCrcCheck(
				ADI_EMSI_HANDLE const hInfo,
				bool bEnable
				);

/*Preset register usage*/
ADI_EMSI_RESULT adi_emsi_SetPresetRegUse(
				ADI_EMSI_HANDLE const hInfo,
				bool bEnable\
				);

/*Enable or Disable response check by controller.*/
ADI_EMSI_RESULT adi_emsi_ControllerRespChek(
				ADI_EMSI_HANDLE const hInfo,
				bool bEnable
				);

/*To add tuning support for correct sampling of data.*/
ADI_EMSI_RESULT adi_emsi_SetTuning(
				ADI_EMSI_HANDLE const hInfo,
				bool bTuning
				);

/*To change eMMC QoS priority.*/
ADI_EMSI_RESULT adi_emsi_SetPriority(
				ADI_EMSI_HANDLE const hInfo,
				bool bChangeQosPrio
				);

/*Reset eMMC device using reset_n signal*/
ADI_EMSI_RESULT adi_emsi_ResetEmmc(
				ADI_EMSI_HANDLE const hInfo,
				bool bEnableRst
				);

/*Bus Test */
ADI_EMSI_RESULT adi_emsi_BusTest(
				ADI_EMSI_HANDLE const hInfo,
				ADI_EMSI_BUS_WIDTH eWidthBusTest
				);

/*CQE control API*/
ADI_EMSI_RESULT adi_emsi_CqeCntrl(
				ADI_EMSI_HANDLE const hInfo,
				bool bEnableHalt,
				uint32_t nTaskClear,
				bool bEnableCqe,
				bool bCqrmem,
				uint8_t nUpdateIntcTh
				);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */
#endif /* __ADI_EMSI_V1_H__ */

/** @}*/
