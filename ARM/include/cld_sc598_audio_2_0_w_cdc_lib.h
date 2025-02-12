#ifndef __CLD_AUDIO_2_0_W_CDC_LIB__
#define __CLD_AUDIO_2_0_W_CDC_LIB__
/*=============================================================================
    FILE:           cld_sc598_audio_2_0_w_cdc_lib.h

    DESCRIPTION:    CLD Audio v2.0 with CDC peripheral Library.

    Copyright (c) 2023 Closed Loop Design, LLC

    This software is supplied "AS IS" without any warranties, express, implied
    or statutory, including but not limited to the implied warranties of fitness
    for purpose, satisfactory quality and non-infringement. Closed Loop Design LLC
    extends you a royalty-free right to use, reproduce and distribute this source
    file as well as executable files created using this source file for use with
    Analog Devices SC5xx family processors only. Nothing else gives you
    the right to use this source file.

==============================================================================*/
/*!
 * @file      cld_sc598_audio_2_0_w_cdc_lib.h
 * @brief     CLD USB Audio 2.0 library header file.
 *
 * @details
 *            This file contains the functionality needed to interface with the
 *            CLD USB Audio 2.0 and CDC/ACM library using the SC598's Cortex-A55 processor.
 *
 *            Copyright (c) 2023 Closed Loop Design, LLC
 *
 *            This software is supplied "AS IS" without any warranties, express, implied
 *            or statutory, including but not limited to the implied warranties of fitness
 *            for purpose, satisfactory quality and non-infringement. Closed Loop Design LLC
 *            extends you a royalty-free right to use, reproduce and distribute this source
 *            file as well as executable files created using this source file for use with
 *            Analog Devices SC5xx family processors only. Nothing else gives you
 *            the right to use this source file.
 *
 */

#ifndef CLD_NULL
/**
 * Defines a CLD library NULL value
 */
#define CLD_NULL    0
#endif

#if defined(_LANGUAGE_C) || defined(__cplusplus) || (defined(__GNUC__) && !defined(__ASSEMBLER__))

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * CLD_Time typedef
 * used by CLD time keeping functions.
 */
typedef unsigned long CLD_Time;

/**
 * CLD_RV enum defines the CLD library function return codes.
 */
typedef enum
{
    CLD_SUCCESS = 0,    /*!< Process completed successfully. */
    CLD_FAIL = 1,       /*!< Process failed. */
    CLD_ONGOING = 2     /*!< Process isn't complete. */
} CLD_RV;

/**
 * CLD_Boolean enum defines the CLD library's boolean values.
 */
typedef enum
{
    CLD_FALSE = 0,      /*!< True. */
    CLD_TRUE  = 1,      /*!< False. */
} CLD_Boolean;

/**
 * CLD Library USB Events
 */
typedef enum
{
    CLD_USB_CABLE_CONNECTED = 0,      /*!< USB Cable Connected. */
    CLD_USB_CABLE_DISCONNECTED,       /*!< USB Cable Disconnected. */
    CLD_USB_ENUMERATED_CONFIGURED_FS, /*!< USB Device configured (USB Set Configuration command received enabling the device) at Full Speed. */
    CLD_USB_ENUMERATED_CONFIGURED_HS, /*!< USB Device configured (USB Set Configuration command received enabling the device) at High Speed. */
    CLD_USB_UN_CONFIGURED,            /*!< USB Device unconfigured (USB Set Configuration command received disabling the device). */
    CLD_USB_BUS_RESET,                /*!< USB Bus Reset. */
} CLD_USB_Event;

/**
 * Value returned by the User code to select how the requested transfer should
 * be handled.
 */
typedef enum
{
    CLD_USB_TRANSFER_ACCEPT = 0,                        /*!< Accept the requested transfer using
                                                             the supplied CLD_USB_Transfer_Params. */
    CLD_USB_TRANSFER_PAUSE,                             /*!< Pause the current transfer. The USB
                                                             endpoint will be Nak'ed until the
                                                             appropriate 'resume' function is
                                                             called. This is used to throttle
                                                             the USB host. */
    CLD_USB_TRANSFER_DISCARD,                           /*!< Discard the transfer. For OUT requests
                                                             the received data is discarded. For IN
                                                             requests the device will return a
                                                             zero length packet. */
    CLD_USB_TRANSFER_STALL,                             /*!< Stall the transfer request. */
} CLD_USB_Transfer_Request_Return_Type;

/**
 * Value returned by the User code to notify the USB library if the received data
 * was valid.
 */
typedef enum
{
    CLD_USB_DATA_GOOD = 0,                              /*!< Received data is good.
                                                             For Control OUT requests this
                                                             allows the Status Stage to
                                                             complete. */
    CLD_USB_DATA_BAD_STALL,                             /*!< Received data is bad so stall the endpoint
                                                             For Control OUT requests this
                                                             stalls the Status Stage, for
                                                             other endpoint types the
                                                             next OUT packet will be stalled. */
} CLD_USB_Data_Received_Return_Type;

/**
 * Value returned by the USB library to notify the User code if the requested data
 * will be transmitted.
 */
typedef enum
{
    CLD_USB_TRANSMIT_SUCCESSFUL = 0,                    /*!< The requested USB transfer is being processed. */
    CLD_USB_TRANSMIT_FAILED,                            /*!< The requested USB transfer failed. */
    CLD_USB_TRANSMIT_FAILED_MISALIGNED,                 /*!< The requested USB transfer failed because the specified
                                                             memory location isn't 32-bit aligned. */
} CLD_USB_Data_Transmit_Return_Type;

/**
 * Value returned by the USB library to notify the User code if the requested data
 * will be received.
 */
typedef enum
{
    CLD_USB_RECEIVE_SUCCESSFUL = 0,                    /*!< The requested USB transfer is being processed. */
    CLD_USB_RECEIVE_FAILED,                            /*!< The requested USB transfer failed. */
    CLD_USB_RECEIVE_FAILED_MISALIGNED,                 /*!< The requested USB transfer failed because the specified
                                                             memory location isn't 32-bit aligned. */
    CLD_USB_RECEIVE_FAILED_NUM_BYTES,                  /*!< The requested USB transfer failed because the specified
                                                             num_bytes is not a multiple of the Endpoint Max Packet Size. */
} CLD_USB_Data_Receive_Return_Type;

/**
 * USB transfer request parameters used to tell the USB library how to handle the
 * current transfer.
 */
typedef struct
{
    unsigned long num_bytes;                            /*!< The number of bytes to
                                                             transmit or receive depending
                                                             on the transfer direction (IN/OUT) */
    unsigned char * p_data_buffer;                      /*!< Pointer to the data to transmit
                                                             (IN Transfer) or a pointer to
                                                             the buffer to store the received
                                                             data (OUT transfer). */

    /* Function pointers used by the USB Library to notify the User code when certain events occur.
       These function pointers can be set to NULL if the User does not want to be notified.*/
    union
    {                                                   /*!< Function called when the data
                                                             has been received. */
        CLD_USB_Data_Received_Return_Type (*fp_usb_out_transfer_complete)(unsigned int num_bytes);
        void (*fp_usb_in_transfer_complete) (void);     /*!< Function called when the requested
                                                             data has been transmitted. */
    }callback;/*!< Function pointers used by the USB Library to notify the User code when certain events occur.
                   These function pointers can be set to NULL if the User does not want to be notified.*/

    void (*fp_transfer_aborted_callback) (void);        /*!< Function called if the requested
                                                             transfer is aborted. */
    CLD_Time transfer_timeout_ms;                       /*!< Transfer Timeout in milliseconds
                                                             (0 - Disables the timeout) */
} CLD_USB_Transfer_Params;

/**
 * Parameters used to read or write the USB Phy registers
 */
typedef struct
{
    CLD_Boolean write;                   /*!< TRUE = register write, FALSE = register read */
    unsigned char reg_addr;             /*!< Register Address */
    unsigned char v_ctrl;               /*!< ULPI Vendor Control Register Address */
    unsigned char reg_data;             /*!< For a Read this variable will contain the read data.
                                             For a write this variable contains the data to be written. */
} CLD_USB_PHY_Access_Params;


typedef struct
{
    float desired_data_rate;                            /*!< Feeback value in kHz (for example use 44.1 for 44.1kHz). */
    void (*fp_usb_in_transfer_complete) (void);         /*!< Function called when the requested
                                                             data has been transmitted. */
    void (*fp_transfer_aborted_callback) (void);        /*!< Function called if the requested
                                                             transfer is aborted. */
    CLD_Time transfer_timeout_ms;                       /*!< Transfer Timeout in milliseconds
                                                             (0 - Disables the timeout) */
} CLD_USB_Audio_Feedback_Params;

/**
 * Parameters used to configure the CDC Serial Data Bulk
 * endpoints.
 */
typedef struct
{
    unsigned char endpoint_number;                      /*!< USB Endpoint Number */
    unsigned short max_packet_size_full_speed;          /*!< Full-Speed max packet size */
    unsigned short max_packet_size_high_speed;          /*!< High-Speed max packet size */
} CLD_Serial_Data_Bulk_Endpoint_Params;


#pragma pack (1)

/**
 * USB CDC Line Coding structure (refer to: USB CDC PTSM SubClass Rev 1.2 pg 24).
 */
typedef struct
{
    unsigned int data_terminal_rate;                    /*!< CDC Data Terminal Rate in bits per second. */
    unsigned char num_stop_bits;                        /*!< CDC Number of stop bits
                                                            0 = 1 stop bit
                                                            1 = 1.5 stop bits
                                                            2 = 2 stop bits */
    unsigned char parity;                               /*!< CDC Parity setting
                                                            0 = None
                                                            1 = Odd
                                                            2 = Even
                                                            3 = Mark
                                                            4 = Space */
    unsigned char num_data_bits;                        /*!< CDC number of data bits
                                                            (Only 5, 6, 7, 8 and 16 allowed) */
} CLD_CDC_Line_Coding;

/* CDC Line Coding #defines */
#define CLD_CDC_LINE_CODING_1_STOP_BITS                 0 /*<! CDC Line Coding: 1 Stop Bit */
#define CLD_CDC_LINE_CODING_1_5_STOP_BITS               1 /*<! CDC Line Coding: 1.5 Stop Bits */
#define CLD_CDC_LINE_CODING_2_STOP_BITS                 2 /*<! CDC Line Coding: 2 Stop Bits */

#define CLD_CDC_LINE_CODING_PARITY_NONE                 0 /*<! CDC Line Coding: No Parity */
#define CLD_CDC_LINE_CODING_PARITY_ODD                  1 /*<! CDC Line Coding: Odd Parity */
#define CLD_CDC_LINE_CODING_PARITY_EVEN                 2 /*<! CDC Line Coding: Even Parity */
#define CLD_CDC_LINE_CODING_PARITY_MARK                 3 /*<! CDC Line Coding: Force Mark */
#define CLD_CDC_LINE_CODING_PARITY_SPACE                4 /*<! CDC Line Coding: Force Space */

#define CLD_CDC_LINE_CODING_NUM_DATA_BITS_5             5 /*<! CDC Line Coding: 5 data Bits */
#define CLD_CDC_LINE_CODING_NUM_DATA_BITS_6             6 /*<! CDC Line Coding: 6 data Bits */
#define CLD_CDC_LINE_CODING_NUM_DATA_BITS_7             7 /*<! CDC Line Coding: 7 data Bits */
#define CLD_CDC_LINE_CODING_NUM_DATA_BITS_8             8 /*<! CDC Line Coding: 8 data Bits */
#define CLD_CDC_LINE_CODING_NUM_DATA_BITS_16            16 /*<! CDC Line Coding: 16 data Bits */

/*
 * USB CDC Control Line State bitfield (refer to: USB CDC PTSM SubClass Rev 1.2 pg 25).
 */
typedef struct
{
    union
    {
        struct
        {
            unsigned short dte_present  : 1;            /*<! Indicates to DCE if DTE is present or not.
                                                             This signal corresponds to V.24 signal 108/2
                                                             and RS-232 signal DTR.
                                                                0 - Not Present
                                                                1 - Present  */
            unsigned short activate_carrier : 1;        /*<! Carrier control for half duplex modems.
                                                             This signal corresponds to V.24 signal
                                                             105 and RS-232 signal RTS.
                                                                0 - Deactivate carrier
                                                                1 - Activate carrier
                                                             The device ignores the value of this
                                                             bit when operating in full duplex mode.  */
            unsigned short reserved         : 14;
        } bits;
        unsigned short state;
    } u;
} CLD_CDC_Control_Line_State;

/**
 * USB Audio v2.0 Audio Stream Isochronous Data Endpoint Descriptor
 * (refer to: USB Device Class Definition of Audio Devices v 2.0 section 4.10.1.2).
 */
typedef struct
{
    unsigned char  b_length;                            /*!< Descriptor length */
    unsigned char  b_descriptor_type;                   /*!< Descriptor type = 0x25 (Class Specific Endpoint) */
    unsigned char  b_descriptor_subtype;                /*!< Descriptor subtype = 0x01 (Endpoint General) */
    unsigned char  bm_attributes;                       /*!< Identifies the attributes supported by this endpoint
                                                             Bit 7: Max Packets Only */
    unsigned char bm_controls;                          /*!< - Bits 0-1: Pitch Control
                                                             - Bits 2-3: Data Overrun Control
                                                             - Bits 4-5: Data Underrun Control */
    unsigned char  b_lock_delay_units;                  /*!< Specifies the units used for w_lock_delay
                                                             - 0 = undefined
                                                             - 1 = Milliseconds
                                                             - 2 = Decoded PCM samples */
    unsigned short w_lock_delay;                        /*!< Specifies the time it takes the endpoint
                                                             to reliably lock its internal clock recovery */
} CLD_Audio_2_0_Audio_Stream_Data_Endpoint_Descriptor;

#pragma pack ()
/**
 * Parameters used to configure the USB Audio Stream Isochronous Interface.
 */
typedef struct
{
    /* Parameters used for the Isochronous Endpoint Descriptor */
    unsigned char endpoint_number;                      /*!< Isochronous endpoint number */
    unsigned short max_packet_size_full_speed;          /*!< Isochronous endpoint full-speed max packet size */
    unsigned short max_packet_size_high_speed;          /*!< Isochronous endpoint high-speed max packet size */
    unsigned char b_interval_full_speed;                /*!< Isochronous endpoint full-speed bInterval */
    unsigned char b_interval_high_speed;                /*!< Isochronous endpoint high-speed bInterval */

    /* Parameters used for Audio Stream Interface Descriptor
      (refer to: USB Device Class Definition of Audio Devices v 2.0 section 4.9.2) */
    unsigned char b_terminal_link;                      /*!< The Terminal ID connected to the Isochronous endpoint */
    unsigned char b_format_type;                        /*!< Format Type of the Audio Streaming interface */
    unsigned long bm_formats;                           /*!< Supported Audio data format(s) */
    unsigned char b_nr_channels;                        /*!< Number of physical channels in the audio channel cluster */
    unsigned int  bm_channel_config;                    /*!< Describes the spatial location of
                                                           the physical channel(s) */
    unsigned char i_channel_config;                     /*!< Index of the string descriptor describing the
                                                           first physical channel.  These strings
                                                           should be defined in the p_user_string_descriptor_table. */

    unsigned char * p_encoder_descriptor;               /*!< Optional encoder descriptor pointer (set to CLD_NULL of not used)
                                                             (refer to: USB Device Class Definition of Audio Devices v 2.0 sections 4.9.4) */
    unsigned char * p_decoder_descriptor;               /*!< Optional decoder descriptor pointer (set to CLD_NULL of not used)
                                                             (refer to: USB Device Class Definition of Audio Devices v 2.0 sections 4.9.5) */

    unsigned char * p_format_descriptor;                /*!< Pointer to the Interface's format descriptor.
                                                             (refer to: USB Device Class Definition for Audio Data Formats, part of the Audio Devices v 2.0 specification) */
    CLD_Audio_2_0_Audio_Stream_Data_Endpoint_Descriptor * p_audio_stream_endpoint_data_descriptor; /*!< Pointer to the Audio Stream endpoint data descriptor */
} CLD_Audio_2_0_Stream_Interface_Params;

/**
 * Parameters used to configure the optional Audio control Interrupt IN endpoint
 */
typedef struct
{
    unsigned char endpoint_number;                      /*!< Interrupt endpoint number */
    unsigned char b_interval_full_speed;                /*!< Interrupt endpoint full-speed bInterval */
    unsigned char b_interval_high_speed;                /*!< Interrupt endpoint high-speed bInterval */
} CLD_Audio_2_0_Control_Interrupt_Params;

/**
 * Parameters used to configure the optional USB Audio OUT stream's rate feedback Isochonous IN endpoint
 */
typedef struct
{
    /* Parameters used for the Isochronous Rate Feedback Endpoint Descriptor */
    unsigned short max_packet_size_full_speed;          /* Isochronous endpoint full-speed max packet size */
    unsigned short max_packet_size_high_speed;          /* Isochronous endpoint high-speed max packet size */
    unsigned char b_interval_full_speed;                /* Isochronous endpoint full-speed bInterval */
    unsigned char b_interval_high_speed;                /* Isochronous endpoint high-speed bInterval */
} CLD_Audio_2_0_Rate_Feedback_Params;

/**
 * USB Audio v2.0 Set and Get Control Endpoint bRequest values
 */
typedef enum
{
    CLD_REQ_CURRENT     = 0x01,                         /*!< Used to get/set the current value of an audio Unit */
    CLD_REQ_RANGE       = 0x02,                         /*!< Used to get the support range of an audio Unit */
    CLD_REQ_MEMORY      = 0x03,                         /*!< Used for Memory access requests */
} CLD_Audio_2_0_Requests;

/**
 * Parameters passed to the fp_audio_set_req_cmd and fp_audio_get_req_cmd functions used to
 * identify the active command.
 */
typedef struct
{
    CLD_Audio_2_0_Requests          req;                    /*!< bRequest value of the active command */
    CLD_Boolean                     recipient_is_interface; /*!< CLD_TRUE - The request was sent to the interface specified in interface_or_endpoint_num
                                                                 CLD_FALSE - The request was sent to the streaming endpoint specified in interface_or_endpoint_num */
    unsigned char                   entity_id;              /*!< Target entity ID (Unit ID, Terminal ID, etc) */
    unsigned char                   interface_or_endpoint_num;  /*!< Target interface or endpoint number depending on
                                                                     the entity. */
    unsigned short                  setup_packet_wValue;    /*!< Setup Packet wValue */
} CLD_Audio_2_0_Cmd_Req_Parameters;

/**
 * Table Entry used to specify user defined USB strings. For example these strings could be used along with
   the Unit and Terminal descriptor to define a descriptive string for an entity.
  */
typedef struct
{
    unsigned char string_index;                         /*!< String Descriptor index. */
    unsigned char * p_string;                           /*!< Null terminated string */
} CLD_Audio_2_0_w_CDC_Lib_User_String_Descriptors;


/**
 * CLD USB Library initialization parameters
 */
typedef struct
{
    unsigned short vendor_id;                       /*!< USB Vendor ID */
    unsigned short product_id;                      /*!< USB Product ID */

    unsigned char usb_bus_max_power;                /*!< USB Configuration Descriptor
                                                         bMaxPower value (0 = self powered).
                                                         See USB 2.0 Section 9.6.3. */
    unsigned short device_descriptor_bcdDevice;     /*!< USB Device Descriptor bcdDevice
                                                         value. See USB 2.0 section 9.6.1. */

    unsigned char phy_hs_timeout_calibration;       /*!< High Speed USB timeout PHY calibration value
                                                         See ADSP-SC59x Hw Reference Manual bits 2:0 of the USBC_CFG register*/
    unsigned char phy_fs_timeout_calibration;       /*!< High Speed USB timeout PHY calibration value
                                                         See ADSP-SC59x Hw Reference Manual bits 2:0 of the USBC_CFG register*/

    CLD_Boolean  phy_delay_req_after_ulip_chirp_cmd;/* Some ULPI PHYs like the Microchip USB334x series require
                                                       a delay between the ULPI register write that initiates
                                                       the HS Chirp and the subsequent transmit command,
                                                       otherwise the HS Chirp does not get executed
                                                       and the deivce enumerates in FS mode. Setting this
                                                       parameter to CLD_TRUE enables this delay. */
    /**.
     *  User defined function used to initialize and reset the USB Phy.
     * @return CLD_ONGOING results in this function getting additional runtime.
     * @return CLD_SUCCESS USB Phy initialized successfully.
     * @return CLD_FAIL Phy initialization failed, USB library initialization failure.*/
    CLD_RV (*fp_init_usb_phy) (void);

    unsigned char audio_control_category_code;          /*!< Audio Control Interface Header Descriptor bCategory code
                                                           (refer to: USB Device Class Definition of Audio Devices v 2.0 section 4.7.2)*/

    unsigned char * p_unit_and_terminal_descriptors;    /*!< Pointer to the beginning of the Unit
                                                             and Terminal Descriptors (these descriptors need to be packed). */
    unsigned short  unit_and_terminal_descriptors_length; /*!< The total length of the above Unit and Terminal Descriptors. */

    CLD_Audio_2_0_Stream_Interface_Params * p_audio_streaming_rx_interface_params; /*!< Pointer to the interface parameters
                                                                                              for the Audio Stream RX Isochronous OUT interface.
                                                                                              <b>Set to CLD_NULL if not required.</b>*/

    /* Pointer to the Optional Audio Stream RX Isochronous OUT's rate Feedback Isochonous IN interface.
       Set to CLD_NULL if not required.*/
    CLD_Audio_2_0_Rate_Feedback_Params * p_audio_rate_feedback_rx_params;

    CLD_Audio_2_0_Stream_Interface_Params * p_audio_streaming_tx_interface_params; /*!< Pointer to the interface parameters
                                                                                              for the Audio Stream TX Isochronous IN interface.
                                                                                              <b>Set to CLD_NULL if not required.</b>*/

    /**
     * Set Command received function pointer.
     * Function called when an USB Audio 2.0 Set Request is received. The User code should set
     * p_transfer_data->p_data_buffer to address where to store the data for the command
     * described by p_req_params.
     * @param p_req_param - Pointer to the CLD_SC58x_Audio_2_0_Cmd_Req_Parameters for the current Control Endpoint Set request
     * @param p_transfer_data - Pointer to the CLD_USB_Transfer_Params structure used to define where the command's data should be stored.
     * @return Notifies the CLD Library if the received command should be accepted, stalled, paused, or discarded */
    CLD_USB_Transfer_Request_Return_Type (*fp_audio_set_req_cmd) (CLD_Audio_2_0_Cmd_Req_Parameters * p_req_params, CLD_USB_Transfer_Params * p_transfer_data);

    /**
     * Get Command received function pointer.
     * Function called when an USB Audio 2.0 Get Request is received. The User code should
     * set p_transfer_data->p_data_buffer and p_transfer_data->num_bytes to describe the
     * data to return to the Host.
     * @param p_req_param - Pointer to the CLD_SC58x_Audio_2_0_Cmd_Req_Parameters for the current Control Endpoint Get request
     * @param p_transfer_data - Pointer to the CLD_USB_Transfer_Params structure used to define the data to send to the USB Host.
     * @return Notifies the CLD Library if the received command should be accepted, stalled, paused, or discarded */
    CLD_USB_Transfer_Request_Return_Type (*fp_audio_get_req_cmd) (CLD_Audio_2_0_Cmd_Req_Parameters * p_req_params, CLD_USB_Transfer_Params * p_transfer_data);

    /**
     * Function called when the Isochronous OUT endpoint interface is enabled/disabled by
     * the USB Host using the Set Interface Alternate Setting parameter.
     * @param enabled - CLD_TRUE = Enabled, CLD_FALSE = Disabled
     */
    void (*fp_audio_streaming_rx_endpoint_enabled) (CLD_Boolean enabled);

    /**
     * Function called when the Isochronous IN endpoint interface is enabled/disabled by
     * the USB Host using the Set Interface Alternate Setting parameter.
     * @param enabled - CLD_TRUE = Enabled, CLD_FALSE = Disabled
     */
    void (*fp_audio_streaming_tx_endpoint_enabled) (CLD_Boolean enabled);

    /**
     * Pointer to the endpoint parameters for the serial data RX Bulk OUT endpoint.
     */
    CLD_Serial_Data_Bulk_Endpoint_Params * p_serial_data_rx_endpoint_params;

    /**
     * Pointer to the endpoint parameters for the serial data TX Bulk IN endpoint.
     */
    CLD_Serial_Data_Bulk_Endpoint_Params * p_serial_data_tx_endpoint_params;

    /**
     * Function called when an CDC Send Encapsulated Command request is received. The User code
     * should set p_transfer_data->p_data_buffer to address where to store the
     * encapsulated command data.
     * @param p_transfer_data - Pointer to the CLD_USB_Transfer_Params structure used to define where the encapsulated data from the USB Host is stored.
     * @return Notifies the CLD Library if the received command should be accepted, stalled, paused, or discarded
      */
    CLD_USB_Transfer_Request_Return_Type (*fp_cdc_cmd_send_encapsulated_cmd) (CLD_USB_Transfer_Params * p_transfer_data);

    /**
     * Function called when an CDC Get Encapsulated Command request is received.
     * The User code should set p_transfer_data->p_data_buffer and
     * p_transfer_data->num_bytes to describe the data to return to the Host.
     * @param p_transfer_data - Pointer to the CLD_USB_Transfer_Params structure used to define the encapsulated data to send to the USB Host.
     * @return Notifies the CLD Library if the received command should be accepted, stalled, paused, or discarded
     */
    CLD_USB_Transfer_Request_Return_Type (*fp_cdc_cmd_get_encapsulated_resp) (CLD_USB_Transfer_Params * p_transfer_data);

   /**
    * Function called when an CDC Set Line Coding Command is received. The User code
    * should configure its UART parameters based on the CLD_CDC_Line_Coding
    * structure passed to the function.
    *
    * <b>Set to CLD_NULL if not supported.</b>
    * @param p_line_coding - Pointer to the CLD_CDC_Line_Coding structure with the line coding defined by the USB Host.
    * @return Notifies the CLD Library if the received command should be accepted, stalled, paused, or discarded
    */
    CLD_USB_Data_Received_Return_Type (*fp_cdc_cmd_set_line_coding) (CLD_CDC_Line_Coding * p_line_coding);

   /**
    * Function called when an CDC Get Line Coding Command is received. The User code
    * should load the CLD_CDC_Line_Coding structure passed to the function with
    * the current line coding values.
    *
    * <b>Set to CLD_NULL if not supported.</b>
    * @param p_line_coding - Pointer to the CLD_CDC_Line_Coding structure with the line coding values to send to the USB Host.
    * @return CLD_SUCCESS- line coding values updated, CLD_FAIL - Line coding values were not updated.
    */
    CLD_RV (*fp_cdc_cmd_get_line_coding) (CLD_CDC_Line_Coding * p_line_coding);

   /**
    * Function called when an CDC Set Control Line State Command is received.
    * The User code should process the CLD_CDC_Control_Line_State structure
    * passed to the function.
    *
    * <b>Set to CLD_NULL if not supported.</b>
    * @param p_control_line_state - Pointer to the CLD_CDC_Control_Line_State structure with the control line state defined by the USB Host.
    * @return Notifies the CLD Library if the received command should be accepted, stalled, paused, or discarded
    */
    CLD_USB_Data_Received_Return_Type (*fp_cdc_cmd_set_control_line_state) (CLD_CDC_Control_Line_State * p_control_line_state);

   /**
    * Function called when an CDC Send Break Command is received.
    * The User code should generate a RS-232 style break for the number of milliseconds
    * passed to the function.
    * If the duration is set to 0xFFFF the device should send a break until this
    * function is called again with a duration of 0.
    *
    * <b>Set to CLD_NULL if not supported.</b>
    * @param duration - the requested break duration in milliseconds.
    * @return Notifies the CLD Library if the received command should be accepted, stalled, paused, or discarded
    */
    CLD_USB_Data_Received_Return_Type (*fp_cdc_cmd_send_break) (unsigned short duration);

    unsigned char support_cdc_network_connection;               /*!< 1 = Network Connection Notification supported
                                                                     0 = Network Connection Notification not supported */
    unsigned short cdc_class_bcd_version;                       /*!< CDC Class version in BCD */
    unsigned char  cdc_class_control_protocol_code;             /*!< CDC Protocol Code defined in USB CDC Rev 1.2 (Table 5 Pg 13). */


    const char * p_usb_string_manufacturer;         /*!< Pointer to the Manufacturer string descriptor (set to CLD_NULL if not used) */
    const char * p_usb_string_product;              /*!< Pointer to the Product string descriptor (set to CLD_NULL if not used) */
    const char * p_usb_string_serial_number;        /*!< Pointer to the Serial Number string descriptor (set to CLD_NULL if not used) */
    const char * p_usb_string_configuration;        /*!< Pointer to the Configuration string descriptor (set to CLD_NULL if not used) */
    const char * p_usb_string_audio_control_interface;          /*!< Pointer to the Audio Control Interface string descriptor (set to CLD_NULL if not used) */
    const char * p_usb_string_audio_streaming_out_interface;    /*!< Pointer to the Audio Streaming OUT interface string descriptor (set to CLD_NULL if not used) */
    const char * p_usb_string_audio_streaming_in_interface;     /*!< Pointer to the Audio Streaming IN interface string descriptor (set to CLD_NULL if not used) */
    const char * p_usb_string_communication_class_interface;    /*!< Pointer to the CDC Control interface string descriptor (set to CLD_NULL if not used) */
    const char * p_usb_string_data_class_interface;             /*!< Pointer to the CDC Data interface string descriptor (set to CLD_NULL if not used) */

    unsigned char user_string_descriptor_table_num_entries; /*!< Number of entries in the
                                                                 p_user_string_descriptor_table array
                                                                 (Set to 0 if not used) */
  /**
   * Array of USB specified String Descriptors.(Set to CLD_NULL if not used.)
   */
    CLD_Audio_2_0_w_CDC_Lib_User_String_Descriptors * p_user_string_descriptor_table;

    unsigned short usb_string_language_id;              /*!< USB String Descriptor Language ID code
                                                           (0x0409 = English US). */

    /**
     * Function called when one of the defined USB events occurs.
     * @param event - Defined USB Event
     *                              - CLD_USB_CABLE_CONNECTED
     *                              - CLD_USB_CABLE_DISCONNECTED
     *                              - CLD_USB_ENUMERATED_CONFIGURED
     *                              - CLD_USB_UN_CONFIGURED
     *                              - CLD_USB_BUS_RESET
     */
    void (*fp_cld_usb_event_callback) (CLD_USB_Event event);

     /**
         * Function called when the library has a status to report.
          * @param status_code - Status code returned by the CLD library using the fp_cld_lib_status callback function.
          * @param p_additional_data - optional additional data returned by the CLD library using the fp_cld_lib_status callback function.
          * @param additional_data_size - optional additional data size returned by the CLD library using the fp_cld_lib_status callback function.
         */
    void (*fp_cld_lib_status) (unsigned short status_code, void * p_additional_data, unsigned short additional_data_size);

} CLD_SC598_Audio_2_0_w_CDC_Lib_Init_Params;

/* CLD Audio 2.0 w/CDC Library function that should be called once every 125 microseconds. */
extern void cld_time_125us_tick (void);
/* CLD Audio 2.0 w/CDC Library USB interrupt service routine processing function */
extern void cld_usb0_isr_callback (void);

/**
 * Function called to get the current time reference in milliseconds.
 * @return the current time reference.
 */
extern CLD_Time cld_time_get(void);
/**
 * Function called to get the number of milliseconds that have elapsed since the specified time
 * reference was set using cld_time_get.
 * @param time - time reference used to calculate the elapsed time in milliseconds
 * @return the elapsed time.
 */
extern CLD_Time cld_time_passed_ms(CLD_Time time);

/**
 * Function called to get the current time reference in 125 microsecond increments.
 * @return the current time reference.
 */
extern CLD_Time cld_time_get_125us (void);
/**
 * Function called to get the number of 125 microsecond increments that have elapsed since the
 * specified time reference was set using cld_time_get_125us.
 * @param time - time reference used to calculate the elapsed time in 125 microsecond increments
 * @return the elapsed time.
 */
extern CLD_Time cld_time_passed_125us (CLD_Time time);

/**
 * Initializes the CLD Audio v2.0 with CDC library. This function needs to be called until
 * CLD_SUCCESS or CLD_FAIL is returned.  If CLD_FAIL is returned there was a
 * problem initializing the library.
 * @param cld_sc598_audio_2_0_w_cdc_lib_params - Pointer to the CLD library initialization parameters,
 * @return The status of the library initialization (CLD_SUCCESS, CLD_FAIL, CLD_ONGOING).
 */
extern CLD_RV cld_sc598_audio_2_0_w_cdc_lib_init (CLD_SC598_Audio_2_0_w_CDC_Lib_Init_Params * p_lib_params);

/**
 * CLD Audio v2.0 with CDC library mainline function. <b>The CLD library mainline function is
 * required and should be called in each iteration of the main program loop.</b>
 */
extern void cld_sc598_audio_2_0_w_cdc_lib_main (void);

/**
 * Function used to receive audio data using the Isochronous OUT
 * endpoint. If the Isochronous OUT endpoint is busy this
 * function will return CLD_FAIL.
 * @param p_rx_data - Pointer to the CLD_USB_Transfer_Params
 *                  structure describing the audio data being
 *                  received.
 * @return If the requested reception was successfully
 *         initiated.
 */
extern CLD_USB_Data_Receive_Return_Type cld_audio_2_0_lib_receive_stream_data (CLD_USB_Transfer_Params * p_rx_data);

/**
 * Function used to transmit audio data using the Isochronous IN endpoint.
 * If the Isochronous IN endpoint is disabled this function will return
 * CLD_USB_TRANSMIT_FAILED.
 * @param p_transfer_data - Pointer to the CLD_USB_Transfer_Params structure describing the audio data being transmitted.
 * @return If the requested transmission was successfully initiated or if it failed..
 */
extern CLD_USB_Data_Transmit_Return_Type cld_audio_2_0_lib_transmit_audio_data (CLD_USB_Transfer_Params * p_transfer_data);

/**
 * Function used to transmit audio rate feedback using the optional rate feedback Isochronous IN endpoint.
 * @param p_transfer_data - Pointer to the CLD_USB_Audio_Feedback_Params structure describing the feedback data being transmitted.
 * @return If the requested transmission was successfully initiated or if it failed..
 */
extern CLD_USB_Data_Transmit_Return_Type cld_audio_2_0_lib_transmit_audio_rate_feedback_data (CLD_USB_Audio_Feedback_Params * p_transfer_data);

/**
 * Function used to transmit serial data using the Bulk IN endpoint.
 * @param p_transfer_data - Pointer to the CLD_USB_Transfer_Params structure describing the serial data being transmitted.
 * @return If the requested transmission was successfully initiated or if it failed..
 */
extern CLD_USB_Data_Transmit_Return_Type cld_cdc_lib_transmit_serial_data (CLD_USB_Transfer_Params * p_transfer_data);

/**
 * Function used to receive serial using the Bulk OUT
 * endpoint. If the Bulk OUT endpoint is busy this
 * function will return CLD_FAIL.
 * @param p_rx_data - Pointer to the CLD_USB_Transfer_Params
 *                  structure describing the serial data being
 *                  received.
 * @return If the requested reception was successfully
 *         initiated.
 */
extern CLD_USB_Data_Receive_Return_Type cld_cdc_lib_receive_serial_data (CLD_USB_Transfer_Params * p_rx_data);

/**
 * Function used to resume a paused control transfer. When this function is called
 * the fp_audio_set_req_cmd or fp_audio_get_req_cmd function
 * will be called depending on which request was paused.  The function can then
 * accept, discard or stall the previously paused transfer.
 */
extern void cld_audio_2_0_w_cdc_lib_resume_paused_control_transfer (void);

/**
 * Connects to the USB Host.
  */
extern void cld_lib_usb_connect (void);
/**
 * Disconnects from the USB Host.
 */
extern void cld_lib_usb_disconnect (void);

/**
 * Reads or Writes to the USB Phy registers.
 * @param p_params - Phy access parameters.
 */
extern CLD_RV cld_lib_access_usb_phy_reg (CLD_USB_PHY_Access_Params * p_params);

/**
 * cld_lib_status_decode will return a null terminated string describing the CLD library status
 * code passed to the function.
 * @param status_code - Status code returned by the CLD library using the fp_cld_lib_status callback function.
 * @param p_additional_data - optional additional data returned by the CLD library using the fp_cld_lib_status callback function.
 * @param additional_data_size - optional additional data size returned by the CLD library using the fp_cld_lib_status callback function.
 * @return Decoded null terminated status string.
 */
extern char * cld_lib_status_decode (unsigned short status_cod, void * p_additional_data, unsigned short additional_data_size);

#ifdef __cplusplus
}
#endif

#endif /* _LANGUAGE_C */
#endif  /* __CLD_AUDIO_2_0_W_CDC_LIB__ */
