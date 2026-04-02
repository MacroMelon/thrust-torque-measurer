/**
 * @file usb.h
 * @author B4KUN
 * @date 10-08-2025
 * @copyright Copyright (c) 2025
 *
 * @brief Provides USB interface
 */
#ifndef USB_H_
#define USB_H_

#include <stdint.h>
#include <stdbool.h>

/** @name USB IN endpoint TX transfer statuses
 *  @brief Possible states of an IN endpoint transmission.
 *  @{
 */
#define TX_READY            0U  /**< Endpoint is ready for a new transmission. */
#define TX_PERFORM          1U  /**< A transmission is currently in progress. */
#define TX_ZLP              2U  /**< Zero-Length Packet (ZLP) is being sent. */
/** @} */

/** @name USB return values
 *  @brief General return codes used by USB driver functions.
 *  @{
 */
#define USB_OK              0U  /**< Operation completed successfully. */
#define USB_BUSY            1U  /**< Resource is busy, try again later. */
#define USB_NOTREADY        2U  /**< USB interface is not ready for the requested operation. */
#define USB_ERROR           3U  /**< An error occurred while performing the operation. */
/** @} */

/** @name USB status flags
 *  @brief Bitmask flags representing current USB device state.
 *  @{
 */
#define USB_RESET_FLAG      (1<<0) /**< USB reset has been received. */
#define USB_ENUDME_FLAG     (1<<1) /**< USB enumeration is complete. */
#define USB_LINECODED_FLAG  (1<<2) /**< Line coding has been set by the host (CDC). */
#define USB_DTR_FLAG        (1<<3) /**< Data Terminal Ready (DTR) signal is active. */

/**
 * @brief USB ready status bitmask.
 *
 * Combination of flags that indicate the USB interface is fully ready
 * for data communication.
 */
#define USB_READY_STATUS    (                           \
                             USB_RESET_FLAG |           \
                             USB_ENUDME_FLAG |          \
                             USB_DTR_FLAG               \
                            )
// USB_LINECODED_FLAG |       \  skip check for linecoded

/** @} */

/**
 * @brief USB CDC line coding structure.
 *
 * Describes the serial communication parameters used by the USB CDC interface.
 * This structure follows the USB CDC specification for the SET_LINE_CODING
 * and GET_LINE_CODING control requests.
 */
typedef struct {
    uint32_t dwDTERate;   /**< Data terminal rate (baud rate). */
    uint8_t  bCharFormat; /**< Stop bits: 0 = 1 bit, 1 = 1.5 bits, 2 = 2 bits. */
    uint8_t  bParityType; /**< Parity: 0 = none, 1 = odd, 2 = even, 3 = mark, 4 = space. */
    uint8_t  bDataBits;   /**< Number of data bits (e.g., 8, 7, etc.). */
} __attribute__((packed)) line_coding_t;

/**
 * @brief USB setup packet structure.
 *
 * Represents the 8-byte setup transaction packet used in USB control transfers.
 * Contains request type, request, and parameters as defined in the USB 2.0 specification.
 */
typedef struct {
    uint8_t  bmRequest; /**< Request characteristics: data transfer direction, type, recipient. */
    uint8_t  bRequest;  /**< Specific request code. */
    uint16_t wValue;    /**< Request-specific value field. */
    uint16_t wIndex;    /**< Request-specific index field (often an interface or endpoint number). */
    uint16_t wLength;   /**< Number of bytes to transfer in the data stage. */
} setup_pkt_t;

/**
 * @brief Initialize the USB peripheral and device.
 *
 * This function performs a complete initialization sequence for the USB device:
 * - Initializes the USB peripheral clocks and GPIO pins by calling USB_InitPeriph().
 * - Initializes the USB core and device registers by calling USB_InitDevice().
 * - Initializes internal endpoint data structures by calling USB_InitEPStructs().
 *
 * @note This function should be called once before starting USB communication.
 */
void USB_Init();

/**
 * @brief Starts USB transmission on the specified IN endpoint.
 *
 * This function initiates the transfer of data via USB endpoint.
 * The transmission algorithm is as follows:
 *
 * 1) If data length is greater than the max packet size (64 bytes for CDC),
 *    the function stores data in the endpoint structure, sends the first
 *    packet of max length, and enables the TX FIFO empty interrupt.
 * 2) When the TX FIFO empty interrupt (USB_OTG_DIEPINT_TXFE) fires, the handler
 *    fills the FIFO with next data chunk using USB_WriteFIFO.
 * 3) When the transfer complete interrupt (USB_OTG_DIEPINT_XFRC) occurs, it means
 *    the previous packet was sent successfully, and USB_HandleTXTransfer is called
 *    again to continue sending the remaining data if any.
 * 4) If the last packet is smaller than max packet size (0 < len < 64),
 *    it is sent as is and transmission finishes.
 *    If the last packet length is 0, a Zero-Length Packet (ZLP) is sent
 *    per USB specification to indicate transfer completion.
 * 5) After transmission completion, the OUT endpoint is re-enabled for reception.
 *
 * @param[in] epnum Endpoint number for transmission.
 * @param[in] data  Pointer to data buffer to send.
 * @param[in] len   Length of data in bytes.
 * @return uint8_t Status code (USB_OK, USB_BUSY, USB_NOTREADY).
 */
uint8_t USB_StartTXTransfer(uint8_t epnum, uint8_t* data, uint16_t len);

/**
 * @brief Prepare an OUT endpoint to receive data from the host.
 *
 * This function sets up the endpoint transfer size and packet count to receive
 * one packet of maximum 64 bytes and enables the endpoint for reception.
 *
 * @note This should be called after processing received data in USB_EP1RXCallBack
 * to resume reception. Until USB_PrepareReceive is called, the host will not
 * send new data if USB_OUTEPSNAK was previously called.
 *
 * @param[in] epnum Endpoint number to prepare for receiving data.
 */
void USB_PrepareReceive(uint8_t epnum);

/**
 * @brief Set the specified OUT endpoint to NAK state.
 *
 * Calling this function causes the USB device to respond with NAK
 * for any incoming OUT transactions on the specified endpoint,
 * effectively pausing the host from sending more data.
 *
 * It is recommended to call this function during USB_EP1RXCallBack after
 * receiving a packet and processing it, if the firmware needs to delay
 * further reception until ready.
 *
 * After processing, call USB_PrepareReceive to re-enable reception.
 *
 * @param[in] epnum Endpoint number to set NAK state.
 */
void USB_OUTEPSNAK(uint8_t epnum);

/**
 * @brief Get current USB device status.
 *
 * @return USB_Status Current USB status flags.
 */
uint32_t USB_GetStatus();

/**
 * @brief Callback for receiving data on endpoint 1.
 *
 * This function is called after the USB_OTG_DOEPINT_XFRC interrupt,
 * which indicates reception of one data packet from the host.
 * Note that this callback is invoked for each received packet,
 * not after the entire transfer completes.
 *
 * @warning You must copy data from RX_buff to your own buffer here,
 * because RX_buff will be overwritten by the next received packet.
 *
 * @param[in] RX_buff Pointer to the received data buffer.
 * @param[in] length Length of the received data in bytes.
 */
void USB_EP1RXCallBack(uint8_t *RX_buff, uint16_t length);

/**
 * @brief Callback called upon completion of a transmission started by USB_StartTXTransfer.
 *
 * This function is called when all data queued for transmission on endpoint 1
 * has been successfully sent to the host.
 */
void USB_EP1TXTransferCompliteCallBack();

/**
 * @brief Callback invoked on CDC "Set Control Line State" request.
 *
 * This callback is called when the host sends a control line state command
 * (e.g., DTR/RTS signals) to the device.
 * Use this callback to handle connection state changes or control device behavior.
 *
 * @param[in] line_state Pointer to the setup packet containing the control line state request.
 */
void USB_CtrlLineStateCallBack(setup_pkt_t *line_state_pkt);

/**
 * @brief Callback invoked on CDC "Set Line Coding" request.
 *
 * This function is called after the host sends new line coding parameters,
 * such as baud rate, parity, stop bits, etc.
 * Use this callback to configure your UART or emulate the requested line coding.
 *
 * @param[in] LineCoding Pointer to the line coding parameters structure.
 */
void USB_SetLinecodingCallBack(line_coding_t *LineCoding);

#endif
