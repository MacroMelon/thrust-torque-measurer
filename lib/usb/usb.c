/**
 * @file usb.c
 * @author B4KUN
 * @date 10-08-2025
 * @copyright Copyright (c) 2025
 *
 * @brief USB core
 */
#include <usb.h>
#include <descriptor.h>
#include <string.h>

#define STM32F4xx

#if defined(STM32H7xx)
    #include "stm32h7xx.h"
#elif defined(STM32F4xx)
    #include "stm32f4xx.h"
#endif

/**
 * @cond INTERNAL
 * @name USB Register Access Macros
 * @brief Macros to access USB OTG FS peripheral registers.
 * @{
 */
#define USB_PCGCCTL    *(__IO uint32_t *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)   /**< USB power and clock gating control register. */
#define USB_DEVICE     ((USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))   /**< Pointer to USB device registers. */
#define USB_INEP(i)    ((USB_OTG_INEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))  /**< Pointer to specified USB IN endpoint registers. */
#define USB_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE))) /**< Pointer to specified USB OUT endpoint registers. */
#define USB_DFIFO(i)   *(__IO uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))  /**< Access to specified USB data FIFO register. */
/** @} */
/** @endcond */

/** @cond INTERNAL
 *  @name USB bRequest values
 *  @brief Standard USB request codes used in control transfers (bmRequestType).
 *  @{
 */
#define BREQ_GET_STATUS             0U  /**< Get device, interface, or endpoint status. */
#define BREQ_CLEAR_FEATURE          1U  /**< Clear a specific feature. */
#define BREQ_SET_FEATURE            3U  /**< Set a specific feature. */
#define BREQ_SET_ADDRESS            5U  /**< Assign a USB address to the device. */
#define BREQ_GET_DESCRIPTOR         6U  /**< Retrieve a descriptor from the device. */
#define BREQ_SET_DESCRIPTOR         7U  /**< Set or update a descriptor. */
#define BREQ_GET_CONFIGURATION      8U  /**< Get the current device configuration value. */
#define BREQ_SET_CONFIGURATION      9U  /**< Set the device configuration. */
/** @} */
/** @endcond */

/** @cond INTERNAL
 *  @brief Endpoint transmission state structure.
 *
 * Holds the state of an ongoing USB IN endpoint transmission, including
 * status, data buffer pointer, total length, and number of bytes already sent.
 */
typedef struct {
    uint8_t  txStatus;      /**< Current transmission status (e.g., ready, busy, in progress). */
    uint8_t *txData;        /**< Pointer to the transmission data buffer. */
    uint16_t txDataLen;     /**< Total length of the data to transmit, in bytes. */
    uint16_t txBytesSent;   /**< Number of bytes already transmitted. */
} ep_tx_transfer_t;
/** @endcond */

/**
 * @brief USB device status flags bitmask.
 *
 * Holds current status flags representing the USB device state.
 */
static uint32_t USB_Status = 0;

/**
 * @brief Current USB setup packet.
 *
 * Stores the latest USB control transfer setup packet received from the host.
 */
static setup_pkt_t curr_setup_pkt = {};

/**
 * @brief USB OUT endpoint receive buffers.
 *
 * Double-buffered arrays for data received from the host on OUT endpoints.
 * Two buffers of 64 bytes each to allow ping-pong buffering.
 */
static uint8_t RX_buff[2][64] = {};

/**
 * @brief Length of data currently received in EP1's RX buffer.
 */
static uint16_t EP1_RX_buff_len = 0;

/**
 * @brief Buffer for the setup stage data.
 *
 * Temporarily holds data associated with the USB control setup stage.
 * @{
 */
static uint8_t setup_stage_buff[128] = {};
static uint16_t setup_stage_len = 0;
/** @} */

/**
 * @brief USB CDC line coding settings.
 *
 * Stores current serial line parameters (baud rate, parity, stop bits, data bits)
 * as configured by the host.
 */
static line_coding_t lineCoding;

/**
 * @brief Endpoint transmission state array.
 *
 * Holds transmission state information for each USB IN endpoint.
 */
static ep_tx_transfer_t EndPoint[EP_COUNT];

static void USB_InitPeriph();
static void USB_InitDevice();
static void USB_StartDevice();

static void USB_ResetHandler();
static void USB_EnudmeHandler();
static void USB_EpOutHandler();
static void USB_EpInHandler();
static void USB_SetupStageHandler(setup_pkt_t *request);
static void USB_StdDevReq(setup_pkt_t *request);

static void USB_HandleTXTransfer(uint8_t epnum);
static void USB_ReadFIFO(uint8_t epnum, uint8_t *dst, uint16_t pkt_size);
static void USB_WriteFIFO(uint8_t epnum, uint8_t *src, uint16_t pkt_size);
static void USB_SendZLP(uint8_t epnum);
static void USB_CoreRST();

/** @internal
 * @defgroup Initialization Initialization
 * Initialization of peripherals and variables
 * @{
 */

void USB_Init() {
	USB_InitPeriph();
	USB_InitDevice();

	//Initialize endpoint control structures for all endpoints
    for(uint32_t i = 0; i < EP_COUNT; i++) {
        EndPoint[i].txStatus = TX_READY;
        EndPoint[i].txData = NULL;
        EndPoint[i].txBytesSent = 0;
        EndPoint[i].txDataLen = 0;
    }

    USB_StartDevice();
}

/**
 * @brief Initialize USB peripheral clocks and GPIO pins.
 *
 * This function configures the necessary GPIO pins for USB operation,
 * enables the USB peripheral clock, and waits for the USB core to become idle.
 *
 * The implementation differs depending on the target MCU:
 * - For STM32H7xx:
 *   - Configures PA11 and PA12 for USB Full-Speed pins (DM and DP) with AF10.
 *   - Enables the USB voltage detector.
 *   - Enables the USB peripheral clock on AHB1.
 * - For STM32F4xx:
 *   - Configures PA11 and PA12 for USB Full-Speed pins (DM and DP) with AF10.
 *   - Enables the USB peripheral clock on AHB2.
 *
 * @note This function must be called before USB core initialization.
 */
static void USB_InitPeriph() {
#if defined(STM32H7xx)
	RCC->D2CCIP2R &= ~RCC_D2CCIP2R_USBSEL;
	RCC->D2CCIP2R |= RCC_D2CCIP2R_USBSEL;

	//Enable the USB voltage detector
	PWR->CR3 |= PWR_CR3_USB33DEN;

    //USB_OTG_FS GPIO Configuration
    //PA11     ------> USB_OTG_FS_DM
    //PA12     ------> USB_OTG_FS_DP
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;

    // PA11/PA12 AF10 USB
    GPIOA->MODER &= ~((3 << GPIO_MODER_MODE11_Pos) | (3 << GPIO_MODER_MODE12_Pos));
    GPIOA->MODER |=  ((2 << GPIO_MODER_MODE11_Pos) | (2 << GPIO_MODER_MODE12_Pos));
    GPIOA->AFR[1] &= ~((0xF << GPIO_AFRH_AFSEL11_Pos) | (0xF << GPIO_AFRH_AFSEL12_Pos));
    GPIOA->AFR[1] |=  ((0xA << GPIO_AFRH_AFSEL11_Pos) | (0xA << GPIO_AFRH_AFSEL12_Pos));
    GPIOA->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED11_Pos) | (3 << GPIO_OSPEEDR_OSPEED12_Pos));

    //Peripheral clock enable
    RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGHSEN;

    while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
#elif defined(STM32F4xx)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (0x02 << GPIO_MODER_MODER11_Pos) | (0x02 << GPIO_MODER_MODER12_Pos);
    GPIOA->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED11_Pos) | (0x03 << GPIO_OSPEEDR_OSPEED12_Pos);
    GPIOA->AFR[1] |= (0x0A << GPIO_AFRH_AFSEL11_Pos) | (0x0A << GPIO_AFRH_AFSEL12_Pos);

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL))
    	continue;
#endif
}

/**
 * @brief Initialize the USB device core and endpoints.
 *
 * This function configures the USB peripheral registers to prepare the device
 * for operation in USB device mode. It performs the following main steps:
 * - Disables global USB interrupts during configuration.
 * - Selects Full-Speed embedded PHY and performs a core soft reset.
 * - Sets the device mode in USB core registers.
 * - Clears endpoint FIFO configurations.
 * - Performs a soft disconnect to reset the bus connection state.
 * - Configures VBUS sensing and session valid overrides depending on MCU family.
 * - Restarts the PHY clock and sets the USB core speed to Full-Speed.
 * - Clears all pending device interrupts and disables interrupt masks.
 * - Resets IN and OUT endpoint control, transfer size, and interrupt status registers.
 * - Enables required USB interrupts for device mode operation (reset, enumeration done, IN/OUT EP interrupts).
 * - Configures FIFO sizes for Rx and Tx FIFOs.
 * - Initializes endpoint 0 OUT endpoint transfer parameters to receive SETUP packets.
 *
 * @note FIFO sizes and endpoint configuration are tailored for a typical CDC device.
 */
void USB_InitDevice() {
    uint32_t i;

    // Disable global Interrupts
    USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;

    // Select FS Embedded PHY
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    // Reset after a PHY select
    USB_CoreRST();

    // USB Set current mode
    USB_OTG_FS->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD); // clear mode
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD; // set device mode
    while (((USB_OTG_FS->GINTSTS) & 0x1U) != 0);

    for (i = 0U; i < 15U; i++) {
        USB_OTG_FS->DIEPTXF[i] = 0U;
    }

    // Soft disconnect
    USB_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;

#if defined(STM32H7xx)
    // Deactivate VBUS Sensing B
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

    // B-peripheral session valid override enable
    USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
#elif defined(STM32F4xx)
    // Disable VBUS sensing
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
#endif

    // Restart the Phy Clock
    USB_PCGCCTL = 0U;

    // Set Core speed to Full speed mode
    USB_DEVICE->DCFG |= 3U;

    // Clear all pending Device Interrupts
    USB_DEVICE->DIEPMSK = 0U;
    USB_DEVICE->DOEPMSK = 0U;
    USB_DEVICE->DAINTMSK = 0U;

    for (i = 0U; i < 9; i++) {
        USB_INEP(i)->DIEPCTL = 0U;
        USB_INEP(i)->DIEPTSIZ = 0U;
        USB_INEP(i)->DIEPINT  = 0xFB7FU;
    }

    for (i = 0U; i < 9; i++) {
        USB_OUTEP(i)->DOEPCTL = 0U;
        USB_OUTEP(i)->DOEPTSIZ = 0U;
        USB_OUTEP(i)->DOEPINT  = 0xFB7FU;
    }

    USB_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

    // Disable all interrupts
    USB_OTG_FS->GINTMSK = 0U;

    // Clear any pending interrupts
    USB_OTG_FS->GINTSTS = 0xBFFFFFFFU;

    // Enable the common interrupts
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

    // Enable interrupts matching to the Device mode ONLY
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBRST    	| // USB Reset from Host
                           USB_OTG_GINTMSK_ENUMDNEM  	| // Enumeration done
                           USB_OTG_GINTMSK_IEPINT    	| //IN endpoint interrupt
                           USB_OTG_GINTMSK_OEPINT;		  // OUT endpoint interrupt

    // Soft disconnect
    // In case PHY is stopped, ensure to ungate and restore the PHY clock
    USB_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

    // Set FIFO size for each endpoint
    USB_OTG_FS->GRXFSIZ = 0x80;  // RX FIFO size
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (0x40 << USB_OTG_TX0FD_Pos) | 0x80;  // TX FIFO size for EP0
    USB_OTG_FS->DIEPTXF[0] = (0x80 << USB_OTG_TX0FD_Pos) | (0x80 + 0x40); // TX FIFO size for EP1

    // Initialize EP0 OUT endpoint: 1 packet, 3 * 8 bytes (setup packets)
    USB_OUTEP(0)->DOEPTSIZ = 0;
    USB_OUTEP(0)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19)); 			// Set packet count = 1
    USB_OUTEP(0)->DOEPTSIZ |= 64;                                  				// Max packet size
    USB_OUTEP(0)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT;            				// Setup packet count = 3
    USB_OUTEP(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA); 	// Clear NAK and enable EP0
}

/**
 * @brief Enable USB device operation and interrupts.
 *
 * This function enables global USB interrupts in the core and configures
 * the NVIC for the USB OTG FS interrupt with a priority of 6.
 * It also powers up the USB transceiver and performs a soft connect to
 * signal the host that the device is ready.
 */
static void USB_StartDevice() {
    // Enable global Interrupts
    USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

    // Initialize the NVIC
    NVIC_SetPriority(OTG_FS_IRQn, 6);
    NVIC_EnableIRQ(OTG_FS_IRQn);

    // Activate the USB Transceiver
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN;

    // Soft connect
    USB_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
}

/** @} */ // end of Initialization




/** @internal
 * @defgroup Interrupt_handler Interrupt Handler
 * Interrupt handler
 * @{
 */

/**  @internal
 * @brief USB OTG FS interrupt handler.
 *
 * This function handles all USB OTG FS global interrupts.
 * It checks that the controller is in device mode,
 * then processes pending interrupt sources including:
 *  - USB Reset interrupt
 *  - Enumeration done interrupt
 *  - RX FIFO non-empty interrupt (data or setup packets)
 *  - OUT endpoint interrupt
 *  - IN endpoint interrupt
 *
 * Interrupt flags are cleared after handling.
 * Specific event handlers are called for each interrupt type.
 *
 * @note
 *  This handler must be linked as the interrupt vector in the startup file.
 *  It is **crucial** that the function name matches exactly
 *  the name in the startup vector table (e.g. `OTG_FS_IRQHandler`),
 *  otherwise USB interrupts will not be serviced.
 *
 * @see USB_ResetHandler(), USB_EnudmeHandler(), USB_ReadFIFO(), USB_EpOutHandler(), USB_EpInHandler()
 */
void OTG_FS_IRQHandler() {
    uint32_t int_source = USB_OTG_FS->GINTSTS;

    // Ensure controller is in device mode
    if ((USB_OTG_FS->GINTSTS) & 0x1U)
        return;

    // Avoid spurious interrupts
    if (int_source == 0U)
        return;

    // USB Reset interrupt
    if (int_source & USB_OTG_GINTSTS_USBRST) {
        USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_USBRST; // Clear interrupt
        USB_ResetHandler();
    }

    // Enumeration done interrupt (after reset)
    if (int_source & USB_OTG_GINTSTS_ENUMDNE) {
        USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_ENUMDNE; // Clear interrupt
        USB_EnudmeHandler();
    }

    // RX FIFO non-empty interrupt
    if (int_source & USB_OTG_GINTSTS_RXFLVL) {
        USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_RXFLVL; // Clear interrupt

        uint32_t RegVal = USB_OTG_FS->GRXSTSP;

        uint32_t pkt_status = (RegVal & USB_OTG_GRXSTSP_PKTSTS) >> 17;

        if (pkt_status == 2U) { // Data packet received
            uint32_t bcnt = (RegVal & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
            if (bcnt != 0U) {
                uint8_t epnum = (RegVal & USB_OTG_GRXSTSP_EPNUM) >> USB_OTG_GRXSTSP_EPNUM_Pos;

                // Read data from endpoint FIFO
                USB_ReadFIFO(epnum, RX_buff[epnum], bcnt);
                EP1_RX_buff_len = bcnt;
            }
        } else if (pkt_status == 6U) { // Setup packet received
            USB_ReadFIFO(0, (uint8_t *)&curr_setup_pkt, 8U);
        }
    }

    // OUT endpoint interrupt
    if (int_source & USB_OTG_GINTSTS_OEPINT) {
        USB_OTG_FS->GINTSTS &= USB_OTG_GINTSTS_OEPINT; // Clear interrupt
        USB_EpOutHandler();
    }

    // IN endpoint interrupt
    if (int_source & USB_OTG_GINTSTS_IEPINT) {
        USB_OTG_FS->GINTSTS &= USB_OTG_GINTMSK_IEPINT; // Clear interrupt
        USB_EpInHandler();
    }
}

/**
 * @brief USB OTG FS hardware reset handler.
 *
 * This handler is called upon a USB hardware reset event.
 * It performs the initial setup of internal USB peripheral registers,
 * configures endpoint interrupts, sets the device address to zero,
 * and prepares EP0/EP1 endpoints for operation.
 *
 * Sequence of actions:
 *  - Clears the global USB status (USB_Status).
 *  - Clears all global interrupt flags (GINTSTS).
 *  - Enables interrupt masks for EP0, EP1 (IN/OUT) and EP2 (IN).
 *  - Configures DOEPMSK and DIEPMSK masks for transfer and SETUP interrupts.
 *  - Sets the device address to 0 (DCFG register).
 *  - Configures Endpoint 1 as Bulk IN and Bulk OUT.
 *  - Sets DOEPTSIZ for OUT Endpoint 1 with one 64-byte packet.
 *
 * @note
 *  - The Max Packet Size is explicitly set to 64 bytes for Bulk transfers,
 *    which corresponds to USB Full Speed Bulk endpoints.
 *  - It should only be called in the context of the USB Reset Interrupt.
 *
 * @see USB_OTG_FS, USB_DEVICE, USB_INEP(), USB_OUTEP()
 */
static void USB_ResetHandler(void) {
    USB_Status = 0;

    // Handle Reset Interrupt
    USB_OTG_FS->GINTSTS &= ~0xFFFFFFFF;

    USB_DEVICE->DAINTMSK = 0x30003;
    // Unmask interrupts IEPM, OEPM for EP0, EP1, and IEPM for EP2
    USB_DEVICE->DOEPMSK  = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM; // SETUP + Transfer Complete OUT
    USB_DEVICE->DIEPMSK  = USB_OTG_DIEPMSK_XFRCM; // Transfer Complete IN

    // Set Default Address to 0
    USB_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

    // Endpoint 1 (Bulk IN)
    USB_INEP(1)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |	// Enable NAK
                USB_OTG_DIEPCTL_TXFNUM_0 |  		// TX FIFO 1
                USB_OTG_DIEPCTL_EPTYP_1 |  			// Bulk
                USB_OTG_DIEPCTL_USBAEP |   			// Endpoint active
                64;                        			// Max Packet Size

    // Endpoint 1 (Bulk OUT)
    USB_OUTEP(1)->DOEPCTL = USB_OTG_DOEPCTL_EPENA |  // Enable Endpoint
                USB_OTG_DOEPCTL_CNAK |               // Clear NAK
                USB_OTG_DOEPCTL_EPTYP_1 |            // Bulk
                USB_OTG_DOEPCTL_USBAEP |             // Endpoint active
                64;                                  // Max Packet Size

    USB_OUTEP(1)->DOEPTSIZ = 0;
    USB_OUTEP(1)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos)); // 1 packet
    // A transfer size of 64 means that the USB_OTG_DOEPINT_XFRC interrupt will be generated for any packet size
    USB_OUTEP(1)->DOEPTSIZ |= 64;

    USB_Status |= USB_RESET_FLAG;
}

/**
 * @brief USB Enumeration Done interrupt handler.
 *
 * This function is called when the USB enumeration process is complete.
 * It performs final configuration steps required after enumeration,
 * such as setting the maximum packet size for Endpoint 0 IN and
 * clearing the global NAK to allow data transfers.
 *
 * Specifically, it:
 *  - Sets the maximum packet size (MPS) of the IN Endpoint 0 to 64 bytes.
 *  - Clears the global NAK condition to enable communication.
 *  - Configures the USB turnaround time (TRDT) to a recommended value.
 *  - Updates the USB status flag to indicate enumeration completion.
 *
 * @note
 *  - Should be called only in response to the Enumeration Done interrupt.
 *
 * @see USB_INEP(), USB_DEVICE, USB_OTG_FS, USB_Status
 */
static void USB_EnudmeHandler() {
	// Handle Enumeration done Interrupt

	// Set the MPS of the IN EP0 to 64 bytes
    USB_INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;

    USB_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK; //clear global NAK

    USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
    USB_OTG_FS->GUSBCFG |= (uint32_t)((0x6U << 10) & USB_OTG_GUSBCFG_TRDT);

    USB_Status |= USB_ENUDME_FLAG;
}

/**
 * @brief USB OUT Endpoint interrupt handler.
 *
 * This function handles interrupts for all USB OUT endpoints.
 * It checks which OUT endpoints have pending interrupts,
 * processes setup packet interrupts and transfer complete interrupts.
 *
 * For each OUT endpoint with an interrupt:
 *  - If a SETUP packet interrupt (STUP) is detected, the interrupt flag is cleared,
 *    and the setup stage handler is called to process the setup packet.
 *  - If a transfer complete interrupt (XFRC) is detected, the interrupt flag is cleared.
 *    For endpoint 1, the registered callback is called with the received data.
 *
 * The function loops over all OUT endpoints with active interrupts,
 * clears their flags, and handles events accordingly.
 *
 * @note
 *  - The function reads the endpoint interrupt status from DAINT and DAINTMSK registers.
 *  - Only IN endpoint interrupts (upper 16 bits) are ignored here.
 *  - Accesses hardware registers directly; should be called in USB interrupt context.
 *
 * @see USB_DEVICE, USB_OUTEP(), USB_SetupStageHandler(), USB_EP1RXCallBack()
 */
static void USB_EpOutHandler() {
	//USB_OTG_GINTSTS_OEPINT
	uint32_t ep_intr_state = 0;
	uint32_t epnum = 0;


	ep_intr_state = USB_DEVICE->DAINT;          				//read endpoint interrupt register
	ep_intr_state &= USB_DEVICE->DAINTMSK;      				//read interrupt only of active endpoints
	ep_intr_state = ((ep_intr_state & 0xFFFF0000U) >> 16);      //read only in EP interrupts

	while (ep_intr_state != 0) {
        if (!(ep_intr_state & 0x1U)) {
            epnum++;
            ep_intr_state >>= 1U; //go to the next EP
        	continue; //if current EP intr state is 0, go to the next EP
        }

        if (USB_OUTEP(epnum)->DOEPINT & USB_OTG_DOEPINT_STUP) {
        	USB_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_STUP;

        	USB_SetupStageHandler(&curr_setup_pkt);
        }

        if (USB_OUTEP(epnum)->DOEPINT & USB_OTG_DOEPINT_XFRC) {
        	USB_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_XFRC;

        	if (epnum == 1) {
        		USB_EP1RXCallBack(RX_buff[1], EP1_RX_buff_len);
        	}

        }

        epnum++;
        ep_intr_state >>= 1U; //go to the next EP
	}
}

/**
 * @brief USB IN Endpoint interrupt handler.
 *
 * This function handles interrupts for all USB IN endpoints.
 * It processes endpoint-specific interrupts such as:
 *  - Transfer Complete (XFRC)
 *  - FIFO Empty (TXFE)
 *
 * The handler loops over all IN endpoints with pending interrupts,
 * clears the interrupt flags, and performs necessary actions:
 *  - On transfer complete interrupt, calls USB_HandleTXTransfer to finalize transfer.
 *  - On FIFO empty interrupt, writes next chunk of data to the FIFO if available,
 *    and updates the bytes sent count.
 *
 * @note
 *  - Reads interrupt status from DAINT and DAINTMSK registers.
 *  - Processes only IN endpoints (lower 16 bits of DAINT).
 *  - Direct hardware register access; intended to be called within USB interrupt context.
 *  - FIFO empty interrupt mask (DIEPEMPMSK) is cleared after writing data.
 *
 * @see USB_DEVICE, USB_INEP(), USB_HandleTXTransfer(), USB_WriteFIFO()
 */
static void USB_EpInHandler() {
	//USB_OTG_GINTSTS_IEPINT
	uint32_t ep_intr_state = 0;
	uint32_t epnum = 0;
	uint32_t len;


	ep_intr_state = USB_DEVICE->DAINT;          //read endpoint interrupt register
	ep_intr_state &= USB_DEVICE->DAINTMSK;      //read interrupt only of active endpoints
	ep_intr_state &= 0x0000FFFFU;                //read only in EP interrupts

	while (ep_intr_state != 0) {
        if (!(ep_intr_state & 0x1U)) {
            epnum++;
            ep_intr_state >>= 1U; //go to the next EP
        	continue; //if current EP intr state is 0, go to the next EP
        }

        //check interrupt of current EP
        if (USB_INEP(epnum)->DIEPINT & USB_OTG_DIEPINT_XFRC) { //transfer complete int
        	USB_INEP(epnum)->DIEPINT = USB_OTG_DIEPINT_XFRC; //clear int

        	USB_HandleTXTransfer(epnum);
        }

        if (USB_INEP(epnum)->DIEPINT & USB_OTG_DIEPINT_TXFE) { //fifo empty int
        	USB_INEP(epnum)->DIEPINT = USB_OTG_DIEPINT_TXFE; //clear int

        	len = EndPoint[epnum].txDataLen - EndPoint[epnum].txBytesSent;

        	if (len > 0) {
				if (len >= USB_CDC_MAX_PACKET_SIZE)
					len = USB_CDC_MAX_PACKET_SIZE;
				USB_WriteFIFO(epnum, EndPoint[epnum].txData + EndPoint[epnum].txBytesSent, (uint16_t)len);
				EndPoint[epnum].txBytesSent += len;
        	}

			USB_DEVICE->DIEPEMPMSK &= ~((uint32_t)(0x1UL << (epnum & 0xFU)));
        }

        epnum++;
        ep_intr_state >>= 1U; //go to the next EP
	}
}

/**
 * @brief Handles the USB Setup Stage request.
 *
 * This function processes USB control setup packets received on endpoint 0.
 * It interprets the request type and request code, handles standard, class,
 * and vendor-specific requests accordingly, and prepares the data stage
 * response if necessary.
 *
 * The function also triggers callbacks for control line state changes and
 * line coding updates relevant to CDC (Communication Device Class) devices.
 *
 * @param[in] request Pointer to the USB setup packet structure containing
 *                    the request details.
 *
 * @details
 * - Reads the setup packet's wLength to determine expected data stage length.
 * - Clears the setup stage buffer to zero.
 * - Checks the request type (bmRequest & 0x60):
 *   - If non-zero: class or vendor-specific requests.
 *       - 0x22 (SET_CONTROL_LINE_STATE): sets DTR flag and calls the line state callback.
 *       - 0x21 (GET_LINE_CODING): copies current line coding to the buffer for transmission.
 *       - 0x20 (SET_LINE_CODING): updates line coding from received data and calls callback.
 *   - If zero: standard requests.
 *       - Dispatches based on recipient type (device, interface, endpoint).
 * - Starts the USB IN transfer on endpoint 0 with prepared data and length.
 *
 * @note
 * - For class/vendor requests, only CDC relevant requests are handled explicitly.
 * - The setup stage length may be adjusted based on CDC_LINE_CODING_LENGTH.
 * - Data is sent back on endpoint 0 using USB_StartTXTransfer.
 *
 * @see USB_StartTXTransfer(), USB_CtrlLineStateCallBack(), USB_SetLinecodingCallBack(), USB_StdDevReq()
 */
static void USB_SetupStageHandler(setup_pkt_t *request) {
	setup_stage_len = curr_setup_pkt.wLength;
	memset(setup_stage_buff, 0, 128);

    if ((request->bmRequest & 0x60) != 0) {
    	//others requests (Class(1) & Vendor(2))

        switch (request->bRequest) {
    		case 0x22: //control line state
    			USB_Status |= USB_DTR_FLAG;
    			USB_CtrlLineStateCallBack(request);
    			setup_stage_len = 0;
    			break;

    		case 0x21: //get line coding
    			if(CDC_LINE_CODING_LENGTH < setup_stage_len) setup_stage_len = CDC_LINE_CODING_LENGTH;
    			memcpy(setup_stage_buff, &lineCoding, setup_stage_len);
    			break;

    		case 0x20: //set line coding
    			setup_stage_len = 0;

    			memcpy(&lineCoding, RX_buff[0], CDC_LINE_CODING_LENGTH);
    			USB_SetLinecodingCallBack(&lineCoding);

    		    if (lineCoding.dwDTERate != 0) {
    		        USB_Status |= USB_LINECODED_FLAG;
    		    }
    			break;

    		default:
    			break;
        }

    } else if ((request->bmRequest & 0x60) == 0) {
    	//standard requests
		switch (request->bmRequest & 0x1FU) {
			case 0x00U: //recipient - device
				USB_StdDevReq(request);
				break;

			case 0x01U: //recipient - interface
				break;

			case 0x02U: //recipient - endpoint
				break;

			default:
				break;
		}
    }

    USB_StartTXTransfer(0, setup_stage_buff, setup_stage_len);
}


/**
 * @brief Handles standard USB device requests.
 *
 * This function processes USB standard device requests received during
 * the control setup stage on endpoint 0. It performs actions such as
 * getting status, setting device address, and returning descriptors.
 *
 * @param[in] request Pointer to the USB setup packet structure containing
 *                    the standard request details.
 *
 * @details
 * - Handles these standard requests:
 *   - BREQ_GET_STATUS: prepares a 2-byte zero status response.
 *   - BREQ_CLEAR_FEATURE: currently no specific action.
 *   - BREQ_SET_FEATURE: currently no specific action.
 *   - BREQ_SET_ADDRESS: sets the device address by writing to DCFG register.
 *   - BREQ_GET_DESCRIPTOR: returns requested descriptor data:
 *       - Device descriptor (0x01)
 *       - Configuration descriptor (0x02)
 *       - Other descriptor types currently not handled explicitly.
 *   - BREQ_SET_DESCRIPTOR: no action.
 *   - BREQ_GET_CONFIGURATION: no action.
 *   - BREQ_SET_CONFIGURATION: no action.
 *
 * - For descriptor requests, the response length is limited to the requested length.
 * - Copies the descriptor data into the setup stage buffer for transmission.
 *
 * @note
 * - This function only implements a subset of standard requests necessary
 *   for USB enumeration and basic operation.
 * - Some requests are placeholders for future implementation.
 *
 * @see deviceDescriptor, configurationDescriptor, USB_StartTXTransfer()
 */
static void USB_StdDevReq(setup_pkt_t *request) {
	switch (request->bRequest) {
		case BREQ_GET_STATUS:
			//send two zeroes (len = 2 bytes)
			setup_stage_len = 2;
			break;

		case BREQ_CLEAR_FEATURE:
			break;

		case BREQ_SET_FEATURE:
			break;

		case BREQ_SET_ADDRESS:
			setup_stage_len = 0;
			USB_DEVICE->DCFG |= (uint32_t)(curr_setup_pkt.wValue << 4);
			break;

		case BREQ_GET_DESCRIPTOR:
			switch (request->wValue >> 8) {
				case 0x01: //Device descriptor
					if(DEVICE_DESCRIPTOR_LENGTH < setup_stage_len) setup_stage_len = DEVICE_DESCRIPTOR_LENGTH;
					memcpy(setup_stage_buff, deviceDescriptor, setup_stage_len);
					break;

				case 0x02: //Configuration descriptor
					if(CONFIGURATION_DESCRIPTOR_LENGTH < setup_stage_len) setup_stage_len = CONFIGURATION_DESCRIPTOR_LENGTH;
					memcpy(setup_stage_buff, configurationDescriptor, setup_stage_len);
					break;

				case 0x03: //TODO String Descriptor
					break;

				case 0x06: //TODO Device Qualifier Descriptor
					break;

				default:
					break;
			}
			break;

		case BREQ_SET_DESCRIPTOR:
			break;

		case BREQ_GET_CONFIGURATION:
			break;

		case BREQ_SET_CONFIGURATION:
			break;

		default:
			break;
	}
}

/** @} */ // end of Interrupt_handler




/** @internal
 * @defgroup auxiliary_functions auxiliary functions
 *
 * @{
 */

/**
 * @brief Handles the ongoing transmission process for a given IN endpoint.
 *
 * This function is responsible for loading the IN endpoint FIFO with data,
 * managing the packet sizes, and controlling the endpoint state during transmission.
 *
 * The general flow is:
 * - If there is remaining data, load up to max packet size (64 bytes) into the TX FIFO,
 *   set the transfer size and packet count registers, enable the endpoint and clear NAK.
 * - Enable TX FIFO empty interrupt to trigger further data loading when FIFO empties.
 * - If all data has been sent, and the total length was a multiple of max packet size,
 *   send a Zero-Length Packet (ZLP) to properly terminate the transfer.
 * - Call transmission complete callbacks if needed (e.g. for endpoint 1).
 * - Reset endpoint TX state and prepare the OUT endpoint for reception if EP0.
 *
 * @param[in] epnum Endpoint number to process transmission for.
 */
static void USB_HandleTXTransfer(uint8_t epnum) {
	uint32_t len;

	if ((EndPoint[epnum].txDataLen - EndPoint[epnum].txBytesSent) != 0) { //TX transfer in progress
		len = (EndPoint[epnum].txDataLen - EndPoint[epnum].txBytesSent);

		if (len >= USB_CDC_MAX_PACKET_SIZE)
			len = USB_CDC_MAX_PACKET_SIZE;

		USB_INEP(epnum)->DIEPTSIZ =
		    ((1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) & USB_OTG_DIEPTSIZ_PKTCNT_Msk) |	// program TX transfer pkt count to one
		    ((len << USB_OTG_DIEPTSIZ_XFRSIZ_Pos) & USB_OTG_DIEPTSIZ_XFRSIZ_Msk);	// program TX transfer size to pkt size
		USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

		// Enable the Tx FIFO Empty Interrupt for this EP
		USB_DEVICE->DIEPEMPMSK |= 1UL << (epnum & 0xFU);

		return;
	}

	// All data packets have been sent.

	if (!(EndPoint[epnum].txDataLen % USB_CDC_MAX_PACKET_SIZE) && EndPoint[epnum].txStatus == TX_PERFORM) {
		USB_SendZLP(epnum); //end the transmission, a multiple of 64, with a zero-length packet
		return;
	}

	if (epnum == 1 && EndPoint[epnum].txDataLen) {
		USB_EP1TXTransferCompliteCallBack();
	}

	EndPoint[epnum].txStatus = TX_READY;
	EndPoint[epnum].txDataLen = 0;
	EndPoint[epnum].txBytesSent = 0;
	EndPoint[epnum].txData = NULL;

	if (epnum == 0) {
		USB_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}
}

uint8_t USB_StartTXTransfer(uint8_t epnum, uint8_t* data, uint16_t len) {
	if ((USB_Status != USB_READY_STATUS) && epnum != 0) {
		return USB_NOTREADY;
	}

	// Previous transfer is not finished
	if(EndPoint[epnum].txStatus != TX_READY) {
		return USB_BUSY;
	}

	if(len != 0) { // Set data to send
		EndPoint[epnum].txStatus = TX_PERFORM;
		EndPoint[epnum].txData = data;
		EndPoint[epnum].txDataLen = len;
		EndPoint[epnum].txBytesSent = 0;

		// Send data
		USB_HandleTXTransfer(epnum);
	} else { // Zero-Length Packet
		USB_SendZLP(epnum);
		if (epnum == 0) {
			USB_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
		}
	}

	return USB_OK;
}

/**
 * @brief Reads data from USB FIFO of specified endpoint into buffer.
 *
 * Note: Endpoint memory is 4-byte aligned, so data must be read/written in 32-bit words.
 *
 * @param[in]  epnum    Endpoint number (not used in FIFO macro indexing)
 * @param[out] dst      Pointer to buffer where received data will be stored
 * @param[in]  pkt_size Number of bytes to read from FIFO
 */
static void USB_ReadFIFO(uint8_t epnum, uint8_t *dst, uint16_t pkt_size) {
	uint16_t i;
	uint8_t residue_buff[4];
	uint16_t words_count = pkt_size / 4;

	for (i = 0U; i < words_count; i++) {
		*((uint32_t *)dst) = USB_DFIFO(0);
		dst += 4;
	}

	if ((pkt_size % 4) != 0)  {
		*((uint32_t *)residue_buff) = USB_DFIFO(0);

		for (i = 0U; i < pkt_size % 4; i++) {
			*(dst + i) = *(residue_buff + i);
		}
	}
}

/**
 * @brief Writes data from buffer to USB FIFO of specified endpoint.
 *
 * Note: Endpoint memory is 4-byte aligned, so data must be read/written in 32-bit words.
 * If data size is not a multiple of 4, an additional 32-bit word is written including padding bytes.
 *
 * @param[in]  epnum    Endpoint number of FIFO to write to
 * @param[in]  src      Pointer to buffer containing data to transmit
 * @param[in]  pkt_size Number of bytes to write to FIFO
 */
static void USB_WriteFIFO(uint8_t epnum, uint8_t *src, uint16_t pkt_size) {
	uint16_t i;
	uint16_t residue = (pkt_size % 4 == 0) ? 0 : 1;
	uint16_t words_count = (pkt_size / 4) + residue;

	for (i = 0; i < words_count; i++){
		USB_DFIFO(epnum) = *((uint32_t *)(void *)src);
		src+=4;
	}
}

/**
 * @brief Send a Zero-Length Packet (ZLP) on the specified IN endpoint.
 *
 * This function is used to send a ZLP, which is required by USB specification
 * to indicate the end of a transfer when the total data length is a multiple of
 * the maximum packet size.
 *
 * @param[in] epnum Endpoint number to send the ZLP on.
 */
static void USB_SendZLP(uint8_t epnum) {
    EndPoint[epnum].txStatus = TX_ZLP;
    USB_INEP(epnum)->DIEPTSIZ = ((USB_OTG_DIEPTSIZ_PKTCNT_Msk & ((1) << USB_OTG_DIEPTSIZ_PKTCNT_Pos))); /* One Packet */
    USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
}

/**
 * @brief Perform a core soft reset of the USB peripheral.
 *
 * This function waits for the AHB bus to become idle and then triggers a core
 * soft reset by setting the CSRST bit in the GRSTCTL register.
 * It waits until the reset bit clears before returning.
 */
static void USB_CoreRST() {
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);
    // Core Soft Reset
    USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);
}

void USB_PrepareReceive(uint8_t epnum) {
    USB_OUTEP(epnum)->DOEPTSIZ = 0;
    USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos));
    USB_OUTEP(epnum)->DOEPTSIZ |= 64;  // Set transfer size to 64 bytes
    USB_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
}

void USB_OUTEPSNAK(uint8_t epnum) {
    USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
}

uint32_t USB_GetStatus() {
    return USB_Status;
}
/** @} */ // end of auxiliary functions




/** @internal
 * @defgroup external_handlers external handlers
 *
 * @{
 */

__WEAK void USB_EP1RXCallBack(uint8_t *RX_buff, uint16_t length) {
    // User-defined data processing and buffering goes here
}

__WEAK void USB_EP1TXTransferCompliteCallBack() {
    // User-defined post-transmission processing goes here
}

__WEAK void USB_CtrlLineStateCallBack(setup_pkt_t *line_state) {
    // User-defined handling of control line state changes
}

__WEAK void USB_SetLinecodingCallBack(line_coding_t *LineCoding) {
    // User-defined processing of new line coding parameters
}

/** @} */ // end of external_handlers
