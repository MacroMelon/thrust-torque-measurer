/**
 * @file descriptor.h
 * @author B4KUN
 * @date 10-08-2025
 * @copyright Copyright (c) 2025
 *
 * @brief contains USB descriptors
 */
#ifndef DESCRIPTOR_H_
#define DESCRIPTOR_H_

#include <stdint.h>

#define LOBYTE(x) 							(uint8_t)(x & ~0xFF00)
#define HIBYTE(x) 							(uint8_t)((x >> 8) & ~0xFF00)

#define USB_CDC_MAX_PACKET_SIZE				64
#define CDC_CMD_PACKET_SIZE					8  // Control Endpoint Packet size
#define EP0_SIZE							64
#define EP_COUNT							3


#define USBD_VID							1155
#define USBD_PID_FS							22336

#define DEVICE_DESCRIPTOR_LENGTH			18
#define CONFIGURATION_DESCRIPTOR_LENGTH		67

#define CDC_LINE_CODING_LENGTH				7

// Device string descriptor
static const uint8_t deviceDescriptor[DEVICE_DESCRIPTOR_LENGTH] = {
	DEVICE_DESCRIPTOR_LENGTH, //
	0x01, // Descriptor type - device
	0x00, //  0x0110 = usb 1.1 ; 0x0200 = usb 2.0
	0x02,
	0x02, // CDC
	0x02, //  Abstract Control Model subclass
	0x00,  // protocol
	EP0_SIZE, // EP0 size
	LOBYTE(USBD_VID), 
	HIBYTE(USBD_VID),
	LOBYTE(USBD_PID_FS),
	HIBYTE(USBD_PID_FS),
	0x00, // ver. (BCD)
	0x02, // ver. (BCD)
	0x01, // Manufactor string index
	0x02, // Product string index
	0x03, // Serial number string index
	1 // configuration count
};

// Configuration descriptor
static const uint8_t configurationDescriptor[CONFIGURATION_DESCRIPTOR_LENGTH] = {
	//Configuration Descriptor
	0x09,   // bLength: Configuration Descriptor size
	0x02,      // bDescriptorType: Configuration
	CONFIGURATION_DESCRIPTOR_LENGTH,                // wTotalLength:no of returned bytes
	0x00,
	0x02,   // bNumInterfaces: 2 interface
	0x01,   // bConfigurationValue: Configuration value
	0x00,   // iConfiguration: Index of string descriptor describing the configuration
	0x80,   // bmAttributes: self powered	//RIN - changed to bus powered from 0xc0
	0x32,   // MaxPower 0 mA

	//---------------------------------------------------------------------------

	//Interface Descriptor
	0x09,   // bLength: Interface Descriptor size
	0x04,  // bDescriptorType: Interface
	// Interface descriptor type
	0x00,   // bInterfaceNumber: Number of Interface
	0x00,   // bAlternateSetting: Alternate setting
	0x01,   // bNumEndpoints: One endpoints used
	0x02,   // bInterfaceClass: Communication Interface Class
	0x02,   // bInterfaceSubClass: Abstract Control Model
	0x01,   // bInterfaceProtocol: Common AT commands
	0x00,   // iInterface:

	//Header Functional Descriptor
	0x05,   // bLength: Endpoint Descriptor size
	0x24,   // bDescriptorType: CS_INTERFACE
	0x00,   // bDescriptorSubtype: Header Func Desc
	0x10,   // bcdCDC: spec release number
	0x01,

	//Call Management Functional Descriptor
	0x05,   // bFunctionLength
	0x24,   // bDescriptorType: CS_INTERFACE
	0x01,   // bDescriptorSubtype: Call Management Func Desc
	0x00,   // bmCapabilities: D0+D1
	0x01,   // bDataInterface: 1

	//ACM Functional Descriptor
	0x04,   // bFunctionLength
	0x24,   // bDescriptorType: CS_INTERFACE
	0x02,   // bDescriptorSubtype: Abstract Control Management desc
	0x02,   // bmCapabilities

	//Union Functional Descriptor
	0x05,   // bFunctionLength
	0x24,   // bDescriptorType: CS_INTERFACE
	0x06,   // bDescriptorSubtype: Union func desc
	0x00,   // bMasterInterface: Communication class interface
	0x01,   // bSlaveInterface0: Data Class Interface

	//Endpoint 2 Descriptor
	0x07,                           // bLength: Endpoint Descriptor size
	0x05,   // bDescriptorType: Endpoint
	0x82,                     // bEndpointAddress
	0x03,                           // bmAttributes: Interrupt
	LOBYTE(CDC_CMD_PACKET_SIZE),     // wMaxPacketSize:
	HIBYTE(CDC_CMD_PACKET_SIZE),
	0x10,                           // bInterval:
	//---------------------------------------------------------------------------

	//Data class interface descriptor
	0x09,   // bLength: Endpoint Descriptor size
	0x04,  // bDescriptorType:
	0x01,   // bInterfaceNumber: Number of Interface
	0x00,   // bAlternateSetting: Alternate setting
	0x02,   // bNumEndpoints: Two endpoints used
	0x0A,   // bInterfaceClass: CDC
	0x00,   // bInterfaceSubClass:
	0x00,   // bInterfaceProtocol:
	0x00,   // iInterface:

	//Endpoint OUT Descriptor
	0x07,   // bLength: Endpoint Descriptor size
	0x05,      // bDescriptorType: Endpoint
	0x01,                        // bEndpointAddress
	0x02,                              // bmAttributes: Bulk
	LOBYTE(USB_CDC_MAX_PACKET_SIZE),  // wMaxPacketSize:
	HIBYTE(USB_CDC_MAX_PACKET_SIZE),
	0x00,                              // bInterval: ignore for Bulk transfer

	//Endpoint IN Descriptor
	0x07,   // bLength: Endpoint Descriptor size
	0x05,      // bDescriptorType: Endpoint
	0x81,                         // bEndpointAddress
	0x02,                              // bmAttributes: Bulk
	LOBYTE(USB_CDC_MAX_PACKET_SIZE),  // wMaxPacketSize:
	HIBYTE(USB_CDC_MAX_PACKET_SIZE),
	0x00                               // bInterval: ignore for Bulk transfer
};

#endif
