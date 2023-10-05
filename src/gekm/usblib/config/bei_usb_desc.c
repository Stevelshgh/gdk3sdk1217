/********************************** (C) COPYRIGHT *******************************
* File Name          : usb_desc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/07/15
* Description        : USB Descriptors for NanoCode side port, aka the North (BEI) 
*    port labeled as USB2.
*******************************************************************************/ 
#include "usb_lib.h"
#include "bei_usb_desc.h"

/* USB Device Descriptors */
const uint8_t  USBD_DeviceDescriptor[] = { 
    USBD_SIZE_DEVICE_DESC,           // bLength
    0x01,                            // bDescriptorType (Device)
    0x10, 0x01,                      // bcdUSB 1.10
    0x00,                            // bDeviceClass (Use class information in the Interface Descriptors)
    0x00,                            // bDeviceSubClass 
    0x00,                            // bDeviceProtocol 
    DEF_USBD_UEP0_SIZE,              // bMaxPacketSize0 8
    0x88, 0x15,                      // idVendor 0x1588 // 0x1A86
    0x12, 0x22,                      // idProduct 0x2212 // 0xFE07
    0x00, 0x01,                      // bcdDevice 2.00
    0x01,                            // iManufacturer (String Index)
    0x02,                            // iProduct (String Index)
    0x00,                            // iSerialNumber (String Index)
    0x01,                            // bNumConfigurations 1
};

/* USB Configration Descriptors */
const uint8_t  USBD_ConfigDescriptor[] = { 
    /* Configuration Descriptor */
    0x09,                           // bLength
    0x02,                           // bDescriptorType
    USBD_SIZE_CONFIG_DESC & 0xFF, USBD_SIZE_CONFIG_DESC >> 8, // wTotalLength
    0x01,                           // bNumInterfaces
    0x01,                           // bConfigurationValue
    0x03,                           // iConfiguration (String Index)
    0x80,                           // bmAttributes Remote Wakeup
    0x23,                           // bMaxPower 70mA

    /* Interface Descriptor */
    0x09,                           // bLength
    0x04,                           // bDescriptorType (Interface)
    0x00,                           // bInterfaceNumber 0
    0x00,                           // bAlternateSetting
    0x02,                           // bNumEndpoints 2
    0x03,                           // bInterfaceClass
    0x00,                           // bInterfaceSubClass
    0x00,                           // bInterfaceProtocol
    0x00,                           // iInterface (String Index)

    /* HID Descriptor */
    0x09,                           // bLength
    0x21,                           // bDescriptorType
    0x11, 0x01,                     // bcdHID
    0x00,                           // bCountryCode
    0x01,                           // bNumDescriptors
    0x22,                           // bDescriptorType
    USBD_SIZE_REPORT_DESC & 0xFF, USBD_SIZE_REPORT_DESC >> 8, // wDescriptorLength

    /* Endpoint Descriptor */
    0x07,                           // bLength
    0x05,                           // bDescriptorType
    0x01,                           // bEndpointAddress: OUT Endpoint 1
    0x03,                           // bmAttributes
    0x40, 0x00,                     // wMaxPacketSize
    0x01,                           // bInterval: 1mS
    
    /* Endpoint Descriptor */
    0x07,                           // bLength
    0x05,                           // bDescriptorType
    0x82,                           // bEndpointAddress: IN Endpoint 2
    0x03,                           // bmAttributes
    0x40, 0x00,                     // wMaxPacketSize
    0x01,                           // bInterval: 1mS
};

/* USB String Descriptors */
const uint8_t BeiUSB_StrLangID[USBD_SIZE_STRING_LANGID] = {
	USBD_SIZE_STRING_LANGID,
	USB_STRING_DESCRIPTOR_TYPE,
	0x09,
	0x04 
};

/* USB Device String Vendor */
const uint8_t BeiUSB_StrVendor[] = {
	USBD_SIZE_STRING_VENDOR,    
	USB_STRING_DESCRIPTOR_TYPE,           
	'n',0,'a',0,'n',0,'o',0,'c',0,'o',0,'d',0,'e',0,'.',0,'c',0,'n',0
};

/* USB Device String Product */
const uint8_t BeiUSB_StrProduct[] = {
	USBD_SIZE_STRING_PRODUCT,         
	USB_STRING_DESCRIPTOR_TYPE,        
    'N', 0, 'K', 0, 'M', 0
};

/* USB Device String Serial */
uint8_t BeiUSB_StrSerial[] = {
	USBD_SIZE_STRING_SERIAL,          
	USB_STRING_DESCRIPTOR_TYPE,                   
	'2', 0, '0', 0, '2', 0, '2', 0, '1', 0, '2', 0 , '1', 0, '2', 0, '0', 0, '1', 0
};

/* HID Report Descriptor */
const uint8_t USBD_HidRepDesc[USBD_SIZE_REPORT_DESC] =
{
    0x06, 0x00, 0xFF,               // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,                     // Usage (0x01)
    0xA1, 0x01,                     // Collection (Application)
    0x09, 0x02,                     //   Usage (0x02)
    0x26, 0xFF, 0x00,               //   Logical Maximum (255)
    0x75, 0x08,                     //   Report Size (8)
    0x15, 0x00,                     //   Logical Minimum (0)
    0x95, 0x40,                     //   Report Count (64)
    0x81, 0x06,                     //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x02,                     //   Usage (0x02)
    0x15, 0x00,                     //   Logical Minimum (0)
    0x26, 0xFF, 0x00,               //   Logical Maximum (255)
    0x75, 0x08,                     //   Report Size (8)
    0x95, 0x40,                     //   Report Count (64)
    0x91, 0x06,                     //   Output (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,                           // End Collection
};




