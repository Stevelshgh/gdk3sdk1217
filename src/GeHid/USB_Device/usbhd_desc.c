/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/20
 * Description        : usb device descriptor,configuration descriptor,
 *                      string descriptors and other descriptors.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "usbhd_desc.h"

/* Device Descriptor */
const uint8_t  MyDevDescr[] =
{
    0x12,       // bLength
    0x01,       // bDescriptorType (Device)
    0x00, 0x02, // bcdUSB 1.10
    0x00,       // bDeviceClass
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,   // bMaxPacketSize0 64
    (uint8_t)DEF_USB_VID, (uint8_t)(DEF_USB_VID >> 8),  // idVendor 0x1A86
    (uint8_t)DEF_USB_PID, (uint8_t)(DEF_USB_PID >> 8),  // idProduct 0x5537
    DEF_IC_PRG_VER, 0x00, // bcdDevice 0.01
    0x01,       // iManufacturer (String Index)
    0x02,       // iProduct (String Index)
    0x03,       // iSerialNumber (String Index)
    0x01,       // bNumConfigurations 1
};

/* Configuration Descriptor */
#define CONFIGUARATION_LENGTH (9+9+9+7+7)
const uint8_t  MyCfgDescr[] =
{
	0x09,                                        // 0:  bLength
    0x02,                                        // 1:  bDescriptorType
    CONFIGUARATION_LENGTH,                       // 2:  wTotalLength(LSB)(All Descriptor ,
    0x00,                                        // 3:  wTotalLength(MSB)(Exclude "String")
    0x01,                                        // 4:  bNumInterface,(how many function, at least one function)
    0x01,                                        // 5:  bConfigurationValue
    0x00,                                        // 6:  iConfiguration (String Index)
    0x80,                                        // 7:  bmAttributes (Bus power & Remote wakeup)
                                                 //     D7:      Reserved (Set to one)
                                                 //     D6:      Self Powered
                                                 //     D5:      Remote Wakeup
                                                 //     D4..0:   Reserved (Reset to 0)
    0x23,                                      // 8:  Power    (100mA)

    /* Interface Descriptor */
    0x09,                                        // 0:  bLength
    0x04,                                        // 1:  bDescriptorType
    0x00,                                        // 2:  bInterfaceNumber
    0x00,                                        // 3:  bAlternateSetting.          replace configuration
    0x02,                                        // 4:  bNumEndpoints.              how many end point exclude EP0
    0x03,                                        // 5:  bInterfaceClass.          3:belong to HID class
    0x00,                                        // 6:  bInterfaceSubClass.       0:no sun class,1:boot class
    0x00,                                        // 7:  bInterfaceProtocol.       0:no;1:keyboard;2:mouse
    0x00,                                        // 8:  iInterface.(String Index) 0:flag string interface    
    /* HID Descriptor */
    0x09,                                        // 0:  bLength
    0x21,                                        // 1:  bDescriptorType  0x21:HID  0x22:report  0x23:physical
    0x10,                                        // 2:  bcdHIDClassL
    0x01,                                        // 3:  bcdHIDClassH
    0x00,                                        // 4:  Hardware Target Country
    0x01,                                        // 5:  Number of HID class descriptor to follow
    0x22,                                        // 6:  Report descriptor type  0x22:report follow
    sizeof(MyHIDReportDesc),                      // 7:  Total length of Report descriptor L
    0x00,                                        // 8:  Total length of Report descriptor H
    /* Endpoint Descriptor */ 
    0x07,           				             // 0:  bLength
    0x05,       				                 // 1:  bDescriptorType
    0x81,       				                 // 2:  bEndpointerAddress
                                                 //      D7:     Direction(0:OUT, 1:IN)
                                                 //      D6..4:  Reserved (reset to 0)
                                                 //      D3..0:  Endpoint Number(0000-1111)
    0x03,       				                 // 3:  bmAttributes
                                                 //      D7..2:  Reserved (reset to 0)
                                                 //      D1..0:  Transfer Type(00:Control,01:Bulk,02:Iso,03:Int)
    (uint8_t)DEF_USBD_ENDP2_SIZE,       	     // 4:  wPacketSize(LSB)
    (uint8_t)( DEF_USBD_ENDP2_SIZE >> 8 ),       //     wPacketSize(MSB)
    1,       				                 	 // 6:  bInterval (Unit 1ms)
                                                 //      Isochronous Endpoint: 01
                                                 //      Interrupt Endpoint: 01-ff     
    /* Endpoint Descriptor */ 
    0x07,           				             // 0:  bLength
    0x05,       				                 // 1:  bDescriptorType
    0x02,       				                 // 2:  bEndpointerAddress
                                                 //      D7:     Direction(0:OUT, 1:IN)
                                                 //      D6..4:  Reserved (reset to 0)
                                                 //      D3..0:  Endpoint Number(0000-1111)
    0x03,       				                 // 3:  bmAttributes
                                                 //      D7..2:  Reserved (reset to 0)
                                                 //      D1..0:  Transfer Type(00:Control,01:Bulk,02:Iso,03:Int)
    0x40,       	     // 4:  wPacketSize(LSB)
    (uint8_t)( 0x40 >> 8 ),       //     wPacketSize(MSB)
    0x01       				                 	 // 6:  bInterval (Unit 1ms)
                                                 //      Isochronous Endpoint: 01
                                                 //      Interrupt Endpoint: 01-ff  

};

const uint8_t MyHIDReportDesc[] = 
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

/* Language Descriptor */
const uint8_t  MyLangDescr[] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufacturer Descriptor */
const uint8_t  MyManuInfo[] =
{
    0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

/* Product Information */
const uint8_t  MyProdInfo[] =
{
    0x16, 0x03, 'U', 0x00, 'S', 0x00, 'B', 0x00, ' ', 0x00, 'S', 0x00, 'e', 0x00,
                    'r', 0x00, 'i', 0x00, 'a', 0x00, 'l', 0x00
};

/* Serial Number Information */
const uint8_t  MySerNumInfo[] =
{
    0x16, 0x03, '0', 0, '1', 0, '2', 0, '3', 0, '4', 0, '5', 0
              , '6', 0, '7', 0, '8', 0, '9', 0
};

