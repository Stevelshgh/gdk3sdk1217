/********************************** (C) COPYRIGHT *******************************
 * File Name          : bei_usb_desc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/12/11
* Description        : USB Descriptors for NanoCode side port, aka the North (BEI) 
*    port labeled as USB2.
*******************************************************************************/ 
#ifndef __USB_DESC_H
#define __USB_DESC_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "ch32f10x.h"

	 
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define DEF_USBD_UEP0_SIZE          64      
#define DEF_USBD_MAX_PACK_SIZE      64

       
#define USBD_SIZE_DEVICE_DESC        18
#define USBD_SIZE_CONFIG_DESC        41
#define USBD_SIZE_REPORT_DESC        34
#define USBD_SIZE_STRING_LANGID      4
#define USBD_SIZE_STRING_VENDOR      24
#define USBD_SIZE_STRING_PRODUCT     8
#define USBD_SIZE_STRING_SERIAL      22


extern const uint8_t USBD_DeviceDescriptor[USBD_SIZE_DEVICE_DESC];
extern const uint8_t USBD_ConfigDescriptor[USBD_SIZE_CONFIG_DESC];
                    
extern const uint8_t BeiUSB_StrLangID [USBD_SIZE_STRING_LANGID];
extern const uint8_t BeiUSB_StrVendor [];
extern const uint8_t BeiUSB_StrProduct[];
extern uint8_t BeiUSB_StrSerial [];
extern const uint8_t USBD_HidRepDesc[USBD_SIZE_REPORT_DESC];

#ifdef __cplusplus
}
#endif

#endif /* __USB_DESC_H */
