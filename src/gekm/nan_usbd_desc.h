/********************************** (C) COPYRIGHT *******************************
 * File Name          : usbd_descriptor.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/18
 * Description        : All descriptors for the keyboard and mouse composite device.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/


#ifndef __USBD_DESC_H
#define __USBD_DESC_H

/*******************************************************************************/
/* Header File */
#include "stdint.h"

/*******************************************************************************/
/* Macro Definition */

/* File Version */
#define DEF_FILE_VERSION              0x01

/* USB Device Info */
#define NAN_USB_VID                   0x1588
#define NAN_USB_PID                   0xA205

/* USB Device Descriptor, Device Serial Number(bcdDevice) */
#define DEF_IC_PRG_VER                DEF_FILE_VERSION

/* USB Device Endpoint Size */
#define DEF_USBD_UEP0_SIZE            64     /* usbhd full speed device end-point 0 size */
/* FS */
#define DEF_USBD_FS_PACK_SIZE         64     /* usbhd full speed device max bluk/int pack size */
#define DEF_USBD_FS_ISO_PACK_SIZE     1023   /* usbhd full speed device max iso pack size */
/* LS */
#define DEf_USBD_LS_UEP0_SIZE         8      /* usbhd low speed device end-point 0 size */
#define DEF_USBD_LS_PACK_SIZE         64     /* usbhd low speed device max int pack size */

/* USB Device Endpoint1-6 Size */
/* FS */
#define DEF_USBD_EP1_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USBD_EP2_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USBD_EP3_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USBD_EP4_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USBD_EP5_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USBD_EP6_FS_SIZE          DEF_USBD_FS_PACK_SIZE
/* LS */
/* ... */

/* USB Device Descriptor Length */
/* Note: If a descriptor does not exist, set the length to 0. */
#define DEF_USBD_DEVICE_DESC_LEN      ( (uint16_t)NanUsbDescDevDesc[ 0 ] )
#define DEF_USBD_CONFIG_DESC_LEN      ( (uint16_t)MyCfgDescr[ 2 ] + ( (uint16_t)MyCfgDescr[ 3 ] << 8 ) )
#define DEF_USBD_REPORT_DESC_LEN_KB   0x3E
#define DEF_USBD_REPORT_DESC_LEN_MS   0x34
#define DEF_USBD_LANG_DESC_LEN        ( (uint16_t)NanUsbDescLangDesc[ 0 ] )
#define DEF_USBD_MANU_DESC_LEN        ( (uint16_t)NanUsbDescManuName[ 0 ] )
#define DEF_USBD_PROD_DESC_LEN        ( (uint16_t)NanUsbDescProdName[ 0 ] )
#define DEF_USBD_SN_DESC_LEN          ( (uint16_t)NanUsbDescSerialNo[ 0 ] )
#define DEF_USBD_QUALFY_DESC_LEN      ( (uint16_t)NanUsbDescQuaDesc[ 0 ])
#define DEF_USBD_BOS_DESC_LEN         0
#define DEF_USBD_FS_OTH_DESC_LEN      0
#define DEF_USBD_HS_OTH_DESC_LEN      0

/*******************************************************************************/
/* Descriptor Declaration */
extern const uint8_t NanUsbDescDevDesc[ ];
extern const uint8_t MyCfgDescr[ ];
extern const uint8_t KeyRepDesc[ ];
extern const uint8_t MouseRepDesc[ ];
extern const uint8_t NanUsbDescQuaDesc[ ];
extern const uint8_t NanUsbDescLangDesc[ ];
extern const uint8_t NanUsbDescManuName[ ];
extern const uint8_t NanUsbDescProdName[ ];
extern const uint8_t NanUsbDescSerialNo[ ];

#endif
