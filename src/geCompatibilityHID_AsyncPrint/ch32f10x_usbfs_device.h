/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32f10x_usbfs_device.h
 * Author             : WCH
 * Version            : V1.0.0
* Date               : 2022/08/20
 * Description        : header file for ch32v10x_usbfs_device.c
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef USER_CH32V10X_USBFS_DEVICE_H_
#define USER_CH32V10X_USBFS_DEVICE_H_
#ifdef __cplusplus
 extern "C" {
#endif 

#include "debug.h"
#include "string.h"
#include "ch32f10x_usb.h"
#include "usb_desc.h"


/******************************************************************************/
/* Global Define */


/* end-point number */
#define DEF_UEP_IN                    0x80
#define DEF_UEP_OUT                   0x00
#define DEF_UEP0                      0x00
#define DEF_UEP1                      0x01
#define DEF_UEP2                      0x02
#define DEF_UEP3                      0x03
#define DEF_UEP4                      0x04
#define DEF_UEP5                      0x05
#define DEF_UEP6                      0x06
#define DEF_UEP7                      0x07
#define DEF_UEP_NUM                   8

#define USBHD_UEP_MOD_BASE            0x4002340C
#define USBHD_UEP_DMA_BASE            0x40023410
//#define USBHD_UEP_LEN_BASE            0x40023430
//#define USBHD_UEP_CTL_BASE            0x40023432
#define USBHD_UEP_LEN_BASE            0x40023420
#define USBHD_UEP_CTL_BASE            0x40023422
#define USBHD_UEP_RX_EN               0x08
#define USBHD_UEP_TX_EN               0x04
#define USBHD_UEP_BUF_MOD             0x01
#define DEF_UEP_DMA_LOAD              0 /* Direct the DMA address to the data to be processed */
#define DEF_UEP_CPY_LOAD              1 /* Use memcpy to move data to a buffer */
#define USBHD_UEP_MOD(n)              (*((volatile uint8_t *)(USBHD_UEP_MOD_BASE+n)))
#define USBHD_UEP_CTRL(n)             (*((volatile uint8_t *)(USBHD_UEP_CTL_BASE+n*0x04)))
#define USBHD_UEP_DMA(n)              (*((volatile uint32_t *)(USBHD_UEP_DMA_BASE+n*0x04)))
#define USBHD_UEP_BUF(n)              ((uint8_t *)(*((volatile uint32_t *)(USBHD_UEP_DMA_BASE+n*0x04)))+0x20000000)
#define USBHD_UEP_TLEN(n)             (*((volatile uint16_t *)(USBHD_UEP_LEN_BASE+n*0x04)))

/* Ringbuffer define  */
#define DEF_Ring_Buffer_Max_Blks      16
#define DEF_RING_BUFFER_SIZE          (DEF_Ring_Buffer_Max_Blks*DEF_USBD_FS_PACK_SIZE)
#define DEF_RING_BUFFER_REMINE        4
#define DEF_RING_BUFFER_RESTART       10

/* Ring Buffer typedef */
__PACKED typedef struct  _RING_BUFF_COMM
{
    volatile uint8_t LoadPtr;
    volatile uint8_t DealPtr;
    volatile uint8_t RemainPack;
    volatile uint8_t PackLen[DEF_Ring_Buffer_Max_Blks];
    volatile uint8_t StopFlag;
} RING_BUFF_COMM, pRING_BUFF_COMM;

/*******************************************************************************/
/* Variable Definition */
/* Global */
extern const    uint8_t  *pUSBHD_Descr;

/* Setup Request */
extern volatile uint8_t  USBHD_SetupReqCode;
extern volatile uint8_t  USBHD_SetupReqType;
extern volatile uint16_t USBHD_SetupReqValue;
extern volatile uint16_t USBHD_SetupReqIndex;
extern volatile uint16_t USBHD_SetupReqLen;

/* USB Device Status */
extern volatile uint8_t  USBHD_DevConfig;
extern volatile uint8_t  USBHD_DevAddr;
extern volatile uint8_t  USBHD_DevSleepStatus;
extern volatile uint8_t  USBHD_DevEnumStatus;

/* Endpoint Buffer */
extern __attribute__ ((aligned(4))) uint8_t USBHD_EP0_Buf[];
extern __attribute__ ((aligned(4))) uint8_t USBHD_EP2_Buf[];


/* USB IN Endpoint Busy Flag */
extern volatile uint8_t  USBHD_Endp_Busy[ ];

/* Ringbuffer variables */
extern RING_BUFF_COMM  RingBuffer_Comm;
extern __attribute__ ((aligned(4))) uint8_t  Data_Buffer[];

/******************************************************************************/
/* external functions */
extern void USBHD_Device_Init( FunctionalState sta );
extern void USBHD_Device_Endp_Init(void);
extern void USBHD_RCC_Init(void);
extern void USBHD_Send_Resume(void);

#ifdef __cplusplus
}
#endif


#endif /* USER_CH32V10X_USB_DEVICE_H_ */
