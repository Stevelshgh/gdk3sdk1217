/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v10x_usbfs_device.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/20
 * Description        : ch32v10x series usb interrupt Processing.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "ch32f10x_usbfs_device.h"

#define SET_REPORT_DEAL_OVER          0x00
#define SET_REPORT_WAIT_DEAL          0x01

/*******************************************************************************/
/* Variable Definition */
/* Global */
const    uint8_t  *pUSBHD_Descr;

/* Setup Request */
volatile uint8_t  USBHD_SetupReqCode;
volatile uint8_t  USBHD_SetupReqType;
volatile uint16_t USBHD_SetupReqValue;
volatile uint16_t USBHD_SetupReqIndex;
volatile uint16_t USBHD_SetupReqLen;

/* USB Device Status */
volatile uint8_t  USBHD_DevConfig;
volatile uint8_t  USBHD_DevAddr;
volatile uint8_t  USBHD_DevSleepStatus;
volatile uint8_t  USBHD_DevEnumStatus;

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBHD_EP0_Buf[ DEF_USBD_UEP0_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHD_EP1_Buf[ DEF_USBD_ENDP1_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHD_EP2_Buf[ DEF_USBD_ENDP2_SIZE*2 ];

__attribute__ ((aligned(4))) uint8_t  HID_Report_Buffer[DEF_USBD_FS_PACK_SIZE];  
volatile uint8_t HID_Set_Report_Flag = SET_REPORT_DEAL_OVER;               // HID SetReport flag
/* HID Class Command */
volatile uint8_t USBHD_HidIdle;
volatile uint8_t USBHD_HidProtocol;

/* USB IN Endpoint Busy Flag */
volatile uint8_t  USBHD_Endp_Busy[ DEF_UEP_NUM ];

/*********************************************************************
 * @fn      USBHD_RCC_Init
 *
 * @brief   Set USB clock.
 *
 * @return  none
 */
void USBHD_RCC_Init( void )
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    EXTEN->EXTEN_CTR |= EXTEN_USBHD_IO_EN;
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHD,ENABLE);
}

/*********************************************************************
 * @fn      USBHD_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBHD_Device_Endp_Init(void)
{
    uint8_t i;

    R8_UEP4_1_MOD = RB_UEP1_TX_EN;
    R8_UEP2_3_MOD = /*RB_UEP2_TX_EN |*/ RB_UEP2_RX_EN;

    pEP0_RAM_Addr = USBHD_EP0_Buf;
    R16_UEP0_DMA = (uint16_t)(uint32_t)USBHD_EP0_Buf;
    R16_UEP1_DMA = (uint16_t)(uint32_t)USBHD_EP1_Buf;
    R16_UEP2_DMA = (uint16_t)(uint32_t)USBHD_EP2_Buf;

    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

    R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

    /* Clear End-points Busy Status */
    for( i=0; i<DEF_UEP_NUM; i++ )
    {
        USBHD_Endp_Busy[ i ] = 0;
    }
}

/*********************************************************************
 * @fn      USBHD_Device_Init
 *
 * @brief   Initializes USB device.
 *
 * @return  none
 */
void USBHD_Device_Init( FunctionalState sta )
{
    if( sta )
    {
		R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
        Delay_Us( 10 );
        R8_USB_CTRL = 0x00;
        R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
        R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
		USBHD_Device_Endp_Init( );
        R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;
        NVIC_EnableIRQ(USBHD_IRQn);
    }
    else
    {
        R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
        Delay_Us( 10 );
        R8_USB_CTRL = 0x00;
        NVIC_DisableIRQ(USBHD_IRQn);
    }
}

/*********************************************************************
 * @fn      USBHD_Endp_DataUp
 *
 * @brief   usbhd-fs device data upload
 *          input: endp  - end-point numbers
 *                 *pubf - data buffer
 *                 len   - load data length
 *                 mod   - 0: DEF_UEP_DMA_LOAD 1: DEF_UEP_CPY_LOAD
 *
 * @return  none
 */
uint8_t USBHD_Endp_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod)
{
    uint8_t data_endp = endp;
    uint16_t data_len = len;
    
    if(data_endp == DEF_UEP1)
    {
        /* Check if endp1 upload end  */
        if(USBHD_Endp_Busy[DEF_UEP1])
        {
            return 1;
        }
        if( (R8_UEP4_1_MOD & RB_UEP1_TX_EN) && (R8_UEP4_1_MOD & RB_UEP1_RX_EN) )
        {
            /* enpd1 tx/rx enable, 64-bytes out, 64-bytes in */
            memcpy( ((uint8_t *)(R16_UEP1_DMA + 0x20000000)) + 64, pbuf, data_len );
        }
        else if( R8_UEP4_1_MOD & RB_UEP1_TX_EN )
        {
            /* enpd1 tx enable, 64-bytes in */
            if( mod == DEF_UEP_DMA_LOAD )
            {
                R16_UEP1_DMA = (uint16_t)(uint32_t)pbuf;
            }
            else 
            {
                memcpy( ((uint8_t *)(R16_UEP1_DMA + 0x20000000)), pbuf, data_len );
            }
        }
        else 
        {
            return 1;
        }
        USBHD_Endp_Busy[DEF_UEP1] = 0x01;
        R8_UEP1_T_LEN = data_len;
        R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
    }
    else if(data_endp == DEF_UEP2)
    {
        /* Check if endp2 upload end  */
        if(USBHD_Endp_Busy[DEF_UEP2])
        {
            return 1;
        }
        if( (R8_UEP2_3_MOD & RB_UEP2_TX_EN) && (R8_UEP2_3_MOD & RB_UEP2_RX_EN) )
        {
            /* enpd2 tx/rx enable, 64-bytes out, 64-bytes in */
            memcpy( ((uint8_t *)(R16_UEP2_DMA + 0x20000000)) + 64, pbuf, data_len );
        }
        else if( R8_UEP2_3_MOD & RB_UEP2_TX_EN )
        {
            /* enpd2 tx enable, 64-bytes in */
            if( mod == DEF_UEP_DMA_LOAD )
            {
                R16_UEP2_DMA = (uint16_t)(uint32_t)pbuf;
            }
            else 
            {
                memcpy( ((uint8_t *)(R16_UEP2_DMA + 0x20000000)), pbuf, data_len );
            }
        }
        else 
        {
            return 1;
        }
        USBHD_Endp_Busy[DEF_UEP2] = 0x01;
        R8_UEP2_T_LEN = data_len;
        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
    }
    else if(data_endp == DEF_UEP3)
    {
        /* Check if endp3 upload end  */
        if(USBHD_Endp_Busy[DEF_UEP3])
        {
            return 1;
        }
        if( (R8_UEP2_3_MOD & RB_UEP3_TX_EN) && (R8_UEP2_3_MOD & RB_UEP3_RX_EN) )
        {
            /* enpd3 tx/rx enable, 64-bytes out, 64-bytes in */
            memcpy( ((uint8_t *)(R16_UEP3_DMA + 0x20000000)) + 64, pbuf, data_len );
        }
        else if( R8_UEP2_3_MOD & RB_UEP3_TX_EN )
        {
            /* enpd3 tx enable, 64-bytes in */
            if( mod == DEF_UEP_DMA_LOAD )
            {
                R16_UEP3_DMA = (uint16_t)(uint32_t)pbuf;
            }
            else 
            {
                memcpy( ((uint8_t *)(R16_UEP3_DMA + 0x20000000)), pbuf, data_len );
            }
        }
        else 
        {
            return 1;
        }
        USBHD_Endp_Busy[DEF_UEP3] = 0x01;
        R8_UEP3_T_LEN = data_len;
        R8_UEP3_CTRL = (R8_UEP3_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
    }
    else if(data_endp == DEF_UEP4)
    {
        if(USBHD_Endp_Busy[DEF_UEP4])
        {
            return 1;
        }
        if( (R8_UEP4_1_MOD & RB_UEP4_TX_EN) && (R8_UEP4_1_MOD & RB_UEP4_RX_EN) )
        {
            /* enpd4 tx/rx enable, share dma with endp0 */
            memcpy( ((uint8_t *)(R16_UEP0_DMA + 0x20000000)) + 128, pbuf, data_len );
        }
        else if( R8_UEP4_1_MOD & RB_UEP4_TX_EN )
        {
            /* enpd4 only tx enable, share dma with endp0 */
            memcpy( ((uint8_t *)(R16_UEP0_DMA + 0x20000000)) + 64, pbuf, data_len );
        }
        else 
        {
             return 1;
        }
        USBHD_Endp_Busy[DEF_UEP4] = 0x01;
        R8_UEP4_T_LEN = data_len;
        R8_UEP4_CTRL = (R8_UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
    }
    else
    {
        return 1;
    }
    return 0;
}

void DecodeUSBHD_IRQHandlerData(IRQHandler_Data_t *IRQHandler_Data_X )
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    uint8_t *pUSBHD_Descr;

    intflag = IRQHandler_Data_X->intflag;
    intst   = IRQHandler_Data_X->intst;

    Debug_printf("intflag:%x instst:%x\r\n",intflag,intst);

    if( intflag & RB_UIF_TRANSFER )
    {
        Debug_printf(">RB_UIF_TRANSFER:%x\r\n",RB_UIF_TRANSFER);
        switch ( intst & MASK_UIS_TOKEN )
        {
            /* data-in stage processing */
            case UIS_TOKEN_IN:
                Debug_printf(">>UIS_TOKEN_IN:%x\r\n",UIS_TOKEN_IN);
                switch ( intst & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
                {
                    /* end-point 0 data in interrupt */
                    case UIS_TOKEN_IN | DEF_UEP0:
                        Debug_printf(">>>UIS_TOKEN_IN | DEF_UEP0:USBHD_SetupReqLen(%x),USBHD_SetupReqCode(%x)\r\n",USBHD_SetupReqLen,USBHD_SetupReqCode);
                        #if 0
                        if( USBHD_SetupReqLen == 0 )
                        {
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                        }
                        if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            /* Non-standard request endpoint 0 Data upload */
                        }
                        else
                        {
                            switch( USBHD_SetupReqCode )
                            {
                                case USB_GET_DESCRIPTOR:
                                        len = USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                                        memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                                        USBHD_SetupReqLen -= len;
                                        pUSBHD_Descr += len;
                                        R8_UEP0_T_LEN   = len;
                                        R8_UEP0_CTRL ^= RB_UEP_T_TOG;
                                        break;

                                case USB_SET_ADDRESS:
                                        R8_USB_DEV_AD = ( R8_USB_DEV_AD & RB_UDA_GP_BIT ) | USBHD_DevAddr;
                                        break;

                                default:
                                        break;
                            }
                        }
                        #endif
                        break;

                        /* end-point 1 data in interrupt */
                        case UIS_TOKEN_IN | DEF_UEP1:
                            #if 0
                            R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
                            R8_UEP1_CTRL ^= RB_UEP_T_TOG;
                            USBHD_Endp_Busy[ DEF_UEP1 ] = 0;
                            #endif
                            break;

                        /* end-point 2 data in interrupt */
                        case UIS_TOKEN_IN | DEF_UEP2:
                            #if 0
                            R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
                            R8_UEP2_CTRL ^= RB_UEP_T_TOG;
                            USBHD_Endp_Busy[ DEF_UEP2 ] = 0;
                            //Uart.USB_Up_IngFlag = 0x00;
                            #endif
                            break;

                    default :
                        break;
                }
                break;

            /* data-out stage processing */
            case UIS_TOKEN_OUT:
                Debug_printf(">>UIS_TOKEN_OUT:%x\r\n",UIS_TOKEN_OUT);
                switch ( intst & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
                {
                    /* end-point 0 data out interrupt */
                    case UIS_TOKEN_OUT | DEF_UEP0:
                        len = R8_USB_RX_LEN;
                        if ( intst & RB_UIS_TOG_OK )
                        {
                            Debug_printf(">>>UIS_TOKEN_OUT | DEF_UEP0:RB_UIS_TOG_OK(%x) len(%x)\r\n",RB_UIS_TOG_OK,len);
                            if ( ( IRQHandler_Data_X->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                            {
                                /* Non-standard request end-point 0 Data download */
                                #if 0
                                USBHD_SetupReqLen = 0;
                                #endif
                                /* Non-standard request end-point 0 Data download */
                                #if 0
                                if (( USBHD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS)
                                {
                                    switch( USBHD_SetupReqCode )
                                    {
                                        case HID_SET_REPORT:
                                            memcpy(&HID_Report_Buffer[0],USBHD_EP0_Buf,DEF_USBD_FS_PACK_SIZE);
                                            HID_Set_Report_Flag = SET_REPORT_WAIT_DEAL;
                                            R8_UEP0_CTRL = ( R8_UEP0_CTRL & ~MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            break;
                                        default:
                                            break;
                                    }
                                }
                                #endif

                            }
                            else
                            {
                                /* Standard request end-point 0 Data download */
                                /* Add your code here */
                            }
                            #if 0
                            if( USBHD_SetupReqLen == 0 )
                            {
                                R8_UEP0_T_LEN = 0;
                                R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK | RB_UEP_T_TOG;
                            }
                            #endif
                        }
                        break;

                    /* end-point 1 data out interrupt */
                    case UIS_TOKEN_OUT | DEF_UEP2:
                        #if 0
                        R8_UEP2_CTRL ^= RB_UEP_R_TOG;
                        #endif
                        break;

                    default:
                        break;
                }
                break;

            /* Setup stage processing */
            case UIS_TOKEN_SETUP:
                Debug_printf(">>UIS_TOKEN_SETUP:%x\r\n",UIS_TOKEN_SETUP);
                //R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
                /* Store All Setup Values */
                uint8_t bSetupReqType  = IRQHandler_Data_X->bRequestType;
                uint8_t bSetupReqCode  = IRQHandler_Data_X->bRequest;
                uint16_t wSetupReqLen   = IRQHandler_Data_X->wLength;
                uint16_t wSetupReqValue = IRQHandler_Data_X->wValue;
                uint16_t wSetupReqIndex = IRQHandler_Data_X->wIndex;

                Debug_printf(">>>UIS_TOKEN_SETUP:ReqType:%x,ReqCode:%x,ReqLen:%x,ReqValue:%x,ReqIndex:%x\r\n",bSetupReqType,bSetupReqCode,wSetupReqLen,wSetupReqValue,wSetupReqIndex);

                len = 0;
                errflag = 0;
                if ( ( bSetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    /* usb non-standard request processing */
                    /* errflag = 0xFF; if this request or cmd dose not support */
                    /* usb non-standard request processing */
                    #if 0
                       if( USBHD_SetupReqType & USB_REQ_TYP_CLASS )
                       {
                           /* Class requests */
                           switch( USBHD_SetupReqCode )
                           {
                               case CDC_GET_LINE_CODING:
                                   pUSBHD_Descr = (uint8_t *)&Uart.Com_Cfg[ 0 ];
                                   len = 7;
                                   break;

                               case CDC_SET_LINE_CODING:
                                   break;

                               case CDC_SET_LINE_CTLSTE:
                                   break;

                               case CDC_SEND_BREAK:
                                   break;

                               default:
                                   errflag = 0xff;
                                   break;
                           }
                       }
                       else if( USBHD_SetupReqType & USB_REQ_TYP_VENDOR )
                       {
                           /* Manufacturer request */
                       }
                       else
                       {
                           errflag = 0xFF;
                       }

                       /* Copy Descriptors to Endp0 DMA buffer */
                       len = (USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                       memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                       pUSBHD_Descr += len;
                    #endif
                    #if 0
                    if (( USBHD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS)
                    {
                        switch( USBHD_SetupReqCode )
                        {
                            case HID_SET_REPORT:
                                break;

                            case HID_GET_REPORT:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    len = DEF_USBD_UEP0_SIZE;
                                    memcpy(USBHD_EP0_Buf,&HID_Report_Buffer[0],DEF_USBD_UEP0_SIZE);
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_SET_IDLE:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_HidIdle = USBHD_EP0_Buf[ 3 ];
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_SET_PROTOCOL:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_HidProtocol = USBHD_EP0_Buf[ 2 ];
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_GET_IDLE:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = USBHD_HidIdle;
                                    len = 1;
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;
                            case HID_GET_PROTOCOL:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = USBHD_HidProtocol;
                                    len = 1;
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;
                            default:
                                errflag = 0xFF;
                                break;
                        }
                    }
                    #endif
                }
                else
                {
                    /* usb standard request processing */
                    switch( bSetupReqCode )
                    {
                        /* get device/configuration/string/report/... descriptors */
                        case USB_GET_DESCRIPTOR:
                            Debug_printf(">>>>USB_GET_DESCRIPTOR:%x\r\n",USB_GET_DESCRIPTOR);
                            switch( (uint8_t)( USBHD_SetupReqValue >> 8 ) )
                            {
                                /* get usb device descriptor */
                                case USB_DESCR_TYP_DEVICE:
                                    pUSBHD_Descr = MyDevDescr;
                                    len = DEF_USBD_DEVICE_DESC_LEN;
                                    break;

                                /* get usb configuration descriptor */
                                case USB_DESCR_TYP_CONFIG:
                                    pUSBHD_Descr = MyCfgDescr;
                                    len = DEF_USBD_CONFIG_DESC_LEN;
                                    break;

                                /* get usb report descriptor */
                                case USB_DESCR_TYP_REPORT:
                                    if( wSetupReqIndex == 0x00 )
                                    {
                                        pUSBHD_Descr = MyHIDReportDesc;
                                        len = sizeof(MyHIDReportDesc);
                                    }
                                    else if( wSetupReqIndex == 0x01 )
                                    {
                                        //pUSBHD_Descr = MouseRepDesc;
                                        //len = DEF_USBD_REPORT_DESC_LEN_MS;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;

                               /* get usb hid descriptor */
                                case USB_DESCR_TYP_HID:
                                    if( wSetupReqIndex == 0x00 )
                                    {
                                        pUSBHD_Descr = &MyCfgDescr[ 18 ];
                                        len = 9;
                                    }
                                    else if( wSetupReqIndex == 0x01 )
                                    {
                                        //pUSBHD_Descr = &MyCfgDescr[ 43 ];
                                        //len = 9;
                                        errflag = 0xFF;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;

                                /* get usb string descriptor */
                                case USB_DESCR_TYP_STRING:
                                    switch( (uint8_t)( USBHD_SetupReqValue & 0xFF ) )
                                    {
                                        /* Descriptor 0, Language descriptor */
                                        case DEF_STRING_DESC_LANG:
                                            pUSBHD_Descr = MyLangDescr;
                                            len = DEF_USBD_LANG_DESC_LEN;
                                            break;

                                        /* Descriptor 1, Manufacturers String descriptor */
                                        case DEF_STRING_DESC_MANU:
                                            pUSBHD_Descr = MyManuInfo;
                                            len = DEF_USBD_MANU_DESC_LEN;
                                            break;

                                        /* Descriptor 2, Product String descriptor */
                                        case DEF_STRING_DESC_PROD:
                                            pUSBHD_Descr = MyProdInfo;
                                            len = DEF_USBD_PROD_DESC_LEN;
                                            break;

                                        /* Descriptor 3, Serial-number String descriptor */
                                        case DEF_STRING_DESC_SERN:
                                            pUSBHD_Descr = MySerNumInfo;
                                            len = DEF_USBD_SN_DESC_LEN;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                    break;

                                default :
                                    errflag = 0xFF;
                                    break;
                            }
                            Debug_printf(">>>>USBHD_SetupReqLen:%x vs len:%x\r\n",wSetupReqLen,len);
                            /* Copy Descriptors to Endp0 DMA buffer */
                            if( wSetupReqLen>len )
                            {
                                wSetupReqLen = len;
                            }
                            len = (wSetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : wSetupReqLen;
                            Debug_printf(">>>>USBHD_SetupReqLen2:%x vs len:%x\r\n",wSetupReqLen,len);

                            #if 0
                            memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                            pUSBHD_Descr += len;
                            #endif
                            break;

                        /* Set usb address */
                        case USB_SET_ADDRESS:
                            Debug_printf(">>>>USB_SET_ADDRESS:%x\r\n",USB_SET_ADDRESS);
                            #if 0
                            USBHD_DevAddr = (uint8_t)( USBHD_SetupReqValue & 0xFF );
                            #endif
                            break;

                        /* Get usb configuration now set */
                        case USB_GET_CONFIGURATION:
                            Debug_printf(">>>>USB_GET_CONFIGURATION:%x\r\n",USB_GET_CONFIGURATION);
                            #if 0
                            USBHD_EP0_Buf[0] = USBHD_DevConfig;
                            if ( USBHD_SetupReqLen > 1 )
                            {
                                USBHD_SetupReqLen = 1;
                            }
                            #endif
                            break;

                        /* Set usb configuration to use */
                        case USB_SET_CONFIGURATION:
                            Debug_printf(">>>>USB_SET_CONFIGURATION:%x\r\n",USB_SET_CONFIGURATION);
                            #if 0
                            USBHD_DevConfig = (uint8_t)( USBHD_SetupReqValue & 0xFF );
                            USBHD_DevEnumStatus = 0x01;
                            #endif
                            break;

                        /* Clear or disable one usb feature */
                        case USB_CLEAR_FEATURE:
                            Debug_printf(">>>>USB_CLEAR_FEATURE:%x\r\n",USB_CLEAR_FEATURE);
                            #if 0
                            if ( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* clear one device feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    /* clear usb sleep status, device not prepare to sleep */
                                    USBHD_DevSleepStatus &= ~0x01;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Clear End-point Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_IN | DEF_UEP1 ):
                                            /* Set End-point 1 IN NAK */
                                            R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_NAK;
                                            break;

                                        case ( DEF_UEP_OUT | DEF_UEP2 ):
                                            /* Set End-point 2 OUT ACK */
                                            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_ACK;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP3 ):
                                            /* Set End-point 3 IN NAK */
                                            R8_UEP3_CTRL = ( R8_UEP3_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_NAK;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            #endif
                            break;

                        /* set or enable one usb feature */
                        case USB_SET_FEATURE:
                            Debug_printf(">>>>USB_SET_FEATURE:%x\r\n",USB_SET_FEATURE);
                            #if 0
                            if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* Set Device Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    if( MyCfgDescr[ 7 ] & 0x20 )
                                    {
                                        /* Set Wake-up flag, device prepare to sleep */
                                        USBHD_DevSleepStatus |= 0x01;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Set End-point Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    /* Set end-points status stall */
                                    switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_IN | DEF_UEP1 ):
                                            /* Set End-point 1 IN STALL */
                                            R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_STALL;
                                            break;

                                        case ( DEF_UEP_OUT | DEF_UEP2 ):
                                            /* Set End-point 2 OUT STALL */
                                            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_STALL;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP3 ):
                                            /* Set End-point 3 IN STALL */
                                            R8_UEP3_CTRL = ( R8_UEP3_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_STALL;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            #endif
                            break;

                        /* This request allows the host to select another setting for the specified interface  */
                        case USB_GET_INTERFACE:
                            Debug_printf(">>>>USB_GET_INTERFACE:%x\r\n",USB_GET_INTERFACE);
                            #if 0
                            USBHD_EP0_Buf[0] = 0x00;
                            if ( USBHD_SetupReqLen > 1 )
                            {
                                USBHD_SetupReqLen = 1;
                            }
                            #endif
                            break;

                        case USB_SET_INTERFACE:
                            Debug_printf(">>>>USB_SET_INTERFACE:%x\r\n",USB_SET_INTERFACE);
                            break;

                        /* host get status of specified device/interface/end-points */
                        case USB_GET_STATUS:
                            Debug_printf(">>>>USB_GET_STATUS:%x\r\n",USB_GET_STATUS);
                            #if 0
                            USBHD_EP0_Buf[ 0 ] = 0x00;
                            USBHD_EP0_Buf[ 1 ] = 0x00;
                            if ( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                if( USBHD_DevSleepStatus & 0x01 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = 0x02;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                {
                                    case ( DEF_UEP_IN | DEF_UEP1 ):
                                        if( ( R8_UEP1_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_OUT | DEF_UEP2 ):
                                        if( ( R8_UEP2_CTRL & MASK_UEP_R_RES ) == UEP_R_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_IN | DEF_UEP3 ):
                                        if( ( R8_UEP3_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    default:
                                        errflag = 0xFF;
                                        break;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }

                            if ( USBHD_SetupReqLen > 2 )
                            {
                                USBHD_SetupReqLen = 2;
                            }
                            #endif

                            break;

                        default:
                            Debug_printf(">>>>default:%x\r\n",0);
                            errflag = 0xFF;
                            break;
                    }
                }

                /* errflag = 0xFF means a request not support or some errors occurred, else correct */
                #if 0
                if( errflag == 0xFF)
                {
                    /* if one request not support, return stall */
                    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
                }
                else
                {
                    /* end-point 0 data Tx/Rx */
                    if( USBHD_SetupReqType & DEF_UEP_IN )
                    {
                        /* tx */
                        len = ( USBHD_SetupReqLen > DEF_USBD_UEP0_SIZE ) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                        USBHD_SetupReqLen -= len;
                        R8_UEP0_T_LEN  = len;
                        R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                    }
                    else
                    {
                        /* rx */
                        if( USBHD_SetupReqLen == 0 )
                        {
                            R8_UEP0_T_LEN = 0;
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                        }
                        else
                        {
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK ;
                        }
                    }
                }
                #endif 
                break;

            /* Sof pack processing */
            case UIS_TOKEN_SOF:
                Debug_printf(">>UIS_TOKEN_SOF:%x\r\n",UIS_TOKEN_SOF);
                break;

            default :
                break;
        }
        #if 0
        R8_USB_INT_FG = RB_UIF_TRANSFER;
        #endif
    }
    else if( intflag & RB_UIF_BUS_RST )
    {
        Debug_printf(">RB_UIF_BUS_RST:%x\r\n",RB_UIF_BUS_RST);
        /* usb reset interrupt processing */
        #if 0
        USBHD_DevConfig = 0;
        USBHD_DevAddr = 0;
        USBHD_DevSleepStatus = 0;
        USBHD_DevEnumStatus = 0;

        R8_USB_DEV_AD = 0;
        USBHD_Device_Endp_Init( );
        R8_USB_INT_FG |= RB_UIF_BUS_RST;
        #endif
    }
    else if( intflag & RB_UIF_SUSPEND )
    {
        Debug_printf(">RB_UIF_SUSPEND:%x\r\n",RB_UIF_SUSPEND);
        /* usb suspend interrupt processing */
        #if 0
        R8_USB_INT_FG = RB_UIF_SUSPEND;
        if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )
        {
            USBHD_DevSleepStatus |= 0x02;
            if( USBHD_DevSleepStatus == 0x03 )
            {
                // Handling usb sleep here 
            }
        }
        else
        {
            USBHD_DevSleepStatus &= ~0x02;
        }
        #endif
        
    }
    else
    {
        /* other interrupts */
        DebguPrintf(">other interrupts:%x\r\n",intflag);
        #if 0
        R8_USB_INT_FG = intflag;
        #endif
    }
    IRQHandler_TxCnt++;
    IRQHandler_TxCnt = IRQHandler_TxCnt % BUFSIZE;
}

/*********************************************************************
 * @fn      USBHD_IRQHandler
 *
 * @brief   This function handles USB FS exception.
 *
 * @return  none
 */
#if 1
void USBHD_IRQHandler( void )
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = R8_USB_INT_FG;
    intst = R8_USB_INT_ST;

    //AsyncPrintf("intflag:%x instst:%x\r\n",intflag,intst);

    IRQHandler_Data_Buffer[IRQHandler_RxCnt].intflag = intflag;
    IRQHandler_Data_Buffer[IRQHandler_RxCnt].intst = intst;

    if( intflag & RB_UIF_TRANSFER )
    {
        //AsyncPrintf(">RB_UIF_TRANSFER:%x\r\n",RB_UIF_TRANSFER);
        switch ( intst & MASK_UIS_TOKEN )
        {
            /* data-in stage processing */
            case UIS_TOKEN_IN:
                //AsyncPrintf(">>UIS_TOKEN_IN:%x\r\n",UIS_TOKEN_IN);
                switch ( intst & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
                {
                    /* end-point 0 data in interrupt */
                    case UIS_TOKEN_IN | DEF_UEP0:
                        //AsyncPrintf(">>>UIS_TOKEN_IN | DEF_UEP0:USBHD_SetupReqLen(%x),USBHD_SetupReqCode(%x)\r\n",USBHD_SetupReqLen,USBHD_SetupReqCode);
                        if( USBHD_SetupReqLen == 0 )
                        {
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                        }
                        if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            /* Non-standard request endpoint 0 Data upload */
                        }
                        else
                        {
                            switch( USBHD_SetupReqCode )
                            {
                                case USB_GET_DESCRIPTOR:
                                        len = USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                                        memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                                        USBHD_SetupReqLen -= len;
                                        pUSBHD_Descr += len;
                                        R8_UEP0_T_LEN   = len;
                                        R8_UEP0_CTRL ^= RB_UEP_T_TOG;
                                        break;

                                case USB_SET_ADDRESS:
                                        R8_USB_DEV_AD = ( R8_USB_DEV_AD & RB_UDA_GP_BIT ) | USBHD_DevAddr;
                                        break;

                                default:
                                        break;
                            }
                        }
                        break;

                        /* end-point 1 data in interrupt */
                        case UIS_TOKEN_IN | DEF_UEP1:
                            R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
                            R8_UEP1_CTRL ^= RB_UEP_T_TOG;
                            USBHD_Endp_Busy[ DEF_UEP1 ] = 0;
                            break;

                        /* end-point 2 data in interrupt */
                        case UIS_TOKEN_IN | DEF_UEP2:
                            R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
                            R8_UEP2_CTRL ^= RB_UEP_T_TOG;
                            USBHD_Endp_Busy[ DEF_UEP2 ] = 0;
                            //Uart.USB_Up_IngFlag = 0x00;
                            break;

                    default :
                        break;
                }
                break;

            /* data-out stage processing */
            case UIS_TOKEN_OUT:
                //AsyncPrintf(">>UIS_TOKEN_OUT:%x\r\n",UIS_TOKEN_OUT);
                switch ( intst & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
                {
                    /* end-point 0 data out interrupt */
                    case UIS_TOKEN_OUT | DEF_UEP0:
                        len = R8_USB_RX_LEN;
                        if ( intst & RB_UIS_TOG_OK )
                        {
                            //AsyncPrintf(">>>UIS_TOKEN_OUT | DEF_UEP0:RB_UIS_TOG_OK(%x) len(%x)\r\n",RB_UIS_TOG_OK,len);
                            if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                            {
                                /* Non-standard request end-point 0 Data download */
                                #if 0
                                USBHD_SetupReqLen = 0;
                                #endif
                                /* Non-standard request end-point 0 Data download */

                                if (( USBHD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS)
                                {
                                    switch( USBHD_SetupReqCode )
                                    {
                                        case HID_SET_REPORT:
                                            memcpy(&HID_Report_Buffer[0],USBHD_EP0_Buf,DEF_USBD_FS_PACK_SIZE);
                                            HID_Set_Report_Flag = SET_REPORT_WAIT_DEAL;
                                            R8_UEP0_CTRL = ( R8_UEP0_CTRL & ~MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            break;
                                        default:
                                            break;
                                    }
                                }

                            }
                            else
                            {
                                /* Standard request end-point 0 Data download */
                                /* Add your code here */
                            }
                            #if 0
                            if( USBHD_SetupReqLen == 0 )
                            {
                                R8_UEP0_T_LEN = 0;
                                R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK | RB_UEP_T_TOG;
                            }
                            #endif
                        }
                        break;

                    /* end-point 1 data out interrupt */
                    case UIS_TOKEN_OUT | DEF_UEP2:
                        R8_UEP2_CTRL ^= RB_UEP_R_TOG;
                        break;

                    default:
                        break;
                }
                break;

            /* Setup stage processing */
            case UIS_TOKEN_SETUP:
                //AsyncPrintf(">>UIS_TOKEN_SETUP:%x\r\n",UIS_TOKEN_SETUP);
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
                /* Store All Setup Values */
                USBHD_SetupReqType  = pUSBHD_SetupReqPak->bRequestType;
                USBHD_SetupReqCode  = pUSBHD_SetupReqPak->bRequest;
                USBHD_SetupReqLen   = pUSBHD_SetupReqPak->wLength;
                USBHD_SetupReqValue = pUSBHD_SetupReqPak->wValue;
                USBHD_SetupReqIndex = pUSBHD_SetupReqPak->wIndex;

                IRQHandler_Data_Buffer[IRQHandler_RxCnt].bRequestType  = USBHD_SetupReqType;
                IRQHandler_Data_Buffer[IRQHandler_RxCnt].bRequest      = USBHD_SetupReqCode;
                IRQHandler_Data_Buffer[IRQHandler_RxCnt].wLength       = USBHD_SetupReqLen;
                IRQHandler_Data_Buffer[IRQHandler_RxCnt].wValue        = USBHD_SetupReqValue;
                IRQHandler_Data_Buffer[IRQHandler_RxCnt].wIndex        = USBHD_SetupReqIndex;

                //AsyncPrintf(">>>UIS_TOKEN_SETUP:ReqType:%x,ReqCode:%x,ReqLen:%x,ReqValue:%x,ReqIndex:%x\r\n",USBHD_SetupReqType,USBHD_SetupReqCode,USBHD_SetupReqLen,USBHD_SetupReqValue,USBHD_SetupReqIndex);

                len = 0;
                errflag = 0;
                if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    /* usb non-standard request processing */
                    /* errflag = 0xFF; if this request or cmd dose not support */
                    /* usb non-standard request processing */
                    #if 0
                       if( USBHD_SetupReqType & USB_REQ_TYP_CLASS )
                       {
                           /* Class requests */
                           switch( USBHD_SetupReqCode )
                           {
                               case CDC_GET_LINE_CODING:
                                   pUSBHD_Descr = (uint8_t *)&Uart.Com_Cfg[ 0 ];
                                   len = 7;
                                   break;

                               case CDC_SET_LINE_CODING:
                                   break;

                               case CDC_SET_LINE_CTLSTE:
                                   break;

                               case CDC_SEND_BREAK:
                                   break;

                               default:
                                   errflag = 0xff;
                                   break;
                           }
                       }
                       else if( USBHD_SetupReqType & USB_REQ_TYP_VENDOR )
                       {
                           /* Manufacturer request */
                       }
                       else
                       {
                           errflag = 0xFF;
                       }

                       /* Copy Descriptors to Endp0 DMA buffer */
                       len = (USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                       memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                       pUSBHD_Descr += len;
                    #endif

                    if (( USBHD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS)
                    {
                        switch( USBHD_SetupReqCode )
                        {
                            case HID_SET_REPORT:
                                break;

                            case HID_GET_REPORT:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    len = DEF_USBD_UEP0_SIZE;
                                    memcpy(USBHD_EP0_Buf,&HID_Report_Buffer[0],DEF_USBD_UEP0_SIZE);
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_SET_IDLE:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_HidIdle = USBHD_EP0_Buf[ 3 ];
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_SET_PROTOCOL:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_HidProtocol = USBHD_EP0_Buf[ 2 ];
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_GET_IDLE:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = USBHD_HidIdle;
                                    len = 1;
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;
                            case HID_GET_PROTOCOL:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = USBHD_HidProtocol;
                                    len = 1;
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;
                            default:
                                errflag = 0xFF;
                                break;
                        }
                    }

                }
                else
                {
                    /* usb standard request processing */
                    switch( USBHD_SetupReqCode )
                    {
                        /* get device/configuration/string/report/... descriptors */
                        case USB_GET_DESCRIPTOR:
                            //AsyncPrintf(">>>>USB_GET_DESCRIPTOR:%x\r\n",USB_GET_DESCRIPTOR);
                            switch( (uint8_t)( USBHD_SetupReqValue >> 8 ) )
                            {
                                /* get usb device descriptor */
                                case USB_DESCR_TYP_DEVICE:
                                    pUSBHD_Descr = MyDevDescr;
                                    len = DEF_USBD_DEVICE_DESC_LEN;
                                    break;

                                /* get usb configuration descriptor */
                                case USB_DESCR_TYP_CONFIG:
                                    pUSBHD_Descr = MyCfgDescr;
                                    len = DEF_USBD_CONFIG_DESC_LEN;
                                    break;

                                /* get usb report descriptor */
                                case USB_DESCR_TYP_REPORT:
                                    if( USBHD_SetupReqIndex == 0x00 )
                                    {
                                        pUSBHD_Descr = MyHIDReportDesc;
                                        len = sizeof(MyHIDReportDesc);
                                    }
                                    else if( USBHD_SetupReqIndex == 0x01 )
                                    {
                                        //pUSBHD_Descr = MouseRepDesc;
                                        //len = DEF_USBD_REPORT_DESC_LEN_MS;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;

                               /* get usb hid descriptor */
                                case USB_DESCR_TYP_HID:
                                    if( USBHD_SetupReqIndex == 0x00 )
                                    {
                                        pUSBHD_Descr = &MyCfgDescr[ 18 ];
                                        len = 9;
                                    }
                                    else if( USBHD_SetupReqIndex == 0x01 )
                                    {
                                        //pUSBHD_Descr = &MyCfgDescr[ 43 ];
                                        //len = 9;
                                        errflag = 0xFF;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;

                                /* get usb string descriptor */
                                case USB_DESCR_TYP_STRING:
                                    switch( (uint8_t)( USBHD_SetupReqValue & 0xFF ) )
                                    {
                                        /* Descriptor 0, Language descriptor */
                                        case DEF_STRING_DESC_LANG:
                                            pUSBHD_Descr = MyLangDescr;
                                            len = DEF_USBD_LANG_DESC_LEN;
                                            break;

                                        /* Descriptor 1, Manufacturers String descriptor */
                                        case DEF_STRING_DESC_MANU:
                                            pUSBHD_Descr = MyManuInfo;
                                            len = DEF_USBD_MANU_DESC_LEN;
                                            break;

                                        /* Descriptor 2, Product String descriptor */
                                        case DEF_STRING_DESC_PROD:
                                            pUSBHD_Descr = MyProdInfo;
                                            len = DEF_USBD_PROD_DESC_LEN;
                                            break;

                                        /* Descriptor 3, Serial-number String descriptor */
                                        case DEF_STRING_DESC_SERN:
                                            pUSBHD_Descr = MySerNumInfo;
                                            len = DEF_USBD_SN_DESC_LEN;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                    break;

                                default :
                                    errflag = 0xFF;
                                    break;
                            }
                            //AsyncPrintf(">>>>USBHD_SetupReqLen:%x vs len:%x\r\n",USBHD_SetupReqLen,len);
                            /* Copy Descriptors to Endp0 DMA buffer */
                            if( USBHD_SetupReqLen>len )
                            {
                                USBHD_SetupReqLen = len;
                            }
                            len = (USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                            //AsyncPrintf(">>>>USBHD_SetupReqLen2:%x vs len:%x\r\n",USBHD_SetupReqLen,len);
                            memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                            pUSBHD_Descr += len;
                            break;

                        /* Set usb address */
                        case USB_SET_ADDRESS:
                            //AsyncPrintf(">>>>USB_SET_ADDRESS:%x\r\n",USB_SET_ADDRESS);
                            USBHD_DevAddr = (uint8_t)( USBHD_SetupReqValue & 0xFF );
                            break;

                        /* Get usb configuration now set */
                        case USB_GET_CONFIGURATION:
                            //AsyncPrintf(">>>>USB_GET_CONFIGURATION:%x\r\n",USB_GET_CONFIGURATION);
                            USBHD_EP0_Buf[0] = USBHD_DevConfig;
                            if ( USBHD_SetupReqLen > 1 )
                            {
                                USBHD_SetupReqLen = 1;
                            }
                            break;

                        /* Set usb configuration to use */
                        case USB_SET_CONFIGURATION:
                            //AsyncPrintf(">>>>USB_SET_CONFIGURATION:%x\r\n",USB_SET_CONFIGURATION);
                            USBHD_DevConfig = (uint8_t)( USBHD_SetupReqValue & 0xFF );
                            USBHD_DevEnumStatus = 0x01;
                            break;

                        /* Clear or disable one usb feature */
                        case USB_CLEAR_FEATURE:
                            //AsyncPrintf(">>>>USB_CLEAR_FEATURE:%x\r\n",USB_CLEAR_FEATURE);
                            if ( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* clear one device feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    /* clear usb sleep status, device not prepare to sleep */
                                    USBHD_DevSleepStatus &= ~0x01;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Clear End-point Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_IN | DEF_UEP1 ):
                                            /* Set End-point 1 IN NAK */
                                            R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_NAK;
                                            break;

                                        case ( DEF_UEP_OUT | DEF_UEP2 ):
                                            /* Set End-point 2 OUT ACK */
                                            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_ACK;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP3 ):
                                            /* Set End-point 3 IN NAK */
                                            R8_UEP3_CTRL = ( R8_UEP3_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_NAK;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        /* set or enable one usb feature */
                        case USB_SET_FEATURE:
                            //AsyncPrintf(">>>>USB_SET_FEATURE:%x\r\n",USB_SET_FEATURE);
                            if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* Set Device Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    if( MyCfgDescr[ 7 ] & 0x20 )
                                    {
                                        /* Set Wake-up flag, device prepare to sleep */
                                        USBHD_DevSleepStatus |= 0x01;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Set End-point Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    /* Set end-points status stall */
                                    switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_IN | DEF_UEP1 ):
                                            /* Set End-point 1 IN STALL */
                                            R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_STALL;
                                            break;

                                        case ( DEF_UEP_OUT | DEF_UEP2 ):
                                            /* Set End-point 2 OUT STALL */
                                            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_STALL;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP3 ):
                                            /* Set End-point 3 IN STALL */
                                            R8_UEP3_CTRL = ( R8_UEP3_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_STALL;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        /* This request allows the host to select another setting for the specified interface  */
                        case USB_GET_INTERFACE:
                            //AsyncPrintf(">>>>USB_GET_INTERFACE:%x\r\n",USB_GET_INTERFACE);
                            USBHD_EP0_Buf[0] = 0x00;
                            if ( USBHD_SetupReqLen > 1 )
                            {
                                USBHD_SetupReqLen = 1;
                            }
                            break;

                        case USB_SET_INTERFACE:
                            //AsyncPrintf(">>>>USB_SET_INTERFACE:%x\r\n",USB_SET_INTERFACE);
                            break;

                        /* host get status of specified device/interface/end-points */
                        case USB_GET_STATUS:
                            //AsyncPrintf(">>>>USB_GET_STATUS:%x\r\n",USB_GET_STATUS);
                            USBHD_EP0_Buf[ 0 ] = 0x00;
                            USBHD_EP0_Buf[ 1 ] = 0x00;
                            if ( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                if( USBHD_DevSleepStatus & 0x01 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = 0x02;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                {
                                    case ( DEF_UEP_IN | DEF_UEP1 ):
                                        if( ( R8_UEP1_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_OUT | DEF_UEP2 ):
                                        if( ( R8_UEP2_CTRL & MASK_UEP_R_RES ) == UEP_R_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_IN | DEF_UEP3 ):
                                        if( ( R8_UEP3_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    default:
                                        errflag = 0xFF;
                                        break;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }

                            if ( USBHD_SetupReqLen > 2 )
                            {
                                USBHD_SetupReqLen = 2;
                            }

                            break;

                        default:
                            //AsyncPrintf(">>>>default:%x\r\n",0);
                            errflag = 0xFF;
                            break;
                    }
                }

                /* errflag = 0xFF means a request not support or some errors occurred, else correct */
                if( errflag == 0xFF)
                {
                    /* if one request not support, return stall */
                    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
                }
                else
                {
                    /* end-point 0 data Tx/Rx */
                    if( USBHD_SetupReqType & DEF_UEP_IN )
                    {
                        /* tx */
                        len = ( USBHD_SetupReqLen > DEF_USBD_UEP0_SIZE ) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                        USBHD_SetupReqLen -= len;
                        R8_UEP0_T_LEN  = len;
                        R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                    }
                    else
                    {
                        /* rx */
                        if( USBHD_SetupReqLen == 0 )
                        {
                            R8_UEP0_T_LEN = 0;
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                        }
                        else
                        {
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK ;
                        }
                    }
                }
                break;

            /* Sof pack processing */
            case UIS_TOKEN_SOF:
                //AsyncPrintf(">>UIS_TOKEN_SOF:%x\r\n",UIS_TOKEN_SOF);
                break;

            default :
                break;
        }
        R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
    else if( intflag & RB_UIF_BUS_RST )
    {
        //AsyncPrintf(">RB_UIF_BUS_RST:%x\r\n",RB_UIF_BUS_RST);
        /* usb reset interrupt processing */
        USBHD_DevConfig = 0;
        USBHD_DevAddr = 0;
        USBHD_DevSleepStatus = 0;
        USBHD_DevEnumStatus = 0;

        R8_USB_DEV_AD = 0;
        USBHD_Device_Endp_Init( );
        R8_USB_INT_FG |= RB_UIF_BUS_RST;
    }
    else if( intflag & RB_UIF_SUSPEND )
    {
        //AsyncPrintf(">RB_UIF_SUSPEND:%x\r\n",RB_UIF_SUSPEND);
        /* usb suspend interrupt processing */
        R8_USB_INT_FG = RB_UIF_SUSPEND;
        if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )
        {
            USBHD_DevSleepStatus |= 0x02;
            if( USBHD_DevSleepStatus == 0x03 )
            {
                // Handling usb sleep here 
            }
        }
        else
        {
            USBHD_DevSleepStatus &= ~0x02;
        }
        
    }
    else
    {
        /* other interrupts */
        //AsyncPrintf(">other interrupts:%x\r\n",intflag);
        R8_USB_INT_FG = intflag;
    }
    IRQHandler_RxCnt++;
    IRQHandler_RxCnt = IRQHandler_RxCnt % BUFSIZE;
}
#elif 0
void USBHD_IRQHandler( void )
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = R8_USB_INT_FG;
    intst = R8_USB_INT_ST;

    AsyncPrintf("intflag:%x instst:%x\r\n",intflag,intst);

    if( intflag & RB_UIF_TRANSFER )
    {
        AsyncPrintf(">RB_UIF_TRANSFER:%x\r\n",RB_UIF_TRANSFER);
        switch ( intst & MASK_UIS_TOKEN )
        {
            /* data-in stage processing */
            case UIS_TOKEN_IN:
                AsyncPrintf(">>UIS_TOKEN_IN:%x\r\n",UIS_TOKEN_IN);
                switch ( intst & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
                {
                    /* end-point 0 data in interrupt */
                    /* end-point 0 data in interrupt */
                    case UIS_TOKEN_IN | DEF_UEP0:
                        if( USBHD_SetupReqLen == 0 )
                        {
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                        }
                        if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            /* Non-standard request endpoint 0 Data upload */
                        }
                        else
                        {
                            switch( USBHD_SetupReqCode )
                            {
                                case USB_GET_DESCRIPTOR:
                                        len = USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                                        memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                                        USBHD_SetupReqLen -= len;
                                        pUSBHD_Descr += len;
                                        R8_UEP0_T_LEN   = len;
                                        R8_UEP0_CTRL ^= RB_UEP_T_TOG;
                                        break;

                                case USB_SET_ADDRESS:
                                        R8_USB_DEV_AD = ( R8_USB_DEV_AD & RB_UDA_GP_BIT ) | USBHD_DevAddr;
                                        break;

                                default:
                                        break;
                            }
                        }
                        break;


                    /* end-point 2 data in interrupt */
                    case UIS_TOKEN_IN | DEF_UEP2:
                        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
                        R8_UEP2_CTRL ^= RB_UEP_T_TOG;
                        USBHD_Endp_Busy[ DEF_UEP2 ] = 0;
                        break;

                    default :
                        break;
                }
                break;

            /* data-out stage processing */
            case UIS_TOKEN_OUT:
                AsyncPrintf(">>UIS_TOKEN_OUT:%x\r\n",UIS_TOKEN_OUT);
                switch ( intst & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
                {
                    /* end-point 0 data out interrupt */
                    case UIS_TOKEN_OUT | DEF_UEP0:
                        if ( intst & RB_UIS_TOG_OK )
                        {
                            if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                            {
                                if (( USBHD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS)
                                {
                                    switch( USBHD_SetupReqCode )
                                    {
                                        case HID_SET_REPORT:
                                            memcpy(&HID_Report_Buffer[0],USBHD_EP0_Buf,DEF_USBD_FS_PACK_SIZE);
                                            HID_Set_Report_Flag = SET_REPORT_WAIT_DEAL;
                                            R8_UEP0_CTRL = ( R8_UEP0_CTRL & ~MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            break;
                                        default:
                                            break;
                                    }
                                }
                            }
                            else
                            {
                                /* Standard request end-point 0 Data download */
                                    /* Add your code here */
                            }
                        }
                        break;

                    /* end-point 1 data out interrupt */
                    case UIS_TOKEN_OUT | DEF_UEP1:
                        if ( intst & RB_UIS_TOG_OK )
                        {
                            /* Write In Buffer */
                            R8_UEP1_CTRL ^= RB_UEP_R_TOG;
                            #if 0
                            RingBuffer_Comm.PackLen[RingBuffer_Comm.LoadPtr] = (uint16_t)(R8_USB_RX_LEN);
                            RingBuffer_Comm.LoadPtr ++;
                            if(RingBuffer_Comm.LoadPtr == DEF_Ring_Buffer_Max_Blks)
                            {
                                RingBuffer_Comm.LoadPtr = 0;
                            }
                            R16_UEP1_DMA = (uint32_t)(&Data_Buffer[ (RingBuffer_Comm.LoadPtr) * DEF_USBD_FS_PACK_SIZE] );
                            RingBuffer_Comm.RemainPack ++;
                            if(RingBuffer_Comm.RemainPack >= DEF_Ring_Buffer_Max_Blks-DEF_RING_BUFFER_REMINE)
                            {
                                R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
                                RingBuffer_Comm.StopFlag = 1;
                            }
                            #endif
                        }
                        break;
                    default:
                        break;

                }
                break;

            /* Setup stage processing */
            case UIS_TOKEN_SETUP:
                AsyncPrintf(">>UIS_TOKEN_SETUP:%x\r\n",UIS_TOKEN_SETUP);
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
                /* Store All Setup Values */
                USBHD_SetupReqType  = pUSBHD_SetupReqPak->bRequestType;
                USBHD_SetupReqCode  = pUSBHD_SetupReqPak->bRequest;
                USBHD_SetupReqLen   = pUSBHD_SetupReqPak->wLength;
                USBHD_SetupReqValue = pUSBHD_SetupReqPak->wValue;
                USBHD_SetupReqIndex = pUSBHD_SetupReqPak->wIndex;
                len = 0;
                errflag = 0;
                if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    if (( USBHD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS)
                    {
                        switch( USBHD_SetupReqCode )
                        {
                            case HID_SET_REPORT:
                                break;

                            case HID_GET_REPORT:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    len = DEF_USBD_UEP0_SIZE;
                                    memcpy(USBHD_EP0_Buf,&HID_Report_Buffer[0],DEF_USBD_UEP0_SIZE);
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_SET_IDLE:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_HidIdle = USBHD_EP0_Buf[ 3 ];
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_SET_PROTOCOL:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_HidProtocol = USBHD_EP0_Buf[ 2 ];
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_GET_IDLE:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = USBHD_HidIdle;
                                    len = 1;
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;
                            case HID_GET_PROTOCOL:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = USBHD_HidProtocol;
                                    len = 1;
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;
                            default:
                                errflag = 0xFF;
                                break;
                        }
                    }
                }
                else
                {
                    /* usb standard request processing */
                    switch( USBHD_SetupReqCode )
                    {
                        /* get device/configuration/string/report/... descriptors */
                        case USB_GET_DESCRIPTOR:
                            switch( (uint8_t)( USBHD_SetupReqValue >> 8 ) )
                            {
                                /* get usb device descriptor */
                                case USB_DESCR_TYP_DEVICE:
                                    pUSBHD_Descr = MyDevDescr;
                                    len = DEF_USBD_DEVICE_DESC_LEN;
                                    break;

                                /* get usb configuration descriptor */
                                case USB_DESCR_TYP_CONFIG:
                                    pUSBHD_Descr = MyCfgDescr;
                                    len = DEF_USBD_CONFIG_DESC_LEN;
									break;
                              /* get usb report descriptor */
                              case USB_DESCR_TYP_REPORT:
                                    if (USBHD_SetupReqIndex == 0)
                                    {
                                        pUSBHD_Descr = MyHIDReportDesc;
                                        len = DEF_USBD_REPORT_DESC_LEN;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;
                                /* get hid descriptor */
                                case USB_DESCR_TYP_HID:
                                    if (USBHD_SetupReqIndex == 0)
                                    {
                                        pUSBHD_Descr = &MyCfgDescr[18];
                                        len = 0x09;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;

                                /* get usb string descriptor */
                                case USB_DESCR_TYP_STRING:
                                    switch( (uint8_t)( USBHD_SetupReqValue & 0xFF ) )
                                    {
                                        /* Descriptor 0, Language descriptor */
                                        case DEF_STRING_DESC_LANG:
                                            pUSBHD_Descr = MyLangDescr;
                                            len = DEF_USBD_LANG_DESC_LEN;
                                            break;

                                        /* Descriptor 1, Manufacturers String descriptor */
                                        case DEF_STRING_DESC_MANU:
                                            pUSBHD_Descr = MyManuInfo;
                                            len = DEF_USBD_MANU_DESC_LEN;
                                            break;

                                        /* Descriptor 2, Product String descriptor */
                                        case DEF_STRING_DESC_PROD:
                                            pUSBHD_Descr = MyProdInfo;
                                            len = DEF_USBD_PROD_DESC_LEN;
                                            break;

                                        /* Descriptor 3, Serial-number String descriptor */
                                        case DEF_STRING_DESC_SERN:
                                            pUSBHD_Descr = MySerNumInfo;
                                            len = DEF_USBD_SN_DESC_LEN;
                                            break;

                                            default:
                                                errflag = 0xFF;
                                                break;
                                        }
                                        break;
  
                                    default :
                                        errflag = 0xFF;
                                        break;
                                }

                            /* Copy Descriptors to Endp0 DMA buffer */
                            if( USBHD_SetupReqLen>len )
                            {
                                USBHD_SetupReqLen = len;
                            }
                            len = (USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                            memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                            pUSBHD_Descr += len;
                            break;

                        /* Set usb address */
                        case USB_SET_ADDRESS:
                            USBHD_DevAddr = (uint8_t)( USBHD_SetupReqValue & 0xFF );
                            break;

                        /* Get usb configuration now set */
                        case USB_GET_CONFIGURATION:
                            USBHD_EP0_Buf[0] = USBHD_DevConfig;
                            if ( USBHD_SetupReqLen > 1 )
                            {
                                USBHD_SetupReqLen = 1;
                            }
                            break;

                        /* Set usb configuration to use */
                        case USB_SET_CONFIGURATION:
                            USBHD_DevConfig = (uint8_t)( USBHD_SetupReqValue & 0xFF );
                            USBHD_DevEnumStatus = 0x01;
                            break;

                        /* Clear or disable one usb feature */
                        case USB_CLEAR_FEATURE:
                            if ( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* clear one device feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    /* clear usb sleep status, device not prepare to sleep */
                                    USBHD_DevSleepStatus &= ~0x01;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_OUT | DEF_UEP1 ):
                                            /* Set End-point 1 OUT ACK*/
                                            R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_ACK;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP2 ):
                                            /* Set End-point 2 IN NAK */
                                            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_NAK;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        /* set or enable one usb feature */
                        case USB_SET_FEATURE:
                            if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* Set Device Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    if( MyCfgDescr[ 7 ] & 0x20 )
                                    {
                                        /* Set Wake-up flag, device prepare to sleep */
                                        USBHD_DevSleepStatus |= 0x01;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Set End-point Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {

                                    switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_OUT | DEF_UEP1 ):
                                            R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_STALL;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP2 ):
                                            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_STALL;
                                            break;
                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        /* This request allows the host to select another setting for the specified interface  */
                        case USB_GET_INTERFACE:
                            USBHD_EP0_Buf[0] = 0x00;
                            if ( USBHD_SetupReqLen > 1 )
                            {
                                USBHD_SetupReqLen = 1;
                            }
                            break;

                        case USB_SET_INTERFACE:
                            break;

                        /* host get status of specified device/interface/end-points */
                        case USB_GET_STATUS:
                            USBHD_EP0_Buf[ 0 ] = 0x00;
                            USBHD_EP0_Buf[ 1 ] = 0x00;
                            if ( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                if( USBHD_DevSleepStatus & 0x01 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = 0x02;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                {
                                    case ( DEF_UEP_OUT | DEF_UEP1 ):
                                        if( ( R8_UEP1_CTRL & MASK_UEP_R_RES ) == UEP_R_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_IN | DEF_UEP2 ):
                                        if( ( R8_UEP2_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;


                                    default:
                                        errflag = 0xFF;
                                        break;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }

                            if ( USBHD_SetupReqLen > 2 )
                            {
                                USBHD_SetupReqLen = 2;
                            }

                            break;

                        default:
                            errflag = 0xFF;
                            break;
                    }
                }

                /* errflag = 0xFF means a request not support or some errors occurred, else correct */
                if( errflag == 0xFF)
                {
                    /* if one request not support, return stall */
                    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
                }
                else
                {
                    /* end-point 0 data Tx/Rx */
                    if( USBHD_SetupReqType & DEF_UEP_IN )
                    {
                        len = ( USBHD_SetupReqLen > DEF_USBD_UEP0_SIZE ) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                        USBHD_SetupReqLen -= len;
                        R8_UEP0_T_LEN  = len;
                        R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                    }
                    else
                    {
                        if( USBHD_SetupReqLen == 0 )
                        {
                            R8_UEP0_T_LEN = 0;
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                        }
                        else
                        {
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK ;
                        }
                    }
                }
                break;

            /* Sof pack processing */
            case UIS_TOKEN_SOF:
                break;

            default :
                break;
        }
        R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
    else if( intflag & RB_UIF_BUS_RST )
    {
        AsyncPrintf(">RB_UIF_BUS_RST:%x\r\n",RB_UIF_BUS_RST);
        /* usb reset interrupt processing */
        USBHD_DevConfig = 0;
        USBHD_DevAddr = 0;
        USBHD_DevSleepStatus = 0;
        USBHD_DevEnumStatus = 0;

        R8_USB_DEV_AD = 0;
        USBHD_Device_Endp_Init( );
        R8_USB_INT_FG |= RB_UIF_BUS_RST;
    }
    else if( intflag & RB_UIF_SUSPEND )
    {
        AsyncPrintf(">RB_UIF_SUSPEND:%x\r\n",RB_UIF_SUSPEND);
        /* usb suspend interrupt processing */
        R8_USB_INT_FG = RB_UIF_SUSPEND;
        if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )
        {
            USBHD_DevSleepStatus |= 0x02;
            if( USBHD_DevSleepStatus == 0x03 )
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USBHD_DevSleepStatus &= ~0x02;
        }
        
    }
    else
    {
        /* other interrupts */
        AsyncPrintf(">other interrupts:%x\r\n",intflag);
        R8_USB_INT_FG = intflag;
    }
}

#else
void USBHD_IRQHandler( void )
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = R8_USB_INT_FG;
    intst   = R8_USB_INT_ST;

    if( intflag & RB_UIF_TRANSFER )
    {
        switch ( intst & MASK_UIS_TOKEN )
        {
            /* data-in stage processing */
            case UIS_TOKEN_IN:
                switch ( intst & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
                {
                    /* end-point 0 data in interrupt */
                    case UIS_TOKEN_IN | DEF_UEP0:
                        if( USBHD_SetupReqLen == 0 )
                        {
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                        }
                        if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            /* Non-standard request endpoint 0 Data upload */
                        }
                        else
                        {
                            switch( USBHD_SetupReqCode )
                            {
                                case USB_GET_DESCRIPTOR:
                                        len = USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                                        memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                                        USBHD_SetupReqLen -= len;
                                        pUSBHD_Descr += len;
                                        R8_UEP0_T_LEN   = len;
                                        R8_UEP0_CTRL ^= RB_UEP_T_TOG;
                                        break;

                                case USB_SET_ADDRESS:
                                        R8_USB_DEV_AD = ( R8_USB_DEV_AD & RB_UDA_GP_BIT ) | USBHD_DevAddr;
                                        break;

                                default:
                                        break;
                            }
                        }
                        break;


                    /* end-point 2 data in interrupt */
                    case UIS_TOKEN_IN | DEF_UEP2:
                        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
                        R8_UEP2_CTRL ^= RB_UEP_T_TOG;
                        USBHD_Endp_Busy[ DEF_UEP2 ] = 0;
                        break;

                    default :
                        break;
                }
                break;

            /* data-out stage processing */
            case UIS_TOKEN_OUT:
                switch ( intst & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
                {
                    /* end-point 0 data out interrupt */
                    case UIS_TOKEN_OUT | DEF_UEP0:
                        if ( intst & RB_UIS_TOG_OK )
                        {
                            if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                            {
                                if (( USBHD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS)
                                {
                                    switch( USBHD_SetupReqCode )
                                    {
                                        case HID_SET_REPORT:
                                            memcpy(&HID_Report_Buffer[0],USBHD_EP0_Buf,DEF_USBD_FS_PACK_SIZE);
                                            HID_Set_Report_Flag = SET_REPORT_WAIT_DEAL;
                                            R8_UEP0_CTRL = ( R8_UEP0_CTRL & ~MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            break;
                                        default:
                                            break;
                                    }
                                }
                            }
                            else
                            {
                                /* Standard request end-point 0 Data download */
                                    /* Add your code here */
                            }
                        }
                        break;

                    /* end-point 1 data out interrupt */
                    case UIS_TOKEN_OUT | DEF_UEP1:
                        if ( intst & RB_UIS_TOG_OK )
                        {
                            /* Write In Buffer */
                            R8_UEP1_CTRL ^= RB_UEP_R_TOG;
                        }
                        break;
                    default:
                        break;

                }
                break;

            /* Setup stage processing */
            case UIS_TOKEN_SETUP:
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
                /* Store All Setup Values */
                USBHD_SetupReqType  = pUSBHD_SetupReqPak->bRequestType;
                USBHD_SetupReqCode  = pUSBHD_SetupReqPak->bRequest;
                USBHD_SetupReqLen   = pUSBHD_SetupReqPak->wLength;
                USBHD_SetupReqValue = pUSBHD_SetupReqPak->wValue;
                USBHD_SetupReqIndex = pUSBHD_SetupReqPak->wIndex;
                len = 0;
                errflag = 0;
                if ( ( USBHD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    if (( USBHD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS)
                    {
                        switch( USBHD_SetupReqCode )
                        {
                            case HID_SET_REPORT:
                                break;

                            case HID_GET_REPORT:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    len = DEF_USBD_UEP0_SIZE;
                                    memcpy(USBHD_EP0_Buf,&HID_Report_Buffer[0],DEF_USBD_UEP0_SIZE);
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_SET_IDLE:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_HidIdle = USBHD_EP0_Buf[ 3 ];
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_SET_PROTOCOL:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_HidProtocol = USBHD_EP0_Buf[ 2 ];
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;

                            case HID_GET_IDLE:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = USBHD_HidIdle;
                                    len = 1;
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;
                            case HID_GET_PROTOCOL:
                                if( USBHD_SetupReqIndex == 0x00 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = USBHD_HidProtocol;
                                    len = 1;
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                                break;
                            default:
                                errflag = 0xFF;
                                break;
                        }
                    }
                }
                else
                {
                    /* usb standard request processing */
                    switch( USBHD_SetupReqCode )
                    {
                        /* get device/configuration/string/report/... descriptors */
                        case USB_GET_DESCRIPTOR:
                            switch( (uint8_t)( USBHD_SetupReqValue >> 8 ) )
                            {
                                /* get usb device descriptor */
                                case USB_DESCR_TYP_DEVICE:
                                    pUSBHD_Descr = MyDevDescr;
                                    len = DEF_USBD_DEVICE_DESC_LEN;
                                    break;

                                /* get usb configuration descriptor */
                                case USB_DESCR_TYP_CONFIG:
                                    pUSBHD_Descr = MyCfgDescr;
                                    len = DEF_USBD_CONFIG_DESC_LEN;
									break;
                              /* get usb report descriptor */
                              case USB_DESCR_TYP_REPORT:
                                    if (USBHD_SetupReqIndex == 0)
                                    {
                                        pUSBHD_Descr = MyHIDReportDesc;
                                        len = DEF_USBD_REPORT_DESC_LEN;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;
                                /* get hid descriptor */
                                case USB_DESCR_TYP_HID:
                                    if (USBHD_SetupReqIndex == 0)
                                    {
                                        pUSBHD_Descr = &MyCfgDescr[18];
                                        len = 0x09;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;

                                /* get usb string descriptor */
                                case USB_DESCR_TYP_STRING:
                                    switch( (uint8_t)( USBHD_SetupReqValue & 0xFF ) )
                                    {
                                        /* Descriptor 0, Language descriptor */
                                        case DEF_STRING_DESC_LANG:
                                            pUSBHD_Descr = MyLangDescr;
                                            len = DEF_USBD_LANG_DESC_LEN;
                                            break;

                                        /* Descriptor 1, Manufacturers String descriptor */
                                        case DEF_STRING_DESC_MANU:
                                            pUSBHD_Descr = MyManuInfo;
                                            len = DEF_USBD_MANU_DESC_LEN;
                                            break;

                                        /* Descriptor 2, Product String descriptor */
                                        case DEF_STRING_DESC_PROD:
                                            pUSBHD_Descr = MyProdInfo;
                                            len = DEF_USBD_PROD_DESC_LEN;
                                            break;

                                        /* Descriptor 3, Serial-number String descriptor */
                                        case DEF_STRING_DESC_SERN:
                                            pUSBHD_Descr = MySerNumInfo;
                                            len = DEF_USBD_SN_DESC_LEN;
                                            break;

                                            default:
                                                errflag = 0xFF;
                                                break;
                                        }
                                        break;
  
                                    default :
                                        errflag = 0xFF;
                                        break;
                                }

                            /* Copy Descriptors to Endp0 DMA buffer */
                            if( USBHD_SetupReqLen>len )
                            {
                                USBHD_SetupReqLen = len;
                            }
                            len = (USBHD_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                            memcpy( USBHD_EP0_Buf, pUSBHD_Descr, len );
                            pUSBHD_Descr += len;
                            break;

                        /* Set usb address */
                        case USB_SET_ADDRESS:
                            USBHD_DevAddr = (uint8_t)( USBHD_SetupReqValue & 0xFF );
                            break;

                        /* Get usb configuration now set */
                        case USB_GET_CONFIGURATION:
                            USBHD_EP0_Buf[0] = USBHD_DevConfig;
                            if ( USBHD_SetupReqLen > 1 )
                            {
                                USBHD_SetupReqLen = 1;
                            }
                            break;

                        /* Set usb configuration to use */
                        case USB_SET_CONFIGURATION:
                            USBHD_DevConfig = (uint8_t)( USBHD_SetupReqValue & 0xFF );
                            USBHD_DevEnumStatus = 0x01;
                            break;

                        /* Clear or disable one usb feature */
                        case USB_CLEAR_FEATURE:
                            if ( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* clear one device feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    /* clear usb sleep status, device not prepare to sleep */
                                    USBHD_DevSleepStatus &= ~0x01;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_OUT | DEF_UEP1 ):
                                            /* Set End-point 1 OUT ACK*/
                                            R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_ACK;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP2 ):
                                            /* Set End-point 2 IN NAK */
                                            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_NAK;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        /* set or enable one usb feature */
                        case USB_SET_FEATURE:
                            if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* Set Device Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    if( MyCfgDescr[ 7 ] & 0x20 )
                                    {
                                        /* Set Wake-up flag, device prepare to sleep */
                                        USBHD_DevSleepStatus |= 0x01;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Set End-point Feature */
                                if( (uint8_t)( USBHD_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {

                                    switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_OUT | DEF_UEP1 ):
                                            R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_STALL;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP2 ):
                                            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_STALL;
                                            break;
                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        /* This request allows the host to select another setting for the specified interface  */
                        case USB_GET_INTERFACE:
                            USBHD_EP0_Buf[0] = 0x00;
                            if ( USBHD_SetupReqLen > 1 )
                            {
                                USBHD_SetupReqLen = 1;
                            }
                            break;

                        case USB_SET_INTERFACE:
                            break;

                        /* host get status of specified device/interface/end-points */
                        case USB_GET_STATUS:
                            USBHD_EP0_Buf[ 0 ] = 0x00;
                            USBHD_EP0_Buf[ 1 ] = 0x00;
                            if ( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                if( USBHD_DevSleepStatus & 0x01 )
                                {
                                    USBHD_EP0_Buf[ 0 ] = 0x02;
                                }
                            }
                            else if( ( USBHD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                switch( (uint8_t)( USBHD_SetupReqIndex & 0xFF ) )
                                {
                                    case ( DEF_UEP_OUT | DEF_UEP1 ):
                                        if( ( R8_UEP1_CTRL & MASK_UEP_R_RES ) == UEP_R_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_IN | DEF_UEP2 ):
                                        if( ( R8_UEP2_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_STALL )
                                        {
                                            USBHD_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;


                                    default:
                                        errflag = 0xFF;
                                        break;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }

                            if ( USBHD_SetupReqLen > 2 )
                            {
                                USBHD_SetupReqLen = 2;
                            }

                            break;

                        default:
                            errflag = 0xFF;
                            break;
                    }
                }

                /* errflag = 0xFF means a request not support or some errors occurred, else correct */
                if( errflag == 0xFF)
                {
                    /* if one request not support, return stall */
                    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
                }
                else
                {
                    /* end-point 0 data Tx/Rx */
                    if( USBHD_SetupReqType & DEF_UEP_IN )
                    {
                        len = ( USBHD_SetupReqLen > DEF_USBD_UEP0_SIZE ) ? DEF_USBD_UEP0_SIZE : USBHD_SetupReqLen;
                        USBHD_SetupReqLen -= len;
                        R8_UEP0_T_LEN  = len;
                        R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                    }
                    else
                    {
                        if( USBHD_SetupReqLen == 0 )
                        {
                            R8_UEP0_T_LEN = 0;
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                        }
                        else
                        {
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK ;
                        }
                    }
                }
                break;

            /* Sof pack processing */
            case UIS_TOKEN_SOF:
                break;

            default :
                break;
        }
        R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
    else if( intflag & RB_UIF_BUS_RST )
    {
        /* usb reset interrupt processing */
        USBHD_DevConfig = 0;
        USBHD_DevAddr = 0;
        USBHD_DevSleepStatus = 0;
        USBHD_DevEnumStatus = 0;

        R8_USB_DEV_AD = 0;
        USBHD_Device_Endp_Init( );
        R8_USB_INT_FG |= RB_UIF_BUS_RST;
    }
    else if( intflag & RB_UIF_SUSPEND )
    {
        /* usb suspend interrupt processing */
        R8_USB_INT_FG = RB_UIF_SUSPEND;
        if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )
        {
            USBHD_DevSleepStatus |= 0x02;
            if( USBHD_DevSleepStatus == 0x03 )
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USBHD_DevSleepStatus &= ~0x02;
        }
    }
    else
    {
        /* other interrupts */
        R8_USB_INT_FG = intflag;
    }
}
#endif

/*********************************************************************
 * @fn      USBHD_Send_Resume
 *
 * @brief   USBHD device sends wake-up signal to host
 *
 * @return  none
 */
void USBHD_Send_Resume(void)
{
    R8_UDEV_CTRL ^= RB_UD_LOW_SPEED;
    Delay_Ms(5);
    R8_UDEV_CTRL ^= RB_UD_LOW_SPEED;
    Delay_Ms(1);
}
