/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

/*
 *@Note
 *Example routine to emulate a simulate USB-CDC Device, USE USART1(PA9/PA10)
 *Recommended to use UART2(PA2) as debug port, you can modify the debug port
 *by modifying the macro definition in debug.h
*/

__attribute__((section(".ndb_note")))
const char elf_name[] = ".ndb_note:gecdc.elf";


#include "USB_Device/ch32f10x_usbfs_device.h"
#include "debug.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	Delay_Init( );
	USART_Printf_Init( 115200 );
	//printf("SystemClk:%d\r\n",SystemCoreClock);
	RCC_Configuration( );

	/* Tim2 init */
    TIM2_Init( );

	/* Usart1 init */
    UART1_Init( 1, DEF_UARTx_BAUDRATE, DEF_UARTx_STOPBIT, DEF_UARTx_PARITY );

    /* USB20 device init */
    USBHD_RCC_Init( );
    USBHD_Device_Init( ENABLE );

	/* Tim3 init */
    TIM3_Init( );		

    //printf( "main\r\n" );

	while(1)
	{
        //UART1_DataRx_Deal(0);
        UART1_DataTx_Deal( );
	}
}
