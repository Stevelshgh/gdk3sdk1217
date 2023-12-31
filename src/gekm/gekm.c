/********************************** (C) COPYRIGHT *******************************
 * File Name          : gekm.c
 * Author             : GEDU Tech.
 * Version            : V1.0.0
 * Date               : 2022/12/11
 * Description        : Firware for Nano Keyboad and Mouse
 * Copyright (c) 2021-22 GEDU Tech. Co., Ltd.
 *******************************************************************************/
// !program d:\\work\\gdk3\\sdk\\src\\gekm\\gekm.hex
/*
 * @Note
 * Composite Keyboard and Mouse Example:
 * This example uses PB12-PB15 and PA4-PA7 to simulate keyboard key pressing and mouse
 * movement respectively, active low. 
 * At the same time, it also uses USART2(PA3) to receive the specified data sent from 
 * the host to simulate the pressing and releasing of the following specific keyboard 
 * keys. Data is sent in hexadecimal format and 1 byte at a time.
 * 'W' -> 0x1A
 * 'A' -> 0x04 
 * 'S' -> 0x16
 * 'D' -> 0x07
 */
 
 /*
 * @Note
 * In addition, when the system frequency is selected as the USB clock source, only 72MHz/48MHz 
 * are supported.
 */


/*******************************************************************************/
/* Header Files */
#include "km_sender.h"
#include "usbhd.h"
#include "hw_config.h"
#include "usb_lib.h"

__attribute__((section(".ndb_note")))
const char elf_name[] = "gekm";


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program
 *
 * @return  none
 */
int main( void )
{
    /* Initialize system configuration */
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	Delay_Init( );
	USART_Printf_Init( 115200 );
	printf( "SystemClk:%d\r\n", SystemCoreClock );

	/* Initialize USART2 for receiving the specified keyboard data */
	USART2_Init( 115200 );
	printf( "USART2 Init OK!\r\n" );

	/* Initialize GPIO for keyboard scan */
	KB_Scan_Init( );
	KB_Sleep_Wakeup_Cfg( );
	printf( "KB Scan Init OK!\r\n" );

	/* Initialize GPIO for mouse scan */
	MS_Scan_Init( );
	MS_Sleep_Wakeup_Cfg( );
	printf( "MS Scan Init OK!\r\n" );

	/* Initialize timer for Keyboard and mouse scan timing */
	TIM3_Init( 1, SystemCoreClock / 10000 - 1 );
	printf( "TIM3 Init OK!\r\n" );

	/* Initialize USBHD interface to communicate with the host  */
	USBHD_RCC_Init( );
	USBHD_Device_Init( ENABLE );
	USB_Sleep_Wakeup_CFG( );

	// init the receiver port (usb1), aka the USBD controller, we call it North(Bei) USB.
	Set_USBConfig(); 
    USB_Init();	    
 	USB_Interrupts_Config();   

	printf("USBHD Composite KM Device Test\r\n");

	while( 1 )
    {
	    if( USBHD_DevEnumStatus )
	    {
	        /* Handle keyboard scan data */
	        KB_Scan_Handle(  );

            /* Handle keyboard lighting */
	        KB_LED_Handle( );
            
            /* Handle mouse scan data */
            MS_Scan_Handle( );

            /* Handle USART2 receiving data */
            USART2_Receive_Handle( );
	    }
    }
}
