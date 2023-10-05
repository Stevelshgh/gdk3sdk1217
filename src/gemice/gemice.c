/********************************** (C) COPYRIGHT *******************************
 * File Name          : gemice.c
 * Author             : WCH and GEDU Lab
 * Version            : V1.0.0
 * Date               : 2021/08/08 - 2022-12-15
 * Description        : Main program for magic mouse and keybaord.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*
 * @Note
 * Composite Keyboard and Mouse Example: send mouse and keyboard event automatically.
 * !program d:\\work\\gdk3\\sdk\\src\\gemice\\gemice.hex
 */

#include "debug.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_prop.h"
#include "usbd_composite_km.h"

/* Global define */

/* Global Variable */
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Delay_Init();
	USART_Printf_Init(921600);
	printf("SystemClk:%d\r\n", SystemCoreClock);
	printf("USBD mouse demo on GDK3\r\n");

	/* Initialize USART2 for receiving the specified keyboard data */
	USART2_Init(115200);
	printf("USART2 Init OK!\r\n");

	Set_USBConfig();
	USB_Init();
	USB_Interrupts_Config();

	printf("Sending start...\n");
	int gm_counter = 0;
	while (1) {
		if (bDeviceState == CONFIGURED)	{
			++gm_counter;
			gm_send();
		}
	}
}
