/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32f10x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2019/10/15
 * Description        : Main Interrupt Service Routines.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/ 
#include "ch32f10x_it.h"
#include "UART/UART.h"

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler( void )
{
}

/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   This function handles TIM2 exception.
 *
 * @return  none
 */
void TIM2_IRQHandler( void )
{
    /* Test IO comment:control led flick */
    static uint8_t tog;
    if(Uart.Rx_RemainLen)
    {
        if((Uart.Rx_TimeOut & 0x3ff)==0)
        {
            tog ? (GPIOB->BSHR = GPIO_Pin_12):(GPIOB->BCR = GPIO_Pin_12);
            tog ^= 1;
        }
    }else if(tog==1)
    {
        GPIOB->BSHR = GPIO_Pin_12;
        tog ^= 1;
    }
    /* uart timeout counts */
    Uart.Rx_TimeOut++;
    Uart.USB_Up_TimeOut++;

    /*if(Uart.Rx_RemainLen>DEF_UARTx_RX_BUF_LEN/2)
    {
        UART1_DataRx_Deal(1);
    }*/

    /* clear status */
    TIM_ClearITPendingBit( TIM2, TIM_IT_Update );
}

/*********************************************************************
 * @fn      TIM3_IRQHandler
 *
 * @brief   This function handles TIM2 exception.
 *
 * @return  none
 */
void TIM3_IRQHandler( void )
{
    /* Test IO comment:control led flick */
    if(UART1_DataRx_Deal(0)==1)
    {
        NVIC_DisableIRQ( TIM3_IRQn );
    }
    /* clear status */
    TIM_ClearITPendingBit( TIM3, TIM_IT_Update );
}


/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler( void )
{
    while( 1 )
    {
    }
}

/*********************************************************************
 * @fn      MemManage_Handler
 *
 * @brief   This function handles Memory Manage exception.
 *
 * @return  none
 */
void MemManage_Handler( void )
{
    while( 1 )
    {
    }
}

/*********************************************************************
 * @fn      BusFault_Handler
 *
 * @brief   This function handles Bus Fault exception.
 *
 * @return  none
 */
void BusFault_Handler( void )
{
    while( 1 )
    {
    }
}

/*********************************************************************
 * @fn      UsageFault_Handler
 *
 * @brief   This function handles Usage Fault exception.
 *
 * @return  none
 */
void UsageFault_Handler( void )
{
    while( 1 )
    {
    }
}

/*********************************************************************
 * @fn      SVC_Handler
 *
 * @brief   This function handles SVCall exception.
 *
 * @return  none
 */
void SVC_Handler( void )
{
}

/*********************************************************************
 * @fn      DebugMon_Handler
 *
 * @brief   This function handles Debug Monitor exception.
 *
 * @return  none
 */
void DebugMon_Handler( void )
{
}

/*********************************************************************
 * @fn      PendSV_Handler
 *
 * @brief   This function handles PendSVC exception.
 *
 * @return  none
 */
void PendSV_Handler( void )
{
}

/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   This function handles SysTick Handler.
 *
 * @return  none
 */
void SysTick_Handler( void )
{
}





