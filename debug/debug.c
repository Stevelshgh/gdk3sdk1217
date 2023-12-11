/********************************** (C) COPYRIGHT  *******************************
* File Name          : debug.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2019/10/15
* Description        : This file contains all the functions prototypes for UART
*                      Printf , Delay functions.
*******************************************************************************/
#include "debug.h"

/* Support Printf Function Definition */
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;


static u8  p_us=0;								   
static u16 p_ms=0;					

/*******************************************************************************
* Function Name  : Delay_Init
* Description    : Initializes Delay Funcation.
* Input          : None
* Return         : None
*******************************************************************************/	
void Delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	
	p_us=SystemCoreClock/8000000;			  
	p_ms=(u16)p_us*1000;					   
}								    

/*******************************************************************************
* Function Name  : Delay_Us
* Description    : Microsecond Delay Time.
* Input          : n：Microsecond number.
*                     n * p_us < 0xFFFFFF
* Return         : None
*******************************************************************************/		    								   
void Delay_Us(u32 n)
{		
	u32 i;	
	
	SysTick->LOAD=n*p_us; 						  		 
	SysTick->VAL=0x00;        					
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
	
	do
	{
		i=SysTick->CTRL;
	}while((i&0x01)&&!(i&(1<<16)));
	
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	
	SysTick->VAL =0X00;     	 
}

/*******************************************************************************
* Function Name  : Delay_Ms
* Description    : Millisecond Delay Time.
* Input          : n：Millisecond number.
*                     n * p_ms < 0xFFFFFF
* Return         : None
*******************************************************************************/	
void Delay_Ms(u16 n)
{	 		  	  
	u32 i;	
	
	SysTick->LOAD=(u32)n*p_ms;				
	SysTick->VAL =0x00;							
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
	
	do
	{
		i=SysTick->CTRL;
	}while((i&0x01)&&!(i&(1<<16)));	
	
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	
	SysTick->VAL =0X00;        	    
} 

/*******************************************************************************
* Function Name  : fputc
* Description    : Support Printf Function 
* Input          : data: UART send Data.
* Return         : data: UART send Data.
*******************************************************************************/

int My_fputc(int data, FILE *f)
{
#if (DEBUG == DEBUG_UART1)
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	USART_SendData(USART1, (u8) data);
#elif (DEBUG == DEBUG_UART2)
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
  USART_SendData(USART2, (u8) data);	
#elif (DEBUG == DEBUG_UART3)
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
  USART_SendData(USART3, (u8) data);	
#endif
	
  return data;
}

__attribute__ ((used))
int _write(int file, char* ptr, int len)
{
   for(int i = 0; i < len; ++i)
   {
       My_fputc(ptr[i], 0);
   }
   return len;
}

#define BUFSIZE  1024
char myprintf_buf[BUFSIZE];
#define Buffer_len 4096
u8 ComBuffer1[Buffer_len-1] = {0};     //format:len + id + Data + len+id+data+...                                            
u32 ComTxCnt1 = 0, ComRxCnt1 = 0; 

IRQHandler_Data_t IRQHandler_Data_Buffer[BUFSIZE-1];
u32 IRQHandler_TxCnt = 0, IRQHandler_RxCnt = 0;

void Debug_printf(const char* fmt, ...)
{
    va_list args;
    int n;

    va_start(args, fmt);
    n = vsnprintf(myprintf_buf, BUFSIZE, fmt, args);
    va_end(args);
    int i = 0;
    for(i = 0; i < n; i++)
    {
       //HAL_UART_Transmit(&huart2, (uint8_t *)&myprintf_buf[i], 1, 0xFFFF); //根据不同的平台，修改串口输出的函数
       My_fputc(myprintf_buf[i],0);
    }
}

u8 _IsComBuffFull()
{
  int iResult=0;
  int iComTxCnt1=ComTxCnt1,iComRxCnt1=ComRxCnt1;
  if(iComTxCnt1<iComRxCnt1)
  {
    if(iComRxCnt1 - iComTxCnt1>=Buffer_len-1)
      iResult= 1;
  }else if(iComTxCnt1>iComRxCnt1){
    if(iComRxCnt1 + Buffer_len - iComTxCnt1>=Buffer_len-1)
      iResult = 1;
  } else
  {
    iResult = 0;
  }
  return iResult;
}

void AsyncPrintf(const char* fmt, ...)
{
  return;
    va_list args;
    int n;

    va_start(args, fmt);
    n = vsnprintf(myprintf_buf, BUFSIZE, fmt, args);
    va_end(args);  

    int i = 0;
    for(i = 0; i < n; i++)
    {
       ComBuffer1[ComRxCnt1++] = myprintf_buf[i];
       ComRxCnt1 = ComRxCnt1 % Buffer_len;

      if(_IsComBuffFull())
      {
        break;
      }

    }

}

void DoWorkComSend()
{
  int i=0;
  int iComTxCnt1=ComTxCnt1,iComRxCnt1=ComRxCnt1;
  int iCnt=0;
  if(iComTxCnt1<iComRxCnt1)
  {
    iCnt = iComRxCnt1 - iComTxCnt1;
  }else if(iComTxCnt1>iComRxCnt1){
    iCnt = iComRxCnt1 + Buffer_len - iComTxCnt1;
  }

  for(i=0 ; i < iCnt ; i++)
  {
    My_fputc(ComBuffer1[iComTxCnt1++],0);  
    iComTxCnt1 = iComTxCnt1 % Buffer_len;
  } 

  ComTxCnt1 = iComTxCnt1;

}

void DoWorkUSBHD_IRQHandler()
{
  int i=0;
  int iIRQHandler_TxCnt=IRQHandler_TxCnt,iIRQHandler_RxCnt=IRQHandler_RxCnt;
  int iCnt=0;
  if(iIRQHandler_TxCnt<iIRQHandler_RxCnt)
  {
    iCnt = iIRQHandler_RxCnt - iIRQHandler_TxCnt;
  }else if(iIRQHandler_TxCnt>iIRQHandler_RxCnt){
    iCnt = iIRQHandler_RxCnt + BUFSIZE - iIRQHandler_TxCnt;
  }

  for(i=0 ; i < iCnt ; i++)
  {
    DecodeUSBHD_IRQHandlerData(&IRQHandler_Data_Buffer[iIRQHandler_TxCnt]);
    IRQHandler_TxCnt = iIRQHandler_RxCnt % BUFSIZE;
  } 

  IRQHandler_TxCnt = iIRQHandler_TxCnt;

}



/*******************************************************************************
* Function Name  : USART_Printf_Init
* Description    : Initializes the USARTx peripheral.  
* Input          : baudrate: USART communication baud rate.                                 
* Return         : None
*******************************************************************************/
void USART_Printf_Init(u32 baudrate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	
#if (DEBUG == DEBUG_UART1)	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
#elif (DEBUG == DEBUG_UART2)
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
#elif (DEBUG == DEBUG_UART3)	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
#endif	
   
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;

#if (DEBUG == DEBUG_UART1)	
  USART_Init(USART1, &USART_InitStructure); 
  USART_Cmd(USART1, ENABLE);  
	
#elif (DEBUG == DEBUG_UART2)	
  USART_Init(USART2, &USART_InitStructure); 
  USART_Cmd(USART2, ENABLE);
	
#elif (DEBUG == DEBUG_UART3)
  USART_Init(USART3, &USART_InitStructure); 
  USART_Cmd(USART3, ENABLE);
	
#endif	
}


