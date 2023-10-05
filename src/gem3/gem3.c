/*
gem3 program by RaymondZ based on the GPIO sample from WCH.
2022-7-31
*/
#include <stdint.h>
#include "debug.h"
#include "io_config.h"

/* Global define */


/* Global Variable */ 


/*******************************************************************************
* Function Name  : GPIO_Toggle_INIT
* Description    : Initializes GPIOA.0
* Input          : None
* Return         : None
*******************************************************************************/
void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    HAL_GPIO_WritePin(CONNECTED_LED_PORT, CONNECTED_LED_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.GPIO_Pin = CONNECTED_LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(CONNECTED_LED_PORT, &GPIO_InitStructure);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;              
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;           
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);     
               
}
/*

*/
/** Debug Unit: Set status of Connected LED.
\param bit status of the Connect LED.
           - 1: Connect LED ON: debugger is connected to CMSIS-DAP Debug Unit.
           - 0: Connect LED OFF: debugger is not connected to CMSIS-DAP Debug Unit.
*/
void LED_CONNECTED_OUT(uint32_t bit)
{
    if (bit & 1)
        CONNECTED_LED_PORT->BCR/*BRR*/ = CONNECTED_LED_PIN; // LED on
    else
        CONNECTED_LED_PORT->BSHR/*BSRR*/ = CONNECTED_LED_PIN;// LED off
}

int  gd_blink(int blinking)
{
    int delay = 500;
    int counter = 0;
    
	while(blinking)
    {	
		Delay_Ms(delay);
		PBout(12) ^= (1<<0);
        LED_CONNECTED_OUT((counter++)%2);
	}

    return counter;
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Return         : None
*******************************************************************************/
int main(void)
{
    int blinking = 1, printing = 1;
    int baudrate = 115200;
    uint64_t long_imm = 0x123456789abcdef0;

    printf("Hello longlong integer:%llx\n", long_imm);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
	USART_Printf_Init(baudrate);
	GPIO_Toggle_INIT();
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf("GPIO Toggle TEST\r\n");
    
    gd_blink(blinking);

	while(printing) {
	    printf("Hi GDK3!\n");
	}
    
    return (int)long_imm;
}

