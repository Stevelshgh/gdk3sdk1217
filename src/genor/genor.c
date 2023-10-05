/*
geflash by RaymondZ based on the Flash_Program sample from WCH.
2022-11-25
*/
#include <stdint.h>
#include "debug.h"
#include "io_config.h"

/* Global define */
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
#define PAGE_WRITE_START_ADDR  ((uint32_t)0x0800F000) /* Start from 60K */
#define PAGE_WRITE_END_ADDR    ((uint32_t)0x08010000) /* End at 63K */
#define FLASH_PAGE_SIZE                   1024
#define FLASH_PAGES_TO_BE_PROTECTED FLASH_WRProt_Pages60to63

/* Global Variable */
uint32_t EraseCounter = 0x0, Address = 0x0;

uint32_t WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0;
uint32_t NbrOfPage;
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
volatile TestStatus MemoryEraseStatus = PASSED;
u32 buf[32];

#define GENOR_OPT_SKIP_ERASE  1

/*********************************************************************
 * @fn      Flash_Test
 *
 * @brief   Flash Program Test.
 *
 * @return  none
 */
void Flash_Test( uint32_t options, uint16_t Data )
{
    FLASH_Unlock();
    WRPR_Value = FLASH_GetWriteProtectionOptionByte();

    NbrOfPage = ( PAGE_WRITE_END_ADDR - PAGE_WRITE_START_ADDR ) / FLASH_PAGE_SIZE;

    if( ( WRPR_Value & FLASH_PAGES_TO_BE_PROTECTED ) != 0x00 )
    {
        FLASH_ClearFlag( FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR );
        if( (options& GENOR_OPT_SKIP_ERASE) == 0) {   
            for( EraseCounter = 0; ( EraseCounter < NbrOfPage ) && ( FLASHStatus == FLASH_COMPLETE ); EraseCounter++ )
            {
                FLASHStatus = FLASH_ErasePage( PAGE_WRITE_START_ADDR + ( FLASH_PAGE_SIZE * EraseCounter ) );
                if( FLASHStatus != FLASH_COMPLETE )
                {
                    printf( "FLASH Erase ERR at Page%d\r\n", EraseCounter + 60 );
                }
                printf( "FLASH Erase Page%d...\r\n", EraseCounter + 60 );
            }
        } else {
            printf("erase is skipped for fun\r\n");
        }

        Address = PAGE_WRITE_START_ADDR;
        printf( "Erase Cheking...\r\n" );
        while( ( Address < PAGE_WRITE_END_ADDR ) && ( MemoryEraseStatus != FAILED ) )
        {
            if( ( *( __IO uint16_t * ) Address ) != 0xFFFF )
            {
                MemoryEraseStatus = FAILED;
            }
            Address += 2;
        }
        if( MemoryEraseStatus == FAILED )
        {
            printf( "Erase Flash FAIL!\r\n" );
            printf( "\r\n" );
        }
        else
        {
            printf( "Erase Flash PASS!\r\n" );
            printf( "\r\n" );
        }

        Address = PAGE_WRITE_START_ADDR;
        printf( "Programing...\r\n" );
        while( ( Address < PAGE_WRITE_END_ADDR ) && ( FLASHStatus == FLASH_COMPLETE ) )
        {
            FLASHStatus = FLASH_ProgramHalfWord( Address, Data );
            Address = Address + 2;
        }

        Address = PAGE_WRITE_START_ADDR;
        printf( "Program Cheking...\r\n" );
        while( ( Address < PAGE_WRITE_END_ADDR ) && ( MemoryProgramStatus != FAILED ) )
        {
            if( ( *( __IO uint16_t * ) Address ) != Data )
            {
                MemoryProgramStatus = FAILED;
            }
            Address += 2;
        }
        if( MemoryProgramStatus == FAILED )
        {
            printf( "Memory Program FAIL!\r\n" );
            printf( "\r\n" );
        }
        else
        {
            printf( "Memory Program PASS!\r\n" );
            printf( "\r\n" );
        }

    }
    else
    {
        MemoryProgramStatus = FAILED;
        printf( "Error to program the flash : The desired pages are write protected\r\n" );
    }

    FLASH_Lock();

}

/*********************************************************************
 * @fn      Flash_Test_Fast
 *
 * @brief   Flash Fast Program Test.
 *
 * @return  none
 */
void Flash_Test_Fast( void )
{
    u8 i, Verity_Flag = 0;

    for( i = 0; i < 32; i++ ){
        buf[i] = i;
    }

    FLASH_Unlock_Fast();

    FLASH_ErasePage_Fast( 0x0800E000 );

    printf( "128Byte Page Erase Sucess\r\n" );

    FLASH_BufReset();
    FLASH_BufLoad( 0x0800E000, buf[0], buf[1], buf[2], buf[3] );
    FLASH_BufLoad( 0x0800E000 + 0x10, buf[4], buf[5], buf[6], buf[7] );
    FLASH_BufLoad( 0x0800E000 + 0x20, buf[8], buf[9], buf[10], buf[11] );
    FLASH_BufLoad( 0x0800E000 + 0x30, buf[12], buf[13], buf[14], buf[15] );
    FLASH_BufLoad( 0x0800E000 + 0x40, buf[16], buf[17], buf[18], buf[19] );
    FLASH_BufLoad( 0x0800E000 + 0x50, buf[20], buf[21], buf[22], buf[23] );
    FLASH_BufLoad( 0x0800E000 + 0x60, buf[24], buf[25], buf[26], buf[27] );
    FLASH_BufLoad( 0x0800E000 + 0x70, buf[28], buf[29], buf[30], buf[31] );
    FLASH_ProgramPage_Fast( 0x0800E000 );

    printf( "128Byte Page Program Sucess\r\n" );

    FLASH_Lock_Fast();

    for( i = 0; i < 32; i++ ){
        if( buf[i] == *( u32 * )( 0x0800E000 + 4 * i ) )
        {
            Verity_Flag = 0;
        }
        else
        {
            Verity_Flag = 1;
            break;
        }
    }

    if( Verity_Flag )
    {
        printf( "128Byte Page Verity Fail\r\n" );
    }
    else
    {
        printf( "128Byte Page Verity Sucess\r\n" );
    }
}

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
    
	while(blinking--)
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

    printf( "Flash Program Test\r\n" );
    gd_blink(5);
    Flash_Test(0, 0xABCD);
    Flash_Test(GENOR_OPT_SKIP_ERASE, 0x5678);
    Flash_Test_Fast();    

	printf("GPIO Toggle TEST\r\n");

    gd_blink(2);

	while(printing) {
	    printf("Hi GDK3!\n");
	}
    
    return (int)long_imm;
}

