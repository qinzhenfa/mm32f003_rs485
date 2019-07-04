/**
******************************************************************************
* @file     main.c
* @author   AE team
* @version  V1.0.8
* @date     10/04/2019
* @brief
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, MindMotion SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2019 MindMotion</center></h2>
*/
#include "HAL_device.h"
#include "stdio.h"
#include "HAL_conf.h"

char printBuf[100];
void Uart_ConfigInit(u32 bound);
void UartSendGroup(u8* buf, u16 len);
void UartSendAscii(char *str);

void RCC_ConfigInit(void);
void GPIO_fromSWDConfig(void);
void setPA13PA14asSWD(void);
#define GPIOA_MASK_PIN 0x99ff
#define GPIOB_MASK_PIN 0xffff
#define GPIOC_MASK_PIN 0xffff
#define GPIOD_MASK_PIN 0x0004
static u32 cnt = 0;
/*******************************************************************************
* @name   : InitSystick
* @brief  : Init Systick
* @param  : None
* @retval : void
*******************************************************************************/
void InitSystick()
{
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0x00);
}

/*******************************************************************************
* @name   : SysTick_Handler
* @brief  : Systick interrupt
* @param  : None
* @retval : void
*******************************************************************************/
void SysTick_Handler()
{

    cnt++;
}

void delay(u32 uldelay)
{
    cnt = 0;
    while(1) {
        if(cnt >= uldelay)
            break;
    }
}

/*******************************************************************************
* @name   : GPIO_Clock_Set
* @brief  : RCC clock set
* @param  : Portx , State
* @retval : void
*******************************************************************************/
void GPIO_Clock_Set(GPIO_TypeDef* GPIOx, FunctionalState NewState)
{

    if(GPIOx == GPIOA) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, NewState);                         //GPIO clock starts
    }
    if(GPIOx == GPIOB) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, NewState);                         //GPIO clock starts
    }
    if(GPIOx == GPIOC) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, NewState);                         //GPIO clock starts
    }
    if(GPIOx == GPIOD) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, NewState);                         //GPIO clock starts
    }
}
void setPA13PA14SWDasPushPullGPIO(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    GPIO_Clock_Set(GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_2);                       //AF to not pararell
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_2);                       //AF to not pararell

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13 | GPIO_Pin_14;                   //SPI_MOSI | SPI_SCK
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


}

void setPA13PA14asSWD(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_Clock_Set(GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_0);                       //Set PA13 as SWDIO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_0);                       //Set PA14 as SWDCLK

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13 | GPIO_Pin_14;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //Pull-up input
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}
/*******************************************************************************
* @name   : main
* @brief  : GPIO output toggle
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{

    //--------------------------------------- NOTE ----------------------------------------------
    //     use the Flywire connet the PA13 or PA14 to LD(LED) pin for watch the LED blink
    //     the Appliction can be debug, use Flash download to MCU, Push RESET pin or Power-Up the
    //     Board , MCU run then find the LED blink us 1 Second
    //---------------------------------------- End  ----------------------------------------------
    InitSystick();
    delay(1000);                    //set 1000mS Delay for Debug tools to Capture and Hold the MCU;
    setPA13PA14SWDasPushPullGPIO();
    Uart_ConfigInit(9600);
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nsprintf ok\r\n"));
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nStart GPIO from SWD test\r\n"));
    while(1) {
        GPIO_SetBits( GPIOA,  GPIO_Pin_13);
        GPIO_SetBits( GPIOA,  GPIO_Pin_14);                                          //toggle IO, on/off LED
        delay(1000);
        GPIO_ResetBits( GPIOA,  GPIO_Pin_13);
        GPIO_ResetBits( GPIOA,  GPIO_Pin_14);
        delay(1000);
    }
    setPA13PA14asSWD();           //let Debug tools re-connect MCU and Erase MCU,
    //this function need to move in final application
    while(1) {

    }
}



/*******************************************************************************
* @name   : Uart_ConfigInit
* @brief  : Uart Config Init
* @param  : u32 bound
* @retval : void
*******************************************************************************/
void Uart_ConfigInit(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);

    UART_InitStructure.UART_BaudRate = bound;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;

    UART_Init(UART, &UART_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    UART_Cmd(UART, ENABLE);
}

/*******************************************************************************
* @name   : UartSendByte
* @brief  : Uart Send Byte
* @param  : u8 dat
* @retval : void
*******************************************************************************/
void UartSendByte(u8 dat)
{
    UART_SendData(UART, dat);
    while(!UART_GetFlagStatus(UART, UART_FLAG_TXEPT));
}

/*******************************************************************************
* @name   : UartSendGroup
* @brief  : Uart Send Group
* @param  : u8* buf,u16 len
* @retval : void
*******************************************************************************/
void UartSendGroup(u8* buf, u16 len)
{
    while(len--)
        UartSendByte(*buf++);
}

/*******************************************************************************
* @name   : UartSendAscii
* @brief  : Uart Send Ascii
* @param  : char *str
* @retval : void
*******************************************************************************/
void UartSendAscii(char *str)
{
    while(*str)
        UartSendByte(*str++);
}


/**
* @}
*/

/**
* @}
*/

/**
* @}
*/
/*-------------------------(C) COPYRIGHT 2019 MindMotion ----------------------*/
