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

void GPIO_ConfigInit(void);
void RCC_ConfigInit(void);

#define GPIOA_MASK_PIN 0x99ff
#define GPIOB_MASK_PIN 0xffff
#define GPIOC_MASK_PIN 0xffff
#define GPIOD_MASK_PIN 0x0004

u16 gpioInputBuf[4];
u16 gpioTempBuf[4];

/*******************************************************************************
* @name   : main
* @brief  : GPIO input data
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    Uart_ConfigInit(9600);
    RCC_ConfigInit();
    GPIO_ConfigInit();

    while(1) {
#if 1
        gpioTempBuf[ 0 ] = GPIO_ReadInputData(GPIOA) & GPIOA_MASK_PIN;          //get GPIOA input data;
        gpioTempBuf[ 1 ] = GPIO_ReadInputData(GPIOB) & GPIOB_MASK_PIN;          //get GPIOB input data;
        gpioTempBuf[ 2 ] = GPIO_ReadInputData(GPIOC) & GPIOC_MASK_PIN;          //get GPIOC input data;
        gpioTempBuf[ 3 ] = GPIO_ReadInputData(GPIOD) & GPIOD_MASK_PIN;          //get GPIOD input data;

        if(gpioTempBuf[ 0 ] != gpioInputBuf[0]) {                               //if level of IO changed,printf current status;
            gpioInputBuf[0] = gpioTempBuf[ 0 ];
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "GPIOA = %4x\r\n", gpioInputBuf[0]));
        }
        if(gpioTempBuf[ 1 ] != gpioInputBuf[1]) {
            gpioInputBuf[1] = gpioTempBuf[ 1 ];
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "GPIOB = %4x\r\n", gpioInputBuf[1]));
        }
        if(gpioTempBuf[ 2 ] != gpioInputBuf[2]) {
            gpioInputBuf[2] = gpioTempBuf[ 2 ];
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "GPIOC = %4x\r\n", gpioInputBuf[2]));
        }
        if(gpioTempBuf[ 3 ] != gpioInputBuf[3]) {
            gpioInputBuf[3] = gpioTempBuf[ 3 ];
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "GPIOD = %4x\r\n", gpioInputBuf[3]));
        }
#endif
    }
}

/*******************************************************************************
* @name   : RCC_ConfigInit
* @brief  : RCC config
* @param  : None
* @retval : void
*******************************************************************************/
void RCC_ConfigInit(void)
{

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

}

/*******************************************************************************
* @name   : GPIO_ConfigInit
* @brief  : GPIO config
* @param  : None
* @retval : void
*******************************************************************************/
void GPIO_ConfigInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_All & GPIOB_MASK_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_All & GPIOC_MASK_PIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_All & GPIOD_MASK_PIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

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
    UART_SendData( UART, dat);
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
