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

void EXTI_Configuration(void);
void NVIC_Configuration(void);

/*******************************************************************************
* @name   : PVD_IRQHandler
* @brief  : PVD IRQ
* @param  : None
* @retval : void
*******************************************************************************/
void PVD_IRQHandler(void)
{

    if(EXTI_GetITStatus(EXTI_Line16) != RESET) {
        /* Clear the Key Button EXTI line pending bit */
        EXTI_ClearITPendingBit(EXTI_Line16);
    }
}

/*******************************************************************************
* @name   : main
* @brief  : Adjust mcu voltage generate PVD flag
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    /* Configure EXTI Line to generate an interrupt on falling edge */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    EXTI_Configuration();
    Uart_ConfigInit(9600);
    /* NVIC configuration */
    NVIC_Configuration();

    /* Configure the PVD Level to 1.8V */
//    PWR_PVDLevelConfig(PWR_PVDLevel_1V8);
//    PWR_PVDLevelConfig(PWR_PVDLevel_2V1);
//    PWR_PVDLevelConfig(PWR_PVDLevel_2V4);
//    PWR_PVDLevelConfig(PWR_PVDLevel_2V7);
//    PWR_PVDLevelConfig(PWR_PVDLevel_3V0);
//    PWR_PVDLevelConfig(PWR_PVDLevel_3V3);
//    PWR_PVDLevelConfig(PWR_PVDLevel_3V6);
//    PWR_PVDLevelConfig(PWR_PVDLevel_3V9);
//    PWR_PVDLevelConfig(PWR_PVDLevel_4V2);
//    PWR_PVDLevelConfig(PWR_PVDLevel_4V5);
    PWR_PVDLevelConfig(PWR_PVDLevel_4V8);
    /* Enable the PVD Output */
    PWR_PVDCmd(ENABLE);

    while(1) {
    }
}

/*******************************************************************************
* @name   : EXTI_Configuration
* @brief  : EXTI config
* @param  : None
* @retval : void
*******************************************************************************/
void EXTI_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure EXTI Line16(PVD Output) to generate an interrupt on rising and
       falling edges */
    EXTI_ClearITPendingBit(EXTI_Line16);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;// PVD map to EXTI_Line16
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
* @name   : NVIC_Configuration
* @brief  : NVIC config
* @param  : None
* @retval : void
*******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the PVD Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* @name   : RCC_ConfigInit
* @brief  : RCC config
* @param  : None
* @retval : void
*******************************************************************************/
void RCC_ConfigInit(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
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

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
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

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_3);

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

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    UART_Cmd(UART, ENABLE);

    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
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
