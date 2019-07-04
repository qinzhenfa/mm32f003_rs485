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
void UartSendByte(u8 dat);
void UartSendGroup(u8* buf, u16 len);
void UartSendAscii(char *str);

void RCC_ConfigInit(void);
void GPIO_ConfigInit(void);
void NVIC_ConfigInit(void);
void initGPIO_UART(void);
void uartConfig(unsigned int bound);

u16 rxdata;
char *pTx = printBuf;

u8 rx_cnt = 0, tx_flag;
u32 SysTick_Count;
u32 gDlycnt;

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
    if(SysTick_Count++ > 500) {
        SysTick_Count = 0;
    }
    if(gDlycnt > 0) gDlycnt --;
}

/*******************************************************************************
* @name   : main
* @brief  : Send the received data
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    InitSystick();
    RCC_ConfigInit();
    GPIO_ConfigInit();
    NVIC_ConfigInit();
    Uart_ConfigInit(9600);
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "sprintf ok\r\n"));          //pintf stdio data
    UartSendAscii("UartSendAscii\r\n");                                         //printf string
    UartSendAscii("Please input data ,end with '\\n' \r\n");

    while(1) {

        if(tx_flag != 0) {
            UART_ITConfig( UART,  UART_IT_TXIEN, ENABLE);                       //clear send flag;
        }
    }
}

/*******************************************************************************
* @name   : UART_IRQHandler
* @brief  : UART interrupt function
* @param  : None
* @retval : void
*******************************************************************************/
void UART_IRQHandler(void)
{
    if(UART_GetITStatus(UART, UART_IT_RXIEN) == SET) {                          //check receive status

        UART_ClearITPendingBit(UART, UART_IT_RXIEN);
        printBuf[rx_cnt % 100] = UART_ReceiveData(UART);

        if( printBuf[rx_cnt % 100] == '\n') {                                   //check '\n',and set send flag;
            tx_flag = 1;
            pTx = printBuf;
        }
        rx_cnt++;
    }
    if(UART->IER & UART_IT_TXIEN) {                                             //judge send empty interrupt enable
        if((UART_GetITStatus(UART, UART_IT_TXIEN) == SET)) {
            UartSendByte(*pTx++);
            UART_ClearITPendingBit(UART, UART_IT_TXIEN);
            rx_cnt--;
            if(rx_cnt == 0) {
                UART_ITConfig( UART,  UART_IT_TXIEN, DISABLE);
                tx_flag = 0;
            }
        }
    }
}

/*******************************************************************************
* @name   : NVIC_ConfigInit
* @brief  : NVIC Config Init
* @param  : None
* @retval : void
*******************************************************************************/
void NVIC_ConfigInit(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = UART_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    NVIC_Init(& NVIC_InitStruct);

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

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_4;
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
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_3); //RX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4); //TX

    UART_InitStructure.UART_BaudRate = bound;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;

    UART_Init(UART, &UART_InitStructure);

    UART_ITConfig( UART,  UART_IT_RXIEN, ENABLE);

    UART_Cmd(UART, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
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
