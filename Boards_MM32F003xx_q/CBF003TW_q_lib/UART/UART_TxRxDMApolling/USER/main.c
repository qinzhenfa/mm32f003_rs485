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
#include "string.h"
#include "HAL_conf.h"

#define LENGTH 16

static unsigned char rxdata[LENGTH];
static unsigned char txdata[LENGTH] = {0x55, 0xaa, 0xbb, 0xdd, 0x33, 0x77, 0x22, 0x11, 0x55, 0xaa, 0xbb, 0xdd, 0x33, 0x77, 0x22, 0x11};

char printBuf[100];
void Uart_ConfigInit(u32 bound);
void UartSendByte(u8 dat);
void UartSendGroup(u8* buf, u16 len);
void UartSendAscii(char *str);




u32 SysTick_Count;
u32 gDlycnt;

/*******************************************************************************
* @name   : InitSystick
* @brief  : Init Systick
* @param  : None
* @retval : void
*******************************************************************************/
void InitSystick(void)
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
void SysTick_Handler(void)
{
    if(SysTick_Count++ > 500) {
        SysTick_Count = 0;
    }
    if(gDlycnt > 0) gDlycnt --;
}

static void delay(u32 uldelay)
{
    gDlycnt = uldelay;
    while(1) {
        if(gDlycnt == 0)
            break;
    }
}

/********************************************************************************************************
**函数信息 UART2_DMA_TX_Init
**功能描述 : UART2_DMA_TX数据初始化
**输入参数 ：无
**输出参数 ：无
**函数备注 ：
********************************************************************************************************/
void UART2_DMA_TX_Init(void)
{
    DMA_InitTypeDef  DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART2->TDR); //UART2_DMA_TX数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)txdata; //UART2_DMA_TX的数据存放基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//数据从memory到外设
    DMA_InitStructure.DMA_BufferSize = 1;//UART2_DMA_TX数据个数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);//SPI2 RX为DMA通道4
}

/********************************************************************************************************
**函数信息 ：UART2_DMA_RX_Init
**功能描述 : UART2_DMA_RX初始化
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void UART2_DMA_RX_Init()
{
    DMA_InitTypeDef  DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART2->RDR); //UART2_DMA_RX数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rxdata;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//数据从外设到memory
    DMA_InitStructure.DMA_BufferSize = 1;//UART2_DMA_RX数据个数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不改变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//memory地址递增开启
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);//
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Set the DMA Channeln's Peripheral address.
/// @param  channel : where n can be 1 to 7 for DMA1 to select the DMA Channel.
/// @param  length : Transmit lengths.
/// @retval : None
////////////////////////////////////////////////////////////////////////////////
void DMA_Set_TransmitLen(DMA_Channel_TypeDef* channel, u16 length)
{
    channel->CNDTR = length;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Set the DMA Channeln's Peripheral address.
/// @param  channel : where n can be 1 to 7 for DMA1 to select the DMA Channel.
/// @param  address : DMA memery address.
/// @retval : None
////////////////////////////////////////////////////////////////////////////////
void DMA_Set_MemoryAddress(DMA_Channel_TypeDef* channel, u32 address)
{
    channel->CMAR = address;
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Set the DMA Channeln's Peripheral address.
/// @param  channel : where n can be 1 to 7 for DMA1 to select the DMA Channel.
/// @param  length : Transmit lengths.
/// @retval : None
////////////////////////////////////////////////////////////////////////////////
void DMA_Set_MemoryInc_Enable(DMA_Channel_TypeDef* channel, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if(NewState == ENABLE) {
        channel->CCR |= DMA_MemoryInc_Enable;
    } else {
        channel->CCR  &= ~(DMA_MemoryInc_Enable);
    }
}
////////////////////////////////////////////////////////////////////////////////
/// @brief   DMA transmits packet
/// @param   ch: Pointer to a DMA channel.
/// @param   addr: The memory Buffer address.
/// @param   len: The length of data to be transferred.
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////
void DRV_DMA_TransmitPacket(DMA_Channel_TypeDef* ch, u32 addr, u32 len)
{
    DMA_ITConfig(ch, DMA_IT_TC, ENABLE);
    DMA_Cmd(ch, DISABLE);
    DMA_Set_MemoryAddress(ch, addr);
    DMA_Set_TransmitLen(ch, len);
    DMA_Cmd(ch, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief   DMA transmit and receive packet
/// @param   ptx_buf: Pointer to SPI DMA send buffer(include send and recv bytes).
/// @param   prx_buf: Pointer to SPI DMA recv buffer(include send and recv bytes).
/// @param   len: The length of data , length equal send + recv len).
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////********************************************************************************************************/
void UART2_WriteBuf(u8 *ptx_buf,  u32 len)
{
    DMA_Set_MemoryInc_Enable(DMA1_Channel4, ENABLE);
    DRV_DMA_TransmitPacket(DMA1_Channel4, (u32)ptx_buf, len);

//    UART_DMACmd(UART2,UART_DMAReq_EN,ENABLE);

    while(!DMA_GetFlagStatus(DMA1_FLAG_TC4));
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief   DMA transmit and receive packet
/// @param   ptx_buf: Pointer to SPI DMA send buffer(include send and recv bytes).
/// @param   prx_buf: Pointer to SPI DMA recv buffer(include send and recv bytes).
/// @param   len: The length of data , length equal send + recv len).
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////********************************************************************************************************/
void UART2_Readbuf( u8 *prx_buf, u32 len)
{
    DMA_Set_MemoryInc_Enable(DMA1_Channel5, ENABLE);
    DRV_DMA_TransmitPacket(DMA1_Channel5, (u32)prx_buf, len);

    while(!DMA_GetFlagStatus(DMA1_FLAG_TC5));
    DMA_ClearFlag(DMA1_FLAG_TC5);

    DMA_Cmd(DMA1_Channel5, DISABLE);
}
#define UARTTIMEOUT 0xf
#define UARTSUCESSFUL 0x1

////////////////////////////////////////////////////////////////////////////////
/// @brief   DMA transmit and receive packet
/// @param   ptx_buf: Pointer to SPI DMA send buffer(include send and recv bytes).
/// @param   prx_buf: Pointer to SPI DMA recv buffer(include send and recv bytes).
/// @param   len: The length of data , length equal send + recv len).
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////********************************************************************************************************/
int UART2ReadbufTimeOut( u8 *prx_buf, u32 len, u32 Timeout)
{
    int result;
    u32 i = 0;
    DMA_Set_MemoryInc_Enable(DMA1_Channel5, ENABLE);
    DRV_DMA_TransmitPacket(DMA1_Channel5, (u32)prx_buf, len);

    while(1) {
        if(!DMA_GetFlagStatus(DMA1_FLAG_TC5) ) {
            delay(1);
            i++;
            if(i > Timeout) {
                result = UARTTIMEOUT;
                break;
            }
        } else {
            result = UARTSUCESSFUL;
            break;
        }
    }
    DMA_ClearFlag(DMA1_FLAG_TC5);

    DMA_Cmd(DMA1_Channel5, DISABLE);
    return result;
}

uint8_t UART2ReadBurst(uint8_t* pBuf, uint16_t len)
{
    if(len == 0)
        return 0;
    UART2_Readbuf(pBuf, len);
    return len;

}

void UART2WriteBurst(uint8_t* pBuf, uint16_t len)
{
    if(len == 0)
        return ;
    UART2_WriteBuf(pBuf, len);
    return ;
}
u8  rxbuf[10];
/*******************************************************************************
* @name   : main
* @brief  : Send the received data
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    int ret;
	
		//my core
	
		//
	

    InitSystick();
    delay(10);
    UART2_DMA_TX_Init();
    UART2_DMA_RX_Init();
    Uart_ConfigInit(115200);
    UART_DMACmd(UART2, UART_DMAReq_EN, ENABLE);
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "sprintf ok\r\n"));          //pintf stdio data
    UartSendAscii("UartSendAscii\r\n");                                         //printf string
    UartSendAscii("Please input data ,end with '\\n' \r\n");

    while(1) {
        memset(rxbuf, 0, sizeof(rxbuf));
        ret = UART2ReadbufTimeOut(rxbuf, 10, 100);
        if(ret == UARTSUCESSFUL) {
            UartSendGroup(rxbuf, 10);
        }
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
    UART2_WriteBuf(&dat, (u32)1);
}

/*******************************************************************************
* @name   : UartSendGroup
* @brief  : Uart Send Group
* @param  : u8* buf,u16 len
* @retval : void
*******************************************************************************/
void UartSendGroup(u8* buf, u16 len)
{
    UART2_WriteBuf(buf, (u32)len);
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
