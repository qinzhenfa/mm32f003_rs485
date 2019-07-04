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
#include "HAL_conf.h"
#include "HAL_device.h"
#include "stdio.h"
#include "string.h"

#define DISSWDIO

static u32 cnt = 0;
#define LENGTH 16
//#define USENORMALPINASCS
static unsigned char rxdata[LENGTH];
static unsigned char txdata[LENGTH] = {0x55, 0xaa, 0xbb, 0xdd, 0x33, 0x77, 0x22, 0x11, 0x55, 0xaa, 0xbb, 0xdd, 0x33, 0x77, 0x22, 0x11};
extern uint32_t SystemCoreClock;
char printBuf[100];
void Uart_ConfigInit(u32 bound);
void UartSendGroup(u8* buf, u16 len);
void UartSendAscii(char *str);

void RCC_ConfigInit(void);
void GPIO_ConfigInit(void);



#define GPIO_Pin_0_2  GPIO_Pin_2

void SPIM_CSLow(SPI_TypeDef* SPIx);
void SPIM_CSHigh(SPI_TypeDef* SPIx);
unsigned int SPI_FLASH_WriteRead(SPI_TypeDef* SPIx, unsigned char tx_data);
void SPIM_Init(SPI_TypeDef* SPIx, unsigned short spi_baud_div);
void SPIM_RXEn(SPI_TypeDef* SPIx);
void SPIM_TXEn(SPI_TypeDef* SPIx);

unsigned char tmpdata[256];
unsigned char rxtmpdata[256];

#define READ            0x03
#define FAST_READ       0x0B
#define RDID            0x9F
#define WREN            0x06
#define WRDI            0x04
#define SE              0xD8
#define BE              0xC7
#define PP              0x02
#define RDSR            0x05
#define WRSR            0x01
#define DP              0xB9
#define RES             0xAB

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
* @name   : SPIM_TXEn
* @brief  : SPIM TX Enable
* @param  : SPI_TypeDef* SPIx
* @retval : void
*******************************************************************************/
void SPIM_TXEn(SPI_TypeDef* SPIx)
{

    SPI_BiDirectionalLineConfig(SPIx, SPI_Direction_Tx);
}

/*******************************************************************************
* @name   : SPIM_RXEn
* @brief  : SPIM RX Enable
* @param  : SPI_TypeDef* SPIx
* @retval : void
*******************************************************************************/
void SPIM_RXEn(SPI_TypeDef* SPIx)
{
    SPI_BiDirectionalLineConfig(SPIx, SPI_Direction_Rx);
}

/*******************************************************************************
* @name   : SPIM_CSLow
* @brief  : SPIM CS Low
* @param  : SPI_TypeDef* SPIx
* @retval : void
*******************************************************************************/
void SPIM_CSLow(SPI_TypeDef* SPIx)
{
#ifdef    USENORMALPINASCS
//    GPIO_ResetBits(GPIOA,GPIO_Pin_0);
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
#else
    SPI_CSInternalSelected(SPIx, SPI_CS_BIT0, ENABLE);
#endif
}

/*******************************************************************************
* @name   : SPIM_CSHigh
* @brief  : SPIM CS High
* @param  : SPI_TypeDef* SPIx
* @retval : void
*******************************************************************************/
void SPIM_CSHigh(SPI_TypeDef* SPIx)
{
#ifdef    USENORMALPINASCS
//	GPIO_SetBits(GPIOA,GPIO_Pin_0);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
#else
    SPI_CSInternalSelected(SPIx, SPI_CS_BIT0, DISABLE);
#endif
}


/********************************************************************************************************
**函数信息 SPI_DMA_RX_Init
**功能描述 : DMA接收数据初始化
**输入参数 ：无
**输出参数 ：无
**函数备注 ：需要将MISO&MOSI短接
********************************************************************************************************/
void SPI2_DMA_RX_Init()
{
    DMA_InitTypeDef  DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI2->RXREG); //SPI接收数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rxdata; //SPI接收到的数据存放基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//数据从外设到memory
    DMA_InitStructure.DMA_BufferSize = 20;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);//SPI2 RX为DMA通道4
}

/********************************************************************************************************
**函数信息 ：SPI_DMA_TX_Init
**功能描述 : SPI发送DMA初始化
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SPI2_DMA_TX_Init()
{
    DMA_InitTypeDef  DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI2->TXREG); //SPI发送数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)txdata;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//数据从memory到外设
    DMA_InitStructure.DMA_BufferSize = 8;//发送数据个数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不改变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//memory地址递增开启
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);//SPI RX为DMA通道5
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
#if 0
////////////////////////////////////////////////////////////////////////////////
/// @brief   DMA transmit and receive packet
/// @param   ptx_buf: Pointer to SPI DMA send buffer(include send and recv bytes).
/// @param   prx_buf: Pointer to SPI DMA recv buffer(include send and recv bytes).
/// @param   len: The length of data , length equal send + recv len).
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////********************************************************************************************************/
void SPI2MasterReadWritebuf(u8 *ptx_buf, u8 *prx_buf, u32 len)
{
    DRV_DMA_TransmitPacket(DMA1_Channel5, (u32)ptx_buf, len);
    DRV_DMA_TransmitPacket(DMA1_Channel4, (u32)prx_buf, len);
    SPIM_CSLow(SPI2);
    SPI_DMACmd(SPI2, SPI_DMAReq_EN, ENABLE);
    while(!DMA_GetFlagStatus(DMA1_FLAG_TC5));
    DMA_ClearFlag(DMA1_FLAG_TC5);
    while(!DMA_GetFlagStatus(DMA1_FLAG_TC4));
    DMA_ClearFlag(DMA1_FLAG_TC4);
    SPI_DMACmd(SPI2, SPI_DMAReq_EN, DISABLE);
    SPIM_CSHigh(SPI2);
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// @brief   DMA transmit and receive packet
/// @param   ptx_buf: Pointer to SPI DMA send buffer(include send and recv bytes).
/// @param   prx_buf: Pointer to SPI DMA recv buffer(include send and recv bytes).
/// @param   len: The length of data , length equal send + recv len).
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////********************************************************************************************************/
void SPI2MasterWritebuf(u8 *ptx_buf,  u32 len)
{
    u32 uByte[2];
    DMA_Set_MemoryInc_Enable(DMA1_Channel5, ENABLE);
    DRV_DMA_TransmitPacket(DMA1_Channel5, (u32)ptx_buf, len);
    DMA_Set_MemoryInc_Enable(DMA1_Channel4, DISABLE);
    DRV_DMA_TransmitPacket(DMA1_Channel4, (u32)uByte, len);
    SPI_DMACmd(SPI2, SPI_DMAReq_EN, ENABLE);
    while(!DMA_GetFlagStatus(DMA1_FLAG_TC5));
    DMA_ClearFlag(DMA1_FLAG_TC5);
    while(!DMA_GetFlagStatus(DMA1_FLAG_TC4));
    DMA_ClearFlag(DMA1_FLAG_TC4);
    SPI_DMACmd(SPI2, SPI_DMAReq_EN, DISABLE);

}
////////////////////////////////////////////////////////////////////////////////
/// @brief   DMA transmit and receive packet
/// @param   ptx_buf: Pointer to SPI DMA send buffer(include send and recv bytes).
/// @param   prx_buf: Pointer to SPI DMA recv buffer(include send and recv bytes).
/// @param   len: The length of data , length equal send + recv len).
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////********************************************************************************************************/
void SPI2MasterReadbuf( u8 *prx_buf, u32 len)
{
    u32 uByte[2];
    DMA_Set_MemoryInc_Enable(DMA1_Channel5, DISABLE);
    DRV_DMA_TransmitPacket(DMA1_Channel5, (u32)uByte, len);
    DMA_Set_MemoryInc_Enable(DMA1_Channel4, ENABLE);
    DRV_DMA_TransmitPacket(DMA1_Channel4, (u32)prx_buf, len);

    SPI_DMACmd(SPI2, SPI_DMAReq_EN, ENABLE);

    while(!DMA_GetFlagStatus(DMA1_FLAG_TC4));
    DMA_ClearFlag(DMA1_FLAG_TC4);
    SPI_DMACmd(SPI2, SPI_DMAReq_EN, DISABLE);

}



uint8_t spiReadBurst(uint8_t* pBuf, uint16_t len)
{
    if(len == 0)
        return 0;
    SPI2MasterReadbuf(pBuf, len);
    return len;

}

void spiWriteBurst(uint8_t* pBuf, uint16_t len)
{
    if(len == 0)
        return ;
    SPI2MasterWritebuf(pBuf, len);
    return ;
}

/********************************************************************************************************
**函数信息 ：SSPIM_checkStatus(SPI_TypeDef* SPIx)
**功能描述 :检查数据发送是否正确
**输入参数 ：SPI_TypeDef* SPIx,可选择SPI2,SPI2
**输出参数 ：无
********************************************************************************************************/
void SPI25xx_checkStatus(void)
{

    unsigned char temp[5];
    unsigned int i = 0;

    /*Spi cs assign to this pin,select*/
    SPIM_CSLow(SPI2);
    temp[i++] = RDSR;
    spiWriteBurst(temp, i);
    while(1) {
        spiReadBurst(&temp[i], 1);
        if(((temp[i]) & 0x01) == 0x0)
            break;
    }
    SPIM_CSHigh(SPI2);

}
/********************************************************************************************************
**函数信息 ：SPIM_WriteEnable(SPI_TypeDef* SPIx)
**功能描述 :写数据使能
**输入参数 ：SPI_TypeDef* SPIx,可选择SPI2,SPI2
**输出参数 ：无
********************************************************************************************************/
void SPI25xx_WriteEnable(void)
{
    unsigned char temp[5];
    unsigned int i = 0;
    temp[i++] = WREN;
    /*Spi cs assign to this pin,select*/
    SPIM_CSLow(SPI2);

    spiWriteBurst(temp, i);

    /*Spi cs release*/
    SPIM_CSHigh(SPI2);

}
/********************************************************************************************************
**函数信息 ：SPIM_ReadID(SPI_TypeDef* SPIx)
**功能描述 : 读取ID
**输入参数 ：SPI_TypeDef* SPIx,可选择SPI2,SPI2
**输出参数 ：无
********************************************************************************************************/
void SPI25xx_ReadID(void)
{
    unsigned char temp[5];
    unsigned int i = 0;
    temp[i++] = RDID;
    /*Spi cs assign to this pin,select*/
    SPIM_CSLow(SPI2);

    spiWriteBurst(temp, i);
    spiReadBurst(&temp[i], 3);

    /*Spi cs release*/
    SPIM_CSHigh(SPI2);
}


/********************************************************************************************************
**函数信息 ：SPIM_SectorErase(SPI_TypeDef* SPIx)
**功能描述 :擦除数据
**输入参数 ：SPI_TypeDef* SPIx,可选择SPI2,SPI2
**输入参数 ：unsigned long address,删除指定sector的地址 each sector = 64Kbytes
**输出参数 ：无
********************************************************************************************************/
void SPI25xx_SectorErase(unsigned long address)
{
    unsigned char temp[5];
    unsigned int i = 0;

    address = address & 0xffff0000;
    temp[i++] = SE;
    temp[i++] = ((unsigned char)(address >> 16)) & 0xff;
    temp[i++] = ((unsigned char)(address >> 8)) & 0xff;
    temp[i++] = ((unsigned char)address) & 0xff;
    SPI25xx_WriteEnable();
    /*Spi cs assign to this pin,select*/
    SPIM_CSLow(SPI2);
    spiWriteBurst(temp, i);
    /*Spi cs release*/
    SPIM_CSHigh(SPI2);

    SPI25xx_checkStatus();
}


/********************************************************************************************************
**函数信息 ：SPIM_PageRead(SPI_TypeDef* SPIx,unsigned long address,unsigned char *p,unsigned int number)
**功能描述 :读取256页数据
**输入参数 ：SPI_TypeDef* SPIx,可选择SPI2,SPI2  ;  address  ;*p ;number
**输出参数 ：无
********************************************************************************************************/
void SPI25xx_PageRead(unsigned long address, unsigned char *p, unsigned int number)
{


    unsigned char temp[5];
    unsigned int i = 0;

    address = address & 0xffff0000;
    temp[i++] = READ;
    temp[i++] = ((unsigned char)(address >> 16)) & 0xff;
    temp[i++] = ((unsigned char)(address >> 8)) & 0xff;
    temp[i++] = ((unsigned char)address) & 0xff;
    SPI25xx_checkStatus();
    /*Spi cs assign to this pin,select*/
    SPIM_CSLow(SPI2);
    spiWriteBurst(temp, i);
    spiReadBurst(p, number);

    /*Spi cs release*/
    SPIM_CSHigh(SPI2);
}

/********************************************************************************************************
**函数信息 ：SPIM_PageProgram(SPI_TypeDef* SPIx,unsigned long address,unsigned char *p,unsigned int number)
**功能描述 : 发送256页数据
**输入参数 ：SPI_TypeDef* SPIx,可选择SPI2,SPI2  ;  address;*p;number
**输出参数 ：无
********************************************************************************************************/
void SPI25xx_PageProgram(unsigned long address, unsigned char *p, unsigned int number)
{
    unsigned char temp[5];
    unsigned int i = 0;

    address = address & 0xffff0000;
    temp[i++] = PP;
    temp[i++] = ((unsigned char)(address >> 16)) & 0xff;
    temp[i++] = ((unsigned char)(address >> 8)) & 0xff;
    temp[i++] = ((unsigned char)address) & 0xff;
    SPI25xx_WriteEnable();
    /*Spi cs assign to this pin,select*/
    SPIM_CSLow(SPI2);
    spiWriteBurst(temp, i);
    spiWriteBurst(p, number);

    /*Spi cs release*/
    SPIM_CSHigh(SPI2);
    SPI25xx_checkStatus();
}

void SPI25xxPolling_Init(void)
{
    SPI2_DMA_RX_Init();
    SPI2_DMA_TX_Init();
    SPIM_Init(SPI2, 0x04);
}

/********************************************************************************************************
**函数信息 ：SPIM_Test(SPI_TypeDef* SPIx)
**功能描述 :测试程序，使用串口打印256页数据
**输入参数 ：SPI_TypeDef* SPIx,可选择SPI2,SPI2
**输出参数 ：无
********************************************************************************************************/
void SPI_TxRx_25xx_DMAPOLLing_Test(void)
{
    unsigned int i, result = 0;
    for(i = 0; i < 256; i++) {
        tmpdata[i] = i * 2;
    }
    {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 test\r\n"));
        SPI25xxPolling_Init();
        //while(1)
        {
            SPI25xx_ReadID();
        }
        //return;

        SPI25xx_SectorErase(0);

        SPI25xx_PageProgram(0, tmpdata, 256);


        memset(rxtmpdata, 0x0, 256);

        /*把写进去的一页256字节读出来*/
        SPI25xx_PageRead(0, rxtmpdata, 256);


        for(i = 0; i < 10; i++) {
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "rx[%d]=0x%x\r\n", i, rxtmpdata[i]));
        }
        for(i = 0; i < 256; i++) {
            if(tmpdata[i] != rxtmpdata[i]) {
                result = 1;
                break;
            }

        }
        if(result == 1) {
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 WR 25xx Fail\r\n"));
        } else {
            UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 WR 25xx Successful\r\n"));

        }
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 test over\r\n"));
    }

}
#if 0
/********************************************************************************************************
**函数信息 ：SPI_master_DMA_TXRX
**功能描述 : 通过DMA发送模式，完成自发自收
**输入参数 ：无
**输出参数 ：无
**函数备注 ：需要将MISO&MOSI短接
********************************************************************************************************/
void SPI_master_DMA_TXRXloop_Test(void)
{
    unsigned int i, j;



    SPIM_Init(SPI2, 2); //SPI CLK = 72/2 =36MHz
    SPI2_DMA_RX_Init();
    SPI2_DMA_TX_Init();


    UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 test\r\n"));
    //while(1)
    {
        for(i = 1; i <= LENGTH; i++) {
            for(j = 0; j < LENGTH; j++) {
                rxdata[j] = 0x0;
            }
            //memset(rxdata,0,sizeof(rxdata));
            SPI2MasterReadWritebuf(txdata, rxdata, i);

        }
    }
    for(i = 0; i < LENGTH; i++) {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "rx[%d]=0x%x\r\n", i, rxdata[i]));
    }
    j = 0;
    for(i = 0; i < LENGTH; i++) {
        if(rxdata[i] != txdata[i]) {
            j = 1;
            break;
        }

    }
    if(j == 1) {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 TxRx Loopback test fail\r\n"));
    } else {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 TxRx Loopback test Sucessful\r\n"));
    }
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "SPI2 test over\r\n"));

}
#endif
void setPA13asSWDIO(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
#ifdef DISSWDIO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_0);                       //SPI_MISO

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;                                 //SPI_MISO

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //Pull-up input
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
}

/*******************************************************************************
* @name   : main
* @brief  : Read/Write SPI FLASH by SPI interface
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
//    u32 i ;

    InitSystick();
    delay(1000);

    Uart_ConfigInit(9600);
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nsprintf ok\r\n"));
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nStart SPI test\r\n"));

//    SPI_master_DMA_TXRXloop_Test();
    SPI_TxRx_25xx_DMAPOLLing_Test();
    setPA13asSWDIO();
    while(1) {

    }
}


/*******************************************************************************
* @name   : SPI2GPIOPinConfig
* @brief  : SPI2GPIOPinConfig
* @param  : void
* @retval : void
*******************************************************************************/
void SPI2GPIOPinConfig(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
#ifdef USENORMALPINASCS

#else
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_3);                      //SPI_NSS  PA0
#endif

#ifdef DISSWDIO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_4);                       //SPI_MISO PA13
#else
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_4);                       //SPI_MISO PA12
#endif
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_4);                       //SPI_MOSI PB13
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_3);                       //SPI_SCK PB14
#ifdef USENORMALPINASCS
//    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;                                //SPI_NSS PA0
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;                                //SPI_NSS PA0
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                             //Push to avoid multiplexing output
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#else
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;                                //SPI_NSS PA0
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                             //Push to avoid multiplexing output
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13 | GPIO_Pin_14;                   //SPI_MOSI | SPI_SCK
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#ifdef DISSWDIO
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;                                 //SPI_MISO
#else
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;                                 //SPI_MISO
#endif
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //Pull-up input
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}


/*******************************************************************************
* @name   : SPIM_Init
* @brief  : SPIM Init
* @param  : SPI_TypeDef* SPIx,unsigned short spi_baud_div
* @retval : void
*******************************************************************************/
void SPIM_Init(SPI_TypeDef* SPIx, unsigned short spi_baud_div)
{
    SPI_InitTypeDef SPI_InitStructure;

    if(SPIx == SPI2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
        SPI2GPIOPinConfig();
    }

    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_DataWidth = SPI_DataWidth_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = spi_baud_div;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPIx, &SPI_InitStructure);
    SPIM_CSHigh(SPI2);
    SPI_Cmd(SPIx, ENABLE);
    SPIM_TXEn(SPIx);
    SPIM_RXEn(SPIx);
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_3); //RX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4); //TX

    UART_InitStructure.UART_BaudRate = bound;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;

    UART_Init(UART2, &UART_InitStructure);
    UART_Cmd(UART2, ENABLE);

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
    UART_SendData( UART2, dat);
    while(!UART_GetFlagStatus(UART2, UART_FLAG_TXEPT));
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