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


static u32 cnt = 0;
char printBuf[100];
void Uart_ConfigInit(u32 bound);
void UartSendGroup(u8* buf, u16 len);
void UartSendAscii(char *str);

void RCC_ConfigInit(void);
void GPIO_ConfigInit(void);

#define SectorErase           0x20
#define PP                    0x02
#define ReadData              0x03
#define ChipErase             0xC7
#define RDSR                  0x05
#define Dummy_Byte            0x00
#define W25X_BUSY             0
#define W25X_NotBUSY          1
#define FlashSize             0x400
#define ReadStatusReg         0x05


#define READ        	0x03
#define FAST_READ   	0x0B
#define RDID        	0x9F
#define WREN 	        0x06
#define WRDI 	        0x04
#define SE              0xD8
#define BE              0xC7
#define PP              0x02
#define RDSR            0x05
#define WRSR            0x01
#define DP              0xB9
#define RES             0xAB


#define GPIO_Pin_0_2  GPIO_Pin_2

void SPIM_CSLow(SPI_TypeDef* SPIx);
void SPIM_CSHigh(SPI_TypeDef* SPIx);
unsigned int SPI_FLASH_WriteRead(SPI_TypeDef* SPIx, unsigned char tx_data);
void SPIM_Init(SPI_TypeDef* SPIx, unsigned short spi_baud_div);
void SPIM_RXEn(SPI_TypeDef* SPIx);
void SPIM_TXEn(SPI_TypeDef* SPIx);

void SPIM_ReadID(SPI_TypeDef* SPIx, u8 *buf);
void SPIM_WriteEnable(SPI_TypeDef* SPIx);
void SPIM_checkStatus(SPI_TypeDef* SPIx);
void SPIM_WriteDisable(SPI_TypeDef* SPIx);
void SPI_FLASH_Read(SPI_TypeDef* SPIx, u32 Addr, u32 NumBytes, void *data);
void SPI_FLASH_Write(SPI_TypeDef* SPIx, unsigned long address, unsigned char* data, u32 PageSize);
void SPI_SectorErase(SPI_TypeDef* SPIx, unsigned long address);
void SPI_ChipErase(SPI_TypeDef* SPIx);
u8 SPI_FLASH_WaitForWrite(SPI_TypeDef* SPIx);

unsigned char tmpdata[256];
unsigned char rxtmpdata[256];

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
* @name   : main
* @brief  : Read/Write SPI FLASH by SPI interface
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    u32 i ;

    InitSystick();
    delay(1000);
    RCC_ConfigInit();
    GPIO_ConfigInit();
    Uart_ConfigInit(9600);
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nsprintf ok\r\n"));
    SPIM_Init(SPI, 0x30);
    for(i = 0; i < 256; i++) {
        tmpdata[i] = i * 2;
    }

    SPIM_ReadID(SPI, rxtmpdata);
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nSPIM_ReadID:\r\n"));
    for(i = 0; i < 4; i++) {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "%2x ", rxtmpdata[i]));
    }

    SPI_SectorErase(SPI, 0);
    SPI_FLASH_Write(SPI, 0, tmpdata, 256);

    for(i = 0; i < 256; i++) {
        rxtmpdata[i] = 0x0;
    }

    SPI_FLASH_Read(SPI, 0, 256, rxtmpdata);
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "\r\nSPI_FLASH_Read:\r\n"));
    for(i = 0; i < 256; i++) {
        UartSendGroup((u8*)printBuf, sprintf(printBuf, "%2x ", rxtmpdata[i]));
    }
    while(1) {

    }
}

/*******************************************************************************
* @name   : SPIM_ReadID
* @brief  : Read ID
* @param  : SPIx,u8 *buf
* @retval : void
*******************************************************************************/
void SPIM_ReadID(SPI_TypeDef* SPIx, u8 *buf)
{
    SPIM_CSLow(SPIx);                                                           //Spi cs assign to this pin,select

    SPI_FLASH_WriteRead(SPIx, RDID);                                            //Send Read ID instruction
    buf[0] = SPI_FLASH_WriteRead(SPIx, Dummy_Byte);                             //Send the waste data to generate the clock
    buf[1] = SPI_FLASH_WriteRead(SPIx, Dummy_Byte);                             //Send the waste data to generate the clock
    buf[2] = SPI_FLASH_WriteRead(SPIx, Dummy_Byte);                             //Send the waste data to generate the clock

    SPIM_CSHigh(SPIx);                                                          //Spi cs release
}

/*******************************************************************************
* @name   : SPIM_WriteEnable
* @brief  : SPI Write Enable
* @param  : SPIx
* @retval : void
*******************************************************************************/
void SPIM_WriteEnable(SPI_TypeDef* SPIx)
{
    SPIM_CSLow(SPIx);
    SPI_FLASH_WriteRead(SPIx, WREN);
    SPIM_CSHigh(SPIx);                                                          //Spi cs release
}

/*******************************************************************************
* @name   : SPIM_checkStatus
* @brief  : check Status
* @param  : SPIx
* @retval : void
*******************************************************************************/
void SPIM_checkStatus(SPI_TypeDef* SPIx)
{
    unsigned char temp;
    SPIM_CSLow(SPIx);                                                           //Spi cs assign to this pin,select
    SPI_FLASH_WriteRead(SPIx, RDSR);
    while(1) {
        temp = SPI_FLASH_WriteRead(SPIx, Dummy_Byte);
        if((temp & 0x01) == 0x0)
            break;
    }
    SPIM_CSHigh(SPIx);                                                          //Spi cs release
}

/*******************************************************************************
* @name   : SPIM_WriteDisable
* @brief  : SPI Write Disable
* @param  : SPIx
* @retval : void
*******************************************************************************/
void SPIM_WriteDisable(SPI_TypeDef* SPIx)
{
    SPIM_CSLow(SPIx);
    SPI_FLASH_WriteRead(SPIx, WRDI);
    SPIM_CSHigh(SPIx);
}

/*******************************************************************************
* @name   : SPI_FLASH_Read
* @brief  : SPI Read The Flash Data
* @param  : SPI_TypeDef* SPIx, u32 Addr, u32 NumBytes, void *data
* @retval : void
*******************************************************************************/
void SPI_FLASH_Read(SPI_TypeDef* SPIx, u32 Addr, u32 NumBytes, void *data)
{
    u8 *Buffer = (u8 *) data;

    SPIM_CSLow(SPIx);


    SPI_FLASH_WriteRead(SPIx, ReadData);                                        //Send Read Data instruction
    SPI_FLASH_WriteRead(SPIx, Addr >> 16);                                      //Send destination address 16-23 bits
    SPI_FLASH_WriteRead(SPIx, Addr >> 8);                                       //Send destination address 8-15 bits
    SPI_FLASH_WriteRead(SPIx, Addr);                                            //Send destination address 0-7 bits

    while (NumBytes--) {
        *Buffer = SPI_FLASH_WriteRead(SPIx, Dummy_Byte);                            //Save the data read from flash into the specified array
        Buffer++;
    }


    SPIM_CSHigh(SPIx);

}

/*******************************************************************************
* @name   : SPI_FLASH_WaitForWrite
* @brief  : SPI FLASH Wait For Write
* @param  : SPI_TypeDef* SPIx
* @retval : u8
*******************************************************************************/
u8 SPI_FLASH_WaitForWrite(SPI_TypeDef* SPIx)
{
    unsigned char k;
    SPIM_CSLow(SPIx);

    SPI_FLASH_WriteRead(SPIx, ReadStatusReg);                                   //Read Status Register
    k = SPI_FLASH_WriteRead(SPIx, Dummy_Byte);

    SPIM_CSHigh(SPIx);
    if(k & 0x01)
        return (W25X_BUSY);
    else
        return (W25X_NotBUSY);
}

/*******************************************************************************
* @name   : SPI_FLASH_Write
* @brief  : SPI FLASH Write
* @param  : SPI_TypeDef* SPIx, unsigned long address, unsigned char* data, u32 PageSize
* @retval : void
*******************************************************************************/
void SPI_FLASH_Write(SPI_TypeDef* SPIx, unsigned long address, unsigned char* data, u32 PageSize)
{
    int i;
    unsigned char addr0, addr1, addr2;
    u8 *Buffer = data;
    u32 count = ((address / FlashSize) + 1) * FlashSize;


    while((FlashSize - address) < PageSize) {                                   //Judge flash this layer is enough space left

        SPI_SectorErase(SPIx, count);
        count = count + FlashSize;
        address = count;
    }

    while(1) {
        if(SPI_FLASH_WaitForWrite(SPIx) == W25X_NotBUSY) break;                     //Waiting for the flash is not busy state
    }

    address = address & 0xffffff00;
    addr0 = (unsigned char)(address >> 16);
    addr1 = (unsigned char)(address >> 8);
    addr2 = (unsigned char)address;


    SPIM_WriteEnable(SPIx);
    SPIM_CSLow(SPIx);

    SPI_FLASH_WriteRead(SPIx, PP);                                              //Send Page Program instruction
    SPI_FLASH_WriteRead(SPIx, addr0);                                           //Send destination address 16-23 bits
    SPI_FLASH_WriteRead(SPIx, addr1);                                           //Send destination address 8-15 bits
    SPI_FLASH_WriteRead(SPIx, addr2);                                           //Send destination address 0-7 bits

    for(i = 0; i < PageSize; i++) {
        SPI_FLASH_WriteRead(SPIx, *Buffer);
        Buffer++;
    }
    SPIM_CSHigh(SPIx);

    SPIM_checkStatus(SPIx);
}

/*******************************************************************************
* @name   : SPI_SectorErase
* @brief  : SPI Sector Erase
* @param  : SPI_TypeDef* SPIx, unsigned long address
* @retval : void
*******************************************************************************/
void SPI_SectorErase(SPI_TypeDef* SPIx, unsigned long address)
{
    unsigned char addr0, addr1, addr2;
    address = address & 0xffff0000;
    addr0 = ((unsigned char)(address >> 16)) & 0xff;
    addr1 = ((unsigned char)(address >> 8)) & 0xff;
    addr2 = ((unsigned char)address) & 0xff;

    SPIM_WriteEnable(SPIx);

    SPIM_CSLow(SPIx);

    SPI_FLASH_WriteRead(SPIx, SE);                                              //Send Sector Erase instruction
    SPI_FLASH_WriteRead(SPIx, addr0);                                           //Send destination address 16-23 bits
    SPI_FLASH_WriteRead(SPIx, addr1);                                           //Send destination address 8-15 bits
    SPI_FLASH_WriteRead(SPIx, addr2);                                           //Send destination address 0-7 bits
    SPIM_CSHigh(SPIx);                                                          //Spi cs release

    SPIM_checkStatus(SPIx);                                                     //Check flash current status
}

/*******************************************************************************
* @name   : SPI_ChipErase
* @brief  : SPI Chip Erase
* @param  : SPI_TypeDef* SPIx
* @retval : void
*******************************************************************************/
void SPI_ChipErase(SPI_TypeDef* SPIx)
{


    SPIM_WriteEnable(SPIx);

    SPIM_CSLow(SPIx);

    SPI_FLASH_WriteRead(SPIx, ChipErase);                                       //Send Chip Erase instruction
    SPI_FLASH_WriteRead(SPIx, Dummy_Byte);                                      //Send the waste data to generate the clock
    SPI_FLASH_WriteRead(SPIx, Dummy_Byte);                                      //Send the waste data to generate the clock
    SPI_FLASH_WriteRead(SPIx, Dummy_Byte);                                      //Send the waste data to generate the clock
    SPIM_CSHigh(SPIx);                                                          //Spi cs release

    SPIM_checkStatus(SPIx);
}

/*******************************************************************************
* @name   : SPIM_CSLow
* @brief  : SPIM CS Low
* @param  : SPI_TypeDef* SPIx
* @retval : void
*******************************************************************************/
void SPIM_CSLow(SPI_TypeDef* SPIx)
{

    SPI_CSInternalSelected(SPIx, SPI_CS_BIT0, ENABLE);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_0_2);
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
* @name   : SPIM_CSHigh
* @brief  : SPIM CS High
* @param  : SPI_TypeDef* SPIx
* @retval : void
*******************************************************************************/
void SPIM_CSHigh(SPI_TypeDef* SPIx)
{
    SPI_CSInternalSelected(SPIx, SPI_CS_BIT0, DISABLE);
//	GPIO_SetBits(GPIOA,GPIO_Pin_0_2);
}

/*******************************************************************************
* @name   : SPI_FLASH_WriteRead
* @brief  : SPI FLASH Write Read
* @param  : SPI_TypeDef* SPIx,unsigned char tx_data
* @retval : unsigned int
*******************************************************************************/
unsigned int SPI_FLASH_WriteRead(SPI_TypeDef* SPIx, unsigned char tx_data)
{

    SPI_SendData(SPIx, tx_data);                                                //Write data to TXREG
    while(!(SPIx->CSTAT & SPI_FLAG_TXEPT));
    while(!(SPIx->CSTAT & SPI_CSTAT_RXAVL));
    return (u32)SPIx->RXREG;

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
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_3);                      //SPI_NSS
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_4);                       //SPI_MISO
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_4);                       //SPI_MOSI
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_3);                       //SPI_SCK

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;                                //SPI_NSS
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                             //Push to avoid multiplexing output
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13 | GPIO_Pin_14;                   //SPI_MOSI | SPI_SCK
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;                                 //SPI_MISO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //Pull-up input
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_DataWidth = SPI_DataWidth_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = spi_baud_div;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPIx, &SPI_InitStructure);

    SPI_Cmd(SPIx, ENABLE);
    SPIM_TXEn(SPIx);
    SPIM_RXEn(SPIx);
}

/*******************************************************************************
* @name   : RCC_ConfigInit
* @brief  : RCC Config Init
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
* @brief  : GPIO Config Init
* @param  : None
* @retval : void
*******************************************************************************/
void GPIO_ConfigInit(void)
{
#if 0
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
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
