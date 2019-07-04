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
#include "HAL_conf.h"
#include "stdio.h"

extern u32 SystemCoreClock;
void delay_init(void);

void delay_ms(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
void LED_Init(void);
void DMA_m8tom8_test(void);
void DMA_m8tom16_test(void);
void DMA_m8tom32_test(void);
void DMA_m16tom8_test(void);
void DMA_m16tom16_test(void);
void DMA_m16tom32_test(void);
void DMA_m32tom8_test(void);
void DMA_m32tom16_test(void);
void DMA_m32tom32_test(void);

unsigned char dma1Flag = 0x0;
unsigned char dma2Flag = 0x0;
u8  dmaTxDATA[64 * 4] ;
u8  dmaRxDATA[64 * 4] ;


static __IO uint32_t TimingDelay;

#define LED4_ON()  GPIO_ResetBits(GPIOA,GPIO_Pin_15)	// PA15
#define LED4_OFF()  GPIO_SetBits(GPIOA,GPIO_Pin_15)	// PA15
#define LED4_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_15))?(GPIO_ResetBits(GPIOA,GPIO_Pin_15)):(GPIO_SetBits(GPIOA,GPIO_Pin_15)) // PA15

#define LED3_ON()  GPIO_ResetBits(GPIOB,GPIO_Pin_3)	// PB3
#define LED3_OFF()  GPIO_SetBits(GPIOB,GPIO_Pin_3)	// PB3
#define LED3_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3))?(GPIO_ResetBits(GPIOB,GPIO_Pin_3)):(GPIO_SetBits(GPIOB,GPIO_Pin_3))	// PB3

#define LED2_ON()  GPIO_ResetBits(GPIOB,GPIO_Pin_4)	// PB4
#define LED2_OFF()  GPIO_SetBits(GPIOB,GPIO_Pin_4)	// PB4
#define LED2_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_4))?(GPIO_ResetBits(GPIOB,GPIO_Pin_4)):(GPIO_SetBits(GPIOB,GPIO_Pin_4))	// PB4

#define LED1_ON()  GPIO_ResetBits(GPIOB,GPIO_Pin_5)	// PB5
#define LED1_OFF()  GPIO_SetBits(GPIOB,GPIO_Pin_5)	// PB5
#define LED1_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_5))?(GPIO_ResetBits(GPIOB,GPIO_Pin_5)):(GPIO_SetBits(GPIOB,GPIO_Pin_5))	// PB5


void (*pfun[])(void) = {
    DMA_m8tom8_test,
    DMA_m8tom16_test,
    DMA_m8tom32_test,
    DMA_m16tom8_test,
    DMA_m16tom16_test,
    DMA_m16tom32_test,
    DMA_m32tom8_test,
    DMA_m32tom16_test,
    DMA_m32tom32_test
};


int functiontest(void)
{
    u16 i;
    int FunNumber;

    FunNumber = sizeof(pfun) / 4;
    for(i = 0; i < FunNumber; i++) {
        pfun[i]();
    }

    return 0;
}
/********************************************************************************************************
**函数信息 ：int main (void)
**功能描述 ：开机后，ARMLED闪动
**输入参数 ：
**输出参数 ：
********************************************************************************************************/
int main(void)
{

    delay_init();

    LED_Init();
    functiontest();
    while(1) {
        LED1_TOGGLE();
        LED2_TOGGLE();
        LED3_TOGGLE();
        LED4_TOGGLE();
        delay_ms(1000);
    }
}

/********************************************************************************************************
**函数信息 ：delay_init(void)
**功能描述 ：systick延时函数初始化
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void delay_init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000)) {
        /* Capture error */
        while (1);
    }
    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/********************************************************************************************************
**函数信息 ：SysTick_Handler(void)
**功能描述 ：进入该中断函数后，Systick进行递减
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
}

/********************************************************************************************************
**函数信息 ：TimingDelay_Decrement(void)
**功能描述 ：以1ms的速度递减
**输入参数 ：pclk2，例如系统时钟为8MHz，参数应传入8
**输出参数 ：无
********************************************************************************************************/
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    }
}

/********************************************************************************************************
**函数信息 ：delay_ms(__IO uint32_t nTime)
**功能描述 ：程序应用调用延时，使用systick
**输入参数 ：nTime：延时
**输出参数 ：无
********************************************************************************************************/
void delay_ms(__IO uint32_t nTime)
{
    TimingDelay = nTime;

    while(TimingDelay != 0);
}

/********************************************************************************************************
**函数信息 ：LED_Init(void)
**功能描述 ：LED初始化
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void LED_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);   //开启GPIOA,GPIOB时钟

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    LED1_ON();
    LED2_ON();
    LED3_ON();
    LED4_ON();
}

/********************************************************************************************************
**函数信息 ：DMA1_Channel1_IRQHandler(void)
**功能描述 : DMA1通道1的中断函数
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC1)) {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        dma1Flag = 0x1;
    }
}

/********************************************************************************************************
**函数信息 ：DMA1_Channel2_IRQHandler(void)
**功能描述 : DMA1通道2的中断函数
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA1_Channel2_3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC2)) {
        DMA_ClearITPendingBit(DMA1_IT_TC2);
        dma2Flag = 0x1;
    }

}

/********************************************************************************************************
**函数信息 ：DMAcheckStatus(uint32_t DMA_FLAG)
**功能描述 : 查询DMA的标志位
**输入参数 ：uint32_t DMA_FLAG，DMA的状态标志位
**输出参数 ：无
********************************************************************************************************/
void DMAcheckStatus(uint32_t DMA_FLAG)
{
    while(1) {
        if(DMA_GetFlagStatus(DMA_FLAG)) {
            DMA_ClearFlag(DMA_FLAG);
            break;
        }
    }
}

/********************************************************************************************************
**函数信息 ：DMAdisable(DMA_Channel_TypeDef* DMAy_Channelx)
**功能描述 : DMA通道失能
**输入参数 ：DMA_Channel_TypeDef* DMAy_Channelx，选择通道
**输出参数 ：无
********************************************************************************************************/
void DMAdisable(DMA_Channel_TypeDef* DMAy_Channelx)
{
    DMA_Cmd(DMAy_Channelx, DISABLE);
}

/********************************************************************************************************
**函数信息 ：DMA_m8tom8_test(void)
**功能描述 : 把存储器中的某一地址开始的64个8位数据搬到存储器的另一个地址，并以8位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m8tom8_test(void)
{
    unsigned int i;
    u8 *p;
    u8 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    p = (u8*)dmaTxDATA;
    q = (u8*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel2);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);


    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

    dma2Flag = 0x0;

    DMA_Cmd(DMA1_Channel2, ENABLE);

    while(1) {
        if(dma2Flag) {
            dma2Flag = 0x0;
            break;
        }
    }


    DMA_Cmd(DMA1_Channel2, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if((u8)(*p++) == (u8)(*q++)) {
            temp++;
        }
    }
}

/********************************************************************************************************
**函数信息 ：DMA_m8tom16_test(void)
**功能描述 : 把存储器中的某一地址开始的64个8位数据搬到存储器的另一个地址，并以16位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m8tom16_test(void)
{
    unsigned int i;
    u8 *p;
    u16 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    p = (u8*)dmaTxDATA;
    q = (u16*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel1);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    dma1Flag = 0x0;

    DMA_Cmd(DMA1_Channel1, ENABLE);

    while(1) {
        if(dma1Flag) {
            dma1Flag = 0x0;
            break;
        }
    }

    DMA_Cmd(DMA1_Channel1, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if(((u16)(*p++)) == (u16)(*q++)) {
            temp++;
        }
    }
}

/********************************************************************************************************
**函数信息 ：DMA_m8tom32_test(void)
**功能描述 : 把存储器中的某一地址开始的64个8位数据搬到存储器的另一个地址，并以32位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m8tom32_test()
{
    unsigned int i;
    u8 *p;
    u32 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;

    p = (u8*)dmaTxDATA;
    q = (u32*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel3);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel3, ENABLE);

    DMAcheckStatus(DMA1_FLAG_TC3);

    DMA_Cmd(DMA1_Channel3, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if(((u32)(*p++)) == (u32)(*q++)) {
            temp++;
        }
    }
}

/********************************************************************************************************
**函数信息 ：DMA_m16tom8_test(void)
**功能描述 : 把存储器中的某一地址开始的64个16位数据搬到存储器的另一个地址，并以8位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m16tom8_test(void)
{
    unsigned int i;
    u16 *p;
    u8 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;

    p = (u16*)dmaTxDATA;
    q = (u8*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel4);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);


    DMA_Cmd(DMA1_Channel4, ENABLE);

    DMAcheckStatus(DMA1_FLAG_TC4);

    DMA_Cmd(DMA1_Channel4, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if(((u8)(*p++)) == ((u8)(*q++))) {
            temp++;
        }
    }
}

/********************************************************************************************************
**函数信息 ：DMA_m16tom16_test(void)
**功能描述 : 把存储器中的某一地址开始的64个16位数据搬到存储器的另一个地址，并以16位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m16tom16_test(void)
{
    unsigned int i;
    u16 *p;
    u16 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;

    p = (u16*)dmaTxDATA;
    q = (u16*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel5);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);


    DMA_Cmd(DMA1_Channel5, ENABLE);

    DMAcheckStatus(DMA1_FLAG_TC5);

    DMA_Cmd(DMA1_Channel5, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if(((u16)(*p++)) == ((u16)(*q++))) {
            temp++;
        }
    }

}

/********************************************************************************************************
**函数信息 ：DMA_m16tom32_test(void)
**功能描述 : 把存储器中的某一地址开始的64个16位数据搬到存储器的另一个地址，并以32位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m16tom32_test(void)
{
    unsigned int i;
    u16 *p;
    u32 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;

    p = (u16*)dmaTxDATA;
    q = (u32*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel5);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);

    DMAcheckStatus(DMA1_FLAG_TC5);

    DMA_Cmd(DMA1_Channel5, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if(((u32)(*p++)) == ((u32)(*q++))) {
            temp++;
        }
    }
}


/********************************************************************************************************
**函数信息 ：DMA_m32tom8_test(void)
**功能描述 : 把存储器中的某一地址开始的64个32位数据搬到存储器的另一个地址，并以8位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m32tom8_test(void)
{
    unsigned int i;
    u32 *p;
    u8 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;

    p = (u32*)dmaTxDATA;
    q = (u8*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel5);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);

    DMAcheckStatus(DMA1_FLAG_TC5);

    DMA_Cmd(DMA1_Channel5, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if(((u8)(*p++)) == ((u8)(*q++))) {
            temp++;
        }
    }
}

/********************************************************************************************************
**函数信息 ：DMA_m32tom16_test(void)
**功能描述 : 把存储器中的某一地址开始的64个32位数据搬到存储器的另一个地址，并以16位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m32tom16_test(void)
{
    unsigned int i;
    u32 *p;
    u16 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;

    p = (u32*)dmaTxDATA;
    q = (u16*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel5);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);

    DMAcheckStatus(DMA1_FLAG_TC5);

    DMA_Cmd(DMA1_Channel5, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if(((u16)(*p++)) == ((u16)(*q++))) {
            temp++;
        }
    }
}

/********************************************************************************************************
**函数信息 ：DMA_m32tom32_test(void)
**功能描述 : 把存储器中的某一地址开始的64个32位数据搬到存储器的另一个地址，并以32位方式存储
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void DMA_m32tom32_test(void)
{
    unsigned int i;
    u32 *p;
    u32 *q;
    unsigned int temp;
    DMA_InitTypeDef DMA_InitStructure;

    p = (u32*)dmaTxDATA;
    q = (u32*)dmaRxDATA;
    for(i = 0; i < 64; i++) {
        *(p + i) = i + 1;
    }

    DMA_DeInit(DMA1_Channel5);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)dmaRxDATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dmaTxDATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 64;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);

    DMAcheckStatus(DMA1_FLAG_TC5);

    DMA_Cmd(DMA1_Channel5, DISABLE);
    temp = 0;
    for(i = 0; i < 64; i++) {
        if(((u32)(*p++)) == ((u32)(*q++))) {
            temp++;
        }
    }
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

