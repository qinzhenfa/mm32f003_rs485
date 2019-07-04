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
void KEY_Init(void);
void EXTIX_Init(void);



static __IO uint32_t TimingDelay;

#define LED1_Port  GPIOA
#define LED1_Pin   GPIO_Pin_4
#define LED1_ON()  GPIO_ResetBits(LED1_Port,LED1_Pin)
#define LED1_OFF()  GPIO_SetBits(LED1_Port,LED1_Pin)
#define LED1_TOGGLE()  (GPIO_ReadOutputDataBit(LED1_Port,LED1_Pin))?(GPIO_ResetBits(LED1_Port,LED1_Pin)):(GPIO_SetBits(LED1_Port,LED1_Pin))

#define KEY1_Port  GPIOA
#define KEY1_Pin   GPIO_Pin_5

#define KEY1  GPIO_ReadInputDataBit(KEY1_Port,KEY1_Pin)//读取按键1



/********************************************************************************************************
**函数信息 ：main(void)
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
int main(void)
{
    delay_init();
    LED_Init();
    KEY_Init();
    EXTIX_Init();
    while(1) {
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
    NVIC_SetPriority(SysTick_IRQn, 0x0);                                        //SysTick中断优先级设置
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
/********************************************************************************************************
**函数信息 ：LED_Init(void)
**功能描述 ：LED初始化
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void LED_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_Clock_Set(LED1_Port, ENABLE);  //开启GPIOA时钟

    GPIO_InitStructure.GPIO_Pin  =  LED1_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED1_Port, &GPIO_InitStructure);


    LED1_OFF();

}

/********************************************************************************************************
**函数信息 ：void KEY_Init(void)
**功能描述 ：按键初始化函数
**输入参数 ：
**输出参数 ：
**    备注 ：
********************************************************************************************************/
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_Clock_Set(KEY1_Port, ENABLE);  //开启KEY1 GPIO Port 时钟


    GPIO_InitStructure.GPIO_Pin  = KEY1_Pin;                                    // K1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;                       //设置成上拉输入
    GPIO_Init(KEY1_Port, &GPIO_InitStructure);                                  //初始化GPIOA5
    GPIO_SetBits(KEY1_Port, KEY1_Pin);
}

/********************************************************************************************************
**函数信息 ：void EXTIX_Init(void)
**功能描述 ：外部中断初始化函数
**输入参数 ：
**输出参数 ：
**常用函数 ：
********************************************************************************************************/
void EXTIX_Init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);                      //外部中断，需要使能AFIO时钟

    KEY_Init();                                                                 //初始化按键对应io模式


    //GPIOA.0	  中断线以及中断初始化配置
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;                      //下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);                                             //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;                          //使能按键所在的外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;                          //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);

}

/********************************************************************************************************
**函数信息 ：void EXTI0_1_IRQHandler(void)
**功能描述 ：外部中断函数
**输入参数 ：
**输出参数 ：
**常用函数 ：
********************************************************************************************************/
u16 EXTI_DELAY = 0;

void EXTI4_15_IRQHandler(void)
{
    for(EXTI_DELAY = 0; EXTI_DELAY < 1000; EXTI_DELAY++);
    if(KEY1 == 0) {
        LED1_TOGGLE();
    }
    EXTI_ClearFlag(EXTI_Line5);                                                 //清除EXTI5线路挂起位
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

