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
u8 KEY_Scan(u8 mode);

static __IO uint32_t TimingDelay;

#define LED1_Port  GPIOA
#define LED1_Pin   GPIO_Pin_4
#define LED1_ON()  GPIO_ResetBits(LED1_Port,LED1_Pin)
#define LED1_OFF()  GPIO_SetBits(LED1_Port,LED1_Pin)
#define LED1_TOGGLE()  (GPIO_ReadOutputDataBit(LED1_Port,LED1_Pin))?(GPIO_ResetBits(LED1_Port,LED1_Pin)):(GPIO_SetBits(LED1_Port,LED1_Pin))

#define KEY1_Port  GPIOA
#define KEY1_Pin   GPIO_Pin_5

#define KEY1  GPIO_ReadInputDataBit(KEY1_Port,KEY1_Pin)//读取按键1


#define KEY1_PRES	1		//KEY1 

__IO  u8 t = 0;
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

    while(1) {
        t = KEY_Scan(0);                                                        //得到键值
        if(t == KEY1_PRES) {
            //所以按下K1会复位
            LED1_TOGGLE();
        } else {
            delay_ms(10);
        }
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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                       //设置成上拉输入
    GPIO_Init(KEY1_Port, &GPIO_InitStructure);                                  //初始化GPIOA5

}
static u8 key_up = 1;
/********************************************************************************************************
**函数信息 ：u8 KEY_Scan(u8 mode)
**功能描述 ：按键处理函数
**输入参数 ：mode  mode:0,不支持连续按;1,支持连续按;
**输出参数 ：0,没有按键按下;KEY1_PRES,KEY1按下;WKUP_PRES,WK_UP按下;KEY3_PRES,KEY3按下;KEY4_PRES，KEY4按下
**    备注 ：
********************************************************************************************************/
u8 KEY_Scan(u8 mode)
{
    if(mode) {
        key_up = 1;                                                         //支持连按
    }
    if(key_up && (KEY1 == 0)) {
        delay_ms(10);                                                           //去抖动
        key_up = 0;
        if(KEY1 == 0) {
            return KEY1_PRES;
        }
    } else if(KEY1 == 1) {
        key_up = 1;
    }
    return 0;                                                                   //无按键按下
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

