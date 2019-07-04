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

void OnTIM1(void);
void initNVIC_TIM1(void);
void initTIM1(u16 prescaler, u16 period, u16 dutyCycle);
void initGPIO_Timer1(void);

u32 SysTick_Count;

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
}

/*******************************************************************************
* @name   : main
* @brief  : TIM PWM output
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    initGPIO_Timer1();
    initTIM1(47, 999, 500);                                                     //config TIM to PWM output
    initNVIC_TIM1();
    OnTIM1();                                                                   //enable TIM;

    while(1) {
    }


}

/*******************************************************************************
* @name   : initGPIO_Timer1
* @brief  : Initialize TIM1 GPIO
* @param  : None
* @retval : void
*******************************************************************************/
void initGPIO_Timer1(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_7);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_2);

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_14;                                //TIM1_CH1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_13;                                //TIM1_CH1N
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* @name   : initTIM1
* @brief  : Initialize TIM1
* @param  : u16 prescaler, u16 period, u16 dutyCycle
* @retval : void
*******************************************************************************/
void initTIM1(u16 prescaler, u16 period, u16 dutyCycle)
{
    TIM_TimeBaseInitTypeDef TIM_StructInit;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseStructInit(&TIM_StructInit);
    TIM_StructInit.TIM_Period = period;
    TIM_StructInit.TIM_Prescaler = prescaler;
    TIM_StructInit.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_StructInit.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_StructInit.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_StructInit);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = dutyCycle;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/*******************************************************************************
* @name   : initNVIC_TIM1
* @brief  : Initialize TIM1 NVIC
* @param  : None
* @retval : void
*******************************************************************************/
void initNVIC_TIM1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;                          // Config TIM1_CC_IRQn NVIC config
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;                             // PreemptionPriority : 1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             // Enable IRQn
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* @name   : TIM1_CC_IRQHandler
* @brief  : TIM1 interrupt function
* @param  : None
* @retval : void
*******************************************************************************/
void TIM1_CC_IRQHandler()
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);                                // Clear the CC1 flag
    }
}

/*******************************************************************************
* @name   : OnTIM1
* @brief  : Enable TIM1
* @param  : None
* @retval : void
*******************************************************************************/
void OnTIM1(void)
{
    TIM_Cmd(TIM1, ENABLE);
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
