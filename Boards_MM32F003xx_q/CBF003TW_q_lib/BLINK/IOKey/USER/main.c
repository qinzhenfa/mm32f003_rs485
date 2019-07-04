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

#define KEY1  GPIO_ReadInputDataBit(KEY1_Port,KEY1_Pin)//��ȡ����1


#define KEY1_PRES	1		//KEY1 

__IO  u8 t = 0;
/********************************************************************************************************
**������Ϣ ��main(void)
**�������� ��
**������� ����
**������� ����
********************************************************************************************************/
int main(void)
{
    delay_init();
    LED_Init();
    KEY_Init();

    while(1) {
        t = KEY_Scan(0);                                                        //�õ���ֵ
        if(t == KEY1_PRES) {
            //���԰���K1�Ḵλ
            LED1_TOGGLE();
        } else {
            delay_ms(10);
        }
    }
}

/********************************************************************************************************
**������Ϣ ��delay_init(void)
**�������� ��systick��ʱ������ʼ��
**������� ����
**������� ����
********************************************************************************************************/
void delay_init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000)) {
        /* Capture error */
        while (1);
    }
    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);                                        //SysTick�ж����ȼ�����
}

/********************************************************************************************************
**������Ϣ ��SysTick_Handler(void)
**�������� ��������жϺ�����Systick���еݼ�
**������� ����
**������� ����
********************************************************************************************************/
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
}

/********************************************************************************************************
**������Ϣ ��TimingDelay_Decrement(void)
**�������� ����1ms���ٶȵݼ�
**������� ��pclk2������ϵͳʱ��Ϊ8MHz������Ӧ����8
**������� ����
********************************************************************************************************/
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    }
}

/********************************************************************************************************
**������Ϣ ��delay_ms(__IO uint32_t nTime)
**�������� ������Ӧ�õ�����ʱ��ʹ��systick
**������� ��nTime����ʱ
**������� ����
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
**������Ϣ ��LED_Init(void)
**�������� ��LED��ʼ��
**������� ����
**������� ����
********************************************************************************************************/
void LED_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_Clock_Set(LED1_Port, ENABLE);  //����GPIOAʱ��

    GPIO_InitStructure.GPIO_Pin  =  LED1_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED1_Port, &GPIO_InitStructure);


    LED1_OFF();

}

/********************************************************************************************************
**������Ϣ ��void KEY_Init(void)
**�������� ��������ʼ������
**������� ��
**������� ��
**    ��ע ��
********************************************************************************************************/
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_Clock_Set(KEY1_Port, ENABLE);  //����KEY1 GPIO Port ʱ��


    GPIO_InitStructure.GPIO_Pin  = KEY1_Pin;                                    // K1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                       //���ó���������
    GPIO_Init(KEY1_Port, &GPIO_InitStructure);                                  //��ʼ��GPIOA5

}
static u8 key_up = 1;
/********************************************************************************************************
**������Ϣ ��u8 KEY_Scan(u8 mode)
**�������� ������������
**������� ��mode  mode:0,��֧��������;1,֧��������;
**������� ��0,û�а�������;KEY1_PRES,KEY1����;WKUP_PRES,WK_UP����;KEY3_PRES,KEY3����;KEY4_PRES��KEY4����
**    ��ע ��
********************************************************************************************************/
u8 KEY_Scan(u8 mode)
{
    if(mode) {
        key_up = 1;                                                         //֧������
    }
    if(key_up && (KEY1 == 0)) {
        delay_ms(10);                                                           //ȥ����
        key_up = 0;
        if(KEY1 == 0) {
            return KEY1_PRES;
        }
    } else if(KEY1 == 1) {
        key_up = 1;
    }
    return 0;                                                                   //�ް�������
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

