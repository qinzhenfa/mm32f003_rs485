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


typedef enum {
    ADCch0                 = ADC_Channel_0,
    ADCch4                 = ADC_Channel_4,
    ADCch5                 = ADC_Channel_5,
    ADCch6                 = ADC_Channel_6,
    ADCch8                 = ADC_Channel_8,
    ADCch9                 = ADC_Channel_9,
    ADCch10                = ADC_Channel_10,
    ADCch11                = ADC_Channel_11,
    ADCch12                = ADC_Channel_12,
    ADCchTemp              = ADC_Channel_TempSensor,
    ADCchVref              = ADC_Channel_Vrefint,
} ADCch;
void ADCConfig(ADCch ADC_Channel);
u16 Get_Adc_Average(uint8_t times);


extern u32 SystemCoreClock;

void delay_init(void);


void delay_ms(__IO uint32_t nTime);


void TimingDelay_Decrement(void);

static __IO uint32_t TimingDelay;

u16 ADCVAL;
float fValue;
/********************************************************************************************************
**������Ϣ ��main(void)
**�������� ��
**������� ��
**������� ��
**    ��ע ��
********************************************************************************************************/
int main(void)
{

    delay_init();

    ADCConfig(ADCch0); //use PA0
    while(1) {
        ADCVAL = Get_Adc_Average(5);
        fValue = ((float)ADCVAL / 4095) * 3.3; //use 3.3V as VDD
        //fValue = (((float)ADCVAL)/4095)*5; //use 5V as VDD

        delay_ms(200);
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
    NVIC_SetPriority(SysTick_IRQn, 0x0);//SysTick�ж����ȼ�����
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
**������Ϣ ��GPIO_Config_AIN(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_n)
**�������� ������ADC1����ת��ģʽ��ӦPIN
**������� ��GPIOx GPIO_Pin_n
**������� ����
**    ��ע ��
********************************************************************************************************/
void GPIO_Config_AIN(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_n)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    if(GPIOx == GPIOA) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);                         //GPIO clock starts
    }
    if(GPIOx == GPIOB) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);                         //GPIO clock starts
    }
    if(GPIOx == GPIOC) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);                         //GPIO clock starts
    }
    if(GPIOx == GPIOD) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);                         //GPIO clock starts
    }
    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_n;                                 //pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                           //Output speed
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                               //GPIO mode
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/********************************************************************************************************
**������Ϣ ��void ADCSingleChannelInit(ADCch ADC_Channel_x)
**�������� ������ADC1����ת��ģʽ
**������� ��ADCch ADC_Channel
**������� ����
**    ��ע ��
********************************************************************************************************/
void ADCSingleChannelInit(ADCch ADC_Channel_x)
{
    ADC_InitTypeDef  ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);

    //Initialize PA1 to analog input mode
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);                         //Enable ADC clock

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_PRESCARE = ADC_PCLK2_PRESCARE_16;                     //ADC prescale factor
    ADC_InitStructure.ADC_Mode = ADC_Mode_Continuous_Scan;                      //Set ADC mode to continuous conversion mode
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      //AD data right-justified
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //	ADC_RegularChannelConfig(ADC, ADC_Channel_All_Disable, 0, ADC_SampleTime_13_5Cycles);//Block all channels
    ADC_RegularChannelConfig(ADC1, ADC_Channel_x, 0, ADC_SampleTime_239_5Cycles);//Enable the channel


    if(ADC_Channel_x == ADC_Channel_TempSensor) {
        ADC_TempSensorCmd(ENABLE);                                       //Enable internal temperature sensor
    }
    if(ADC_Channel_x == ADC_Channel_Vrefint) {
        ADC_VrefintCmd(ENABLE);                                       //Enable internal temperature sensor
    }
    //Enable ADCDMA
    ADC_Cmd(ADC1, ENABLE);                                                       //Enable AD conversion
}

/********************************************************************************************************
**������Ϣ ��void ADCConfig(uint8_t ADC_Channel)
**�������� �����ò���GPIO��ADC1����ת��ģʽ
**������� ADC_Channel
**������� ����
**    ��ע ��
********************************************************************************************************/
void ADCConfig(ADCch ADC_Channel)
{
    if( ADCch0 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOA, GPIO_Pin_0);
    } else if( ADCch4 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOA, GPIO_Pin_4);
    } else if( ADCch5 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOA, GPIO_Pin_5);
    } else if( ADCch6 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOA, GPIO_Pin_6);
    } else if( ADCch8 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOB, GPIO_Pin_0);
    } else if( ADCch9 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOB, GPIO_Pin_1);
    } else if( ADCch10 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOB, GPIO_Pin_3);
    } else if( ADCch11 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOB, GPIO_Pin_4);
    } else if( ADCch12 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOB, GPIO_Pin_7);
    } else {
    }
    ADCSingleChannelInit(ADC_Channel);
}



/********************************************************************************************************
**������Ϣ ��ADC1_SingleChannel_Get()
**�������� ����ȡADC1ת������
**������� ��ADC_Channel_x , xΪ0~8
*puiADData ,ADC1ʵ��ת������
**������� ��ucStatus ,0 ��ʾ���ݻ�ȡʧ��,1 ��ʾ�ɹ�
********************************************************************************************************/
u16 ADC1_SingleChannel_Get(void)
{
    u16 puiADData;
    /*ADCR�Ĵ�����ADSTλʹ�ܣ��������ת��*/
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    puiADData = ADC_GetConversionValue(ADC1);
    return puiADData;
}

/********************************************************************************************************
**������Ϣ ��Get_Adc_Average(uint8_t times)
**�������� ����ȡ����ADC1����ֵ��ƽ��ֵ
**������� ��ADC_Channel_x , xΪ0~8
**������� ��puiADDataΪADC������ֵ
********************************************************************************************************/
u16 Get_Adc_Average(uint8_t times)
{
    u32 temp_val = 0;
    u8 t;
    u8 delay;
    for(t = 0; t < times; t++) {
        temp_val += ADC1_SingleChannel_Get();
        for(delay = 0; delay < 100; delay++);
    }
    return temp_val / times;
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

