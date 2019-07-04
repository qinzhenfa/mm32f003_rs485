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
#include "string.h"
#include "HAL_device.h"
#include "HAL_conf.h"

#define REFVOLATGE 3.3 //5.0                                                         //Reference voltage, the unit is: V 


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

u16 ADCValue[20];                                                               //Used to store ADC conversion value
float ADCVolatge;                                                               //Used to store the voltage value
u16 ADCFilterValue;                                                             //Used to store the value of the ADC second-order filter
//u8 ADCflag;                                                                     //DMA transfer completed flag
u8 ADCFilterflag;                                                               //Filter completed sign

void GPIO_Config_AIN(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_n);
void ADCSingleChannelInit(ADCch ADC_Channel_x);
void DMAInit(void);
void ADCFilter(void);
void Get_ADCVolatge(void);

u32 adc_value = 0;
extern uint32_t SystemCoreClock;
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
    static u32 cnt;
    cnt++;
}

void ADCConfig(ADCch ADC_Channel)
{
    if( ADCch0 == ADC_Channel ) {
        GPIO_Config_AIN(GPIOA, GPIO_Pin_0);
        ADCSingleChannelInit(ADCch0);
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



/*******************************************************************************
* @name   : main
* @brief  :
* @param  :
* @retval :
*******************************************************************************/
int main(void)
{
    //	SystemInit();
    //	InitSystick();
    //Initialize DMA
    DMAInit();
    ADCConfig(ADCch0);  //PA0

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);                                      //Start Conversion

    while(1) {

        if(ADCFilterflag) {                                                 //Determine whether the second-order filter is over
            ADCFilterflag = 0;                                              //Clear the filter end flag
            Get_ADCVolatge();                                               //Convert the filtered value to voltage
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);                          //Start Conversion
        }

    }
}

/*******************************************************************************
* @name   : GPIO_Config_AIN
* @brief  : Initialize GPIO
* @param  : GPIO_TypeDef* GPIOx, u16 GPIO_Pin_n, GPIOMode_TypeDef GPIO_Mode (x can be A,B..D, n can be 0,1..15)
* @retval : void
*******************************************************************************/
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

/*******************************************************************************
* @name   : ADC1_SingleChannel
* @brief  : ADC initialization
* @param  : u8 ADC_Channel_x (x can be 0,1..8)
* @retval : void
*******************************************************************************/
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
    ADC_DMACmd(ADC1, ENABLE);                                                    //Enable ADCDMA
    ADC_Cmd(ADC1, ENABLE);                                                       //Enable AD conversion
}

/*******************************************************************************
* @name   : DMAInit
* @brief  : Initialize DMA
* @param  : void
* @retval : void
*******************************************************************************/
void DMAInit(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) & (ADC1->ADDATA);           //DMA transfer peripheral address
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADCValue;                      //DMA transfer memory address
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          //DMA transfer direction from peripheral to memory
    DMA_InitStructure.DMA_BufferSize = 20;                                       //DMA cache size
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //After receiving the data, the peripheral address is forbidden to move backward
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //After receiving the data, the memory address is shifted backward
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //Define the peripheral data width to 16 bits
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //Define the memory data width to 16 bits
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //Cycle conversion mode
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         //DMA priority is high
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //M2M mode is disabled
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);                              //DMA interrupt initialization
    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/*******************************************************************************
* @name   : ADCFilter
* @brief  : Second order filtering
* @param  : void
* @retval : void
*******************************************************************************/
void ADCFilter(void)                                                            //ADC second-order filter, when the filter is completed erection filter end marker
{
    static u16 cntFilter;
    static u32 lADCFilterValue = 0;
    for(cntFilter = 0; cntFilter < 20; cntFilter++) {
        lADCFilterValue += ADCValue[cntFilter];
    }
    cntFilter = 0;

    ADCFilterflag = 1;
    ADCFilterValue = (lADCFilterValue / 20 * 30 + ADCFilterValue * 70) / 100;
    lADCFilterValue = 0;
}

/*******************************************************************************
* @name   : Get_ADCVolatge
* @brief  : Convert the conversion data to voltage
* @param  : void
* @retval : void
*******************************************************************************/
void Get_ADCVolatge(void)
{
    ADCVolatge = ((float)ADCFilterValue / 4095) * REFVOLATGE;
}

/*******************************************************************************
* @name   : DMA1_Channel1_IRQHandler
* @brief  : When DMA send data, set ADCflag, PA8, clear the interrupt flag, stop the conversion
* @param  : void
* @retval : void
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);                                     //Stop Conversion
    DMA_ClearITPendingBit(DMA1_IT_TC1);                                          //Clear interrupt flag
    //ADCflag = 1;                                                                //Erected transmission complete flag
    ADCFilter();
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
