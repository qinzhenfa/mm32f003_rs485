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
void SystemInit (void);
#define SetPB0Low()    GPIO_ResetBits( GPIOB,  GPIO_Pin_0)
#define SetPB0High()    GPIO_SetBits( GPIOB,  GPIO_Pin_0)

void GPIO_ConfigInit(void);
void RCC_ConfigInit(void);

void Write_Iwdg_ON(u16 IWDG_Prescaler, u16 Reload);
void Write_Iwdg_RL(void);

static void delayus(u32 uldelay)
{
    u32 i, j;
    for(i = 0; i < uldelay; i++) {
        for(j = 0; j < 10000; j++) {

        }
    }
}

/*******************************************************************************
* @name   : main
* @brief  : IWDG reset
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    u32 ulA = 0;
    SystemInit();
    RCC_ConfigInit();
    GPIO_ConfigInit();

    while(1) {
        ulA++;
        delayus(500);
        SetPB0Low();
        delayus(500);
        SetPB0High();
        if(ulA == 20) {
            ulA = 0;
            break;
        }
    }

    Write_Iwdg_ON(IWDG_Prescaler_256, 0xff);
    while(1) {
#if 1
        Write_Iwdg_RL();                                                        //reload IWDG counter;
#endif
        ulA++;
        delayus(10);
        if(ulA == 1000) {
            SetPB0Low();

        } else if(ulA == 2000) {
            SetPB0High();
            ulA = 0;
        }


    }
}

/*******************************************************************************
* @name   : PVU_CheckStatus
* @brief  : Check Status
* @param  : None
* @retval : void
*******************************************************************************/
void PVU_CheckStatus(void)
{
    while(IWDG_GetFlagStatus(IWDG_FLAG_PVU));
}

/*******************************************************************************
* @name   : RVU_CheckStatus
* @brief  : Check Status
* @param  : None
* @retval : void
*******************************************************************************/
void RVU_CheckStatus(void)
{
    while(IWDG_GetFlagStatus(IWDG_FLAG_RVU));
}

/*******************************************************************************
* @name   : Write_Iwdg_ON
* @brief  : Write_Iwdg_ON
* @param  : None
* @retval : void
*******************************************************************************/
void Write_Iwdg_ON(u16 IWDG_Prescaler, u16 Reload)
{

    RCC_LSICmd(ENABLE);                                                         //enable LSI
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    PVU_CheckStatus();                                                          //get IWDG status
    IWDG_WriteAccessCmd(0x5555);
    IWDG_SetPrescaler(IWDG_Prescaler);

    RVU_CheckStatus();                                                          //get IWDG status
    IWDG_WriteAccessCmd(0x5555);
    IWDG_SetReload(Reload & 0xfff);

    IWDG_ReloadCounter();                                                       //load and enable IWDG
    IWDG_Enable();
}

/*******************************************************************************
* @name   : Write_Iwdg_RL
* @brief  : Write_Iwdg_RL
* @param  : None
* @retval : void
*******************************************************************************/
void Write_Iwdg_RL(void)
{
    IWDG_ReloadCounter();
}

/*******************************************************************************
* @name   : RCC_ConfigInit
* @brief  : RCC config
* @param  : None
* @retval : void
*******************************************************************************/
void RCC_ConfigInit(void)
{


    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);


}

/*******************************************************************************
* @name   : GPIO_ConfigInit
* @brief  : GPIO config
* @param  : None
* @retval : void
*******************************************************************************/
void GPIO_ConfigInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

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
