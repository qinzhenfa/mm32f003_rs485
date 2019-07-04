/**
******************************************************************************
* @file     main.c
* @author   AE Team
* @version  V1.0.8
* @date     10/04/2019
* @brief    Use simulate IIC interface write and read data
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



//------------------------------------------------------------------------------
extern u32 SystemCoreClock;
void delay_init(void);
void delay_ms(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
void LED_Init(void);

static __IO uint32_t TimingDelay;

#define LED1_Port      GPIOA
#define LED1_Pin       GPIO_Pin_4
#define LED1_ON()      GPIO_ResetBits(LED1_Port,LED1_Pin)
#define LED1_OFF()     GPIO_SetBits(LED1_Port,LED1_Pin)
#define LED1_TOGGLE()  (GPIO_ReadOutputDataBit(LED1_Port,LED1_Pin))?    \
    (GPIO_ResetBits(LED1_Port,LED1_Pin)):            \
    (GPIO_SetBits(LED1_Port,LED1_Pin))



void I2CWrite(u16 addr, u8* ptr, u16 cnt);
void I2CRead(u16 addr, u8* ptr, u16 cnt);
void I2C_Initialize(void);

//------------ Write/Read 24C02-Flash ------------------------------------------
//#define PAGESIZE 16                                                             //The size of each EEPROM page
#define EEPROM_ADDR 0xA0

void I2C_GPIO_Config(void);
bool EEPROM_ByteWrite(u8 SendByte, u16 WriteAddress, u8 DeviceAddress);
bool EEPROM_SequentialRead(u8 *pBuffer, u8 length, u16 ReadAddress, u8 DeviceAddress);
void EEPROM_BufferWrite(u8 *pBuffer, u16 length, u16 WriteAddress, u8 DeviceAddress );
void I2C_Initialize(void);
void I2CEEPROMWrite(u16 addr, u8 *ptr, u16 cnt);
void I2CEEPROMRead(u16 addr, u8* ptr, u16 cnt);

u8 buffer0[128] = {0xab, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};
u8 buffer1[128];

/*
This sample show how to use MM32 write read EEPROM AT2404 through
GPIO pin to simulate I2C interface
�������MM32ͨ��GPIOģ��I2C��дEEPROM����
*/
#define I2C1_SCL_PORT               GPIOB
#define I2C1_SCL_PIN                GPIO_Pin_13
#define I2C1_SCL_BUSCLK             RCC_AHBPeriph_GPIOB

#define I2C1_SDA_PORT               GPIOB
#define I2C1_SDA_PIN                GPIO_Pin_14
#define I2C1_SDA_BUSCLK             RCC_AHBPeriph_GPIOB

#define SCL_H                       I2C1_SCL_PORT->BSRR = I2C1_SCL_PIN
#define SCL_L                       I2C1_SCL_PORT->BRR  = I2C1_SCL_PIN
#define SCL_read                    (I2C1_SCL_PORT-IDR  & I2C1_SCL_PIN)

#define	SDA_H	                    I2C1_SDA_PORT->BSRR	= I2C1_SDA_PIN
#define SDA_L	                    I2C1_SDA_PORT->BRR	= I2C1_SDA_PIN
#define SDA_read                    (I2C1_SDA_PORT->IDR & I2C1_SDA_PIN)

void I2C_GPIO_Config(void);
bool EEPROM_ByteWrite(u8 SendByte, u16 WriteAddress, u8 DeviceAddress);
bool EEPROM_SequentialRead(u8 *pBuffer, u8 length, u16 ReadAddress, u8 DeviceAddress);
#define I2C_PageSize 16//8
void delay_ms(__IO uint32_t nTime);
#define Systick_Delay_1ms delay_ms
void I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /*Configure I2C1 Pins:SCL and SDA*/
    RCC_AHBPeriphClockCmd(I2C1_SCL_BUSCLK, ENABLE);
    RCC_AHBPeriphClockCmd(I2C1_SDA_BUSCLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = I2C1_SDA_PIN;
    GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStructure);
}

static void I2C_delay(void)
{
    //set I2C SCL speed ����ͨ���ٶ�
    u8 i = 13; //100;

    while(i) {
        i--;
    }
}

bool I2C_Start(void)
{
    I2C_delay();
    SDA_H;
    SCL_H;
    I2C_delay();
    if(!SDA_read) return FALSE;  //SDA��Ϊ�͵�ƽ����æ���˳�

    SDA_L;
    I2C_delay();
    if(SDA_read) return FALSE; //SDA��Ϊ�ߵ�ƽ�����߳����˳�

    SDA_L;
    I2C_delay();
    return TRUE;
}

void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

bool I2C_WaitAck(void)  //����Ϊ��-1��ACK�� =0 ��ACK
{
    bool bstatus;
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if(SDA_read) {
        bstatus = FALSE;
    } else {
        bstatus = TRUE;
    }
    SCL_L;
    return bstatus;

}

void I2C_SendByte(u8 SendByte) //���ݴӸ�λ����λ
{
    u8 i = 8;
    while(i--) {
        SCL_L;
        I2C_delay();
        if(SendByte & 0x80)
            SDA_H;
        else
            SDA_L;
        SendByte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();

    }
    SCL_L;
}
u8 I2C_ReceiveByte(void) //���ݴӸ�λ����λ
{
    u8 i = 8;
    u8 ReceiveByte = 0;

    SDA_H;
    while(i--) {
        ReceiveByte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if(SDA_read) {
            ReceiveByte |= 0x01;
        }
    }
    SCL_L;
    return ReceiveByte;
}
//д��1�ֽ����� ����д�����ݣ���д���ַ���������ͣ�
bool EEPROM_ByteWrite(u8 SendByte, u16 WriteAddress, u8 DeviceAddress)
{
    if(!I2C_Start()) return FALSE;

    I2C_SendByte(((WriteAddress & 0x0700) >> 7) | (DeviceAddress & 0xFE));	//���ø���ʼ��ַ + ������ַ

    if(!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte((u8)(WriteAddress & 0x00FF)); //���õ���ʼ��ַ
    I2C_WaitAck();
    I2C_SendByte(SendByte);
    I2C_WaitAck();
    I2C_Stop();
    //ע�⣺��Ϊ����Ҫ�ȴ�EERPOMд��ɣ����Բ��ò�ѯ ����ʱ��ʽ��10ms��
    Systick_Delay_1ms(10);
    return TRUE;
}

//ע�ⲻ�ܿ�ҳд
//д��1�����ݣ���д�������ַ����д�볤�ȣ���д���ַ���������ͣ�
bool EEPROM_PageWrite(u8 *pBuffer, u8 length, u16 WriteAddress, u8 DeviceAddress)
{
    if((length + WriteAddress % I2C_PageSize) > I2C_PageSize) return FALSE;
    if(!I2C_Start()) return FALSE;
    I2C_SendByte(((WriteAddress & 0x0700) >> 7) | (DeviceAddress & 0xFE));	//���ø���ʼ��ַ + ������ַ

    if(!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte((u8)(WriteAddress & 0x00FF)); //���õ���ʼ��ַ
    I2C_WaitAck();

    while(length--) {
        I2C_SendByte(*pBuffer);
        I2C_WaitAck();
        pBuffer++;
    }
    I2C_Stop();
    //ע�⣺��Ϊ����Ҫ�ȴ�EERPOMд��ɣ����Բ��ò�ѯ ����ʱ��ʽ��10ms��
    Systick_Delay_1ms(10);
    return TRUE;
}
//��ҳд��1�����ݣ���д�������ַ����д�볤�ȣ���д���ַ���������ͣ�
void EEPROM_BufferWrite(u8 *pBuffer, u16 length, u16 WriteAddress, u8 DeviceAddress )
{
    u16 i;
    u16 Addr = 0, count = 0;
    Addr = WriteAddress % I2C_PageSize; //д���ַ�ǿ�ʼҳ�ĵڼ�ҳ
    count = I2C_PageSize - Addr; //�ڿ�ʼҳҪд��ĸ���
    if(length <= count) {
        EEPROM_PageWrite(pBuffer, length, WriteAddress, DeviceAddress); //��дһҳ������
    } else {
        EEPROM_PageWrite(pBuffer, count, WriteAddress, DeviceAddress); //��д��һҳ������
        if((length - count) <= I2C_PageSize) {
            EEPROM_PageWrite(pBuffer + count, length - count, WriteAddress + count, DeviceAddress); //����дһҳ�����ݽ���
        } else {
            for(i = 0; i < ((length - count) / I2C_PageSize); i++) {
                EEPROM_PageWrite(pBuffer + count + i * I2C_PageSize, I2C_PageSize, WriteAddress + count + i * I2C_PageSize, DeviceAddress);
            }
            if( ((length - count) % I2C_PageSize) != 0 ) {
                EEPROM_PageWrite(pBuffer + count + i * I2C_PageSize, ((length - count) % I2C_PageSize), WriteAddress + count + i * I2C_PageSize, DeviceAddress);
            }
        }
    }
}


//����1�ֽ����ݣ���Ŷ������ݣ����������ȣ���������ַ���������ͣ�
bool EEPROM_RandomRead(u8 *pByte, u16 ReadAddress, u8 DeviceAddress)
{
    if(!I2C_Start()) return FALSE;
    I2C_SendByte(((ReadAddress & 0x0700) >> 7) | (DeviceAddress & 0xFE)); //���ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte((u8)(ReadAddress & 0x00FF));//���õ���ʼ��ַ
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(((ReadAddress & 0x0700) >> 7) | (DeviceAddress | 0x0001));
    I2C_WaitAck();
    *pByte = I2C_ReceiveByte();
    I2C_NoAck();
    I2C_Stop();
    return TRUE;
}

//����1�����ݣ���Ŷ������ݣ����������ȣ���������ַ���������ͣ�
bool EEPROM_SequentialRead(u8 *pBuffer, u8 length, u16 ReadAddress, u8 DeviceAddress)
{
    if(!I2C_Start()) return FALSE;
    I2C_SendByte(((ReadAddress & 0x0700) >> 7) | (DeviceAddress & 0xFE)); //���ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte((u8)(ReadAddress & 0x00FF));//���õ���ʼ��ַ
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(((ReadAddress & 0x0700) >> 7) | (DeviceAddress | 0x0001));
    I2C_WaitAck();
    while(length) {
        *pBuffer = I2C_ReceiveByte();
        if(length == 1) I2C_NoAck();
        else I2C_Ack();
        pBuffer++;
        length--;
    }
    I2C_Stop();
    return TRUE;
}


void I2CEEPROMWriteany(u16 addr, u8* ptr, u16 cnt)
{
    EEPROM_BufferWrite(ptr, cnt, addr, 0xA0);
}
/*******************************************************************************
* @name   : main
* @brief  : I2C poll mode read and write EEPROM
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    u16 i;
    u16 temp;


    delay_init();
    LED_Init();

    /*
        Use I2C1 Port to connect external I2C interface type EEPROM 24C02;
        run Write and Read series bytes Data;
    */
    //Initial I2C
    I2C_Initialize();
    //Write 16 bytes from buffer0[128] to 0x10 of EEPROM
    I2CEEPROMWrite(0x10, buffer0, 0x10);
    //Read 16 bytes from 0x10 of EEPROM to buffer1[128]
    I2CEEPROMRead(0x10, buffer1, 0x10);
    for(i = 0; i < 0x10; i++) {
        buffer0[i] = i;
    }
    I2CEEPROMWriteany(0x20, buffer0, 0x8);
    I2CEEPROMRead(0x20, buffer1, 0x08);
    I2CEEPROMWriteany(0x20 + 0x8, buffer0, 0x8 + 0x8);
    I2CEEPROMRead(0x20 + 0x8, buffer1, 0x8 + 0x8);
    I2CEEPROMRead(0x20, buffer1, 0x8 + 0x8);
    temp = 0;
    for(i = 0; i < 0x10; i++) {
        if((buffer0[i]) == (buffer1[i])) {
            temp++;
        }
    }
    while(1) {
        LED1_TOGGLE();
        if(temp < 0x10) {
            delay_ms(100);
        }

        else {
            delay_ms(500);
        }
    }
}


/*******************************************************************************
* @name   : I2C_Initialize
* @brief  : Initial I2C
* @param  : None
* @retval : None
*******************************************************************************/
void I2C_Initialize()
{
    I2C_GPIO_Config();
}



/*******************************************************************************
* @name   : I2CWrite
* @brief  : Write a data packet
* @param  : addr (Sub address of EEPROM)
* @param  : ptr (Data in the buffer)
* @param  : cnt (Number of data)
* @retval : None
*******************************************************************************/
void I2CEEPROMWrite(u16 addr, u8 *ptr, u16 cnt)
{
    u16 i;
    for(i = 0; i < cnt; i++) {
        EEPROM_ByteWrite(*ptr++, addr + i, 0xA0);
        delay_ms(1);
    }
}

/*******************************************************************************
* @name   : I2CRead
* @brief  : Receive a data packet
* @param  : addr (Sub address of EEPROM)
* @param  : ptr (Buffer to storage data)
* @param  : cnt (Number of data)
* @retval : None
*******************************************************************************/
void I2CEEPROMRead(u16 addr, u8* ptr, u16 cnt)
{
    EEPROM_SequentialRead(ptr, cnt, addr, 0xA0);
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
    NVIC_SetPriority(SysTick_IRQn, 0x0);
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
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, NewState);
    }
    if(GPIOx == GPIOB) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, NewState);
    }
    if(GPIOx == GPIOC) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, NewState);
    }
    if(GPIOx == GPIOD) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, NewState);
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

