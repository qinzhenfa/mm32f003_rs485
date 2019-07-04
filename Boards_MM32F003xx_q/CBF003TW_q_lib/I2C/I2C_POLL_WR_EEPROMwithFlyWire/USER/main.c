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
#include "stdio.h"
#include "stdbool.h"
#include "HAL_conf.h"

#define USEPB1314AS_SCLSDA
#ifdef USEPB67AS_SCLSDA

#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SCL_PIN                     GPIO_Pin_6
#define I2C1_SCL_PORT                    GPIOB
#define I2C1_SCL_AFSOURCE                GPIO_PinSource6
#define I2C1_SCL_AFMODE                  GPIO_AF_1

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_PIN                     GPIO_Pin_7
#define I2C1_SDA_PORT                    GPIOB
#define I2C1_SDA_AFSOURCE                GPIO_PinSource7
#define I2C1_SDA_AFMODE                  GPIO_AF_1
#endif


#ifdef USEPB89AS_SCLSDA

#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SCL_PIN                     GPIO_Pin_8
#define I2C1_SCL_PORT                    GPIOB
#define I2C1_SCL_AFSOURCE                GPIO_PinSource8
#define I2C1_SCL_AFMODE                  GPIO_AF_1

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_PIN                     GPIO_Pin_9
#define I2C1_SDA_PORT                    GPIOB
#define I2C1_SDA_AFSOURCE                GPIO_PinSource9
#define I2C1_SDA_AFMODE                  GPIO_AF_1
#endif
#ifdef USEPB1011AS_SCLSDA

#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SCL_PIN                     GPIO_Pin_10
#define I2C1_SCL_PORT                    GPIOB
#define I2C1_SCL_AFSOURCE                GPIO_PinSource10
#define I2C1_SCL_AFMODE                  GPIO_AF_1

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_PIN                     GPIO_Pin_11
#define I2C1_SDA_PORT                    GPIOB
#define I2C1_SDA_AFSOURCE                GPIO_PinSource11
#define I2C1_SDA_AFMODE                  GPIO_AF_1
#endif
#ifdef USEPB1314AS_SCLSDA

#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SCL_PIN                     GPIO_Pin_13
#define I2C1_SCL_PORT                    GPIOB
#define I2C1_SCL_AFSOURCE                GPIO_PinSource13
#define I2C1_SCL_AFMODE                  GPIO_AF_5

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_PIN                     GPIO_Pin_14
#define I2C1_SDA_PORT                    GPIOB
#define I2C1_SDA_AFSOURCE                GPIO_PinSource14
#define I2C1_SDA_AFMODE                  GPIO_AF_5
#endif
#ifdef USEPA54AS_SCLSDA

#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOA
#define I2C1_SCL_PIN                     GPIO_Pin_5
#define I2C1_SCL_PORT                    GPIOA
#define I2C1_SCL_AFSOURCE                GPIO_PinSource5
#define I2C1_SCL_AFMODE                  GPIO_AF_5

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOA
#define I2C1_SDA_PIN                     GPIO_Pin_4
#define I2C1_SDA_PORT                    GPIOA
#define I2C1_SDA_AFSOURCE                GPIO_PinSource4
#define I2C1_SDA_AFMODE                  GPIO_AF_5
#endif

#ifdef USEPA910AS_SCLSDA

#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOA
#define I2C1_SCL_PIN                     GPIO_Pin_9
#define I2C1_SCL_PORT                    GPIOA
#define I2C1_SCL_AFSOURCE                GPIO_PinSource9
#define I2C1_SCL_AFMODE                  GPIO_AF_4

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOA
#define I2C1_SDA_PIN                     GPIO_Pin_10
#define I2C1_SDA_PORT                    GPIOA
#define I2C1_SDA_AFSOURCE                GPIO_PinSource10
#define I2C1_SDA_AFMODE                  GPIO_AF_4
#endif
#ifdef USEPA1112AS_SCLSDA

#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOA
#define I2C1_SCL_PIN                     GPIO_Pin_11
#define I2C1_SCL_PORT                    GPIOA
#define I2C1_SCL_AFSOURCE                GPIO_PinSource11
#define I2C1_SCL_AFMODE                  GPIO_AF_5

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOA
#define I2C1_SDA_PIN                     GPIO_Pin_12
#define I2C1_SDA_PORT                    GPIOA
#define I2C1_SDA_AFSOURCE                GPIO_PinSource12
#define I2C1_SDA_AFMODE                  GPIO_AF_5
#endif
#ifdef USEPD10AS_SCLSDA

#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOD
#define I2C1_SCL_PIN                     GPIO_Pin_1
#define I2C1_SCL_PORT                    GPIOD
#define I2C1_SCL_AFSOURCE                GPIO_PinSource1
#define I2C1_SCL_AFMODE                  GPIO_AF_1

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOD
#define I2C1_SDA_PIN                     GPIO_Pin_0
#define I2C1_SDA_PORT                    GPIOD
#define I2C1_SDA_AFSOURCE                GPIO_PinSource0
#define I2C1_SDA_AFMODE                  GPIO_AF_1
#endif

#define PAGESIZE 16                                                             //The size of each EEPROM page
#define EEPROM_ADDR 0xA8                                                        //Device address of EEPROM
#define MAX(a,b)((a)>(b)?(a):(b))
#define MIN(a,b)((a)<(b)?(a):(b))
typedef struct {
    u8 busy;                                                                    //Flag of whether I2C is transmitting
    u8 ack;
    u8 fault;                                                                   //Flag of whether I2C is right
    u8 opt;                                                                     //Flag of I2C transmission direction
    u8 sub;                                                                     //sub address
    u8 cnt;                                                                     //number
    u8 *ptr;                                                                    //buffer
    u8 sadd;                                                                    //used to determine if sub addresses need to be sent in interrupt
} i2c_def;
i2c_def i2c;
enum {WR, RD};                                                                  //write or read

char printBuf[100];
void Uart_ConfigInit(u32 bound);
void UartSendGroup(u8* buf, u16 len);
void UartSendAscii(char *str);

void I2CWrite(u8 sub, u8* ptr, u16 len);
void I2CRead(u8 sub, u8* ptr, u16 len);
void I2C_MasterMode_Init(I2C_TypeDef *I2Cx, u32 uiI2C_speed);
void I2C_SetDeviceAddr(I2C_TypeDef *I2Cx, u8 deviceaddr);
void I2C_WaitEEready(void);
void I2C_TXByte(u8 dat);
u8   I2C_SendBytes(u8 sub, u8* ptr, u16 cnt);
u8   I2C_SendPacket(u8 sub, u8* ptr, u16 cnt);
void I2C_RevBytes(void);
void I2C_RcvPacket(u8 sub, u8* ptr, u16 cnt);
void I2C_Check(void);
void I2C_Initialize(void);

u8 buffer0[128] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};
u8 buffer1[128];

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

/*******************************************************************************
* @name   : main
* @brief  : I2C poll mode read and write EEPROM
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{

    InitSystick();
    /*
        Use I2C1 Port to connect external I2C interface type EEPROM 24C02;
        run Write and Read series bytes Data;
    */
    I2C_Initialize();                                                                    //Initial I2C

    I2CWrite(0x10, buffer0, 0x10);                                                //Write 16 bytes from buffer0[128] to 0x10 of EEPROM
    I2CRead(0x10, buffer1, 0x10);                                                 //Read 16 bytes from 0x10 of EEPROM to buffer1[128]

    while(1) {
    }
}


/*******************************************************************************
* @name   : I2CWrite
* @brief  : Write a data packet
* @param  : sub (Sub address of EEPROM)
* @param  : ptr (Data in the buffer)
* @param  : len (Number of data)
* @retval : None
*******************************************************************************/
void I2CWrite(u8 sub, u8* ptr, u16 len)
{
    do {
        I2C_SendPacket(sub, ptr, len);                                      //write data
        while(i2c.busy);                                                    //till I2C is not work
    } while(!i2c.ack);
}

/*******************************************************************************
* @name   : I2CRead
* @brief  : Receive a data packet
* @param  : sub (Sub address of EEPROM)
* @param  : ptr (Buffer to storage data)
* @param  : len (Number of data)
* @retval : None
*******************************************************************************/
void I2CRead(u8 sub, u8* ptr, u16 len)
{
    do {
        I2C_RcvPacket(sub, ptr, len);                                           //read data
        while(i2c.busy);                                                        //till I2C is not work
    } while(!i2c.ack);
}


void I2C1BusFreeGPIOMode(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(I2C1_SCL_BUSCLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = I2C1_SCL_PIN;                                //I2C uses PB6, PB7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_20MHz;                            //Set GPIO spped
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //Keep the bus free which means SCK & SDA is high
    GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStructure);                              //Initializes the GPIOB peripheral

    RCC_AHBPeriphClockCmd(I2C1_SDA_BUSCLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = I2C1_SDA_PIN;                                //I2C uses PB6, PB7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_20MHz;                            //Set GPIO spped
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //Keep the bus free which means SCK & SDA is high
    GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStructure);                              //Initializes the GPIOB peripheral
}
void I2C1ConfigGPIOMode(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(I2C1_SCL_BUSCLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = I2C1_SCL_PIN;                                //I2C uses PB6, PB7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_20MHz;                            //Set GPIO spped
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;                             //Keep the bus free which means SCK & SDA is high
    GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStructure);                              //Initializes the GPIOB peripheral

    RCC_AHBPeriphClockCmd(I2C1_SDA_BUSCLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = I2C1_SDA_PIN;                                //I2C uses PB6, PB7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_20MHz;                            //Set GPIO spped
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;                             //Keep the bus free which means SCK & SDA is high
    GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStructure);                              //Initializes the GPIOB peripheral

    GPIO_PinAFConfig(I2C1_SCL_PORT, I2C1_SCL_AFSOURCE, I2C1_SCL_AFMODE);
    GPIO_PinAFConfig(I2C1_SDA_PORT, I2C1_SDA_AFSOURCE, I2C1_SDA_AFMODE);
}

/*******************************************************************************
* @name   : I2C_MasterMode_Init
* @brief  : Initializes the I2Cx master mode
* @param  : I2Cx(where x can be 1 or 2 to select the I2C peripheral)
* @param  : uiI2C_speed: I2C speed
* @retval : None
*******************************************************************************/
void I2C_MasterMode_Init(I2C_TypeDef *I2Cx, u32 uiI2C_speed)
{
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);  	                        //Enable I2C reset state

    I2C1BusFreeGPIOMode();




    I2C_InitStructure.I2C_Mode = I2C_Mode_MASTER;                                 //Configure I2C as master mode
    I2C_InitStructure.I2C_OwnAddress = 0;
    I2C_InitStructure.I2C_Speed = I2C_Speed_STANDARD;
    I2C_InitStructure.I2C_ClockSpeed = uiI2C_speed;

    I2C_Init(I2Cx, &I2C_InitStructure);                                           //Initializes the I2Cx peripheral according to the specified
    I2C_Cmd(I2Cx, ENABLE);                                                        //Enable I2C

    I2C1ConfigGPIOMode();

}

/*******************************************************************************
* @name   : I2C_SetDeviceAddr
* @brief  : Set the device address
* @param  : I2Cx(where x can be 1 or 2 to select the I2C peripheral)
* @param  : deviceaddr(device address)
* @retval : None
*******************************************************************************/
void I2C_SetDeviceAddr(I2C_TypeDef *I2Cx, u8 deviceaddr)
{
    I2C1BusFreeGPIOMode();

    I2C_Cmd(I2Cx, DISABLE);                                                     //Disable I2C
    I2C_Send7bitAddress(I2Cx, deviceaddr, I2C_Direction_Transmitter);	         //Set the device address
    I2C_Cmd(I2Cx, ENABLE);                                                      //Enable I2C

    I2C1ConfigGPIOMode();
}

/*******************************************************************************
* @name   : I2C_WaitEEready
* @brief  : Wait for EEPROM getting ready.
* @param  : dat(None)
* @retval : None
*******************************************************************************/
void I2C_WaitEEready(void)
{
    u32 i = 10000;                                                              //eeprom operation interval delay
    while(i--);
}

/*******************************************************************************
* @name   : I2C_TXByte
* @brief  : Send a byte
* @param  : dat(data)
* @retval : None
*******************************************************************************/
void I2C_TXByte(u8 dat)
{
    I2C_SendData(I2C1, dat);                                                     //Send data
    while(I2C_GetFlagStatus(I2C1, I2C_STATUS_FLAG_TFE) == 0);                    //Checks whether transmit FIFO completely empty or not
}

/*******************************************************************************
* @name   : I2C_RevBytes
* @brief  : Receive a byte
* @param  : ptr(None)
* @retval : None
*******************************************************************************/
void I2C_RevBytes(void)
{
    u8 i, flag = 0, _cnt = 0;
    for (i = 0; i < i2c.cnt; i++) {
        while(1) {
            if ((I2C_GetFlagStatus(I2C1, I2C_STATUS_FLAG_TFNF)) && (flag == 0)) { //Write command is sent when RX FIFO is not full
                I2C_ReadCmd(I2C1);                                               //Configure to read
                _cnt++;
                if (_cnt == i2c.cnt)                                            //When flag is set, receive complete
                    flag = 1;
            }
            if (I2C_GetFlagStatus(I2C1, I2C_STATUS_FLAG_RFNE)) {                 //Check receive FIFO not empty
                i2c.ptr[i] = I2C_ReceiveData(I2C1);                              //read data to i2c.ptr
                break;
            }
        }
    }
}

/*******************************************************************************
* @name   : I2C_SendBytes
* @brief  : Send bytes
* @param  : sub(Sub address of EEPROM)
* @param  : ptr(Data in the buffer)
* @param  : cnt(Number of data)
* @retval : The state of this transmission
*******************************************************************************/
u8 I2C_SendBytes(u8 sub, u8* ptr, u16 cnt)
{
    I2C_TXByte(sub);                                                            //Send sub address
    while (cnt --) {
        I2C_TXByte(*ptr);                                                       //Send data
        ptr++;                                                                  //Point to the next data
    }
    I2C_GenerateSTOP(I2C1, ENABLE);                                              //Stop transmission
    while((I2C_GetITStatus(I2C1, I2C_IT_STOP_DET)) == 0);                        //Checks whether stop condition has occurred or not.
    i2c.ack = true;
    i2c.busy = false;                                                           //I2C operation stops
    I2C_WaitEEready();                                                          //Wait for EEPROM getting ready.
    return true;
}

/*******************************************************************************
* @name   : I2C_SendPacket
* @brief  : Send a data packet
* @param  : sub(Sub address of EEPROM)
* @param  : ptr(Data in the buffer)
* @param  : cnt(Number of data)
* @retval : The state of this transmission
*******************************************************************************/
u8 I2C_SendPacket(u8 sub, u8* ptr, u16 cnt)
{
    u8 i;
    i2c.opt = WR;                                                               //i2c option flag set to write
    i2c.cnt = cnt;                                                              //number to send
    i2c.sub = sub;                                                              //sub address
    i2c.busy = true;                                                            //I2C operation starts
    i2c.ack = false;

    if ((sub % PAGESIZE) > 0) {
        u8 temp = MIN((PAGESIZE - sub % PAGESIZE), i2c.cnt);                    //Need temp number of data, just right to the page address
        if(I2C_SendBytes(sub, ptr, temp)) {                                     //If WRITE successful
            ptr +=  temp;                                                       //Point to the next page
            i2c.cnt -=  temp;
            sub += temp;
        }
        if (i2c.cnt == 0) return true;                                          //i2c.cnt = 0 means transmition complete
    }
    for (i = 0; i < (i2c.cnt / PAGESIZE); i++) {
        if (I2C_SendBytes(sub, ptr, PAGESIZE)) {                                //Full page write
            ptr += PAGESIZE;                                                    //Point to the next page
            sub += PAGESIZE;
            i2c.cnt -= PAGESIZE;
        }
        if (i2c.cnt == 0) return true;
    }
    if (i2c.cnt > 0) {
        if (I2C_SendBytes(sub, ptr, i2c.cnt)) return true;
    }
    i2c.busy = false;                                                           //I2C operation ends
    i2c.ack = true;
    return false;
}

/*******************************************************************************
* @name   : I2C_RcvPacket
* @brief  : Receive a data packet
* @param  : sub(Sub address of EEPROM)
* @param  : ptr(Buffer to storage data)
* @param  : cnt(Number of data)
* @retval : None
*******************************************************************************/
void I2C_RcvPacket(u8 sub, u8* ptr, u16 cnt)
{
    i2c.busy = true;                                                            //I2C operation starts
    i2c.ack = false;
    i2c.sub = sub;
    i2c.ptr = ptr;
    i2c.cnt = cnt;

    I2C_TXByte(i2c.sub);                                                        //Send sub address
    I2C_RevBytes();                                                             //receive bytes
    I2C_GenerateSTOP(I2C1, ENABLE);                                              //Stop transmission
    while((I2C_GetITStatus(I2C1, I2C_IT_STOP_DET)) == 0);                        //Checks whether stop condition has occurred or not.

    i2c.busy = false;                                                           //I2C operation ends
    i2c.ack = true;
    I2C_WaitEEready();
}

/*******************************************************************************
* @name   : I2C_Initialize
* @brief  : Initial I2C
* @param  : None
* @retval : None
*******************************************************************************/
void I2C_Initialize()
{
    memset(&i2c, 0x00, sizeof(i2c));                                            //Initial value of i2c struct
    I2C_MasterMode_Init(I2C1, 100000);                                           //Initializes the I2C master mode
    I2C_SetDeviceAddr(I2C1, EEPROM_ADDR);                                         //Set the EEPROM address
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
