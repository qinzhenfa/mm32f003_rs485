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
#include "string.h"
#include "HAL_conf.h"

/*my_include*/
#include "HAL_rcc.h"



u8 RxBuf[100];
u8 TxBuf[100];
char printBuf[100];
void Uart_ConfigInit(u32 bound);
void UartSendByte(u8 dat);
void UartSendGroup(u8* buf, u16 len);
void UartSendAscii(char *str);


void NVIC_ConfigInit(void);
void initGPIO_UART(void);
void uartConfig(unsigned int bound);


//***mygpio
void GPIO_ConfigInit(void);
void RCC_ConfigInit(void);

#define GPIOA_MASK_PIN 0x99ff
#define GPIOB_MASK_PIN 0xffff
#define GPIOC_MASK_PIN 0xffff
#define GPIOD_MASK_PIN 0x0004
//**********

typedef enum {
    emType_Polling = 0,
    emType_IT,
    emType_DMA,
    emType_Sync,
    emType_ASync,
    emType_Blocking,
    emType_NonBlocking,
} EM_Type;
typedef struct {
    EM_Type     	type;  		// polling, interrupt, dma
    EM_Type     	sync;       // Sync, ASync
    bool        	block;      // Blocking, NonBlocking
    bool 			Process;
    bool 			Complete;
    u16 			Cnt;
    u8* 			Ptr;
} UART_Process_TypeDef;


typedef struct {
    UART_TypeDef*	UARTx;
    u16 			idx;
    u32 			syncTx;
    u32 			syncRx;

    UART_Process_TypeDef *TxProcess;
    UART_Process_TypeDef *RxProcess;

} DRV_UART_INSTANT_TypeDef;

DRV_UART_INSTANT_TypeDef vUART2INSTANT, *pUART2INSTANT;
UART_Process_TypeDef vUART2TxProcess;
UART_Process_TypeDef vUART2RxProcess;
u32 SysTick_Count;
u32 gDlycnt;

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
    if(gDlycnt > 0) gDlycnt --;
}

void UART2_TxProcessStructInit(void)
{
    vUART2TxProcess.type     = emType_IT;
    vUART2TxProcess.sync     = emType_ASync;
    vUART2TxProcess.block    = TRUE;
    vUART2TxProcess.Process  = FALSE;
    vUART2TxProcess.Complete   = FALSE;
    vUART2TxProcess.Ptr      = NULL;
    vUART2TxProcess.Cnt      = 0;
}

void UART2_RxProcessStructInit(void)
{
    vUART2RxProcess.type     = emType_IT;
    vUART2RxProcess.sync     = emType_ASync;
    vUART2RxProcess.block    = FALSE;
    vUART2RxProcess.Process  = FALSE;
    vUART2RxProcess.Complete   = FALSE;
    vUART2RxProcess.Ptr      = NULL;
    vUART2RxProcess.Cnt      = 0;
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  the UART receives data according to the ID in the HANDLE.
/// @param  handle: pointer to a HANDLE structure that contains
///                the instance for the specified UART.
/// @param  ptr: Pointer to data buffer.
/// @param  count: Amount of data to be received.
/// @retval : Received data number.
////////////////////////////////////////////////////////////////////////////////
int UART2_ReadBuf(u8* ptr, u16 len)
{

    if (!vUART2RxProcess.Process) {
        vUART2RxProcess.Process = TRUE;
        vUART2RxProcess.Complete = FALSE;
        vUART2RxProcess.Cnt = len;
        vUART2RxProcess.Ptr = ptr;
        if (emType_Polling == vUART2RxProcess.type) {

            //UART2_PollingRcvPacket_Block();
        } else if (emType_IT == vUART2RxProcess.type) {
            //   UART_ITConfig(UART2, UART_IT_RXIEN, DISABLE);
            UART_ITConfig(UART2, UART_IT_RXIEN, ENABLE);

        } else if (emType_DMA == vUART2RxProcess.type) {
            //DRV_UART2_DMA_RcvPacket();
        } else {
        }
        if (vUART2RxProcess.block == 1) {
            while (!vUART2RxProcess.Complete) {
                //			if (vUART2RxProcess.bTimeOut && (nTimeOutCnt >= vUART2RxProcess.TimeOut)) {
                //				return emRETURN_TimeOut;
                //			}
            }
            vUART2RxProcess.Process = FALSE;
            return 1;
        }
    }


    else {
        if (vUART2RxProcess.Complete) {
            vUART2RxProcess.Process = FALSE;
            return 1;
        } else {
            return 0;
        }

    }

    return 0;
}
/*******************************************************************************
* @name   : UART2_WriteBuf
* @brief  : Send the data
* @param  : None
* @retval :
*******************************************************************************/
int UART2_WriteBuf(u8* ptr, u16 count)
{

    if (!(vUART2TxProcess.Process)) {
        vUART2TxProcess.Process = TRUE;
        vUART2TxProcess.Complete = FALSE;

        vUART2TxProcess.Ptr = ptr;
        vUART2TxProcess.Cnt = count;

        if (emType_Polling == vUART2TxProcess.type) {

            //UART2_PollingSendPacket_Block(idx);
        } else if (emType_IT == vUART2TxProcess.type) {

            //UART_ITConfig(UART2, UART_IT_TXIEN, ENABLE);
            vUART2TxProcess.Cnt--;
            UART_SendData(UART2, *(u8*)(vUART2TxProcess.Ptr++));
            UART_ITConfig(UART2, UART_IT_TXIEN, ENABLE);
        } else if (emType_DMA == vUART2TxProcess.type) {
            //DRV_UART2_DMA_SendPacket(idx);
        } else {
        }
    }

    if (vUART2TxProcess.block == 1) {
        while (!vUART2TxProcess.Complete);
    } else {
        if (!vUART2TxProcess.Complete)
            return 0;
    }

    vUART2TxProcess.Process = FALSE;
    return 1;
}



/************mydelay*****************/
static void delay(uint32_t uldelay)
{
    u32 i;
   i = uldelay; while(i--);
}


/*******************************************************************************
* @name   : main
* @brief  : Send the received data
* @param  : None
* @retval : void
*******************************************************************************/
int main(void)
{
    int ret;
//    int t=0;
//		RCC_ConfigInit();
    InitSystick();
	
	//****************my*****************
		GPIO_ConfigInit();        //初始化pb0
		//***************************************

    Uart_ConfigInit(115200);
		 //   UART2_WriteBuf((u8*)printBuf, sprintf(printBuf,"sprintf ok\r\n"));
    UartSendGroup((u8*)printBuf, sprintf(printBuf, "sprintf ok\r\n"));          //pintf stdio data
    UartSendAscii("UartSendAscii\r\n");                                         //printf string
    UartSendAscii("Please input data ,end with '\\n' \r\n");
	
	
	
	//**********my//	  
//					delay(1000);
					GPIO_ResetBits(GPIOB, GPIO_Pin_0);            
//					delay(1000);
					GPIO_SetBits(GPIOB, GPIO_Pin_0);                 //将PB0拉高(DE拉高),让D3082EHY8一直发rs485信号(硬件部分RE已接地)
					

    while(1) {
                    delay(1000000);
			//mytest
//			UartSendAscii("start pb0 pull up successful！\r\n");			
//			UartSendByte(1);
//			UartSendByte(0);
//			UartSendAscii("1 '\\n' \r\n");

//                        UartSendByte(t);
//                        t++; 
                        UartSendAscii("the card receive！\r\n");
            		UartSendAscii("插卡已收到！\r\n");
                        
        ret = UART2_ReadBuf(RxBuf, 0);
        if(ret == 1) {
            memcpy(TxBuf, RxBuf, 0);
            UART2_WriteBuf(TxBuf, 0);
        }
				  
//		UartSendGroup((u8*)printBuf, sprintf(printBuf, "sprintf ok\r\n"));          //pintf stdio data
//    UartSendAscii("UartSendAscii\r\n");                                         //printf string
//    UartSendAscii("Please input data ,end with '\\n' \r\n");								
//				memset(TxBuf, 0, sizeof(TxBuf));
//				memset(RxBuf, 0, sizeof(RxBuf));
				
    }

}
u8 vtemp[256];
u8 vNo = 0;
void UART2_IRQHandler(void)
{
    UART_TypeDef* UARTx = UART2;
    u8 temp;

// Receive packet
    if (UART_GetITStatus(UARTx, UART_IT_RXIEN) != RESET) {
        UART_ClearITPendingBit(UARTx, UART_IT_RXIEN);
        temp = UART_ReceiveData(UARTx);
        vtemp[vNo++] = temp;

        if ((vUART2RxProcess.Cnt) > 0) {
            *(u8*)(vUART2RxProcess.Ptr) = temp;
            vUART2RxProcess.Ptr++;
            vUART2RxProcess.Cnt--;
            if ((vUART2RxProcess.Cnt) == 0) {
                vUART2RxProcess.Complete = TRUE;
                UART_ITConfig(UARTx, UART_IT_RXIEN, DISABLE);
//                if (vUART2RxProcess.sync == emType_Sync) {
//                    funcptr fun = (funcptr)(vUART2RxProcess.syncRx);
//                   (*fun)();
//                }
            }

        }

    }
// Send packet
    if (UART_GetITStatus(UARTx, UART_IT_TXIEN) != RESET) {
        UART_ClearITPendingBit(UARTx, UART_IT_TXIEN);

        if (vUART2TxProcess.Cnt > 0) {
            UART_SendData(UARTx, *(u8*)(vUART2TxProcess.Ptr));
            vUART2TxProcess.Ptr++;
            vUART2TxProcess.Cnt--;
        } else {
            UART_ITConfig(UARTx, UART_IT_TXIEN, DISABLE);
            while (!UART_GetFlagStatus(UARTx, UART_CSR_TXC));
            vUART2TxProcess.Complete = TRUE;
//			if (vUART2TxProcess.sync == emType_Sync) {
//				funcptr fun = (funcptr)(vUART2TxProcess.syncfun);
//			   (*fun)();
//			}
        }

    }
}


/*******************************************************************************
* @name   : NVIC_ConfigInit
* @brief  : NVIC Config Init
* @param  : None
* @retval : void
*******************************************************************************/
void NVIC_ConfigInit(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = UART_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    NVIC_Init(& NVIC_InitStruct);

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
    UART2_TxProcessStructInit();
    UART2_RxProcessStructInit();
    NVIC_ConfigInit();
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

    UART_ITConfig( UART,  UART_IT_RXIEN, ENABLE);

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
    UART2_WriteBuf(&dat, (u32)1);
}

/*******************************************************************************
* @name   : UartSendGroup
* @brief  : Uart Send Group
* @param  : u8* buf,u16 len
* @retval : void
*******************************************************************************/
void UartSendGroup(u8* buf, u16 len)
{
    UART2_WriteBuf(buf, (u32)len);
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






/*my_gpio*/
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

/*******************************************************************************
* @name   : GPIO_ConfigInit
* @brief  : GPIO config
* @param  : None
* @retval : void
*******************************************************************************/
void GPIO_ConfigInit(void)   							//初始化pb0
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_Clock_Set(GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*my_rcc*/

void RCC_ConfigInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;   //MCO  PB1
    GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 推免复用输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig( GPIOB, GPIO_PinSource1, GPIO_AF_5);

    RCC_MCOConfig(RCC_MCO_SYSCLK);  //通过PB1 pin 观察频率
}



