实验器材:
	CordBoard MM32F003TW_q(TSSOP20)
	
实验目的:
	学习GPIO IN的使用
	
硬件资源:
	1,WK_UP(PA0),KEY1(PA5))
	2,电位器RV1
	3,UART1(TXD--PB7,RXD--PA6)
	
实验现象:
    使用杜邦线或跳冒连接PA4到LD PIN 或 用杜邦线连接任意其他GPIO到LD PIN 修改GPIO配置代码
    使用杜邦线或跳冒连接PA5到KEY PIN  P5 接pull up

	
注意事项:  
    要使用SWD接口调试时，需使用跳冒短接P12的SWCLK和SWDIO
    使用5V 时，需更改P2跳冒到5V电源
    使用UART时，需要使用跳冒短接P2的TXD和RXD

    
    
	