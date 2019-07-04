实验器材:
	CordBoard MM32F003TW_q(TSSOP20)
	
实验目的:
	学习ADC的使用
	
硬件资源:
	1,WK_UP(PA0),KEY1(PA5))
	2,电位器RV1
	3,UART1(TXD--PB7,RXD--PA6)
	
实验方法与结果:
    使用杜邦线连接PA0 到外置SPI Flash 25xx 的NSS  Pin
	使用杜邦线连接PB14到外置SPI Flash 25xx 的SCLK Pin
	使用杜邦线连接PB13到外置SPI Flash 25xx 的MOSI Pin
	使用杜邦线连接PA13到外置SPI Flash 25xx 的MISO Pin
	编译后，使用调试器烧录(Flash Download)到目标板，因使用了PA13(SWDIO) 做为SPI的MISO pin，无法仿真；
	复位或重新上电后，看到正常写入，读取目标SPI Flash中的数据；

	
注意事项:  
    要使用SWD接口调试时，需使用跳冒短接P12的SWCLK和SWDIO
    使用5V 时，需更改P2跳冒到5V电源
    使用UART时，需要使用跳冒短接P2的TXD和RXD
    使用杜邦线或跳冒连接PA5到KEY PIN
	使用杜邦线或跳冒连接PA4到LD PIN
    
    
	