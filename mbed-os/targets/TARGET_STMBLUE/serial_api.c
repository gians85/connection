#ifdef DEVICE_SERIAL

#include "serial_api.h"
#include "cmsis.h"
#include "serial_device.h"
#include "pin_device.h"


int stdio_uart_inited = 0;
serial_t stdio_uart;
static uart_irq_handler irq_handler;


void serial_init(serial_t *obj, PinName tx, PinName rx){

	UART_InitType UART_InitStructure;


	/* GPIO Periph clock enable */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART | CLOCK_PERIPH_GPIO, ENABLE);

	GPIO_InitType GPIO_InitStructure;

	/* Configure GPIO_Pin_8 and GPIO_Pin_11 as UART_TXD and UART_RXD*/
	GPIO_InitStructure.GPIO_Pin = getGpioPin(tx);//getGpioPin(SERIAL_TX);   //SDK_EVAL_UART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_InitStructure.GPIO_HighPwr = DISABLE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = getGpioPin(rx);//getGpioPin(SERIAL_RX);   //SDK_EVAL_UART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = Serial1_Mode;  //SDK_EVAL_UART_RX_MODE;
	GPIO_Init(&GPIO_InitStructure);

	/*
	  ------------ USART configuration -------------------
	  - BaudRate = 115200 baud
	  - Word Length = 8 Bits
	  - One Stop Bit
	  - No parity
	  - Hardware flow control disabled (RTS and CTS signals)
	  - Receive and transmit enabled
	 */
	UART_InitStructure.UART_BaudRate = BAUDRATE;
	UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
	UART_InitStructure.UART_WordLengthReceive = UART_WordLength_8b;
	UART_InitStructure.UART_StopBits = UART_StopBits_1;
	UART_InitStructure.UART_Parity = UART_Parity_No;
	UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
	UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
	UART_InitStructure.UART_FifoEnable = ENABLE;
	UART_Init(&UART_InitStructure);

	obj->uart = UART_1;

	obj->init = &UART_InitStructure;

	/* Interrupt as soon as data is received. */
	UART_RxFifoIrqLevelConfig(FIFO_LEV_1_64);

	/* Enable UART */
	UART_Cmd(ENABLE);

	stdio_uart_inited = 1;
}

void serial_putc(serial_t *obj, int c){
	  /* Wait if TX fifo is full. */
	  while (UART_GetFlagStatus(UART_FLAG_TXFF) == SET);
	  /* send the data */
	  UART_SendData((uint16_t)c);
}

int serial_getc(serial_t *obj){
	/* Loop until the UART Receive Data Register is not empty */
	while (UART_GetFlagStatus(UART_FLAG_RXFE) == SET);
	/* Store the received byte in RxBuffer */
	return (int) UART_ReceiveData();
}

void serial_baud(serial_t *obj, int baudrate){
	obj->init->UART_BaudRate = baudrate;
}

/*void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id){
}*/

/*void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable){
}*/
static uint32_t serial_irq_ids = 0;

void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id){
    irq_handler = handler;
    serial_irq_ids = id;
    obj->index_irq = id;
}

void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable){
	/* NVIC configuration */
	NVIC_InitType NVIC_InitStructure;
	/* Enable the UART Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = (FunctionalState)enable;
	NVIC_Init(&NVIC_InitStructure);
	//UART_ITConfig(UART_IT_RX, enable);
	if (irq == RxIrq)
		UART_ITConfig(UART_IT_RX, (FunctionalState)enable);
	else// TxIrq
		UART_ITConfig(UART_IT_TXFE, (FunctionalState)enable);
}

void UART_Handler(void){
	if (UART_GetITStatus(UART_IT_RX) != RESET){
		UART_ClearITPendingBit(UART_IT_RX);
		irq_handler(serial_irq_ids, RxIrq);
	}
    if (UART_GetITStatus(UART_IT_TXFE) != RESET){
        UART_ClearITPendingBit(UART_IT_TXFE);
        irq_handler(serial_irq_ids, TxIrq);
    }
}

#endif //DEVICE_SERIAL
