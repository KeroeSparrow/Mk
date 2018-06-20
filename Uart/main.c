#include <stm32f10x_conf.h>
#include <stdio.h>
#include <string.h>
void USART_Print(void);

//1 delay tick = 4 mcu ticks
void delay_loop(uint32_t DelayTicks) {
__asm__ __volatile__ (
"1: \n"
"sub %[DelayTicks], %[DelayTicks], #1\n" //1tick
"cmp %[DelayTicks], #0 \n" // 1tick
"bne 1b \n" //1 or 2 ticks
: [DelayTicks] "+r"(DelayTicks)
);
}


void Usart_sendchar(uint16_t x ){
while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_SendData(USART1, x);
}

void Usart_string(char* x){
	int i;
	for (i=0; i<=stlen(x);i++){
		Usart_sendchar(x[i]);
	}

}

int main(void) {
        GPIO_InitTypeDef gpio;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        GPIO_StructInit(&gpio);
        gpio.GPIO_Pin = GPIO_Pin_1;
        gpio.GPIO_Mode = GPIO_Mode_Out_PP;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio);

        //GPIO_PinLockConfig(GPIO_Mode_AF_USART2 GPIOA, GPIO_PinSource2);
        //GPIO_PinLockConfig(GPIO_Mode_AF_USART2 GPIOA, GPIO_PinSource3);

        //заполняем поля структуры
        // PD5 -> TX UART.
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        gpio.GPIO_Pin = GPIO_Pin_9;
        GPIO_Init(GPIOA, &gpio);

        //PD6  -> RX UART.
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        gpio.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOA, &gpio);

        USART_InitTypeDef USART;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        USART.USART_BaudRate = 9600;// скорость
        USART.USART_WordLength = USART_WordLength_8b; //8 бит данных
        USART.USART_StopBits = USART_StopBits_1; //один стоп бит
        USART.USART_Parity = USART_Parity_No; //четность - нет
        USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // управлени потоком - нет
        USART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       // разрешаем прием и передачу
        USART_Init(USART1, &USART);

        USART_Cmd(USART1, ENABLE);

        Usart_sendchar(H);

        int led_state=0;
        int count=0;
	while(1) {
		count++;
        //volatile int i;
        //for (i=0; i<100000; ++i);
        delay_loop(SystemCoreClock);
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, led_state ? Bit_SET : Bit_RESET);
        Usart_sendchar(H);
        //while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        //USART_SendData(USART1,'\n');

        delay_loop(SystemCoreClock/90);
        led_state = !led_state;
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, led_state ? Bit_SET : Bit_RESET);
        //volatile uint8_t value = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
        led_state = !led_state;
        if (count>10000) {count=0;}
	}
}
