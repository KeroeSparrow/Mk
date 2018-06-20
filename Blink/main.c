#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

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

int main(void) {
        GPIO_InitTypeDef gpio;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        GPIO_StructInit(&gpio);
        gpio.GPIO_Pin = GPIO_Pin_1;
        gpio.GPIO_Mode = GPIO_Mode_Out_PP;
        gpio.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOA, &gpio);

        int led_state=0;
        while(1)  {
                volatile int i;
                //for (i=0; i<100000; ++i);
                delay_loop(SystemCoreClock/40);
                GPIO_WriteBit(GPIOA, GPIO_Pin_1, led_state ? Bit_SET : Bit_RESET);
                volatile uint8_t value = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
                led_state = !led_state;

        }
}
