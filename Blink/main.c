#include <stm32f10x_conf.h>

//1 delay tick = 4 mcu ticks
void delay_loop(uint32_t DelayTicks)
{
	__asm__ __volatile__ (
	"1: \n"
	"sub %[DelayTicks], %[DelayTicks], #1\n" //1tick
	"cmp %[DelayTicks], #0 \n" // 1tick
	"bne 1b \n" //1 or 2 ticks
	: [DelayTicks] "+r"(DelayTicks)
	);
}

int main(void)
{
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);

	 int led_state=0;

    while(1)
    {
    	delay_loop(SystemCoreClock/40);
    	GPIO_WriteBit(GPIOC, GPIO_Pin_13, led_state ? Bit_SET : Bit_RESET);
    	volatile uint8_t value = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    	led_state = !led_state;
    }
}
