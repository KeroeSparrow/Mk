#include <stm32f10x_conf.h>



void TIM4_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
        {
            // Обязательно сбрасываем флаг
            TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
            GPIOC->ODR ^= GPIO_Pin_13;
        }
}

int main(void)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* Initialize LED which connected to PC13 */
    GPIO_InitTypeDef  gpio;
    // Enable PORTC Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
    /* Configure the GPIO_LED pin */
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Set C13 to Low level ("0")

    // TIMER4
    TIM_TimeBaseInitTypeDef Timer;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
#if 0
    TIM_TimeBaseStructInit(&Timer);
    Timer.TIM_CounterMode = TIM_CounterMode_Up;
    Timer.TIM_Prescaler =8000;
    Timer.TIM_Period = 500;
    //Timer.TIM_
    TIM_TimeBaseInit(TIM4, &Timer);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
#endif

    //Blue
	gpio.GPIO_Pin = GPIO_Pin_0;
    	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    	gpio.GPIO_Speed = GPIO_Speed_50MHz;
    	GPIO_Init(GPIOB, &gpio);

    	TIM_TimeBaseStructInit(&Timer);
    	Timer.TIM_CounterMode = TIM_CounterMode_Up;
    	Timer.TIM_Prescaler =(SystemCoreClock/1000000)-1;
    	Timer.TIM_Period = 100;
    	TIM_TimeBaseInit(TIM3, &Timer);


    	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 100;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

        TIM_OC3Init(TIM3, &TIM_OCInitStructure);

        TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

       TIM_Cmd(TIM3, ENABLE);


       //Red

    	gpio.GPIO_Pin = GPIO_Pin_1;
        	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        	gpio.GPIO_Speed = GPIO_Speed_50MHz;
        	GPIO_Init(GPIOB, &gpio);

        	TIM_TimeBaseStructInit(&Timer);
        	Timer.TIM_CounterMode = TIM_CounterMode_Up;
        	Timer.TIM_Prescaler =(SystemCoreClock/1000000)-1;
        	Timer.TIM_Period = 100;
        	TIM_TimeBaseInit(TIM3, &Timer);


        	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
            TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
            TIM_OCInitStructure.TIM_Pulse = 100;
            TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

            TIM_OC4Init(TIM3, &TIM_OCInitStructure);

            TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

            TIM_Cmd(TIM3, ENABLE);

            //Green

            gpio.GPIO_Pin = GPIO_Pin_6;
            gpio.GPIO_Mode = GPIO_Mode_AF_PP;
            gpio.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &gpio);

            TIM_TimeBaseStructInit(&Timer);
            Timer.TIM_CounterMode = TIM_CounterMode_Up;
            Timer.TIM_Prescaler =(SystemCoreClock/1000000)-1;
            Timer.TIM_Period = 100;
            TIM_TimeBaseInit(TIM3, &Timer);


            TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
            TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
            TIM_OCInitStructure.TIM_Pulse = 100;
            TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

            TIM_OC1Init(TIM3, &TIM_OCInitStructure);

            TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

            TIM_Cmd(TIM3, ENABLE);







    /* NVIC Configuration */
    /* Enable the TIM4_IRQn Interrupt */
    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    while(1)
    {
    }
}
