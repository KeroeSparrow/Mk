#include <stm32f10x_conf.h>

#define redc 102
#define greenc 102
#define bluec 0

void initPWM(char colour, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,TIM_TypeDef* TIMx, int channel ){
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef  gpio;
	gpio.GPIO_Pin = GPIO_Pin;
			gpio.GPIO_Mode = GPIO_Mode_AF_PP;
			gpio.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOx, &gpio);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 switch(colour) {
	 case 'g': {TIM_OCInitStructure.TIM_Pulse = greenc; break;}
	 case 'b': {TIM_OCInitStructure.TIM_Pulse = bluec; break;}
	 case 'r': {TIM_OCInitStructure.TIM_Pulse = redc; break;}
	 }
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	 switch(channel){
		 case 1 : {
			 TIM_OC1Init(TIMx, &TIM_OCInitStructure);
			 TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
		 }
		 case 2 : {
			 TIM_OC2Init(TIMx, &TIM_OCInitStructure);
			 TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
		 }
		 case 3 : {
			 TIM_OC3Init(TIMx, &TIM_OCInitStructure);
			 TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
		 }
		 case 4 : {
			 TIM_OC4Init(TIMx, &TIM_OCInitStructure);
			 TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
		 }
	 }

	TIM_Cmd(TIMx, ENABLE);
}




int main(void)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef  gpio;
    // Enable PORTC Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
      TIM_TimeBaseInitTypeDef Timer;
      NVIC_InitTypeDef nvic;

      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  	gpio.GPIO_Pin = GPIO_Pin_13;
  		gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  		gpio.GPIO_Speed = GPIO_Speed_50MHz;
  		GPIO_Init(GPIOC, &gpio);

  		//init TIM3
	TIM_TimeBaseStructInit(&Timer);
	Timer.TIM_CounterMode = TIM_CounterMode_Up;
	Timer.TIM_Prescaler =(SystemCoreClock/1000000)-1;
	Timer.TIM_Period = 255;
	TIM_TimeBaseInit(TIM3, &Timer);
		//Init TIM3 Channels


	initPWM('g', GPIOA, GPIO_Pin_6, TIM3, 1);
	initPWM('b', GPIOB, GPIO_Pin_0, TIM3, 3);
	initPWM('r', GPIOB, GPIO_Pin_1, TIM3, 4);



//       NVIC Configuration
//       Enable the TIM4_IRQn Interrupt
      nvic.NVIC_IRQChannel = TIM4_IRQn;
      nvic.NVIC_IRQChannelPreemptionPriority = 0;
      nvic.NVIC_IRQChannelSubPriority = 0;
      nvic.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&nvic);

      while(1)
      {
      }
  }
