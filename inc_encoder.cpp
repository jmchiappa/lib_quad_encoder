#include "inc_encoder.h"
#include "analog.h"
#include "timer.h"

INC_ENCODER::INC_ENCODER(void) {};
bool INC_ENCODER::begin(uint32_t Pin_Channel1, uint32_t Pin_Channel2,float Dist_per_mm)
{
	c=Dist_per_mm;
	distance=0;
	counter=0;
	msb=0;
	TIM_TypeDef *pch1_tim;
	TIM_TypeDef *pch2_tim;

	PinName pch1 = digitalPinToPinName(Pin_Channel1);
	PinName pch2 = digitalPinToPinName(Pin_Channel2);

	// 1. Chack if both pins exist
	if((pch1!=NC)&&(pch2!=NC))
	{
	// 2. Check if timer handler is the same for both pins
		if(pin_in_pinmap(pch1, PinMap_INC_ENC))
			pch1_tim=(TIM_TypeDef *)pinmap_peripheral(pch1, PinMap_INC_ENC);
		if(pin_in_pinmap(pch1, PinMap_INC_ENC))
			pch2_tim=(TIM_TypeDef *)pinmap_peripheral(pch2, PinMap_INC_ENC);
		if(pch1_tim!=pch2_tim)
			return false;	// exit if handler is different

		if((pch1_tim==TIM2)||(pch1_tim==TIM5))
			Max_Cnt = 2^32;
		if((pch1_tim==TIM3)||(pch1_tim==TIM4))
			Max_Cnt = 2^16;
		_tim = pch1_tim; // store the timer instance
	// 3. initialize timer for inc encoder mode
		timer.Instance = pch1_tim;	// get the linked timer
		timer.Init.Period = 0xFFFF;
		timer.Init.CounterMode = TIM_COUNTERMODE_UP;
		timer.Init.Prescaler = 0;
		timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		
		encoder.EncoderMode = TIM_ENCODERMODE_TI12;

		encoder.IC1Filter = 0x0F;
		encoder.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
		encoder.IC1Prescaler = TIM_ICPSC_DIV4;
		encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;

		encoder.IC2Filter = 0x0F;
		encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		encoder.IC2Prescaler = TIM_ICPSC_DIV4;
		encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;

		if (HAL_TIM_Encoder_Init(&timer, &encoder) != HAL_OK) {
			Error_Handler();
		}

		if(HAL_TIM_Encoder_Start_IT(&timer,TIM_CHANNEL_1)!=HAL_OK){
			Error_Handler();
		return true;
		}
	}
	return false;
}

uint64_t INC_ENCODER::GetTimerCounter(void) {
	if(counter> _tim->CNT)	// roll over ?
		msb++;		// increment number of step
	counter = _tim->CNT; 
	return msb*Max_Cnt + counter;
}

uint64_t INC_ENCODER::GetDistance(void) {
	distance = GetTimerCounter() * c;
	return distance;
}

uint32_t INC_ENCODER::GetSpeed(void) {
	uint32_t speed=0;
	distance = GetTimerCounter() * c;
	uint32_t time=millis();
	if(previous_distance!=0) {
		speed = (distance - previous_distance) / (time-previous_time);
	}
	previous_distance = distance;
	previous_distance = time;
	return speed;
}

void INC_ENCODER::ResetCounter(void) {
	counter=0;
	msb=0;
	previous_distance=0;
	previous_time =0;
	_tim->CNT=0;
}

/*
int main(void) {
 
 HAL_Init();
 SystemClock_Config();
 HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
 HAL_NVIC_SetPriority(SysTick_IRQn, 0, 1);
 
 timer.Instance = TIM3;
 timer.Init.Period = 0xFFFF;
 timer.Init.CounterMode = TIM_COUNTERMODE_UP;
 timer.Init.Prescaler = 0;
 timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 
 
	while (1) {
	uint16_t count=TIM3->CNT;
	}
}
*/
static PinName g_ch1_pin = NC;
static PinName g_ch2_pin = NC;

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_TypeDef *port;

	uint32_t function_ch1 = pinmap_function(g_ch1_pin, PinMap_INC_ENC);
	uint32_t function_ch2 = pinmap_function(g_ch2_pin, PinMap_INC_ENC);
	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* TIMx Peripheral clock enable */
	timer_enable_clock(htim);

	/* Enable GPIO Channels Clock */
	/* Enable GPIO clock ****************************************/
	port = set_GPIO_Port_Clock(STM_PORT(g_ch1_pin));

	/* Common configuration for all channels */
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin 	 	= STM_GPIO_PIN(g_ch1_pin);
	GPIO_InitStruct.Alternate 	= STM_PIN_AFNUM(function_ch1);
	HAL_GPIO_Init(port, &GPIO_InitStruct);

	port = set_GPIO_Port_Clock(STM_PORT(g_ch2_pin));
	GPIO_InitStruct.Pin 	 	= STM_GPIO_PIN(g_ch2_pin);
	GPIO_InitStruct.Alternate 	= STM_PIN_AFNUM(function_ch2);
	HAL_GPIO_Init(port, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);

	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/*
extern TIM_HandleTypeDef timer;
 
void TIM3_IRQHandler(void){
 HAL_TIM_IRQHandler(&timer);
}
*/