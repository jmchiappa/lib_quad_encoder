#include "inc_encoder.h"
#include "analog.h"
#include "timer.h"

//#define DEBUG(x,y)	Serial.print(x); Serial.print(y)
#define DEBUG(x,y)
static PinName g_ch1_pin = NC;
static PinName g_ch2_pin = NC;

const char *dbg_str[]={"Avance","Recule"};

INC_ENCODER::INC_ENCODER(void) {};
int32_t INC_ENCODER::begin(uint32_t Pin_Channel1, uint32_t Pin_Channel2,float Dist_per_mm)
{
	c=Dist_per_mm;
	distance=0;
	counter=0;
	msb=0;
	TIM_TypeDef *pch1_tim;
	TIM_TypeDef *pch2_tim;

	PinName pch1 = digitalPinToPinName(Pin_Channel1);
	PinName pch2 = digitalPinToPinName(Pin_Channel2);
	DEBUG("pch1=",pch1);
	DEBUG("pch2=",pch2);
	ResetCounter();
	// 1. Chack if both pins exist
	if((pch1!=NC)&&(pch2!=NC))
	{
		g_ch1_pin = pch1;
		g_ch2_pin = pch2;
	// 2. Check if timer handler is the same for both pins
		if(pin_in_pinmap(pch1, PinMap_INC_ENC))
			pch1_tim=(TIM_TypeDef *)pinmap_peripheral(pch1, PinMap_INC_ENC);
		if(pin_in_pinmap(pch1, PinMap_INC_ENC))
			pch2_tim=(TIM_TypeDef *)pinmap_peripheral(pch2, PinMap_INC_ENC);
		if(pch1_tim!=pch2_tim)
			return 1;	// exit if handler is different

		if((pch1_tim==TIM2)||(pch1_tim==TIM5))
			Max_Cnt = 1<<32;
		if((pch1_tim==TIM1)||(pch1_tim==TIM3)||(pch1_tim==TIM4)||(pch1_tim==TIM8))
			Max_Cnt = 1<<16;
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
			return 2;
		}

		if(HAL_TIM_Encoder_Start_IT(&timer,TIM_CHANNEL_1)!=HAL_OK){
			Error_Handler();
			return 2;
		}
		return 0;
	}
	return 3;
}

int64_t INC_ENCODER::GetTimerCounter(void) {
	bool roll=false;
	DEBUG("\nENTREE : direction=",dbg_str[dir]);
	DEBUG("-counter=",counter);
	DEBUG("-_tim->CNT=",_tim->CNT);
	DEBUG("-msb=",msb);
	if(_tim->CNT!=counter) {
		if((int32_t)(counter - _tim->CNT)>(int32_t)(Max_Cnt>>1)) {	// roll over ?
			msb++;		// increment number of step
			roll=true;
		}
		if((int32_t)(_tim->CNT - counter)>(int32_t)(Max_Cnt>>1)) {	// roll over ?
			DEBUG("(int32_t)(_tim->CNT - counter)=",(int32_t)(_tim->CNT - counter));
			msb--;
			roll=true;
		}
		if(roll==false) { // timer counter has not roll over
			if(_tim->CNT>counter)
				dir=FORWARD;
			if(_tim->CNT<counter)
				dir=BACKWARD;
		}
		counter = _tim->CNT; 
	}
	DEBUG("\nSORTIE : direction=",dbg_str[dir]);
	DEBUG("-counter=",counter);
	DEBUG("-_tim->CNT=",_tim->CNT);
	DEBUG("-msb*Max_Cnt=",msb*Max_Cnt);
	DEBUG("","\n");
	return msb*(int64_t)Max_Cnt + counter;
	//return counter;
}

int64_t INC_ENCODER::GetDistance(void) {
	distance = GetTimerCounter() * c;
	return distance;
}

int32_t INC_ENCODER::GetSpeed(void) {
	uint32_t speed=0;
	distance = GetTimerCounter() * c;
	//DEBUG("ENC::GetSpeed:distance=",distance);
	uint64_t time=millis();
	if(previous_distance!=0) {
		speed = (distance - previous_distance) / (time-previous_time);
	}
	previous_distance = distance;
	previous_time = time;
	return speed;
}

void INC_ENCODER::ResetCounter(void) {
	_tim->CNT=0;
	counter=0;
	msb=0;
	previous_distance=0;
	previous_time =0;
	dir=BACKWARD;
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

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_TypeDef *port;
	uint32_t function_ch1 = pinmap_function(g_ch1_pin, PinMap_INC_ENC);
	uint32_t function_ch2 = pinmap_function(g_ch2_pin, PinMap_INC_ENC);
	// DEBUG("g_ch1_pin=",g_ch1_pin);
	// DEBUG("g_ch2_pin=",g_ch1_pin);
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
	
	HAL_NVIC_SetPriority(TIM1_IRQn, 0, 1);

	HAL_NVIC_EnableIRQ(TIM1_IRQn);
}

/*
extern TIM_HandleTypeDef timer;
 
void TIM3_IRQHandler(void){
 HAL_TIM_IRQHandler(&timer);
}
*/