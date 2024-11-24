#include "inc_encoder.h"
#include "analog.h"
#include "timer.h"

// #define DBG

#if defined(DBG)
#	define DEBUGF(x,y,format)	Serial.print(x); Serial.println(y,format)
#	define DEBUG(x,y)	Serial.print(x); Serial.println(y)
#else
#	define DEBUG(x,y)
#	define DEBUGF(x,y,format)
#endif

#define MAXPOSIVE_32BITS	(0x7FFFFFFF)
#define MAXPOSIVE_16BITS	(0x7FFF)

static PinName g_ch1_pin = NC;
static PinName g_ch2_pin = NC;

const char *dbg_str[]={"Avance","Recule"};

#if defined(STM32L476xx)
# include "l476.init"
#elif defined(STM32WB55xx)
# include "wb55.init"
#elif defined(STM32L432xx)
# include "l432.init"
#else
# error "from library quad encoder : MCU not supported"
#endif

INC_ENCODER::INC_ENCODER(void) {};
int32_t INC_ENCODER::begin(uint32_t Pin_Channel1, uint32_t Pin_Channel2,float Dist_per_mm)
{
	c=Dist_per_mm;
	// distance=0;
	counter=0;
	msb=0;
	// setValue=false;
	return _init(Pin_Channel1,Pin_Channel2);
}

int32_t INC_ENCODER::GetTimerCounter(void) {
	bool roll=false;
	volatile int32_t currentTimer;
	// DEBUG("\nENTREE : direction=",dbg_str[dir]);
	// DEBUG("-counter=",counter);
	// DEBUG("-_tim->CNT=",_tim->CNT);
	// DEBUG("-msb=",msb);
	// if(!setValue) {
	if(type==ENCODER::TIMERS::TIMER_16BITS) {
		currentTimer = (int16_t)_tim->CNT;
	} else {
		currentTimer = _tim->CNT;
	}
	if( currentTimer != counter ) {
		volatile uint32_t UIF = _tim->SR & 0x01;
		volatile uint32_t DIR = _tim->CR1 & 0x10;
		DEBUGF("SR : 0x",_tim->SR,HEX);
		DEBUGF("CR1 : 0x",_tim->CR1,HEX);
		if(UIF) {
			// update Interrupt Event
			if(DIR == 0) {
				// count up
				msb++;
			} else {
				// count down
				msb--;
			}
			DEBUG("overflow : msb=",msb);
			LL_TIM_ClearFlag_UPDATE(_tim);
		}
		// if((int32_t)(counter - currentTimer)>(int32_t)(Max_Cnt>>1)) {	// roll over ?
		// 	msb++;		// increment number of step
		// 	roll=true;
		// }
		// if((int32_t)(_tim->CNT - counter)>(int32_t)(Max_Cnt>>1)) {	// roll over ?
		// 	DEBUG("(int32_t)(_tim->CNT - counter)=",(int32_t)(_tim->CNT - counter));
		// 	msb--;
		// 	roll=true;
		// }
		// if(roll==false) { // timer counter has not roll over
		// 	if(_tim->CNT>counter)
		// 		dir=FORWARD;
		// 	if(_tim->CNT<counter)
		// 		dir=BACKWARD;
		// }
		counter = currentTimer; 
	}
	// }else{
	// 	setValue=false;
	// 	_tim->CNT=counter;
	// }

	// DEBUG("\nSORTIE : direction=",dbg_str[dir]);
	// DEBUG("-counter=",counter);
	// DEBUG("-_tim->CNT=",_tim->CNT);
	// DEBUG("-msb*Max_Cnt=",msb*Max_Cnt);
	// DEBUG("","\n");
	return (int32_t)convertTickForThisMCU(counter);
	//return counter;
}

int64_t INC_ENCODER::GetDistance(void) {
	DEBUG("c=",c);
	int64_t distance = (int64_t)(msb*(int32_t)Max_Cnt) + (float)GetTimerCounter() * c;
	return distance;
}

int32_t INC_ENCODER::GetSpeed(void) {
	int32_t speed=0;
	int64_t distance = GetTimerCounter();
	//DEBUG("ENC::GetSpeed:distance=",distance);
	uint64_t time=millis();
	if(previous_distance!=0) {
		speed = 1000*((float)(distance - previous_distance) / (float)(time-previous_time));
	}
	previous_distance = distance;
	previous_time = time;
	return speed*c;
}

void INC_ENCODER::ResetCounter(void) {
	setCounterValue(0);
}

void INC_ENCODER::setCounterValue(int32_t value) {
	// this->setValue = true; // tim->CNT will be changed later in GetCounter (for safe-thread)
	stop();
	_tim->CNT=value<<1;
	// counter=value<<1; //encoder step is 2
	msb=0;
	previous_distance=0;
	previous_time =0;
	dir=BACKWARD;
	start();
	LL_TIM_ClearFlag_UPDATE(_tim);
}

