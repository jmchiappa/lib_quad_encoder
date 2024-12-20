#ifndef __inc_encoder
#define __inc_encoder

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace ENCODER {
	enum TIMERS {
		TIMER_16BITS,
		TIMER_32BITS
	};
};

class INC_ENCODER
{
	public:
		INC_ENCODER(void);      // constructor
		int32_t begin(uint32_t Pin_Channel1, uint32_t Pin_Channel2,float Dist_per_mm);
		int32_t GetTimerCounter(void);
		int64_t GetDistance(void);
		int32_t GetSpeed(void);
		void ResetCounter(void);
        void setCounterValue(int32_t value);
	private:
		// provide by specific driver
		int32_t start(void);
		int32_t stop(void);
		int32_t _init(uint32_t Pin_Channel1, uint32_t Pin_Channel2);
		int32_t convertTickForThisMCU(uint32_t tickCount);
		enum {FORWARD=0,BACKWARD} dir=BACKWARD;
		TIM_Encoder_InitTypeDef encoder;
		TIM_HandleTypeDef timer;
		float c;
		int32_t counter;
		int8_t msb;
		//uint32_t previous_counter;
		//int32_t previous_msb;
		int64_t previous_distance=0;;
		uint64_t previous_time;
        //uint32_t previous_counter;
		// int64_t distance;
		uint32_t Max_Cnt;
		TIM_TypeDef *_tim;
		// bool setValue=false;
		uint8_t type;
};

#ifdef __cplusplus
}
#endif


#endif //__inc_encoder
