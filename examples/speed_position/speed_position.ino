#include "inc_encoder.h"

INC_ENCODER h;
#define INCREMENT	85 / 800 // 85 mm par tour de 800 pas

//uint32_t previous_time=0;
//uint64_t previous_distance;

void setup() {
	Serial.begin(9600);
	if(!h.begin(7,8,INCREMENT))
		Serial.println("Erreur dans l'initialisation");
}

void loop() {
//	uint32_t time = millis();
//	uint64_t d = h.GetDistance();
//	if(previous_time!=0) {

		Serial.print("\n\rincr=");
		Serial.print(h.GetTimerCounter());
		Serial.print("/tdist=");
		Serial.print(h.GetDistance());
		Serial.print("/tspeed=");
		Serial.print(h.GetSpeed());
//	}
//	previous_distance = d;
//	previous_time = time;
}