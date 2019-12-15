#include "Bistable.h"
#include "inc_encoder.h"

INC_ENCODER h;
#define INCREMENT	 85.0/800.0 // 85 mm par tour de 800 pas

//uint32_t previous_time=0;
//uint64_t previous_distance;

#define RISING_EDGE  true
#define FALLING_EDGE false

#define GEN_CH1 14
#define GEN_CH2 15
Bistable button;

void setPulse(int chA, int chB) {
    digitalWrite(chA,HIGH);
    digitalWrite(chB,LOW);
    delay(1);
    digitalWrite(chA,HIGH);
    digitalWrite(chB,HIGH);
    delay(1);
    digitalWrite(chA,LOW);
    digitalWrite(chB,HIGH);
    delay(1);
    digitalWrite(chA,LOW);
    digitalWrite(chB,LOW);
    delay(1);
}

void Pulse(int channel)
{
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  if(button.getState()){
    setPulse(GEN_CH1,GEN_CH2);
  }else{
    setPulse(GEN_CH2,GEN_CH1);
  }
}

void setup() {
  ;
	Serial.begin(115200);
  int32_t ret=h.begin(7,8,INCREMENT);
	if(ret!=0){
		Serial.print("Erreur dans l'initialisation: ");
    Serial.println(ret);
    while(1);
	}
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(USER_BTN,INPUT_PULLUP);

  pinMode(GEN_CH1,OUTPUT);
  pinMode(GEN_CH2,OUTPUT);
  button.begin(USER_BTN , FALLING_EDGE, INPUT_PULLUP);
  h.ResetCounter();
}

void loop() {
//	uint32_t time = millis();
//	uint64_t d = h.GetDistance();
//	if(previous_time!=0) {
  //   if(button.detect()) {
		  Pulse(2);
      if(!digitalRead(USER_BTN)) h.setCounterValue(50);
  		Serial.print("rincr=");
  		Serial.print(h.GetTimerCounter());
  		Serial.print("\tdist=");
  		Serial.print(h.GetDistance());
  		Serial.print("\tspeed=");
  		Serial.println(h.GetSpeed());
  //	}
  //	previous_distance = d;
  //	previous_time = time;
    // }
    // delay(50);
}
