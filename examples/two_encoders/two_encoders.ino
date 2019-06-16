#include "transient.h"
#include "inc_encoder.h"

INC_ENCODER wheel_1,wheel_2;
#define INCREMENT	 85/1024.0

// incremental encoder on TIM4
#define Q1A 22
#define Q1B 10

// incremental encoder on TIM8
#define Q2A 34
#define Q2B 9

#define GEN_DELAY 1
// blue use button on D23
#define BUTTON 23

// direction select on D2
#define GEN_DIR 2

// quadrature pulse generator on D14/D15
#define GEN_CH1 14
#define GEN_CH2 15

#define RISING_EDGE  true
#define FALLING_EDGE false

// edge detector
Transient button;
uint8_t led=0;
//-----------------------------------------
// Functions declaration

// quadrature pulse generator
void setPulse(int chA, int chB) {
    digitalWrite(chA,HIGH);
    digitalWrite(chB,LOW);
    delay(GEN_DELAY);
    digitalWrite(chA,HIGH);
    digitalWrite(chB,HIGH);
    delay(GEN_DELAY);
    digitalWrite(chA,LOW);
    digitalWrite(chB,HIGH);
    delay(GEN_DELAY);
    digitalWrite(chA,LOW);
    digitalWrite(chB,LOW);
    delay(GEN_DELAY);
}

// Pulse generator
// channel : direction of pulsing
void Pulse(int channel)
{
  analogWrite(LED_BUILTIN,led++);
  if(digitalRead(GEN_DIR)){
    setPulse(GEN_CH1,GEN_CH2);
  }else{
    setPulse(GEN_CH2,GEN_CH1);
  }
}

// Initialization
void setup() {
	Serial.begin(115200);
  // incremental encoder init
  // INCREMENT defines the moving quantity in mm per tick (float number)
  int32_t ret1=wheel_1.begin(Q1A,Q1B,INCREMENT);
  int32_t ret2=wheel_2.begin(Q2A,Q2B,INCREMENT);

	if(ret1!=0){
		Serial.print("Erreur dans l'initialisation wheel_1: ");
    Serial.println(ret1);
    while(1);
	}
  if(ret2!=0){
    Serial.print("Erreur dans l'initialisation wheel_2: ");
    Serial.println(ret2);
    while(1);
  }
  pinMode(LED_BUILTIN,OUTPUT);

  pinMode(GEN_DIR,INPUT_PULLUP);
  pinMode(GEN_CH1,OUTPUT);
  pinMode(GEN_CH2,OUTPUT);

  // declare the edge detector on blue button, falling edge
  button.begin(BUTTON , FALLING_EDGE, INPUT);
  wheel_1.ResetCounter();
  wheel_2.ResetCounter();
}

void loop() {
//	uint32_t time = millis();
//	uint64_t d = h.GetDistance();
//	if(previous_time!=0) {
  //   if(button.detect()) {
		  Pulse(GEN_DIR);
  		Serial.print("t1=");
  		Serial.print(wheel_1.GetTimerCounter());
  		Serial.print("\td1=");
  		Serial.print(wheel_1.GetDistance());
  		Serial.print("\ts1=");
  		Serial.print(wheel_1.GetSpeed());
      Serial.print("\tt2=");
      Serial.print(wheel_2.GetTimerCounter());
      Serial.print("\td2=");
      Serial.print(wheel_2.GetDistance());
      Serial.print("\ts2=");
      Serial.println(wheel_2.GetSpeed());
  //	}
  //	previous_distance = d;
  //	previous_time = time;
    // }
    // delay(50);
}
