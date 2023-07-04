

#include <MozziGuts.h>
// #include "oscillator.h"
// #include "adenv.h"

#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/saw2048_int8.h>
#include <Ead.h>
#include <mozzi_rand.h>
#include <ResonantFilter.h>

// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 256 // Hz, powers of 2 are most reliable

Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> osc1(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> osc2(SIN2048_DATA);
// Oscil <SQUARE_ANALOGUE512_NUM_CELLS, AUDIO_RATE> osc2(SQUARE_ANALOGUE512_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> lfo(SIN2048_DATA);
Ead env(CONTROL_RATE);
LowPassFilter lowPassFilt;

#define SERIAL_ENABLED true

#define JACKOUT1 16
#define JACKOUT2 17
#define JACKOUT3 18
#define JACKOUT4 8
#define JACKOUT5 4
#define JACKOUT6 12

#define LED3 15
#define LED4 13
#define LED5 22

#define JACKIN1 9
#define JACKIN2 1
#define JACKIN3 21
#define JACKIN4 19
#define JACKIN5 20
#define JACKIN6 3
#define JACKIN7 2
#define JACKIN8 5
#define JACKIN9 10
#define JACKIN10 6
#define JACKIN11 7
#define JACKIN12 11

#define NUM_OUT_JACKS 6
#define NUM_IN_JACKS 12

int jackWriteIter = 0;
const int outJackPinId[NUM_OUT_JACKS] = {JACKOUT1, JACKOUT2, JACKOUT3, JACKOUT4, JACKOUT5, JACKOUT6};
int jackReadIter = 0;
const int inJackPinId[NUM_IN_JACKS] = {JACKIN1, JACKIN2, JACKIN3, JACKIN4, JACKIN5, JACKIN6, JACKIN7, JACKIN8, JACKIN9, JACKIN10, JACKIN11, JACKIN12};
int inJackConnectionsTmp[NUM_IN_JACKS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
int inJackConnections[NUM_IN_JACKS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
int jackValRead = 0;
int getJackValueOut = 0;
int connectedOutJackId = 0;
int jack1Val = 0;
int jack2Val = 0;
int jack3Val = 0;
int jack4Val = 0;
int jack5Val = 0;
int jack6Val = 0;
int jack7Val = 0;
int jack8Val = 0;
int jack9Val = 0;
int jack10Val = 0;
int jack11Val = 0;
int jack12Val = 0;

unsigned long currentTimeMicr;
unsigned long timeLed3 = 0;
unsigned long timeLed4 = 0;
unsigned long timeLed5 = 0;

int jacksIter = 0; 

int analogReadIter = 0;
int envIter = 0;

int pot1Val = 0;
int pot2Val = 0;
int envVal = 0;
int envAttackVal = 0;
int envReleaseVal = 0;
int lfoVal = 0;
int osc1Enved = 0;
int osc1Val = 0;
int osc2Val = 0;
int phaseIter = 0;
int potReadIter = 0;
bool clockVal = false;
int prevClockVal = 9999;
bool clockSignalRising = false;
char shiftRegisterData = 0;
int shiftRegisterIter = 0;
int pulse1Val = 0;
int pulse2Val = 0;
int osc1FreqVal = 0;

void print(float value){
	#ifdef SERIAL_ENABLED
		Serial.println(value);
	#endif
}

void print(int value){
	#ifdef SERIAL_ENABLED
		Serial.println(value);
	#endif
}

void print(char* value){
	#ifdef SERIAL_ENABLED
		Serial.println(value);
	#endif
}

inline float getJackValue(int jackId, int defaultVal){
	getJackValueOut = defaultVal;

	connectedOutJackId = inJackConnections[jackId - 1];
	if(connectedOutJackId != -1){
		if(connectedOutJackId == 0){
			getJackValueOut = pot1Val;
		}
		else if(connectedOutJackId == 1){
			getJackValueOut = pot2Val;
		}
		else if(connectedOutJackId == 2){
			getJackValueOut = lfoVal;
		}
		else if(connectedOutJackId == 3){
			getJackValueOut = envVal;
		}
		else if(connectedOutJackId == 4){
			getJackValueOut = shiftRegisterData * 2;
		}
		else if(connectedOutJackId == 5){
			getJackValueOut = 6;
		}
	}

	return getJackValueOut;
}

inline void writeOutputJacks(){
	if(jackReadIter > NUM_OUT_JACKS){
		return;
	}

	for(int i = 0; i < NUM_OUT_JACKS; ++i){
		if(jackWriteIter <= i){
			digitalWrite(outJackPinId[i], HIGH);
		}
		else{
			digitalWrite(outJackPinId[i], LOW);
		}
	}

	jackWriteIter += 1;

	if(jackWriteIter > NUM_OUT_JACKS){
		jackWriteIter = 0;
	}
}

inline void readInputJacks(){
	for(int i = 0; i < NUM_IN_JACKS; ++i){
		jackValRead = digitalRead(inJackPinId[i]);
		
		if(jackValRead == HIGH){
			inJackConnectionsTmp[i] += 1;
		}
	}

	jackReadIter += 1;

	if(jackReadIter > NUM_IN_JACKS){
		jackReadIter = 0;

		for(int i = 0; i < NUM_IN_JACKS; ++i){
			jackValRead = inJackConnectionsTmp[i];

			if(jackValRead >= NUM_OUT_JACKS){
				jackValRead = -1;
			}
			
			inJackConnections[i] = jackValRead;
			
			inJackConnectionsTmp[i] = -1;
		}
	}
}

inline int mapInt(int x, int in_min, int in_max, int out_min, int out_max) {
  int v = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // if(v < out_min){
  // 	v = out_min;
  // }
  // else if(v > out_max){
  // 	v = out_max;
  // }

  return v;
}

inline void setLedFade(uint pin, int value, int maxValue, unsigned long &currentTimeMicr, unsigned long &outTime){
	// if(currentTimeMicr - outTime > (int)mapFloat(1.0 - value, 0.0, 1.0, 10, 1000)){
	if(currentTimeMicr - outTime > (maxValue - value) * 100){
		digitalWrite(pin, HIGH);

		outTime = currentTimeMicr;
	}
	else{
		digitalWrite(pin, LOW);
	}
}

void setup(){
	#ifdef SERIAL_ENABLED
		Serial.begin(9600);
	#endif

  // setup synth
	startMozzi(CONTROL_RATE);

	osc1.setFreq(0.1f);
	osc2.setFreq(0.1f);
	lfo.setFreq(10);

	env.set(32, CONTROL_RATE);

	// setup pins
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);

	pinMode(JACKOUT1, OUTPUT);
	pinMode(JACKOUT2, OUTPUT);
	pinMode(JACKOUT3, OUTPUT);
	pinMode(JACKOUT4, OUTPUT);
	pinMode(JACKOUT5, OUTPUT);
	pinMode(JACKOUT6, OUTPUT);

	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);
	pinMode(LED5, OUTPUT);

	pinMode(JACKIN1, INPUT_PULLUP);
	pinMode(JACKIN2, INPUT_PULLUP);
	pinMode(JACKIN3, INPUT_PULLUP);
	pinMode(JACKIN4, INPUT_PULLUP);
	pinMode(JACKIN5, INPUT_PULLUP);
	pinMode(JACKIN6, INPUT_PULLUP);
	pinMode(JACKIN7, INPUT_PULLUP);
	pinMode(JACKIN8, INPUT_PULLUP);
	pinMode(JACKIN9, INPUT_PULLUP);
	pinMode(JACKIN10, INPUT_PULLUP);
	pinMode(JACKIN11, INPUT_PULLUP);
	pinMode(JACKIN12, INPUT_PULLUP);
}

inline bool getBit(char& ch, unsigned int pos){
	return ch & 1 << pos;
}

inline void setBit(char& ch, unsigned int pos, unsigned int value)
{
    // ch |= value << pos;
    ch = (ch & ~(1<<pos)) | (value << pos);
}

unsigned int shiftRegVal = 0;

inline void updateShiftRegister(){
	shiftRegisterData = (jack4Val * 0.5);

	pulse1Val = 0;
	if(lfoVal > 128){
		pulse1Val = 1;
	}
	
	pulse2Val = 0;
	if(osc1Val > 0){
		pulse2Val = 1;
	}

	shiftRegVal = (int)(pulse1Val == pulse2Val);

	setBit(shiftRegisterData, shiftRegisterIter, shiftRegVal);

	shiftRegisterIter++;
	if(shiftRegisterIter > jack5Val % 8){
		shiftRegisterIter = 0;
	}

	print(shiftRegisterData);
}

void updateControl(){
	if(jacksIter == 10){
    	// pinMode(1, INPUT_PULLUP);
    
		jacksIter = 0;

		writeOutputJacks();
		readInputJacks();
	}
	jacksIter++;

	if(analogReadIter == 10){
		analogReadIter = 0;
		
		if(potReadIter == 0){
			pot1Val = analogRead(A2) / 4;// 0 - 1024 converted to 0 - 256

			potReadIter = 1;
		}
		else{
			pot2Val = analogRead(A1) / 4;// 0 - 1024 converted to 0 - 256

			potReadIter = 0;
		}
	}
	analogReadIter++;

	jack1Val = getJackValue(1, 0); // 0 - 256
	jack2Val = getJackValue(2, -1); // 0 - 256
	jack3Val = getJackValue(3, 1); // 0 - 256
	jack4Val = getJackValue(4, 0); // 0 - 256
	jack5Val = getJackValue(5, 7); // 0 - 256
	jack6Val = getJackValue(6, 0); // 0 - 256
	jack12Val = getJackValue(12, 1); // 0 - 256

	osc1FreqVal = pot1Val;
	if(jack1Val > 0){
		osc1FreqVal = jack1Val;
	}

	osc1.setFreq(osc1FreqVal);

	lfo.setFreq(pot1Val * jack12Val * 100);

	lfoVal = lfo.next() + 128;// -128 - 1024 converted to 0 - 256

	lowPassFilt.setCutoffFreqAndResonance(128, 200);

	clockVal = false;

	if(jack2Val > -1){
		if(jack2Val < prevClockVal){// if falling
			if(clockSignalRising){// if was rising
		  		clockVal = true;
		  	}

		  	clockSignalRising = false;
		}
		else if(jack2Val > prevClockVal){
			clockSignalRising = true;
		}

		prevClockVal = jack2Val;
	}
	else{
		if(envIter > pot2Val + 2){
		  envIter = 0;

		  clockVal = true;
		}
		envIter++;
	}

	if(clockVal){
		updateShiftRegister();

		env.start();
	}

	envAttackVal = 2;
	envReleaseVal = 50;

	env.set(envAttackVal, envReleaseVal);

	envVal = (int)env.next();

	currentTimeMicr = micros();
	setLedFade(LED3, lfoVal, 256, currentTimeMicr, timeLed3);
	setLedFade(LED4, envVal, 255, currentTimeMicr, timeLed4);
	setLedFade(LED5, shiftRegisterData, 255, currentTimeMicr, timeLed5);
}

AudioOutput_t updateAudio(){
	osc1Val = osc1.next();
	// osc2Val = osc2.next();

  // int pulse2Val = 0;
  // if(osc2Val > 0){
  //   pulse2Val = 1;
  // }

  // float mod1 = (float)(osc1Val + 128) / 256;
  // float mod2 = (float)(osc2Val + 128) / 256;

  // osc1.setFreq((float)((float)pot1Val));
  // osc2.setFreq((float)((float)pot1Val) * mod1);
  // osc2.setFreq(pot2Val * ((osc1Val + 128) / 256));
// print(pot2Val);
  // osc1.setFreq(pot1Val);
  // osc2.setFreq(pot2Val);

// print(mod1);
	// int sig = MonoOutput::fromNBit(10, (osc1Val * pulse2Val) + (osc2Val * pulse1Val));

	osc1Enved = osc1Val * envVal * 0.4;

	int lwpVal = lowPassFilt.next(osc1Enved);

	int sig = MonoOutput::fromNBit(16, lwpVal);

	return sig;
}

void loop(){
	audioHook();
}