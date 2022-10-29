#include "LED.h"

//sig1 == out2 
//sig2 = out1
void LEDbegin(int devNum){
    pinMode(OLLO_SLEEP, OUTPUT);
    digitalWrite(OLLO_SLEEP, HIGH);

	switch(devNum){
	case 1:
		pinMode(PORT1_SIG1, OUTPUT); //RED  (right)
		pinMode(PORT1_SIG2, OUTPUT); //BLUE (left)
		pinMode(PORT1_ADC, INPUT_ANALOG); //ADC input
		break;
	case 2:
		pinMode(PORT2_SIG1, OUTPUT); //RED  (right)
		pinMode(PORT2_SIG2, OUTPUT); //BLUE (left)
		pinMode(PORT2_ADC, INPUT_ANALOG);//ADC input
		break;
	case 3:
		pinMode(PORT3_SIG1, OUTPUT); //RED  (right)
		pinMode(PORT3_SIG2, OUTPUT); //BLUE (left)
		pinMode(PORT3_ADC, INPUT_ANALOG);//ADC input
		break;
	case 4:
		pinMode(PORT4_SIG1, OUTPUT); //RED  (right)
		pinMode(PORT4_SIG2, OUTPUT); //BLUE (left)
		pinMode(PORT4_ADC, INPUT_ANALOG);//ADC input
		break;
	default:
		break;
	}
}

void turnOnRightLED(int devNum){ // LOW = TURN ON
	switch(devNum){
	case 1:
        digitalWrite(PORT1_SIG1,LOW); 
		break;
	case 2:
        digitalWrite(PORT2_SIG1,LOW); 
		break;
	case 3:
		digitalWrite(PORT3_SIG1,LOW); 
		break;
	case 4:
		digitalWrite(PORT4_SIG1,LOW); 
		break;
	default:
		break;
	}
}

void turnOffRightLED(int devNum){ // LOW = TURN OFF
	switch(devNum){
	case 1:
        digitalWrite(PORT1_SIG1,HIGH); 
		break;
	case 2:
        digitalWrite(PORT2_SIG1,HIGH); 
		break;
	case 3:
		digitalWrite(PORT3_SIG1,HIGH); 
		break;
	case 4:
		digitalWrite(PORT4_SIG1,HIGH); 
		break;
	default:
		break;
	}
}

void turnOnLeftLED(int devNum){ // LOW = TURN ON
	switch(devNum){
	case 1:
        digitalWrite(PORT1_SIG2,LOW); 
		break;
	case 2:
        digitalWrite(PORT2_SIG2,LOW); 
		break;
	case 3:
		digitalWrite(PORT3_SIG2,LOW); 
		break;
	case 4:
		digitalWrite(PORT4_SIG2,LOW); 
		break;
	default:
		break;
	}
}

void turnOffLeftLED(int devNum){ // LOW = TURN OFF
	switch(devNum){
	case 1:
        digitalWrite(PORT1_SIG2,HIGH); 
		break;
	case 2:
        digitalWrite(PORT2_SIG2,HIGH); 
		break;
	case 3:
		digitalWrite(PORT3_SIG2,HIGH); 
		break;
	case 4:
		digitalWrite(PORT4_SIG2,HIGH); 
		break;
	default:
		break;
	}
}