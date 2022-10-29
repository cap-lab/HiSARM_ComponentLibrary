#ifndef LED_H_
#define LED_H_
#include <Arduino.h>

#define PORT1_SIG1  40
#define PORT1_SIG2  41
#define PORT1_ADC   42

#define PORT2_SIG1  43
#define PORT2_SIG2  44
#define PORT2_ADC   45

#define PORT3_SIG1  70
#define PORT3_SIG2  71
#define PORT3_ADC   72

#define PORT4_SIG1  73
#define PORT4_SIG2  74
#define PORT4_ADC   75

#define OLLO_SLEEP  46

void LEDbegin(int devNum);
void turnOnRightLED(int devNum);
void turnOffRightLED(int devNum);
void turnOnLeftLED(int devNum);
void turnOffLeftLED(int devNum);