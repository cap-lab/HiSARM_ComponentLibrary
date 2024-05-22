
#include <inttypes.h>
#include <DynamixelSDK.h>

#include "turtlebot3_burger.h"
#include "turtlebot3_motor_driver.h"

#define CONTROL_PERIOD                   8000

static HardwareTimer Timer(TIMER_CH1);
static Turtlebot3MotorDriver motor_driver;
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
int motorSet = 0;
static int motorIsInit = 0;

static void controlMoter();
static void startDynamixelControlInterrupt();

void initTurtleBotMotor() {
    if(motorIsInit == 0) {
        motor_driver.init(NAME);
        startDynamixelControlInterrupt();
        motorIsInit = 1;
    }

}

static void controlMoter(){
    if (motorSet == 1){        // in microseconds
        motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
        motorSet = 0;
    }
}


static void startDynamixelControlInterrupt(){
    Timer.pause();
    Timer.setPeriod(CONTROL_PERIOD);   
    
    Timer.attachInterrupt(controlMoter);
    
    Timer.refresh();
    Timer.resume();
}



