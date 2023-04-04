#ifndef TURTLEBOT3_MOTOR_CONTROL_H_
#define TURTLEBOT3_MOTOR_CONTROL_H_

#include "turtlebot3_burger.h"

extern float goal_velocity[WHEEL_NUM];
extern int motorSet;

void initTurtleBotMotor();

#endif