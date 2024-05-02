#include "EPuckSim_move.h"
#include "UFPort.h"
#include "UFMulticastPort.h"
#include "semo_logger.h"
#include <iostream>
#include <cmath>

#define PI 3.14159265358979
#define MAX_VEL 7.0
#define SCALE_VEL 3.0

enum turning {
    NO_TURN,
    SOFT_TURN,
    HARD_TURN
};

typedef struct _EPUCK_WHEEL {
    double left_vel;
    double right_vel;
} EPUCK_WHEEL;

using namespace std;

void move_init(int *turningMechanism)
{
    *turningMechanism = NO_TURN;
}


/////////////////////////////////////
// go code
/////////////////////////////////////

static double eulerOrientationToRadian(double *orientation)
{
    double radian;
    if (orientation[0] <= 0)
        radian = - orientation[1] + PI/2;
    else 
        radian = orientation[1] - PI/2;
    return radian;
}

static double signedNormalize(double radian){
    while(radian > PI) {
        radian -= 2*PI;
    }
    while(radian < -PI) {
        radian += 2*PI;
    }
    return radian;
}

static double vectorToRadian(double *vector)
{
    return atan2(vector[1], vector[0]);
}

static void vectorSum(double *vector1, double *vector2, double *vector)
{
    vector[0] = vector1[0] + vector2[0];
    vector[1] = vector1[1] + vector2[1];
}

static double lengthOfVector(double *vector)
{
    return sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
}

static void getVectorFromRadianAndLength(double radian, double length, double *vector)
{
    vector[0] = cos(radian) * length;
    vector[1] = sin(radian) * length;
}

static void diffusionVector(EPUCKSIM_MOVE_PORTS *ports, int *collision, double *vector)
{
    double proximityValue[8];
    double tempVector[2] = {0, 0};
    double proxRadians[8] = {PI/2, PI/4, PI/12, -PI/12, -PI/4, -PI/2, -3*PI/4, 3*PI/4};
    int dataLength;
    *collision = FALSE;
    UFMulticastPort_ReadFromBuffer(ports->proximity_group, ports->proximity_port, (unsigned char *)proximityValue, sizeof(double) * 8, &dataLength);
    for (int i=0 ; i <= 7 ; i++){
        double temp[2] = {0, 0};
        if(proximityValue[i] >  0.0) {
            *collision = TRUE;
            getVectorFromRadianAndLength(proxRadians[i], 0.05-proximityValue[i], temp);
        }
        vectorSum(tempVector, temp, tempVector);
    }
    
    if(*collision == TRUE) {
        vector[0] = -tempVector[0]/0.05;
        vector[1] = -tempVector[1]/0.05;
    } else {
        vector[0] = 0;
        vector[1] = 0;
    }
}

static double getDistance(double *point1, double *point2)
{
	return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

semo_int8 is_arrived(double arround, EPUCKSIM_MOVE_PORTS *ports, double *target)
{
    double position[3];
    int dataLength;
    int result;
    result = UFMulticastPort_ReadFromBuffer(ports->position_group, ports->position_port, (unsigned char *)position, sizeof(double) * 3, &dataLength);
    ERRIFGOTO(result, EXIT_);
	if(getDistance(target, position) <= arround) {
        return TRUE;
    }
EXIT_:
    if (result != ERR_UEM_NOERROR) {
        SEMO_LOG_ERROR("Communication error(%X)", result);
    }
    return FALSE;
}

static uem_result vectorToTarget(EPUCKSIM_MOVE_PORTS *ports, double *targetPoint, double *position, double *vector)
{
    double targetRadian;
    uem_result result;
    double currentOrientation[3];
    int dataLength;
    result = UFMulticastPort_ReadFromBuffer(ports->orientation_group, ports->orientation_port, (unsigned char *)currentOrientation, sizeof(double) * 3, &dataLength);
    ERRIFGOTO(result, EXIT_);
    vector[0] = targetPoint[0] - position[0];
    vector[1] = targetPoint[1] - position[1];
    targetRadian = vectorToRadian(vector);
    if (currentOrientation[1] > 2*PI || currentOrientation[1] < -2*PI) {
        getVectorFromRadianAndLength(signedNormalize(targetRadian), 1, vector);
    } else {
        getVectorFromRadianAndLength(signedNormalize(targetRadian-eulerOrientationToRadian(currentOrientation)), 1, vector);
    }
EXIT_:
    return result;
}
static uem_result setWheelSpeedFromVector(EPUCKSIM_MOVE_PORTS *ports, int *turningMechanism, double *vector)
{
    int dataNum;
    double radian;
    double baseAngularWheelSpeed;
    radian = signedNormalize(vectorToRadian(vector));
    baseAngularWheelSpeed = MAX_VEL;
    if(*turningMechanism == HARD_TURN) {
       if(abs(radian) <= PI/3) {
          *turningMechanism = SOFT_TURN;
       }
    }
	else if(*turningMechanism == SOFT_TURN) {
       if(fabs(radian) > PI/2) {
          *turningMechanism = HARD_TURN;
       }
       else if(fabs(radian) <= PI/8) {
          *turningMechanism = NO_TURN;
       }
    }
	else if(*turningMechanism == NO_TURN) {
       if(fabs(radian) > PI/2) {
          *turningMechanism = HARD_TURN;
       }
       else if(fabs(radian) > PI/8) {
          *turningMechanism = SOFT_TURN;
       }    
	}
    EPUCK_WHEEL vel;
    switch(*turningMechanism){
        case NO_TURN:
            vel.left_vel = baseAngularWheelSpeed;
            vel.right_vel = baseAngularWheelSpeed;
            break;
        case SOFT_TURN:
                vel.left_vel = baseAngularWheelSpeed/2 - baseAngularWheelSpeed * (fabs(radian) / PI);
                vel.right_vel = baseAngularWheelSpeed/2 + baseAngularWheelSpeed * (fabs(radian) / PI);
            break;
        case HARD_TURN:
                vel.left_vel = -baseAngularWheelSpeed/2;
                vel.right_vel = +baseAngularWheelSpeed/2;
            break;
    }
    if (radian < 0) {
        double temp;
        temp = vel.right_vel;
        vel.right_vel = vel.left_vel;
        vel.left_vel = temp;
    }
    return UFMulticastPort_WriteToBuffer(ports->wheel_group, ports->wheel_port, (unsigned char *)&vel, sizeof(EPUCK_WHEEL), &dataNum);
}

void move_to_target(EPUCKSIM_MOVE_PORTS *ports, int *turning_mechanism, double *targetPoint)
{
    int result;
    int dataLength;
    int collision;
    double position[3];
    double vector[2];
    double diffusedVector[2];
    result = UFMulticastPort_ReadFromBuffer(ports->position_group, ports->position_port, (unsigned char *)position, sizeof(double) * 3, &dataLength);
    ERRIFGOTO(result, EXIT_);
    result = vectorToTarget(ports, targetPoint, position, vector);
    ERRIFGOTO(result, EXIT_);
    diffusionVector(ports, &collision, diffusedVector);
	if (collision == TRUE) {
		diffusedVector[0] *= SCALE_VEL;
		diffusedVector[1] *= SCALE_VEL;
	} else {
		vector[0] *= SCALE_VEL;
		vector[1] *= SCALE_VEL;
    }
    vectorSum(vector, diffusedVector, vector);
    result = setWheelSpeedFromVector(ports, turning_mechanism, vector);

EXIT_:
    if (result != ERR_UEM_NOERROR) {
        SEMO_LOG_ERROR("Communication error(%X)", result);
    }
}

void move_wrapup(EPUCKSIM_MOVE_PORTS *ports)
{
    // TODO: task wrapup code
    EPUCK_WHEEL vel = {0, 0};
    int dataNum;
    UFMulticastPort_WriteToBuffer(ports->wheel_group, ports->wheel_port, (unsigned char *)&vel, sizeof(EPUCK_WHEEL), &dataNum);
}
