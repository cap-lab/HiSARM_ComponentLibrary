#ifndef __EPUCKSIM_MOVE_HEADER__
#define __EPUCKSIM_MOVE_HEADER__

#include "semo_common.h"

typedef struct _MOVE_PORTS {
    int robot_id;
    int position_group;
    int position_port;
    int orientation_group;
    int orientation_port;
    int proximity_group;
    int proximity_port;
    int wheel_group;
    int wheel_port;
} MOVE_PORTS;

void move_init(int *turning_mechanism);
void move_to_target(MOVE_PORTS *ports, int *turning_mechanism, double *target);
semo_int8 is_arrived(double arround, MOVE_PORTS *ports, double *target);
void move_wrapup(MOVE_PORTS *ports);

#endif
