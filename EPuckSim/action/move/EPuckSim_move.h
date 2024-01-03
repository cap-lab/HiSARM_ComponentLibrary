#ifndef __EPUCKSIM_MOVE_HEADER__
#define __EPUCKSIM_MOVE_HEADER__

#include "semo_common.h"

typedef struct _EPUCKSIM_MOVE_PORTS {
    int position_group;
    int position_port;
    int orientation_group;
    int orientation_port;
    int proximity_group;
    int proximity_port;
    int wheel_group;
    int wheel_port;
} EPUCKSIM_MOVE_PORTS;

void move_init(int *turning_mechanism);
void move_to_target(EPUCKSIM_MOVE_PORTS *ports, int *turning_mechanism, double *target);
semo_int8 is_arrived(EPUCKSIM_MOVE_PORTS *ports, double *target);
void move_wrapup(EPUCKSIM_MOVE_PORTS *ports);

#endif