#ifndef __TURTLEBOT_LDS_HEADER__
#define __TURTLEBOT_LDS_HEADER__

#include "pointdata.h"
#include "cmd_interface_linux.h"
#include "lipkg.h"
#include "slbf.h"
#include "transform.h"

typedef struct _TURTLEBOT_LDS {
    int front;
    int back;
    int left;
    int right;
} TURTLEBOT_LDS;

#endif