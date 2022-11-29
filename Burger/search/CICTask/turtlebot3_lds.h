#ifndef __TURTLEBOT_LDS_HEADER__
#define __TURTLEBOT_LDS_HEADER__

#include "pointdata.h"
#include "cmd_interface_linux.h"
#include "lipkg.h"
#include "slbf.h"
#include "transform.h"

bool isFrameReady();
FrameData getFrameData();
void init_lds();
int getFrontDistance(const FrameData &data, int l_angle, int r_angle);

#endif