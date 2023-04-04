#ifndef __TURTLEBOT3_CAMERA_HEADER__
#define __TURTLEBOT3_CAMERA_HEADER__

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define TURTLEBOT_CAMERA_WIDTH 480
#define TURTLEBOT_CAMERA_HEIGHT 480
#define TURTLEBOT_CAMERA_CHANNEL 3

typedef unsigned char TURTLEBOT_CAMERA[TURTLEBOT_CAMERA_WIDTH*TURTLEBOT_CAMERA_HEIGHT*TURTLEBOT_CAMERA_CHANNEL];

void convertCVMatToArray(unsigned char *dst, cv::Mat &src);
#endif