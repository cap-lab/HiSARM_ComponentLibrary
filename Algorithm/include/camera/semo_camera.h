#ifndef __SEMO_CAMERA_HEADER__
#define __SEMO_CAMERA_HEADER__

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define SEMO_CAMERA_WIDTH 480
#define SEMO_CAMERA_HEIGHT 480
#define SEMO_CAMERA_CHANNEL 3

typedef unsigned char SEMO_CAMERA[SEMO_CAMERA_WIDTH*SEMO_CAMERA_HEIGHT*SEMO_CAMERA_CHANNEL];

void convertCVMatToArray(unsigned char *dst, cv::Mat &src);
#endif