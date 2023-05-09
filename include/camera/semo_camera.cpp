#include "semo_camera.h"

void convertCVMatToArray(unsigned char *dst, cv::Mat &src) {
    for (int i = 0; i < src.rows; ++i) {
        memcpy(dst + i * src.cols * src.channels(), src.ptr<uchar>(i), src.cols * src.channels());
    }
}