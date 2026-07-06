#ifndef BG_SUBTRACT_H
#define BG_SUBTRACT_H

#include <opencv2/opencv.hpp>
#include "common.h"

extern cv::Mat background;
extern bool bg_ready;
extern bool detection_started;

bool get_diff_roi(const cv::Mat& frame, cv::Rect& roi);

#endif