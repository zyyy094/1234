#ifndef UI_DRAW_H
#define UI_DRAW_H

#include <opencv2/opencv.hpp>
#include "common.h"

void drawOverlay(cv::Mat& frame, State state, int fps, bool emergency,
                 int warning_countdown, double stable_seconds);

#endif