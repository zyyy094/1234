#ifndef UI_DRAW_H
#define UI_DRAW_H

#include <opencv2/opencv.hpp>
#include "common.h"

void draw_overlay(cv::Mat& frame, State s, int fps, bool emergency,
                   int warning_seconds, double stable_seconds, int photo_count);

#endif