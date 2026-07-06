#ifndef BG_SUBTRACT_H
#define BG_SUBTRACT_H

#include <opencv2/opencv.hpp>

extern cv::Mat background;
extern bool bg_ready;
extern bool detection_started;
extern int photo_count;

bool getDiffROI(const cv::Mat& frame, cv::Rect& roi);
void saveBackground(const cv::Mat& frame);
void resetBackground();

#endif