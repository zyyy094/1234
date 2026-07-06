#include "bg_subtract.h"

cv::Mat background;
bool bg_ready = false;
bool detection_started = false;

bool get_diff_roi(const cv::Mat& frame, cv::Rect& roi) {
    if (!bg_ready) return false;
    cv::Mat gray, diff, thresh;
    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::GaussianBlur(background, background, cv::Size(5, 5), 0);

    absdiff(background, gray, diff);
    threshold(diff, thresh, 40, 255, cv::THRESH_BINARY);

    cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    morphologyEx(thresh, thresh, cv::MORPH_CLOSE, kernel);
    morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Rect max_rect;
    double max_area = 0;

    for (auto& cnt : contours) {
        cv::Rect r = boundingRect(cnt);
        double area = r.area();
        float aspect_ratio = (float)r.width / r.height;
        if (area > max_area && area > MIN_OBJECT_AREA && aspect_ratio > 0.4 && aspect_ratio < 2.5) {
            max_area = area;
            max_rect = r;
        }
    }
    if (max_area > 0) {
        roi = max_rect;
        return true;
    }
    return false;
}