#include "bg_subtract.h"
#include "common.h"
#include <iostream>

using namespace std;
using namespace cv;

Mat background;
bool bg_ready = false;
bool detection_started = false;
int photo_count = 0;

bool getDiffROI(const Mat& frame, Rect& roi) {
    if (!bg_ready) return false;
    Mat gray, diff, thresh;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5, 5), 0);

    Mat bg_blur;
    GaussianBlur(background, bg_blur, Size(5, 5), 0);

    absdiff(bg_blur, gray, diff);
    threshold(diff, thresh, 40, 255, THRESH_BINARY);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(thresh, thresh, MORPH_CLOSE, kernel);
    morphologyEx(thresh, thresh, MORPH_OPEN, kernel);

    vector<vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Rect max_rect;
    double max_area = 0;

    for (auto& cnt : contours) {
        Rect r = boundingRect(cnt);
        double area = r.area();
        float aspect = (float)r.width / r.height;
        if (area > max_area && area > MIN_OBJECT_AREA && aspect > 0.4 && aspect < 2.5) {
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

void saveBackground(const Mat& frame) {
    background = frame.clone();
    cvtColor(background, background, COLOR_BGR2GRAY);
    bg_ready = true;
    cout << "[INFO] 背景已保存" << endl;
}

void resetBackground() {
    bg_ready = false;
    detection_started = false;
}