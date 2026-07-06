#include "common.h"
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace cv;

string getCurrentTimestamp() {
    auto now = chrono::system_clock::now();
    auto now_time_t = chrono::system_clock::to_time_t(now);
    auto ms = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    struct tm* tm_info = localtime(&now_time_t);
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", tm_info);
    stringstream ss;
    ss << buffer << "_" << setfill('0') << setw(3) << ms.count();
    return ss.str();
}

float getIoU(const Rect& a, const Rect& b) {
    int x1 = max(a.x, b.x);
    int y1 = max(a.y, b.y);
    int x2 = min(a.x + a.width, b.x + b.width);
    int y2 = min(a.y + a.height, b.y + b.height);
    int inter = max(0, x2 - x1) * max(0, y2 - y1);
    int areaA = a.width * a.height;
    int areaB = b.width * b.height;
    if (areaA + areaB - inter == 0) return 0.f;
    return (float)inter / (areaA + areaB - inter);
}