extern int photo_count;
extern bool bg_ready;
extern bool detection_started;

#include "ui_draw.h"
#include <ctime>

using namespace std;
using namespace cv;

void drawOverlay(Mat& frame, State state, int fps, bool emergency,
                 int countdown, double stable_sec) {
    int right_x = frame.cols - 180;

    char fps_str[32];
    snprintf(fps_str, sizeof(fps_str), "FPS: %d", fps);
    putText(frame, fps_str, Point(right_x, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);

    if (state != IDLE && !emergency) {
        char timer_str[64];
        snprintf(timer_str, sizeof(timer_str), "Timer: %.1f s", stable_sec);
        putText(frame, timer_str, Point(right_x, 60), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(200, 200, 200), 1);
    }

    if (state == WARNING && !emergency) {
        char cnt_str[32];
        snprintf(cnt_str, sizeof(cnt_str), "Alert: %d s", countdown);
        putText(frame, cnt_str, Point(right_x, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
    }

    time_t now = time(nullptr);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
    putText(frame, time_str, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

    const char* state_text = "";
    Scalar state_color;
    if (emergency) {
        state_text = "FIRE ALERT!";
        state_color = Scalar(0, 0, 255);
    } else {
        switch (state) {
            case IDLE:      state_text = "FREE"; state_color = Scalar(0, 255, 0); break;
            case WARNING:   state_text = "WARNING"; state_color = Scalar(0, 255, 255); break;
            case OCCUPIED:  state_text = "OCCUPIED"; state_color = Scalar(0, 0, 255); break;
        }
    }
    putText(frame, state_text, Point(10, 65), FONT_HERSHEY_SIMPLEX, 1.0, state_color, 2);

    int y = 100;
    if (emergency) {
        putText(frame, "FLAME DETECTED! BUZZER ON", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        y += 30;
        putText(frame, "PLEASE TAKE ACTION", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    } else if (state == WARNING) {
        putText(frame, "Object detected, stable?", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        y += 30;
        putText(frame, "BUZZER: OFF", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
    } else if (state == OCCUPIED) {
        putText(frame, "OCCUPIED! BUZZER INTERMITTENT", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        y += 30;
        putText(frame, "PLEASE CLEAR THE AREA", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    } else {
        if (!bg_ready) {
            putText(frame, "Press 's' to save background", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        } else if (!detection_started) {
            putText(frame, "Press 'c' to start detection", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        }
    }

    char photo_str[64];
    snprintf(photo_str, sizeof(photo_str), "Photos: %d (Press 'p' to save)", photo_count);
    putText(frame, photo_str, Point(10, frame.rows - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 2);
}